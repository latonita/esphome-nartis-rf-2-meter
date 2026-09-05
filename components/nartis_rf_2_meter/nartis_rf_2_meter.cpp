#include "nartis_rf_2_meter.h"

#include "esphome/core/hal.h"
#include "esphome/core/helpers.h"

#include <cinttypes>
#include <cstdio>
#include <cstring>

namespace esphome::nartis_rf_2_meter {

static const char *const TAG = "nartis_rf_2_meter";

// TEMPORARY: DI 0xF101 is not put on air, so TAGs 0x0A..0x13 go unfilled. Set to
// false to restore it - the request and the decode are both still there.
static constexpr bool SKIP_F101 = true;

void NartisRf2MeterComponent::setup() {
  if (this->pin_sdio_ == nullptr || this->pin_sclk_ == nullptr || this->pin_csb_ == nullptr ||
      this->pin_fcsb_ == nullptr || this->pin_gpio3_ == nullptr) {
    ESP_LOGE(TAG, "CMT2300A pins are not configured");
    this->publish_cycle_outcome_(false);
    this->mark_failed();
    return;
  }

  serial_to_bcd_le(this->address_.c_str(), this->serial_le_);
  this->rf_frequency_hz_ =
      (this->frequency_override_ != 0) ? this->frequency_override_ : frequency_from_serial(this->address_.c_str());

  this->hal_.set_pins(this->pin_sdio_, this->pin_sclk_, this->pin_csb_, this->pin_fcsb_, this->pin_gpio3_);
  // Must precede init(): init() is what writes the computed frequency bank.
  this->hal_.set_frequency(this->rf_frequency_hz_);

  if (!this->hal_.init()) {
    ESP_LOGE(TAG, "CMT2300A initialization failed - check wiring");
    // Fail safe: a radio that never came up must not leave the entity unknown.
    this->publish_cycle_outcome_(false);
    this->mark_failed();
    return;
  }
  this->hal_.go_standby();
  this->radio_ready_ = true;

  // The diagnostic entity reports on a poll cycle, so with no value entities
  // configured it needs one asked for - the status exchange, being the shorter.
  if (this->last_read_ok_bs_ != nullptr && !this->need_data_ && !this->need_status_) {
    this->need_status_ = true;
    ESP_LOGI(TAG, "Only the last_read_ok entity is configured - polling the status page to exercise the link");
  }

  if (!this->need_data_ && !this->need_status_) {
    ESP_LOGW(TAG, "No sensors configured - nothing will be polled");
  }

  ESP_LOGI(TAG, "CMT2300A ready at %.3f MHz (meter %s)", this->rf_frequency_hz_ / 1e6f, this->address_.c_str());
  this->set_state_(State::IDLE);
}

void NartisRf2MeterComponent::handle_list_reply_(uint8_t request_idx, const ParsedResponse &resp) {
  const ListRequest &req = LIST_REQUESTS[request_idx];
  const uint8_t list = static_cast<uint8_t>(req.list);
  const bool got_status_half = (resp.shape == PayloadShape::STATUS_HALF);

  if ((req.part == ListPart::STATUS) != got_status_half) {
    // Request and reply framing disagree. The reply is taken at what it decodes
    // as, the exact-fit rule having proved that reading consumes DATA.
    this->warn_unexpected_half_once_(request_idx, resp);
  }

  // The leftovers a status half brings are the same list continued.
  this->merge_records_(resp);
  this->list_arrived_[list] = static_cast<uint8_t>(this->list_arrived_[list] + resp.count);

  if (!got_status_half) {
    // COUNT is the whole list; the status half is what brings the rest.
    this->list_announced_[list] = resp.announced_count;
    return;
  }

  if (!resp.has_status_block) {
    return;  // unreachable: the shape is only chosen when the block is there
  }
  if (!this->status_ok_) {
    std::memcpy(this->status_block_, resp.status_block, STATUS_BLOCK_SIZE);
    this->status_ok_ = true;
    return;
  }
  // Both blocks are built from the same device state, so they should agree.
  if (std::memcmp(this->status_block_, resp.status_block, STATUS_BLOCK_SIZE) != 0) {
    ESP_LOGW(TAG, "The two lists' status blocks differ: kept %s",
             format_hex_pretty(this->status_block_, STATUS_BLOCK_SIZE).c_str());
    ESP_LOGW(TAG, "  DI 0x%04X sent %s", req.di,
             format_hex_pretty(resp.status_block, STATUS_BLOCK_SIZE).c_str());
  }
}

void NartisRf2MeterComponent::warn_unexpected_half_once_(uint8_t request_idx, const ParsedResponse &resp) {
  const uint8_t bit = static_cast<uint8_t>(1u << request_idx);
  if ((this->warned_half_ & bit) != 0) {
    return;
  }
  this->warned_half_ |= bit;
  const ListRequest &req = LIST_REQUESTS[request_idx];
  ESP_LOGW(TAG, "DI 0x%04X is list %s's %s half, but its reply is framed as the other one: %s", req.di,
           list_id_to_string(req.list), (req.part == ListPart::STATUS) ? "status" : "records",
           payload_shape_to_string(resp.shape));
  ESP_LOGW(TAG, "  Read as what it decodes to, since that reading fits DATA exactly. Please report the");
  ESP_LOGW(TAG, "  RX line above - it means this meter frames its lists differently.");
}

/* Neither fixed block carries a TAG - a value is identified by where it sits - so
 * each is stored whole here and mapped onto TAGs later, by the maps in
 * d101_frame.cpp. The reply length also tells a three-phase meter from a
 * single-phase one.
 */
void NartisRf2MeterComponent::handle_fixed_reply_(uint8_t fixed_idx, const ParsedResponse &resp) {
  switch (resp.shape) {
    case PayloadShape::FIXED_F101:
      std::memcpy(this->f101_raw_, resp.payload, sizeof(this->f101_raw_));
      this->f101_ok_ = true;
      this->log_f101_();
      return;

    case PayloadShape::FIXED_F102_3PH:
    case PayloadShape::FIXED_F102_1PH:
      // Length is the variant, so it is what gets stored.
      std::memcpy(this->f102_raw_, resp.payload, resp.payload_len);
      this->f102_len_ = resp.payload_len;
      this->log_f102_();
      return;

    default:
      // Unreachable: parse_response() only gives these two DIs the shapes above.
      ESP_LOGW(TAG, "DI 0x%04X answered with shape '%s', which is not a fixed block", FIXED_REQUESTS[fixed_idx].di,
               payload_shape_to_string(resp.shape));
      return;
  }
}

void NartisRf2MeterComponent::log_f101_() const {
  nartis_f101 b{};
  std::memcpy(&b, this->f101_raw_, sizeof(b));

  /* Two groups of nine: [total, T1..T8], reactive energy import and export. All
   * nine of each are printed even though only total and T1..T4 have a TAG, because
   * this line is the only place T5..T8 appear at all.
   */
  struct Group {
    const char *name;
    const uint32_t *v;
  };
  const Group groups[2] = {{"R+", b.group20}, {"R-", b.group30}};

  ESP_LOGD(TAG, "F101 reactive energy (%u B of DATA)", static_cast<unsigned>(sizeof(b)));
  for (const Group &g : groups) {
    // Sized for the worst case: nine values at up to "T8 4294967.295, " each.
    char line[192];
    int at = std::snprintf(line, sizeof(line), "total %.3f", g.v[0] * 0.001);
    for (uint8_t i = 1; i < 9 && at > 0 && at < static_cast<int>(sizeof(line)); i++) {
      const int n = std::snprintf(line + at, sizeof(line) - static_cast<size_t>(at), ", T%u %.3f", i, g.v[i] * 0.001);
      if (n <= 0) {
        break;
      }
      at += n;
    }
    ESP_LOGD(TAG, "  %-5s %s kvarh", g.name, line);

    // Exact, unlike F102's checks: both numbers come out of the same reply. Only
    // prints when it fails.
    uint32_t sum = 0;
    for (uint8_t i = 1; i < 9; i++) {
      sum += g.v[i];
    }
    if (sum != g.v[0]) {
      ESP_LOGD(TAG, "  %-5s total != T1..T8 sum (%.3f kvarh) - decode is suspect", g.name, sum * 0.001);
    }
  }

  ESP_LOGD(TAG, "  stat  %s",
           format_hex_pretty(reinterpret_cast<const uint8_t *>(&b.status), sizeof(b.status)).c_str());
}

void NartisRf2MeterComponent::log_f102_() const {
  if (this->f102_len_ == sizeof(f102_1ph)) {
    f102_1ph b{};
    std::memcpy(&b, this->f102_raw_, sizeof(b));
    ESP_LOGI(TAG, "F102 is the SINGLE-PHASE layout (%u B of DATA, lead 0x%02X)", this->f102_len_, b.lead);
    // Deliberately raw: the firmware pre-scales this variant unevenly, so any
    // multiplier printed here would be a guess.
    ESP_LOGD(TAG, "  P=%" PRId32 " Q=%" PRId32 " U=%" PRIu32 " I=%" PRId32 " f=%" PRIu32, bcd32_signed(&b.p),
             bcd32_signed(&b.q), bcd32_value(&b.u), bcd32_signed(&b.i), bcd32_value(&b.freq));
    return;
  }

  if (this->f102_len_ != sizeof(f102_3ph)) {
    // parse_response() rejects any other length, so this is unreachable from air.
    ESP_LOGW(TAG, "F102 length %u matches neither layout", this->f102_len_);
    return;
  }

  f102_3ph b{};
  std::memcpy(&b, this->f102_raw_, sizeof(b));
  ESP_LOGI(TAG, "F102 is the THREE-PHASE layout (%u B of DATA, marker 0x%02X)", this->f102_len_, b.marker);
  ESP_LOGD(TAG, "  P     total %.1f, L1 %.1f, L2 %.1f, L3 %.1f W", bcd32_signed(&b.p_total) * 0.1,
           bcd32_signed(&b.p_l1) * 0.1, bcd32_signed(&b.p_l2) * 0.1, bcd32_signed(&b.p_l3) * 0.1);
  ESP_LOGD(TAG, "  Q     total %.1f, L1 %.1f, L2 %.1f, L3 %.1f var", bcd32_signed(&b.q_total) * 0.1,
           bcd32_signed(&b.q_l1) * 0.1, bcd32_signed(&b.q_l2) * 0.1, bcd32_signed(&b.q_l3) * 0.1);
  ESP_LOGD(TAG, "  U     L1 %.2f, L2 %.2f, L3 %.2f V", bcd32_value(&b.u_l1) * 0.01, bcd32_value(&b.u_l2) * 0.01,
           bcd32_value(&b.u_l3) * 0.01);
  ESP_LOGD(TAG, "  I     L1 %.2f, L2 %.2f, L3 %.2f A", bcd32_value(&b.i_l1) * 0.01, bcd32_value(&b.i_l2) * 0.01,
           bcd32_value(&b.i_l3) * 0.01);
  ESP_LOGD(TAG, "  f     %.2f Hz", bcd32_value(&b.freq) * 0.01);

  // The per-phase values summing to the totals pinned this field order, but it is
  // not a per-reply test: the phases and the total are sampled a moment apart, so
  // any threshold tight enough to catch a swapped field fires on ordinary replies.
  // f1xx_check tests the order against the reference capture instead.
}

void NartisRf2MeterComponent::merge_records_(const ParsedResponse &resp) {
  for (uint8_t i = 0; i < resp.count && i < MAX_ITEMS; i++) {
    const ParsedItem &item = resp.items[i];

    // The lists overlap, so a TAG can arrive twice in a cycle. The later copy wins:
    // the pages are read seconds apart, so a difference is the meter having moved on.
    bool replaced = false;
    for (uint8_t j = 0; j < this->merged_count_; j++) {
      if (this->merged_[j].tag == item.tag) {
        this->merged_[j] = item;
        replaced = true;
        break;
      }
    }
    if (replaced) {
      continue;
    }

    if (this->merged_count_ >= MAX_MERGED_ITEMS) {
      // Unreachable - duplicates are folded above - but bounded rather than trusted.
      ESP_LOGE(TAG, "More than %u distinct TAGs in one cycle - TAG 0x%02X dropped",
               static_cast<unsigned>(MAX_MERGED_ITEMS), item.tag);
      return;
    }
    this->merged_[this->merged_count_++] = item;
  }
}

const ParsedItem *NartisRf2MeterComponent::find_merged_(uint8_t tag) const {
  for (uint8_t i = 0; i < this->merged_count_; i++) {
    if (this->merged_[i].tag == tag) {
      return &this->merged_[i];
    }
  }
  return nullptr;
}

void NartisRf2MeterComponent::report_silent_fixed_() {
  // A meter with no fixed-block handler answers nothing, which for a cycle or two
  // is indistinguishable from a bad link.
  if (this->warned_fixed_silent_ || this->cycles_ < REQUEST_SILENT_WARN_CYCLES) {
    return;
  }
  const uint8_t silent = static_cast<uint8_t>(this->fixed_polled_ & ~this->fixed_seen_);
  if (silent == 0) {
    return;
  }
  this->warned_fixed_silent_ = true;
  for (uint8_t i = 0; i < FIXED_REQUEST_COUNT; i++) {
    if ((silent & (1u << i)) == 0) {
      continue;
    }
    ESP_LOGW(TAG,
             "Fixed block DI 0x%04X has not answered once in %" PRIu32 " cycle(s) - most likely this meter "
             "does not implement it. It still costs one exchange per cycle",
             FIXED_REQUESTS[i].di, this->cycles_);
  }
}

void NartisRf2MeterComponent::report_silent_requests_() {
  // A meter configured with only one of the two lists answers nothing at all to
  // the other, which for a cycle or two looks exactly like a bad link.
  if (this->warned_silent_requests_ || this->cycles_ < REQUEST_SILENT_WARN_CYCLES) {
    return;
  }
  const uint8_t silent = static_cast<uint8_t>(this->requests_polled_ & ~this->requests_seen_);
  if (silent == 0) {
    return;
  }
  this->warned_silent_requests_ = true;

  for (uint8_t i = 0; i < LIST_REQUEST_COUNT; i++) {
    if ((silent & (1u << i)) == 0) {
      continue;
    }
    const ListRequest &req = LIST_REQUESTS[i];
    ESP_LOGW(TAG,
             "DI 0x%04X (list %s, %s) has not answered once in %" PRIu32 " cycle(s) - most likely this "
             "meter does not have that list. It still costs one exchange per cycle",
             req.di, list_id_to_string(req.list), (req.part == ListPart::STATUS) ? "status half" : "records half",
             this->cycles_);
  }
}

void NartisRf2MeterComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "Nartis RF-2 meter:");
  ESP_LOGCONFIG(TAG, "  Meter address: %s", this->address_.c_str());
  ESP_LOGCONFIG(TAG, "  Frequency: %.3f MHz%s", this->rf_frequency_hz_ / 1e6f,
                (this->frequency_override_ != 0) ? " (override)" : " (derived from address)");
  ESP_LOGCONFIG(TAG, "  RX centre offset: %d codes (~%.1f kHz)", this->rx_center_offset_,
                this->rx_center_offset_ * RX_CODE_HZ / 1000.0f);
  ESP_LOGCONFIG(TAG, "  RX timeout: %" PRIu32 " ms, retries: %u, request gap: %" PRIu32 " ms", this->rf_rx_timeout_ms_,
                this->rf_retries_, this->request_gap_ms_);
  ESP_LOGCONFIG(TAG, "  Sources: list A %s, list B %s, fixed blocks %s",
                YESNO(this->read_list_[static_cast<uint8_t>(ListId::A)]),
                YESNO(this->read_list_[static_cast<uint8_t>(ListId::B)]), YESNO(this->read_fixed_));
  ESP_LOGCONFIG(TAG, "  Polling: list records %s, status blocks %s", YESNO(this->need_data_),
                YESNO(this->need_data_ || this->need_status_));
  ESP_LOGCONFIG(TAG, "  Values are published scaled to the unit in tags.md - no multiply filter needed");
  LOG_BINARY_SENSOR("  ", "Last read OK", this->last_read_ok_bs_);  // macro is nullptr-safe
  LOG_SENSOR("  ", "RSSI", this->rssi_sensor_);
  LOG_PIN("  SDIO pin: ", this->pin_sdio_);
  LOG_PIN("  SCLK pin: ", this->pin_sclk_);
  LOG_PIN("  CSB pin: ", this->pin_csb_);
  LOG_PIN("  FCSB pin: ", this->pin_fcsb_);
  LOG_PIN("  GPIO3 pin: ", this->pin_gpio3_);
  LOG_UPDATE_INTERVAL(this);

  // Print the frames we will actually put on the air, to compare against a capture.
  std::array<uint8_t, MAX_REQUEST_FRAME_SIZE> frame{};
  for (const ListRequest &req : LIST_REQUESTS) {
    if (!this->read_list_[static_cast<uint8_t>(req.list)]) {
      continue;  // not a source this cycle asks for, so not a frame we will send
    }
    const size_t n = build_request(frame.data(), frame.size(), this->serial_le_, req.di);
    const char *half = (req.part == ListPart::STATUS) ? "status " : "records";
    if (n == 0) {
      ESP_LOGCONFIG(TAG, "  List %s %s DI 0x%04X: FAILED TO BUILD", list_id_to_string(req.list), half, req.di);
    } else {
      ESP_LOGCONFIG(TAG, "  List %s %s DI 0x%04X: %s", list_id_to_string(req.list), half, req.di,
                    format_hex_pretty(frame.data(), n).c_str());
    }
  }

  if (this->read_fixed_) {
    for (const FixedRequest &req : FIXED_REQUESTS) {
      if (SKIP_F101 && req.di == DI_FIXED_F101) {
        ESP_LOGCONFIG(TAG, "  Fixed block DI 0x%04X: temporarily not queried", req.di);
        continue;
      }
      const size_t n = build_request(frame.data(), frame.size(), this->serial_le_, req.di);
      if (n == 0) {
        ESP_LOGCONFIG(TAG, "  Fixed block DI 0x%04X: FAILED TO BUILD", req.di);
      } else {
        ESP_LOGCONFIG(TAG, "  Fixed block DI 0x%04X: %s", req.di, format_hex_pretty(frame.data(), n).c_str());
      }
    }
  }

  for (uint8_t i = 0; i < this->probe_count_; i++) {
    const ProbeRequest &probe = this->probes_[i];
    std::array<uint8_t, MAX_REQUEST_FRAME_SIZE> pframe{};
    const size_t n =
        build_read_request(pframe.data(), pframe.size(), this->serial_le_, probe.di, probe.body, probe.body_len);
    ESP_LOGCONFIG(TAG, "  Probe DI 0x%04X body %s -> %s", probe.di,
                  format_hex_pretty(probe.body, probe.body_len).c_str(),
                  (n == 0) ? "FAILED TO BUILD" : format_hex_pretty(pframe.data(), n).c_str());
  }

  for (const auto &e : this->entries_) {
    if (e.reads_status()) {
      ESP_LOGCONFIG(TAG, "  Entity: status field %u", static_cast<unsigned>(e.status));
    } else {
      ESP_LOGCONFIG(TAG, "  Entity: TAG 0x%02X", e.tag);
    }
  }
}

void NartisRf2MeterComponent::add_probe(uint16_t di, const std::vector<uint8_t> &body) {
  if (this->probe_count_ >= MAX_PROBES) {
    ESP_LOGE(TAG, "Too many probes configured (max %u) - DI 0x%04X ignored", static_cast<unsigned>(MAX_PROBES), di);
    return;
  }
  if (body.size() > MAX_REQUEST_BODY) {
    ESP_LOGE(TAG, "Probe DI 0x%04X body too long (%zu > %u)", di, body.size(),
             static_cast<unsigned>(MAX_REQUEST_BODY));
    return;
  }
  ProbeRequest &probe = this->probes_[this->probe_count_++];
  probe.di = di;
  probe.body_len = static_cast<uint8_t>(body.size());
  for (size_t i = 0; i < body.size(); i++) {
    probe.body[i] = body[i];
  }
}

void NartisRf2MeterComponent::register_sensor(esphome::sensor::Sensor *s, uint8_t tag, StatusField field,
                                              uint8_t width) {
  this->note_tag_width_(tag, field, width);
  SensorEntry e;
  e.sensor = s;
  e.tag = tag;
  e.status = field;
  this->entries_.push_back(e);
  if (field == StatusField::NONE) {
    this->need_data_ = true;
  } else {
    this->need_status_ = true;
  }
}

void NartisRf2MeterComponent::register_text_sensor(esphome::text_sensor::TextSensor *s, uint8_t tag,
                                                   StatusField field, uint8_t width) {
  this->note_tag_width_(tag, field, width);
  SensorEntry e;
  e.text_sensor = s;
  e.tag = tag;
  e.status = field;
  this->entries_.push_back(e);
  if (field == StatusField::NONE) {
    this->need_data_ = true;
  } else {
    this->need_status_ = true;
  }
}

void NartisRf2MeterComponent::update() {
  if (!this->radio_ready_) {
    return;
  }
  if (this->state_ != State::IDLE) {
    ESP_LOGW(TAG, "Previous cycle still running (%s) - skipping this update",
             LOG_STR_ARG(state_to_string_(this->state_)));
    return;
  }
  this->start_cycle_();
}

void NartisRf2MeterComponent::start_cycle_() {
  this->cycles_++;
  this->cycle_start_ms_ = millis();
  this->merged_count_ = 0;
  this->status_ok_ = false;
  this->answered_ = 0;
  for (uint8_t l = 0; l < LIST_COUNT; l++) {
    this->list_announced_[l] = 0;
    this->list_arrived_[l] = 0;
  }
  this->attempt_ = 0;
  this->rssi_valid_ = false;

  this->f101_ok_ = false;
  this->f102_len_ = 0;

  // Walk LIST_REQUESTS in order: a status half must follow its own records half
  // back to back, or the cursor holding the leftover records is already gone.
  //
  // Two gates, both of which have to pass: the YAML's source gate, and what the
  // entities need. A status half is needed either way, since it carries both the
  // status block and the records that did not fit.
  this->step_count_ = 0;
  this->step_idx_ = 0;
  for (uint8_t i = 0; i < LIST_REQUEST_COUNT; i++) {
    if (!this->read_list_[static_cast<uint8_t>(LIST_REQUESTS[i].list)]) {
      continue;
    }
    const bool wanted = (LIST_REQUESTS[i].part == ListPart::RECORDS)
                            ? this->need_data_
                            : (this->need_data_ || this->need_status_);
    if (wanted) {
      this->steps_[this->step_count_++] = Step{StepKind::LIST, i};
    }
  }

  // The fixed blocks go last. A request of any kind drops the lists' cursor, so
  // they must not land between a records half and its status half. Not
  // capability-gated: asking for them is the point, the log is the product.
  if (this->read_fixed_) {
    for (uint8_t i = 0; i < FIXED_REQUEST_COUNT; i++) {
      if (SKIP_F101 && FIXED_REQUESTS[i].di == DI_FIXED_F101) {
        continue;
      }
      this->steps_[this->step_count_++] = Step{StepKind::FIXED, i};
      this->fixed_polled_ |= static_cast<uint8_t>(1u << i);
    }
  }

  for (uint8_t i = 0; i < this->probe_count_; i++) {
    this->steps_[this->step_count_++] = Step{StepKind::PROBE, i};
  }

  if (this->step_count_ > 0) {
    this->set_state_(State::TX_REQUEST);
  }
}

uint16_t NartisRf2MeterComponent::current_di_() const {
  const Step &step = this->steps_[this->step_idx_];
  if (step.kind == StepKind::PROBE) {
    return this->probes_[step.idx].di;
  }
  if (step.kind == StepKind::FIXED) {
    return FIXED_REQUESTS[step.idx].di;
  }
  return LIST_REQUESTS[step.idx].di;
}

void NartisRf2MeterComponent::loop() {
  switch (this->state_) {
    case State::NOT_INITIALIZED:
    case State::IDLE:
      break;

    case State::TX_REQUEST:
      if (this->send_request_()) {
        this->set_state_(State::WAIT_REPLY);
      } else {
        this->retry_or_finish_();
      }
      break;

    case State::WAIT_REPLY:
      this->handle_wait_();
      break;

    case State::GAP:
      if (millis() - this->state_entered_ms_ >= this->request_gap_ms_) {
        this->attempt_ = 0;
        this->set_state_(State::TX_REQUEST);
      }
      break;

    case State::PUBLISH:
      this->handle_publish_();
      this->set_state_(State::IDLE);
      break;

    default:
      ESP_LOGE(TAG, "Unhandled state %u - returning to idle", static_cast<unsigned>(this->state_));
      this->hal_.go_standby();
      this->set_state_(State::IDLE);
      break;
  }
}

bool NartisRf2MeterComponent::send_request_() {
  const Step &step = this->steps_[this->step_idx_];
  const uint16_t di = this->current_di_();
  this->attempt_++;

  if (step.kind == StepKind::PROBE) {
    const ProbeRequest &probe = this->probes_[step.idx];
    this->tx_len_ = build_read_request(this->tx_buf_.data(), this->tx_buf_.size(), this->serial_le_, probe.di,
                                       probe.body, probe.body_len);
  } else {
    this->tx_len_ = build_request(this->tx_buf_.data(), this->tx_buf_.size(), this->serial_le_, di);
  }
  if (this->tx_len_ == 0) {
    ESP_LOGE(TAG, "Failed to build request for DI 0x%04X", di);
    return false;
  }
  if (step.kind == StepKind::LIST) {
    this->requests_polled_ |= static_cast<uint8_t>(1u << step.idx);
  }

  ESP_LOGD(TAG, "TX DI 0x%04X%s attempt %u: %s", di, (step.kind == StepKind::PROBE) ? " (probe)" : "", this->attempt_,
           format_hex_pretty(this->tx_buf_.data(), this->tx_len_).c_str());

  // transmit() is synchronous: it applies the TX profile, fills the FIFO and blocks
  // until TX_DONE - tens of milliseconds for a 28-byte frame at 1.2 kbps.
  if (!this->hal_.transmit(this->tx_buf_.data(), this->tx_len_)) {
    ESP_LOGW(TAG, "DI 0x%04X: transmit did not complete", di);
    return false;
  }
  if (!this->hal_.begin_rx(this->rx_center_offset_)) {
    ESP_LOGW(TAG, "DI 0x%04X: could not enter RX", di);
    return false;
  }

  this->rx_len_ = 0;
  this->rx_last_chunk_ms_ = millis();
  return true;
}

NartisRf2MeterComponent::RxPoll NartisRf2MeterComponent::poll_rx_() {
  const uint32_t now = millis();

  if (this->rx_len_ + FIFO_TH_VALUE <= this->rx_buf_.size()) {
    const size_t got = this->hal_.drain_rx(this->rx_buf_.data() + this->rx_len_, this->rx_buf_.size() - this->rx_len_);
    if (got > 0) {
      this->rx_len_ += got;
      this->rx_last_chunk_ms_ = now;
    }
  }

  if (this->rx_len_ == 0) {
    return RxPoll::NOTHING;
  }

  // The first received byte is LEN, so the whole frame is LEN + 3 bytes. Fixed-length
  // capture keeps feeding noise past it, so stop as soon as the frame is in.
  const size_t env_len = this->rx_buf_[0];
  if (env_len >= D101_HDR_AFTER_LEN + DLT645_OVERHEAD && this->rx_len_ >= env_len + 3) {
    return RxPoll::COMPLETE;
  }
  // Fallbacks for a bogus LEN: buffer full, or the chunks stopped arriving.
  if (this->rx_len_ + FIFO_TH_VALUE > this->rx_buf_.size() || (now - this->rx_last_chunk_ms_) >= RX_END_GAP_MS) {
    return RxPoll::COMPLETE;
  }
  return RxPoll::BUSY;
}

void NartisRf2MeterComponent::handle_wait_() {
  const Step &step = this->steps_[this->step_idx_];
  const uint16_t di = this->current_di_();
  const bool is_probe = (step.kind == StepKind::PROBE);

  if (this->poll_rx_() == RxPoll::COMPLETE) {
    // Read RSSI while still in RX - go_standby() would invalidate it.
    this->last_rssi_dbm_ = this->hal_.get_rssi_dbm();
    this->rssi_valid_ = true;

    // Always show what came off the air, whether or not it decodes - this is the
    // primary record for working out an unfamiliar meter's indication set.
    ESP_LOGD(TAG, "RX rssi=%d dBm: %s", this->last_rssi_dbm_,
             format_hex_pretty(this->rx_buf_.data(), this->rx_len_).c_str());

    ParsedResponse resp{};
    const ParseResult r = parse_response(this->rx_buf_.data(), this->rx_len_, this->serial_le_, &resp,
                                         this->tag_width_);

    if (r == ParseResult::OK && resp.di == di) {
      if (is_probe) {
        // Nothing consumes a probe; the log is the whole point.
        ESP_LOGI(TAG, "PROBE DI 0x%04X ANSWERED: %u item(s)", di, resp.count);
        this->log_response_(resp);
      } else if (step.kind == StepKind::FIXED) {
        this->fixed_seen_ |= static_cast<uint8_t>(1u << step.idx);
        this->handle_fixed_reply_(step.idx, resp);
      } else {
        ESP_LOGV(TAG, "DI 0x%04X: %u record(s), rssi %d dBm", di, resp.count, this->last_rssi_dbm_);
        this->log_response_(resp);
        const uint8_t bit = static_cast<uint8_t>(1u << step.idx);
        this->requests_seen_ |= bit;
        this->answered_ |= bit;
        this->handle_list_reply_(step.idx, resp);
      }
      this->finish_exchange_();
      return;
    }

    this->bad_frame_count_++;
    if (r == ParseResult::ERROR_RESPONSE) {
      // A clean, CRC-valid refusal. For a probe that is a real result: the meter
      // heard us and declined, which is very different from silence.
      ESP_LOGI(TAG, "%sDI 0x%04X REFUSED: control 0x%02X, error payload: %s", is_probe ? "PROBE " : "", di,
               resp.control, format_hex_pretty(resp.payload, resp.payload_len).c_str());
      if (resp.payload_len == 1) {
        ESP_LOGI(TAG, "  error 0x%02X: %s", resp.payload[0], dlt645_error_hint(resp.payload[0]));
      }
    } else if (r == ParseResult::OK) {
      ESP_LOGW(TAG, "DI 0x%04X: reply carries DI 0x%04X instead", di, resp.di);
    } else if (r == ParseResult::UNKNOWN_TAG) {
      this->log_unknown_tag_(di, resp);
    } else if (resp.payload_len > 0) {
      // payload_len is only set once the CRC, checksum, address and length have all
      // verified, so the bytes are sound and it is the record layout that is not.
      this->log_bad_records_(di, r, resp);
    } else {
      ESP_LOGW(TAG, "DI 0x%04X: %s", di, parse_result_to_string(r));
    }

    // A refusal is a final answer - retransmitting will not change it.
    if (r == ParseResult::ERROR_RESPONSE) {
      this->finish_exchange_();
      return;
    }
    this->retry_or_finish_();
    return;
  }

  if (millis() - this->state_entered_ms_ >= this->rf_rx_timeout_ms_) {
    if (this->rx_len_ == 0) {
      this->no_reply_count_++;
      ESP_LOGW(TAG, "%sDI 0x%04X: no reply within %" PRIu32 " ms", is_probe ? "PROBE " : "", di,
               this->rf_rx_timeout_ms_);
    } else {
      this->bad_frame_count_++;
      ESP_LOGW(TAG, "DI 0x%04X: incomplete reply (%zu bytes) within %" PRIu32 " ms", di, this->rx_len_,
               this->rf_rx_timeout_ms_);
      // A truncated frame still says the meter answered, and often how far it got.
      ESP_LOGD(TAG, "RX partial: %s", format_hex_pretty(this->rx_buf_.data(), this->rx_len_).c_str());
    }
    this->retry_or_finish_();
  }
}

void NartisRf2MeterComponent::retry_or_finish_() {
  const uint16_t di = this->current_di_();
  this->hal_.go_standby();

  if (this->attempt_ <= this->rf_retries_) {
    this->retry_count_++;
    ESP_LOGD(TAG, "DI 0x%04X: retrying (attempt %u of %u)", di, this->attempt_ + 1, this->rf_retries_ + 1);
    this->set_state_(State::TX_REQUEST);
    return;
  }

  this->giveup_count_++;
  ESP_LOGW(TAG, "DI 0x%04X: giving up after %u attempt(s)", di, this->attempt_);
  this->finish_exchange_();
}

void NartisRf2MeterComponent::finish_exchange_() {
  this->hal_.go_standby();
  this->rx_len_ = 0;

  // Every step in the list is sent; nothing is conditional on what an earlier reply
  // held.
  this->step_idx_++;
  if (this->step_idx_ < this->step_count_) {
    this->set_state_(State::GAP);
    return;
  }
  this->set_state_(State::PUBLISH);
}

void NartisRf2MeterComponent::handle_publish_() {
  this->resolve_values_();

  for (const auto &e : this->entries_) {
    if (e.reads_status()) {
      this->publish_from_status_(e);
    } else {
      this->publish_from_data_(e);
    }
  }

  // Free self-check: the sum register must equal the two tariff registers, as it did
  // on every response in the reference capture.
  if (this->merged_count_ > 0) {
    const ParsedItem *total = this->find_merged_(0x00);
    const ParsedItem *t1 = this->find_merged_(0x01);
    const ParsedItem *t2 = this->find_merged_(0x02);
    if (total != nullptr && t1 != nullptr && t2 != nullptr) {
      const uint32_t sum = item_as_u32(*t1) + item_as_u32(*t2);
      if (item_as_u32(*total) != sum) {
        ESP_LOGW(TAG, "Energy registers inconsistent: total %" PRIu32 " != T1 + T2 = %" PRIu32 " - decode is suspect",
                 item_as_u32(*total), sum);
      }
    }
  }

  // A shortfall means records the meter announced never arrived.
  for (uint8_t l = 0; l < LIST_COUNT; l++) {
    if (this->list_announced_[l] > this->list_arrived_[l]) {
      ESP_LOGD(TAG, "List %s announced %u record(s) but %u arrived", list_id_to_string(static_cast<ListId>(l)),
               this->list_announced_[l], this->list_arrived_[l]);
    }
  }

  this->report_silent_requests_();
  this->report_silent_fixed_();

  char asked[72];
  size_t at = 0;
  for (uint8_t i = 0; i < LIST_REQUEST_COUNT && at + 1 < sizeof(asked); i++) {
    const bool sent = (this->requests_polled_ & (1u << i)) != 0;
    const int n = std::snprintf(asked + at, sizeof(asked) - at, "%s%04X %s", (i != 0) ? ", " : "",
                                LIST_REQUESTS[i].di,
                                !sent ? "-" : (((this->answered_ & (1u << i)) != 0) ? "yes" : "no"));
    if (n <= 0) {
      break;
    }
    at += static_cast<size_t>(n);
  }
  ESP_LOGD(TAG, "Cycle %" PRIu32 " finished in %" PRIu32 " ms (%u merged record(s); %s)", this->cycles_,
           millis() - this->cycle_start_ms_, this->merged_count_, asked);
  ESP_LOGV(TAG, "Counters: no-reply %" PRIu32 ", bad frame %" PRIu32 ", retries %" PRIu32 ", give-ups %" PRIu32,
           this->no_reply_count_, this->bad_frame_count_, this->retry_count_, this->giveup_count_);

  // A cycle counts as successful only if every exchange it needed came back: a
  // partial cycle left some entity holding a stale value, which is what this entity
  // exists to expose. Lenient by design about which list answered - a meter
  // configured with only one is fully read from that one - and the fixed blocks are
  // left out entirely, since they publish nothing.
  bool any_records = false;
  bool any_list = false;
  for (uint8_t i = 0; i < LIST_REQUEST_COUNT; i++) {
    if (!this->read_list_[static_cast<uint8_t>(LIST_REQUESTS[i].list)]) {
      continue;
    }
    any_list = true;
    if (LIST_REQUESTS[i].part == ListPart::RECORDS && (this->answered_ & (1u << i)) != 0) {
      any_records = true;
    }
  }
  this->publish_cycle_outcome_(!any_list ||
                               ((!this->need_data_ || any_records) && (!this->need_status_ || this->status_ok_)));

  if (this->rssi_sensor_ != nullptr && this->rssi_valid_) {
    this->rssi_sensor_->publish_state(this->last_rssi_dbm_);
    ESP_LOGD(TAG, "  -> '%s' = %d dBm", this->rssi_sensor_->get_name().c_str(), this->last_rssi_dbm_);
  }
}

void NartisRf2MeterComponent::publish_cycle_outcome_(bool ok) {
  if (this->last_read_ok_bs_ == nullptr) {
    return;
  }
  this->last_read_ok_bs_->publish_state(ok);
}

void NartisRf2MeterComponent::describe_item_(const ParsedItem &item, char *out, size_t cap,
                                             const uint8_t *width_overrides) {
  const std::string hex = format_hex_pretty(item.raw, item.len);

  char interp[48] = "";
  TagInfo info{};
  // The width check keeps the status block out of this: it arrives as TAG 0x00 with
  // 9 bytes, where the TAG table has a 4-byte energy register.
  if (tag_info(item.tag, &info, width_overrides) && info.width == item.len) {
    switch (info.enc) {
      case TagEnc::UINT_LE: {
        const uint32_t v = item_as_u32(item);
        std::snprintf(interp, sizeof(interp), " (%" PRIu32 " -> %.3f %s)", v, v * info.scale, info.unit);
        break;
      }
      case TagEnc::INT_LE: {
        const int32_t v = item_as_i32(item);
        std::snprintf(interp, sizeof(interp), " (%" PRId32 " -> %.3f %s)", v, v * info.scale, info.unit);
        break;
      }
      case TagEnc::BCD_LE: {
        uint32_t v = 0;
        if (item_as_bcd(item, &v)) {
          std::snprintf(interp, sizeof(interp), " (%" PRIu32 " -> %.3f %s)", v, v * info.scale, info.unit);
        } else {
          std::snprintf(interp, sizeof(interp), " (not valid BCD)");
        }
        break;
      }
      case TagEnc::BCD_LE_SIGNED: {
        int32_t v = 0;
        if (item_as_bcd_signed(item, &v)) {
          std::snprintf(interp, sizeof(interp), " (%" PRId32 " -> %.3f %s)", v, v * info.scale, info.unit);
        } else {
          std::snprintf(interp, sizeof(interp), " (not valid BCD)");
        }
        break;
      }
      case TagEnc::BCD_CLOCK: {
        char clock[20];
        if (item_clock_to_string(item, clock, sizeof(clock))) {
          std::snprintf(interp, sizeof(interp), " (%s)", clock);
        }
        break;
      }
      case TagEnc::USER:
        if (item.len <= 4) {
          std::snprintf(interp, sizeof(interp), " (%" PRIu32 ", unit unknown)", item_as_u32(item));
        }
        break;
    }
  }

  std::snprintf(out, cap, "TAG 0x%02X = %s%s", item.tag, hex.c_str(), interp);
}

void NartisRf2MeterComponent::log_response_(const ParsedResponse &resp) const {
  ESP_LOGVV(TAG, "  payload: %s", format_hex_pretty(resp.payload, resp.payload_len).c_str());
  if (resp.announced_count > resp.count) {
    // Normal: the meter announces its whole record set and sends what fits.
    ESP_LOGV(TAG, "  page holds %u of the %u record(s) the meter announced", resp.count, resp.announced_count);
  }
  char line[96];
  for (uint8_t i = 0; i < resp.count && i < MAX_ITEMS; i++) {
    describe_item_(resp.items[i], line, sizeof(line), this->tag_width_);
    ESP_LOGD(TAG, "  %s", line);
  }
}

void NartisRf2MeterComponent::log_unknown_tag_(uint16_t di, const ParsedResponse &resp) const {
  // Without a known width there is no way to find where the next item starts, so
  // everything needed to work it out by hand goes into the log.
  ESP_LOGW(TAG, "DI 0x%04X: unknown item TAG 0x%02X at payload offset %u - its value width is not known, so the",
           di, resp.unknown_tag, resp.unknown_offset);
  ESP_LOGW(TAG, "  rest of this response cannot be decoded. Your meter's indication set differs from the");
  ESP_LOGW(TAG, "  reference capture (it is configurable with the vendor tool).");
  ESP_LOGW(TAG, "  payload: %s", format_hex_pretty(resp.payload, resp.payload_len).c_str());

  if (resp.count > 0) {
    ESP_LOGW(TAG, "  decoded %u item(s) before it:", resp.count);
    char line[96];
    for (uint8_t i = 0; i < resp.count && i < MAX_ITEMS; i++) {
      describe_item_(resp.items[i], line, sizeof(line), this->tag_width_);
      ESP_LOGW(TAG, "    %s", line);
    }
  } else {
    ESP_LOGW(TAG, "  no items decoded - the unknown TAG is the first one");
  }

  if (resp.unknown_offset < resp.payload_len) {
    const uint8_t tail_len = static_cast<uint8_t>(resp.payload_len - resp.unknown_offset);
    ESP_LOGW(TAG, "  undecoded tail from offset %u: %s", resp.unknown_offset,
             format_hex_pretty(resp.payload + resp.unknown_offset, tail_len).c_str());
  }
  ESP_LOGW(TAG, "  Please report the lines above together with the parameters your display shows.");
}

void NartisRf2MeterComponent::log_bad_records_(uint16_t di, ParseResult r, const ParsedResponse &resp) const {
  ESP_LOGW(TAG, "DI 0x%04X: %s - the frame itself verified (envelope CRC, DL/T 645 checksum,", di,
           parse_result_to_string(r));
  ESP_LOGW(TAG, "  address and length all good), so it is the record layout that is not understood.");
  ESP_LOGW(TAG, "  read as: %s", payload_shape_to_string(resp.shape));
  ESP_LOGW(TAG, "  payload: %s", format_hex_pretty(resp.payload, resp.payload_len).c_str());
  if (resp.count > 0) {
    ESP_LOGW(TAG, "  decoded %u record(s) before the layout stopped adding up:", resp.count);
    char line[96];
    for (uint8_t i = 0; i < resp.count && i < MAX_ITEMS; i++) {
      describe_item_(resp.items[i], line, sizeof(line), this->tag_width_);
      ESP_LOGW(TAG, "    %s", line);
    }
  } else {
    ESP_LOGW(TAG, "  no records decoded at all");
  }
  ESP_LOGW(TAG, "  Please report the payload line together with the parameters your display shows.");
}

void NartisRf2MeterComponent::note_tag_width_(uint8_t tag, StatusField field, uint8_t width) {
  if (field != StatusField::NONE || width == 0 || tag >= TAG_WIDTH_TABLE_SIZE) {
    return;
  }
  this->tag_width_[tag] = width;
}

namespace {

const char *value_source_to_string(ValueSource s) {
  switch (s) {
    case ValueSource::LIST:
      return "list";
    case ValueSource::FIXED:
      return "F102";
    default:
      return "none";
  }
}

}  // namespace

void NartisRf2MeterComponent::resolve_values_() {
  for (ValueSlot &slot : this->values_) {
    slot = ValueSlot{};
  }

  /* The list goes in first, so it wins wherever both sources carry a TAG - not as
   * the better reading, but as the pinned one: every scale it uses has been checked
   * against a capture. Letting the fresher source win instead would make a value's
   * units depend on which requests happened to answer that cycle.
   */
  for (uint8_t i = 0; i < this->merged_count_; i++) {
    const ParsedItem &item = this->merged_[i];
    if (item.tag >= this->values_.size()) {
      continue;
    }
    TagInfo info{};
    if (!tag_info(item.tag, &info, this->tag_width_)) {
      continue;  // unreachable: the parser aborts the page on an unknown TAG
    }
    float value = 0.0f;
    if (!item_as_scaled(item, info, &value)) {
      // The clock is not a scalar and the text path reads it straight out of merged_,
      // so it is expected here. Anything else means the framing is off.
      if (info.enc != TagEnc::BCD_CLOCK) {
        ESP_LOGW(TAG, "TAG 0x%02X: %u byte(s) not valid for its encoding: %s", item.tag, item.len,
                 format_hex_pretty(item.raw, item.len).c_str());
      }
      continue;
    }
    this->values_[item.tag] = ValueSlot{value, ValueSource::LIST};
  }

  // Then the fixed blocks, filling only what no list carried: on a three-phase meter
  // reading list B that is the per-phase power, plus the reactive energy from F101.
  const FixedValue *f102_map = nullptr;
  const uint8_t f102_count = f102_value_map(this->f102_len_, &f102_map);
  this->fill_values_from_fixed_(this->f102_raw_, this->f102_len_, f102_map, f102_count, "F102");

  if (this->f101_ok_) {
    this->fill_values_from_fixed_(this->f101_raw_, sizeof(this->f101_raw_), F101_MAP, F101_VALUE_COUNT, "F101");
  }
}

void NartisRf2MeterComponent::fill_values_from_fixed_(const uint8_t *payload, uint8_t payload_len,
                                                      const FixedValue *map, uint8_t count, const char *what) {
  for (uint8_t i = 0; i < count; i++) {
    const FixedValue &fv = map[i];
    if (fv.tag >= this->values_.size() || this->values_[fv.tag].src != ValueSource::NONE) {
      continue;
    }
    float value = 0.0f;
    if (!fixed_value(payload, payload_len, fv, &value)) {
      ESP_LOGW(TAG, "%s: the field mapped to TAG 0x%02X did not decode", what, fv.tag);
      continue;
    }
    this->values_[fv.tag] = ValueSlot{value, ValueSource::FIXED};
  }
}

const ValueSlot *NartisRf2MeterComponent::find_value_(uint8_t tag) const {
  if (tag >= this->values_.size() || this->values_[tag].src == ValueSource::NONE) {
    return nullptr;
  }
  return &this->values_[tag];
}

void NartisRf2MeterComponent::publish_from_data_(const SensorEntry &e) {
  if (this->merged_count_ == 0 && this->f102_len_ == 0) {
    return;  // nothing arrived this cycle; leave the entity at its previous state
  }

  TagInfo info{};
  if (!tag_info(e.tag, &info, this->tag_width_)) {
    return;  // cannot happen: the parser would have aborted on an unknown TAG
  }

  const ValueSlot *slot = this->find_value_(e.tag);
  if (e.sensor != nullptr) {
    if (slot != nullptr) {
      e.sensor->publish_state(slot->value);
      ESP_LOGD(TAG, "  -> '%s' = %.3f %s (%s)", e.sensor->get_name().c_str(), slot->value, info.unit,
               value_source_to_string(slot->src));
    } else if (info.enc == TagEnc::BCD_CLOCK) {
      ESP_LOGW(TAG, "TAG 0x%02X is a date and time, not a number - use a text_sensor", e.tag);
    } else {
      // Every configured source was asked, so either the meter does not report this
      // TAG or its record arrived undecodable - resolve_values_() said which.
      ESP_LOGD(TAG, "TAG 0x%02X: no configured source carried it", e.tag);
    }
  }

  if (e.text_sensor == nullptr) {
    return;
  }

  char buf[32];

  // The clock is the one TAG with no scalar reading, so it goes to its own record.
  if (info.enc == TagEnc::BCD_CLOCK) {
    const ParsedItem *item = this->find_merged_(e.tag);
    if (item == nullptr) {
      ESP_LOGD(TAG, "TAG 0x%02X: no configured source carried it", e.tag);
    } else if (item_clock_to_string(*item, buf, sizeof(buf))) {
      e.text_sensor->publish_state(buf);
      ESP_LOGD(TAG, "  -> '%s' = %s", e.text_sensor->get_name().c_str(), buf);
    } else {
      ESP_LOGW(TAG, "TAG 0x%02X: clock is not valid BCD: %s", e.tag,
               format_hex_pretty(item->raw, item->len).c_str());
    }
    return;
  }

  if (slot != nullptr) {
    std::snprintf(buf, sizeof(buf), "%.3f", slot->value);
    e.text_sensor->publish_state(buf);
    ESP_LOGD(TAG, "  -> '%s' = %s %s (%s)", e.text_sensor->get_name().c_str(), buf, info.unit,
             value_source_to_string(slot->src));
    return;
  }

  // No scalar reading, but bytes did arrive - hand those over rather than invent a
  // number.
  const ParsedItem *item = this->find_merged_(e.tag);
  if (item == nullptr) {
    ESP_LOGD(TAG, "TAG 0x%02X: no configured source carried it", e.tag);
    return;
  }
  const std::string hex = format_hex_pretty(item->raw, item->len);
  e.text_sensor->publish_state(hex);
  ESP_LOGD(TAG, "  -> '%s' = %s", e.text_sensor->get_name().c_str(), hex.c_str());
}

void NartisRf2MeterComponent::publish_from_status_(const SensorEntry &e) {
  if (!this->status_ok_) {
    return;
  }

  if (e.status == StatusField::RAW) {
    if (e.text_sensor != nullptr) {
      const std::string hex = format_hex_pretty(this->status_block_, STATUS_BLOCK_SIZE);
      e.text_sensor->publish_state(hex);
      ESP_LOGD(TAG, "  -> '%s' = %s", e.text_sensor->get_name().c_str(), hex.c_str());
    } else {
      ESP_LOGW(TAG, "status: raw is only available on a text_sensor");
    }
    return;
  }

  uint8_t value = 0;
  switch (e.status) {
    case StatusField::ACTIVE_TARIFF:
      value = this->status_block_[STATUS_OFF_ACTIVE_TARIFF];
      break;
    case StatusField::TARIFF_COUNT:
      value = this->status_block_[STATUS_OFF_TARIFF_COUNT];
      break;
    default:
      return;
  }

  if (e.sensor != nullptr) {
    e.sensor->publish_state(static_cast<float>(value));
    ESP_LOGD(TAG, "  -> '%s' = %u", e.sensor->get_name().c_str(), value);
  }
  if (e.text_sensor != nullptr) {
    char buf[8];
    std::snprintf(buf, sizeof(buf), "%u", value);
    e.text_sensor->publish_state(buf);
    ESP_LOGD(TAG, "  -> '%s' = %s", e.text_sensor->get_name().c_str(), buf);
  }
}

void NartisRf2MeterComponent::set_state_(State state) {
  if (this->state_ != state) {
    ESP_LOGV(TAG, "State: %s -> %s", LOG_STR_ARG(state_to_string_(this->state_)), LOG_STR_ARG(state_to_string_(state)));
    this->state_ = state;
  }
  this->state_entered_ms_ = millis();
}

const LogString *NartisRf2MeterComponent::state_to_string_(State state) {
  switch (state) {
    case State::NOT_INITIALIZED:
      return LOG_STR("NOT_INITIALIZED");
    case State::IDLE:
      return LOG_STR("IDLE");
    case State::TX_REQUEST:
      return LOG_STR("TX_REQUEST");
    case State::WAIT_REPLY:
      return LOG_STR("WAIT_REPLY");
    case State::GAP:
      return LOG_STR("GAP");
    case State::PUBLISH:
      return LOG_STR("PUBLISH");
    default:
      return LOG_STR("UNKNOWN");
  }
}

}  // namespace esphome::nartis_rf_2_meter
