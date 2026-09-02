#include "nartis_rf_2_meter.h"

#include "esphome/core/hal.h"
#include "esphome/core/helpers.h"

#include <cinttypes>
#include <cstdio>
#include <cstring>

namespace esphome::nartis_rf_2_meter {

static const char *const TAG = "nartis_rf_2_meter";

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
    // Fail safe: a radio that never came up must not leave the entity unknown, or a
    // dashboard shows a blank where it should show a fault.
    this->publish_cycle_outcome_(false);
    this->mark_failed();
    return;
  }
  this->hal_.go_standby();
  this->radio_ready_ = true;

  // The diagnostic entity reports on a poll cycle, so it needs one to happen. With
  // no value entities configured nothing would ever be polled and it would sit at
  // its boot state forever, so ask for the status exchange - the shorter of the two.
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

// These two lists are the definition of the undecoded pages, and they are indexed
// in lockstep by Step::idx - keep them adjacent and the same length.
uint16_t NartisRf2MeterComponent::raw_page_di_(uint8_t idx) {
  static const uint16_t DIS[RAW_PAGE_COUNT] = {DI_F101, DI_F104};
  return (idx < RAW_PAGE_COUNT) ? DIS[idx] : 0;
}

uint8_t NartisRf2MeterComponent::raw_page_bit_(uint8_t idx) {
  static const uint8_t BITS[RAW_PAGE_COUNT] = {PAGE_BIT_F101, PAGE_BIT_F104};
  return (idx < RAW_PAGE_COUNT) ? BITS[idx] : 0;
}

uint8_t NartisRf2MeterComponent::page_bit_of_(const Step &step) {
  switch (step.kind) {
    case StepKind::ENERGY:
      return PAGE_BIT_ENERGY;
    case StepKind::PARAMS:
      return PAGE_BIT_PARAMS;
    case StepKind::PARAMS_CONT:
      return PAGE_BIT_PARAMS_CONT;
    case StepKind::F102:
      return PAGE_BIT_F102;
    case StepKind::RAW_PAGE:
      return raw_page_bit_(step.idx);
    default:
      return 0;  // the status poll and the probes are not part of the merged set
  }
}

void NartisRf2MeterComponent::merge_page_(const ParsedResponse &resp) {
  for (uint8_t i = 0; i < resp.count && i < MAX_ITEMS; i++) {
    const ParsedItem &item = resp.items[i];
    const ParsedItem *seen = this->find_merged_(item.tag);
    if (seen != nullptr) {
      // DI 0xF202 is a superset, and it is read first, so the DI 0xF200 that
      // follows repeats records already held and the first copy stands. They have
      // to agree, though: a disagreement means one of the two decodes is wrong,
      // which is worth saying out loud.
      if (seen->len != item.len || std::memcmp(seen->raw, item.raw, item.len) != 0) {
        ESP_LOGW(TAG, "TAG 0x%02X differs between pages: kept %s, DI 0x%04X sent %s", item.tag,
                 format_hex_pretty(seen->raw, seen->len).c_str(), resp.di,
                 format_hex_pretty(item.raw, item.len).c_str());
      }
      continue;
    }
    if (this->merged_count_ >= MAX_MERGED_ITEMS) {
      // Unreachable for 6-bit TAGs, since duplicates are folded above and there
      // are only 64 of them. Bounded anyway rather than trusted.
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

void NartisRf2MeterComponent::report_silent_pages_() {
  // Every cycle asks for all three TAG pages. A meter configured without one of
  // them answers nothing at all to it, which for a cycle or two looks exactly like
  // a bad link - so wait a few, then say it once.
  if (this->warned_silent_pages_ || this->cycles_ < PAGE_SILENT_WARN_CYCLES) {
    return;
  }
  const uint8_t silent = static_cast<uint8_t>(this->pages_polled_ & ~this->pages_seen_);
  if (silent == 0) {
    return;
  }
  this->warned_silent_pages_ = true;

  struct PageName {
    uint8_t bit;
    uint16_t di;
  };
  static const PageName PAGES[] = {{PAGE_BIT_PARAMS, DI_PARAMS},
                                   {PAGE_BIT_PARAMS_CONT, DI_PARAMS_CONT},
                                   {PAGE_BIT_ENERGY, DI_ENERGY},
                                   {PAGE_BIT_F102, DI_F102},
                                   {PAGE_BIT_F101, DI_F101},
                                   {PAGE_BIT_F104, DI_F104}};
  for (const PageName &page : PAGES) {
    if ((silent & page.bit) != 0) {
      ESP_LOGW(TAG,
               "DI 0x%04X has not answered once in %" PRIu32 " cycle(s) - most likely this meter does not "
               "implement it. It still costs one exchange per cycle",
               page.di, this->cycles_);
    }
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
  ESP_LOGCONFIG(TAG, "  Polling: data pages %s, status %s", YESNO(this->need_data_), YESNO(this->need_status_));
  LOG_BINARY_SENSOR("  ", "Last read OK", this->last_read_ok_bs_);  // macro is nullptr-safe
  LOG_PIN("  SDIO pin: ", this->pin_sdio_);
  LOG_PIN("  SCLK pin: ", this->pin_sclk_);
  LOG_PIN("  CSB pin: ", this->pin_csb_);
  LOG_PIN("  FCSB pin: ", this->pin_fcsb_);
  LOG_PIN("  GPIO3 pin: ", this->pin_gpio3_);
  LOG_UPDATE_INTERVAL(this);

  // Print the frames we will actually put on the air, so they can be compared
  // against a capture without having to catch a VERBOSE log line.
  std::array<uint8_t, MAX_REQUEST_FRAME_SIZE> frame{};
  for (const uint16_t di : {DI_PARAMS, DI_PARAMS_CONT, DI_ENERGY, DI_STATUS, DI_F102, DI_F101, DI_F104}) {
    const size_t n = build_request(frame.data(), frame.size(), this->serial_le_, di);
    if (n == 0) {
      ESP_LOGCONFIG(TAG, "  Request DI 0x%04X: FAILED TO BUILD", di);
    } else {
      ESP_LOGCONFIG(TAG, "  Request DI 0x%04X: %s", di, format_hex_pretty(frame.data(), n).c_str());
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
  this->energy_ok_ = false;
  this->status_ok_ = false;
  this->params_ok_ = false;
  this->params_cont_ok_ = false;
  this->f102_ok_ = false;
  this->raw_ok_ = 0;
  this->params_complete_ = false;
  this->attempt_ = 0;

  // Build this cycle's exchange list. Capability-gated: an exchange nobody
  // consumes is not worth waking the radio for.
  //
  // The order is not free. DI 0xF203 resumes the DI 0xF202 list from a cursor the
  // meter holds, and DI 0xF200 and DI 0xF202 both reset it, so the continuation has
  // to sit immediately after its DI 0xF202. Putting the pair first is what makes
  // that structural rather than a convention to remember: nothing can be inserted
  // ahead of DI 0xF203, and the reset DI 0xF200 performs happens once the tail is
  // already in hand. DI 0xF102 goes last, being the least understood of the five.
  this->step_count_ = 0;
  this->step_idx_ = 0;
  if (this->need_data_) {
    this->steps_[this->step_count_++] = Step{StepKind::PARAMS, 0};
    this->steps_[this->step_count_++] = Step{StepKind::PARAMS_CONT, 0};
    this->steps_[this->step_count_++] = Step{StepKind::ENERGY, 0};
  }
  if (this->need_status_) {
    this->steps_[this->step_count_++] = Step{StepKind::STATUS, 0};
  }
  if (this->need_data_) {
    this->steps_[this->step_count_++] = Step{StepKind::F102, 0};
    // Read but not decoded. They publish nothing, so they are gated on need_data_
    // like the rest: worth the airtime while the radio is up for real readings,
    // not worth waking it on their own.
    for (uint8_t i = 0; i < RAW_PAGE_COUNT; i++) {
      this->steps_[this->step_count_++] = Step{StepKind::RAW_PAGE, i};
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
  switch (step.kind) {
    case StepKind::ENERGY:
      return DI_ENERGY;
    case StepKind::STATUS:
      return DI_STATUS;
    case StepKind::PARAMS:
      return DI_PARAMS;
    case StepKind::PARAMS_CONT:
      return DI_PARAMS_CONT;
    case StepKind::F102:
      return DI_F102;
    case StepKind::RAW_PAGE:
      return raw_page_di_(step.idx);
    case StepKind::PROBE:
      return this->probes_[step.idx].di;
    default:
      return 0;
  }
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
  this->pages_polled_ |= page_bit_of_(step);

  // Logged at DEBUG alongside the RX dump, so a log excerpt always shows the
  // complete exchange - what we asked and what came back.
  ESP_LOGD(TAG, "TX DI 0x%04X%s attempt %u: %s", di, (step.kind == StepKind::PROBE) ? " (probe)" : "", this->attempt_,
           format_hex_pretty(this->tx_buf_.data(), this->tx_len_).c_str());

  // transmit() is synchronous: it applies the TX profile, pads and bit-reverses,
  // fills the FIFO and blocks until TX_DONE. At 1.2 kbps a 28-byte frame is tens
  // of milliseconds of airtime.
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

  // The first received byte is LEN, so the whole frame is LEN + 3 bytes (LEN +
  // content + 2-byte CRC). Fixed-length capture keeps feeding the FIFO with noise
  // past the real frame, so stop as soon as the frame is in.
  const size_t env_len = this->rx_buf_[0];
  if (env_len >= D101_HDR_AFTER_LEN + DLT645_OVERHEAD && this->rx_len_ >= env_len + 3) {
    return RxPoll::COMPLETE;
  }
  // Fallbacks for when LEN looks bogus and no clean frame is coming: buffer full,
  // or the chunks stopped arriving.
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

    // Always show what came off the air, whether or not it decodes. This is the
    // primary record for working out an unfamiliar meter's indication set, so it
    // must not be hidden behind VERBOSE.
    ESP_LOGD(TAG, "RX rssi=%d dBm: %s", this->last_rssi_dbm_,
             format_hex_pretty(this->rx_buf_.data(), this->rx_len_).c_str());

    ParsedResponse resp{};
    const ParseResult r = parse_response(this->rx_buf_.data(), this->rx_len_, this->serial_le_, &resp,
                                         this->tag_width_);

    // A DI 0xF203 reply may echo either its own data identifier or the DI 0xF202
    // list it continues - it has never been captured, and both are this exchange
    // answering.
    const bool di_matches = (resp.di == di) || (step.kind == StepKind::PARAMS_CONT && resp.di == DI_PARAMS);

    if (r == ParseResult::OK && di_matches) {
      if (is_probe) {
        // Nothing consumes a probe; the log is the whole point.
        ESP_LOGI(TAG, "PROBE DI 0x%04X ANSWERED: %u item(s)", di, resp.count);
        this->log_response_(di, resp);
      } else {
        ESP_LOGV(TAG, "DI 0x%04X: %u item(s), rssi %d dBm", di, resp.count, this->last_rssi_dbm_);
        this->log_response_(di, resp);
        this->pages_seen_ |= page_bit_of_(step);
        switch (step.kind) {
          case StepKind::STATUS:
            this->status_ = resp;
            this->status_ok_ = true;
            break;
          case StepKind::ENERGY:
            this->energy_ok_ = true;
            this->merge_page_(resp);
            break;
          case StepKind::PARAMS:
            this->params_ok_ = true;
            this->merge_page_(resp);
            // COUNT is what the meter holds; the frame carries what fits. Ask
            // DI 0xF203 for a tail only when there is one to fetch.
            this->params_complete_ = (resp.announced_count <= resp.count);
            if (this->params_complete_) {
              ESP_LOGV(TAG, "DI 0x%04X delivered all %u announced record(s) - skipping DI 0x%04X", DI_PARAMS,
                       resp.announced_count, DI_PARAMS_CONT);
            }
            break;
          case StepKind::PARAMS_CONT: {
            this->params_cont_ok_ = true;
            ESP_LOGD(TAG, "DI 0x%04X reply shape: %s", di, payload_shape_to_string(resp.shape));
            if (resp.shape == PayloadShape::COUNTED_STATUS) {
              // Not the record tail this exchange asked for: the meter answered
              // with a status block, the shape DI 0xF201 returns. Deliberately kept
              // out of the merged TAG set - there TAG 0x00 is a 4-byte energy
              // register, and folding a 9-byte block in under the same TAG would
              // collide with the real one on every cycle.
              if (!this->warned_params_cont_shape_) {
                this->warned_params_cont_shape_ = true;
                const ParsedItem *blob = resp.find(0x00);
                ESP_LOGW(TAG, "DI 0x%04X answered with a status block, not the tail of the DI 0x%04X list: %s", di,
                         DI_PARAMS, (blob == nullptr) ? "(no TAG 0x00)"
                                                      : format_hex_pretty(blob->raw, blob->len).c_str());
                ESP_LOGW(TAG, "  So on this meter DI 0x%04X is not the continuation it was taken for, and the", di);
                ESP_LOGW(TAG, "  records DI 0x%04X announces but does not send have no known way to be read.",
                         DI_PARAMS);
                ESP_LOGW(TAG, "  The block is ignored rather than published - please report the line above.");
              }
              break;
            }
            if (resp.count == 0) {
              // Nothing was pending: the cursor got cleared before we asked, or the
              // meter had already sent its whole list.
              ESP_LOGD(TAG, "DI 0x%04X returned an empty tail", di);
            }
            this->merge_page_(resp);
            break;
          }
          case StepKind::F102: {
            this->f102_ok_ = true;
            if (!this->reported_f102_shape_) {
              this->reported_f102_shape_ = true;
              // First answer from a page little is known about. Saying what shape it
              // turned out to have is most of the reason it is polled at all. A
              // fixed block has no records to count, so count its values instead.
              if (resp.shape == PayloadShape::FIXED_BLOCK) {
                ESP_LOGI(TAG, "DI 0x%04X answered: %s, %u value(s)", di, payload_shape_to_string(resp.shape),
                         static_cast<unsigned>((resp.payload_len - 3) / F102_VALUE_SIZE));
              } else {
                ESP_LOGI(TAG, "DI 0x%04X answered: %s, %u record(s)", di, payload_shape_to_string(resp.shape),
                         resp.count);
              }
            }
            if (resp.shape == PayloadShape::FIXED_BLOCK) {
              // The expected answer: instantaneous values with no TAGs on them.
              // Logged and dropped - there is no TAG to select one by, so nothing
              // here can reach an entity yet.
              this->log_f102_block_(resp);
              break;
            }
            if (resp.shape == PayloadShape::COUNTED_STATUS) {
              // A status block, not TAG-page data. Kept out of the merged set for
              // the same reason as on DI 0xF203: TAG 0x00 means a 4-byte energy
              // register there, and folding a 9-byte block in would collide with it.
              ESP_LOGD(TAG, "DI 0x%04X holds a status block - not merged into the TAG set", di);
              break;
            }
            // It fits the TAG-page layout exactly, so its records read like any
            // other page's. A TAG only this page carries still goes through
            // warn_unconfirmed_tag_once_() before it reaches an entity.
            this->merge_page_(resp);
            break;
          }
          case StepKind::RAW_PAGE:
            this->raw_ok_ |= static_cast<uint8_t>(1u << step.idx);
            this->log_raw_page_(di, step.idx, resp);
            break;
          default:
            break;
        }
      }
      this->finish_exchange_();
      return;
    }

    this->bad_frame_count_++;
    if (r == ParseResult::ERROR_RESPONSE) {
      // A clean, CRC-valid refusal. For a probe this is a real result: the meter
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
      // payload_len is only set once the envelope CRC, the DL/T 645 checksum, the
      // address and the length fields have all verified, so reaching here means the
      // bytes are sound and it is the record layout that is not understood. Show it.
      this->log_bad_records_(di, r, resp);
    } else {
      // Nothing decoded far enough to have a payload; the RX dump above is all
      // there is to say.
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
      // Show the partial capture too - a truncated frame still tells you the
      // meter answered, and often how far it got.
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

bool NartisRf2MeterComponent::skip_step_(const Step &step) const {
  return step.kind == StepKind::PARAMS_CONT && this->params_complete_;
}

void NartisRf2MeterComponent::finish_exchange_() {
  this->hal_.go_standby();
  this->rx_len_ = 0;

  this->step_idx_++;
  while (this->step_idx_ < this->step_count_ && this->skip_step_(this->steps_[this->step_idx_])) {
    this->step_idx_++;
  }
  if (this->step_idx_ < this->step_count_) {
    this->set_state_(State::GAP);
    return;
  }
  this->set_state_(State::PUBLISH);
}

void NartisRf2MeterComponent::handle_publish_() {
  for (const auto &e : this->entries_) {
    if (e.reads_status()) {
      this->publish_from_status_(e);
    } else {
      this->publish_from_data_(e);
    }
  }

  // Free self-check: the sum register must equal the two tariff registers. This
  // held on every response in the reference capture, so a mismatch means the
  // decode is wrong rather than the meter being odd.
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

  this->report_silent_pages_();

  ESP_LOGD(TAG,
           "Cycle %" PRIu32 " finished in %" PRIu32 " ms (%u merged record(s); "
           "F202 %s, F203 %s, F200 %s, F201 %s, F102 %s, F101 %s, F104 %s)",
           this->cycles_, millis() - this->cycle_start_ms_, this->merged_count_, YESNO(this->params_ok_),
           this->params_complete_ ? "skipped" : YESNO(this->params_cont_ok_), YESNO(this->energy_ok_),
           YESNO(this->status_ok_), YESNO(this->f102_ok_), YESNO((this->raw_ok_ & raw_page_bit_(0)) != 0),
           YESNO((this->raw_ok_ & raw_page_bit_(1)) != 0));
  ESP_LOGV(TAG, "Counters: no-reply %" PRIu32 ", bad frame %" PRIu32 ", retries %" PRIu32 ", give-ups %" PRIu32,
           this->no_reply_count_, this->bad_frame_count_, this->retry_count_, this->giveup_count_);

  // A cycle counts as successful only if every exchange it needed came back. A
  // partial cycle is reported as a failure on purpose: it left some entity holding
  // a stale value, which is the thing this entity exists to expose.
  //
  // "Needed" is per-kind, not per-page. DI 0xF202 is a superset of DI 0xF200, so a
  // meter that implements only one of them is fully read by whichever answers, and
  // demanding both would peg this entity to false forever. The DI 0xF203 tail is
  // not required either: it is skipped when nothing is pending, and an empty tail
  // is a valid answer.
  this->publish_cycle_outcome_((!this->need_data_ || this->energy_ok_ || this->params_ok_) &&
                               (!this->need_status_ || this->status_ok_));
}

void NartisRf2MeterComponent::publish_cycle_outcome_(bool ok) {
  if (this->last_read_ok_bs_ == nullptr) {
    return;
  }
  this->last_read_ok_bs_->publish_state(ok);
}

void NartisRf2MeterComponent::describe_item_(uint16_t di, const ParsedItem &item, char *out, size_t cap,
                                             const uint8_t *width_overrides) {
  const std::string hex = format_hex_pretty(item.raw, item.len);

  // Append the interpreted value where we have one, so the log is readable
  // without decoding little-endian bytes by eye.
  char interp[48] = "";
  TagInfo info{};
  if (tag_info(di, item.tag, &info, width_overrides)) {
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
      case TagEnc::STATUS_BLOB:
        break;  // no single scalar interpretation
    }
  }

  std::snprintf(out, cap, "TAG 0x%02X = %s%s", item.tag, hex.c_str(), interp);
}

void NartisRf2MeterComponent::log_response_(uint16_t di, const ParsedResponse &resp) const {
  ESP_LOGVV(TAG, "  payload: %s", format_hex_pretty(resp.payload, resp.payload_len).c_str());
  if (resp.announced_count > resp.count) {
    // Normal: the meter announces its whole record set and sends what fits.
    ESP_LOGV(TAG, "  page holds %u of the %u record(s) the meter announced", resp.count, resp.announced_count);
  }
  char line[96];
  for (uint8_t i = 0; i < resp.count && i < MAX_ITEMS; i++) {
    describe_item_(di, resp.items[i], line, sizeof(line), this->tag_width_);
    ESP_LOGD(TAG, "  %s", line);
  }
}

void NartisRf2MeterComponent::log_unknown_tag_(uint16_t di, const ParsedResponse &resp) const {
  // An item carries no length field, so without a known width there is no way to
  // find where the next item starts - the rest of the payload is unrecoverable.
  // Everything needed to work the width out by hand goes into the log.
  ESP_LOGW(TAG, "DI 0x%04X: unknown item TAG 0x%02X at payload offset %u - its value width is not known, so the",
           di, resp.unknown_tag, resp.unknown_offset);
  ESP_LOGW(TAG, "  rest of this response cannot be decoded. Your meter's indication set differs from the");
  ESP_LOGW(TAG, "  reference capture (it is configurable with the vendor tool).");
  ESP_LOGW(TAG, "  payload: %s", format_hex_pretty(resp.payload, resp.payload_len).c_str());

  if (resp.count > 0) {
    ESP_LOGW(TAG, "  decoded %u item(s) before it:", resp.count);
    char line[96];
    for (uint8_t i = 0; i < resp.count && i < MAX_ITEMS; i++) {
      describe_item_(di, resp.items[i], line, sizeof(line), this->tag_width_);
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
      describe_item_(di, resp.items[i], line, sizeof(line), this->tag_width_);
      ESP_LOGW(TAG, "    %s", line);
    }
  } else {
    ESP_LOGW(TAG, "  no records decoded at all");
  }
  ESP_LOGW(TAG, "  Please report the payload line together with the parameters your display shows.");
}

void NartisRf2MeterComponent::log_f102_block_(const ParsedResponse &resp) const {
  F102Block b{};
  if (!parse_f102_block(resp.payload, resp.payload_len, &b)) {
    // parse_response() accepted the shape by running this same function, so a
    // failure here would mean the two disagree.
    ESP_LOGE(TAG, "DI 0x%04X: fixed block failed to re-parse: %s", DI_F102,
             format_hex_pretty(resp.payload, resp.payload_len).c_str());
    return;
  }

  if (b.count != F102_VALUE_COUNT) {
    // A different number of values than the layout describes, so position no
    // longer identifies anything. Dump them raw and say so; this is the evidence
    // a second layout would be worked out from.
    ESP_LOGW(TAG, "DI 0x%04X: %u values, not the %u the known layout has - order unknown, values raw:", DI_F102,
             b.count, static_cast<unsigned>(F102_VALUE_COUNT));
    for (uint8_t i = 0; i < b.count; i++) {
      ESP_LOGW(TAG, "  [%2u] %" PRId32, i + 1, b.values[i]);
    }
    ESP_LOGW(TAG, "  marker 0x%02X, payload: %s", b.marker,
             format_hex_pretty(resp.payload, resp.payload_len).c_str());
    return;
  }

  // Grouped the way the quantities relate rather than one line per value: three
  // lines a cycle is readable in a long log, fifteen is not. Indices are the
  // F102_VALUES order - see the layout there before rearranging these.
  const auto v = [&](uint8_t i) { return static_cast<float>(b.values[i]) * F102_VALUES[i].scale; };
  ESP_LOGD(TAG, "DI 0x%04X: P %.1f W (L1 %.1f, L2 %.1f, L3 %.1f)", DI_F102, v(0), v(1), v(2), v(3));
  ESP_LOGD(TAG, "DI 0x%04X: Q %.1f var (L1 %.1f, L2 %.1f, L3 %.1f)", DI_F102, v(4), v(5), v(6), v(7));
  ESP_LOGD(TAG, "DI 0x%04X: L1 %.2f V %.2f A | L2 %.2f V %.2f A | L3 %.2f V %.2f A | %.2f Hz", DI_F102, v(8), v(9),
           v(10), v(11), v(12), v(13), v(14));
  if (b.marker != 0x01) {
    // The one byte of the block nothing is known about. Worth a line if it ever
    // differs from the value every observed reply carries.
    ESP_LOGD(TAG, "DI 0x%04X: marker 0x%02X, not the 0x01 seen so far", DI_F102, b.marker);
  }
}

void NartisRf2MeterComponent::log_raw_page_(uint16_t di, uint8_t idx, const ParsedResponse &resp) {
  const uint8_t bit = static_cast<uint8_t>(1u << idx);
  const bool first = (this->reported_raw_pages_ & bit) == 0;
  this->reported_raw_pages_ |= bit;

  // The payload IS the result of this exchange - nothing else is done with it - so
  // it goes in the log every cycle. The first answer gets INFO instead of DEBUG:
  // that a page nobody has decoded answers at all is the finding, and it should
  // not take a DEBUG log to notice. The control code was already checked, so a
  // refusal never reaches here; it goes through the error path with its hint.
  if (first) {
    ESP_LOGI(TAG, "DI 0x%04X answered: %u byte(s) of DATA, not decoded (there is no known layout for it):", di,
             resp.payload_len);
    ESP_LOGI(TAG, "  payload: %s", format_hex_pretty(resp.payload, resp.payload_len).c_str());
  } else {
    ESP_LOGD(TAG, "DI 0x%04X: %u byte(s) not decoded, payload: %s", di, resp.payload_len,
             format_hex_pretty(resp.payload, resp.payload_len).c_str());
  }
}

void NartisRf2MeterComponent::note_tag_width_(uint8_t tag, StatusField field, uint8_t width) {
  if (field != StatusField::NONE || width == 0 || tag >= TAG_WIDTH_TABLE_SIZE) {
    return;
  }
  this->tag_width_[tag] = width;
}

void NartisRf2MeterComponent::warn_unconfirmed_tag_once_(uint8_t tag, TagConfidence conf) {
  if (tag >= TAG_WIDTH_TABLE_SIZE) {
    return;
  }
  const uint64_t bit = 1ULL << tag;
  if ((this->warned_tags_ & bit) != 0) {
    return;
  }
  this->warned_tags_ |= bit;
  ESP_LOGW(TAG, "TAG 0x%02X: value width not yet seen on this link (%s) - check the value looks sane", tag,
           tag_confidence_to_string(conf));
}

void NartisRf2MeterComponent::publish_from_data_(const SensorEntry &e) {
  if (this->merged_count_ == 0) {
    return;  // nothing arrived this cycle; leave the entity at its previous state
  }

  const ParsedItem *item = this->find_merged_(e.tag);
  if (item == nullptr) {
    // No longer a page-selection question - all three were asked for, so this TAG
    // is simply not in the meter's indication set.
    ESP_LOGD(TAG, "TAG 0x%02X is not in this meter's indication set", e.tag);
    return;
  }

  TagInfo info{};
  // Any of the three pages resolves the same table, so the DI here only picks the
  // TAG-page family rather than a specific page.
  if (!tag_info(DI_PARAMS, e.tag, &info, this->tag_width_)) {
    return;  // cannot happen: the parser would have aborted on an unknown TAG
  }
  if (info.conf != TagConfidence::OBSERVED) {
    this->warn_unconfirmed_tag_once_(e.tag, info.conf);
  }

  if (e.sensor != nullptr) {
    switch (info.enc) {
      case TagEnc::UINT_LE:
      case TagEnc::USER: {
        // Published raw. Scaling is left to a `multiply` filter so the YAML
        // says what unit the entity is in.
        const uint32_t value = item_as_u32(*item);
        e.sensor->publish_state(static_cast<float>(value));
        ESP_LOGD(TAG, "  -> '%s' = %" PRIu32, e.sensor->get_name().c_str(), value);
        break;
      }
      case TagEnc::INT_LE: {
        const int32_t value = item_as_i32(*item);
        e.sensor->publish_state(static_cast<float>(value));
        ESP_LOGD(TAG, "  -> '%s' = %" PRId32, e.sensor->get_name().c_str(), value);
        break;
      }
      case TagEnc::BCD_LE: {
        uint32_t value = 0;
        if (!item_as_bcd(*item, &value)) {
          ESP_LOGW(TAG, "TAG 0x%02X: value is not valid BCD: %s", e.tag,
                   format_hex_pretty(item->raw, item->len).c_str());
          break;
        }
        e.sensor->publish_state(static_cast<float>(value));
        ESP_LOGD(TAG, "  -> '%s' = %" PRIu32, e.sensor->get_name().c_str(), value);
        break;
      }
      default:
        ESP_LOGW(TAG, "TAG 0x%02X is not a numeric value - use a text_sensor", e.tag);
        break;
    }
  }

  if (e.text_sensor != nullptr) {
    char buf[24];
    switch (info.enc) {
      case TagEnc::BCD_CLOCK:
        if (item_clock_to_string(*item, buf, sizeof(buf))) {
          e.text_sensor->publish_state(buf);
          ESP_LOGD(TAG, "  -> '%s' = %s", e.text_sensor->get_name().c_str(), buf);
        } else {
          ESP_LOGW(TAG, "TAG 0x%02X: clock is not valid BCD: %s", e.tag,
                   format_hex_pretty(item->raw, item->len).c_str());
        }
        break;
      case TagEnc::INT_LE:
        std::snprintf(buf, sizeof(buf), "%" PRId32, item_as_i32(*item));
        e.text_sensor->publish_state(buf);
        ESP_LOGD(TAG, "  -> '%s' = %s", e.text_sensor->get_name().c_str(), buf);
        break;
      case TagEnc::BCD_LE: {
        uint32_t value = 0;
        if (!item_as_bcd(*item, &value)) {
          ESP_LOGW(TAG, "TAG 0x%02X: value is not valid BCD: %s", e.tag,
                   format_hex_pretty(item->raw, item->len).c_str());
          break;
        }
        std::snprintf(buf, sizeof(buf), "%" PRIu32, value);
        e.text_sensor->publish_state(buf);
        ESP_LOGD(TAG, "  -> '%s' = %s", e.text_sensor->get_name().c_str(), buf);
        break;
      }
      default:
        if (item->len > 4) {
          // Too wide for a scalar, so hand over the raw bytes rather than
          // invent a number.
          const std::string hex = format_hex_pretty(item->raw, item->len);
          e.text_sensor->publish_state(hex);
          ESP_LOGD(TAG, "  -> '%s' = %s", e.text_sensor->get_name().c_str(), hex.c_str());
        } else {
          std::snprintf(buf, sizeof(buf), "%" PRIu32, item_as_u32(*item));
          e.text_sensor->publish_state(buf);
          ESP_LOGD(TAG, "  -> '%s' = %s", e.text_sensor->get_name().c_str(), buf);
        }
        break;
    }
  }
}

void NartisRf2MeterComponent::publish_from_status_(const SensorEntry &e) {
  if (!this->status_ok_) {
    return;
  }

  const ParsedItem *item = this->status_.find(0x00);
  if (item == nullptr || item->len != STATUS_VALUE_SIZE) {
    ESP_LOGW(TAG, "Status block missing or wrong size");
    return;
  }

  if (e.status == StatusField::RAW) {
    if (e.text_sensor != nullptr) {
      const std::string hex = format_hex_pretty(item->raw, item->len);
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
      value = item->raw[STATUS_OFF_ACTIVE_TARIFF];
      break;
    case StatusField::TARIFF_COUNT:
      value = item->raw[STATUS_OFF_TARIFF_COUNT];
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
