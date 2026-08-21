#include "nartis_rf_2_meter.h"

#include "esphome/core/hal.h"
#include "esphome/core/helpers.h"

#include <cinttypes>
#include <cstdio>

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

  this->resolve_data_page_();

  if (!this->need_data_ && !this->need_status_) {
    ESP_LOGW(TAG, "No sensors configured - nothing will be polled");
  }

  ESP_LOGI(TAG, "CMT2300A ready at %.3f MHz (meter %s)", this->rf_frequency_hz_ / 1e6f, this->address_.c_str());
  this->set_state_(State::IDLE);
}

void NartisRf2MeterComponent::resolve_data_page_() {
  // Does anything configured live only on the larger page?
  const SensorEntry *reason = nullptr;
  for (const auto &e : this->entries_) {
    if (!e.reads_status() && tag_needs_params_page(e.tag)) {
      reason = &e;
      break;
    }
  }

  if (this->data_page_ != DATA_PAGE_AUTO) {
    // Explicit choice: honour it, but say so if it asks for more than is needed.
    if (this->data_page_ == DI_PARAMS && reason == nullptr && this->need_data_) {
      ESP_LOGI(TAG,
               "Data page DI 0x%04X is configured, but no TAG needs it - DI 0x%04X would "
               "be a shorter exchange",
               DI_PARAMS, DI_ENERGY);
    }
    return;
  }

  // DI 0xF202 is a superset of DI 0xF200 - it carries the energy registers and
  // the clock as well - so a mixed set never needs both pages polled. Ask for the
  // larger page only when something on it is actually wanted.
  this->data_page_ = (reason != nullptr) ? DI_PARAMS : DI_ENERGY;
  this->data_page_auto_ = true;
  if (reason != nullptr) {
    ESP_LOGI(TAG, "Data page: DI 0x%04X, chosen because TAG 0x%02X is only on that page", this->data_page_,
             reason->tag);
  } else {
    ESP_LOGI(TAG, "Data page: DI 0x%04X, no configured TAG needs the larger page", this->data_page_);
  }
}

void NartisRf2MeterComponent::maybe_escalate_data_page_() {
  // Which TAGs a page actually carries is set with the vendor tool, so the
  // range-based choice in resolve_data_page_() can be too optimistic: on the
  // reference meter the DI 0xF200 page holds only TAGs 0x00-0x02 and 0x29, so a
  // cumulative register such as export energy 0x05 is absent from it even though
  // nothing about the TAG says it should be. Rather than leave those entities
  // permanently unavailable, move up to the superset page once and stay there.
  //
  // Upwards only, and only for an auto-selected page, so this cannot oscillate -
  // a TAG the meter never sends at all is simply missing on 0xF202 too.
  if (!this->data_page_auto_ || this->data_page_ != DI_ENERGY || !this->data_ok_ || this->missing_tags_ == 0) {
    return;
  }
  ESP_LOGW(TAG,
           "%u configured TAG(s) are not on the DI 0x%04X page - switching to DI 0x%04X, "
           "which carries the same registers plus more",
           this->missing_tags_, DI_ENERGY, DI_PARAMS);
  this->data_page_ = DI_PARAMS;
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
  ESP_LOGCONFIG(TAG, "  Data page: DI 0x%04X%s", this->data_page_,
                (this->data_page_ == DI_PARAMS) ? " (energy + instantaneous)" : " (energy + clock)");
  ESP_LOGCONFIG(TAG, "  Instantaneous values: %s", YESNO(this->data_page_ == DI_PARAMS));
  ESP_LOGCONFIG(TAG, "  Polling: data %s, status %s", YESNO(this->need_data_), YESNO(this->need_status_));
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
  for (const uint16_t di : {this->data_page_, DI_STATUS}) {
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
  this->data_ok_ = false;
  this->status_ok_ = false;
  this->attempt_ = 0;
  this->missing_tags_ = 0;

  // Build this cycle's exchange list. Capability-gated: an exchange nobody
  // consumes is not worth waking the radio for.
  this->step_count_ = 0;
  this->step_idx_ = 0;
  if (this->need_data_) {
    this->steps_[this->step_count_++] = Step{StepKind::DATA, 0};
  }
  if (this->need_status_) {
    this->steps_[this->step_count_++] = Step{StepKind::STATUS, 0};
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
    case StepKind::DATA:
      return this->data_page_;
    case StepKind::STATUS:
      return DI_STATUS;
    case StepKind::PROBE:
      return this->probes_[step.probe_idx].di;
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
    const ProbeRequest &probe = this->probes_[step.probe_idx];
    this->tx_len_ = build_read_request(this->tx_buf_.data(), this->tx_buf_.size(), this->serial_le_, probe.di,
                                       probe.body, probe.body_len);
  } else {
    this->tx_len_ = build_request(this->tx_buf_.data(), this->tx_buf_.size(), this->serial_le_, di);
  }
  if (this->tx_len_ == 0) {
    ESP_LOGE(TAG, "Failed to build request for DI 0x%04X", di);
    return false;
  }

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

    if (r == ParseResult::OK && resp.di == di) {
      if (is_probe) {
        // Nothing consumes a probe; the log is the whole point.
        ESP_LOGI(TAG, "PROBE DI 0x%04X ANSWERED: %u item(s)", di, resp.count);
        this->log_response_(di, resp);
      } else if (step.kind == StepKind::DATA) {
        this->data_ = resp;
        this->data_ok_ = true;
        ESP_LOGV(TAG, "DI 0x%04X: %u item(s), rssi %d dBm", di, resp.count, this->last_rssi_dbm_);
        this->log_response_(di, resp);
      } else {
        this->status_ = resp;
        this->status_ok_ = true;
        ESP_LOGV(TAG, "DI 0x%04X: %u item(s), rssi %d dBm", di, resp.count, this->last_rssi_dbm_);
        this->log_response_(di, resp);
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
    } else {
      // The captured bytes were already logged above, so just say what was wrong
      // with them.
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

void NartisRf2MeterComponent::finish_exchange_() {
  this->hal_.go_standby();
  this->rx_len_ = 0;

  this->step_idx_++;
  if (this->step_idx_ < this->step_count_) {
    this->set_state_(State::GAP);
    return;
  }
  this->set_state_(State::PUBLISH);
}

void NartisRf2MeterComponent::handle_publish_() {
  this->missing_tags_ = 0;
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
  if (this->data_ok_) {
    const ParsedItem *total = this->data_.find(0x00);
    const ParsedItem *t1 = this->data_.find(0x01);
    const ParsedItem *t2 = this->data_.find(0x02);
    if (total != nullptr && t1 != nullptr && t2 != nullptr) {
      const uint32_t sum = item_as_u32(*t1) + item_as_u32(*t2);
      if (item_as_u32(*total) != sum) {
        ESP_LOGW(TAG, "Energy registers inconsistent: total %" PRIu32 " != T1 + T2 = %" PRIu32 " - decode is suspect",
                 item_as_u32(*total), sum);
      }
    }
  }

  this->maybe_escalate_data_page_();

  ESP_LOGD(TAG, "Cycle %" PRIu32 " finished in %" PRIu32 " ms (data %s, status %s)", this->cycles_,
           millis() - this->cycle_start_ms_, YESNO(this->data_ok_), YESNO(this->status_ok_));
  ESP_LOGV(TAG, "Counters: no-reply %" PRIu32 ", bad frame %" PRIu32 ", retries %" PRIu32 ", give-ups %" PRIu32,
           this->no_reply_count_, this->bad_frame_count_, this->retry_count_, this->giveup_count_);

  // A cycle counts as successful only if every exchange it needed came back. A
  // partial cycle is reported as a failure on purpose: it left some entity holding
  // a stale value, which is the thing this entity exists to expose.
  this->publish_cycle_outcome_((!this->need_data_ || this->data_ok_) &&
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
  if (!this->data_ok_) {
    return;  // nothing arrived this cycle; leave the entity at its previous state
  }

  const ParsedItem *item = this->data_.find(e.tag);
  if (item == nullptr) {
    ESP_LOGD(TAG, "TAG 0x%02X is not on the DI 0x%04X page", e.tag, this->data_page_);
    if (this->missing_tags_ < 0xFF) {
      this->missing_tags_++;
    }
    return;
  }

  TagInfo info{};
  if (!tag_info(this->data_page_, e.tag, &info, this->tag_width_)) {
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
