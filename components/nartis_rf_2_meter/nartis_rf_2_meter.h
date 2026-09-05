/*
 * Nartis RF-2 meter - ESPHome component.
 *
 * Emulates the НАРТИС-Д101-2 display: constant DL/T 645 read requests over the
 * 443 MHz CMT2300A link. See d101_frame.h for the wire format and the sources.
 *
 *   Cmt2300aHal   - radio (bit-bang SPI, register banks, TX/RX profiles)
 *   d101_frame.*  - envelope + DL/T 645 + item payload, pure protocol
 *   this file     - polling state machine and entity publishing
 *
 * One cycle walks the selected requests then any probes, from loop():
 *
 *   IDLE -> TX_REQUEST -> WAIT_REPLY -> GAP -> TX_REQUEST -> ... -> PUBLISH -> IDLE
 *
 * A failed exchange does not abort the cycle. Request order is load-bearing: a
 * list's status half must follow its own records half back to back - see
 * LIST_REQUESTS.
 *
 * The cycle is planned in two parts. The lists are planned up front; the fixed
 * blocks and the probes are planned once the last list exchange is done, by
 * plan_tail_steps_(), so that a fixed block goes on air only when it maps a TAG
 * some entity wants and no list delivered. On a meter whose list already carries
 * everything configured, that is one exchange saved every cycle.
 *
 * SAFETY: read-only by construction. Every frame comes from build_read_request(),
 * which hard-wires the DL/T 645 control code to "read data". This link can also
 * command the meter's load relay, so do not add a write path and do not fuzz for
 * one.
 */

#pragma once

#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/core/component.h"
#include "esphome/core/log.h"

#include "cmt2300a_hal.h"
#include "d101_frame.h"

#include <array>
#include <string>
#include <vector>

namespace esphome::nartis_rf_2_meter {

/// Which status-block field an entity reads; NONE = it reads a TAG item instead.
enum class StatusField : uint8_t {
  NONE = 0,
  ACTIVE_TARIFF,
  TARIFF_COUNT,
  RAW,
};

enum class ValueSource : uint8_t { NONE = 0, LIST, FIXED };

/// One TAG's value for this cycle, scaled into the unit tag_info() names for it.
struct ValueSlot {
  float value{0.0f};
  ValueSource src{ValueSource::NONE};
};

struct SensorEntry {
  esphome::sensor::Sensor *sensor{nullptr};
  esphome::text_sensor::TextSensor *text_sensor{nullptr};
  uint8_t tag{0};
  StatusField status{StatusField::NONE};

  bool reads_status() const { return this->status != StatusField::NONE; }
};

class NartisRf2MeterComponent : public esphome::PollingComponent {
 public:
  void setup() override;
  void dump_config() override;
  void update() override;
  void loop() override;
  float get_setup_priority() const override { return esphome::setup_priority::DATA; }

  // CMT2300A wiring: bit-bang 3-wire SPI + INT2 on the chip's GPIO3 pad.
  void set_pin_sdio(esphome::InternalGPIOPin *p) { this->pin_sdio_ = p; }
  void set_pin_sclk(esphome::InternalGPIOPin *p) { this->pin_sclk_ = p; }
  void set_pin_csb(esphome::InternalGPIOPin *p) { this->pin_csb_ = p; }
  void set_pin_fcsb(esphome::InternalGPIOPin *p) { this->pin_fcsb_ = p; }
  void set_pin_gpio3(esphome::InternalGPIOPin *p) { this->pin_gpio3_ = p; }

  /// 12-digit nameplate serial: the DL/T 645 address, and it selects the channel.
  void set_address(const std::string &address) { this->address_ = address; }
  /// Channel frequency override; 0 = derive it from the serial.
  void set_frequency_override(uint32_t hz) { this->frequency_override_ = hz; }

  void set_sources(bool list_a, bool list_b, bool fixed) {
    this->read_list_[static_cast<uint8_t>(ListId::A)] = list_a;
    this->read_list_[static_cast<uint8_t>(ListId::B)] = list_b;
    this->read_fixed_ = fixed;
  }

  void set_last_read_ok_binary_sensor(esphome::binary_sensor::BinarySensor *s) { this->last_read_ok_bs_ = s; }
  /// Diagnostic entity for the RSSI of the last reply heard in a cycle.
  void set_rssi_sensor(esphome::sensor::Sensor *s) { this->rssi_sensor_ = s; }

  void set_request_gap_ms(uint32_t ms) { this->request_gap_ms_ = ms; }
  void set_rf_rx_timeout_ms(uint32_t ms) { this->rf_rx_timeout_ms_ = ms; }
  /// Retransmissions per exchange. Total attempts = 1 + n.
  void set_rf_retries(uint8_t n) { this->rf_retries_ = n; }
  /// RX centre offset in CMT2300A frequency codes (1 code ~= 6.199 Hz).
  void set_rx_center_offset(int off) { this->rx_center_offset_ = off; }

  /// Extra read once per cycle, logged in full and driving no entity.
  void add_probe(uint16_t di, const std::vector<uint8_t> &body);

  /// `width` > 0 declares the on-wire width of a TAG the decoder does not know.
  void register_sensor(esphome::sensor::Sensor *s, uint8_t tag, StatusField field, uint8_t width);
  void register_text_sensor(esphome::text_sensor::TextSensor *s, uint8_t tag, StatusField field, uint8_t width);

  enum class State : uint8_t {
    NOT_INITIALIZED,
    IDLE,
    TX_REQUEST,  // build + transmit the current step's request, then arm RX
    WAIT_REPLY,  // drain RX, parse, retry or advance
    GAP,         // pause between exchanges
    PUBLISH,
  };

 protected:
  enum class RxPoll : uint8_t {
    NOTHING,   // no bytes yet
    BUSY,      // partial frame, keep draining
    COMPLETE,  // hand the buffer to the carve/parse step
  };

  void set_state_(State state);
  static const LogString *state_to_string_(State state);

  void start_cycle_();
  bool send_request_();
  RxPoll poll_rx_();
  void handle_wait_();
  void retry_or_finish_();
  void finish_exchange_();
  /// Fold a page into merged_; where the lists overlap, the last page to carry a
  /// TAG wins.
  void merge_records_(const ParsedResponse &resp);
  const ParsedItem *find_merged_(uint8_t tag) const;
  /// Plan the rest of the cycle - the fixed blocks worth asking for, then the
  /// probes. Called once a cycle, after the last list exchange has been folded in.
  void plan_tail_steps_();
  /// Whether this cycle still has something to gain from a fixed block.
  bool fixed_step_wanted_(uint8_t fixed_idx) const;
  /// Whether any field of `map` names a TAG an entity wants and no list carried.
  bool map_fills_wanted_(const FixedValue *map, uint8_t count) const;
  void report_silent_requests_();
  void handle_publish_();
  void publish_cycle_outcome_(bool ok);
  uint16_t current_di_() const;

  static void describe_item_(const ParsedItem &item, char *out, size_t cap,
                             const uint8_t *width_overrides);
  void note_tag_width_(uint8_t tag, StatusField field, uint8_t width);
  void log_response_(const ParsedResponse &resp) const;
  void log_unknown_tag_(uint16_t di, const ParsedResponse &resp) const;
  void log_bad_records_(uint16_t di, ParseResult r, const ParsedResponse &resp) const;

  /// One pass before any entity is touched, so a cycle publishes one consistent set.
  void resolve_values_();
  /// Fold a fixed block's mapped fields in, skipping any TAG a list already filled.
  void fill_values_from_fixed_(const uint8_t *payload, uint8_t payload_len, const FixedValue *map, uint8_t count,
                               const char *what);
  const ValueSlot *find_value_(uint8_t tag) const;

  void publish_from_data_(const SensorEntry &e);
  void publish_from_status_(const SensorEntry &e);

  Cmt2300aHal hal_;

  esphome::binary_sensor::BinarySensor *last_read_ok_bs_{nullptr};
  esphome::sensor::Sensor *rssi_sensor_{nullptr};

  esphome::InternalGPIOPin *pin_sdio_{nullptr};
  esphome::InternalGPIOPin *pin_sclk_{nullptr};
  esphome::InternalGPIOPin *pin_csb_{nullptr};
  esphome::InternalGPIOPin *pin_fcsb_{nullptr};
  esphome::InternalGPIOPin *pin_gpio3_{nullptr};

  std::string address_;
  uint8_t serial_le_[SERIAL_BCD_SIZE]{};
  uint32_t frequency_override_{0};
  uint32_t rf_frequency_hz_{0};

  uint32_t request_gap_ms_{500};
  uint32_t rf_rx_timeout_ms_{1500};
  uint8_t rf_retries_{2};
  int rx_center_offset_{758};

  State state_{State::NOT_INITIALIZED};
  uint32_t state_entered_ms_{0};
  uint32_t cycle_start_ms_{0};
  bool radio_ready_{false};
  uint8_t attempt_{0};  // on-air attempts made for the current exchange

  // Which exchanges this configuration needs - the radio does not wake for data
  // nobody consumes.
  bool need_data_{false};
  bool need_status_{false};

  static constexpr size_t MAX_PROBES = 8;
  struct ProbeRequest {
    uint16_t di{0};
    uint8_t body[MAX_REQUEST_BODY]{};
    uint8_t body_len{0};
  };
  std::array<ProbeRequest, MAX_PROBES> probes_{};
  uint8_t probe_count_{0};

  /// A cycle is a list of exchanges, built at its start: a list request, a
  /// fixed-block request, or a probe.
  enum class StepKind : uint8_t { LIST, FIXED, PROBE };
  struct Step {
    StepKind kind{StepKind::LIST};
    /// Index into LIST_REQUESTS / FIXED_REQUESTS / probes_, per `kind`.
    uint8_t idx{0};
  };
  std::array<Step, LIST_REQUEST_COUNT + FIXED_REQUEST_COUNT + MAX_PROBES> steps_{};
  uint8_t step_count_{0};
  uint8_t step_idx_{0};
  /// Whether plan_tail_steps_() has already run for this cycle.
  bool tail_planned_{false};

  void handle_list_reply_(uint8_t request_idx, const ParsedResponse &resp);
  void warn_unexpected_half_once_(uint8_t request_idx, const ParsedResponse &resp);

  void handle_fixed_reply_(uint8_t fixed_idx, const ParsedResponse &resp);
  void log_f101_() const;
  void report_silent_fixed_();
  void log_f102_() const;

  /// TAGs an entity reads, by TAG. What a cycle is trying to fill, and so what
  /// decides whether a fixed block is worth an exchange.
  bool wanted_tag_[TAG_WIDTH_TABLE_SIZE]{};

  /// YAML-declared widths by TAG, 0 = none. Consulted only for unknown TAGs.
  uint8_t tag_width_[TAG_WIDTH_TABLE_SIZE]{};

  /// This cycle's records from all TAG pages, merged. Sized to the whole TAG
  /// space, so no combination of pages can overflow it.
  static constexpr size_t MAX_MERGED_ITEMS = TAG_WIDTH_TABLE_SIZE;
  std::array<ParsedItem, MAX_MERGED_ITEMS> merged_{};
  uint8_t merged_count_{0};

  /// This cycle's scaled values by TAG - the only thing an entity reads.
  std::array<ValueSlot, TAG_WIDTH_TABLE_SIZE> values_{};

  /// This cycle's status block. Both lists end with one; the first is kept and a
  /// second compared against it.
  uint8_t status_block_[STATUS_BLOCK_SIZE]{};
  bool status_ok_{false};

  bool read_list_[LIST_COUNT]{};
  bool read_fixed_{false};

  /// This cycle's fixed blocks, kept whole. f102_len_ identifies the variant:
  /// 63 = three-phase, 23 = single-phase, 0 = nothing arrived.
  uint8_t f101_raw_[sizeof(nartis_f101)]{};
  bool f101_ok_{false};
  uint8_t f102_raw_[sizeof(f102_3ph)]{};
  uint8_t f102_len_{0};
  /// The DI 0xF102 length last seen, kept across cycles. Nothing else says which
  /// layout this meter has, so until one reply has arrived the gate has to assume
  /// the three-phase map - which is what makes the first F102 read the one that
  /// learns the layout.
  uint8_t f102_layout_len_{0};

  /// Bit per LIST_REQUESTS entry that answered this cycle.
  uint8_t answered_{0};

  /// Per list: records announced vs records that actually arrived. A shortfall is
  /// reported once a cycle.
  uint8_t list_announced_[LIST_COUNT]{};
  uint8_t list_arrived_[LIST_COUNT]{};

  // --- Diagnostics ---
  uint32_t cycles_{0};
  uint32_t no_reply_count_{0};
  uint32_t bad_frame_count_{0};
  uint32_t retry_count_{0};
  uint32_t giveup_count_{0};
  int8_t last_rssi_dbm_{0};
  /// Whether last_rssi_dbm_ was measured in THIS cycle. A cycle that heard nothing
  /// publishes no RSSI rather than repeating the last one or a floor value.
  bool rssi_valid_{false};
  /// Bit per LIST_REQUESTS entry asked for at all, and answered at least once.
  uint8_t requests_polled_{0};
  uint8_t requests_seen_{0};
  bool warned_silent_requests_{false};
  static constexpr uint32_t REQUEST_SILENT_WARN_CYCLES = 3;
  uint8_t warned_half_{0};
  /// The same, for FIXED_REQUESTS.
  uint8_t fixed_polled_{0};
  uint8_t fixed_seen_{0};
  bool warned_fixed_silent_{false};

  std::array<uint8_t, MAX_REQUEST_FRAME_SIZE> tx_buf_{};
  size_t tx_len_{0};

  /// RX accumulation. Fixed-length capture never raises PKT_DONE, so a frame is
  /// bounded by its own LEN byte. Sized from the protocol: the radio hands over
  /// 1 + LEN + 2 and LEN = 15 + payload, so the largest frame accepted is
  /// 18 + MAX_PAYLOAD = 146 bytes, and draining only happens in whole
  /// FIFO_TH_VALUE chunks.
  static constexpr size_t RX_DRAIN_CAP = 192;
  /// No new FIFO chunk for this long means reception is over.
  static constexpr uint32_t RX_END_GAP_MS = 400;
  std::array<uint8_t, RX_DRAIN_CAP> rx_buf_{};
  size_t rx_len_{0};
  uint32_t rx_last_chunk_ms_{0};

  std::vector<SensorEntry> entries_;
};

}  // namespace esphome::nartis_rf_2_meter
