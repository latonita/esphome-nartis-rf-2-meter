/*
 * Nartis RF-2 meter - ESPHome component.
 *
 * Emulates the НАРТИС-Д101-2 display: it sends the four constant DL/T 645 read
 * requests over the 443 MHz CMT2300A link and decodes what comes back. No
 * pairing, no session, no password, no encryption - see d101_frame.h for the
 * wire format.
 *
 * Layering:
 *   Cmt2300aHal   - radio (bit-bang SPI, register banks, TX/RX profiles)
 *   d101_frame.*  - envelope + DL/T 645 + item payload, pure protocol
 *   this file     - polling state machine and entity publishing
 *
 * One poll cycle is a list of exchanges - all four data identifiers, then any
 * configured probes - walked by a state machine driven from loop(), with every
 * state bounded by a timeout:
 *
 *   IDLE -> TX_REQUEST -> WAIT_REPLY -> GAP -> TX_REQUEST -> ... -> PUBLISH -> IDLE
 *
 * An exchange no entity consumes is skipped. A failed exchange does not abort the
 * cycle - the remaining steps still run and whatever arrived is published.
 *
 * The three TAG-bearing pages are read every cycle and merged, rather than one
 * being selected from the configured TAGs. Which page carries which TAG is set
 * with the vendor tool, so selection was guesswork; reading all of them costs one
 * more exchange and no guessing. The order is fixed and load-bearing:
 *
 *   DI 0xF200 -> DI 0xF201 -> DI 0xF202 -> DI 0xF203
 *
 * DI 0xF203 resumes the DI 0xF202 record list from a cursor the meter keeps, and
 * both DI 0xF200 and DI 0xF202 clear that cursor, so DI 0xF203 must follow its
 * DI 0xF202 with nothing in between. It is skipped altogether when DI 0xF202
 * already delivered every record it announced.
 *
 * SAFETY: this component is read-only by construction. Every frame it can build
 * comes from build_read_request(), which hard-wires the DL/T 645 control code to
 * "read data"; there is no code path that emits any other control code. Probes may
 * vary the data identifier and the request body, never the control code.
 *
 * That matters here because the display can also command the meter's load relay
 * over this same link, and an unintended relay operation is a real-world hazard.
 * Do not add a write path, and do not fuzz for one.
 *
 * Embedded constraints: no heap allocation after setup() (fixed std::array
 * buffers; the entity list is filled by register_*() from generated code before
 * setup() runs), every wait state has a timeout, and the state switch has a
 * default safe branch.
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

/// Which field of the DI 0xF201 status block an entity reads.
/// NONE => the entity reads a DI 0xF200 item selected by its TAG instead.
enum class StatusField : uint8_t {
  NONE = 0,
  ACTIVE_TARIFF,
  TARIFF_COUNT,
  RAW,
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

  // --- CMT2300A wiring (bit-bang 3-wire SPI + INT2 on the chip's GPIO3 pad) ---
  void set_pin_sdio(esphome::InternalGPIOPin *p) { this->pin_sdio_ = p; }
  void set_pin_sclk(esphome::InternalGPIOPin *p) { this->pin_sclk_ = p; }
  void set_pin_csb(esphome::InternalGPIOPin *p) { this->pin_csb_ = p; }
  void set_pin_fcsb(esphome::InternalGPIOPin *p) { this->pin_fcsb_ = p; }
  void set_pin_gpio3(esphome::InternalGPIOPin *p) { this->pin_gpio3_ = p; }

  /// 12-digit meter serial from the nameplate. Becomes the DL/T 645 address and
  /// selects the channel frequency.
  void set_address(const std::string &address) { this->address_ = address; }
  /// Override the channel frequency instead of deriving it from the serial.
  /// 0 = derive (the normal case).
  void set_frequency_override(uint32_t hz) { this->frequency_override_ = hz; }

  /// Diagnostic entity: true when the last poll cycle got everything it asked for.
  /// Optional - nullptr when the YAML declares no binary sensor.
  void set_last_read_ok_binary_sensor(esphome::binary_sensor::BinarySensor *s) { this->last_read_ok_bs_ = s; }

  /// Pause between exchanges.
  void set_request_gap_ms(uint32_t ms) { this->request_gap_ms_ = ms; }
  /// How long to wait for a reply before retrying (per on-air attempt).
  void set_rf_rx_timeout_ms(uint32_t ms) { this->rf_rx_timeout_ms_ = ms; }
  /// Retransmissions per exchange on no-reply or a bad frame. Total attempts =
  /// 1 + rf_retries.
  void set_rf_retries(uint8_t n) { this->rf_retries_ = n; }
  /// RX centre offset in CMT2300A frequency codes (1 code ~= 6.199 Hz); shifts
  /// the RX-half LO onto the meter's reply carrier.
  void set_rx_center_offset(int off) { this->rx_center_offset_ = off; }

  /// Queue an extra read of `di` with an explicit body, transmitted once per poll
  /// cycle after the normal exchanges. Diagnostic only: the reply is logged in
  /// full and drives no entity. Always a read - see build_read_request().
  void add_probe(uint16_t di, const std::vector<uint8_t> &body);

  /// `width` > 0 declares the on-wire value width for a TAG the decoder does not
  /// know, so it can be walked past and read. Ignored for TAGs with a built-in
  /// width, and for status-block entities.
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
  /// Build and transmit the current step's request, then arm RX. False if the
  /// radio refused, in which case the caller gives up on this exchange.
  bool send_request_();
  /// Drain the RX FIFO into rx_buf_ and report whether a frame is ready.
  RxPoll poll_rx_();
  /// WAIT_REPLY: poll, parse, retry or advance.
  void handle_wait_();
  /// Retransmit the current request if attempts remain, otherwise give up on this
  /// exchange and move on.
  void retry_or_finish_();
  /// Leave the current exchange and move to the next step of the cycle, skipping
  /// any step this cycle has since decided it does not need.
  void finish_exchange_();
  /// Fold a decoded TAG page into this cycle's merged record set. The first page
  /// to carry a TAG wins; a later page repeating it is only cross-checked, since
  /// DI 0xF202 is a superset of DI 0xF200 and the two must agree.
  void merge_page_(const ParsedResponse &resp);
  /// Merged record for `tag`, or nullptr.
  const ParsedItem *find_merged_(uint8_t tag) const;
  /// Warn once when a data identifier polled every cycle has never answered.
  void report_silent_pages_();
  void handle_publish_();
  /// Report the finished cycle on the `last_read_ok` entity, if one is configured.
  void publish_cycle_outcome_(bool ok);
  /// DI of the step currently in flight, for logging.
  uint16_t current_di_() const;

  /// Render one item as "TAG 0x00 = 35.4B.C9.00 (13191989 Wh)" for the log.
  static void describe_item_(uint16_t di, const ParsedItem &item, char *out, size_t cap,
                             const uint8_t *width_overrides);
  /// Record a YAML-declared value width for a TAG page item.
  void note_tag_width_(uint8_t tag, StatusField field, uint8_t width);
  /// DEBUG breakdown of a good response: payload plus one line per item.
  void log_response_(uint16_t di, const ParsedResponse &resp) const;
  /// WARN breakdown of a response stopped by an unrecognised TAG - everything
  /// needed to work out the missing width by hand.
  void log_unknown_tag_(uint16_t di, const ParsedResponse &resp) const;

  void publish_from_data_(const SensorEntry &e);
  void publish_from_status_(const SensorEntry &e);
  /// Warn once per TAG when publishing a value whose width has not actually
  /// been seen on this link, naming where the width came from.
  void warn_unconfirmed_tag_once_(uint8_t tag, TagConfidence conf);

  Cmt2300aHal hal_;

  /// Optional diagnostic entity; nullptr unless the YAML declares one.
  esphome::binary_sensor::BinarySensor *last_read_ok_bs_{nullptr};

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

  // Which exchanges this configuration actually needs (capability-gated polling:
  // there is no point waking the radio for data nobody consumes). need_data_
  // covers all three TAG pages - they are no longer selected between.
  bool need_data_{false};
  bool need_status_{false};

  /// Extra reads to try once per cycle, purely to see what comes back.
  static constexpr size_t MAX_PROBES = 8;
  struct ProbeRequest {
    uint16_t di{0};
    uint8_t body[MAX_REQUEST_BODY]{};
    uint8_t body_len{0};
  };
  std::array<ProbeRequest, MAX_PROBES> probes_{};
  uint8_t probe_count_{0};

  /// One poll cycle is a list of exchanges: the energy poll, the status poll, then
  /// each probe. Built at the start of every cycle so the state machine is just
  /// "transmit step, await reply, advance" regardless of how many there are.
  enum class StepKind : uint8_t { ENERGY, STATUS, PARAMS, PARAMS_CONT, PROBE };
  struct Step {
    StepKind kind{StepKind::ENERGY};
    uint8_t probe_idx{0};
  };
  std::array<Step, 4 + MAX_PROBES> steps_{};
  uint8_t step_count_{0};
  uint8_t step_idx_{0};

  /// True when `step` no longer needs sending - currently only the DI 0xF203
  /// continuation, once DI 0xF202 has delivered its whole record set.
  bool skip_step_(const Step &step) const;
  /// PAGE_BIT_* this step maintains, or 0 for steps outside the merged TAG set.
  static uint8_t page_bit_of_(StepKind kind);

  /// YAML-declared value widths, indexed by TAG; 0 = none. Consulted by the
  /// parser only after every built-in width has been ruled out.
  uint8_t tag_width_[TAG_WIDTH_TABLE_SIZE]{};

  /// This cycle's records from the three TAG pages, merged into one lookup so an
  /// entity does not care which page carried its TAG. A TAG is a 6-bit index, so
  /// 64 is the true upper bound on distinct records and no combination of pages
  /// can overflow it.
  static constexpr size_t MAX_MERGED_ITEMS = 64;
  std::array<ParsedItem, MAX_MERGED_ITEMS> merged_{};
  uint8_t merged_count_{0};

  ParsedResponse status_{};
  bool status_ok_{false};
  /// Whether each TAG page answered this cycle. The DI 0xF203 tail decides
  /// nothing on its own - an empty tail is a legitimate answer.
  bool energy_ok_{false};
  bool params_ok_{false};
  bool params_cont_ok_{false};
  /// Set when DI 0xF202 delivered every record it announced, so there is no tail
  /// for DI 0xF203 to fetch and the exchange can be dropped from this cycle.
  bool params_complete_{false};

  // --- Diagnostics ---
  uint32_t cycles_{0};
  uint32_t no_reply_count_{0};
  uint32_t bad_frame_count_{0};
  uint32_t retry_count_{0};
  uint32_t giveup_count_{0};
  int8_t last_rssi_dbm_{0};
  /// Bit per TAG page that this boot has polled at all, and that has answered at
  /// least once. Every cycle asks for all three, so a meter which does not
  /// implement one would otherwise burn airtime in silence - report_silent_pages_()
  /// says so once.
  static constexpr uint8_t PAGE_BIT_ENERGY = 1 << 0;
  static constexpr uint8_t PAGE_BIT_PARAMS = 1 << 1;
  static constexpr uint8_t PAGE_BIT_PARAMS_CONT = 1 << 2;
  /// Cycles to allow before deciding a page is not coming - enough that a marginal
  /// link burning its retry budget is not mistaken for an unimplemented page.
  static constexpr uint32_t PAGE_SILENT_WARN_CYCLES = 3;
  uint8_t pages_polled_{0};
  uint8_t pages_seen_{0};
  bool warned_silent_pages_{false};
  /// Bit per TAG (0x00..0x3F) already warned about; keeps the unconfirmed-width
  /// warning to one line per TAG per boot.
  uint64_t warned_tags_{0};

  std::array<uint8_t, MAX_REQUEST_FRAME_SIZE> tx_buf_{};
  size_t tx_len_{0};

  /// RX accumulation. The chip runs fixed-length capture and never raises
  /// PKT_DONE, so the frame is bounded by its own LEN byte, not by a timer: at
  /// 1.2 kbps the FIFO threshold only fires every ~100 ms.
  ///
  /// Size this from the protocol, not from the shortest response. The radio hands
  /// over 1 + LEN + 2 bytes, and LEN = 15 + payload, so the largest frame we
  /// accept is 18 + MAX_PAYLOAD = 146 bytes. Draining happens in whole
  /// FIFO_TH_VALUE chunks and only while rx_len_ + FIFO_TH_VALUE fits, so the
  /// usable capacity is the largest multiple of 15 below this - 180 at 192 bytes,
  /// comfortably above 146.
  ///
  /// It was 96, which capped accumulation at 90 bytes and silently truncated the
  /// 85-byte-and-up DI 0xF202 page.
  static constexpr size_t RX_DRAIN_CAP = 192;
  /// Fallback frame terminator: no new FIFO chunk for this long means reception
  /// is over. Must exceed the in-flight inter-chunk gap.
  static constexpr uint32_t RX_END_GAP_MS = 400;
  std::array<uint8_t, RX_DRAIN_CAP> rx_buf_{};
  size_t rx_len_{0};
  uint32_t rx_last_chunk_ms_{0};

  std::vector<SensorEntry> entries_;
};

}  // namespace esphome::nartis_rf_2_meter
