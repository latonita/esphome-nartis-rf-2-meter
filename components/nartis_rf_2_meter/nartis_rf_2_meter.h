/*
 * Nartis RF-2 meter - ESPHome component.
 *
 * Emulates the НАРТИС-Д101-2 display: it sends constant DL/T 645 read requests
 * over the 443 MHz CMT2300A link and decodes what comes back. No pairing, no
 * session, no password, no encryption - see d101_frame.h for the wire format, for
 * the two indication lists, and for the two fixed blocks.
 *
 * Three sources, selected in YAML (see set_sources), because each costs airtime:
 *
 *   list A   DI 0xF200 + DI 0xF201    a pre-defined indication list, tagged
 *   list B   DI 0xF202 + DI 0xF203    the other one
 *   fixed    DI 0xF101 + DI 0xF102    positional blocks, no TAGs, log-only
 *
 * Layering:
 *   Cmt2300aHal   - radio (bit-bang SPI, register banks, TX/RX profiles)
 *   d101_frame.*  - envelope + DL/T 645 + item payload, pure protocol
 *   this file     - polling state machine and entity publishing
 *
 * One poll cycle walks the selected requests and then any configured probes,
 * driven by a state machine from loop(), with every state bounded by a timeout:
 *
 *   IDLE -> TX_REQUEST -> WAIT_REPLY -> GAP -> TX_REQUEST -> ... -> PUBLISH -> IDLE
 *
 * An exchange no entity consumes is skipped. A failed exchange does not abort the
 * cycle - the remaining steps still run and whatever arrived is published.
 *
 * Both lists are read every cycle and their records merged into one lookup, so an
 * entity selects a value by TAG without caring which list carried it. Which list
 * holds what is configured per meter with the vendor tool, so choosing between
 * them would be guesswork; reading both costs two more exchanges and no guessing.
 *
 * The request order is fixed and load-bearing: each list's status half must
 * follow its own records half back to back, because the leftover records it
 * returns come from a cursor the meter drops as soon as anything else is asked.
 * LIST_REQUESTS is in that order and the cycle walks it as it stands, which is
 * what makes the rule structural rather than a convention to remember.
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

  /// Which sources a cycle reads. Each costs airtime, so this is how a meter that
  /// only needs one of them stops paying for the rest:
  ///
  ///   list A   DI 0xF200 + DI 0xF201
  ///   list B   DI 0xF202 + DI 0xF203
  ///   fixed    DI 0xF101 + DI 0xF102 - positional blocks, no TAGs
  ///
  /// Enabling none is caught in the YAML, not here.
  void set_sources(bool list_a, bool list_b, bool fixed) {
    this->read_list_[static_cast<uint8_t>(ListId::A)] = list_a;
    this->read_list_[static_cast<uint8_t>(ListId::B)] = list_b;
    this->read_fixed_ = fixed;
  }

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
  void merge_records_(const ParsedResponse &resp);
  /// Merged record for `tag`, or nullptr.
  const ParsedItem *find_merged_(uint8_t tag) const;
  /// Warn once when one of the four requests, polled every cycle, has never
  /// answered - a meter that does not implement a list stays silent on it.
  void report_silent_requests_();
  void handle_publish_();
  /// Report the finished cycle on the `last_read_ok` entity, if one is configured.
  void publish_cycle_outcome_(bool ok);
  /// DI of the step currently in flight, for logging.
  uint16_t current_di_() const;

  /// Render one item as "TAG 0x00 = 35.4B.C9.00 (13191989 Wh)" for the log.
  static void describe_item_(const ParsedItem &item, char *out, size_t cap,
                             const uint8_t *width_overrides);
  /// Record a YAML-declared value width for a TAG page item.
  void note_tag_width_(uint8_t tag, StatusField field, uint8_t width);
  /// DEBUG breakdown of a good response: payload plus one line per item.
  void log_response_(const ParsedResponse &resp) const;
  /// WARN breakdown of a response stopped by an unrecognised TAG - everything
  /// needed to work out the missing width by hand.
  void log_unknown_tag_(uint16_t di, const ParsedResponse &resp) const;
  /// WARN breakdown of a response that verified at every layer below the records
  /// but whose record layout was not understood. Same purpose as
  /// log_unknown_tag_(): put everything needed to work the layout out in the log,
  /// rather than leave a bare "malformed" and an undecoded RX dump.
  void log_bad_records_(uint16_t di, ParseResult r, const ParsedResponse &resp) const;

  void publish_from_data_(const SensorEntry &e);
  void publish_from_status_(const SensorEntry &e);

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

  /// One poll cycle is a list of exchanges, built at the start of every cycle so
  /// the state machine is just "transmit step, await reply, advance" regardless of
  /// how many there are. Three kinds of exchange: a list request, a fixed-block
  /// request, or one of the user's diagnostic probes.
  enum class StepKind : uint8_t { LIST, FIXED, PROBE };
  struct Step {
    StepKind kind{StepKind::LIST};
    /// Index into LIST_REQUESTS for LIST, FIXED_REQUESTS for FIXED, probes_ for
    /// PROBE.
    uint8_t idx{0};
  };
  std::array<Step, LIST_REQUEST_COUNT + FIXED_REQUEST_COUNT + MAX_PROBES> steps_{};
  uint8_t step_count_{0};
  uint8_t step_idx_{0};

  /// Take a reply to one of the four list requests: merge its records, and keep
  /// the status block of a status half.
  void handle_list_reply_(uint8_t request_idx, const ParsedResponse &resp);
  /// Warn once per request when a reply is framed as the other half. That is how a
  /// meter which lays its lists out differently shows up.
  void warn_unexpected_half_once_(uint8_t request_idx, const ParsedResponse &resp);

  /// Take a reply to DI 0xF101 or DI 0xF102: keep the payload and log it decoded.
  /// Nothing is published from a fixed block yet - it carries no TAGs, so there is
  /// nothing for a `tag:` entity to select by - so this is read-and-report.
  void handle_fixed_reply_(uint8_t fixed_idx, const ParsedResponse &resp);
  void log_f101_() const;
  /// Warn once when a fixed read never answers - the sign that a meter has no
  /// handler for it at all.
  void report_silent_fixed_();
  void log_f102_() const;
  /// Compare the fixed blocks against each other and against the list records.
  /// Every check here is arithmetic that must hold if the layouts are right, so a
  /// failure means a layout is wrong rather than the meter being odd.
  void cross_check_fixed_() const;

  /// YAML-declared value widths, indexed by TAG; 0 = none. Consulted by the
  /// parser only after every built-in width has been ruled out.
  uint8_t tag_width_[TAG_WIDTH_TABLE_SIZE]{};

  /// This cycle's records from the three TAG pages, merged into one lookup so an
  /// entity does not care which page carried its TAG. Sized to the whole TAG
  /// space, so it is the true upper bound on distinct records: duplicates are
  /// folded on the way in, and no combination of pages can overflow it.
  static constexpr size_t MAX_MERGED_ITEMS = TAG_WIDTH_TABLE_SIZE;
  std::array<ParsedItem, MAX_MERGED_ITEMS> merged_{};
  uint8_t merged_count_{0};

  /// The status block from this cycle, and whether one arrived. Both lists end
  /// with one; the first to arrive is kept and a second is compared against it.
  uint8_t status_block_[STATUS_BLOCK_SIZE]{};
  bool status_ok_{false};

  /// Which sources this cycle reads - see set_sources().
  bool read_list_[LIST_COUNT]{};
  bool read_fixed_{false};

  /// This cycle's fixed blocks, kept whole. `f102_len_` is what identifies the
  /// variant, so it is stored rather than the decoded shape: 63 bytes is the
  /// three-phase layout, 23 the single-phase one, 0 nothing arrived.
  uint8_t f101_raw_[sizeof(nartis_f101)]{};
  bool f101_ok_{false};
  uint8_t f102_raw_[sizeof(f102_3ph)]{};
  uint8_t f102_len_{0};

  /// Bit per LIST_REQUESTS entry that answered this cycle. The bit IS the index
  /// into that table, so there is no second naming to keep in step.
  uint8_t answered_{0};

  /// Per list: how many records it announced, and how many actually arrived
  /// across both halves. A shortfall is the interesting case - it means records
  /// the meter says it holds did not reach us - so it is reported once a cycle.
  uint8_t list_announced_[LIST_COUNT]{};
  uint8_t list_arrived_[LIST_COUNT]{};

  // --- Diagnostics ---
  uint32_t cycles_{0};
  uint32_t no_reply_count_{0};
  uint32_t bad_frame_count_{0};
  uint32_t retry_count_{0};
  uint32_t giveup_count_{0};
  int8_t last_rssi_dbm_{0};
  /// Bit per LIST_REQUESTS entry that this boot has asked for at all, and that has
  /// answered at least once. A meter which does not implement a list would
  /// otherwise burn airtime in silence - report_silent_requests_() says so once.
  uint8_t requests_polled_{0};
  uint8_t requests_seen_{0};
  bool warned_silent_requests_{false};
  /// Cycles to allow before deciding a request is not going to be answered -
  /// enough that a marginal link burning its retry budget is not mistaken for a
  /// list the meter does not have.
  static constexpr uint32_t REQUEST_SILENT_WARN_CYCLES = 3;
  /// Bit per request already warned about for answering as the other half; one
  /// line per boot, because it is a finding to report rather than an error.
  uint8_t warned_half_{0};
  /// Bit per FIXED_REQUESTS entry asked for at all, and answered at least once.
  /// Separate from requests_polled_/requests_seen_, which index LIST_REQUESTS.
  uint8_t fixed_polled_{0};
  uint8_t fixed_seen_{0};
  bool warned_fixed_silent_{false};

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
