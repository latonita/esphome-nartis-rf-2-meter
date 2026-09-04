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

void NartisRf2MeterComponent::handle_list_reply_(uint8_t request_idx, const ParsedResponse &resp) {
  const ListRequest &req = LIST_REQUESTS[request_idx];
  const uint8_t list = static_cast<uint8_t>(req.list);
  const bool got_status_half = (resp.shape == PayloadShape::STATUS_HALF);

  if ((req.part == ListPart::STATUS) != got_status_half) {
    // The request and the framing of the reply disagree. The reply is taken at
    // what it decodes as, not at what was asked for - the exact-fit rule already
    // proved that reading consumes DATA - but it is a finding worth one line.
    this->warn_unexpected_half_once_(request_idx, resp);
  }

  // Records go into the merged set whichever half carried them: the leftovers a
  // status half brings are the same list continued, over the same TAG numbering.
  this->merge_records_(resp);
  this->list_arrived_[list] = static_cast<uint8_t>(this->list_arrived_[list] + resp.count);

  if (!got_status_half) {
    // COUNT is the whole list; this frame held what fitted. The status half is
    // what brings the rest, so the two are compared once the cycle is done.
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
  // Both lists end with a block. They should agree - they are built from the same
  // device state - so a difference means one of the two readings is wrong.
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

/* The fixed blocks.
 *
 * Read-and-report for now: there is no TAG in either block for a `tag:` entity to
 * select by, so nothing is published from them. What they are good for today is
 * checking the list decode - the same quantities arrive here on an independent
 * path, with the meter's own raw precision and its own scales - and telling a
 * three-phase meter from a single-phase one, which DI 0xF102 does by length
 * alone.
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
      // parse_response() only ever hands these two data identifiers one of the
      // shapes above, so this is unreachable - but a silent wrong branch here
      // would look like a meter that answers nothing.
      ESP_LOGW(TAG, "DI 0x%04X answered with shape '%s', which is not a fixed block", FIXED_REQUESTS[fixed_idx].di,
               payload_shape_to_string(resp.shape));
      return;
  }
}

void NartisRf2MeterComponent::log_f101_() const {
  nartis_f101 b{};
  std::memcpy(&b, this->f101_raw_, sizeof(b));

  /* Two groups of nine: [total, T1..T8]. Eight tariffs, where the TAG list only
   * reaches four - so if these decode, this block is the only way to see T5..T8.
   *
   * Printed RAW, with no multiplier. Every block in the F10x family carries the
   * firmware's own object units, and those are not the list's: F102 hands out
   * x0.1 W where the list TAGs are x1, and what F101's energy objects are scaled
   * by is not known at all. Applying the list's x0.001 kWh here would be a guess
   * dressed up as a reading, so the raw counts are what goes in the log until a
   * capture can be compared against the meter's own display.
   */
  struct Group {
    const char *name;
    const uint32_t *v;
  };
  const Group groups[2] = {{"0x20", b.group20}, {"0x30", b.group30}};

  for (const Group &g : groups) {
    char line[128];
    size_t at = 0;
    for (uint8_t i = 1; i < 9 && at + 1 < sizeof(line); i++) {
      const int n = std::snprintf(line + at, sizeof(line) - at, "%sT%u=%" PRIu32, (i != 1) ? " " : "", i, g.v[i]);
      if (n <= 0) {
        break;
      }
      at += static_cast<size_t>(n);
    }
    ESP_LOGD(TAG, "F101 energy group %s: total=%" PRIu32, g.name, g.v[0]);
    ESP_LOGD(TAG, "  %s", line);

    // The one arithmetic check the block offers on its own. It only holds if the
    // meter uses every tariff register it reports, so a mismatch is a hint rather
    // than a fault - said at DEBUG for that reason.
    uint32_t sum = 0;
    for (uint8_t i = 1; i < 9; i++) {
      sum += g.v[i];
    }
    if (sum != g.v[0]) {
      ESP_LOGD(TAG, "  total != T1..T8 sum (%" PRIu32 ") - expected unless all eight tariffs are in use", sum);
    }
  }

  ESP_LOGD(TAG, "F101 values above are raw object counts - the scale is not the list's x0.001 kWh");
  ESP_LOGD(TAG, "F101 status block (10 B): %s",
           format_hex_pretty(reinterpret_cast<const uint8_t *>(&b.status), sizeof(b.status)).c_str());
}

void NartisRf2MeterComponent::log_f102_() const {
  if (this->f102_len_ == sizeof(f102_1ph)) {
    f102_1ph b{};
    std::memcpy(&b, this->f102_raw_, sizeof(b));
    ESP_LOGI(TAG, "F102 is the SINGLE-PHASE layout (%u B of DATA, lead 0x%02X)", this->f102_len_, b.lead);
    // Deliberately raw. The firmware pre-scales this variant unevenly - U by 10,
    // I and f by 100, P and Q not at all - so a multiplier printed here would be
    // a guess, and the raw values are what a capture can be checked against.
    ESP_LOGD(TAG, "  P=%" PRId32 " Q=%" PRId32 " U=%" PRIu32 " I=%" PRId32 " f=%" PRIu32, bcd32_signed(&b.p),
             bcd32_signed(&b.q), bcd32_value(&b.u), bcd32_signed(&b.i), bcd32_value(&b.freq));
    return;
  }

  if (this->f102_len_ != sizeof(f102_3ph)) {
    // parse_response() rejects any other length, so this cannot be reached from
    // the air - only by a future caller filling f102_raw_ some other way.
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

  /* Internal arithmetic: the per-phase values must add up to the totals. This is
   * how the field order in this struct was pinned from a single capture, so a
   * failure means the order is wrong - which makes it worth checking every time.
   *
   * The two totals do not get the same tolerance, because they did not behave the
   * same way in the reference capture. Q summed exactly. P was 40 counts light of
   * 14650 - 0.27%, the phases having been sampled a moment apart - so an exact
   * test on P would cry wolf on every live reply.
   */
  const int32_t p_total = bcd32_signed(&b.p_total);
  const int32_t q_total = bcd32_signed(&b.q_total);
  const int32_t p_sum = bcd32_signed(&b.p_l1) + bcd32_signed(&b.p_l2) + bcd32_signed(&b.p_l3);
  const int32_t q_sum = bcd32_signed(&b.q_l1) + bcd32_signed(&b.q_l2) + bcd32_signed(&b.q_l3);
  const int32_t p_err = (p_total > p_sum) ? (p_total - p_sum) : (p_sum - p_total);
  const int32_t p_mag = (p_total < 0) ? -p_total : p_total;
  // 2% plus a floor, so a near-zero load is not judged on its own noise.
  if (p_err > 20 + p_mag / 50) {
    ESP_LOGW(TAG, "  P total %" PRId32 " != phase sum %" PRId32 " - the F102 field order is suspect", p_total, p_sum);
  }
  if (q_total != q_sum) {
    ESP_LOGW(TAG, "  Q total %" PRId32 " != phase sum %" PRId32 " - the F102 field order is suspect", q_total, q_sum);
  }
}

void NartisRf2MeterComponent::cross_check_fixed_() const {
  // --- DI 0xF101's status block against a list's -----------------------------
  //
  // The list halves carry 11 bytes and F101 carries 10. If the extra byte is the
  // leading 0x01 marker, the rest must match byte for byte, both being the same
  // device state read moments apart. That is a claim worth testing rather than
  // assuming, so it is tested here.
  if (this->f101_ok_ && this->status_ok_) {
    nartis_f101 b{};
    std::memcpy(&b, this->f101_raw_, sizeof(b));
    const uint8_t *f101_status = reinterpret_cast<const uint8_t *>(&b.status);
    const bool same = std::memcmp(f101_status, this->status_block_ + 1, sizeof(b.status)) == 0;
    ESP_LOGD(TAG, "F101 status vs the list block after its 0x%02X marker: %s", this->status_block_[0],
             same ? "identical" : "DIFFERENT");
    if (!same) {
      ESP_LOGD(TAG, "  list  %s", format_hex_pretty(this->status_block_, STATUS_BLOCK_SIZE).c_str());
      ESP_LOGD(TAG, "  F101  %s", format_hex_pretty(f101_status, sizeof(b.status)).c_str());
    }
  }

  // --- DI 0xF102 against the list records ------------------------------------
  //
  // Every value in the three-phase block also has a TAG, so where a list carried
  // that TAG the same quantity arrived twice by different paths. The two raw
  // numbers then give the list's multiplier directly:
  //
  //     list_raw * list_scale = fixed_raw * fixed_scale
  //
  // which is the only measurement that settles the open x10 question on current
  // and power - see tags.md. Printed as the implied list scale so it can be read
  // straight off against TAG_TABLE.
  //
  // Note what this rests on: `fixed_scale` below is F102's own scale, which is
  // NOT the list's for any of these quantities and is itself read out of the
  // firmware rather than measured. So the numbers here are only as good as that,
  // and the log says so - what makes them worth having anyway is that the ratio
  // is a clean power of ten if both sources are understood, and something else
  // entirely if either is not.
  if (this->f102_len_ != sizeof(f102_3ph) || this->merged_count_ == 0) {
    return;
  }
  f102_3ph b{};
  std::memcpy(&b, this->f102_raw_, sizeof(b));

  // Struct order, with the TAG that names the same quantity and this block's own
  // scale. This is also the mapping a virtual-TAG fallback would need.
  struct Pair {
    uint8_t tag;
    const bcd32_le *field;
    double fixed_scale;
    const char *unit;
  };
  const Pair pairs[] = {
      {0x20, &b.p_total, 0.1, "W"},   {0x21, &b.p_l1, 0.1, "W"},    {0x22, &b.p_l2, 0.1, "W"},
      {0x23, &b.p_l3, 0.1, "W"},      {0x24, &b.q_total, 0.1, "var"}, {0x25, &b.q_l1, 0.1, "var"},
      {0x26, &b.q_l2, 0.1, "var"},    {0x27, &b.q_l3, 0.1, "var"},  {0x15, &b.u_l1, 0.01, "V"},
      {0x16, &b.u_l2, 0.01, "V"},     {0x17, &b.u_l3, 0.01, "V"},   {0x1D, &b.i_l1, 0.01, "A"},
      {0x1E, &b.i_l2, 0.01, "A"},     {0x1F, &b.i_l3, 0.01, "A"},   {0x28, &b.freq, 0.01, "Hz"},
  };

  bool any = false;
  for (const Pair &p : pairs) {
    const ParsedItem *item = this->find_merged_(p.tag);
    if (item == nullptr) {
      continue;
    }
    uint32_t list_raw = 0;
    if (!item_as_bcd(*item, &list_raw) || list_raw == 0) {
      continue;  // not BCD, or zero - either way no ratio to take
    }
    const uint32_t fixed_raw = bcd32_value(p.field);
    if (!any) {
      ESP_LOGD(TAG, "F102 vs the list records, per TAG. The scale shown is what the list's");
      ESP_LOGD(TAG, "multiplier would have to be for the two to agree, taking F102 as x0.1 W /");
      ESP_LOGD(TAG, "x0.1 var / x0.01 V / x0.01 A / x0.01 Hz - which are F102's own, not the list's:");
      any = true;
    }
    ESP_LOGD(TAG, "  TAG 0x%02X: list %" PRIu32 ", F102 %" PRIu32 " -> list scale x%.4g %s", p.tag, list_raw,
             fixed_raw, p.fixed_scale * static_cast<double>(fixed_raw) / static_cast<double>(list_raw), p.unit);
  }
}

void NartisRf2MeterComponent::merge_records_(const ParsedResponse &resp) {
  for (uint8_t i = 0; i < resp.count && i < MAX_ITEMS; i++) {
    const ParsedItem &item = resp.items[i];
    const ParsedItem *seen = this->find_merged_(item.tag);
    if (seen != nullptr) {
      // The lists overlap, and list B - the superset - is read first, so the
      // records list A repeats are already held and the first copy stands. They
      // have to agree, though: a disagreement means one of the two decodes is
      // wrong, which is worth saying out loud.
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

void NartisRf2MeterComponent::report_silent_fixed_() {
  // Same reasoning as report_silent_requests_(): a meter with no fixed-block
  // handler answers nothing, which for a cycle or two is indistinguishable from a
  // bad link. DI 0xF101 in particular has never been seen answering, so this is
  // the line that will say whether it exists on a given meter.
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
  // the other, which for a cycle or two looks exactly like a bad link - so wait a
  // few cycles, then say it once.
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

  this->f101_ok_ = false;
  this->f102_len_ = 0;

  // Build this cycle's exchange list by walking LIST_REQUESTS in order. That
  // order is load-bearing - a status half has to follow its own records half back
  // to back, or the cursor holding the leftover records is already gone - so the
  // table is walked as it stands and nothing is inserted between a pair.
  //
  // Two gates, and both have to pass. The source gate is the YAML's: a list
  // nobody selected is not asked for at all. The capability gate is what the
  // entities need: a records half is only worth the airtime when something reads
  // a TAG, but a status half is needed either way, since it carries both the
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

  // The fixed blocks go last. They hold no cursor of their own, so nothing about
  // their position matters to them - but a request of any kind drops the lists'
  // cursor, so they must not land between a records half and its status half.
  //
  // Not capability-gated: a fixed block carries no TAGs, so no entity can select
  // from it yet. Asking for it is the point - the log is the product.
  if (this->read_fixed_) {
    for (uint8_t i = 0; i < FIXED_REQUEST_COUNT; i++) {
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

void NartisRf2MeterComponent::finish_exchange_() {
  this->hal_.go_standby();
  this->rx_len_ = 0;

  // Every step in the list is sent. Nothing is conditional on what an earlier
  // reply held: a status half is still worth asking for when its records half
  // delivered the whole list, because the status block comes back either way.
  this->step_idx_++;
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

  this->cross_check_fixed_();

  // A list that announced more records than arrived is the interesting case: the
  // records half sent what fitted and the status half should have brought the
  // rest, so a shortfall means some are unaccounted for.
  for (uint8_t l = 0; l < LIST_COUNT; l++) {
    if (this->list_announced_[l] > this->list_arrived_[l]) {
      ESP_LOGD(TAG, "List %s announced %u record(s) but %u arrived", list_id_to_string(static_cast<ListId>(l)),
               this->list_announced_[l], this->list_arrived_[l]);
    }
  }

  this->report_silent_requests_();
  this->report_silent_fixed_();

  // Which of the four answered, built from LIST_REQUESTS so the line cannot fall
  // out of step with what was actually asked.
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

  // A cycle counts as successful only if every exchange it needed came back. A
  // partial cycle is reported as a failure on purpose: it left some entity holding
  // a stale value, which is the thing this entity exists to expose.
  //
  // Deliberately lenient about which list answered: a meter configured with only
  // one of the two is fully read from that one, so requiring both would peg this
  // entity to false forever. A status half is not required for data either.
  //
  // With no list source selected at all there is nothing to require: entities do
  // go stale, but reporting failure every cycle for a configuration the user
  // chose says nothing they can act on. The fixed blocks are left out for the
  // same reason from the other side - they publish nothing, so their arrival
  // cannot make an entity fresh, and DI 0xF101 not existing on a meter is not a
  // cycle failure.
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

  // Append the interpreted value where we have one, so the log is readable
  // without decoding little-endian bytes by eye.
  char interp[48] = "";
  TagInfo info{};
  // The width check is what keeps the DI 0xF201 status block out of this: it
  // arrives as TAG 0x00 with 9 bytes, where the TAG table has a 4-byte energy
  // register, so there is no sensible scalar reading and none is printed.
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
  if (!tag_info(e.tag, &info, this->tag_width_)) {
    return;  // cannot happen: the parser would have aborted on an unknown TAG
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
      case TagEnc::BCD_LE_SIGNED: {
        int32_t value = 0;
        if (!item_as_bcd_signed(*item, &value)) {
          ESP_LOGW(TAG, "TAG 0x%02X: value is not valid signed BCD: %s", e.tag,
                   format_hex_pretty(item->raw, item->len).c_str());
          break;
        }
        e.sensor->publish_state(static_cast<float>(value));
        ESP_LOGD(TAG, "  -> '%s' = %" PRId32, e.sensor->get_name().c_str(), value);
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
      case TagEnc::BCD_LE_SIGNED: {
        int32_t value = 0;
        if (!item_as_bcd_signed(*item, &value)) {
          ESP_LOGW(TAG, "TAG 0x%02X: value is not valid signed BCD: %s", e.tag,
                   format_hex_pretty(item->raw, item->len).c_str());
          break;
        }
        std::snprintf(buf, sizeof(buf), "%" PRId32, value);
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
