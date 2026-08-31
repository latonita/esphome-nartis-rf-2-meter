#include "d101_frame.h"

#include <cstdio>
#include <cstring>

namespace esphome::nartis_rf_2_meter {

const ParsedItem *ParsedResponse::find(uint8_t tag) const {
  for (uint8_t i = 0; i < this->count && i < MAX_ITEMS; i++) {
    if (this->items[i].tag == tag) {
      return &this->items[i];
    }
  }
  return nullptr;
}

const char *tag_confidence_to_string(TagConfidence c) {
  switch (c) {
    case TagConfidence::OBSERVED:
      return "observed on this link";
    case TagConfidence::DLMS_DERIVED:
      return "width from the meter's DLMS data type";
    case TagConfidence::FAMILY_ASSUMED:
      return "extrapolated from a sibling register";
    default:
      return "unknown";
  }
}

bool tag_info(uint16_t di, uint8_t tag, TagInfo *out, const uint8_t *tag_width_overrides) {
  if (out == nullptr) {
    return false;
  }

  if (di == DI_STATUS) {
    // The status block is a single item: TAG 0x00 carrying 9 opaque bytes.
    if (tag != 0x00) {
      return false;
    }
    *out = TagInfo{STATUS_VALUE_SIZE, TagEnc::STATUS_BLOB, TagConfidence::OBSERVED, 1.0f, ""};
    return true;
  }

  // DI 0xF202 is a larger page over the same TAG numbering, so the same table
  // serves both.
  if (di != DI_ENERGY && di != DI_PARAMS) {
    return false;
  }

  /* Widths and encodings, and where each comes from.
   *
   * A capture of the DI 0xF202 page (15 items) settled the instantaneous values
   * and corrected an earlier guess. The lesson is worth recording: their widths
   * had been derived from the DLMS/COSEM data types the same meter reports for
   * the same OBIS codes, and that was wrong in two ways - currents and power are
   * 2 bytes rather than 4, and every instantaneous value is BCD rather than
   * binary. DLMS carried the rough magnitude across but not the encoding.
   *
   * Only the energy registers are binary little-endian, which is exactly the one
   * family the DLMS mapping had been validated against.
   *
   *   energy registers      4 bytes binary LE, Wh      observed
   *   voltages              4 bytes BCD, x0.1 V        observed (0x15..0x17)
   *   currents              4 bytes BCD, x0.01 A       observed (0x1C..0x1E)
   *   active power          4 bytes BCD, x0.01 kW      observed (0x20)
   *   frequency             4 bytes BCD, x0.01 Hz      observed (0x28)
   *   clock                 7 bytes BCD                observed
   *
   * The instantaneous values are 4 bytes with the upper two always zero, so only
   * four of the eight BCD digits are ever used. Reading them as 2 bytes yields the
   * same number and looks right - the DL/T 645 length field is what gives it away:
   * the captured page reports L = 0x51 = 81, which is 3 + 6*(1+4) + 8*(1+4) +
   * (1+7). With 2-byte values it would have been 65.
   *
   * Everything still marked assumed is extrapolated from an observed sibling of
   * the same kind - the same reasoning that just failed for the DLMS widths, so
   * treat it accordingly. A wrong width desyncs every item after it.
   */
  switch (tag) {
    // --- active energy import, sum and per-tariff ---
    case 0x00:  // 1.0.1.8.0.255
    case 0x01:  // 1.0.1.8.1.255
    case 0x02:  // 1.0.1.8.2.255
    // --- active energy export, sum and tariffs 1-2 (zeroes, but 4 bytes wide) ---
    case 0x05:  // 1.0.2.8.0.255
    case 0x06:  // 1.0.2.8.1.255
    case 0x07:  // 1.0.2.8.2.255
      *out = TagInfo{4, TagEnc::UINT_LE, TagConfidence::OBSERVED, 0.001f, "kWh"};
      return true;
    // --- date and time ---
    case 0x29:  // 0.0.1.0.0.255
      *out = TagInfo{7, TagEnc::BCD_CLOCK, TagConfidence::OBSERVED, 1.0f, ""};
      return true;
    default:
      break;
  }

  // --- voltages: phase-neutral 0x14..0x17, line-to-line 0x18..0x1A.
  //     0x15..0x17 observed; 0x14 (single-phase) and the line voltages are the
  //     same quantity and assumed identical.
  if (tag >= 0x14 && tag <= 0x1A) {
    const bool seen = (tag >= 0x15 && tag <= 0x17);
    *out = TagInfo{4, TagEnc::BCD_LE, seen ? TagConfidence::OBSERVED : TagConfidence::FAMILY_ASSUMED, 0.1f, "V"};
    return true;
  }
  // --- currents: single-phase 0x1B, neutral 0x1C, per-phase 0x1D..0x1F.
  //     0x1C..0x1E observed; 0x1B and 0x1F assumed to match.
  if (tag >= 0x1B && tag <= 0x1F) {
    const bool seen = (tag >= 0x1C && tag <= 0x1E);
    *out = TagInfo{4, TagEnc::BCD_LE, seen ? TagConfidence::OBSERVED : TagConfidence::FAMILY_ASSUMED, 0.01f, "A"};
    return true;
  }
  // --- active power, sum and per-phase. 0x20 observed as 8.40 kW; the per-phase
  //     registers are assumed to match. Note the manual says these are shown with
  //     a sign, and how a sign is carried in BCD here is not known - no negative
  //     value has been captured.
  if (tag >= 0x20 && tag <= 0x23) {
    *out = TagInfo{4, TagEnc::BCD_LE, (tag == 0x20) ? TagConfidence::OBSERVED : TagConfidence::FAMILY_ASSUMED, 0.01f,
                   "kW"};
    return true;
  }
  // --- reactive power, sum and per-phase. Never captured; assumed to mirror
  //     active power.
  if (tag >= 0x24 && tag <= 0x27) {
    *out = TagInfo{4, TagEnc::BCD_LE, TagConfidence::FAMILY_ASSUMED, 0.01f, "kvar"};
    return true;
  }
  // --- mains frequency ---
  if (tag == 0x28) {
    *out = TagInfo{4, TagEnc::BCD_LE, TagConfidence::OBSERVED, 0.01f, "Hz"};
    return true;
  }
  // --- temperature. Never captured. Assumed to follow the other instantaneous
  //     values (4 bytes BCD) rather than the DLMS type, since that mapping is now
  //     known to get the encoding wrong. Sign handling unknown.
  if (tag == 0x2A) {
    *out = TagInfo{4, TagEnc::BCD_LE, TagConfidence::FAMILY_ASSUMED, 0.1f, "\302\260C"};
    return true;
  }
  // --- cumulative registers of the same family as the observed energy ones:
  //     remaining tariffs, reactive energy, and the billing-period mirrors ---
  if ((tag >= 0x03 && tag <= 0x13) || (tag >= 0x2C && tag <= 0x3F)) {
    *out = TagInfo{4, TagEnc::UINT_LE, TagConfidence::FAMILY_ASSUMED, 0.001f, "kWh"};
    return true;
  }

  // Left over: 0x2B (LCD test, a Data object rather than a register). A width
  // declared in YAML fills the gap; consulted only after everything built in.
  if (tag_width_overrides != nullptr && tag < TAG_WIDTH_TABLE_SIZE && tag_width_overrides[tag] != 0) {
    *out = TagInfo{tag_width_overrides[tag], TagEnc::USER, TagConfidence::FAMILY_ASSUMED, 1.0f, ""};
    return true;
  }
  return false;
}

bool tag_needs_params_page(uint8_t tag) {
  // 0x14..0x28 instantaneous, 0x2A temperature, 0x2B LCD test. TAG 0x29 is the
  // clock, which DI 0xF200 carries too.
  return (tag >= 0x14 && tag <= 0x28) || tag == 0x2A || tag == 0x2B;
}

const char *parse_result_to_string(ParseResult r) {
  switch (r) {
    case ParseResult::OK:
      return "OK";
    case ParseResult::ERROR_RESPONSE:
      return "meter refused the read";
    case ParseResult::NO_FRAME:
      return "no length/CRC-consistent frame";
    case ParseResult::BAD_CHECKSUM:
      return "DL/T645 checksum mismatch";
    case ParseResult::MALFORMED:
      return "malformed DL/T645 frame";
    case ParseResult::WRONG_ADDRESS:
      return "address is not our meter";
    case ParseResult::NOT_RESPONSE:
      return "control code is not a read response";
    case ParseResult::UNKNOWN_TAG:
      return "unknown item TAG (width unknown, framing lost)";
    case ParseResult::TOO_MANY_ITEMS:
      return "too many items";
    default:
      return "unknown";
  }
}

const char *dlt645_error_hint(uint8_t err) {
  switch (err) {
    case 0x02:
      return "no such data identifier (observed on this meter for DI 0xF300)";
    case 0x01:
      return "other error (DL/T 645-1997 bit 0, unverified here)";
    case 0x04:
      return "password / authorisation error (DL/T 645-1997 bit 2, unverified here)";
    default:
      return "meaning unknown";
  }
}

uint16_t crc16_x25(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8; bit++) {
      crc = (crc & 1) ? static_cast<uint16_t>((crc >> 1) ^ 0x8408) : static_cast<uint16_t>(crc >> 1);
    }
  }
  return static_cast<uint16_t>(crc ^ 0xFFFF);
}

void serial_to_bcd_le(const char *digits12, uint8_t out[SERIAL_BCD_SIZE]) {
  if (out == nullptr) {
    return;
  }
  std::memset(out, 0, SERIAL_BCD_SIZE);
  if (digits12 == nullptr) {
    return;
  }
  for (size_t i = 0; i < 2 * SERIAL_BCD_SIZE; i++) {
    if (digits12[i] < '0' || digits12[i] > '9') {
      std::memset(out, 0, SERIAL_BCD_SIZE);
      return;  // not 12 digits (or a short string) - refuse to guess
    }
  }
  for (size_t i = 0; i < SERIAL_BCD_SIZE; i++) {
    const uint8_t hi = static_cast<uint8_t>(digits12[2 * i] - '0');
    const uint8_t lo = static_cast<uint8_t>(digits12[2 * i + 1] - '0');
    // BCD pairs are produced MSB-first, then stored reversed.
    out[SERIAL_BCD_SIZE - 1 - i] = static_cast<uint8_t>((hi << 4) | lo);
  }
}

uint8_t channel_from_serial(const char *digits12) {
  uint32_t n3 = 0;
  if (digits12 != nullptr) {
    const size_t len = std::strlen(digits12);
    const size_t start = (len >= 3) ? (len - 3) : 0;
    for (size_t i = start; i < len; i++) {
      const char c = digits12[i];
      if (c >= '0' && c <= '9') {
        n3 = n3 * 10 + static_cast<uint32_t>(c - '0');
      }
    }
  }
  return static_cast<uint8_t>(n3 % CHANNEL_COUNT);
}

uint8_t channel_from_frequency(uint32_t freq_hz) {
  // The grid is not uniform (see channel_frequency()), so invert it by search
  // rather than by division - 24 comparisons, once, at setup.
  uint8_t best = 0;
  uint32_t best_err = UINT32_MAX;
  for (uint8_t k = 0; k < CHANNEL_COUNT; k++) {
    const uint32_t f = channel_frequency(k);
    const uint32_t err = (f > freq_hz) ? (f - freq_hz) : (freq_hz - f);
    if (err < best_err) {
      best_err = err;
      best = k;
    }
  }
  return best;
}

uint32_t channel_frequency(uint8_t channel) {
  const uint8_t k = (channel >= CHANNEL_COUNT) ? static_cast<uint8_t>(CHANNEL_COUNT - 1) : channel;
  return CHANNEL_BASE_HZ + k * CHANNEL_STEP_HZ + (k > CHANNEL_STEP_BREAK ? CHANNEL_STEP_EXTRA_HZ : 0u);
}

uint32_t frequency_from_serial(const char *digits12) {
  return channel_frequency(channel_from_serial(digits12));
}

const uint8_t ENERGY_REQUEST_BODY[6] = {0x00, 0x00, 0x00, 0x00, 0x01, 0x23};
const uint8_t STATUS_REQUEST_BODY[1] = {0x00};
// Variant 2: DI 0x0020 + this body = DATA 20 00 20 0E, which the +0x33 offset
// puts on air as 53 33 53 41 - byte-for-byte the captured request.
const uint8_t V2_REQUEST_BODY[V2_REQUEST_BODY_SIZE] = {0x20, 0x0E};

size_t build_read_request(uint8_t *out, size_t cap, const uint8_t serial_le[SERIAL_BCD_SIZE], uint16_t di,
                          const uint8_t *body, size_t body_len, uint8_t control) {
  if (out == nullptr || serial_le == nullptr || body_len > MAX_REQUEST_BODY) {
    return 0;
  }
  if (body == nullptr && body_len != 0) {
    return 0;
  }
  // The read codes are the only control codes this builder will ever emit. This
  // is what keeps the component read-only by construction, so it is enforced
  // here rather than trusted to the callers.
  if (control != DLT645_C_READ_REQ && control != DLT645_C_READ_REQ_2007) {
    return 0;
  }

  // Application payload before the +0x33 offset: DI little-endian, then the body.
  uint8_t data[2 + MAX_REQUEST_BODY];
  size_t data_len = 0;
  data[data_len++] = static_cast<uint8_t>(di & 0xFF);
  data[data_len++] = static_cast<uint8_t>((di >> 8) & 0xFF);
  for (size_t i = 0; i < body_len; i++) {
    data[data_len++] = body[i];
  }

  const size_t frame645_len = DLT645_OVERHEAD + data_len;
  const size_t env_len = D101_HDR_AFTER_LEN + frame645_len;  // the LEN field
  const size_t total = 2 + 1 + env_len + 2;                  // sync + LEN + content + CRC
  if (env_len > 0xFF || total > cap) {
    return 0;
  }

  size_t p = 0;
  out[p++] = D101_SYNC0;
  out[p++] = D101_SYNC1;
  out[p++] = static_cast<uint8_t>(env_len);
  out[p++] = 0x00;
  out[p++] = 0x01;
  out[p++] = static_cast<uint8_t>(env_len ^ 1);  // HLEN - see the header note

  const size_t f645 = p;
  out[p++] = DLT645_DELIM;
  std::memcpy(out + p, serial_le, SERIAL_BCD_SIZE);
  p += SERIAL_BCD_SIZE;
  out[p++] = DLT645_DELIM;
  // Checked above: this builder emits reads and nothing else.
  out[p++] = control;
  out[p++] = static_cast<uint8_t>(data_len);
  for (size_t i = 0; i < data_len; i++) {
    out[p++] = static_cast<uint8_t>(data[i] + DLT645_DATA_OFFSET);
  }

  // CS: plain 8-bit sum from the first 0x68 through the last DATA byte.
  uint8_t cs = 0;
  for (size_t i = f645; i < p; i++) {
    cs = static_cast<uint8_t>(cs + out[i]);
  }
  out[p++] = cs;
  out[p++] = DLT645_END;

  // CRC range starts at the LEN byte and ends just before the CRC itself.
  const uint16_t crc = crc16_x25(out + 2, env_len + 1);
  out[p++] = static_cast<uint8_t>(crc & 0xFF);
  out[p++] = static_cast<uint8_t>((crc >> 8) & 0xFF);

  return p;
}

size_t build_request(uint8_t *out, size_t cap, const uint8_t serial_le[SERIAL_BCD_SIZE], uint16_t di) {
  // The two polls the display itself sends, with their captured bodies. Both are
  // constant; the meter answers with whatever its indication set holds.
  // DI 0xF202 takes the same 6-byte body as DI 0xF200. With the 1-byte body it is
  // silently dropped - not even an error response.
  if (di == DI_ENERGY || di == DI_PARAMS) {
    return build_read_request(out, cap, serial_le, di, ENERGY_REQUEST_BODY, sizeof(ENERGY_REQUEST_BODY));
  }
  if (di == DI_STATUS) {
    return build_read_request(out, cap, serial_le, di, STATUS_REQUEST_BODY, sizeof(STATUS_REQUEST_BODY));
  }
  return 0;
}

size_t build_v2_request(uint8_t *out, size_t cap, const uint8_t serial_le[SERIAL_BCD_SIZE]) {
  return build_read_request(out, cap, serial_le, V2_REQUEST_DI, V2_REQUEST_BODY, V2_REQUEST_BODY_SIZE,
                            DLT645_C_READ_REQ_2007);
}

/// Smallest LEN that could hold a 645 frame with an empty DATA field.
static constexpr size_t MIN_ENV_LEN = D101_HDR_AFTER_LEN + DLT645_OVERHEAD;
/// How far into the buffer to look for the LEN byte. The radio normally hands
/// back a buffer starting exactly at LEN; a couple of slipped bytes at the head
/// of a weak capture are common enough to be worth scanning for.
static constexpr size_t MAX_START_SCAN = 3;

ParseResult parse_response(const uint8_t *buf, size_t len, const uint8_t serial_le[SERIAL_BCD_SIZE],
                           ParsedResponse *out, const uint8_t *tag_width_overrides) {
  if (buf == nullptr || serial_le == nullptr || out == nullptr) {
    return ParseResult::NO_FRAME;
  }

  for (size_t lp = 0; lp <= MAX_START_SCAN && lp < len; lp++) {
    const size_t env_len = buf[lp];
    if (env_len < MIN_ENV_LEN || lp + env_len + 3 > len) {
      continue;
    }
    // CRC over [LEN .. last content byte], trailer is little-endian.
    const uint16_t calc = crc16_x25(buf + lp, env_len + 1);
    const uint16_t got =
        static_cast<uint16_t>(buf[lp + env_len + 1]) | static_cast<uint16_t>(buf[lp + env_len + 2] << 8);
    if (calc != got) {
      continue;
    }

    // Envelope is good. Header byte 5 (HLEN) is deliberately not checked.
    const uint8_t *f = buf + lp + 1 + D101_HDR_AFTER_LEN;  // start of the 645 frame
    const size_t f_len = env_len - D101_HDR_AFTER_LEN;

    if (f[0] != DLT645_DELIM || f[7] != DLT645_DELIM) {
      return ParseResult::MALFORMED;
    }
    if (std::memcmp(f + 1, serial_le, SERIAL_BCD_SIZE) != 0) {
      return ParseResult::WRONG_ADDRESS;  // a neighbour's meter, or a stray frame
    }
    const size_t data_len = f[9];
    if (DLT645_OVERHEAD + data_len != f_len) {
      return ParseResult::MALFORMED;
    }
    if (f[11 + data_len] != DLT645_END) {
      return ParseResult::MALFORMED;
    }
    uint8_t cs = 0;
    for (size_t i = 0; i < 10 + data_len; i++) {
      cs = static_cast<uint8_t>(cs + f[i]);
    }
    if (cs != f[10 + data_len]) {
      return ParseResult::BAD_CHECKSUM;
    }

    // Strip the +0x33 transmission offset. A run of 0x33 bytes is just zeroes -
    // this is what makes the raw payload look deceptively BCD-ish. Kept in `out`
    // so the caller can log it even when item decoding fails below.
    *out = ParsedResponse{};
    out->control = f[8];
    out->payload_len = static_cast<uint8_t>((data_len < MAX_PAYLOAD) ? data_len : MAX_PAYLOAD);
    for (size_t i = 0; i < out->payload_len; i++) {
      out->payload[i] = static_cast<uint8_t>(f[10 + i] - DLT645_DATA_OFFSET);
    }
    const uint8_t *payload = out->payload;
    if (data_len > MAX_PAYLOAD) {
      return ParseResult::MALFORMED;  // longer than anything this protocol should send
    }

    // Control code. Two read responses are accepted: 0x81 (DL/T 645-1997,
    // variant 1) and 0x91 (DL/T 645-2007, variant 2). They differ only in the DI
    // width that precedes COUNT - 2 bytes for 1997, 4 for 2007 - after which both
    // carry the same COUNT + {TAG, value} item list. Bit 6 set means the meter
    // refused; DATA then holds error bytes, not items, so stop here with the
    // payload kept for the caller to log.
    const bool is_2007 = (f[8] == DLT645_C_READ_RSP_2007);
    const bool is_1997 = (f[8] == DLT645_C_READ_RSP);
    if (!is_1997 && !is_2007) {
      return ((f[8] & 0x40) != 0) ? ParseResult::ERROR_RESPONSE : ParseResult::NOT_RESPONSE;
    }

    const size_t di_bytes = is_2007 ? 4 : 2;
    if (data_len < di_bytes + 1) {
      return ParseResult::MALFORMED;  // need at least DI + COUNT
    }
    uint32_t di_full = 0;
    for (size_t i = 0; i < di_bytes; i++) {
      di_full |= static_cast<uint32_t>(payload[i]) << (8 * i);
    }
    out->di32 = di_full;
    out->di = static_cast<uint16_t>(di_full & 0xFFFF);

    // A 2007 display block reuses the 1997 params-page TAG numbering (block 0xF2
    // is that page re-wrapped), so decode its items with the DI 0xF202 table
    // regardless of which block index came back.
    const uint16_t lookup_di = is_2007 ? DI_PARAMS : out->di;

    const uint8_t count = payload[di_bytes];
    if (count > MAX_ITEMS) {
      return ParseResult::TOO_MANY_ITEMS;
    }

    size_t pos = di_bytes + 1;
    for (uint8_t i = 0; i < count; i++) {
      if (pos >= data_len) {
        return ParseResult::MALFORMED;
      }
      const uint8_t tag = payload[pos++];
      TagInfo info{};
      if (!tag_info(lookup_di, tag, &info, tag_width_overrides)) {
        // No width => no way to find the next item, so everything after this
        // point is unparseable. Abort and hand the caller enough context to work
        // the width out by hand.
        out->count = i;
        out->unknown_tag = tag;
        out->unknown_offset = static_cast<uint8_t>(pos - 1);
        return ParseResult::UNKNOWN_TAG;
      }
      if (pos + info.width > data_len) {
        return ParseResult::MALFORMED;
      }
      out->items[i].tag = tag;
      out->items[i].len = info.width;
      std::memcpy(out->items[i].raw, payload + pos, info.width);
      pos += info.width;
    }
    out->count = count;
    return ParseResult::OK;
  }

  return ParseResult::NO_FRAME;
}

int32_t item_as_i32(const ParsedItem &item) {
  if (item.len == 0 || item.len > 4) {
    return 0;
  }
  uint32_t v = item_as_u32(item);
  // Sign-extend from the item's own width.
  const uint8_t bits = static_cast<uint8_t>(8 * item.len);
  if (bits < 32 && (v & (1u << (bits - 1))) != 0) {
    v |= ~((1u << bits) - 1u);
  }
  return static_cast<int32_t>(v);
}

uint32_t item_as_u32(const ParsedItem &item) {
  if (item.len == 0 || item.len > 4) {
    return 0;
  }
  uint32_t v = 0;
  for (uint8_t i = 0; i < item.len; i++) {
    v |= static_cast<uint32_t>(item.raw[i]) << (8 * i);
  }
  return v;
}

bool item_as_bcd(const ParsedItem &item, uint32_t *out) {
  if (out == nullptr || item.len == 0 || item.len > 4) {
    return false;
  }
  uint32_t v = 0;
  // Least-significant BCD pair first, so walk the bytes backwards.
  for (uint8_t i = item.len; i > 0; i--) {
    const uint8_t b = item.raw[i - 1];
    const uint8_t hi = static_cast<uint8_t>(b >> 4);
    const uint8_t lo = static_cast<uint8_t>(b & 0x0F);
    if (hi > 9 || lo > 9) {
      return false;
    }
    v = v * 100u + hi * 10u + lo;
  }
  *out = v;
  return true;
}

bool item_clock_to_string(const ParsedItem &item, char *out, size_t cap) {
  if (out == nullptr || cap < 20 || item.len != 7) {
    return false;
  }
  uint8_t dec[7];
  for (uint8_t i = 0; i < 7; i++) {
    const uint8_t hi = static_cast<uint8_t>(item.raw[i] >> 4);
    const uint8_t lo = static_cast<uint8_t>(item.raw[i] & 0x0F);
    if (hi > 9 || lo > 9) {
      return false;  // not BCD - the decode is wrong somewhere
    }
    dec[i] = static_cast<uint8_t>(hi * 10 + lo);
  }
  // raw layout: ss mm hh dow DD MM YY  (dow: 1 = Monday .. 7 = Sunday)
  const uint8_t sec = dec[0], minute = dec[1], hour = dec[2];
  const uint8_t day = dec[4], month = dec[5], year = dec[6];
  if (hour > 23 || minute > 59 || sec > 59 || day < 1 || day > 31 || month < 1 || month > 12) {
    return false;
  }
  std::snprintf(out, cap, "20%02u-%02u-%02u %02u:%02u:%02u", year, month, day, hour, minute, sec);
  return true;
}

}  // namespace esphome::nartis_rf_2_meter
