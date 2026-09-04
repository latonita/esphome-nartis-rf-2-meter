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

namespace {

/* The TAG table.
 *
 * A record is TAG followed by its value with no length field, so the width here
 * is what makes the rest of a page readable at all - get one wrong and every
 * record after it is garbage. The rows are contiguous TAG ranges because the
 * meter groups its registers that way: one row per family, ascending.
 *
 * `scale` and `unit` are for the log line only. Published values stay raw so the
 * YAML decides the unit with a `multiply` filter, which is why a scale being
 * uncertain is not a correctness problem here.
 *
 * One row is worth reading twice before trusting: 0x20..0x27, the power group.
 * The vendor table's multiplier puts these in units of 10 W / 10 var. On one
 * meter a cross-check against the raw objects (P ~ U*I*PF, and P against the
 * energy rate) fitted 1 W instead, so builds or CT variants differ by a factor of
 * ten. Check the per-phase P against P_total on your own meter before believing
 * the absolute value - or read the same quantity from DI 0xF102, whose scales are
 * independent, and take the ratio.
 */
struct TagRange {
  uint8_t first;
  uint8_t last;
  uint8_t width;
  TagEnc enc;
  float scale;
  const char *unit;
};

constexpr TagRange TAG_TABLE[] = {
    // Energy accumulators. Binary little-endian - the one family that is not BCD.
    // 0x00..0x07 are active import/export; the rest of the range is reactive and
    // other cumulative registers, so the log unit covers both.
    {0x00, 0x09, 4, TagEnc::UINT_LE, 0.001f, "kWh"},
    {0x0A, 0x13, 4, TagEnc::UINT_LE, 0.001f, "kvarh"},

    // Voltages: 0x14 single-phase, 0x15..0x17 per phase, 0x18..0x1A line-to-line.
    {0x14, 0x14, 4, TagEnc::BCD_LE, 0.1f, "V"},
    {0x15, 0x17, 4, TagEnc::BCD_LE, 0.1f, "V"},
    {0x18, 0x1A, 4, TagEnc::BCD_LE, 0.1f, "V"},

    // Currents: 0x1B single-phase, 0x1C neutral, 0x1D..0x1F per phase.
    {0x1B, 0x1F, 4, TagEnc::BCD_LE, 0.001f, "A"},

    // Power, total then per phase. Reactive is the only signed BCD on the wire.
    {0x20, 0x23, 4, TagEnc::BCD_LE_SIGNED, 1.0f, "W"},
    {0x24, 0x27, 4, TagEnc::BCD_LE_SIGNED, 1.0f, "var"},

    {0x28, 0x28, 4, TagEnc::BCD_LE, 0.01f, "Hz"},
    {0x29, 0x29, 7, TagEnc::BCD_CLOCK, 1.0f, ""},
    // Temperature is binary two's-complement and 2 bytes wide, not the 4-byte BCD
    // its neighbours use - the one row in this table that breaks the pattern.
    {0x2A, 0x2A, 2, TagEnc::INT_LE, 0.1f, "\302\260C"},

    // 0x2B skipped, special use-case for D101-2 LCD test
    {0x2C, 0x35, 4, TagEnc::BCD_LE, 0.001f, "kWh"}, // active energy import (sum + by 4 tariffs), then export, end of last period
    {0x36, 0x3F, 4, TagEnc::BCD_LE, 0.001f, "kvarh"}, // reactive energy import  (sum + by 4 tariffs), then export end of last period
    // 0x48..0x4F are identity and configuration objects whose widths vary per
    // object, so there is no single width to walk them by. A YAML `bytes:` is the
    // only way to read one.
};

}  // namespace

bool tag_info(uint8_t tag, TagInfo *out, const uint8_t *tag_width_overrides) {
  if (out == nullptr) {
    return false;
  }
  for (const TagRange &r : TAG_TABLE) {
    if (tag >= r.first && tag <= r.last) {
      *out = TagInfo{r.width, r.enc, r.scale, r.unit};
      return true;
    }
  }
  // Not in the table: 0x2B, the identity objects at 0x48..0x4F, or a TAG this
  // meter's indication set has that the vendor table does not. A width declared
  // in YAML fills the gap - consulted last, so it can never shadow a row above.
  if (tag_width_overrides != nullptr && tag < TAG_WIDTH_TABLE_SIZE && tag_width_overrides[tag] != 0) {
    *out = TagInfo{tag_width_overrides[tag], TagEnc::USER, 1.0f, ""};
    return true;
  }
  return false;
}

const char *payload_shape_to_string(PayloadShape s) {
  switch (s) {
    case PayloadShape::RECORDS:
      return "records half: DI, COUNT, records";
    case PayloadShape::STATUS_HALF:
      return "status half: DI, leftover records, status block";
    case PayloadShape::FIXED_F101:
      return "fixed F101: DI, 2 energy groups of 9, status block";
    case PayloadShape::FIXED_F102_3PH:
      return "fixed F102 three-phase: DI, marker, 15 BCD values";
    case PayloadShape::FIXED_F102_1PH:
      return "fixed F102 single-phase: DI, lead, 5 BCD values";
    default:
      return "unknown";
  }
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

const uint8_t REQUEST_BODY_LONG[6] = {0x00, 0x00, 0x00, 0x00, 0x01, 0x23};
const uint8_t REQUEST_BODY_SHORT[1] = {0x00};

const ListRequest LIST_REQUESTS[LIST_REQUEST_COUNT] = {
    {DI_LIST_B_RECORDS, ListId::B, ListPart::RECORDS, REQUEST_BODY_LONG, sizeof(REQUEST_BODY_LONG)},
    {DI_LIST_B_STATUS, ListId::B, ListPart::STATUS, REQUEST_BODY_LONG, sizeof(REQUEST_BODY_LONG)},
    {DI_LIST_A_RECORDS, ListId::A, ListPart::RECORDS, REQUEST_BODY_LONG, sizeof(REQUEST_BODY_LONG)},
    // The one short body. Sending the long one here gets no reply at all - not
    // even an error response - so the pairing matters.
    {DI_LIST_A_STATUS, ListId::A, ListPart::STATUS, REQUEST_BODY_SHORT, sizeof(REQUEST_BODY_SHORT)},
};

// Both fixed reads take the long body, the one DI 0xF102 was captured answering.
// DI 0xF101 has not been seen on air at all; the long body is the better guess of
// the two, being what three of the four list requests use.
const FixedRequest FIXED_REQUESTS[FIXED_REQUEST_COUNT] = {
    {DI_FIXED_F101, REQUEST_BODY_LONG, sizeof(REQUEST_BODY_LONG)},
    {DI_FIXED_F102, REQUEST_BODY_LONG, sizeof(REQUEST_BODY_LONG)},
};

uint8_t fixed_request_index(uint16_t di) {
  for (uint8_t i = 0; i < FIXED_REQUEST_COUNT; i++) {
    if (FIXED_REQUESTS[i].di == di) {
      return i;
    }
  }
  return FIXED_REQUEST_COUNT;
}

const char *list_id_to_string(ListId l) { return (l == ListId::A) ? "A" : "B"; }

uint8_t list_request_index(uint16_t di) {
  for (uint8_t i = 0; i < LIST_REQUEST_COUNT; i++) {
    if (LIST_REQUESTS[i].di == di) {
      return i;
    }
  }
  return LIST_REQUEST_COUNT;
}

size_t build_read_request(uint8_t *out, size_t cap, const uint8_t serial_le[SERIAL_BCD_SIZE], uint16_t di,
                          const uint8_t *body, size_t body_len) {
  if (out == nullptr || serial_le == nullptr || body_len > MAX_REQUEST_BODY) {
    return 0;
  }
  if (body == nullptr && body_len != 0) {
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
  // Hard-wired: this builder emits reads and nothing else.
  out[p++] = DLT645_C_READ_REQ;
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
  const uint8_t req = list_request_index(di);
  if (req < LIST_REQUEST_COUNT) {
    return build_read_request(out, cap, serial_le, di, LIST_REQUESTS[req].body, LIST_REQUESTS[req].body_len);
  }
  const uint8_t fixed = fixed_request_index(di);
  if (fixed < FIXED_REQUEST_COUNT) {
    return build_read_request(out, cap, serial_le, di, FIXED_REQUESTS[fixed].body, FIXED_REQUESTS[fixed].body_len);
  }
  return 0;  // body not known for this DI; a probe has to supply its own
}

/// Smallest LEN that could hold a 645 frame with an empty DATA field.
static constexpr size_t MIN_ENV_LEN = D101_HDR_AFTER_LEN + DLT645_OVERHEAD;
/// How far into the buffer to look for the LEN byte. The radio normally hands
/// back a buffer starting exactly at LEN; a couple of slipped bytes at the head
/// of a weak capture are common enough to be worth scanning for.
static constexpr size_t MAX_START_SCAN = 3;

namespace {

/// Outcome of walking a {TAG, value} record stream.
struct ItemWalk {
  uint8_t count{0};                       ///< records written to `out`
  size_t end{0};                          ///< payload offset just past the last one
  ParseResult result{ParseResult::OK};    ///< why the walk stopped, if it stopped early
  uint8_t unknown_tag{0};                 ///< valid when result == UNKNOWN_TAG
  uint8_t unknown_offset{0};
};

/// Decode records from `payload[start .. end)`, at most `max_records` of them,
/// into `out` (which must hold MAX_ITEMS entries).
///
/// `end` is where the records stop, which is not always the end of DATA: on a
/// status half the last STATUS_BLOCK_SIZE bytes are the block, not a record.
///
/// Records carry no length field, so a TAG of unknown width ends the walk - there
/// is no way to find where the next record starts. `count` and `end` then describe
/// how far it got, which is what the caller needs in order to log the rest.
ItemWalk walk_items(const uint8_t *payload, size_t end, size_t start, size_t max_records, ParsedItem *out,
                    const uint8_t *tag_width_overrides) {
  ItemWalk w{};
  w.end = start;
  size_t pos = start;
  while (w.count < max_records && pos < end) {
    if (w.count >= MAX_ITEMS) {
      w.result = ParseResult::TOO_MANY_ITEMS;
      return w;
    }
    const uint8_t tag = payload[pos];
    TagInfo info{};
    if (!tag_info(tag, &info, tag_width_overrides)) {
      w.result = ParseResult::UNKNOWN_TAG;
      w.unknown_tag = tag;
      w.unknown_offset = static_cast<uint8_t>(pos);
      return w;
    }
    if (pos + 1 + info.width > end) {
      // Cut mid-record. Unlike stopping short by whole records this is not
      // something a meter does deliberately, so it is a framing error.
      w.result = ParseResult::MALFORMED;
      return w;
    }
    out[w.count].tag = tag;
    out[w.count].len = info.width;
    std::memcpy(out[w.count].raw, payload + pos + 1, info.width);
    w.count++;
    pos += 1 + info.width;
    w.end = pos;
  }
  return w;
}

}  // namespace

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

    // Control code. Bit 7 marks the reply direction; bit 6 means the meter
    // refused. On a refusal DATA carries error bytes, not items, so stop here
    // with the payload kept for the caller to log.
    if (f[8] != DLT645_C_READ_RSP) {
      return ((f[8] & 0x80) != 0) ? ParseResult::ERROR_RESPONSE : ParseResult::NOT_RESPONSE;
    }

    if (data_len < 2) {
      return ParseResult::MALFORMED;  // need at least the echoed DI
    }
    out->di = static_cast<uint16_t>(payload[0]) | static_cast<uint16_t>(payload[1] << 8);

    /* A fixed block is neither framing: no COUNT, no TAGs, and a length that is
     * part of the layout rather than a consequence of what fitted. So it is
     * settled here, before the record framings are considered at all.
     *
     * DI narrows it to F101 or F102 and the length finishes the job - which is
     * also the whole 3-phase/1-phase test for F102, there being nothing else in
     * the reply that says which meter sent it. A length that matches neither is
     * MALFORMED with `payload` kept, so the caller can dump the bytes: that is
     * the case worth seeing, since it means a third layout exists.
     */
    if (out->di == DI_FIXED_F101 || out->di == DI_FIXED_F102) {
      if (out->di == DI_FIXED_F101 && data_len == sizeof(nartis_f101)) {
        out->shape = PayloadShape::FIXED_F101;
      } else if (out->di == DI_FIXED_F102 && data_len == sizeof(f102_3ph)) {
        out->shape = PayloadShape::FIXED_F102_3PH;
      } else if (out->di == DI_FIXED_F102 && data_len == sizeof(f102_1ph)) {
        out->shape = PayloadShape::FIXED_F102_1PH;
      } else {
        return ParseResult::MALFORMED;
      }
      // No records and no COUNT byte. The values live in `payload`, to be read
      // through the structs in nartis_dlt645_f1xx.h.
      out->count = 0;
      out->announced_count = 0;
      return ParseResult::OK;
    }

    /* Which of the two framings is this?
     *
     * The request settles it - a records half answers with records, a status half
     * with leftover records and then the block - but the reading is verified, not
     * assumed: a page stops on a record boundary, so the right reading is the one
     * that consumes DATA exactly. The expected framing is tried first and the
     * other kept as a fallback, which is what lets a meter that answers a request
     * the other way round be identified rather than read as garbage. A probe's
     * data identifier is in neither half and is read as records.
     */
    const uint8_t req = list_request_index(out->di);
    const bool expect_status = (req < LIST_REQUEST_COUNT) && (LIST_REQUESTS[req].part == ListPart::STATUS);

    // Records end here under each framing; walk_items() stops there and the fit
    // is judged against it.
    const auto records_end = [&](PayloadShape shape) -> size_t {
      return (shape == PayloadShape::STATUS_HALF) ? data_len - STATUS_BLOCK_SIZE : data_len;
    };

    const auto try_shape = [&](PayloadShape shape) -> ItemWalk {
      if (shape == PayloadShape::STATUS_HALF) {
        // DI(2) plus the block is the shortest a status half can be, and that is
        // the shape of a reply with nothing left over to continue.
        if (data_len < 2 + STATUS_BLOCK_SIZE) {
          return ItemWalk{0, 0, ParseResult::MALFORMED, 0, 0};
        }
        // No COUNT byte, so the byte budget is the only bound; a record is at
        // least 2 bytes, so this never truncates a real list.
        const size_t end = records_end(shape);
        return walk_items(payload, end, 2, end, out->items, tag_width_overrides);
      }
      if (data_len < 3) {
        return ItemWalk{0, 0, ParseResult::MALFORMED, 0, 0};  // no room for a COUNT byte
      }
      return walk_items(payload, data_len, 3, payload[2], out->items, tag_width_overrides);
    };

    PayloadShape candidates[2];
    if (expect_status) {
      candidates[0] = PayloadShape::STATUS_HALF;
      candidates[1] = PayloadShape::RECORDS;
    } else {
      candidates[0] = PayloadShape::RECORDS;
      candidates[1] = PayloadShape::STATUS_HALF;
    }

    ItemWalk walk{};
    PayloadShape shape = candidates[0];
    bool exact = false;
    for (const PayloadShape candidate : candidates) {
      walk = try_shape(candidate);
      if (walk.result == ParseResult::OK && walk.end == records_end(candidate)) {
        shape = candidate;
        exact = true;
        break;
      }
    }
    if (!exact) {
      // Neither reading landed on a boundary. Report the expected one's own
      // failure - its diagnostics are what a reader can act on - and re-run it so
      // out->items holds whatever it did decode.
      shape = candidates[0];
      walk = try_shape(shape);
    }

    out->count = walk.count;
    out->shape = shape;
    out->announced_count = (shape == PayloadShape::RECORDS && data_len >= 3) ? payload[2] : 0;
    if (shape == PayloadShape::STATUS_HALF && data_len >= 2 + STATUS_BLOCK_SIZE) {
      std::memcpy(out->status_block, payload + data_len - STATUS_BLOCK_SIZE, STATUS_BLOCK_SIZE);
      out->has_status_block = true;
    }
    if (walk.result == ParseResult::UNKNOWN_TAG) {
      out->unknown_tag = walk.unknown_tag;
      out->unknown_offset = walk.unknown_offset;
    }
    if (walk.result != ParseResult::OK) {
      return walk.result;
    }
    if (!exact) {
      // Every record read cleanly, yet bytes are left over: DATA holds a whole
      // record that no reading accounted for, so a width above must be wrong.
      return ParseResult::MALFORMED;
    }
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

bool item_as_bcd_signed(const ParsedItem &item, int32_t *out) {
  if (out == nullptr || item.len == 0 || item.len > 4) {
    return false;
  }
  const bool negative = (item.raw[item.len - 1] & BCD_SIGN_BIT) != 0;
  int32_t v = 0;
  // Least-significant BCD pair first, so walk the bytes backwards; the sign bit
  // is masked off the first byte read, which is the most significant one.
  for (uint8_t i = item.len; i > 0; i--) {
    uint8_t b = item.raw[i - 1];
    if (i == item.len) {
      b = static_cast<uint8_t>(b & ~BCD_SIGN_BIT);
    }
    const uint8_t hi = static_cast<uint8_t>(b >> 4);
    const uint8_t lo = static_cast<uint8_t>(b & 0x0F);
    if (hi > 9 || lo > 9) {
      return false;
    }
    v = v * 100 + hi * 10 + lo;
  }
  *out = negative ? -v : v;
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
