// Decode check for the DI 0xF202 parameter page, from a real capture.
//
// The on-air capture was truncated by an undersized RX buffer, so the frame below is
// that page reconstructed from its records. Not circular: the reconstruction still has
// to reproduce the L = 0x51 and LEN = 0x60 the truncated capture did carry, and those
// only come out right if every item width is right.
#include "d101_frame.h"

#include <cstdio>
#include <cstring>
#include <string>

using namespace esphome::nartis_rf_2_meter;

static int fail = 0;

static void check(bool ok, const char *what) {
  std::printf("%s  %s\n", ok ? "PASS" : "FAIL", what);
  if (!ok)
    fail++;
}

// Captured page, from the LEN byte onwards, as the radio hands it over.
static const char *const SHORT_PAGE_HEX =
    "620001636896552740320268815335254B3348523B34347CBEEA3335FFC68333383333333339333333333A333333334898"
    "543333499C5433334ACB5433334FA77C3333504887333351756C333352BA6A333353375B333354843B333355B43C33339D"
    "16BCB3";

static const char *const FRAME_HEX = "6000016168601027403202688151352542335CFDFF33349099C03335FF967233383333333339333333333A333333334864563333494A5633334A695633334FA74A333350B9453333514A44333353C83A33335BCC7C33335C683C47345A3A59C6161E55";

static size_t unhex(const char *hex, uint8_t *out, size_t cap) {
  size_t n = 0;
  for (const char *p = hex; *p && p[1] && n < cap; p += 2) {
    unsigned b = 0;
    std::sscanf(p, "%2x", &b);
    out[n++] = (uint8_t) b;
  }
  return n;
}

/// Wrap `data` (the de-offset application payload) in a DL/T 645 read response and a
/// radio envelope, exactly as parse_response() expects it: from the LEN byte onwards.
/// Lives in the test - the component has no business building the meter's frames.
static size_t build_response(uint8_t *out, size_t cap, const uint8_t serial[SERIAL_BCD_SIZE], const uint8_t *data,
                             size_t data_len) {
  const size_t env_len = D101_HDR_AFTER_LEN + DLT645_OVERHEAD + data_len;
  if (out == nullptr || cap < env_len + 3) {
    return 0;
  }
  size_t p = 0;
  out[p++] = (uint8_t) env_len;
  out[p++] = 0x00;
  out[p++] = 0x01;
  out[p++] = (uint8_t) (env_len ^ 1);  // HLEN

  const size_t f645 = p;
  out[p++] = DLT645_DELIM;
  std::memcpy(out + p, serial, SERIAL_BCD_SIZE);
  p += SERIAL_BCD_SIZE;
  out[p++] = DLT645_DELIM;
  out[p++] = DLT645_C_READ_RSP;
  out[p++] = (uint8_t) data_len;
  for (size_t i = 0; i < data_len; i++) {
    out[p++] = (uint8_t) (data[i] + DLT645_DATA_OFFSET);
  }
  uint8_t cs = 0;
  for (size_t i = f645; i < p; i++) {
    cs = (uint8_t) (cs + out[i]);
  }
  out[p++] = cs;
  out[p++] = DLT645_END;

  const uint16_t crc = crc16_x25(out, env_len + 1);
  out[p++] = (uint8_t) (crc & 0xFF);
  out[p++] = (uint8_t) ((crc >> 8) & 0xFF);
  return p;
}


// The status block every status half ends with, captured from meter ...5596; byte 0 is
// the constant 0x01 marker.
static const uint8_t STATUS_BLOCK[STATUS_BLOCK_SIZE] = {0x01, 0x00, 0x22, 0x84, 0x00, 0x08,
                                                        0x01, 0x1F, 0x00, 0x03, 0x01};

static std::string format_block(const uint8_t *b) {
  char out[STATUS_BLOCK_SIZE * 3];
  size_t at = 0;
  for (size_t i = 0; i < STATUS_BLOCK_SIZE; i++) {
    at += static_cast<size_t>(std::snprintf(out + at, sizeof(out) - at, "%s%02X", (i != 0) ? " " : "", b[i]));
  }
  return std::string(out);
}

struct Expect {
  uint8_t tag;
  uint8_t width;
  TagEnc enc;
  uint32_t raw;
  const char *shown;
};

int main() {
  uint8_t serial[SERIAL_BCD_SIZE];
  serial_to_bcd_le("023240271060", serial);

  uint8_t frame[256];
  const size_t len = unhex(FRAME_HEX, frame, sizeof(frame));
  std::printf("frame %zu bytes, LEN=0x%02X, HLEN=0x%02X\n\n", len, frame[0], frame[3]);
  check(len == 99, "frame is 99 bytes from the LEN byte");
  check(frame[0] == 0x60, "LEN = 0x60 as captured");
  check(frame[3] == 0x61, "HLEN = 0x61 as captured (LEN ^ 1)");

  ParsedResponse r{};
  const ParseResult pr = parse_response(frame, len, serial, &r);
  std::printf("parse: %s, DI 0x%04X, count %u, payload %u B\n", parse_result_to_string(pr), r.di, r.count,
              r.payload_len);
  check(pr == ParseResult::OK, "page parses");
  check(r.di == DI_LIST_B_RECORDS, "DI is 0xF202");
  check(r.count == 15, "15 records");
  check(r.payload_len == 0x51, "DATA length is 0x51 = 81, as captured");

  static const Expect want[] = {
      {0x00, 4, TagEnc::UINT_LE, 13421097, "13421.097 kWh"},
      {0x01, 4, TagEnc::UINT_LE, 9266781, "9266.781 kWh"},
      {0x02, 4, TagEnc::UINT_LE, 4154316, "4154.316 kWh"},
      {0x05, 4, TagEnc::UINT_LE, 0, "0 (A- total)"},
      {0x06, 4, TagEnc::UINT_LE, 0, "0"},
      {0x07, 4, TagEnc::UINT_LE, 0, "0"},
      {0x15, 4, TagEnc::BCD_LE, 2331, "233.1 V"},
      {0x16, 4, TagEnc::BCD_LE, 2317, "231.7 V"},
      {0x17, 4, TagEnc::BCD_LE, 2336, "233.6 V"},
      {0x1C, 4, TagEnc::BCD_LE, 1774, "17.74 A neutral"},
      {0x1D, 4, TagEnc::BCD_LE, 1286, "12.86 A"},
      {0x1E, 4, TagEnc::BCD_LE, 1117, "11.17 A"},
      {0x20, 4, TagEnc::BCD_LE_SIGNED, 795, "7.95 kW"},
      {0x28, 4, TagEnc::BCD_LE, 4999, "49.99 Hz"},
  };

  std::printf("\n");
  for (const Expect &e : want) {
    const ParsedItem *item = r.find(e.tag);
    if (item == nullptr) {
      std::printf("FAIL  tag 0x%02X missing\n", e.tag);
      fail++;
      continue;
    }
    TagInfo info{};
    if (!tag_info(e.tag, &info) || info.width != e.width || info.enc != e.enc) {
      std::printf("FAIL  tag 0x%02X width/encoding: table says %u B, expected %u B\n", e.tag, info.width, e.width);
      fail++;
      continue;
    }
    // `raw` is a magnitude, so a signed BCD row is compared on its magnitude.
    uint32_t got = 0;
    bool ok = true;
    const bool is_bcd = (e.enc == TagEnc::BCD_LE) || (e.enc == TagEnc::BCD_LE_SIGNED);
    if (e.enc == TagEnc::BCD_LE) {
      ok = item_as_bcd(*item, &got);
    } else if (e.enc == TagEnc::BCD_LE_SIGNED) {
      int32_t signed_got = 0;
      ok = item_as_bcd_signed(*item, &signed_got);
      got = static_cast<uint32_t>((signed_got < 0) ? -signed_got : signed_got);
    } else {
      got = item_as_u32(*item);
    }
    const bool pass = ok && got == e.raw && item->len == e.width;
    std::printf("%s  tag 0x%02X %u B %-6s raw %-9u %s\n", pass ? "PASS" : "FAIL", e.tag, item->len,
                is_bcd ? "BCD" : "bin", got, e.shown);
    if (!pass) {
      std::printf("      expected raw %u\n", e.raw);
      fail++;
    }
  }

  const ParsedItem *clk = r.find(0x29);
  char buf[20];
  const bool clock_ok = clk != nullptr && item_clock_to_string(*clk, buf, sizeof(buf));
  std::printf("%s  tag 0x29 7 B BCD  %s\n", clock_ok ? "PASS" : "FAIL", clock_ok ? buf : "(invalid)");
  if (!clock_ok)
    fail++;

  // Energy invariant, as for DI 0xF200.
  const ParsedItem *t = r.find(0x00), *t1 = r.find(0x01), *t2 = r.find(0x02);
  check(t && t1 && t2 && item_as_u32(*t) == item_as_u32(*t1) + item_as_u32(*t2), "total == T1 + T2");

  // The truncation that hid this page: 90 bytes must be rejected, not half-parsed.
  ParsedResponse trunc{};
  check(parse_response(frame, 90, serial, &trunc) != ParseResult::OK, "a 90-byte truncation is rejected");

  // A live page from a second meter whose COUNT byte says 24 while DATA holds 16
  // records. COUNT is an upper bound; taking it literally used to reject the page.
  std::printf("\n-- live page, COUNT 24 / 16 records sent --\n");
  uint8_t serial2[SERIAL_BCD_SIZE];
  serial_to_bcd_le("023240275596", serial2);
  uint8_t frame2[256];
  const size_t len2 = unhex(SHORT_PAGE_HEX, frame2, sizeof(frame2));
  ParsedResponse r2{};
  const ParseResult pr2 = parse_response(frame2, len2, serial2, &r2);
  std::printf("parse: %s, DI 0x%04X, count %u of %u announced, payload %u B\n", parse_result_to_string(pr2), r2.di,
              r2.count, r2.announced_count, r2.payload_len);
  check(pr2 == ParseResult::OK, "short page parses");
  check(r2.di == DI_LIST_B_RECORDS, "DI is 0xF202");
  check(r2.count == 16, "16 records decoded");
  check(r2.announced_count == 24, "COUNT byte reported as announced_count");
  check(r2.payload_len == 0x53, "DATA length is 0x53 = 83");
  const ParsedItem *v2 = r2.find(0x15);
  uint32_t v2_raw = 0;
  check(v2 != nullptr && item_as_bcd(*v2, &v2_raw) && v2_raw == 2165, "last-but-one family still aligned (215... V)");
  check(r2.find(0x22) != nullptr, "the final record, TAG 0x22, is present");
  check(r2.find(0x28) == nullptr, "records the meter did not send are absent");

  // --- the status half of list B ----------------------------------------
  //
  // DI(2) | leftover records | status block. The records are the eight live list B
  // announced and did not fit - 33 bytes of value plus 8 TAG bytes, so DATA is
  // 2 + 41 + 11 = 54. No COUNT byte and the block always last, so the split is
  // arithmetic rather than guesswork.
  std::printf("\n-- list B status half, with leftover records --\n");
  {
    static const uint8_t TAIL[] = {0x23, 0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2A};
    uint8_t data[2 + 41 + STATUS_BLOCK_SIZE];
    size_t n = 0;
    data[n++] = 0x03;  // DI little-endian: 0xF203
    data[n++] = 0xF2;
    for (uint8_t tag : TAIL) {
      TagInfo info{};
      check(tag_info(tag, &info), "the tail TAG has a width");
      data[n++] = tag;
      for (uint8_t b = 0; b < info.width; b++) {
        // Valid BCD digits, so the signed-reactive rows decode rather than being rejected.
        data[n++] = 0x11;
      }
    }
    const size_t records_end = n;
    std::memcpy(data + n, STATUS_BLOCK, STATUS_BLOCK_SIZE);
    n += STATUS_BLOCK_SIZE;
    check(n == sizeof(data), "DATA is 2 + 41 + 11 = 54 bytes");

    uint8_t frame3[256];
    const size_t len3 = build_response(frame3, sizeof(frame3), serial, data, n);
    ParsedResponse r3{};
    const ParseResult pr3 = parse_response(frame3, len3, serial, &r3);
    std::printf("parse: %s, DI 0x%04X, count %u, shape '%s', payload %u B\n", parse_result_to_string(pr3), r3.di,
                r3.count, payload_shape_to_string(r3.shape), r3.payload_len);
    check(pr3 == ParseResult::OK, "the status half parses");
    check(r3.di == DI_LIST_B_STATUS, "DI is 0xF203");
    check(r3.shape == PayloadShape::STATUS_HALF, "read as a status half");
    check(r3.count == 8, "all 8 leftover records decoded");
    check(r3.announced_count == 0, "a status half has no COUNT byte to announce anything");
    check(r3.find(0x23) != nullptr && r3.find(0x2A) != nullptr, "first and last leftover records present");
    check(r3.has_status_block, "the block is there");
    check(std::memcmp(r3.status_block, STATUS_BLOCK, STATUS_BLOCK_SIZE) == 0, "and it is the trailing 11 bytes");
    // The two widths that make this frame add up: get either wrong and the block lands
    // off by a byte.
    const ParsedItem *clock = r3.find(0x29);
    const ParsedItem *temp = r3.find(0x2A);
    check(clock != nullptr && clock->len == 7, "the clock in the tail is 7 bytes");
    check(temp != nullptr && temp->len == 2, "temperature in the tail is 2 bytes");
    (void) records_end;
  }

  // Nothing left to continue: DI(2) plus the block, the shortest a status half can be.
  std::printf("\n-- list B status half with nothing left over --\n");
  {
    uint8_t data[2 + STATUS_BLOCK_SIZE];
    data[0] = 0x03;
    data[1] = 0xF2;
    std::memcpy(data + 2, STATUS_BLOCK, STATUS_BLOCK_SIZE);
    uint8_t frame4[64];
    const size_t len4 = build_response(frame4, sizeof(frame4), serial, data, sizeof(data));
    ParsedResponse r4{};
    const ParseResult pr4 = parse_response(frame4, len4, serial, &r4);
    std::printf("parse: %s, DI 0x%04X, count %u, shape '%s'\n", parse_result_to_string(pr4), r4.di, r4.count,
                payload_shape_to_string(r4.shape));
    check(pr4 == ParseResult::OK, "it parses");
    check(r4.shape == PayloadShape::STATUS_HALF, "read as a status half");
    check(r4.count == 0, "zero leftover records");
    check(r4.has_status_block, "the block is still there - it always is");
    check(r4.payload_len == 2 + STATUS_BLOCK_SIZE, "DATA is exactly DI + block");
  }

  // The reply that first exposed all of this. Every layer below the records verifies,
  // and 2 + 11 = 13 makes it a status half with nothing left over - it was read as
  // DI | COUNT=1 | TAG 0x00 | 9 bytes for a while, the "COUNT" being the block's marker.
  std::printf("\n-- the captured status half --\n");
  {
    static const char *const F203_HEX = "1C00011D6896552740320268810D3625343355B7333B34523336344316C5E2";
    uint8_t frame6[64];
    const size_t len6 = unhex(F203_HEX, frame6, sizeof(frame6));
    ParsedResponse r6{};
    const ParseResult pr6 = parse_response(frame6, len6, serial2, &r6);
    std::printf("parse: %s, DI 0x%04X, count %u, shape '%s', payload %u B\n", parse_result_to_string(pr6), r6.di,
                r6.count, payload_shape_to_string(r6.shape), r6.payload_len);
    check(len6 == 31, "frame is 31 bytes from the LEN byte");
    check(pr6 == ParseResult::OK, "the captured reply parses");
    check(r6.di == DI_LIST_B_STATUS, "DI is 0xF203");
    check(r6.shape == PayloadShape::STATUS_HALF, "read as a status half");
    check(r6.count == 0, "no leftover records, so the cursor was already gone");
    check(r6.has_status_block, "the block is there");
    check(std::memcmp(r6.status_block, STATUS_BLOCK, STATUS_BLOCK_SIZE) == 0,
          "and it is the block the other cases use");
    std::printf("status block: %s\n", format_block(r6.status_block).c_str());
    check(r6.status_block[0] == 0x01, "byte 0 is the constant marker");
    // These pin the POSITIONS the `status:` entities read, not the names - the firmware
    // calls them the relay state and an internal object.
    check(r6.status_block[STATUS_OFF_TARIFF_COUNT] == 3, "byte 9 = 3");
    check(r6.status_block[STATUS_OFF_ACTIVE_TARIFF] == 1, "byte 10 = 1");
  }

  // The TAG table's shape: two rows overlapping, or a gap where the meter does send a
  // record, misaligns every record after it.
  std::printf("\n-- the TAG table --\n");
  {
    bool shape_ok = true;
    bool scale_ok = true;
    for (uint16_t t = 0x00; t <= 0xFF; t++) {
      TagInfo info{};
      const bool known = tag_info(static_cast<uint8_t>(t), &info);
      /* Contiguous 0x00..0x3F apart from 0x2B - the display's LCD test, not a value -
       * and nothing above 0x3F, where the vendor TAG list stops. A record of unknown
       * width has to abort the walk rather than be guessed past.
       */
      const bool want = (t <= 0x3F) && (t != 0x2B);
      if (known != want) {
        std::printf("FAIL  TAG 0x%02X is %s, expected %s\n", t, known ? "known" : "unknown",
                    want ? "known" : "unknown");
        shape_ok = false;
      }
      if (known && (info.width == 0 || info.width > MAX_ITEM_WIDTH)) {
        std::printf("FAIL  TAG 0x%02X width %u is out of range\n", t, info.width);
        shape_ok = false;
      }
      if (known && (info.scale == 0.0f || info.unit == nullptr)) {
        std::printf("FAIL  TAG 0x%02X has scale %g, unit %s\n", t, info.scale, info.unit ? info.unit : "(null)");
        scale_ok = false;
      }
    }
    check(shape_ok, "every TAG 0x00-0x3F but 0x2B has a usable width, and nothing above 0x3F does");
    // A zero scale would publish 0 for every reading of that TAG, silently.
    check(scale_ok, "every known TAG has a non-zero scale and a unit");

    TagInfo info{};
    // The one row that breaks the pattern of its neighbours.
    check(tag_info(0x2A, &info) && info.width == 2 && info.enc == TagEnc::INT_LE,
          "temperature 0x2A is 2 bytes of signed binary, not the 4-byte BCD around it");
    check(tag_info(0x24, &info) && info.enc == TagEnc::BCD_LE_SIGNED, "reactive power 0x24 is signed BCD");
    // Signed because the vendor TAG list marks 0x20..0x23 so; not yet seen negative.
    check(tag_info(0x23, &info) && info.enc == TagEnc::BCD_LE_SIGNED, "active power 0x23 is signed BCD as well");
    check(tag_info(0x00, &info) && info.enc == TagEnc::UINT_LE, "energy 0x00 is binary, the one non-BCD family");
    check(tag_info(0x29, &info) && info.width == 7 && info.enc == TagEnc::BCD_CLOCK, "the clock 0x29 is 7 bytes");
    check(!tag_info(0x48, &info), "the identity objects from 0x48 up have no single width to walk by");

    // The status block is not a record and has no TAG, so nothing shadows TAG 0x00.
    check(tag_info(0x00, &info) && info.width == 4, "TAG 0x00 is the 4-byte energy register in either half");

    // A YAML-declared width fills gaps only - it must never shadow a built-in row.
    uint8_t overrides[TAG_WIDTH_TABLE_SIZE] = {};
    overrides[0x00] = 9;
    overrides[0x2B] = 3;
    check(tag_info(0x00, &info, overrides) && info.width == 4, "a declared width cannot shadow a known TAG");
    check(tag_info(0x2B, &info, overrides) && info.width == 3 && info.enc == TagEnc::USER,
          "a declared width does fill a gap");

    // Signed BCD, with the worked example from the vendor notes.
    ParsedItem q{};
    q.tag = 0x24;
    q.len = 4;
    q.raw[0] = 0x78;
    q.raw[1] = 0x08;
    q.raw[2] = 0x00;
    q.raw[3] = 0x80;
    int32_t sv = 0;
    check(item_as_bcd_signed(q, &sv) && sv == -878, "signed BCD 78 08 00 80 = -878 (-87.8 var)");
    q.raw[3] = 0x00;
    check(item_as_bcd_signed(q, &sv) && sv == 878, "the same digits with the sign bit clear = +878");
    q.raw[1] = 0x0A;
    check(!item_as_bcd_signed(q, &sv), "a non-BCD nibble is rejected rather than guessed at");
  }

  std::printf("\n%s (%d failure(s))\n", fail == 0 ? "DI 0xF202/0xF203 PAGES DECODE CORRECTLY" : "FAILURES", fail);
  return fail ? 1 : 0;
}
