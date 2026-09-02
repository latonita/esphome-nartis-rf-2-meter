// Decode check for the DI 0xF202 parameter page, from a real capture.
//
// The on-air capture was truncated by an undersized RX buffer, so the frame below
// is the same page reconstructed from its records. That is not circular: the
// reconstruction has to reproduce the DL/T 645 length field (L = 0x51) and the
// envelope length (LEN = 0x60) that the truncated capture did carry, and those
// only come out right if every item width is right. With 2-byte instantaneous
// values - which is what the numbers alone suggest, since the upper two bytes are
// always zero - L would be 0x41 instead.
#include "d101_frame.h"

#include <cstdio>
#include <cstring>

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

/// Wrap `data` (the de-offset application payload) in a DL/T 645 read response
/// and a radio envelope, exactly as parse_response() expects to receive it: from
/// the LEN byte onwards. Returns the length written.
///
/// This lives in the test rather than in d101_frame.cpp on purpose - the component
/// has no business constructing frames the meter is supposed to send.
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
  check(r.di == DI_PARAMS, "DI is 0xF202");
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
      {0x20, 4, TagEnc::BCD_LE, 795, "7.95 kW"},
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
    if (!tag_info(r.di, e.tag, &info) || info.width != e.width || info.enc != e.enc) {
      std::printf("FAIL  tag 0x%02X width/encoding: table says %u B, expected %u B\n", e.tag, info.width, e.width);
      fail++;
      continue;
    }
    uint32_t got = 0;
    bool ok = true;
    if (e.enc == TagEnc::BCD_LE) {
      ok = item_as_bcd(*item, &got);
    } else {
      got = item_as_u32(*item);
    }
    const bool pass = ok && got == e.raw && item->len == e.width;
    std::printf("%s  tag 0x%02X %u B %-6s raw %-9u %s\n", pass ? "PASS" : "FAIL", e.tag, item->len,
                e.enc == TagEnc::BCD_LE ? "BCD" : "bin", got, e.shown);
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

  // The truncation that hid this page: 90 bytes is what an undersized RX buffer
  // captured, and it must be rejected rather than half-parsed.
  ParsedResponse trunc{};
  check(parse_response(frame, 90, serial, &trunc) != ParseResult::OK, "a 90-byte truncation is rejected");

  // A live page off the air from a second meter, complete and CRC-valid, whose
  // COUNT byte says 24 while DATA holds 16 records (0x53 = 83 = 3 + 16*5). The
  // meter announces its whole record set and sends what fits, so COUNT is an
  // upper bound; taking it literally used to reject the page as malformed.
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
  check(r2.di == DI_PARAMS, "DI is 0xF202");
  check(r2.count == 16, "16 records decoded");
  check(r2.announced_count == 24, "COUNT byte reported as announced_count");
  check(r2.payload_len == 0x53, "DATA length is 0x53 = 83");
  const ParsedItem *v2 = r2.find(0x15);
  uint32_t v2_raw = 0;
  check(v2 != nullptr && item_as_bcd(*v2, &v2_raw) && v2_raw == 2165, "last-but-one family still aligned (215... V)");
  check(r2.find(0x22) != nullptr, "the final record, TAG 0x22, is present");
  check(r2.find(0x28) == nullptr, "records the meter did not send are absent");

  // --- DI 0xF203, the tail of that list ---------------------------------
  //
  // Not captured: this is the shape the meter firmware builds for a resume - the
  // records stream out with no COUNT byte in front of them - assembled here so the
  // parser is exercised against it. The records chosen are the eight the reference
  // page announced but did not send (reactive power and the billing-period
  // mirrors), each 1 + 4 bytes, so DATA is 2 + 8*5 = 42.
  std::printf("\n-- DI 0xF203 continuation, no COUNT byte --\n");
  {
    static const uint8_t TAIL_TAGS[] = {0x24, 0x25, 0x26, 0x27, 0x30, 0x31, 0x32, 0x33};
    uint8_t data[2 + 8 * 5];
    size_t n = 0;
    data[n++] = 0x03;  // DI little-endian: 0xF203
    data[n++] = 0xF2;
    for (uint8_t tag : TAIL_TAGS) {
      data[n++] = tag;
      data[n++] = 0x11;
      data[n++] = 0x00;
      data[n++] = 0x00;
      data[n++] = 0x00;
    }
    uint8_t frame3[256];
    const size_t len3 = build_response(frame3, sizeof(frame3), serial, data, n);
    ParsedResponse r3{};
    const ParseResult pr3 = parse_response(frame3, len3, serial, &r3);
    std::printf("parse: %s, DI 0x%04X, count %u, announced %u, shape '%s', payload %u B\n",
                parse_result_to_string(pr3), r3.di, r3.count, r3.announced_count,
                payload_shape_to_string(r3.shape), r3.payload_len);
    check(pr3 == ParseResult::OK, "continuation page parses");
    check(r3.di == DI_PARAMS_CONT, "DI is 0xF203");
    check(r3.shape == PayloadShape::CONTINUATION, "read as a continuation - no COUNT byte");
    check(r3.count == 8, "all 8 tail records decoded");
    check(r3.announced_count == 0, "no COUNT byte to announce anything");
    check(r3.find(0x24) != nullptr && r3.find(0x33) != nullptr, "first and last tail records present");
  }

  // With no cursor set the meter answers with the echoed DI and nothing else. That
  // is a valid response holding zero records, not a malformed frame.
  std::printf("\n-- DI 0xF203 with nothing pending --\n");
  {
    const uint8_t data[2] = {0x03, 0xF2};
    uint8_t frame4[64];
    const size_t len4 = build_response(frame4, sizeof(frame4), serial, data, sizeof(data));
    ParsedResponse r4{};
    const ParseResult pr4 = parse_response(frame4, len4, serial, &r4);
    std::printf("parse: %s, DI 0x%04X, count %u\n", parse_result_to_string(pr4), r4.di, r4.count);
    check(pr4 == ParseResult::OK, "an empty tail parses");
    check(r4.di == DI_PARAMS_CONT, "DI is 0xF203");
    check(r4.count == 0, "zero records");
  }

  // The fallback in the other direction: should a resume turn out to carry a COUNT
  // byte after all, the exact-fit tie-breaker has to find it rather than read the
  // COUNT as a TAG. Same eight records, this time with COUNT = 8 in front.
  std::printf("\n-- DI 0xF203 that does carry a COUNT byte --\n");
  {
    static const uint8_t TAIL_TAGS[] = {0x24, 0x25, 0x26, 0x27, 0x30, 0x31, 0x32, 0x33};
    uint8_t data[3 + 8 * 5];
    size_t n = 0;
    data[n++] = 0x03;
    data[n++] = 0xF2;
    data[n++] = 8;  // COUNT
    for (uint8_t tag : TAIL_TAGS) {
      data[n++] = tag;
      data[n++] = 0x11;
      data[n++] = 0x00;
      data[n++] = 0x00;
      data[n++] = 0x00;
    }
    uint8_t frame5[256];
    const size_t len5 = build_response(frame5, sizeof(frame5), serial, data, n);
    ParsedResponse r5{};
    const ParseResult pr5 = parse_response(frame5, len5, serial, &r5);
    std::printf("parse: %s, DI 0x%04X, count %u, announced %u, shape '%s'\n", parse_result_to_string(pr5), r5.di,
                r5.count, r5.announced_count, payload_shape_to_string(r5.shape));
    check(pr5 == ParseResult::OK, "the COUNT-byte form parses too");
    check(r5.shape == PayloadShape::COUNTED, "recognised as a counted page, not a continuation");
    check(r5.count == 8, "all 8 records decoded");
    check(r5.find(0x24) != nullptr && r5.find(0x33) != nullptr, "records are aligned, so COUNT was not read as a TAG");
  }

  // The DI 0xF203 reply as it actually came off the air, from meter ...5596 on
  // 2026-09-02. Every layer below the records verifies - envelope CRC 0xE2C5,
  // DL/T 645 checksum 0x43, our address, control 0x81, L = 0x0D consistent - and
  // DATA de-offsets to 03 F2 | 01 | 00 | 22 84 00 08 01 1F 00 03 01.
  //
  // That is not a record tail. It is DI | COUNT=1 | TAG 0x00 | *nine* bytes: the
  // DI 0xF201 status-block shape, and 3 + 1 + 9 = 13 fits DATA exactly. Read with
  // the TAG-page widths, where TAG 0x00 is a 4-byte register, no reading fits at
  // all - which is how this frame used to be thrown away as malformed.
  std::printf("\n-- DI 0xF203 as captured off the air --\n");
  {
    static const char *const F203_HEX = "1C00011D6896552740320268810D3625343355B7333B34523336344316C5E2";
    uint8_t frame6[64];
    const size_t len6 = unhex(F203_HEX, frame6, sizeof(frame6));
    ParsedResponse r6{};
    const ParseResult pr6 = parse_response(frame6, len6, serial2, &r6);
    std::printf("parse: %s, DI 0x%04X, count %u, announced %u, shape '%s', payload %u B\n",
                parse_result_to_string(pr6), r6.di, r6.count, r6.announced_count,
                payload_shape_to_string(r6.shape), r6.payload_len);
    check(len6 == 31, "frame is 31 bytes from the LEN byte");
    check(pr6 == ParseResult::OK, "the captured DI 0xF203 reply parses (it used to be MALFORMED)");
    check(r6.di == DI_PARAMS_CONT, "DI is 0xF203");
    check(r6.shape == PayloadShape::COUNTED_STATUS, "recognised as a counted page holding a status block");
    check(r6.count == 1 && r6.announced_count == 1, "one record, and COUNT agrees");
    const ParsedItem *blob = r6.find(0x00);
    check(blob != nullptr && blob->len == STATUS_VALUE_SIZE, "TAG 0x00 is the 9-byte status block");
    if (blob != nullptr && blob->len == STATUS_VALUE_SIZE) {
      std::printf("status block: %02X %02X %02X %02X %02X %02X %02X %02X %02X\n", blob->raw[0], blob->raw[1],
                  blob->raw[2], blob->raw[3], blob->raw[4], blob->raw[5], blob->raw[6], blob->raw[7], blob->raw[8]);
      // The three fields the status decoder reads. Plausible values are the only
      // corroboration available for this reading, so state them and let the
      // numbers be judged: 31 C in September, 3 tariffs, tariff 1 active.
      check(blob->raw[STATUS_OFF_TEMPERATURE] == 31, "byte 5 (temperature) = 31");
      check(blob->raw[STATUS_OFF_TARIFF_COUNT] == 3, "byte 7 (tariff count) = 3");
      check(blob->raw[STATUS_OFF_ACTIVE_TARIFF] == 1, "byte 8 (active tariff) = 1");
    }
  }

  std::printf("\n%s (%d failure(s))\n", fail == 0 ? "DI 0xF202/0xF203 PAGES DECODE CORRECTLY" : "FAILURES", fail);
  return fail ? 1 : 0;
}
