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

  std::printf("\n%s (%d failure(s))\n", fail == 0 ? "DI 0xF202 PAGE DECODES CORRECTLY" : "FAILURES", fail);
  return fail ? 1 : 0;
}
