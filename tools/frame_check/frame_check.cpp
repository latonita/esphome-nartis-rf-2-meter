// Host-side check of d101_frame.cpp against PROTOCOL-D101-2.local.md.
// Build: g++ -std=c++17 -o frame_check frame_check.cpp <component>/d101_frame.cpp
#include "d101_frame.h"

#include <cstdio>
#include <cstring>
#include <initializer_list>

using namespace esphome::nartis_rf_2_meter;

static int failures = 0;

static void hex(const char *label, const uint8_t *d, size_t n) {
  std::printf("%-14s", label);
  for (size_t i = 0; i < n; i++)
    std::printf("%02X ", d[i]);
  std::printf("\n");
}

static void check(bool ok, const char *what) {
  std::printf("%s  %s\n", ok ? "PASS" : "FAIL", what);
  if (!ok)
    failures++;
}

int main() {
  uint8_t serial[SERIAL_BCD_SIZE];
  serial_to_bcd_le("023240271060", serial);
  hex("serial_le", serial, sizeof(serial));
  const uint8_t want_serial[6] = {0x60, 0x10, 0x27, 0x40, 0x32, 0x02};
  check(std::memcmp(serial, want_serial, 6) == 0, "serial -> 60 10 27 40 32 02");

  const uint32_t f = frequency_from_serial("023240271060");
  std::printf("frequency      %.3f MHz\n", f / 1e6);
  check(f == 443900000u, "frequency 443.900 MHz for ...060 (k=12, confirmed on air)");

  // The grid steps 0.7 MHz per k, with one extra 100 kHz above k=18. The break
  // is on-air evidence: serial ...596 (k=20) answers on 449.600 and is silent on
  // 449.500, and k=12/k=13 pin the base below it. A reading of the display's own
  // SPI capture suggested a uniform grid - FREQ_CHNL advancing by exactly +8 per
  // k across k=17..19 - but where the meter actually transmits settles it, so
  // these assertions exist to stop the uniform grid creeping back in.
  auto freq_for_k = [](unsigned k) {
    char serial[13];
    std::snprintf(serial, sizeof(serial), "000000000%03u", k);  // last 3 digits = k, k < 24
    return frequency_from_serial(serial);
  };
  bool grid_ok = true;
  for (unsigned k = 1; k < 24; k++) {
    const uint32_t want = (k == 19) ? 800000u : 700000u;  // the one-off +100 kHz lands on k=19
    const uint32_t got = freq_for_k(k) - freq_for_k(k - 1);
    if (got != want) {
      std::printf("FAIL  step k=%u -> %u is %u Hz, expected %u\n", k - 1, k, got, want);
      grid_ok = false;
    }
  }
  check(grid_ok, "channel grid steps 0.7 MHz per k, +100 kHz once above k=18");
  check(freq_for_k(17) == 447400000u, "k=17 -> 447.400 MHz (below the break, not itself verified)");
  check(freq_for_k(20) == 449600000u, "k=20 -> 449.600 MHz (confirmed on air, serial ...596)");
  check(freq_for_k(19) == 448900000u, "k=19 -> 448.900 MHz");
  check(freq_for_k(23) == 451700000u, "k=23 -> 451.700 MHz (the top of the documented band)");

  // --- energy request: must match the doc byte-for-byte (LEN 0x17 is odd, so
  //     HLEN = LEN-1 and LEN^1 agree) ---
  const uint8_t want_energy[] = {0x98, 0xF3, 0x17, 0x00, 0x01, 0x16, 0x68, 0x60, 0x10, 0x27, 0x40, 0x32, 0x02, 0x68,
                                 0x01, 0x08, 0x33, 0x25, 0x33, 0x33, 0x33, 0x33, 0x34, 0x56, 0x92, 0x16, 0xFE, 0xB0};
  uint8_t buf[MAX_REQUEST_FRAME_SIZE];
  size_t n = build_request(buf, sizeof(buf), serial, DI_ENERGY);
  hex("energy built", buf, n);
  hex("energy doc", want_energy, sizeof(want_energy));
  check(n == sizeof(want_energy) && std::memcmp(buf, want_energy, n) == 0, "energy request matches the capture exactly");

  // --- status request: LEN 0x12 is even, the case where LEN ^ 1 and LEN - 1
  //     differ. HLEN = LEN ^ 1 is what the display sends and what we now send, so
  //     this must match the capture byte-for-byte too. ---
  const uint8_t want_status[] = {0x98, 0xF3, 0x12, 0x00, 0x01, 0x13, 0x68, 0x60, 0x10, 0x27, 0x40, 0x32,
                                 0x02, 0x68, 0x01, 0x03, 0x34, 0x25, 0x33, 0x6B, 0x16, 0x29, 0x0A};
  n = build_request(buf, sizeof(buf), serial, DI_STATUS);
  hex("status built", buf, n);
  hex("status doc", want_status, sizeof(want_status));
  check(n == sizeof(want_status) && std::memcmp(buf, want_status, n) == 0,
        "status request matches the capture exactly (HLEN = LEN ^ 1)");
  check(buf[5] == 0x13, "status HLEN is LEN ^ 1 = 0x13");

  // Both rules agree on the odd-length energy poll, so only the status poll can
  // tell them apart - which is why it is the one that matters here.
  check((0x17 ^ 1) == (0x17 - 1), "for odd LEN the two HLEN rules coincide");
  check((0x12 ^ 1) != (0x12 - 1), "for even LEN they differ");

  // --- worked-example response from doc section 6 ---
  // On-air bytes with the 98 F3 sync stripped, as the radio hands them over.
  const uint8_t resp[] = {0x29, 0x00, 0x01, 0x28, 0x68, 0x60, 0x10, 0x27, 0x40, 0x32, 0x02, 0x68,
                          0x81, 0x1A, 0x33, 0x25, 0x37, 0x33, 0x68, 0x7E, 0xFC, 0x33, 0x34, 0x9A,
                          0x2D, 0xBD, 0x33, 0x35, 0x01, 0x83, 0x71, 0x33, 0x5C, 0x8B, 0x87, 0x48,
                          0x38, 0x4A, 0x3A, 0x59, 0x60, 0x16, 0x67, 0x6D,
                          // trailing capture noise, as fixed-length RX would deliver
                          0xAA, 0x55, 0xAA, 0x55};
  ParsedResponse r{};
  const ParseResult pr = parse_response(resp, sizeof(resp), serial, &r);
  std::printf("parse          %s, DI 0x%04X, count %u\n", parse_result_to_string(pr), r.di, r.count);
  hex("payload", r.payload, r.payload_len);
  check(pr == ParseResult::OK, "worked-example response parses");
  check(r.di == DI_ENERGY, "DI is 0xF200");
  check(r.count == 4, "4 items");

  const ParsedItem *total = r.find(0x00);
  const ParsedItem *t1 = r.find(0x01);
  const ParsedItem *t2 = r.find(0x02);
  const ParsedItem *clk = r.find(0x29);
  check(total && t1 && t2 && clk, "TAGs 0x00 0x01 0x02 0x29 all present");
  if (total && t1 && t2) {
    std::printf("total %u, T1 %u, T2 %u\n", item_as_u32(*total), item_as_u32(*t1), item_as_u32(*t2));
    check(item_as_u32(*total) == 13191989u, "total = 13191989 Wh");
    check(item_as_u32(*t1) == 9108071u, "T1 = 9108071 Wh");
    check(item_as_u32(*t2) == 4083918u, "T2 = 4083918 Wh");
    check(item_as_u32(*total) == item_as_u32(*t1) + item_as_u32(*t2), "total == T1 + T2");
  }
  if (clk) {
    char s[20];
    const bool ok = item_clock_to_string(*clk, s, sizeof(s));
    std::printf("clock          %s\n", ok ? s : "(invalid)");
    check(ok && std::strcmp(s, "2026-07-17 15:54:58") == 0, "clock = 2026-07-17 15:54:58");
  }

  // --- rejection cases ---
  ParsedResponse tmp{};
  const uint8_t other_serial[6] = {0x61, 0x10, 0x27, 0x40, 0x32, 0x02};
  check(parse_response(resp, sizeof(resp), other_serial, &tmp) == ParseResult::WRONG_ADDRESS,
        "a different meter's address is rejected");

  uint8_t corrupt[sizeof(resp)];
  std::memcpy(corrupt, resp, sizeof(resp));
  corrupt[20] ^= 0xFF;  // flip a payload byte -> CRC must fail
  check(parse_response(corrupt, sizeof(corrupt), serial, &tmp) == ParseResult::NO_FRAME,
        "a corrupted frame is rejected");

  // Which page a TAG forces. DI 0xF202 is a superset, so only the instantaneous
  // values and temperature force it; energy and the clock are on both pages.
  for (uint8_t t : {0x00, 0x01, 0x02, 0x13, 0x29, 0x2C, 0x3F}) {
    if (tag_needs_params_page(t)) {
      std::printf("FAIL  TAG 0x%02X should not force the 0xF202 page\n", t);
      failures++;
    }
  }
  for (uint8_t t : {0x14, 0x15, 0x1C, 0x1F, 0x20, 0x27, 0x28, 0x2A, 0x2B}) {
    if (!tag_needs_params_page(t)) {
      std::printf("FAIL  TAG 0x%02X should force the 0xF202 page\n", t);
      failures++;
    }
  }
  check(true, "page selection: energy and clock on either page, instantaneous only on 0xF202");

  std::printf("\n%s (%d failure(s))\n", failures == 0 ? "ALL CHECKS PASSED" : "CHECKS FAILED", failures);
  return failures == 0 ? 0 : 1;
}
