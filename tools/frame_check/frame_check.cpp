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

  // The channel grid is uniform: 435.5 MHz + k * 0.7 MHz, with no special step.
  // A +100 kHz jump above k=18 used to be applied here; SPI captures of a real
  // display disprove it - FREQ_CHNL advances by exactly +8 per k right across
  // k=17..19, and a uniform channel step cannot produce a non-uniform frequency
  // step. These assertions exist so that step cannot creep back in.
  auto freq_for_k = [](unsigned k) {
    char serial[13];
    std::snprintf(serial, sizeof(serial), "000000000%03u", k);  // last 3 digits = k, k < 24
    return frequency_from_serial(serial);
  };
  bool uniform = true;
  for (unsigned k = 1; k < 24; k++) {
    if (freq_for_k(k) - freq_for_k(k - 1) != 700000u) {
      std::printf("FAIL  step k=%u -> %u is %u Hz, expected 700000\n", k - 1, k,
                  freq_for_k(k) - freq_for_k(k - 1));
      uniform = false;
    }
  }
  check(uniform, "channel grid steps a uniform 0.7 MHz for all k (no +100 kHz above k=18)");
  check(freq_for_k(17) == 447400000u, "k=17 -> 447.400 MHz (confirmed on air)");
  check(freq_for_k(19) == 448800000u, "k=19 -> 448.800 MHz (was 448.900 before the fix)");
  check(freq_for_k(23) == 451600000u, "k=23 -> 451.600 MHz");

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

  // ---------------------------------------------------------------------
  // Variant 2
  // ---------------------------------------------------------------------
  // The frame below is the FIFO content of a real display's variant-2 request,
  // recovered from an SPI capture (channels CSB / FCSB / SDIO, 2026-08) while it
  // polled meter 023250209637, with the chip-generated 98 f3 sync prepended.
  // Both of its checksums were verified independently against the capture: the
  // DL/T 645 sum 0x70 and the CRC-16/X.25 EA1E.
  uint8_t v2_serial[SERIAL_BCD_SIZE];
  serial_to_bcd_le("023250209637", v2_serial);
  const uint8_t want_v2_serial[6] = {0x37, 0x96, 0x20, 0x50, 0x32, 0x02};
  check(std::memcmp(v2_serial, want_v2_serial, 6) == 0, "v2 serial -> 37 96 20 50 32 02");

  check(channel_from_serial("023250209637") == 13, "...637 -> channel 13");
  check(channel_frequency(13) == 444600000u, "channel 13 = 444.600 MHz");
  check(channel_from_serial("023240271060") == 12, "...060 -> channel 12");
  check(channel_from_frequency(444600000u) == 13, "444.6 MHz -> channel 13");
  check(channel_from_frequency(444500000u) == 13, "an off-grid frequency rounds to the nearest channel");
  check(channel_from_frequency(400000000u) == 0, "below the grid clamps to channel 0");
  check(channel_from_frequency(999000000u) == CHANNEL_COUNT - 1, "above the grid clamps to the last channel");

  const uint8_t want_v2[] = {0x98, 0xF3, 0x13, 0x00, 0x01, 0x12, 0x68, 0x37, 0x96, 0x20, 0x50, 0x32,
                             0x02, 0x68, 0x11, 0x04, 0x53, 0x33, 0x53, 0x41, 0x70, 0x16, 0x1E, 0xEA};
  uint8_t v2[MAX_REQUEST_FRAME_SIZE];
  const size_t v2_len = build_v2_request(v2, sizeof(v2), v2_serial);
  hex("v2 built", v2, v2_len);
  hex("v2 capture", want_v2, sizeof(want_v2));
  check(v2_len == sizeof(want_v2) && std::memcmp(v2, want_v2, sizeof(want_v2)) == 0,
        "variant-2 request matches the SPI capture exactly");

  // The read codes are the only control codes the builder will emit. This is the
  // guarantee that keeps the component read-only, so assert it rather than trust it.
  uint8_t reject[MAX_REQUEST_FRAME_SIZE];
  check(build_read_request(reject, sizeof(reject), v2_serial, V2_REQUEST_DI, V2_REQUEST_BODY,
                           V2_REQUEST_BODY_SIZE, 0x14) == 0,
        "a write control code is refused (0x14)");
  check(build_read_request(reject, sizeof(reject), v2_serial, V2_REQUEST_DI, V2_REQUEST_BODY,
                           V2_REQUEST_BODY_SIZE, 0x04) == 0,
        "a non-read control code is refused (0x04)");
  check(build_read_request(reject, sizeof(reject), serial, DI_ENERGY, ENERGY_REQUEST_BODY,
                           sizeof(ENERGY_REQUEST_BODY)) != 0,
        "the default control code is still accepted");

  std::printf("\n%s (%d failure(s))\n", failures == 0 ? "ALL CHECKS PASSED" : "CHECKS FAILED", failures);
  return failures == 0 ? 0 : 1;
}
