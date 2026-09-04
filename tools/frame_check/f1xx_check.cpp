// Decode checks for the fixed blocks, DI 0xF101 and DI 0xF102.
//
// These are not record pages: no COUNT, no TAGs, and a length that belongs to the
// layout rather than to whatever fitted in the frame. So length is the whole
// identification - it is what tells the three-phase F102 from the single-phase
// one - and that is what most of this file exercises.
//
// The F102 three-phase case is a real capture from meter 023240271060. F101 is
// assembled from the layout in nartis_dlt645_f1xx.h - its own capture came later
// and only confirmed the length arithmetic, which is all the parser does with it.
//
// The last section covers the other half of a fixed block being useful: the map
// from its positional fields onto TAGs, and the scales that put a value from here
// and the same value from a list record into one unit.
#include "d101_frame.h"

#include <cstddef>
#include <cstdio>
#include <cstring>
#include <initializer_list>

using namespace esphome::nartis_rf_2_meter;

static int fail = 0;

static void check(bool ok, const char *what) {
  std::printf("%s  %s\n", ok ? "PASS" : "FAIL", what);
  if (!ok)
    fail++;
}

static size_t unhex(const char *hex, uint8_t *out, size_t cap) {
  size_t n = 0;
  for (const char *p = hex; *p && p[1] && n < cap; p += 2) {
    unsigned b = 0;
    std::sscanf(p, "%2x", &b);
    out[n++] = (uint8_t) b;
  }
  return n;
}

/// Wrap a de-offset payload in a DL/T 645 read response and a radio envelope,
/// exactly as parse_response() expects to receive it: from the LEN byte onwards.
/// Same helper as f202_check.cpp - it lives in the tests because the component
/// has no business building frames the meter is supposed to send.
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

int main() {
  uint8_t serial[SERIAL_BCD_SIZE];
  serial_to_bcd_le("023240271060", serial);

  std::printf("== DI 0xF102, the captured three-phase block ==\n");

  /* Captured off the air from meter 023240271060, from the LEN byte on, with the
   * nine bytes of radio tail past the CRC left in. Those are kept deliberately: a
   * real capture carries them, so the parser has to find the end of the frame
   * from the length fields rather than from where the buffer stops.
   */
  static const char *const CAPTURE =
      "4E00014F6860102740320268813F35243483793433936633337357333343"
      "BB3333B34433B3434633B3C33333B353353333B46735338C343333766935"
      "335B34333379673533BA363333CB7C3333C4160DC25452C972465292544A";

  uint8_t frame[128];
  const size_t buf_len = unhex(CAPTURE, frame, sizeof(frame));
  check(buf_len == 90, "the capture is 90 bytes as handed over by the radio");
  check(frame[0] == 0x4E, "LEN is 0x4E, so the frame proper ends at byte 81");

  ParsedResponse r{};
  const ParseResult pr = parse_response(frame, buf_len, serial, &r);
  std::printf("parse: %s, DI 0x%04X, shape '%s', payload %u B\n", parse_result_to_string(pr), r.di,
              payload_shape_to_string(r.shape), r.payload_len);
  check(pr == ParseResult::OK, "the captured reply parses");
  check(r.di == DI_FIXED_F102, "DI is 0xF102");
  check(r.shape == PayloadShape::FIXED_F102_3PH, "read as the three-phase fixed block");
  check(r.payload_len == sizeof(f102_3ph), "DATA is 63 bytes - which is the whole 3ph/1ph test");
  check(r.count == 0, "no records: a fixed block has no TAGs to walk");
  check(r.announced_count == 0, "and no COUNT byte to announce anything");

  // Overlay the struct, the way the component does.
  f102_3ph b{};
  std::memcpy(&b, r.payload, sizeof(b));
  check(b.marker == 0x01, "the marker byte is 0x01");

  struct Field {
    const char *name;
    const bcd32_le *f;
    int32_t want;
  };
  const Field fields[] = {
      {"P total", &b.p_total, 14650}, {"P L1", &b.p_l1, 3360},  {"P L2", &b.p_l2, 2440},
      {"P L3", &b.p_l3, 8810},        {"Q total", &b.q_total, -1180}, {"Q L1", &b.q_l1, -1310},
      {"Q L2", &b.q_l2, -90},         {"Q L3", &b.q_l3, 220},   {"U L1", &b.u_l1, 23481},
      {"I L1", &b.i_l1, 159},         {"U L2", &b.u_l2, 23643}, {"I L2", &b.i_l2, 128},
      {"U L3", &b.u_l3, 23446},       {"I L3", &b.i_l3, 387},   {"f", &b.freq, 4998},
  };
  bool all = true;
  for (const Field &f : fields) {
    const int32_t got = bcd32_signed(f.f);
    if (got != f.want) {
      std::printf("      %s: got %d, want %d\n", f.name, (int) got, (int) f.want);
      all = false;
    }
  }
  check(all, "all 15 positional values decode to the captured numbers");

  // The two internal cross-checks that pinned the field order from this single
  // capture. Q is exact; P is 40 counts light of 14650 because the phases are
  // sampled a moment apart, which is why the component tolerates 2% on P and
  // nothing on Q.
  const int32_t q_sum = bcd32_signed(&b.q_l1) + bcd32_signed(&b.q_l2) + bcd32_signed(&b.q_l3);
  const int32_t p_sum = bcd32_signed(&b.p_l1) + bcd32_signed(&b.p_l2) + bcd32_signed(&b.p_l3);
  check(q_sum == bcd32_signed(&b.q_total), "reactive: phases sum to the total exactly");
  check(p_sum == 14610, "active: phases sum to 14610 against a total of 14650 (0.27% apart)");
  std::printf("      P %.1f W, Q %.1f var, U1 %.2f V, I1 %.2f A, f %.2f Hz\n", bcd32_signed(&b.p_total) * 0.1,
              bcd32_signed(&b.q_total) * 0.1, bcd32_value(&b.u_l1) * 0.01, bcd32_value(&b.i_l1) * 0.01,
              bcd32_value(&b.freq) * 0.01);

  // Sign lives in the top byte and costs the top digit; the reactive values are
  // the only negatives in the capture.
  check(bcd32_value(&b.q_total) == 1180, "the unsigned reading drops the sign bit rather than reading it as a digit");

  std::printf("\n== DI 0xF102, single-phase ==\n");
  {
    uint8_t data[sizeof(f102_1ph)];
    std::memset(data, 0, sizeof(data));
    data[0] = 0x02;  // DI little-endian: 0xF102
    data[1] = 0xF1;
    data[2] = 0x00;  // lead byte, 0x00 on this variant rather than 0x01
    // Five values, valid BCD, the last one negative to exercise the sign path.
    for (size_t i = 0; i < 5; i++) {
      data[3 + i * 4 + 0] = 0x11;
      data[3 + i * 4 + 1] = 0x22;
    }
    data[3 + 3 * 4 + 3] = 0x80;  // current, negative

    uint8_t f[64];
    const size_t n = build_response(f, sizeof(f), serial, data, sizeof(data));
    ParsedResponse r1{};
    const ParseResult p1 = parse_response(f, n, serial, &r1);
    std::printf("parse: %s, shape '%s', payload %u B\n", parse_result_to_string(p1), payload_shape_to_string(r1.shape),
                r1.payload_len);
    check(p1 == ParseResult::OK, "it parses");
    check(r1.shape == PayloadShape::FIXED_F102_1PH, "23 bytes of DATA reads as the single-phase block");
    check(r1.payload_len == sizeof(f102_1ph), "DATA is 23 bytes");

    f102_1ph s{};
    std::memcpy(&s, r1.payload, sizeof(s));
    check(s.lead == 0x00, "the lead byte is 0x00");
    check(bcd32_value(&s.p) == 2211, "P decodes as BCD little-endian");
    check(bcd32_signed(&s.i) == -2211, "the sign bit on the current is honoured");
  }

  std::printf("\n== DI 0xF101 ==\n");
  {
    uint8_t data[sizeof(nartis_f101)];
    std::memset(data, 0, sizeof(data));
    data[0] = 0x01;  // DI little-endian: 0xF101
    data[1] = 0xF1;
    // Two groups of nine raw little-endian u32. Fill them so total = sum of the
    // eight tariffs, which is the one check the block offers on its own.
    for (uint8_t g = 0; g < 2; g++) {
      const size_t base = 2 + g * 36;
      uint32_t total = 0;
      for (uint8_t t = 1; t < 9; t++) {
        const uint32_t v = 1000u * (g + 1) + t;
        const size_t off = base + t * 4;
        data[off + 0] = (uint8_t) (v & 0xFF);
        data[off + 1] = (uint8_t) ((v >> 8) & 0xFF);
        data[off + 2] = (uint8_t) ((v >> 16) & 0xFF);
        data[off + 3] = (uint8_t) ((v >> 24) & 0xFF);
        total += v;
      }
      data[base + 0] = (uint8_t) (total & 0xFF);
      data[base + 1] = (uint8_t) ((total >> 8) & 0xFF);
      data[base + 2] = (uint8_t) ((total >> 16) & 0xFF);
      data[base + 3] = (uint8_t) ((total >> 24) & 0xFF);
    }
    // The status block, arbitrary bytes - all this case pins is where it sits.
    static const uint8_t STATUS10[10] = {0x00, 0x22, 0x84, 0x00, 0x08, 0x01, 0x1F, 0x00, 0x03, 0x01};
    std::memcpy(data + 2 + 72, STATUS10, sizeof(STATUS10));

    uint8_t f[160];
    const size_t n = build_response(f, sizeof(f), serial, data, sizeof(data));
    ParsedResponse r2{};
    const ParseResult p2 = parse_response(f, n, serial, &r2);
    std::printf("parse: %s, shape '%s', payload %u B\n", parse_result_to_string(p2), payload_shape_to_string(r2.shape),
                r2.payload_len);
    check(p2 == ParseResult::OK, "it parses");
    check(r2.di == DI_FIXED_F101, "DI is 0xF101");
    check(r2.shape == PayloadShape::FIXED_F101, "read as the F101 fixed block");
    check(r2.payload_len == 84, "DATA is 84 bytes: DI, 2 x 9 x u32, 10-byte status");

    nartis_f101 blk{};
    std::memcpy(&blk, r2.payload, sizeof(blk));
    uint32_t sum20 = 0, sum30 = 0;
    for (uint8_t t = 1; t < 9; t++) {
      sum20 += blk.group20[t];
      sum30 += blk.group30[t];
    }
    check(blk.group20[1] == 1001 && blk.group20[8] == 1008, "group 0x20 tariffs land in order");
    check(blk.group30[1] == 2001 && blk.group30[8] == 2008, "group 0x30 tariffs land in order");
    check(blk.group20[0] == sum20 && blk.group30[0] == sum30, "each group's total is its eight tariffs");
    check(std::memcmp(&blk.status, STATUS10, sizeof(STATUS10)) == 0, "the 10-byte status block is the trailing bytes");
    // A live capture showed the list's 11 bytes to be these 10 plus a trailing
    // 0x01, so the sizes have to differ by exactly one.
    check(STATUS_BLOCK_SIZE == sizeof(blk.status) + 1,
          "the list's status block is exactly one byte longer than F101's");

    /* The map onto TAGs. Ten of the eighteen accumulators: the TAG list stops at
     * T4, so T5..T8 have nowhere to go.
     *
     * The offsets are checked against the struct rather than restated, and the
     * order matters as much as the values - group 0x20 is reactive import and
     * lands on 0x0A..0x0E, group 0x30 export on 0x0F..0x13.
     */
    check(F101_VALUE_COUNT == 10, "ten of the eighteen accumulators are mapped");
    bool map_ok = true;
    for (uint8_t i = 0; i < F101_VALUE_COUNT; i++) {
      const FixedValue &fv = F101_MAP[i];
      const bool import_half = i < 5;
      const size_t want_off = (import_half ? offsetof(nartis_f101, group20) : offsetof(nartis_f101, group30)) +
                              4u * (import_half ? i : (i - 5));
      const uint8_t want_tag = (uint8_t) (0x0A + i);
      TagInfo info{};
      if (fv.offset != want_off || fv.tag != want_tag || fv.enc != TagEnc::UINT_LE || fv.scale != 0.001f ||
          !tag_info(want_tag, &info) || info.width != 4 || std::strcmp(info.unit, "kvarh") != 0) {
        std::printf("      entry %u: TAG 0x%02X off %u\n", i, fv.tag, fv.offset);
        map_ok = false;
      }
    }
    check(map_ok, "all ten land on TAGs 0x0A-0x13 at their struct offsets, x0.001 kvarh");

    // Read through the map, binary rather than BCD - the encoding that separates
    // this block from DI 0xF102. Group 0x20 T1 was filled with 1001 counts.
    float value = 0.0f;
    check(fixed_value(r2.payload, r2.payload_len, F101_MAP[1], &value) && value > 1.0005f && value < 1.0015f,
          "1001 raw counts read as binary LE and scale to 1.001 kvarh");
    check(fixed_value(r2.payload, r2.payload_len, F101_MAP[6], &value) && value > 2.0005f && value < 2.0015f,
          "and group 0x30 T1's 2001 counts to 2.001 kvarh");
    // 0xEE would be a bad nibble in BCD; here it is just a large number, which is
    // the point of the encoding field.
    uint8_t bad[sizeof(nartis_f101)];
    std::memcpy(bad, r2.payload, sizeof(bad));
    // 1001 = 0x03E9, so replacing the low byte gives 0x03EE = 1006.
    bad[F101_MAP[1].offset] = 0xEE;
    check(fixed_value(bad, sizeof(bad), F101_MAP[1], &value) && value > 1.0055f && value < 1.0065f,
          "a byte that is not BCD is still a valid binary count");
    check(!fixed_value(r2.payload, (uint8_t) (F101_MAP[9].offset + 3), F101_MAP[9], &value),
          "an entry reaching past the block is still refused");
  }

  std::printf("\n== length is the identification ==\n");
  {
    // One byte either side of each layout has to be refused, not read as a
    // records page. That is what makes the length load-bearing.
    const size_t lengths[] = {sizeof(f102_1ph) - 1, sizeof(f102_1ph) + 1, sizeof(f102_3ph) - 1, sizeof(f102_3ph) + 1};
    for (size_t want : lengths) {
      uint8_t data[128];
      std::memset(data, 0, sizeof(data));
      data[0] = 0x02;
      data[1] = 0xF1;
      uint8_t f[192];
      const size_t n = build_response(f, sizeof(f), serial, data, want);
      ParsedResponse rr{};
      const ParseResult p = parse_response(f, n, serial, &rr);
      char msg[96];
      std::snprintf(msg, sizeof(msg), "DI 0xF102 with %zu bytes of DATA is refused", want);
      check(p != ParseResult::OK, msg);
    }
    // And F101's length is not interchangeable with F102's.
    uint8_t data[128];
    std::memset(data, 0, sizeof(data));
    data[0] = 0x01;  // DI 0xF101
    data[1] = 0xF1;
    uint8_t f[192];
    const size_t n = build_response(f, sizeof(f), serial, data, sizeof(f102_3ph));
    ParsedResponse rr{};
    check(parse_response(f, n, serial, &rr) != ParseResult::OK, "DI 0xF101 with F102's length is refused");
  }

  std::printf("\n== requests ==\n");
  {
    uint8_t buf[64];
    for (uint16_t di : {DI_FIXED_F101, DI_FIXED_F102}) {
      const size_t n = build_request(buf, sizeof(buf), serial, di);
      char msg[64];
      std::snprintf(msg, sizeof(msg), "build_request() knows DI 0x%04X", di);
      check(n > 0, msg);
    }
    // Both fixed reads take the long body, so they differ from each other in the
    // DI alone - and from the list requests only in the DI too.
    uint8_t f101[64], f102[64];
    const size_t n101 = build_request(f101, sizeof(f101), serial, DI_FIXED_F101);
    const size_t n102 = build_request(f102, sizeof(f102), serial, DI_FIXED_F102);
    check(n101 == n102, "the two fixed requests are the same length");
    size_t diff = 0;
    for (size_t i = 0; i < n101; i++) {
      if (f101[i] != f102[i])
        diff++;
    }
    // The DI byte, the checksum and the two CRC bytes.
    check(diff <= 4, "and differ only in the DI, the checksum and the CRC");
    check(fixed_request_index(DI_FIXED_F101) == 0 && fixed_request_index(DI_FIXED_F102) == 1,
          "fixed_request_index() finds both");
    check(fixed_request_index(DI_LIST_B_RECORDS) == FIXED_REQUEST_COUNT, "and rejects a list DI");
    check(list_request_index(DI_FIXED_F102) == LIST_REQUEST_COUNT, "list_request_index() rejects a fixed DI");
    check(build_request(buf, sizeof(buf), serial, 0xF1FF) == 0, "an unknown DI still refuses to build");
  }

  std::printf("\n== the map from F102 fields onto TAGs ==\n");
  {
    const FixedValue *m = nullptr;
    check(f102_value_map(sizeof(f102_3ph), &m) == F102_3PH_VALUE_COUNT && m == F102_3PH_MAP,
          "63 bytes of DATA selects the three-phase map");
    check(f102_value_map(sizeof(f102_1ph), &m) == F102_1PH_VALUE_COUNT && m == F102_1PH_MAP,
          "23 bytes selects the single-phase map");
    // Same length-is-everything rule as the parser: a length that is neither
    // layout has to map to nothing rather than to the nearest layout.
    bool refused = true;
    for (uint8_t len : {(uint8_t) 0, (uint8_t) 22, (uint8_t) 24, (uint8_t) 62, (uint8_t) 64}) {
      const FixedValue *any = F102_3PH_MAP;
      if (f102_value_map(len, &any) != 0 || any != nullptr) {
        std::printf("      length %u mapped to something\n", len);
        refused = false;
      }
    }
    check(refused, "and every other length maps to nothing, nullptr included");

    // Each entry must name a TAG the decoder knows at the width the field is, or
    // there would be nothing to publish the value as.
    bool shape = true;
    for (uint8_t i = 0; i < F102_3PH_VALUE_COUNT; i++) {
      TagInfo info{};
      if (!tag_info(F102_3PH_MAP[i].tag, &info) || info.width != sizeof(bcd32_le)) {
        std::printf("      TAG 0x%02X has no 4-byte entry\n", F102_3PH_MAP[i].tag);
        shape = false;
      }
    }
    for (uint8_t i = 0; i < F102_1PH_VALUE_COUNT; i++) {
      TagInfo info{};
      if (!tag_info(F102_1PH_MAP[i].tag, &info) || info.width != sizeof(bcd32_le)) {
        std::printf("      1ph TAG 0x%02X has no 4-byte entry\n", F102_1PH_MAP[i].tag);
        shape = false;
      }
    }
    check(shape, "every mapped TAG exists in TAG_TABLE as a 4-byte value");

    // A TAG twice in one map would mean two fields racing for one entity, with
    // the winner decided by table order - so it is worth ruling out rather than
    // reading off by eye.
    bool unique = true;
    for (uint8_t i = 0; i < F102_3PH_VALUE_COUNT; i++) {
      for (uint8_t j = (uint8_t) (i + 1); j < F102_3PH_VALUE_COUNT; j++) {
        if (F102_3PH_MAP[i].tag == F102_3PH_MAP[j].tag) {
          std::printf("      TAG 0x%02X appears twice\n", F102_3PH_MAP[i].tag);
          unique = false;
        }
        if (F102_3PH_MAP[i].offset == F102_3PH_MAP[j].offset) {
          std::printf("      offset %u appears twice\n", F102_3PH_MAP[i].offset);
          unique = false;
        }
      }
    }
    check(unique, "no TAG and no offset is mapped twice");

    /* The offsets are the map's grip on the layout, so they are checked against
     * the struct itself rather than restated. Every field of the three-phase
     * block is mapped, in struct order.
     */
    const size_t want[F102_3PH_VALUE_COUNT] = {
        offsetof(f102_3ph, p_total), offsetof(f102_3ph, p_l1), offsetof(f102_3ph, p_l2),
        offsetof(f102_3ph, p_l3),    offsetof(f102_3ph, q_total), offsetof(f102_3ph, q_l1),
        offsetof(f102_3ph, q_l2),    offsetof(f102_3ph, q_l3), offsetof(f102_3ph, u_l1),
        offsetof(f102_3ph, u_l2),    offsetof(f102_3ph, u_l3), offsetof(f102_3ph, i_l1),
        offsetof(f102_3ph, i_l2),    offsetof(f102_3ph, i_l3), offsetof(f102_3ph, freq),
    };
    bool offs = true;
    for (uint8_t i = 0; i < F102_3PH_VALUE_COUNT; i++) {
      if (F102_3PH_MAP[i].offset != want[i]) {
        std::printf("      entry %u: offset %u, struct says %u\n", i, F102_3PH_MAP[i].offset, (unsigned) want[i]);
        offs = false;
      }
    }
    check(offs, "all 15 offsets are the struct's own, so every field is mapped");

    // The current group is mapped by object, not by the vendor's label - which is
    // why 0x1C is in and 0x1F is out. Stated as a check because it is a decision
    // rather than a transcription, and a future edit should have to face it.
    bool cur = true;
    for (uint8_t i = 0; i < F102_3PH_VALUE_COUNT; i++) {
      if (F102_3PH_MAP[i].tag == 0x1F) {
        cur = false;
      }
    }
    check(cur && F102_3PH_MAP[11].tag == 0x1C && F102_3PH_MAP[12].tag == 0x1D && F102_3PH_MAP[13].tag == 0x1E,
          "the currents map to TAGs 0x1C/0x1D/0x1E and nothing maps to 0x1F");

    // Encodings: every DI 0xF102 field is BCD, and only the power group is signed.
    // A voltage or a frequency with the top bit set is a bad reading, not a
    // negative volt, so treating that bit as a sign there would turn a fault into
    // a plausible number.
    bool encs = true;
    for (uint8_t i = 0; i < F102_3PH_VALUE_COUNT; i++) {
      const bool signed_group = F102_3PH_MAP[i].tag >= 0x20 && F102_3PH_MAP[i].tag <= 0x27;
      const TagEnc want = signed_group ? TagEnc::BCD_LE_SIGNED : TagEnc::BCD_LE;
      if (F102_3PH_MAP[i].enc != want) {
        std::printf("      TAG 0x%02X: wrong encoding\n", F102_3PH_MAP[i].tag);
        encs = false;
      }
    }
    check(encs, "every F102 field is BCD, and only the power group is signed");
  }

  std::printf("\n== the capture, read through the map ==\n");
  {
    /* The point of the scales: these are the numbers an entity is handed, in the
     * unit tag_info() names for the TAG - not the raw counts. So this is the same
     * capture as the first section, read the way the component reads it.
     */
    struct Want {
      uint8_t tag;
      float value;
      const char *unit;
    };
    const Want wants[F102_3PH_VALUE_COUNT] = {
        {0x20, 1465.0f, "W"},   {0x21, 336.0f, "W"},    {0x22, 244.0f, "W"},
        {0x23, 881.0f, "W"},    {0x24, -118.0f, "var"}, {0x25, -131.0f, "var"},
        {0x26, -9.0f, "var"},   {0x27, 22.0f, "var"},   {0x15, 234.81f, "V"},
        {0x16, 236.43f, "V"},   {0x17, 234.46f, "V"},   {0x1C, 1.59f, "A"},
        {0x1D, 1.28f, "A"},     {0x1E, 3.87f, "A"},     {0x28, 49.98f, "Hz"},
    };
    bool all_ok = true;
    for (uint8_t i = 0; i < F102_3PH_VALUE_COUNT; i++) {
      float got = 0.0f;
      TagInfo info{};
      const bool ok = fixed_value(r.payload, r.payload_len, F102_3PH_MAP[i], &got) &&
                      tag_info(F102_3PH_MAP[i].tag, &info);
      // A hundredth is the finest raw step any of these has, so anything closer
      // would be testing float arithmetic rather than the map.
      const float err = (got > wants[i].value) ? (got - wants[i].value) : (wants[i].value - got);
      if (!ok || F102_3PH_MAP[i].tag != wants[i].tag || err > 0.005f ||
          std::strcmp(info.unit, wants[i].unit) != 0) {
        std::printf("      TAG 0x%02X: got %.3f %s, want %.3f %s\n", F102_3PH_MAP[i].tag, got,
                    ok ? info.unit : "?", wants[i].value, wants[i].unit);
        all_ok = false;
      }
    }
    check(all_ok, "all 15 scale into the unit their TAG names");

    // The two sources have to meet in that unit, which is the whole reason the
    // scales are on the entries. TAG 0x15 arrived in both: 2348 raw counts in the
    // list at x0.1 V against 23481 here at x0.01 V.
    ParsedItem list_u{};
    list_u.tag = 0x15;
    list_u.len = 4;
    list_u.raw[0] = 0x48;  // BCD, least-significant pair first: 2348
    list_u.raw[1] = 0x23;
    TagInfo u_info{};
    float from_list = 0.0f;
    float from_fixed = 0.0f;
    check(tag_info(0x15, &u_info) && item_as_scaled(list_u, u_info, &from_list) &&
              fixed_value(r.payload, r.payload_len, F102_3PH_MAP[8], &from_fixed),
          "TAG 0x15 decodes from a list record and from the fixed block");
    const float gap = (from_list > from_fixed) ? (from_list - from_fixed) : (from_fixed - from_list);
    std::printf("      list %.2f V, F102 %.2f V\n", from_list, from_fixed);
    check(gap < 0.5f, "and the two land within half a volt of each other - one unit, two paths");

    // Refusals. A field the meter left unset reads back as a plausible number if
    // the nibbles are not checked, and an entry that reaches past the block would
    // read whatever follows it in memory.
    uint8_t bad[sizeof(f102_3ph)];
    std::memcpy(bad, r.payload, sizeof(bad));
    bad[F102_3PH_MAP[8].offset] = 0xEE;
    float sink = 0.0f;
    check(!fixed_value(bad, sizeof(bad), F102_3PH_MAP[8], &sink), "a non-BCD nibble is refused, not read");
    FixedValue past = F102_3PH_MAP[14];
    past.offset = (uint8_t) (sizeof(f102_3ph) - 3);
    check(!fixed_value(r.payload, r.payload_len, past, &sink), "an entry reaching past the block is refused");
    check(!fixed_value(r.payload, sizeof(f102_1ph), F102_3PH_MAP[14], &sink),
          "and so is the three-phase map read against a single-phase length");
  }

  std::printf("\n== item_as_scaled ==\n");
  {
    // The list's half of the same job. The clock is the one TAG with no scalar
    // reading, and it has to say so rather than return a number, because a
    // seven-byte date read as an integer would publish happily.
    ParsedItem clock{};
    clock.tag = 0x29;
    clock.len = 7;
    TagInfo info{};
    float sink = 0.0f;
    check(tag_info(0x29, &info) && !item_as_scaled(clock, info, &sink), "a clock has no scaled reading");

    // Signed BCD keeps its sign through the scale - the export direction on the
    // power group depends on it.
    ParsedItem q{};
    q.tag = 0x24;
    q.len = 4;
    q.raw[0] = 0x80;  // -118.0 var: 1180 counts with the sign bit set
    q.raw[1] = 0x11;
    q.raw[3] = 0x80;
    float value = 0.0f;
    check(tag_info(0x24, &info) && item_as_scaled(q, info, &value) && value < -1179.0f && value > -1181.0f,
          "a negative reactive power stays negative through the scale");
  }

  std::printf("\n%s (%d failure(s))\n", fail == 0 ? "DI 0xF101/0xF102 FIXED BLOCKS DECODE AND MAP CORRECTLY" : "FAILURES",
              fail);
  return fail == 0 ? 0 : 1;
}
