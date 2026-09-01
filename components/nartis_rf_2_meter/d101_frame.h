/*
 * Nartis RF-2 (Д101-2) frame layer - protocol only, no component state.
 *
 * This is the display link: the same 443 MHz CMT2300A PHY the DLMS-HDLC bridge
 * uses, but carrying a DL/T 645-1997 frame directly instead of a type-0x5A
 * envelope. Nothing is encrypted, there is no session, no password and no
 * sequence number - the two request frames are constant for a given meter.
 *
 * Three nested layers:
 *
 *   radio envelope   98 F3 | LEN | 00 01 | HLEN | .................... | CRC16
 *                                                ┌──────────────────┐
 *   DL/T 645-1997                                │ 68 A×6 68 C L D.. CS 16
 *                                                          ┌───────┐
 *   application payload (every byte +0x33)                  │ DI CNT items...
 *
 *   LEN  = byte count after LEN, excluding the CRC (= 3 + len(645))
 *   HLEN = LEN ^ 1  (see the note on build_request(); ignored on receive)
 *   CRC  = CRC-16/X.25 over [LEN .. last 645 byte], little-endian
 *   CS   = 8-bit sum over the 645 frame up to and including the last DATA byte
 *
 * Items carry no length field, so the parser must know each TAG's width up
 * front; an unknown TAG destroys framing for everything after it and MUST abort
 * the whole parse rather than skip ahead.
 */

#pragma once

#include <cstddef>
#include <cstdint>

namespace esphome::nartis_rf_2_meter {

/* ================================================================
 * Radio envelope
 * ================================================================ */
static constexpr uint8_t D101_SYNC0 = 0x98;
static constexpr uint8_t D101_SYNC1 = 0xF3;
/// Bytes of envelope header that sit between LEN and the 645 frame: 00 01 HLEN.
static constexpr size_t D101_HDR_AFTER_LEN = 3;

/* ================================================================
 * DL/T 645-1997 layer
 * ================================================================ */
static constexpr uint8_t DLT645_DELIM = 0x68;
static constexpr uint8_t DLT645_END = 0x16;
static constexpr uint8_t DLT645_C_READ_REQ = 0x01;  // display -> meter
static constexpr uint8_t DLT645_C_READ_RSP = 0x81;  // meter -> display
/// Every DATA byte carries this transmission offset so payload bytes cannot
/// imitate the 0x68 / 0x16 delimiters.
static constexpr uint8_t DLT645_DATA_OFFSET = 0x33;
/// 68 + addr(6) + 68 + C + L + CS + 16 - everything except DATA.
static constexpr size_t DLT645_OVERHEAD = 12;

static constexpr size_t SERIAL_BCD_SIZE = 6;

/* ================================================================
 * Application layer
 * ================================================================ */
static constexpr uint16_t DI_ENERGY = 0xF200;  // energy registers + clock
static constexpr uint16_t DI_STATUS = 0xF201;  // status block
/// Full parameter page: the energy registers AND the instantaneous values.
/// Takes the same 6-byte body as DI 0xF200 - with the 1-byte body of DI 0xF201 it
/// is silently dropped, no error response at all.
static constexpr uint16_t DI_PARAMS = 0xF202;

/// The request body that follows the DI. Its length is per-DI, not fixed: the
/// energy poll carries six bytes, the status poll one. Both are constant in every
/// captured request - there is no parameter selector in either.
extern const uint8_t ENERGY_REQUEST_BODY[6];
extern const uint8_t STATUS_REQUEST_BODY[1];
/// Longest request body a probe may carry.
static constexpr size_t MAX_REQUEST_BODY = 8;

/// Widest item value we can hold (the 9-byte status blob).
static constexpr size_t MAX_ITEM_WIDTH = 9;
/// Items in one response. The DI 0xF200 automatic cycle sends 4, but the DI
/// 0xF202 page sends 15, so this is sized from the payload instead of from what
/// a particular page happens to use: the smallest possible item is 3 bytes (TAG
/// plus a 2-byte value), so a MAX_PAYLOAD-byte payload cannot hold more than
/// (MAX_PAYLOAD - 3) / 3 of them.
static constexpr size_t MAX_ITEMS = 32;
/// Longest request we build (the energy poll is 28 bytes on air).
static constexpr size_t MAX_REQUEST_FRAME_SIZE = 40;

/// Layout of the 9-byte DI 0xF201 / TAG 0x00 status value.
/// Bytes 0..4 and 6 are constant across the capture and unexplained.
static constexpr size_t STATUS_VALUE_SIZE = 9;
static constexpr size_t STATUS_OFF_TEMPERATURE = 5;    // inferred, not confirmed
static constexpr size_t STATUS_OFF_TARIFF_COUNT = 7;   // inferred
static constexpr size_t STATUS_OFF_ACTIVE_TARIFF = 8;  // confirmed against the tariff schedule

/// How an item's value bytes should be read. UINT_LE / INT_LE cover every
/// scalar register; the byte count is TagInfo::width.
enum class TagEnc : uint8_t {
  UINT_LE,      // little-endian unsigned binary - the energy registers
  INT_LE,       // little-endian two's-complement signed binary
  BCD_LE,       // BCD, least-significant pair first: 41 23 -> 2341
  BCD_CLOCK,    // 7 bytes BCD: ss mm hh dow DD MM YY
  STATUS_BLOB,  // 9 raw bytes (DI 0xF201)
  USER,         // width declared in YAML; read as UINT_LE, unit unknown
};

/// Where a TAG's width and encoding came from. Nothing but OBSERVED has been
/// seen on this radio link, so the rest can still be wrong.
enum class TagConfidence : uint8_t {
  /// Decoded from captured D101-2 traffic.
  OBSERVED,
  /// Taken from the DLMS/COSEM data type the same meter reports for the same
  /// OBIS code over the DLMS-HDLC link. Validated by the energy registers,
  /// where DLMS says double-long-unsigned and D101-2 does send 4 bytes.
  DLMS_DERIVED,
  /// Extrapolated from a sibling register of the same kind.
  FAMILY_ASSUMED,
};

/// Per-TAG value widths supplied from YAML for TAGs the decoder does not know,
/// indexed by TAG (0x00..0x3F); 0 = no override. Lets a user who has learned a
/// width from a log dump decode that item without a firmware change. Overrides
/// only fill gaps - they never shadow a built-in width, so a stray entry cannot
/// break the confirmed energy or clock decoding.
static constexpr size_t TAG_WIDTH_TABLE_SIZE = 64;

struct TagInfo {
  uint8_t width;
  TagEnc enc;
  TagConfidence conf;
  /// Multiplier from raw units to `unit`, for the log line only - published
  /// values stay raw so the YAML decides the scaling with a `multiply` filter.
  float scale;
  const char *unit;
};

/// Human-readable provenance, for the log.
const char *tag_confidence_to_string(TagConfidence c);

/// Look up an item TAG's width and encoding for a given DI. Returns false when
/// the TAG is unknown - the caller must then abort the parse, because without a
/// width there is no way to find the next item.
bool tag_info(uint16_t di, uint8_t tag, TagInfo *out, const uint8_t *tag_width_overrides = nullptr);

/// True when a TAG can only come from the DI 0xF202 page: the instantaneous
/// values and temperature. The cumulative energy registers and the clock appear
/// on DI 0xF200 as well, so they do not force the larger page.
///
/// This is a statement about what the two pages *mean*, not about any particular
/// meter's configuration - which page actually carries a given TAG is set with the
/// vendor tool. A TAG missing from whichever page is polled just leaves its entity
/// unavailable and says so in the log.
bool tag_needs_params_page(uint8_t tag);

struct ParsedItem {
  uint8_t tag{0};
  uint8_t len{0};
  uint8_t raw[MAX_ITEM_WIDTH]{};
};

/// How much de-offset application payload we handle. DI 0xF200 uses 26 bytes and
/// the DI 0xF202 page 65; the manual-scroll set is still unobserved, so leave
/// real headroom. The DL/T 645 length field is one byte, so 255 is the absolute
/// ceiling - 128 covers roughly twice the largest page seen.
static constexpr size_t MAX_PAYLOAD = 128;

struct ParsedResponse {
  /// DL/T 645 control code as received. 0x81 = normal read response; anything
  /// else with bit 7 set is the meter refusing, and `payload` then holds the
  /// error bytes rather than items.
  uint8_t control{0};
  uint16_t di{0};
  /// Records actually decoded - the length of `items`.
  uint8_t count{0};
  /// The COUNT byte as received. Meters announce the full record set and then
  /// send only what fits the frame, so this can exceed `count`; a difference is
  /// normal truncation, not an error.
  uint8_t announced_count{0};
  ParsedItem items[MAX_ITEMS]{};

  /// The application payload with the +0x33 transmission offset removed, kept
  /// whenever the envelope and the DL/T 645 layer verified - including on an
  /// item-level failure. This is what you need in the log to work out an
  /// unrecognised TAG's width by hand.
  uint8_t payload[MAX_PAYLOAD]{};
  uint8_t payload_len{0};
  /// Valid when parse_response() returned UNKNOWN_TAG: the TAG that stopped the
  /// parse, and its offset within `payload`. `count` then holds how many items
  /// were decoded before it.
  uint8_t unknown_tag{0};
  uint8_t unknown_offset{0};

  /// Item with this TAG, or nullptr.
  const ParsedItem *find(uint8_t tag) const;
};

enum class ParseResult : uint8_t {
  OK,
  ERROR_RESPONSE,  // the meter refused the read (control code has bit 6 set)
  NO_FRAME,        // no length+CRC-consistent frame in the buffer
  BAD_CHECKSUM,    // 645 sum checksum mismatch
  MALFORMED,       // 645 delimiters / length fields inconsistent
  WRONG_ADDRESS,   // a valid frame, but not from our meter
  NOT_RESPONSE,    // control code is not 0x81 (0xC1 = the meter refused the read)
  UNKNOWN_TAG,     // item TAG of unknown width - framing lost, aborted
  TOO_MANY_ITEMS,  // more records present than MAX_ITEMS
};

const char *parse_result_to_string(ParseResult r);

/// Hint for a DL/T 645 error byte from an ERROR_RESPONSE.
///
/// Only 0x02 is established for this meter, and empirically rather than from the
/// standard: it is what came back for a data identifier the meter does not
/// implement (DI 0xF300). The other bit names follow the DL/T 645-1997 error
/// byte and are NOT verified here - treat them as a starting point, not fact.
///
/// The distinction that matters for probing is error-vs-silence: an error means
/// the meter parsed the request and declined, silence means it did not answer at
/// all, which is a different and more interesting failure.
const char *dlt645_error_hint(uint8_t err);

/* ================================================================
 * Primitives
 * ================================================================ */

/// CRC-16/X.25: poly 0x1021 reflected (0x8408), init/xorout 0xFFFF, reflected
/// in and out. Self-contained on purpose - the equivalent helper in the DLMS
/// bridge borrows reverse8() from the radio driver, which would make this
/// protocol layer depend on the HAL for a pure bit trick.
uint16_t crc16_x25(const uint8_t *data, size_t len);

/// 12 ASCII digits -> 6 BCD bytes -> reversed, as the 645 address field wants it.
/// "023240271060" -> BCD 02 32 40 27 10 60 -> out = 60 10 27 40 32 02.
/// Writes zeroes if `digits12` is not 12 digits long.
void serial_to_bcd_le(const char *digits12, uint8_t out[SERIAL_BCD_SIZE]);

// Meter channel grid: k = last3(serial) % 24, freq = BASE + k * STEP, plus EXTRA
// once k > BREAK. The k=18 -> 19 step is 800 kHz; every other step is 700 kHz.
// Kept here rather than in cmt2300a_defs.h so this protocol layer stays free of
// the HAL headers - the host-side frame checks build it on its own.
static constexpr uint32_t CHANNEL_BASE_HZ = 435500000u;
static constexpr uint32_t CHANNEL_STEP_HZ = 700000u;
static constexpr uint32_t CHANNEL_STEP_BREAK = 18u;
static constexpr uint32_t CHANNEL_STEP_EXTRA_HZ = 100000u;

/// Channel frequency (Hz) from the last three digits of the meter serial:
///   k = last3 % 24;  f = 435.5 MHz + k*0.7 MHz, +100 kHz when k > 18.
/// e.g. "...060" -> 60 % 24 = 12 -> 443.900 MHz; "...596" -> 20 -> 449.600 MHz.
uint32_t frequency_from_serial(const char *digits12);

/* ================================================================
 * Build / parse
 * ================================================================ */

/// Build the complete on-air request frame for `di` (DI_ENERGY or DI_STATUS).
/// Returns the number of bytes written, or 0 on error.
///
/// Header byte 5 is HLEN = LEN ^ 1. The two candidate rules, LEN ^ 1 and
/// LEN - 1, agree for odd LEN and differ for even, and every even-length frame
/// observed uses LEN ^ 1: the display's own status poll (LEN 0x12 -> 0x13), the
/// DI 0xF202 reply (0x60 -> 0x61), and two replies on the DLMS-HDLC link
/// (24 -> 25, 12 -> 13). No counterexample.
///
/// The field is ignored on receive - a meter answers a poll built with LEN - 1
/// just as readily - so this is about being byte-identical to the real display
/// rather than about working at all.
size_t build_request(uint8_t *out, size_t cap, const uint8_t serial_le[SERIAL_BCD_SIZE], uint16_t di);

/// Same, with an explicit request body - for asking a DI whose body shape is not
/// known. The control code is always DLT645_C_READ_REQ; there is deliberately no
/// way to build anything but a read, because this link also carries a
/// relay-close command and a write must never be constructible here.
size_t build_read_request(uint8_t *out, size_t cap, const uint8_t serial_le[SERIAL_BCD_SIZE], uint16_t di,
                          const uint8_t *body, size_t body_len);

/// Carve, verify and decode one response.
///
/// `buf` is what the radio handed back: the chip strips 98 F3, so buf[0] should
/// be LEN, and because RX runs in fixed-length mode the real frame is followed
/// by capture noise. Start offsets 0..3 are tried, and a candidate is accepted
/// only when its LEN and CRC agree. Header byte 5 is not checked.
ParseResult parse_response(const uint8_t *buf, size_t len, const uint8_t serial_le[SERIAL_BCD_SIZE],
                           ParsedResponse *out, const uint8_t *tag_width_overrides = nullptr);

/* ================================================================
 * Value decoders
 * ================================================================ */

/// Little-endian unsigned value of an item (up to 4 bytes). 0 if wider.
uint32_t item_as_u32(const ParsedItem &item);

/// Same, sign-extended from the item's own width. 0 if wider than 4 bytes.
int32_t item_as_i32(const ParsedItem &item);

/// BCD value of an item, least-significant pair first: 41 23 -> 2341.
/// Returns false if any nibble is not a decimal digit.
bool item_as_bcd(const ParsedItem &item, uint32_t *out);

/// Format a BCD_CLOCK item as "YYYY-MM-DD HH:MM:SS". Returns false if the item
/// is not a 7-byte clock or the buffer is too small (needs 20 bytes).
bool item_clock_to_string(const ParsedItem &item, char *out, size_t cap);

}  // namespace esphome::nartis_rf_2_meter
