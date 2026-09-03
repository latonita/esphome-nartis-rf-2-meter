/*
 * Nartis RF-2 (Д101-2) frame layer - protocol only, no component state.
 *
 * This is the display link: the same 443 MHz CMT2300A PHY the DLMS-HDLC bridge
 * uses, but carrying a DL/T 645-1997 frame directly instead of a type-0x5A
 * envelope. Nothing is encrypted, there is no session, no password and no
 * sequence number - every request frame is constant for a given meter.
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

#include "nartis_dlt645_f1xx.h"

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
/* The meter holds two pre-defined indication lists, and each list is read with
 * two requests - one for the tagged values, one for the untagged status data that
 * follows them:
 *
 *                 tagged records    status block
 *      list A       DI 0xF200         DI 0xF201
 *      list B       DI 0xF202         DI 0xF203
 *
 * Every request body is constant and selects nothing; which values a list holds
 * is configured per meter with the vendor tool, and there is no way to ask for
 * one value in particular.
 *
 * The two halves are framed differently, which is why PayloadShape exists:
 *
 *   records half   DI(2) | COUNT | {TAG,value}...
 *   status half    DI(2) | {TAG,value}... | STATUS_BLOCK_SIZE bytes
 *
 * The records half announces the whole list in COUNT and then sends only what
 * fits one DL/T 645 frame - a list of 24 comes back as 16. The status half has no
 * COUNT of its own: it carries whatever records were left over, then the status
 * block, which is ALWAYS present. That makes the split deterministic - peel the
 * last STATUS_BLOCK_SIZE bytes off as the block and everything between the DI and
 * there is records - and it is why a reply with no leftovers is exactly
 * 2 + STATUS_BLOCK_SIZE bytes of payload.
 *
 * ORDERING: the leftover records come from a cursor the meter keeps, and it does
 * not survive an intervening request. A status half must follow its own records
 * half back to back or it answers with the block alone - seen both ways on a live
 * meter, which is why LIST_REQUESTS keeps each pair adjacent.
 *
 * The lists overlap: list B has been seen carrying everything list A does plus
 * the instantaneous values, making list A redundant on that meter. Which list
 * holds what is a per-meter setting, so both are read and their records merged.
 */
static constexpr uint16_t DI_LIST_A_RECORDS = 0xF200;
static constexpr uint16_t DI_LIST_A_STATUS = 0xF201;
static constexpr uint16_t DI_LIST_B_RECORDS = 0xF202;
static constexpr uint16_t DI_LIST_B_STATUS = 0xF203;

enum class ListId : uint8_t { A, B };
static constexpr uint8_t LIST_COUNT = 2;
/// "A" or "B", for the log.
const char *list_id_to_string(ListId l);

enum class ListPart : uint8_t {
  /// The list's tagged values, as many as one frame holds, behind a COUNT.
  RECORDS,
  /// The records that did not fit, then the status block. No COUNT byte.
  STATUS,
};

/// The two request bodies. Both are constant in every captured request - there is
/// no parameter selector in either - and which request takes which is observed
/// rather than derived, so it lives in LIST_REQUESTS below. Note that three of
/// the four take the long body and DI 0xF201 alone takes the short one; that
/// asymmetry is what the display does, not a rule anyone has explained.
extern const uint8_t REQUEST_BODY_LONG[6];
extern const uint8_t REQUEST_BODY_SHORT[1];
/// Longest request body a probe may carry.
static constexpr size_t MAX_REQUEST_BODY = 8;

struct ListRequest {
  uint16_t di;
  ListId list;
  ListPart part;
  const uint8_t *body;
  uint8_t body_len;
};

/// All four list requests, in the order they go on air. List B leads because it
/// is the superset, so where the lists overlap its records are the ones that land
/// in the merged set first; each list's two halves stay together.
///
/// Being in this table means the request CAN be sent, not that it is: which
/// sources a cycle actually polls is a YAML setting. The table stays whole so the
/// parser can recognise a reply to any of them.
static constexpr uint8_t LIST_REQUEST_COUNT = 4;
extern const ListRequest LIST_REQUESTS[LIST_REQUEST_COUNT];

/// Index of `di` in LIST_REQUESTS, or LIST_REQUEST_COUNT when it is not one of
/// the four - which is how a fixed block's or a probe's data identifier reads.
uint8_t list_request_index(uint16_t di);

/* ================================================================
 * The fixed blocks: DI 0xF101 and DI 0xF102
 * ================================================================
 *
 * A third source, and a different animal from the lists. No COUNT, no TAGs, no
 * cursor: each answers with one struct of positional values whose length is
 * fixed, so a value is identified by where it sits. nartis_dlt645_f1xx.h holds
 * those layouts and is the single source of truth for their sizes.
 *
 *   DI 0xF101   energy accumulators - two groups of [total, T1..T8] - then the
 *               10-byte status block. One layout on both meter types.
 *   DI 0xF102   live P/Q/U/I/frequency. TWO layouts, and the length is what tells
 *               them apart: 63 bytes of DATA is the three-phase block of 15
 *               values, 23 bytes the single-phase block of 5.
 *
 * Length is therefore load-bearing rather than a sanity check, which is the same
 * exact-fit principle the two list framings are told apart by.
 */
static constexpr uint16_t DI_FIXED_F101 = 0xF101;
static constexpr uint16_t DI_FIXED_F102 = 0xF102;

struct FixedRequest {
  uint16_t di;
  const uint8_t *body;
  uint8_t body_len;
};

/// Both fixed reads, in air order. They carry no cursor, so unlike the list
/// halves they can be sent in any order and at any point in a cycle.
static constexpr uint8_t FIXED_REQUEST_COUNT = 2;
extern const FixedRequest FIXED_REQUESTS[FIXED_REQUEST_COUNT];

/// Index of `di` in FIXED_REQUESTS, or FIXED_REQUEST_COUNT when it is neither.
uint8_t fixed_request_index(uint16_t di);

/// Widest item value in TAG_TABLE is the 7-byte clock; the extra room is for a
/// YAML-declared width.
static constexpr size_t MAX_ITEM_WIDTH = 9;
/// Items in ONE response. Sized from the payload rather than from what a list
/// happens to hold: the smallest possible record is 3 bytes (TAG plus a 2-byte
/// value), so a MAX_PAYLOAD-byte payload cannot hold more than
/// (MAX_PAYLOAD - 3) / 3 of them.
///
/// Per-response, not per-cycle: a list arrives across its two halves and the
/// component merges them itself.
static constexpr size_t MAX_ITEMS = 32;
/// Longest request we build (28 bytes on air with the long body).
static constexpr size_t MAX_REQUEST_FRAME_SIZE = 40;

/// The status block that ends every status-half reply.
///
/// Device state and alarm flags, not measurements: the firmware builds it from
/// its alarm/status bitmap plus a few internal objects. Byte 0 is a constant
/// marker, which is what makes an all-status reply recognisable at a glance.
///
///    0  constant 0x01 marker
///    1  status group
///    2  status bitfield
///    3  status bitfield
///    4  0x80 when a flag is set, else 0
///    5  0x08 when a flag is set, else 0
///    6  config / identity
///    7  config value
///    8  status bit
///    9  relay / breaker state
///   10  a further object, present only in this variant of the block
///
/// The same block exists 10 bytes long elsewhere in the firmware; the status
/// halves send the 11-byte variant, which is the only one this component sees.
static constexpr size_t STATUS_BLOCK_SIZE = 11;

/* The bytes the `status:` entities read.
 *
 * These were inferred from one capture before the firmware's own layout was
 * known, and the two do not agree: the table above calls byte 9 the relay/breaker
 * state and byte 10 an internal object, neither of which is a tariff. The offsets
 * are kept pointing at the same physical bytes so that no already-configured
 * entity silently changes value, but treat the NAMES as unconfirmed and prefer
 * `status: raw` when reporting what a meter sends.
 *
 * Temperature is not in here at all - it is TAG 0x2A in a list's records.
 */
static constexpr size_t STATUS_OFF_TARIFF_COUNT = 9;    // firmware: relay/breaker state
static constexpr size_t STATUS_OFF_ACTIVE_TARIFF = 10;  // firmware: an internal object

/// How the DATA field of a response is framed. Which one to expect follows from
/// the request - see ListPart - but the reading is verified rather than assumed:
/// a page stops on a record boundary, so the correct reading is the one that
/// consumes DATA exactly, and parse_response() takes the first candidate that
/// does. A meter framing a reply the other way round is then identified instead
/// of being read as garbage.
enum class PayloadShape : uint8_t {
  /// DI | COUNT | {TAG,value}... - a records half. COUNT is how many records the
  /// list *has*, so it is an upper bound on how many this frame carries.
  RECORDS,
  /// DI | {TAG,value}... | status block - a status half. No COUNT byte, and the
  /// block is always there, so the records end STATUS_BLOCK_SIZE bytes before the
  /// end of DATA.
  STATUS_HALF,
  /// DI 0xF101: two energy groups then the 10-byte status block. `payload` holds
  /// it whole - see struct nartis_f101 - and `items` is empty, since there are no
  /// TAGs to walk.
  FIXED_F101,
  /// DI 0xF102, three-phase: marker then 15 BCD values. See struct f102_3ph.
  FIXED_F102_3PH,
  /// DI 0xF102, single-phase: lead byte then 5 BCD values. See struct f102_1ph.
  FIXED_F102_1PH,
};

const char *payload_shape_to_string(PayloadShape s);

/// How an item's value bytes should be read. UINT_LE / INT_LE cover every
/// scalar register; the byte count is TagInfo::width.
enum class TagEnc : uint8_t {
  UINT_LE,      // little-endian unsigned binary - the energy registers
  INT_LE,       // little-endian two's-complement signed binary
  BCD_LE,        // BCD, least-significant pair first: 41 23 -> 2341
  BCD_LE_SIGNED,  // the same, with BCD_SIGN_BIT of the top byte meaning negative
  BCD_CLOCK,      // 7 bytes BCD: ss mm hh dow DD MM YY
  USER,           // width declared in YAML; read as UINT_LE, unit unknown
};

/// Sign of a BCD_LE_SIGNED value: bit 7 of the most-significant byte, set to mean
/// negative, with the remaining digits carrying the magnitude. It overlaps the top
/// BCD digit, so the encoding cannot express a magnitude at or above 80 000 000
/// counts - far past anything a meter reports in these registers.
static constexpr uint8_t BCD_SIGN_BIT = 0x80;

/// Where a TAG's width and encoding came from. Only OBSERVED has been seen on
/// this radio link; the others can still be wrong.
enum class TagConfidence : uint8_t {
  /// Decoded from captured D101-2 traffic.
  OBSERVED,
  /// From the vendor's TAG table, but no frame carrying it has been captured.
  DOCUMENTED,
  /// The sources disagree about what this TAG holds. The width is agreed, so the
  /// record can still be walked past - it is the meaning that is unsettled.
  CONFLICTING,
};

/// One past the highest TAG the vendor table describes (0x4F), and so the size of
/// every per-TAG array here.
static constexpr size_t TAG_WIDTH_TABLE_SIZE = 0x50;

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

/// Look up an item TAG's width, encoding and scale. Returns false when the TAG is
/// unknown - the caller must then abort the parse, because without a width there
/// is no way to find where the next record starts.
///
/// The TAG numbering does not depend on which page carried the record: DI 0xF200,
/// DI 0xF202 and DI 0xF203 are one list read three ways, so one table serves all
/// of them. `tag_width_overrides` is an optional TAG_WIDTH_TABLE_SIZE array of
/// YAML-declared widths, consulted only for TAGs with no built-in entry, so a
/// stray entry cannot shadow a known width.
bool tag_info(uint8_t tag, TagInfo *out, const uint8_t *tag_width_overrides = nullptr);

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
  /// The COUNT byte as received: how many records the list holds in total. A
  /// records half sends only what fits one frame, so this routinely exceeds
  /// `count` - the difference is what the status half then brings. Zero on a
  /// status half, which has no COUNT byte of its own.
  uint8_t announced_count{0};
  /// Which reading of DATA produced `items`. Meaningful when parse_response()
  /// returned OK; on a failure it is the shape whose diagnostics are reported.
  PayloadShape shape{PayloadShape::RECORDS};
  ParsedItem items[MAX_ITEMS]{};

  /// The status block, when `shape` is STATUS_HALF - where it is always present.
  /// It is not a record and is deliberately not in `items`: it has no TAG, and
  /// the byte a TAG would occupy is its constant 0x01 marker.
  uint8_t status_block[STATUS_BLOCK_SIZE]{};
  bool has_status_block{false};

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


/* ================================================================
 * Build / parse
 * ================================================================ */

/// Build the complete on-air request frame for `di` - one of the four in
/// LIST_REQUESTS, whose body it uses. Returns the number of bytes
/// written, or 0 on error.
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

/// Same, for BCD_LE_SIGNED: BCD_SIGN_BIT of the top byte is stripped and applied
/// as the sign. Returns false if the remaining nibbles are not BCD digits.
bool item_as_bcd_signed(const ParsedItem &item, int32_t *out);

/// Format a BCD_CLOCK item as "YYYY-MM-DD HH:MM:SS". Returns false if the item
/// is not a 7-byte clock or the buffer is too small (needs 20 bytes).
bool item_clock_to_string(const ParsedItem &item, char *out, size_t cap);

}  // namespace esphome::nartis_rf_2_meter
