/*
 * Nartis RF-2 (Д101-2) frame layer - protocol only, no component state.
 *
 * The display link: a DL/T 645-1997 frame inside a radio envelope. Nothing is
 * encrypted, there is no session, password or sequence number - every request
 * frame is constant for a given meter.
 *
 *   radio envelope   98 F3 | LEN | 00 01 | HLEN | .................... | CRC16
 *                                                ┌──────────────────┐
 *   DL/T 645-1997                                │ 68 A×6 68 C L D.. CS 16
 *                                                          ┌───────┐
 *   application payload (every byte +0x33)                  │ DI CNT items...
 *
 *   LEN  = byte count after LEN, excluding the CRC (= 3 + len(645))
 *   HLEN = LEN ^ 1  (ignored on receive)
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
/// Transmission offset on every DATA byte, so payload cannot imitate a delimiter.
static constexpr uint8_t DLT645_DATA_OFFSET = 0x33;
/// 68 + addr(6) + 68 + C + L + CS + 16 - everything except DATA.
static constexpr size_t DLT645_OVERHEAD = 12;

static constexpr size_t SERIAL_BCD_SIZE = 6;

/* ================================================================
 * Application layer
 * ================================================================ */
/* Two pre-defined indication lists, each read with two requests:
 *
 *                 tagged records    status block
 *      list A       DI 0xF200         DI 0xF201
 *      list B       DI 0xF202         DI 0xF203
 *
 * Request bodies are constant and select nothing; which values a list holds is a
 * per-meter vendor-tool setting, so both lists are read and their records merged.
 *
 *   records half   DI(2) | COUNT | {TAG,value}...
 *   status half    DI(2) | {TAG,value}... | STATUS_BLOCK_SIZE bytes
 *
 * COUNT announces the whole list; the records half sends only what fits one frame
 * (a list of 24 comes back as 16) and the status half carries the leftovers then
 * the status block, which is always present.
 *
 * ORDERING: the leftovers come from a cursor that does not survive an intervening
 * request. A status half must follow its own records half back to back, or it
 * answers with the block alone.
 */
static constexpr uint16_t DI_LIST_A_RECORDS = 0xF200;
static constexpr uint16_t DI_LIST_A_STATUS = 0xF201;
static constexpr uint16_t DI_LIST_B_RECORDS = 0xF202;
static constexpr uint16_t DI_LIST_B_STATUS = 0xF203;

enum class ListId : uint8_t { A, B };
static constexpr uint8_t LIST_COUNT = 2;
const char *list_id_to_string(ListId l);

enum class ListPart : uint8_t {
  RECORDS,  // tagged values behind a COUNT
  STATUS,   // leftover records, then the status block; no COUNT
};

/// Constant in every captured request. Three of the four requests take the long
/// body, DI 0xF201 the short one.
extern const uint8_t REQUEST_BODY_LONG[6];
extern const uint8_t REQUEST_BODY_SHORT[1];
static constexpr size_t MAX_REQUEST_BODY = 8;

struct ListRequest {
  uint16_t di;
  ListId list;
  ListPart part;
  const uint8_t *body;
  uint8_t body_len;
};

/// All four list requests in air order, each list's halves adjacent (see
/// ORDERING). Being here means a request can be sent, not that a cycle sends it.
static constexpr uint8_t LIST_REQUEST_COUNT = 4;
extern const ListRequest LIST_REQUESTS[LIST_REQUEST_COUNT];

/// Index of `di` in LIST_REQUESTS, or LIST_REQUEST_COUNT when it is not one.
uint8_t list_request_index(uint16_t di);

/* ================================================================
 * The fixed blocks: DI 0xF101 and DI 0xF102
 * ================================================================
 *
 * No COUNT, no TAGs, no cursor: one struct of positional values of fixed length.
 * nartis_dlt645_f1xx.h holds the layouts.
 *
 *   DI 0xF101   energy accumulators - two groups of [total, T1..T8] - then the
 *               10-byte status block. One layout on both meter types.
 *   DI 0xF102   live P/Q/U/I/frequency. The length picks the layout: 63 bytes of
 *               DATA is the three-phase block of 15 values, 23 the single-phase
 *               block of 5.
 */
static constexpr uint16_t DI_FIXED_F101 = 0xF101;
static constexpr uint16_t DI_FIXED_F102 = 0xF102;

struct FixedRequest {
  uint16_t di;
  const uint8_t *body;
  uint8_t body_len;
};

/// Both fixed reads. They carry no cursor, so order does not matter.
static constexpr uint8_t FIXED_REQUEST_COUNT = 2;
extern const FixedRequest FIXED_REQUESTS[FIXED_REQUEST_COUNT];

/// Index of `di` in FIXED_REQUESTS, or FIXED_REQUEST_COUNT when it is neither.
uint8_t fixed_request_index(uint16_t di);

/// Widest TAG_TABLE value is the 7-byte clock; the rest is room for a YAML width.
static constexpr size_t MAX_ITEM_WIDTH = 9;
/// Items in ONE response. The smallest record is 3 bytes.
static constexpr size_t MAX_ITEMS = 32;
/// Longest request we build (28 bytes on air with the long body).
static constexpr size_t MAX_REQUEST_FRAME_SIZE = 40;

/// The status block that ends every status-half reply - device state, not values.
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
static constexpr size_t STATUS_BLOCK_SIZE = 11;

// The bytes the `status:` entities read. Inferred from one capture and not confirmed
// by the layout above - prefer `status: raw`.
static constexpr size_t STATUS_OFF_TARIFF_COUNT = 9;    // firmware: relay/breaker state
static constexpr size_t STATUS_OFF_ACTIVE_TARIFF = 10;  // firmware: an internal object

/// How the DATA field is framed. The right shape is the one that consumes DATA exactly.
enum class PayloadShape : uint8_t {
  RECORDS,         // DI | COUNT | {TAG,value}...
  STATUS_HALF,     // DI | {TAG,value}... | status block; no COUNT
  FIXED_F101,      // struct nartis_f101; no TAGs, so `items` stays empty
  FIXED_F102_3PH,  // struct f102_3ph: marker then 15 BCD values
  FIXED_F102_1PH,  // struct f102_1ph: lead byte then 5 BCD values
};

const char *payload_shape_to_string(PayloadShape s);

/// How an item's value bytes are read; the byte count is TagInfo::width.
enum class TagEnc : uint8_t {
  UINT_LE,      // little-endian unsigned binary - the energy registers
  INT_LE,       // little-endian two's-complement signed binary
  BCD_LE,        // BCD, least-significant pair first: 41 23 -> 2341
  BCD_LE_SIGNED,  // the same, with BCD_SIGN_BIT of the top byte meaning negative
  BCD_CLOCK,      // 7 bytes BCD: ss mm hh dow DD MM YY
  USER,           // width declared in YAML; read as UINT_LE, unit unknown
};

/// BCD_LE_SIGNED sign bit: bit 7 of the top byte, overlapping its top BCD digit.
static constexpr uint8_t BCD_SIGN_BIT = 0x80;

/// One past the highest TAG the vendor table describes (0x4F).
static constexpr size_t TAG_WIDTH_TABLE_SIZE = 0x50;

struct TagInfo {
  uint8_t width;
  TagEnc enc;
  /// Raw counts -> `unit`. This is what an entity's published value is scaled by.
  float scale;
  const char *unit;
};

/// Look up a TAG's width, encoding and scale. False for an unknown TAG - the caller
/// must then abort the parse. `tag_width_overrides` is consulted only for TAGs with no
/// built-in entry.
bool tag_info(uint8_t tag, TagInfo *out, const uint8_t *tag_width_overrides = nullptr);

/* A fixed block's field, named by the TAG of the meter object it holds and carrying
 * this block's own scale, so either source lands in the TAG's one unit.
 */
struct FixedValue {
  uint8_t tag;
  /// Byte offset of the 4-byte field within the DATA block, DI echo included.
  uint8_t offset;
  /// BCD_LE / BCD_LE_SIGNED for DI 0xF102, UINT_LE for DI 0xF101's accumulators.
  TagEnc enc;
  /// Raw counts -> the unit tag_info() gives this TAG.
  float scale;
};

static constexpr uint8_t F102_3PH_VALUE_COUNT = 15;
static constexpr uint8_t F102_1PH_VALUE_COUNT = 2;
static constexpr uint8_t F101_VALUE_COUNT = 10;
extern const FixedValue F102_3PH_MAP[F102_3PH_VALUE_COUNT];
extern const FixedValue F102_1PH_MAP[F102_1PH_VALUE_COUNT];
extern const FixedValue F101_MAP[F101_VALUE_COUNT];

/// Mapping for whichever DI 0xF102 layout `payload_len` identifies, in struct order.
uint8_t f102_value_map(uint8_t payload_len, const FixedValue **out);

/// Read one mapped value out of a stored fixed block, scaled into its TAG's unit.
bool fixed_value(const uint8_t *payload, uint8_t payload_len, const FixedValue &v, float *out);

struct ParsedItem {
  uint8_t tag{0};
  uint8_t len{0};
  uint8_t raw[MAX_ITEM_WIDTH]{};
};

/// De-offset payload capacity. The largest page seen is the 65-byte DI 0xF202 one.
static constexpr size_t MAX_PAYLOAD = 128;

struct ParsedResponse {
  /// 0x81 = read response; anything else with bit 7 set is a refusal, and `payload`
  /// then holds the error bytes.
  uint8_t control{0};
  uint16_t di{0};
  uint8_t count{0};
  /// COUNT as received: records the list holds in total, so routinely more than
  /// `count`. Zero on a status half.
  uint8_t announced_count{0};
  PayloadShape shape{PayloadShape::RECORDS};
  ParsedItem items[MAX_ITEMS]{};

  /// The status block, when `shape` is STATUS_HALF. Not a record - it has no TAG.
  uint8_t status_block[STATUS_BLOCK_SIZE]{};
  bool has_status_block{false};

  /// Payload with the +0x33 offset removed, kept whenever the layers below verified.
  uint8_t payload[MAX_PAYLOAD]{};
  uint8_t payload_len{0};
  /// Valid on UNKNOWN_TAG: the TAG that stopped the parse, and its payload offset.
  uint8_t unknown_tag{0};
  uint8_t unknown_offset{0};

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

/// Hint for a DL/T 645 error byte. Only 0x02 is established for this meter, and
/// empirically; the other names follow the standard and are unverified.
const char *dlt645_error_hint(uint8_t err);

/* ================================================================
 * Primitives
 * ================================================================ */

/// CRC-16/X.25: poly 0x1021 reflected (0x8408), init/xorout 0xFFFF, reflected in
/// and out.
uint16_t crc16_x25(const uint8_t *data, size_t len);

/// 12 ASCII digits -> 6 BCD bytes -> reversed, as the 645 address field wants it:
/// "023240271060" -> 60 10 27 40 32 02. Zeroes if `digits12` is not 12 digits.
void serial_to_bcd_le(const char *digits12, uint8_t out[SERIAL_BCD_SIZE]);

/* ================================================================
 * Build / parse
 * ================================================================ */

/// Build the on-air request frame for `di`, one of the four in LIST_REQUESTS. Returns
/// bytes written, or 0 on error.
size_t build_request(uint8_t *out, size_t cap, const uint8_t serial_le[SERIAL_BCD_SIZE], uint16_t di);

/// Same, with an explicit request body. The control code is always DLT645_C_READ_REQ:
/// this link also carries a relay command, so a write must never be constructible.
size_t build_read_request(uint8_t *out, size_t cap, const uint8_t serial_le[SERIAL_BCD_SIZE], uint16_t di,
                          const uint8_t *body, size_t body_len);

/// Carve, verify and decode one response. The chip strips 98 F3, so buf[0] should
/// be LEN, and fixed-length RX leaves capture noise after the frame: start offsets
/// 0..3 are tried, and a candidate accepted only when its LEN and CRC agree.
ParseResult parse_response(const uint8_t *buf, size_t len, const uint8_t serial_le[SERIAL_BCD_SIZE],
                           ParsedResponse *out, const uint8_t *tag_width_overrides = nullptr);

/* ================================================================
 * Value decoders
 * ================================================================ */

/// Little-endian unsigned value of an item (up to 4 bytes). 0 if wider.
uint32_t item_as_u32(const ParsedItem &item);

/// Same, sign-extended from the item's own width. 0 if wider than 4 bytes.
int32_t item_as_i32(const ParsedItem &item);

/// BCD value of an item, least-significant pair first: 41 23 -> 2341. False on a
/// non-decimal nibble.
bool item_as_bcd(const ParsedItem &item, uint32_t *out);

/// Same, for BCD_LE_SIGNED: the top byte's sign bit is stripped and applied.
bool item_as_bcd_signed(const ParsedItem &item, int32_t *out);

/// Format a BCD_CLOCK item as "YYYY-MM-DD HH:MM:SS"; needs 20 bytes.
bool item_clock_to_string(const ParsedItem &item, char *out, size_t cap);

/// Decode a list record into the unit its TAG names, applying TagInfo::scale.
bool item_as_scaled(const ParsedItem &item, const TagInfo &info, float *out);

}  // namespace esphome::nartis_rf_2_meter
