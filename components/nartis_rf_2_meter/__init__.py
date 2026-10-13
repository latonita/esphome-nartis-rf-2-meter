"""Nartis RF-2 meter - ESPHome external component.

Reads Nartis И100/И300/И500 meters (2024+, the "RF-433-2" / Д101-2 protocol) over
a 443 MHz 2-FSK radio by emulating the НАРТИС-Д101-2 display. DL/T 645-1997
inside a radio envelope: no encryption, no session, no password.

The radio is a replaceable backend, picked with the `radio:` block:

    cmt2300a   CMOSTEK CMT2300A on a bit-banged 3-wire SPI. The original, proven
               against live meters.
    cc1101     TI CC1101 on a 4-wire SPI. Same PHY, different silicon; new and
               not yet confirmed on hardware, and ~10 dB down on TX power.

The meter answers six constant requests, one per `sources:` entry (default
`list_2`):

    list_1   DI 0xF200/0xF201   records + status half
    list_2   DI 0xF202/0xF203   records + status half
    fix_1    DI 0xF101          reactive energy, see fixed.md
    fix_2    DI 0xF102          live P/Q/U/I/frequency, see fixed.md

A status half must follow its own records half back to back: the leftover records
come from a cursor the meter drops as soon as anything else is asked.

An entity selects a value by its 1-byte item TAG, or by a status-block field. Every
reply is folded into one value per TAG, the list winning where both sources carry
one, and published already scaled to the unit tags.md gives - so a `multiply`
filter would scale it twice. A TAG read with `bytes:` is the exception: unknown
unit, published raw.
"""

import logging

from esphome import pins
import esphome.codegen as cg

# The dotted form on purpose: this package has its own sensor.py, and a plain
# `from esphome.components import sensor` resolves to that one instead.
from esphome.components.sensor import new_sensor, sensor_schema
import esphome.config_validation as cv
from esphome.const import (
    CONF_ADDRESS,
    CONF_FREQUENCY,
    CONF_ID,
    CONF_TYPE,
    DEVICE_CLASS_SIGNAL_STRENGTH,
    ENTITY_CATEGORY_DIAGNOSTIC,
    STATE_CLASS_MEASUREMENT,
    UNIT_DECIBEL_MILLIWATT,
)
from esphome.types import ConfigType

_LOGGER = logging.getLogger(__name__)

CODEOWNERS = ["@latonita"]
# binary_sensor is auto-loaded even though the platform is optional: the component
# header includes its type unconditionally.
AUTO_LOAD = ["binary_sensor", "sensor", "text_sensor"]
MULTI_CONF = True

CONF_RADIO = "radio"

RADIO_TYPE_CMT2300A = "cmt2300a"
RADIO_TYPE_CC1101 = "cc1101"

# CMT2300A. These five names are also the pre-`radio:` flat keys, kept working by
# migrate_legacy_radio() below.
CONF_PIN_SDIO = "pin_sdio"
CONF_PIN_SCLK = "pin_sclk"
CONF_PIN_CSB = "pin_csb"
CONF_PIN_FCSB = "pin_fcsb"
CONF_PIN_GPIO3 = "pin_gpio3"
# The legacy spelling of the RX centre offset, in CMT2300A frequency codes. Still
# accepted; `rx_offset` below, in Hz, is the preferred one.
CONF_RX_CENTER_OFFSET = "rx_center_offset"

# CC1101. pin_sclk is shared with the CMT2300A spelling on purpose - it is the
# same wire on the same bus, whichever chip is on the far end.
CONF_PIN_MISO = "pin_miso"
CONF_PIN_MOSI = "pin_mosi"
CONF_PIN_CS = "pin_cs"
CONF_PIN_GDO0 = "pin_gdo0"
# Shared with the cmt2300a block on purpose: "shift the receive centre away from
# the channel centre" is the same user-facing idea on either chip, so it gets the
# same key and the same unit (signed Hz) whichever silicon is on the far end.
CONF_RX_OFFSET = "rx_offset"
CONF_OUTPUT_POWER = "output_power"
CONF_BANDWIDTH = "bandwidth"
CONF_DEVIATION = "deviation"
# Whether the CC1101's frequency-offset and bit-sync loops are gated on carrier
# sense (FOCCFG 0x36) or run freely (0x16). A bring-up knob; see the long note at
# the schema entry for why neither value is obviously right here.
CONF_FOC_CARRIER_SENSE_GATE = "foc_carrier_sense_gate"

CONF_REQUEST_GAP = "request_gap"
CONF_RF_RX_TIMEOUT = "rf_rx_timeout"
CONF_RF_RETRIES = "rf_retries"

CONF_SOURCES = "sources"
CONF_RSSI = "rssi"

CONF_PROBE = "probe"
CONF_DI = "di"
CONF_BODY = "body"

CONF_NARTIS_RF_2_METER_ID = "nartis_rf_2_meter_id"
CONF_TAG = "tag"
CONF_STATUS = "status"
CONF_BYTES = "bytes"

nartis_rf_2_meter_ns = cg.esphome_ns.namespace("nartis_rf_2_meter")
NartisRf2MeterComponent = nartis_rf_2_meter_ns.class_(
    "NartisRf2MeterComponent", cg.PollingComponent
)
RadioHal = nartis_rf_2_meter_ns.class_("RadioHal")
Cmt2300aHal = nartis_rf_2_meter_ns.class_("Cmt2300aHal", RadioHal)
Cc1101Hal = nartis_rf_2_meter_ns.class_("Cc1101Hal", RadioHal)

StatusField = nartis_rf_2_meter_ns.enum("StatusField", is_class=True)

# UNCONFIRMED: inferred from byte positions in one capture, where the firmware layout
# calls the same bytes device state. Real temperature is `tag: 0x2A`.
STATUS_FIELDS = {
    "active_tariff": StatusField.STATUS_FIELD_ACTIVE_TARIFF,
    "tariff_count": StatusField.STATUS_FIELD_TARIFF_COUNT,
}
STATUS_FIELDS_TEXT = {
    **STATUS_FIELDS,
    "raw": StatusField.STATUS_FIELD_RAW,
}

# Mirrors TAG_TABLE in d101_frame.cpp, the one place widths, encodings and scales live.
TAG_CLOCK = 0x29
TAG_NO_WIDTH = {0x2B} | set(range(0x40, 0x50))
TAG_KNOWN = set(range(0x50)) - TAG_NO_WIDTH
TAG_NUMERIC = TAG_KNOWN - {TAG_CLOCK}

TAG_MIN = 0x00
TAG_MAX = 0x4F
TAG_MAX_WIDTH = 9  # MAX_ITEM_WIDTH in d101_frame.h


def validate_tag(value):
    tag = cv.hex_int(value)
    if not TAG_MIN <= tag <= TAG_MAX:
        raise cv.Invalid(
            f"tag must be between 0x{TAG_MIN:02X} and 0x{TAG_MAX:02X}; got 0x{tag:02X}"
        )
    return tag


def validate_numeric_tag(value):
    tag = validate_tag(value)
    if tag == TAG_CLOCK:
        raise cv.Invalid(
            f"tag 0x{TAG_CLOCK:02X} is the meter clock - use a text_sensor for it"
        )
    return tag


def validate_tag_entity(config: ConfigType) -> ConfigType:
    tag = config.get(CONF_TAG)
    if tag is None or tag in TAG_KNOWN or CONF_BYTES in config:
        return config
    raise cv.Invalid(
        f"tag 0x{tag:02X} has no known value width, so it cannot be decoded. Add "
        f"`bytes:` with the width you observed in the log (the component prints "
        f"the raw payload whenever it meets an unrecognised TAG). Note that "
        f"declaring a TAG does not make the meter send it: the request is a fixed "
        f"frame and the meter replies with its own configured indication set.",
        path=[CONF_TAG],
    )


# A list costs two exchanges, a fixed block one, ~1 s apiece. Which list holds what
# is set per meter with the vendor tool; list 2 was a superset of list 1 on the
# reference meter, hence the default.
SOURCE_LIST_1 = "list_1"
SOURCE_LIST_2 = "list_2"
SOURCE_FIX_1 = "fix_1"
SOURCE_FIX_2 = "fix_2"
SOURCES = [SOURCE_LIST_1, SOURCE_LIST_2, SOURCE_FIX_1, SOURCE_FIX_2]


def validate_sources(value: list[str]) -> list[str]:
    seen = []
    for item in value:
        if item in seen:
            raise cv.Invalid(f"source '{item}' is listed twice")
        seen.append(item)
    if not seen:
        raise cv.Invalid(
            f"at least one source is required; choose from {', '.join(SOURCES)}"
        )
    return seen


# Body length is per-DI, not fixed, so the body of an unknown DI has to be guessed.
KNOWN_DI = {0xF200: 6, 0xF201: 1, 0xF202: 6, 0xF203: 6, 0xF101: 6, 0xF102: 6}

MAX_REQUEST_BODY = 8

PROBE_SCHEMA = cv.Schema(
    {
        cv.Required(CONF_DI): cv.hex_int_range(min=0, max=0xFFFF),
        # Defaults to a single 0x00, the shape DI 0xF201 uses.
        cv.Optional(CONF_BODY, default=[0x00]): cv.All(
            cv.ensure_list(cv.hex_uint8_t), cv.Length(min=0, max=MAX_REQUEST_BODY)
        ),
    }
)


def validate_probes(value):
    for probe in value:
        di = probe[CONF_DI]
        known = KNOWN_DI.get(di)
        if known is not None and len(probe[CONF_BODY]) != known:
            raise cv.Invalid(
                f"DI 0x{di:04X} is polled normally with a {known}-byte body; a probe "
                f"with a {len(probe[CONF_BODY])}-byte body would ask the same DI a "
                f"second time with a different body. Remove it, or probe a different DI.",
                path=[CONF_BODY],
            )
    return value


def validate_address(value) -> str:
    s = cv.string_strict(value)
    if not s.isdigit() or len(s) != 12:
        raise cv.Invalid(
            f"address must be exactly 12 digits (the meter serial, e.g. "
            f"'023240123456'); got {len(s)} characters '{s}'"
        )
    return s


# --------------------------------------------------------------------------- #
# radio: - the replaceable RF backend
# --------------------------------------------------------------------------- #

# The CC1101 PA table for the 433 MHz band stops at +10 dBm (the +12 dBm entry in
# the datasheet is a 868/915 MHz figure). A genuine Nartis display runs +20 dBm, so
# the uplink is roughly 10 dB down no matter what is asked for here; the cure is a
# module with an external PA, not a larger number.
CC1101_POWER_MIN_DBM = -30
CC1101_POWER_MAX_DBM = 10

# Channel filter range the CC1101 can actually synthesise from its 26 MHz crystal:
# 26e6 / (8 * (4 + CHANBW_M) * 2^CHANBW_E), i.e. 58.0 kHz .. 812.5 kHz.
CC1101_BW_MIN_HZ = 58000
CC1101_BW_MAX_HZ = 812000

# The PHY is +-25 kHz at 1200 bps. Carson gives ~51 kHz of occupied bandwidth, and
# the meter's crystal plus ours drift on top of that, so anything much under the
# chip's 58 kHz floor would clip the reply - which is why the floor is also the
# practical minimum. 25 kHz is the one correct value for `deviation`; the range
# exists only so a different Nartis variant could be tried.
DEVIATION_MIN_HZ = 1000
DEVIATION_MAX_HZ = 200000

# The meter answers a few kHz above where we transmitted. Beyond a couple of
# hundred kHz the request would no longer be the same channel at all.
RX_OFFSET_MAX_HZ = 200000

# The CMT2300A spells the same shift in frequency codes rather than Hz. This is
# RX_CODE_HZ from cmt2300a_defs.h; nothing here converts with it - Hz goes to the
# C++ as Hz and Cmt2300aHal::set_rx_offset_hz() rounds it to codes - it only lets
# the messages below quote both units of the one proven number.
CMT2300A_RX_CODE_HZ = 6.199
CMT2300A_OFFSET_CODE_MIN = -4000
CMT2300A_OFFSET_CODE_MAX = 4000
# The hardware-proven centre offset, and the C++ default when neither key is given.
# The two spellings agree exactly: 758 codes is 4698.8 Hz, which rounds to 4699 Hz,
# and 4699 Hz rounds back to 758 codes.
CMT2300A_PROVEN_OFFSET_CODES = 758
CMT2300A_PROVEN_OFFSET_HZ = 4699


def validate_output_power(value):
    """TX power in dBm, an integer. The CC1101 cannot be pushed past ~+10 dBm here."""
    dbm = cv.int_(value)
    if not CC1101_POWER_MIN_DBM <= dbm <= CC1101_POWER_MAX_DBM:
        raise cv.Invalid(
            f"output_power is in dBm and the CC1101 PA table for the 433 MHz band "
            f"runs from {CC1101_POWER_MIN_DBM} to +{CC1101_POWER_MAX_DBM} dBm; got "
            f"{dbm}. The chip cannot exceed about +10 dBm in this band - a real "
            f"Nartis display transmits +20 dBm, so if the meter does not hear us the "
            f"answer is a CC1101 module with an external PA, not a bigger number."
        )
    return dbm


def _frequency_hz(value):
    """`cv.frequency` with the result pinned to a whole number of Hz.

    `cv.frequency` lets an empty mantissa through its regex and then trips over
    `float("")`, so the ValueError is turned into a sentence here.
    """
    try:
        return int(round(cv.frequency(value)))
    except ValueError as err:
        raise cv.Invalid(
            f"expected a frequency, with or without a unit ('25kHz', '81250Hz', "
            f"'58000'); got '{value}'"
        ) from err


def validate_rx_bandwidth(value):
    """RX channel filter width. Accepts `81250Hz`, `81.25kHz`, or a bare number of Hz."""
    hz = _frequency_hz(value)
    if not CC1101_BW_MIN_HZ <= hz <= CC1101_BW_MAX_HZ:
        raise cv.Invalid(
            f"bandwidth must be between {CC1101_BW_MIN_HZ} Hz and {CC1101_BW_MAX_HZ} "
            f"Hz; got {hz} Hz. That is the whole range the CC1101 channel filter can "
            f"synthesise, and the low end is already tight: the link is +-25 kHz at "
            f"1200 bps, which occupies ~51 kHz before either crystal has drifted, so "
            f"roughly 58 kHz is the narrowest filter that still passes the reply. "
            f"The default 81250 Hz leaves room for the offset the meter answers on."
        )
    return hz


def validate_deviation(value):
    """FSK deviation. 25 kHz is the protocol's value; anything else is an experiment."""
    hz = _frequency_hz(value)
    if not DEVIATION_MIN_HZ <= hz <= DEVIATION_MAX_HZ:
        raise cv.Invalid(
            f"deviation must be between {DEVIATION_MIN_HZ} Hz and {DEVIATION_MAX_HZ} "
            f"Hz; got {hz} Hz. The RF433-2 PHY is +-25 kHz, so `25kHz` is the value "
            f"that works - change it only to chase a meter that provably does not "
            f"follow the spec."
        )
    return hz


def validate_rx_offset(value):
    """Signed RX offset in Hz.

    `cv.frequency` has no notion of a negative frequency, so the sign is peeled off
    here and reapplied: `-3kHz`, `+0Hz`, `2500` and `0` all work, and a bare number
    is read as Hz.
    """
    if isinstance(value, str):
        text = value.strip()
        sign = 1
        if text[:1] in ("+", "-"):
            sign = -1 if text[0] == "-" else 1
            text = text[1:].lstrip()
        try:
            hz = int(round(sign * cv.frequency(text)))
        except ValueError as err:
            # cv.frequency's regex matches an empty mantissa, e.g. a bare "-".
            raise cv.Invalid(
                f"rx_offset must be a signed number of Hz, optionally with a unit "
                f"('-3kHz', '0Hz', '2500'); got '{value}'"
            ) from err
    else:
        hz = int(round(cv.float_(value)))
    if abs(hz) > RX_OFFSET_MAX_HZ:
        raise cv.Invalid(
            f"rx_offset is a signed number of Hz and must be within "
            f"+-{RX_OFFSET_MAX_HZ} Hz; got {hz} Hz. It nudges the receiver onto the "
            f"carrier the meter actually answers on, which sits a few kHz above our "
            f"transmission - a correction of that size, not a channel change."
        )
    return hz


def validate_cmt2300a_offset(config: ConfigType) -> ConfigType:
    """Allow at most one spelling of the RX centre offset.

    `rx_offset` (Hz) and `rx_center_offset` (CMT2300A frequency codes) set the same
    field through two different setters, so a config giving both would get whichever
    one the codegen happened to emit last - a silent surprise rather than an error.
    """
    if CONF_RX_OFFSET in config and CONF_RX_CENTER_OFFSET in config:
        raise cv.Invalid(
            f"`{CONF_RX_OFFSET}` and `{CONF_RX_CENTER_OFFSET}` are the same knob - how "
            f"far the receiver sits from the channel centre - written in two different "
            f"units, so only one of them can be given. Keep `{CONF_RX_OFFSET}`: it is "
            f"signed Hz, it is the preferred spelling, and `type: {RADIO_TYPE_CC1101}` "
            f"uses the same key for the same thing. `{CONF_RX_CENTER_OFFSET}` is the "
            f"legacy spelling, in CMT2300A frequency codes of ~{CMT2300A_RX_CODE_HZ} Hz "
            f"each; the proven {CMT2300A_PROVEN_OFFSET_CODES} codes are exactly "
            f"`{CONF_RX_OFFSET}: {CMT2300A_PROVEN_OFFSET_HZ}Hz`.",
            path=[CONF_RX_CENTER_OFFSET],
        )
    return config


CMT2300A_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(Cmt2300aHal),
            cv.Required(CONF_PIN_SDIO): pins.internal_gpio_output_pin_schema,
            cv.Required(CONF_PIN_SCLK): pins.internal_gpio_output_pin_schema,
            cv.Required(CONF_PIN_CSB): pins.internal_gpio_output_pin_schema,
            cv.Required(CONF_PIN_FCSB): pins.internal_gpio_output_pin_schema,
            cv.Required(CONF_PIN_GPIO3): pins.internal_gpio_input_pin_schema,
            # Where the RX-half LO sits relative to the channel centre, as signed Hz
            # - the preferred spelling, and the same key and unit `type: cc1101`
            # takes. Deliberately without a schema default: left out entirely it
            # emits nothing and the C++ default (758 codes ~= 4699 Hz, proven on
            # hardware) stands, which is also what lets the legacy key below keep
            # working without the two fighting.
            cv.Optional(CONF_RX_OFFSET): validate_rx_offset,
            # The legacy spelling of the line above, in CMT2300A frequency codes
            # (1 code ~= 6.199 Hz). Accepted unchanged so that configs tuned against
            # a live meter keep their proven number; new ones should say `rx_offset`.
            cv.Optional(CONF_RX_CENTER_OFFSET): cv.int_range(
                min=CMT2300A_OFFSET_CODE_MIN, max=CMT2300A_OFFSET_CODE_MAX
            ),
        }
    ),
    validate_cmt2300a_offset,
)

CC1101_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(Cc1101Hal),
        cv.Required(CONF_PIN_SCLK): pins.internal_gpio_output_pin_schema,
        cv.Required(CONF_PIN_MISO): pins.internal_gpio_input_pin_schema,
        cv.Required(CONF_PIN_MOSI): pins.internal_gpio_output_pin_schema,
        cv.Required(CONF_PIN_CS): pins.internal_gpio_output_pin_schema,
        # Optional: the backend falls back to polling the status byte over SPI when
        # GDO0 is not wired. Leaving it out costs latency, not function.
        cv.Optional(CONF_PIN_GDO0): pins.internal_gpio_input_pin_schema,
        cv.Optional(CONF_RX_OFFSET, default=0): validate_rx_offset,
        cv.Optional(CONF_OUTPUT_POWER, default=10): validate_output_power,
        cv.Optional(CONF_BANDWIDTH, default="81250Hz"): validate_rx_bandwidth,
        cv.Optional(CONF_DEVIATION, default="25kHz"): validate_deviation,
        # Gate the frequency-offset compensation and bit-synchronisation loops on
        # carrier sense. `false` (FOCCFG = 0x16) lets both loops track all the time;
        # `true` (FOCCFG = 0x36) freezes them until carrier sense asserts.
        #
        # Neither is safe on this waveform, and the datasheet cannot settle it: the
        # link is 2-FSK at 1200 baud with +-25 kHz deviation, a modulation index of
        # about 41, which is nothing like the near-h=1 signals those loops were
        # designed around. Left ungated, the loops spend the long wait for a reply
        # tracking noise and can wander to their limits before the preamble ever
        # arrives; gated, a weak preamble may never trip carrier sense and the reply
        # is dropped whole. The default is `false` because that matches the C++
        # default and the free-running case at least cannot silence a reply outright.
        #
        # This is a hardware-tuning knob, so decide it by measurement: count
        # CRC-valid replies with each value over many cycles. Bytes landing in the
        # FIFO are not the test - a slipped loop still fills the FIFO, with garbage.
        cv.Optional(CONF_FOC_CARRIER_SENSE_GATE, default=False): cv.boolean,
    }
)

RADIO_SCHEMA = cv.typed_schema(
    {
        RADIO_TYPE_CMT2300A: CMT2300A_SCHEMA,
        RADIO_TYPE_CC1101: CC1101_SCHEMA,
    },
    key=CONF_TYPE,
    default_type=RADIO_TYPE_CMT2300A,
    lower=True,
)


# The flat keys this component used before the radio became selectable. All five
# pins were required and `rx_center_offset` was optional, so the whole set folds
# into exactly one `radio:` block of type cmt2300a.
LEGACY_PIN_KEYS = (
    CONF_PIN_SDIO,
    CONF_PIN_SCLK,
    CONF_PIN_CSB,
    CONF_PIN_FCSB,
    CONF_PIN_GPIO3,
)
LEGACY_KEYS = LEGACY_PIN_KEYS + (CONF_RX_CENTER_OFFSET,)


def migrate_legacy_radio(config):
    """Fold the pre-`radio:` flat pin keys into an implicit cmt2300a radio block.

    Runs before the schema, so by the time anything else looks at the config there
    is exactly one way to spell a radio. Non-dict input is handed straight through
    for the schema to reject with its usual message.
    """
    if not isinstance(config, dict):
        return config

    legacy = [key for key in LEGACY_KEYS if key in config]
    has_radio = CONF_RADIO in config

    if legacy and has_radio:
        raise cv.Invalid(
            f"`radio:` and the old flat radio keys ({', '.join(legacy)}) are two "
            f"spellings of the same thing and cannot be combined. Delete the flat "
            f"keys - everything they said now lives inside `radio:`."
        )

    if not legacy:
        if not has_radio:
            raise cv.Invalid(
                f"`radio:` is required. The component no longer owns the radio pins; "
                f"it is handed a radio object. Add a block with "
                f"`type: {RADIO_TYPE_CMT2300A}` (the CMT2300A, proven on hardware) or "
                f"`type: {RADIO_TYPE_CC1101}`, and put the pins inside it."
            )
        return config

    missing = [key for key in LEGACY_PIN_KEYS if key not in config]
    if missing:
        raise cv.Invalid(
            f"the flat CMT2300A pin keys are the pre-`radio:` spelling and all five "
            f"have to be given together; {', '.join(missing)} "
            f"{'is' if len(missing) == 1 else 'are'} missing. Rather than completing "
            f"the set, move the pins into a `radio:` block with "
            f"`type: {RADIO_TYPE_CMT2300A}` - the flat form is deprecated."
        )

    config = config.copy()
    radio = {CONF_TYPE: RADIO_TYPE_CMT2300A}
    for key in LEGACY_KEYS:
        if key in config:
            radio[key] = config.pop(key)
    config[CONF_RADIO] = radio

    _LOGGER.warning(
        "nartis_rf_2_meter: the flat radio keys (%s) are deprecated and will be "
        "removed in a future release. They have been read as a `radio:` block with "
        "`type: %s`. Please rewrite them as:\n"
        "  nartis_rf_2_meter:\n"
        "    radio:\n"
        "      type: %s\n"
        "%s",
        ", ".join(legacy),
        RADIO_TYPE_CMT2300A,
        RADIO_TYPE_CMT2300A,
        "".join(f"      {key}: ...\n" for key in legacy),
    )
    return config


CONFIG_SCHEMA = cv.All(
    migrate_legacy_radio,
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(NartisRf2MeterComponent),
            cv.Required(CONF_RADIO): RADIO_SCHEMA,
            cv.Required(CONF_ADDRESS): validate_address,
            cv.Optional(CONF_FREQUENCY): cv.All(
                cv.frequency, cv.Range(min=430000000, max=460000000)
            ),
            cv.Optional(
                CONF_REQUEST_GAP, default="300ms"
            ): cv.positive_time_period_milliseconds,
            cv.Optional(
                CONF_RF_RX_TIMEOUT, default="1800ms"
            ): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_RF_RETRIES, default=2): cv.int_range(min=0, max=10),
            cv.Optional(CONF_SOURCES, default=[SOURCE_LIST_2]): cv.All(
                cv.ensure_list(cv.one_of(*SOURCES, lower=True)), validate_sources
            ),
            # Extra reads once per cycle, logged in full and driving no entity.
            cv.Optional(CONF_PROBE): cv.All(
                cv.ensure_list(PROBE_SCHEMA), cv.Length(min=1, max=8), validate_probes
            ),
            cv.Optional(CONF_RSSI): sensor_schema(
                unit_of_measurement=UNIT_DECIBEL_MILLIWATT,
                accuracy_decimals=0,
                device_class=DEVICE_CLASS_SIGNAL_STRENGTH,
                state_class=STATE_CLASS_MEASUREMENT,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
        }
    ).extend(cv.polling_component_schema("300s")),
)


async def radio_to_code(radio_config):
    """Build the RadioHal subclass this config asks for and return the pointer.

    Tunables that carry a schema default matching the C++ default are emitted
    unconditionally - the generated code then says plainly what the radio is running
    with, instead of leaving it implicit in the header. The CMT2300A RX centre offset
    is the one exception: it has two spellings sharing one field, so it has no schema
    default and emits nothing when neither spelling is given.
    """
    hal = cg.new_Pvariable(radio_config[CONF_ID])

    if radio_config[CONF_TYPE] == RADIO_TYPE_CMT2300A:
        pin_exprs = [
            await cg.gpio_pin_expression(radio_config[key])
            for key in (
                CONF_PIN_SDIO,
                CONF_PIN_SCLK,
                CONF_PIN_CSB,
                CONF_PIN_FCSB,
                CONF_PIN_GPIO3,
            )
        ]
        cg.add(hal.set_pins(*pin_exprs))
        # One field, two setters. validate_cmt2300a_offset() has already ruled out
        # both keys at once, so at most one branch can fire; with neither key given
        # nothing is emitted and the C++ default (758 codes ~= 4699 Hz) stands.
        if (rx_offset_hz := radio_config.get(CONF_RX_OFFSET)) is not None:
            cg.add(hal.set_rx_offset_hz(rx_offset_hz))
        elif (offset_codes := radio_config.get(CONF_RX_CENTER_OFFSET)) is not None:
            cg.add(hal.set_rx_center_offset(offset_codes))
    else:
        pin_exprs = [
            await cg.gpio_pin_expression(radio_config[key])
            for key in (CONF_PIN_SCLK, CONF_PIN_MISO, CONF_PIN_MOSI, CONF_PIN_CS)
        ]
        # GDO0 is optional; the backend takes nullptr to mean "poll the status byte".
        if (gdo0 := radio_config.get(CONF_PIN_GDO0)) is not None:
            pin_exprs.append(await cg.gpio_pin_expression(gdo0))
        else:
            pin_exprs.append(cg.nullptr)
        cg.add(hal.set_pins(*pin_exprs))
        cg.add(hal.set_rx_offset_hz(radio_config[CONF_RX_OFFSET]))
        cg.add(hal.set_output_power_dbm(radio_config[CONF_OUTPUT_POWER]))
        cg.add(hal.set_rx_bandwidth_hz(radio_config[CONF_BANDWIDTH]))
        cg.add(hal.set_deviation_hz(radio_config[CONF_DEVIATION]))
        cg.add(hal.set_foc_cs_gate(radio_config[CONF_FOC_CARRIER_SENSE_GATE]))

    return hal


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    cg.add(var.set_radio(await radio_to_code(config[CONF_RADIO])))

    cg.add(var.set_address(config[CONF_ADDRESS]))
    if (frequency := config.get(CONF_FREQUENCY)) is not None:
        cg.add(var.set_frequency_override(int(frequency)))

    cg.add(var.set_request_gap_ms(config[CONF_REQUEST_GAP]))
    cg.add(var.set_rf_rx_timeout_ms(config[CONF_RF_RX_TIMEOUT]))
    cg.add(var.set_rf_retries(config[CONF_RF_RETRIES]))

    sources = config[CONF_SOURCES]
    cg.add(
        var.set_sources(
            SOURCE_LIST_1 in sources,
            SOURCE_LIST_2 in sources,
            SOURCE_FIX_1 in sources,
            SOURCE_FIX_2 in sources,
        )
    )

    if (rssi := config.get(CONF_RSSI)) is not None:
        cg.add(var.set_rssi_sensor(await new_sensor(rssi)))

    for probe in config.get(CONF_PROBE, []):
        cg.add(var.add_probe(probe[CONF_DI], probe[CONF_BODY]))
