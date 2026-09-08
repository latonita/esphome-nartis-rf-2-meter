"""Nartis RF-2 meter - ESPHome external component.

Reads Nartis И100/И300/И500 meters (2024+, the "RF-433-2" / Д101-2 protocol) over
a CMT2300A 443 MHz radio by emulating the НАРТИС-Д101-2 display. DL/T 645-1997
inside a radio envelope: no encryption, no session, no password.

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
    DEVICE_CLASS_SIGNAL_STRENGTH,
    ENTITY_CATEGORY_DIAGNOSTIC,
    STATE_CLASS_MEASUREMENT,
    UNIT_DECIBEL_MILLIWATT,
)
from esphome.types import ConfigType

CODEOWNERS = ["@latonita"]
# binary_sensor is auto-loaded even though the platform is optional: the component
# header includes its type unconditionally.
AUTO_LOAD = ["binary_sensor", "sensor", "text_sensor"]
MULTI_CONF = True

CONF_PIN_SDIO = "pin_sdio"
CONF_PIN_SCLK = "pin_sclk"
CONF_PIN_CSB = "pin_csb"
CONF_PIN_FCSB = "pin_fcsb"
CONF_PIN_GPIO3 = "pin_gpio3"

CONF_REQUEST_GAP = "request_gap"
CONF_RF_RX_TIMEOUT = "rf_rx_timeout"
CONF_RF_RETRIES = "rf_retries"
CONF_RX_CENTER_OFFSET = "rx_center_offset"

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


CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(NartisRf2MeterComponent),
        cv.Required(CONF_PIN_SDIO): pins.internal_gpio_output_pin_schema,
        cv.Required(CONF_PIN_SCLK): pins.internal_gpio_output_pin_schema,
        cv.Required(CONF_PIN_CSB): pins.internal_gpio_output_pin_schema,
        cv.Required(CONF_PIN_FCSB): pins.internal_gpio_output_pin_schema,
        cv.Required(CONF_PIN_GPIO3): pins.internal_gpio_input_pin_schema,
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
        # CMT2300A frequency codes (1 code ~= 6.199 Hz), shifting the RX-half LO onto
        # the meter's reply carrier. The default is proven on hardware.
        cv.Optional(CONF_RX_CENTER_OFFSET, default=758): cv.int_range(
            min=-4000, max=4000
        ),
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
).extend(cv.polling_component_schema("300s"))


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    for key, setter in (
        (CONF_PIN_SDIO, var.set_pin_sdio),
        (CONF_PIN_SCLK, var.set_pin_sclk),
        (CONF_PIN_CSB, var.set_pin_csb),
        (CONF_PIN_FCSB, var.set_pin_fcsb),
        (CONF_PIN_GPIO3, var.set_pin_gpio3),
    ):
        pin = await cg.gpio_pin_expression(config[key])
        cg.add(setter(pin))

    cg.add(var.set_address(config[CONF_ADDRESS]))
    if (frequency := config.get(CONF_FREQUENCY)) is not None:
        cg.add(var.set_frequency_override(int(frequency)))

    cg.add(var.set_request_gap_ms(config[CONF_REQUEST_GAP]))
    cg.add(var.set_rf_rx_timeout_ms(config[CONF_RF_RX_TIMEOUT]))
    cg.add(var.set_rf_retries(config[CONF_RF_RETRIES]))
    cg.add(var.set_rx_center_offset(config[CONF_RX_CENTER_OFFSET]))

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
