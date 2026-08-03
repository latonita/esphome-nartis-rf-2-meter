"""Nartis RF-2 meter - text sensor platform.

Same selection as the numeric platform, plus the two text-only values: item TAG
0x29 (the meter clock, formatted "YYYY-MM-DD HH:MM:SS") and `status: raw`, a hex
dump of the whole status block.
"""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import text_sensor

from . import (
    CONF_BYTES,
    CONF_NARTIS_RF_2_METER_ID,
    CONF_STATUS,
    CONF_TAG,
    STATUS_FIELDS_TEXT,
    TAG_MAX_WIDTH,
    NartisRf2MeterComponent,
    StatusField,
    validate_tag,
    validate_tag_entity,
)

DEPENDENCIES = ["nartis_rf_2_meter"]

CONFIG_SCHEMA = cv.All(
    text_sensor.text_sensor_schema().extend(
        {
            cv.GenerateID(CONF_NARTIS_RF_2_METER_ID): cv.use_id(
                NartisRf2MeterComponent
            ),
            cv.Optional(CONF_TAG): validate_tag,
            cv.Optional(CONF_STATUS): cv.enum(STATUS_FIELDS_TEXT, lower=True),
            cv.Optional(CONF_BYTES): cv.int_range(min=1, max=TAG_MAX_WIDTH),
        }
    ),
    cv.has_exactly_one_key(CONF_TAG, CONF_STATUS),
    validate_tag_entity,
)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_NARTIS_RF_2_METER_ID])
    var = await text_sensor.new_text_sensor(config)

    tag = config.get(CONF_TAG, 0)
    field = config.get(CONF_STATUS, StatusField.NONE)

    cg.add(parent.register_text_sensor(var, tag, field, config.get(CONF_BYTES, 0)))
