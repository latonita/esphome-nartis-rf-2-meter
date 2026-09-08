"""Nartis RF-2 meter - binary sensor platform.

One diagnostic entity: whether the last poll cycle got everything it asked for.

This link is not always up - reception on the reference install is close to 100%
overnight and can sit at zero for hours in the middle of the day. The value
entities publish nothing on a failed cycle and hold their previous state, so a
stale reading looks exactly like a fresh one; this entity is what says otherwise,
and what a template lambda can gate on to stop republishing stale values.
"""

import esphome.codegen as cg
from esphome.components import binary_sensor
import esphome.config_validation as cv
from esphome.const import DEVICE_CLASS_CONNECTIVITY, ENTITY_CATEGORY_DIAGNOSTIC

from . import CONF_NARTIS_RF_2_METER_ID, NartisRf2MeterComponent

DEPENDENCIES = ["nartis_rf_2_meter"]

CONFIG_SCHEMA = binary_sensor.binary_sensor_schema(
    device_class=DEVICE_CLASS_CONNECTIVITY,
    entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
).extend(
    {
        cv.GenerateID(CONF_NARTIS_RF_2_METER_ID): cv.use_id(NartisRf2MeterComponent),
    }
)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_NARTIS_RF_2_METER_ID])
    var = await binary_sensor.new_binary_sensor(config)
    cg.add(parent.set_last_read_ok_binary_sensor(var))
