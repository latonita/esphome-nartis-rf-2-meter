"""Nartis RF-2 meter - binary sensor platform.

One diagnostic entity: whether the last poll cycle got everything it asked for.

This link is not always up - reception on the reference install is close to 100%
overnight and can sit at zero for hours in the middle of the day - and the numeric
entities cannot show that. They deliberately publish nothing on a failed cycle and
hold their previous state, so a stale reading is indistinguishable from a fresh one
unless you watch the entity's last-updated time. This is the entity that says so
outright.

It also gives the template sensors something to gate on: a lambda that returns `{}`
while this reads false stops publishing confidently-timestamped stale values.
"""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor
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
