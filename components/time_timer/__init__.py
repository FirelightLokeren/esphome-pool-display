import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import ble_client, time
from esphome.const import CONF_ID, CONF_BRIGHTNESS

CODEOWNERS = []
DEPENDENCIES = ["ble_client", "esp32"]

time_timer_ns = cg.esphome_ns.namespace("time_timer")
TimeTimer = time_timer_ns.class_("TimeTimer", cg.Component, ble_client.BLEClientNode)

CONF_TIME_ID     = "time_id"
CONF_CLOCK_STYLE = "clock_style"

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(TimeTimer),
            cv.Optional(CONF_TIME_ID): cv.use_id(time.RealTimeClock),
            cv.Optional(CONF_BRIGHTNESS, default=70): cv.int_range(min=0, max=100),
            cv.Optional(CONF_CLOCK_STYLE, default=1): cv.int_range(min=0, max=8),
        }
    )
    .extend(ble_client.BLE_CLIENT_SCHEMA)
    .extend(cv.COMPONENT_SCHEMA)
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await ble_client.register_ble_node(var, config)

    if CONF_TIME_ID in config:
        time_var = await cg.get_variable(config[CONF_TIME_ID])
        cg.add(var.set_time_source(time_var))

    cg.add(var.set_brightness(config[CONF_BRIGHTNESS]))
    cg.add(var.set_clock_style(config[CONF_CLOCK_STYLE]))
