import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import ble_client, time
from esphome.const import CONF_ID, CONF_BRIGHTNESS

CODEOWNERS = ["@your-github-handle"]
DEPENDENCIES = ["ble_client", "esp32"]
AUTO_LOAD = []

act1026_ns = cg.esphome_ns.namespace("act1026")
ACT1026 = act1026_ns.class_("ACT1026", cg.Component, ble_client.BLEClientNode)

CONF_TIME_ID = "time_id"
CONF_CLOCK_STYLE = "clock_style"
CONF_AUTO_SYNC_TIME = "auto_sync_time"

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(ACT1026),
            cv.Optional(CONF_TIME_ID): cv.use_id(time.RealTimeClock),
            cv.Optional(CONF_BRIGHTNESS, default=70): cv.int_range(min=0, max=100),
            cv.Optional(CONF_CLOCK_STYLE, default=1): cv.int_range(min=0, max=8),
            cv.Optional(CONF_AUTO_SYNC_TIME, default=True): cv.boolean,
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
    cg.add(var.set_auto_sync_time(config[CONF_AUTO_SYNC_TIME]))
