import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID

CODEOWNERS = ["@local"]
MULTI_CONF = False

timer_renderer_ns = cg.esphome_ns.namespace("timer_renderer")
TimerRendererComponent = timer_renderer_ns.class_(
    "TimerRendererComponent", cg.Component
)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(TimerRendererComponent),
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
