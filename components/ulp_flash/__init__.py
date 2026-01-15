from esphome import automation, pins
from esphome.automation import maybe_simple_id
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID, CONF_PIN

ulp_flash_ns = cg.esphome_ns.namespace("ulp_flash")
ULPFlash = ulp_flash_ns.class_("ULPFlash", cg.Component)
pulse_width_enum = ulp_flash_ns.enum("FlashPulseWidth", is_class=True)

PULSE_WIDTHS = {
    "narrow": pulse_width_enum.NARROW,
    "medium": pulse_width_enum.MEDIUM,
    "wide": pulse_width_enum.WIDE,
}

FLASH_ON = ulp_flash_ns.FLASH_ON
FLASH_OFF = ulp_flash_ns.FLASH_OFF

FLASH_STATES = {
    "on": FLASH_ON,
    "off": FLASH_OFF,
}

validate_flash_state = cv.enum(FLASH_STATES, lower=True)

# add init state
flash_init_state_enum = ulp_flash_ns.enum("FlashInitState", is_class=True)

INIT_STATES = {
    "on": flash_init_state_enum.FLASH_INIT_ON,
    "off": flash_init_state_enum.FLASH_INIT_OFF,
    "last": flash_init_state_enum.FLASH_INIT_LAST,
}

# actions
OnAction = ulp_flash_ns.class_("OnAction", automation.Action)
OffAction = ulp_flash_ns.class_("OffAction", automation.Action)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(ULPFlash),
        cv.Required(CONF_PIN): pins.internal_gpio_output_pin_schema,
        cv.Optional("interval", default="1s"): cv.positive_time_period_milliseconds,
        cv.Optional("pulse_width", default="narrow"): cv.enum(
            PULSE_WIDTHS,
            lower=True,
        ),
        cv.Optional("init_state", default="last"): cv.enum(
            INIT_STATES,
            lower=True,
        ),
    }
).extend(cv.COMPONENT_SCHEMA)

FLASH_ACTION_SCHEMA = maybe_simple_id(
    {
        cv.Required(CONF_ID): cv.use_id(ULPFlash),
    }
)


@automation.register_action("ulp_flash.on", OnAction, FLASH_ACTION_SCHEMA)
async def flash_on_to_code(config, action_id, template_arg, args):
    paren = await cg.get_variable(config[CONF_ID])
    return cg.new_Pvariable(action_id, template_arg, paren)


@automation.register_action("ulp_flash.off", OffAction, FLASH_ACTION_SCHEMA)
async def flash_off_to_code(config, action_id, template_arg, args):
    paren = await cg.get_variable(config[CONF_ID])
    return cg.new_Pvariable(action_id, template_arg, paren)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    pin = await cg.gpio_pin_expression(config[CONF_PIN])
    cg.add(var.set_pin(pin))
    cg.add(var.set_interval(config["interval"]))
    cg.add(var.set_pulse_width(config["pulse_width"]))
    cg.add(var.set_init_state(config["init_state"]))
