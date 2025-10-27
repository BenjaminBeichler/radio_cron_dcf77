import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.components import time, switch
from esphome.const import CONF_ID, CONF_TIME_ID

import logging  # <- add this import

DEPENDENCIES = ["time"]
MULTI_CONF = True

dcf77_emitter_ns = cg.esphome_ns.namespace("dcf77_emitter")
DCF77Emitter = dcf77_emitter_ns.class_("DCF77Emitter", cg.Component)

CONF_ANTENNA_PIN = "antenna_pin"
CONF_LED_PIN = "led_pin"
CONF_SYNC_SWITCH_ID = "sync_switch_id"

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(DCF77Emitter),
    cv.Required(CONF_TIME_ID): cv.use_id(time.RealTimeClock),
    cv.Required(CONF_ANTENNA_PIN): pins.gpio_output_pin_schema,
    cv.Required(CONF_LED_PIN): pins.gpio_output_pin_schema,
    cv.Required(CONF_SYNC_SWITCH_ID): cv.use_id(switch.Switch),
}).extend(cv.COMPONENT_SCHEMA)

_LOGGER = logging.getLogger(__name__)  # <- logger for structured logs

async def to_code(config):
    # Simple stdout print (always visible)
    print("dcf77_emitter.to_code: starting with config:", config)

    # Structured logging (may be filtered depending on esphome logging)
    _LOGGER.info("dcf77_emitter.to_code: creating Pvariable for id=%s", config.get("id"))

    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    # After registration show confirmation
    print("dcf77_emitter.to_code: registered component variable:", var)

    time_ = await cg.get_variable(config[CONF_TIME_ID])
    cg.add(var.set_time_id(time_))
    print("dcf77_emitter.to_code: set_time_id done")

    pin = await cg.gpio_pin_expression(config[CONF_ANTENNA_PIN])
    cg.add(var.set_antenna_pin(pin))
    print("dcf77_emitter.to_code: set_antenna_pin done ->", pin)

    pin = await cg.gpio_pin_expression(config[CONF_LED_PIN])
    cg.add(var.set_led_pin(pin))
    print("dcf77_emitter.to_code: set_led_pin done ->", pin)
    
    switch_ = await cg.get_variable(config[CONF_SYNC_SWITCH_ID])
    cg.add(var.set_sync_switch(switch_))
    print("dcf77_emitter.to_code: set_sync_switch done ->", switch_)

    _LOGGER.info("dcf77_emitter.to_code: finished") 
