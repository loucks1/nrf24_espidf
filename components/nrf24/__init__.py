import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.components import spi
from esphome.const import CONF_ID

DEPENDENCIES = ["spi"]
AUTO_LOAD = ["spi"]

nrf24_ns = cg.esphome_ns.namespace("nrf24")
NRF24Component = nrf24_ns.class_("NRF24Component", cg.Component, spi.SPIDevice)

CONF_CE_PIN = "ce_pin"

CONFIG_SCHEMA = (
    spi.spi_device_schema(
        cs_pin_required=True,
    )
    .extend(
        {
            cv.GenerateID(): cv.declare_id(NRF24Component),
            cv.Required(CONF_CE_PIN): pins.gpio_output_pin_schema,
            # Optional configuration
            cv.Optional("channel", default=76): cv.int_range(0, 125),
            cv.Optional("data_rate", default="1MBPS"): cv.one_of("250KBPS", "1MBPS", "2MBPS", lower=True),
            cv.Optional("pa_level", default="MAX"): cv.one_of("MIN", "LOW", "HIGH", "MAX", upper=True),
            cv.Optional("payload_size", default=32): cv.int_range(1, 32),
        }
    )
)

def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    yield cg.register_component(var, config)
    yield spi.register_spi_device(var, config)

    ce_pin = yield pins.gpio_pin_expression(config[CONF_CE_PIN])
    cg.add(var.set_ce_pin(ce_pin))

    # Pass configuration to C++
    cg.add(var.set_channel(config["channel"]))
    cg.add(var.set_data_rate_str(config["data_rate"]))
    cg.add(var.set_pa_level_str(config["pa_level"]))
    cg.add(var.set_payload_size(config["payload_size"]))
