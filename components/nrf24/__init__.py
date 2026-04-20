import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.components import spi
from esphome.const import CONF_ID

DEPENDENCIES = ["spi"]
AUTO_LOAD = ["spi"]

nrf24_ns = cg.esphome_ns.namespace("nrf24")
NRF24Component = nrf24_ns.class_("NRF24Component", cg.Component, spi.SPIDevice)

CONF_CE_PIN = "ce_pin"      # ← Custom key, not from const

CONFIG_SCHEMA = (
    spi.spi_device_schema(
        default_data_rate="10MHz",
        default_mode=spi.SPI_MODE_0,
        default_bit_order=spi.BIT_ORDER_MSB_FIRST,
    )
    .extend(
        {
            cv.GenerateID(): cv.declare_id(NRF24Component),
            cv.Required(CONF_CE_PIN): pins.gpio_output_pin_schema,
            # Optional extra config
            cv.Optional("channel", default=76): cv.int_range(0, 125),
            cv.Optional("data_rate", default="1MBPS"): cv.enum({
                "250KBPS": "RF24_250KBPS",
                "1MBPS": "RF24_1MBPS",
                "2MBPS": "RF24_2MBPS",
            }),
            cv.Optional("pa_level", default="MAX"): cv.enum({
                "MIN": "RF24_PA_MIN",
                "LOW": "RF24_PA_LOW",
                "HIGH": "RF24_PA_HIGH",
                "MAX": "RF24_PA_MAX",
            }),
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

    # Pass extra config to C++
    cg.add(var.set_channel(config["channel"]))
    cg.add(var.set_data_rate(config["data_rate"]))
    cg.add(var.set_pa_level(config["pa_level"]))
    cg.add(var.set_payload_size(config["payload_size"]))
