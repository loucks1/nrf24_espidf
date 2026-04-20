import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.components import spi
from esphome.const import CONF_ID, CONF_CE_PIN

DEPENDENCIES = ["spi"]
AUTO_LOAD = ["spi"]

nrf24_ns = cg.esphome_ns.namespace("nrf24")
NRF24Component = nrf24_ns.class_("NRF24Component", cg.Component, spi.SPIDevice)

CONFIG_SCHEMA = (
    spi.spi_device_schema(
        default_data_rate="10MHz",  # nRF24L01+ max recommended
        default_mode=spi.SPI_MODE_0,
        default_bit_order=spi.BIT_ORDER_MSB_FIRST,
    )
    .extend(
        {
            cv.GenerateID(): cv.declare_id(NRF24Component),
            cv.Required(CONF_CE_PIN): pins.gpio_output_pin_schema,
        }
    )
)

def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    yield cg.register_component(var, config)
    yield spi.register_spi_device(var, config)

    ce_pin = yield pins.gpio_pin_expression(config[CONF_CE_PIN])
    cg.add(var.set_ce_pin(ce_pin))