import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.components import spi
from esphome.const import CONF_ID, CONF_CHANNEL

nrf24_ns = cg.esphome_ns.namespace("nrf24")
NRF24Component = cg.global_ns.class_("esphome::nrf24::NRF24Component", cg.Component, spi.SPIDevice)
nrf24l01_header_ns = cg.global_ns.namespace("nRF24L01")

# RF-specific Enums
rf24_datarate_e = nrf24l01_header_ns.enum("rf24_datarate_e")
RF_DATARATES = {
    "250KBPS": rf24_datarate_e.RF24_250KBPS,
    "1MBPS": rf24_datarate_e.RF24_1MBPS,
    "2MBPS": rf24_datarate_e.RF24_2MBPS,
}

rf24_pa_dbm_e = nrf24l01_header_ns.enum("rf24_pa_dbm_e")
PA_LEVELS = {
    "MIN": rf24_pa_dbm_e.RF24_PA_MIN,
    "LOW": rf24_pa_dbm_e.RF24_PA_LOW,
    "HIGH": rf24_pa_dbm_e.RF24_PA_HIGH,
    "MAX": rf24_pa_dbm_e.RF24_PA_MAX,
}

CONF_CE_PIN = "ce_pin"
CONF_CHANNEL = "channel"
CONF_RF_DATA_RATE = "rf_data_rate"
CONF_PA_LEVEL = "pa_level"

CONFIG_SCHEMA = spi.spi_device_schema(cs_pin_required=True).extend(
    {
        cv.GenerateID(): cv.declare_id(NRF24Component),
        cv.Required(CONF_CE_PIN): pins.gpio_output_pin_schema,
        cv.Optional(CONF_CHANNEL, default=76): cv.int_range(0, 125),
        # We use rf_datarate here to distinguish from SPI data_rate
        cv.Optional(CONF_RF_DATA_RATE, default="1MBPS"): cv.enum(RF_DATARATES, upper=True),
        cv.Optional(CONF_PA_LEVEL, default="MAX"): cv.enum(PA_LEVELS, upper=True),    
    }
).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    
    # This handles the SPI 'data_rate' (e.g., 2MHz) and 'cs_pin' automatically
    await spi.register_spi_device(var, config)

    ce_pin = await cg.gpio_pin_expression(config[CONF_CE_PIN])
    cg.add(var.set_ce_pin(ce_pin))
    cg.add(var.set_channel_(config[CONF_CHANNEL]))
    cg.add(var.set_rf_data_rate_(config[CONF_RF_DATA_RATE]))
    cg.add(var.set_pa_level_(config[CONF_PA_LEVEL]))