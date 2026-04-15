import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import spi
from esphome.const import CONF_ID, CONF_CE_PIN, CONF_IRQ_PIN, CONF_CHANNEL

nrf24_ns = cg.esphome_ns.namespace("nrf24")
RF24 = nrf24_ns.class_("RF24", cg.Component, spi.SPISingleDevice)

CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(RF24),
            cv.Required(CONF_CE_PIN): cv.use_id(cg.InternalGPIOPin),
            cv.Optional(CONF_IRQ_PIN): cv.use_id(cg.InternalGPIOPin),
            cv.Optional(CONF_CHANNEL, default=76): cv.int_range(0, 125),
        }
    ).extend(spi.spi_device_schema(cs_pin_required=True)),
    cv.only_on_esp32,
)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await spi.register_spi_device(var, config)

    ce_pin = await cg.get_variable(config[CONF_CE_PIN])
    cg.add(var.set_ce_pin(ce_pin))

    if CONF_IRQ_PIN in config:
        irq_pin = await cg.get_variable(config[CONF_IRQ_PIN])
        cg.add(var.set_irq_pin(irq_pin))

    cg.add(var.set_channel(config[CONF_CHANNEL]))
    return var