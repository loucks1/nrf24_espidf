#include "nrf24.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"

namespace esphome {
namespace nrf24 {

static const char *const TAG = "nrf24";

void RF24::cs_select_() { this->enable(); }
void RF24::cs_unselect_() { this->disable(); }
void RF24::ce_set_(bool level) { this->ce_pin_->digital_write(level); }

uint8_t RF24::get_status_() {
  this->cs_select_();
  uint8_t status = this->transfer_byte(nRF24L01::NOP);
  this->cs_unselect_();
  return status;
}

uint8_t RF24::read_register(uint8_t reg) {
  this->cs_select_();
  this->write_byte(reg & nRF24L01::REGISTER_MASK);
  uint8_t value = this->transfer_byte(nRF24L01::NOP);
  this->cs_unselect_();
  return value;
}

void RF24::read_register(uint8_t reg, uint8_t *buf, uint8_t len) {
  this->cs_select_();
  this->write_byte(reg & nRF24L01::REGISTER_MASK);
  for (uint8_t i = 0; i < len; i++) {
    buf[i] = this->transfer_byte(nRF24L01::NOP);
  }
  this->cs_unselect_();
}

void RF24::write_register(uint8_t reg, uint8_t value) {
  this->cs_select_();
  this->write_byte(nRF24L01::W_REGISTER | (reg & nRF24L01::REGISTER_MASK));
  this->write_byte(value);
  this->cs_unselect_();
}

void RF24::write_register(uint8_t reg, const uint8_t *buf, uint8_t len) {
  this->cs_select_();
  this->write_byte(nRF24L01::W_REGISTER | (reg & nRF24L01::REGISTER_MASK));
  this->write(buf, len);
  this->cs_unselect_();
}

uint8_t RF24::flush_rx_() {
  this->cs_select_();
  uint8_t status = this->transfer_byte(nRF24L01::FLUSH_RX);
  this->cs_unselect_();
  return status;
}

uint8_t RF24::flush_tx_() {
  this->cs_select_();
  uint8_t status = this->transfer_byte(nRF24L01::FLUSH_TX);
  this->cs_unselect_();
  return status;
}

bool RF24::begin() {
  if (!this->ce_pin_) {
    ESP_LOGE(TAG, "CE pin not set");
    return false;
  }

  this->ce_pin_->setup();
  this->ce_pin_->pin_mode(gpio::FLAG_OUTPUT);
  this->ce_set_(false);

  // SPI is already initialized by SPISingleDevice

  // Reset nRF24L01+ and delay
  this->write_register(nRF24L01::CONFIG, 0x0C);  // PWR_UP=0, PRIM_RX=0
  delay(5);

  // Check if chip is present
  this->write_register(nRF24L01::SETUP_AW, 0x03);  // 5-byte address
  if (this->read_register(nRF24L01::SETUP_AW) != 0x03) {
    ESP_LOGE(TAG, "nRF24L01 not detected!");
    return false;
  }

  // Default configuration (matches Arduino RF24)
  this->setChannel(this->channel_);
  this->setPALevel(RF24_PA_LOW);
  this->setDataRate(RF24_1MBPS);
  this->setCRCLength(RF24_CRC_16);
  this->setRetries(15, 15);
  this->setPayloadSize(32);
  this->disableDynamicPayloads();

  this->flush_rx_();
  this->flush_tx_();

  this->write_register(nRF24L01::STATUS, 0x70);  // clear all IRQ flags

  ESP_LOGI(TAG, "nRF24L01 initialized successfully");
  return true;
}

void RF24::setup() { this->begin(); }

void RF24::dump_config() {
  ESP_LOGCONFIG(TAG, "nRF24:");
  LOG_PIN("  CE Pin: ", this->ce_pin_);
  LOG_PIN("  IRQ Pin: ", this->irq_pin_);
  ESP_LOGCONFIG(TAG, "  Channel: %d", this->channel_);
}

void RF24::startListening() {
  this->write_register(nRF24L01::CONFIG, this->read_register(nRF24L01::CONFIG) | 0x03);  // PWR_UP + PRIM_RX
  this->ce_set_(true);
  delay(1);  // Tpd2stby->Rx
}

void RF24::stopListening() {
  this->ce_set_(false);
  this->write_register(nRF24L01::CONFIG, this->read_register(nRF24L01::CONFIG) & ~0x01);  // PRIM_RX=0
}

bool RF24::write(const void *buf, uint8_t len) {
  return this->write_payload(buf, len, nRF24L01::W_TX_PAYLOAD);
}

bool RF24::writeAckPayload(uint8_t pipe, const void *buf, uint8_t len) {
  const uint8_t *current = reinterpret_cast<const uint8_t *>(buf);
  this->cs_select_();
  this->write_byte(nRF24L01::W_ACK_PAYLOAD | (pipe & 0x07));
  this->write(current, len);
  this->cs_unselect_();
  return true;
}

bool RF24::available() { return this->available(nullptr); }

bool RF24::available(uint8_t *pipe_num) {
  uint8_t status = this->get_status_();
  if (!(status & (1 << nRF24L01::RX_DR)))
    return false;

  if (pipe_num) {
    *pipe_num = (status >> 1) & 0x07;
  }
  return true;
}

bool RF24::read(void *buf, uint8_t len) {
  uint8_t actual_len = len;
  if (this->dynamic_payloads_enabled_) {
    actual_len = this->read_register(nRF24L01::R_RX_PL_WID);
  }
  if (actual_len > 32)
    actual_len = 32;

  this->read_payload(buf, actual_len);
  this->write_register(nRF24L01::STATUS, 1 << nRF24L01::RX_DR);  // clear flag
  return true;
}

void RF24::openWritingPipe(uint64_t address) {
  uint8_t addr[5];
  for (uint8_t i = 0; i < 5; i++) {
    addr[4 - i] = address & 0xFF;
    address >>= 8;
  }
  this->openWritingPipe(addr);
}

void RF24::openWritingPipe(const uint8_t *address) {
  this->write_register(nRF24L01::TX_ADDR, address, 5);
  this->write_register(nRF24L01::RX_ADDR_P0, address, 5);
}

void RF24::openReadingPipe(uint8_t number, uint64_t address) {
  uint8_t addr[5];
  for (uint8_t i = 0; i < 5; i++) {
    addr[4 - i] = address & 0xFF;
    address >>= 8;
  }
  this->openReadingPipe(number, addr);
}

void RF24::openReadingPipe(uint8_t number, const uint8_t *address) {
  if (number == 0) {
    memcpy(this->pipe0_reading_address_, address, 5);
  }
  if (number < 6) {
    this->write_register(nRF24L01::RX_ADDR_P0 + number, address, number < 2 ? 5 : 1);
    this->write_register(nRF24L01::EN_RXADDR, this->read_register(nRF24L01::EN_RXADDR) | (1 << number));
  }
}

void RF24::setPALevel(uint8_t level, bool lna_enable) {
  uint8_t setup = this->read_register(nRF24L01::RF_SETUP) & 0xF8;
  if (level == RF24_PA_MAX) {
    setup |= 0x06;
  } else if (level == RF24_PA_HIGH) {
    setup |= 0x04;
  } else if (level == RF24_PA_LOW) {
    setup |= 0x02;
  }
  if (lna_enable)
    setup |= 0x01;
  this->write_register(nRF24L01::RF_SETUP, setup);
}

uint8_t RF24::getPALevel() {
  uint8_t setup = this->read_register(nRF24L01::RF_SETUP) & 0x06;
  if (setup == 0x06)
    return RF24_PA_MAX;
  if (setup == 0x04)
    return RF24_PA_HIGH;
  if (setup == 0x02)
    return RF24_PA_LOW;
  return RF24_PA_MIN;
}

void RF24::setDataRate(uint8_t speed) {
  uint8_t setup = this->read_register(nRF24L01::RF_SETUP) & 0xF7;
  if (speed == RF24_250KBPS) {
    setup |= (1 << 5);
    setup &= ~(1 << 3);
  } else if (speed == RF24_2MBPS) {
    setup |= (1 << 3);
    setup &= ~(1 << 5);
  } else {  // 1MBPS
    setup &= ~(1 << 3);
    setup &= ~(1 << 5);
  }
  this->write_register(nRF24L01::RF_SETUP, setup);
}

uint8_t RF24::getDataRate() {
  uint8_t setup = this->read_register(nRF24L01::RF_SETUP);
  if (setup & (1 << 5))
    return RF24_250KBPS;
  if (setup & (1 << 3))
    return RF24_2MBPS;
  return RF24_1MBPS;
}

void RF24::setCRCLength(uint8_t length) {
  uint8_t config = this->read_register(nRF24L01::CONFIG) & 0xF3;
  if (length == RF24_CRC_16) {
    config |= 0x0C;
  } else if (length == RF24_CRC_8) {
    config |= 0x08;
  }
  this->write_register(nRF24L01::CONFIG, config);
}

uint8_t RF24::getCRCLength() {
  uint8_t config = this->read_register(nRF24L01::CONFIG) & 0x0C;
  if (config == 0x0C)
    return RF24_CRC_16;
  if (config == 0x08)
    return RF24_CRC_8;
  return RF24_CRC_DISABLED;
}

void RF24::disableCRC() { this->setCRCLength(RF24_CRC_DISABLED); }

void RF24::setRetries(uint8_t delay, uint8_t count) {
  this->write_register(nRF24L01::SETUP_RETR, (delay & 0x0F) << 4 | (count & 0x0F));
}

void RF24::setChannel(uint8_t channel) {
  this->write_register(nRF24L01::RF_CH, channel & 0x7F);
}

uint8_t RF24::getChannel() { return this->read_register(nRF24L01::RF_CH); }

void RF24::setPayloadSize(uint8_t size) {
  this->payload_size_ = size > 32 ? 32 : size;
}

uint8_t RF24::getPayloadSize() { return this->payload_size_; }

void RF24::enableDynamicPayloads() {
  this->dynamic_payloads_enabled_ = true;
  this->write_register(nRF24L01::FEATURE, this->read_register(nRF24L01::FEATURE) | 0x04);
  this->write_register(nRF24L01::DYNPD, 0x3F);
}

void RF24::disableDynamicPayloads() {
  this->dynamic_payloads_enabled_ = false;
  this->write_register(nRF24L01::DYNPD, 0x00);
}

bool RF24::isPVariant() { return this->p_variant_; }

void RF24::setAutoAck(bool enable) {
  if (enable)
    this->write_register(nRF24L01::EN_AA, 0x3F);
  else
    this->write_register(nRF24L01::EN_AA, 0x00);
}

void RF24::setAutoAck(uint8_t pipe, bool enable) {
  uint8_t en_aa = this->read_register(nRF24L01::EN_AA);
  if (enable)
    en_aa |= (1 << pipe);
  else
    en_aa &= ~(1 << pipe);
  this->write_register(nRF24L01::EN_AA, en_aa);
}

void RF24::maskIRQ(bool tx_ok, bool tx_fail, bool rx_ready) {
  uint8_t config = this->read_register(nRF24L01::CONFIG);
  config &= 0x8F;  // clear MASK bits
  if (tx_ok)
    config |= 0x40;
  if (tx_fail)
    config |= 0x20;
  if (rx_ready)
    config |= 0x10;
  this->write_register(nRF24L01::CONFIG, config);
}

void RF24::powerDown() {
  this->ce_set_(false);
  this->write_register(nRF24L01::CONFIG, this->read_register(nRF24L01::CONFIG) & ~0x02);
}

void RF24::powerUp() {
  this->write_register(nRF24L01::CONFIG, this->read_register(nRF24L01::CONFIG) | 0x02);
}

bool RF24::write_payload(const void *buf, uint8_t len, uint8_t write_type) {
  const uint8_t *current = reinterpret_cast<const uint8_t *>(buf);
  uint8_t data_len = this->dynamic_payloads_enabled_ ? len : this->payload_size_;

  this->cs_select_();
  this->write_byte(write_type);
  this->write(current, data_len);
  this->cs_unselect_();

  this->ce_set_(true);
  delay(1);
  this->ce_set_(false);
  return true;
}

uint8_t RF24::read_payload(void *buf, uint8_t len) {
  uint8_t *current = reinterpret_cast<uint8_t *>(buf);
  this->cs_select_();
  this->write_byte(nRF24L01::R_RX_PAYLOAD);
  for (uint8_t i = 0; i < len; i++) {
    current[i] = this->transfer_byte(nRF24L01::NOP);
  }
  this->cs_unselect_();
  return len;
}

}  // namespace nrf24
}  // namespace esphome