#pragma once

#include "esphome/core/component.h"
#include "esphome/components/spi/spi.h"
#include "esphome/core/hal.h"

namespace esphome {
namespace nrf24 {

namespace nRF24L01 {
  // (full register map from official library)
  constexpr uint8_t CONFIG = 0x00;
  constexpr uint8_t EN_AA = 0x01;
  constexpr uint8_t EN_RXADDR = 0x02;
  constexpr uint8_t SETUP_AW = 0x03;
  constexpr uint8_t SETUP_RETR = 0x04;
  constexpr uint8_t RF_CH = 0x05;
  constexpr uint8_t RF_SETUP = 0x06;
  constexpr uint8_t STATUS = 0x07;
  constexpr uint8_t OBSERVE_TX = 0x08;
  constexpr uint8_t CD = 0x09;  // RPD on + variants
  constexpr uint8_t RX_ADDR_P0 = 0x0A;
  constexpr uint8_t RX_ADDR_P1 = 0x0B;
  constexpr uint8_t RX_ADDR_P2 = 0x0C;
  constexpr uint8_t RX_ADDR_P3 = 0x0D;
  constexpr uint8_t RX_ADDR_P4 = 0x0E;
  constexpr uint8_t RX_ADDR_P5 = 0x0F;
  constexpr uint8_t TX_ADDR = 0x10;
  constexpr uint8_t RX_PW_P0 = 0x11;
  constexpr uint8_t RX_PW_P1 = 0x12;
  constexpr uint8_t RX_PW_P2 = 0x13;
  constexpr uint8_t RX_PW_P3 = 0x14;
  constexpr uint8_t RX_PW_P4 = 0x15;
  constexpr uint8_t RX_PW_P5 = 0x16;
  constexpr uint8_t FIFO_STATUS = 0x17;
  constexpr uint8_t DYNPD = 0x1C;
  constexpr uint8_t FEATURE = 0x1D;

  constexpr uint8_t R_REGISTER = 0x00;
  constexpr uint8_t W_REGISTER = 0x20;
  constexpr uint8_t REGISTER_MASK = 0x1F;
  constexpr uint8_t R_RX_PAYLOAD = 0x61;
  constexpr uint8_t W_TX_PAYLOAD = 0xA0;
  constexpr uint8_t FLUSH_TX = 0xE1;
  constexpr uint8_t FLUSH_RX = 0xE2;
  constexpr uint8_t REUSE_TX_PL = 0xE3;
  constexpr uint8_t NOP = 0xFF;
  constexpr uint8_t W_ACK_PAYLOAD = 0xA8;
  constexpr uint8_t W_TX_PAYLOAD_NO_ACK = 0xB0;
  constexpr uint8_t R_RX_PL_WID = 0x60;
}  // namespace nRF24L01

// Enums from official RF24 (full compatibility)
enum rf24_pa_dbm_e { RF24_PA_MIN = 0, RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX, RF24_PA_ERROR };
enum rf24_datarate_e { RF24_1MBPS = 0, RF24_2MBPS, RF24_250KBPS };
enum rf24_crclength_e { RF24_CRC_DISABLED = 0, RF24_CRC_8, RF24_CRC_16 };

class RF24 : public Component, public spi::SPISingleDevice {
 public:
  void set_ce_pin(InternalGPIOPin *pin) { ce_pin_ = pin; }
  void set_irq_pin(InternalGPIOPin *pin) { irq_pin_ = pin; }
  void set_channel(uint8_t channel) { channel_ = channel; }

  // === Full public API (100% compatible with Arduino RF24) ===
  bool begin();
  void startListening();
  void stopListening();
  bool write(const void *buf, uint8_t len);
  bool writeAckPayload(uint8_t pipe, const void *buf, uint8_t len);
  bool available();
  bool available(uint8_t *pipe_num);
  bool read(void *buf, uint8_t len);
  void openWritingPipe(uint64_t address);
  void openWritingPipe(const uint8_t *address);
  void openReadingPipe(uint8_t number, uint64_t address);
  void openReadingPipe(uint8_t number, const uint8_t *address);
  void setPALevel(uint8_t level, bool lna_enable = true);
  uint8_t getPALevel();
  void setDataRate(uint8_t speed);
  uint8_t getDataRate();
  void setCRCLength(uint8_t length);
  uint8_t getCRCLength();
  void disableCRC();
  void setRetries(uint8_t delay, uint8_t count);
  void setChannel(uint8_t channel);
  uint8_t getChannel();
  void setPayloadSize(uint8_t size);
  uint8_t getPayloadSize();
  void enableDynamicPayloads();
  void disableDynamicPayloads();
  bool isPVariant();
  void setAutoAck(bool enable);
  void setAutoAck(uint8_t pipe, bool enable);
  void maskIRQ(bool tx_ok, bool tx_fail, bool rx_ready);
  void powerDown();
  void powerUp();

  void setup() override;
  void dump_config() override;

 protected:
  InternalGPIOPin *ce_pin_{nullptr};
  InternalGPIOPin *irq_pin_{nullptr};
  uint8_t channel_{76};

 private:
  void cs_select_();
  void cs_unselect_();
  void ce_set_(bool level);

  uint8_t read_register(uint8_t reg);
  void read_register(uint8_t reg, uint8_t *buf, uint8_t len);
  void write_register(uint8_t reg, uint8_t value);
  void write_register(uint8_t reg, const uint8_t *buf, uint8_t len);

  uint8_t flush_rx_();
  uint8_t flush_tx_();
  uint8_t get_status_();

  bool write_payload(const void *buf, uint8_t len, uint8_t write_type);
  uint8_t read_payload(void *buf, uint8_t len);

  uint8_t payload_size_{32};
  uint8_t pipe0_reading_address_[5]{};
  bool dynamic_payloads_enabled_{false};
  bool ack_payloads_enabled_{false};
  bool p_variant_{false};
  uint8_t config_reg_{0};
};

}  // namespace nrf24
}  // namespace esphome