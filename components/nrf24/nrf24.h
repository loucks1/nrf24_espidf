#pragma once

#include "esphome/core/component.h"
#include "esphome/components/spi/spi.h"
#include "esphome/core/gpio.h"

#include "nRF24L01.h" // You should copy the original nRF24L01.h next to this file

namespace esphome
{
  namespace nrf24
  {

    using ::nRF24L01::rf24_crclength_e;
    using ::nRF24L01::rf24_datarate_e;
    using ::nRF24L01::rf24_pa_dbm_e;

    /** Re-exported enums from original library for full compatibility */
    // Change these lines:
    typedef nRF24L01::rf24_pa_dbm_e rf24_pa_dbm_e;
    typedef nRF24L01::rf24_datarate_e rf24_datarate_e;
    typedef nRF24L01::rf24_crclength_e rf24_crclength_e;
    typedef nRF24L01::rf24_fifo_state_e rf24_fifo_state_e;
    typedef nRF24L01::rf24_irq_flags_e rf24_irq_flags_e;

    /** Main nRF24L01+ component with full original RF24 API compatibility */
    class NRF24Component : public Component, public spi::SPIDevice<spi::BIT_ORDER_MSB_FIRST, spi::CLOCK_POLARITY_LOW, spi::CLOCK_PHASE_LEADING, spi::DATA_RATE_1MHZ>
    {
    public:
      explicit NRF24Component(GPIOPin *ce_pin, uint32_t spi_speed = 10000000);
      explicit NRF24Component(uint32_t spi_speed = 10000000);

      void set_ce_pin(GPIOPin *pin) { this->ce_pin_ = pin; }

      // ==================== Core setup ====================
      void setup() override;
      void dump_config() override;

      bool begin();
      bool begin(GPIOPin *ce_pin); // csn is handled by SPIDevice

      bool is_chip_connected();

      void set_channel(uint8_t channel);
      void set_data_rate_str(const std::string &data_rate);
      void set_pa_level_str(const std::string &pa_level);
      void set_payload_size(uint8_t size);

      // ==================== Operating mode ====================
      void start_listening();
      void stop_listening();
      void stop_listening(const uint8_t *tx_address); // overload for TX address reuse

      // ==================== Data transfer ====================
      bool available();
      bool available(uint8_t *pipe_num);
      void read(void *buf, uint8_t len);
      bool write(const void *buf, uint8_t len);
      bool write(const void *buf, uint8_t len, bool multicast);
      bool writeFast(const void *buf, uint8_t len);
      bool writeFast(const void *buf, uint8_t len, bool multicast);
      bool writeBlocking(const void *buf, uint8_t len, uint32_t timeout);

      // ==================== Pipes & Addressing ====================
      void open_writing_pipe(const uint8_t *address);
      void open_writing_pipe(uint64_t address);
      void open_reading_pipe(uint8_t number, const uint8_t *address);
      void open_reading_pipe(uint8_t number, uint64_t address);
      void set_address_width(uint8_t a_width);

      void close_reading_pipe(uint8_t pipe);

      // ==================== Configuration ====================
      void setPALevel(rf24_pa_dbm_e level, bool lna_enable = true);
      rf24_pa_dbm_e getPALevel();
      bool set_rf_data_rate(rf24_datarate_e speed);
      rf24_datarate_e getDataRate();
      void setCRCLength(rf24_crclength_e length);
      rf24_crclength_e getCRCLength();
      void disableCRC();
      void setRetries(uint8_t delay, uint8_t count);
      void setChannel(uint8_t channel);
      uint8_t getChannel();
      void setPayloadSize(uint8_t size);
      uint8_t getPayloadSize();
      uint8_t getDynamicPayloadSize();

      // ==================== Features ====================
      void enableAckPayload();
      void disableAckPayload();
      void enableDynamicPayloads();
      void disableDynamicPayloads();
      bool isPVariant();
      void setAutoAck(bool enable);
      void setAutoAck(uint8_t pipe, bool enable);

      // ==================== ACK Payloads ====================
      void writeAckPayload(uint8_t pipe, const void *buf, uint8_t len);
      bool isAckPayloadAvailable();

      // ==================== Power management ====================
      void powerDown();
      void powerUp();

      // ==================== Status & Debug ====================
      void maskIRQ(bool tx_ok, bool tx_fail, bool rx_ready);
      uint8_t whatHappened(); // returns rf24_irq_flags_e bits
      void printDetails();
      void printPrettyDetails();
      void printStatus();
      void printObserveTx();

      // FIFO status
      nRF24L01::rf24_fifo_state_e isFifo(bool tx);
      bool txStandBy();
      bool txStandBy(uint32_t timeout, bool startTx = false);

      // Low-level helpers
      void flush_tx();
      void flush_rx();
      void reuse_tx_pl();
      bool testCarrier();
      bool testRPD();

      // CE control (public for advanced use)
      void ce(bool level);

    protected:
      // SPI transaction helpers (used by all register functions)
      void begin_transaction_();
      void end_transaction_();

      uint8_t read_register(uint8_t reg);
      void read_register(uint8_t reg, uint8_t *buf, uint8_t len);
      void write_register(uint8_t reg, uint8_t value);
      void write_register(uint8_t reg, const uint8_t *buf, uint8_t len);

      void toggle_features();

      void start_write(const void *buf, uint8_t len, bool multicast);
      void write_payload(const void *buf, uint8_t data_len, uint8_t writeType);

    private:
      GPIOPin *ce_pin_{nullptr};
      uint32_t spi_speed_;
      uint8_t status_{0};
      uint8_t payload_size_{32};
      uint8_t addr_width_{5};
      uint8_t pipe0_reading_address_[5]{};
      uint8_t pipe0_writing_address_[5]{};
      bool dynamic_payloads_enabled_{false};
      bool ack_payloads_enabled_{false};
      bool _is_p_variant{false};
      bool _is_p0_rx{false};
      uint8_t config_reg_{0};
      uint8_t channel_{76};
      nRF24L01::rf24_pa_dbm_e pa_level_{nRF24L01::RF24_PA_MAX};
      nRF24L01::rf24_datarate_e data_rate_{nRF24L01::RF24_1MBPS};
      nRF24L01::rf24_crclength_e crc_length_{nRF24L01::RF24_CRC_16};

      void setup_pins_();
    };

  } // namespace nrf24
} // namespace esphome