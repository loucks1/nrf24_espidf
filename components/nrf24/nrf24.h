#pragma once

#include "esphome/core/component.h"
#include "esphome/components/spi/spi.h"
#include "esphome/core/gpio.h"

#include "nRF24L01.h"

namespace esphome
{
  namespace nrf24
  {
    /** Re-exported enums from original library for full compatibility */
    using rf24_pa_dbm_e = nRF24L01::rf24_pa_dbm_e;
    using rf24_datarate_e = nRF24L01::rf24_datarate_e;
    using rf24_crclength_e = nRF24L01::rf24_crclength_e;
    using rf24_fifo_state_e = nRF24L01::rf24_fifo_state_e;
    using rf24_irq_flags_e = nRF24L01::rf24_irq_flags_e;

    class NRF24Component : public Component,
                           public spi::SPIDevice<spi::BIT_ORDER_MSB_FIRST,
                                                 spi::CLOCK_POLARITY_LOW,
                                                 spi::CLOCK_PHASE_LEADING,
                                                 spi::DATA_RATE_1MHZ>
    {

    public:
      void set_ce_pin(GPIOPin *pin) { this->ce_pin_ = pin; }

      // ==================== Core setup ====================
      void setup() override;
      void loop() override;
      void dump_config() override;

      bool is_chip_connected();

      using on_data_callback_t = std::function<void(const uint8_t *, uint8_t)>;

      void add_on_data_callback(on_data_callback_t &&callback)
      {
        this->on_data_callbacks_.push_back(std::move(callback));
      }

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
      bool write_fast(const void *buf, uint8_t len);
      bool write_fast(const void *buf, uint8_t len, bool multicast);
      bool write_blocking(const void *buf, uint8_t len, uint32_t timeout);

      // ==================== Pipes & Addressing ====================
      void open_writing_pipe(const uint8_t *address);
      void open_writing_pipe(uint64_t address);
      void open_reading_pipe(uint8_t number, const uint8_t *address);
      void open_reading_pipe(uint8_t number, uint64_t address);
      void set_address_width(uint8_t address_width);

      void close_reading_pipe(uint8_t pipe);

      // ==================== Configuration ====================

      void set_retries(uint8_t delay, uint8_t count);
      void set_channel(uint8_t channel);
      void set_pa_level(rf24_pa_dbm_e level, bool lna_enable);
      bool set_rf_data_rate(rf24_datarate_e speed);
      void set_crc_length(rf24_crclength_e length);
      void disable_crc();

      uint8_t get_channel();
      rf24_pa_dbm_e get_pa_level();
      rf24_datarate_e get_rf_data_rate();
      rf24_crclength_e get_crc_length();

      uint8_t get_payload_size();
      uint8_t get_dynamic_payload_size();

      // ==================== Features ====================
      void enable_ack_payload();
      void disable_ack_payload();
      void enable_dynamic_payloads();
      void disable_dynamic_payloads();
      void set_auto_ack(bool enable);
      void set_auto_ack(uint8_t pipe, bool enable);

      // ==================== ACK Payloads ====================
      void write_ack_payload(uint8_t pipe, const void *buf, uint8_t len);
      bool is_ack_payload_available();

      // ==================== Power management ====================
      void power_down();
      void power_up();

      // ==================== Status & Debug ====================
      void mask_irq(bool tx_ok, bool tx_fail, bool rx_ready);
      uint8_t what_happened(); // returns rf24_irq_flags_e bits
      void print_details();
      void print_pretty_details();
      void print_status();
      void print_observe_tx();

      // FIFO status
      nRF24L01::rf24_fifo_state_e is_fifo(bool tx);
      bool tx_standby();
      bool tx_standby(uint32_t timeout, bool startTx = false);

      // Low-level helpers
      void flush_tx();
      void flush_rx();
      void reuse_tx_pl();
      bool test_carrier();
      bool test_rpd();

      // CE control (public for advanced use)
      void ce(bool level);

      // ESPHOME CODEGEN ONLY (Do not call manually)
      void set_channel_(uint8_t channel) { this->channel_ = channel; }
      void set_rf_data_rate_(rf24_datarate_e rf_data_rate) { this->rf_data_rate_ = rf_data_rate; }
      void set_pa_level_(rf24_pa_dbm_e pa_level) { this->pa_level_ = pa_level; }

    protected:
      // SPI transaction helpers (used by all register functions)
      void begin_transaction_();
      void end_transaction_();

      std::vector<on_data_callback_t> on_data_callbacks_;

      uint8_t read_register(uint8_t reg);
      void read_register(uint8_t reg, uint8_t *buf, uint8_t len);
      void write_register(uint8_t reg, uint8_t value);
      void write_register(uint8_t reg, const uint8_t *buf, uint8_t len);

      void toggle_features();

      void start_write(const void *buf, uint8_t len, bool multicast);
      void write_payload(const void *buf, uint8_t data_len, uint8_t writeType);

      uint32_t last_watchdog_check_{0};
      bool hardware_initialized_{false};
      bool soft_reset();
      bool is_listening_{false};

      GPIOPin *ce_pin_{nullptr};

      uint8_t channel_{76};
      uint8_t payload_size_{32};
      bool dynamic_payloads_enabled_{false};
      uint8_t addr_width_{5};
      uint8_t status_{0};
      uint8_t config_reg_{0};
      bool ack_payloads_enabled_{false};
      uint8_t pipe0_writing_address_[5]{};

      nRF24L01::rf24_pa_dbm_e pa_level_{nRF24L01::RF24_PA_MAX};
      nRF24L01::rf24_datarate_e rf_data_rate_{nRF24L01::RF24_1MBPS};
      nRF24L01::rf24_crclength_e crc_length_{nRF24L01::RF24_CRC_16};

    private:
    };

  } // namespace nrf24
} // namespace esphome