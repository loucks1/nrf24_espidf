#include "nrf24.h"
#include "nRF24L01.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#include <cstring>

namespace esphome
{
  namespace nrf24
  {
    using namespace ::nRF24L01; // This "unpacks" the radio constants into this file

    static const char *const TAG = "nrf24";

    void NRF24Component::setup()
    {
      if (this->ce_pin_)
      {
        this->ce_pin_->setup();
        this->ce_pin_->digital_write(false);
      }

      this->spi_setup();
      delay(10);

      if (!this->soft_reset())
      {
        ESP_LOGE(TAG, "NRF24 hardware not responding at boot. Will retry in loop.");
      }
    }

    bool NRF24Component::soft_reset()
    {
      this->power_down();
      delay(10);
      this->power_up();
      delay(10); // Wait for crystal stabilization

      // 2. Validate Connection
      if (!this->is_chip_connected())
      {
        this->hardware_initialized_ = false;
        return false;
      }

      this->write_register(nRF24L01::STATUS, nRF24L01::RX_DR | nRF24L01::TX_DS | nRF24L01::MAX_RT);

      this->flush_rx();
      this->flush_tx();

      set_channel(this->channel_);
      set_rf_data_rate(this->rf_data_rate_);
      set_pa_level(this->pa_level_, true);
      ESP_LOGI(TAG, "nRF24L01+ initialized successfully.");
      this->hardware_initialized_ = true;
      return true;
    }

    void NRF24Component::loop()
    {
      uint32_t now = millis();

      // Run watchdog every 15 seconds
      if (now - this->last_watchdog_check_ > 15000)
      {
        this->last_watchdog_check_ = now;

        if (!this->hardware_initialized_)
        {
          ESP_LOGW(TAG, "nRF24L01+ connection lost! Attempting recovery...");
          this->soft_reset();
        }

        this->hardware_initialized_ = this->is_chip_connected();
      }

      if (this->hardware_initialized_ && this->is_listening_ && !this->on_data_callbacks_.empty() && this->available())
      {
        uint8_t len = this->get_payload_size();
        if (dynamic_payloads_enabled_)
        {
          len = this->get_dynamic_payload_size();
        }

        if (len == 0 || len > 32)
        {
          this->flush_rx();
          this->write_register(nRF24L01::STATUS, nRF24L01::RX_DR); // Clear interrupt
          return;
        }

        uint8_t buffer[32];
        this->read(buffer, len);

        ESP_LOGV(TAG, "Received packet: %d  %s", len, format_hex_pretty(buffer, len).c_str());

        // Pass the pointer and the length to subscribers
        for (auto &callback : this->on_data_callbacks_)
        {
          callback(buffer, len);
        }
      }
    }

    void NRF24Component::dump_config()
    {
      this->print_pretty_details();
      LOG_PIN("   CE Pin:", this->ce_pin_);
      LOG_SPI_DEVICE(this);
      if (ESPHOME_LOG_LEVEL >= ESPHOME_LOG_LEVEL_VERBOSE)
      {

        yield();

        ESP_LOGCONFIG(TAG, "  --- register states ---");
        uint8_t config = this->read_register(nRF24L01::CONFIG);
        uint8_t en_aa = this->read_register(nRF24L01::EN_AA);
        uint8_t en_rx = this->read_register(nRF24L01::EN_RXADDR);
        uint8_t setup_aw = this->read_register(nRF24L01::SETUP_AW);
        uint8_t setup_retr = this->read_register(nRF24L01::SETUP_RETR);
        uint8_t rf_ch = this->read_register(nRF24L01::RF_CH);
        uint8_t rf_setup = this->read_register(nRF24L01::RF_SETUP);
        uint8_t status = this->read_register(nRF24L01::STATUS);
        uint8_t observe_tx = this->read_register(nRF24L01::OBSERVE_TX);
        uint8_t rpd = this->read_register(nRF24L01::RPD);
        bool carrier = this->test_carrier();
        uint8_t dynpd = this->read_register(nRF24L01::DYNPD);
        uint8_t feature = this->read_register(nRF24L01::FEATURE);

        ESP_LOGCONFIG(TAG, "    CONFIG:   0x%02X (%s)", config, (config & 0x08) ? "CRC ENABLED!" : "CRC Disabled");
        ESP_LOGCONFIG(TAG, "    EN_AA:    0x%02X (%s)", en_aa, (en_aa == 0x00) ? "Promiscuous OK" : "AUTO-ACK ON!");
        ESP_LOGCONFIG(TAG, "    EN_RX:    0x%02X (Pipes Enabled)", en_rx);
        ESP_LOGCONFIG(TAG, "    SETUP_AW: 0x%02X (Address Width)", setup_aw);
        ESP_LOGCONFIG(TAG, "    SETUP_RE: 0x%02X (Retransmission)", setup_retr);
        ESP_LOGCONFIG(TAG, "    RF_CH:    0x%02X (Hex %02X = Dec %d)", rf_ch, rf_ch, rf_ch);
        ESP_LOGCONFIG(TAG, "    RF_SETUP: 0x%02X (Data Rate/Power)", rf_setup);
        ESP_LOGCONFIG(TAG, "    STATUS:   0x%02X", status);
        ESP_LOGCONFIG(TAG, "    RPD:      0x%02X (%s)", rpd, carrier ? "SIGNAL DETECTED" : "Quiet");
        ESP_LOGCONFIG(TAG, "    DYNPD:    0x%02X", dynpd);
        ESP_LOGCONFIG(TAG, "    FEATURE:  0x%02X", feature);

        yield();

        uint8_t addr[5] = {0};
        this->read_register(nRF24L01::RX_ADDR_P1, addr, this->addr_width_);
        ESP_LOGCONFIG(TAG, "    RX_ADDR_P1: %s", format_hex(addr, this->addr_width_).c_str());
      }
    }
    // ====================== SPI Helpers ======================

    void NRF24Component::begin_transaction_()
    {
      this->enable(); // CS low + bus lock
    }

    void NRF24Component::end_transaction_()
    {
      this->disable(); // CS high + bus unlock
    }

    uint8_t NRF24Component::read_register(uint8_t reg)
    {
      uint8_t result;
      begin_transaction_();
      // 1. Send Read Command (0x00 | reg).
      // The byte returned DURING this call is the STATUS register.
      this->status_ = this->transfer_byte(nRF24L01::R_REGISTER | (reg & 0x1F));

      // 2. Send dummy byte to clock out the register value.
      result = this->transfer_byte(0xFF);
      end_transaction_();

      return result;
    }

    void NRF24Component::read_register(uint8_t reg, uint8_t *buf, uint8_t len)
    {
      begin_transaction_();
      // Send Read Command. Store status.
      this->status_ = this->transfer_byte(nRF24L01::R_REGISTER | (reg & 0x1F));

      // ESPHome's read_array is more efficient than a for-loop for multiple bytes
      this->read_array(buf, len);
      end_transaction_();
    }

    void NRF24Component::write_register(uint8_t reg, uint8_t value)
    {
      begin_transaction_();

      this->transfer_byte(nRF24L01::W_REGISTER | (reg & 0x1F));
      this->transfer_byte(value);

      end_transaction_();
    }

    void NRF24Component::write_register(uint8_t reg, const uint8_t *buf, uint8_t len)
    {
      begin_transaction_();
      // Send Write Command. Store status.
      this->status_ = this->transfer_byte(nRF24L01::W_REGISTER | (reg & 0x1F));

      // Use ESPHome's optimized array write
      this->write_array(buf, len);
      end_transaction_();
    }
    // ====================== Core ======================

    bool NRF24Component::is_chip_connected()
    {
      uint8_t aw = this->read_register(nRF24L01::SETUP_AW);
      return (aw >= 1 && aw <= 3);
    }

    void NRF24Component::ce(bool level)
    {
      if (this->ce_pin_)
        this->ce_pin_->digital_write(level);
      if (level)
        esp_rom_delay_us(130);
    }

    // ====================== Mode ======================

    void NRF24Component::start_listening()
    {
      config_reg_ |= BIT(nRF24L01::PWR_UP) | BIT(nRF24L01::PRIM_RX);
      write_register(nRF24L01::CONFIG, config_reg_);
      write_register(nRF24L01::STATUS, RF24_IRQ_ALL);
      this->ce(true);
      this->is_listening_ = true;
    }

    void NRF24Component::stop_listening()
    {
      this->is_listening_ = false;
      this->ce(false);
      this->write_register(nRF24L01::CONFIG, this->read_register(nRF24L01::CONFIG) & ~BIT(nRF24L01::PRIM_RX));
      if (this->ack_payloads_enabled_)
        this->flush_tx();
      for (uint8_t i = 0; i < 6; i++)
      {
        this->close_reading_pipe(i);
      }
    }

    void NRF24Component::stop_listening(const uint8_t *tx_address)
    {
      this->stop_listening();
      if (tx_address)
        this->open_writing_pipe(tx_address);
    }

    // ====================== Data ======================

    bool NRF24Component::available()
    {
      return this->available(nullptr);
    }

    bool NRF24Component::available(uint8_t *pipe_num)
    {
      // 1. Get the status
      uint8_t status = this->read_register(nRF24L01::STATUS);

      // 2. Check the pipe number bits (1, 2, and 3)
      uint8_t pipe = (status >> 1) & 0x07;

      if (pipe == 7)
      {
        // If the Data Ready bit (6) is stuck on, clear it now
        if (status & 0x40)
        {
          this->write_register(nRF24L01::STATUS, 0x40);
        }
        return false;
      }

      if (pipe_num)
        *pipe_num = pipe;
      return true;
    }

    void NRF24Component::read(void *buf, uint8_t len)
    {
      this->begin_transaction_();
      this->status_ = this->transfer_byte(nRF24L01::R_RX_PAYLOAD);
      this->read_array((uint8_t *)buf, len);
      this->end_transaction_();

      this->write_register(nRF24L01::STATUS, BIT(nRF24L01::RX_DR));

      uint8_t fifo_status = this->read_register(nRF24L01::FIFO_STATUS);
      bool more_data = !(fifo_status & BIT(nRF24L01::RX_EMPTY));

      ESP_LOGV(TAG, "Read %d bytes: %s", len, format_hex((const uint8_t *)buf, len).c_str());

      if (more_data)
      {
        ESP_LOGV(TAG, "FIFO not empty, more packets pending...");
      }
    }

    bool NRF24Component::write(const void *buf, uint8_t len)
    {
      return this->write(buf, len, false);
    }

    bool NRF24Component::write(const void *buf, uint8_t len, bool multicast)
    {
      this->start_write(buf, len, multicast);
      uint32_t timeout = millis() + 95;
      while (!(this->read_register(nRF24L01::STATUS) & (nRF24L01::TX_DS | nRF24L01::MAX_RT)))
      {
        if (millis() > timeout)
          return false;
        yield();
      }
      this->ce(false);
      uint8_t status = this->read_register(nRF24L01::STATUS);
      this->write_register(nRF24L01::STATUS, nRF24L01::TX_DS | nRF24L01::MAX_RT);
      return status & nRF24L01::TX_DS;
    }

    void NRF24Component::start_write(const void *buf, uint8_t len, bool multicast)
    {
      // Determine the correct command:
      // W_TX_PAYLOAD (0xA0) or W_TX_PAYLOAD_NOACK (0xB0)
      uint8_t command = multicast ? nRF24L01::W_TX_PAYLOAD_NO_ACK : nRF24L01::W_TX_PAYLOAD;

      this->ce(false); // Ensure we aren't in RX mode
      this->write_register(nRF24L01::CONFIG, (this->read_register(nRF24L01::CONFIG) & ~nRF24L01::PRIM_RX));

      begin_transaction_();
      // 1. Send the RAW command byte, NOT masked with W_REGISTER
      this->status_ = this->transfer_byte(command);

      // 2. Transfer the payload array
      this->write_array((const uint8_t *)buf, len);
      end_transaction_();

      this->ce(true); // Pulse CE to start the transmission
    }

    void NRF24Component::write_payload(const void *buf, uint8_t data_len, uint8_t writeType)
    {
      const uint8_t *current = (const uint8_t *)buf;
      uint8_t blank_len = this->dynamic_payloads_enabled_ ? 0 : (this->payload_size_ - data_len);

      begin_transaction_();
      this->transfer_byte(writeType);
      for (uint8_t i = 0; i < data_len; ++i)
      {
        this->transfer_byte(*current++);
      }
      while (blank_len--)
        this->transfer_byte(0);
      end_transaction_();
    }

    bool NRF24Component::write_fast(const void *buf, uint8_t len)
    {
      return this->write_fast(buf, len, false);
    }

    bool NRF24Component::write_fast(const void *buf, uint8_t len, bool multicast)
    {
      this->start_write(buf, len, multicast);
      return true; 
    }

    bool NRF24Component::write_blocking(const void *buf, uint8_t len, uint32_t timeout)
    {
      uint32_t start = millis();
      while (!this->write_fast(buf, len))
      {
        if (millis() - start > timeout)
          return false;
      }
      return this->tx_standby(timeout - (millis() - start));
    }

    // ====================== Pipes ======================

    void NRF24Component::open_writing_pipe(const uint8_t *address)
    {
      memcpy(this->pipe0_writing_address_, address, 5);
      this->write_register(nRF24L01::TX_ADDR, address, 5);
      this->write_register(nRF24L01::RX_ADDR_P0, address, 5);
    }

    void NRF24Component::open_writing_pipe(uint64_t address)
    {
      uint8_t addr[5] = {0};
      for (int i = 4; i >= 0; --i)
      {
        addr[i] = address & 0xFF;
        address >>= 8;
      }
      this->open_writing_pipe(addr);
    }

    void NRF24Component::open_reading_pipe(uint8_t number, const uint8_t *address)
    {
      if (number > 5)
        return; // Hardware limit

      // 1. Correct Register Mapping
      uint8_t reg = nRF24L01::RX_ADDR_P0 + number;

      // 2. Write the Address
      if (number < 2)
      {
        // Pipes 0 and 1 have full 5-byte addresses
        this->write_register(reg, address, this->addr_width_);
      }
      else
      {
        // Pipes 2-5 only store the LSB (1 byte).
        // They inherit the first 4 bytes of Pipe 1.
        this->write_register(reg, address, 1);
      }

      this->write_register(nRF24L01::RX_PW_P0 + number, this->payload_size_); // Set payload size for this pipe
      // 3. Enable the Pipe (Explicitly)
      // Instead of a Read-Modify-Write, consider if you want to FORCE
      // only this pipe open for sniffing stability.
      uint8_t en_reg = this->read_register(nRF24L01::EN_RXADDR);
      this->write_register(nRF24L01::EN_RXADDR, en_reg | BIT(number));
    }

    void NRF24Component::set_address_width(uint8_t a_width)
    {
      a_width = static_cast<uint8_t>(a_width - 2);
      if (a_width)
      {
        write_register(nRF24L01::SETUP_AW, static_cast<uint8_t>(a_width % 4));
        this->addr_width_ = static_cast<uint8_t>((a_width % 4) + 2);
      }
      else
      {
        write_register(nRF24L01::SETUP_AW, static_cast<uint8_t>(0));
        this->addr_width_ = static_cast<uint8_t>(2);
      }
    }

    void NRF24Component::open_reading_pipe(uint8_t number, uint64_t address)
    {
      uint8_t addr[5] = {0};
      for (int i = 4; i >= 0; --i)
      {
        addr[i] = address & 0xFF;
        address >>= 8;
      }
      this->open_reading_pipe(number, addr);
    }

    void NRF24Component::close_reading_pipe(uint8_t pipe)
    {
      this->write_register(nRF24L01::EN_RXADDR,
                           this->read_register(nRF24L01::EN_RXADDR) & ~(1 << pipe));
    }

    // ====================== Configuration ======================

    void NRF24Component::set_pa_level(rf24_pa_dbm_e level, bool lna_enable)
    {
      uint8_t setup = this->read_register(nRF24L01::RF_SETUP) & 0xF8;
      setup |= (level << 1) | (lna_enable ? 1 : 0);
      this->write_register(nRF24L01::RF_SETUP, setup);
    }

    rf24_pa_dbm_e NRF24Component::get_pa_level()
    {
      return (rf24_pa_dbm_e)((this->read_register(nRF24L01::RF_SETUP) & 0x06) >> 1);
    }

    bool NRF24Component::set_rf_data_rate(rf24_datarate_e speed)
    {
      // 0x28 is correct: it masks out Bit 5 (DR_LOW) and Bit 3 (DR_HIGH)
      uint8_t setup = this->read_register(nRF24L01::RF_SETUP) & ~(BIT(nRF24L01::RF_DR_LOW) | BIT(nRF24L01::RF_DR_HIGH));

      if (speed == RF24_250KBPS)
      {
        setup |= BIT(nRF24L01::RF_DR_LOW);
      }
      else if (speed == RF24_2MBPS)
      {
        setup |= BIT(nRF24L01::RF_DR_HIGH);
      }
      // If 1Mbps, both stay 0, which is correct.

      this->write_register(nRF24L01::RF_SETUP, setup);

      // Arduino-style Verification:
      // Read back to ensure the hardware actually accepted the value
      uint8_t verify = this->read_register(nRF24L01::RF_SETUP);
      if (verify != setup)
      {
        return false;
      }

      // Crucial for ESP32/ESP-IDF: Give the radio a moment to settle
      // its clock timing before the next SPI command hits.
      esp_rom_delay_us(200);

      return true;
    }

    rf24_datarate_e NRF24Component::get_rf_data_rate()
    {
      uint8_t setup = this->read_register(nRF24L01::RF_SETUP);
      if (setup & (BIT(nRF24L01::RF_DR_LOW)))
        return RF24_250KBPS;
      if (setup & (BIT(nRF24L01::RF_DR_HIGH)))
        return RF24_2MBPS;
      return RF24_1MBPS;
    }

    void NRF24Component::set_crc_length(rf24_crclength_e length)
    {
      uint8_t config = this->read_register(nRF24L01::CONFIG) & ~(0x0C);
      if (length == RF24_CRC_8)
        config |= 0x08;
      else if (length == RF24_CRC_16)
        config |= 0x0C;
      this->write_register(nRF24L01::CONFIG, config);
    }

    rf24_crclength_e NRF24Component::get_crc_length()
    {
      uint8_t config = this->read_register(nRF24L01::CONFIG) & 0x0C;
      if (config == 0x0C)
        return RF24_CRC_16;
      if (config == 0x08)
        return RF24_CRC_8;
      return RF24_CRC_DISABLED;
    }

    void NRF24Component::disable_crc()
    {
      this->set_crc_length(RF24_CRC_DISABLED);
    }

    void NRF24Component::set_retries(uint8_t delay, uint8_t count)
    {
      this->write_register(nRF24L01::SETUP_RETR, (delay & 0x0F) << 4 | (count & 0x0F));
    }

    void NRF24Component::set_channel(uint8_t channel)
    {
      this->write_register(nRF24L01::RF_CH, channel);
    }

    uint8_t NRF24Component::get_channel()
    {
      return this->read_register(nRF24L01::RF_CH);
    }

    uint8_t NRF24Component::get_payload_size()
    {
      return this->payload_size_;
    }

    uint8_t NRF24Component::get_dynamic_payload_size()
    {
      return this->read_register(nRF24L01::R_RX_PL_WID);
    }

    // ====================== Features ======================

    void NRF24Component::enable_ack_payload()
    {
      this->ack_payloads_enabled_ = true;
      this->write_register(nRF24L01::FEATURE,
                           this->read_register(nRF24L01::FEATURE) | nRF24L01::EN_ACK_PAY);
    }

    void NRF24Component::disable_ack_payload()
    {
      this->ack_payloads_enabled_ = false;
      this->write_register(nRF24L01::FEATURE,
                           this->read_register(nRF24L01::FEATURE) & ~nRF24L01::EN_ACK_PAY);
    }

    void NRF24Component::enable_dynamic_payloads()
    {
      this->dynamic_payloads_enabled_ = true;
      this->write_register(nRF24L01::FEATURE,
                           this->read_register(nRF24L01::FEATURE) | nRF24L01::EN_DPL);
      this->write_register(nRF24L01::DYNPD, 0x3F);
    }

    void NRF24Component::disable_dynamic_payloads()
    {
      this->dynamic_payloads_enabled_ = false;
      this->write_register(nRF24L01::FEATURE,
                           this->read_register(nRF24L01::FEATURE) & ~nRF24L01::EN_DPL);
      this->write_register(nRF24L01::DYNPD, 0);
    }

    void NRF24Component::set_auto_ack(bool enable)
    {
      this->write_register(nRF24L01::EN_AA, enable ? 0x3F : 0);
    }

    void NRF24Component::set_auto_ack(uint8_t pipe, bool enable)
    {
      uint8_t en_aa = this->read_register(nRF24L01::EN_AA);
      if (enable)
        en_aa |= (1 << pipe);
      else
        en_aa &= ~(1 << pipe);
      this->write_register(nRF24L01::EN_AA, en_aa);
    }

    void NRF24Component::write_ack_payload(uint8_t pipe, const void *buf, uint8_t len)
    {
      begin_transaction_();
      this->transfer_byte(nRF24L01::W_ACK_PAYLOAD | (pipe & 0x07));
      this->write_array((const uint8_t *)buf, len);
      end_transaction_();
    }

    bool NRF24Component::is_ack_payload_available()
    {
      return this->read_register(nRF24L01::FIFO_STATUS) & nRF24L01::TX_EMPTY ? false : true;
    }

    // ====================== Power ======================

    void NRF24Component::power_down()
    {
      this->ce(false);
      this->write_register(nRF24L01::CONFIG,
                           this->read_register(nRF24L01::CONFIG) & ~nRF24L01::PWR_UP);
    }

    void NRF24Component::power_up()
    {
      this->write_register(nRF24L01::CONFIG,
                           this->read_register(nRF24L01::CONFIG) | nRF24L01::PWR_UP);
    }

    // ====================== Status & Debug ======================

    uint8_t NRF24Component::what_happened()
    {
      uint8_t status = this->read_register(nRF24L01::STATUS);
      this->write_register(nRF24L01::STATUS, status & 0x70);
      return status & 0x70;
    }

    void NRF24Component::mask_irq(bool tx_ok, bool tx_fail, bool rx_ready)
    {
      uint8_t config = this->read_register(nRF24L01::CONFIG);
      config |= (tx_fail ? 0 : nRF24L01::MASK_TX_DS) |
                (tx_ok ? 0 : nRF24L01::MASK_MAX_RT) |
                (rx_ready ? 0 : nRF24L01::MASK_RX_DR);
      this->write_register(nRF24L01::CONFIG, config);
    }

    rf24_fifo_state_e NRF24Component::is_fifo(bool tx)
    {
      uint8_t status = this->read_register(nRF24L01::FIFO_STATUS);
      // simplified
      return (status & 0x01) ? RF24_FIFO_EMPTY : RF24_FIFO_FULL;
    }

    void NRF24Component::flush_tx()
    {
      begin_transaction_();
      this->transfer_byte(nRF24L01::FLUSH_TX);
      end_transaction_();
    }

    void NRF24Component::flush_rx()
    {
      begin_transaction_();
      this->transfer_byte(nRF24L01::FLUSH_RX);
      end_transaction_();
    }

    void NRF24Component::reuse_tx_pl()
    {
      begin_transaction_();
      this->transfer_byte(nRF24L01::REUSE_TX_PL);
      end_transaction_();
    }

    bool NRF24Component::test_carrier()
    {
      return this->read_register(nRF24L01::RPD) & 1;
    }

    bool NRF24Component::test_rpd()
    {
      return this->read_register(nRF24L01::RPD) & 1;
    }

    bool NRF24Component::tx_standby()
    {
      while (this->read_register(nRF24L01::FIFO_STATUS) & nRF24L01::FIFO_FULL)
      {
        if (this->read_register(nRF24L01::STATUS) & nRF24L01::MAX_RT)
        {
          this->write_register(nRF24L01::STATUS, nRF24L01::MAX_RT);
          this->ce(false);
          return false;
        }
      }
      return true;
    }

    bool NRF24Component::tx_standby(uint32_t timeout, bool startTx)
    {
      if (startTx)
        this->ce(true);
      uint32_t start = millis();
      while (millis() - start < timeout)
      {
        if (this->read_register(nRF24L01::STATUS) & (nRF24L01::TX_DS | nRF24L01::MAX_RT))
          break;
      }
      this->ce(false);
      return (this->read_register(nRF24L01::STATUS) & nRF24L01::TX_DS);
    }

    void NRF24Component::toggle_features()
    {
      begin_transaction_();
      this->transfer_byte(nRF24L01::ACTIVATE);
      this->transfer_byte(0x73);
      end_transaction_();
    }

    void NRF24Component::print_pretty_details()
    {
      ESP_LOGCONFIG(TAG, "nRF24L01+ Details:");
      ESP_LOGCONFIG(TAG, "  STATUS          = 0x%02X", this->read_register(nRF24L01::STATUS));
      ESP_LOGCONFIG(TAG, "  CONFIG          = 0x%02X", this->read_register(nRF24L01::CONFIG));
      ESP_LOGCONFIG(TAG, "  RF_SETUP        = 0x%02X", this->read_register(nRF24L01::RF_SETUP));
      ESP_LOGCONFIG(TAG, "  RF_CH           = %d", this->get_channel());
      ESP_LOGCONFIG(TAG, "  Data Rate       = %s", this->get_rf_data_rate() == RF24_250KBPS ? "250kbps" : this->get_rf_data_rate() == RF24_2MBPS ? "2Mbps"
                                                                                                                                  : "1Mbps");
      // Add more registers if desired
    }

  } // namespace nrf24
} // namespace esphome