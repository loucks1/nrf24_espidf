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

    void NRF24Component::setup_pins_()
    {
      if (this->ce_pin_)
      {
        this->ce_pin_->setup();
        this->ce_pin_->digital_write(false);
      }
    }

    void NRF24Component::setup()
    {
      ESP_LOGI("nrf24", "BUILD VERSION: 2026-04-20-V4");
      this->setup_pins_();

      ESP_LOGI(TAG, "Setting up SPI...");

      this->spi_setup();
      delay(100);

      ESP_LOGI(TAG, "Setting up NRF24...");

      // Now you can talk to the hardware
      if (!this->begin())
      {
        ESP_LOGE(TAG, "NRF24 hardware not responding!");
        this->mark_failed();
        return;
      }
    }

    void NRF24Component::dump_config()
    {
      ESP_LOGCONFIG(TAG, "nRF24L01+ Radio:");
      LOG_PIN("  CE Pin:", this->ce_pin_);
      LOG_SPI_DEVICE(this);
      this->printPrettyDetails();
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
      // Send Write Command (0x20 | reg). Store status.
      this->status_ = this->transfer_byte(nRF24L01::W_REGISTER | (reg & 0x1F));

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

    bool NRF24Component::begin()
    {
      this->setup_pins_();

      this->write_register(nRF24L01::CONFIG, 0x0C); // default
      esp_rom_delay_us(5000);

      this->flush_tx();
      this->flush_rx();

      this->write_register(nRF24L01::STATUS, 0x70);     // clear interrupts
      this->write_register(nRF24L01::SETUP_RETR, 0x5F); // 1500us, 15 retries

      this->setPALevel(this->pa_level_);
      this->set_rf_data_rate(this->data_rate_);
      this->setCRCLength(RF24_CRC_16);
      this->setChannel(this->channel_);
      this->setPayloadSize(this->payload_size_);

      this->powerUp();
      esp_rom_delay_us(5000);

      this->toggle_features();
      this->write_register(nRF24L01::FEATURE, 0);
      this->write_register(nRF24L01::DYNPD, 0);

      return this->is_chip_connected();
    }

    bool NRF24Component::is_chip_connected()
    {
      uint8_t aw = this->read_register(nRF24L01::SETUP_AW);
      return (aw >= 1 && aw <= 3);
    }

    void NRF24Component::set_channel(uint8_t channel)
    {
      this->channel_ = channel;
    }

    void NRF24Component::set_data_rate_str(const std::string &data_rate)
    {
      if (data_rate == "250KBPS")
        this->data_rate_ = RF24_250KBPS;
      else if (data_rate == "2MBPS")
        this->data_rate_ = RF24_2MBPS;
      else
        this->data_rate_ = RF24_1MBPS;
    }

    void NRF24Component::set_pa_level_str(const std::string &pa_level)
    {
      if (pa_level == "MIN")
        this->pa_level_ = RF24_PA_MIN;
      else if (pa_level == "LOW")
        this->pa_level_ = RF24_PA_LOW;
      else if (pa_level == "HIGH")
        this->pa_level_ = RF24_PA_HIGH;
      else
        this->pa_level_ = RF24_PA_MAX;
    }

    void NRF24Component::set_payload_size(uint8_t size)
    {
      this->payload_size_ = std::min(size, (uint8_t)32);
    }

    void NRF24Component::ce(bool level)
    {
      if (this->ce_pin_)
        this->ce_pin_->digital_write(level);
    }

    // ====================== Mode ======================

    void NRF24Component::start_listening()
    {
      this->write_register(nRF24L01::CONFIG,
                           this->read_register(nRF24L01::CONFIG) | nRF24L01::PRIM_RX);
      this->ce(true);
      esp_rom_delay_us(130); // Tstby2a
    }

    void NRF24Component::stop_listening()
    {
      this->ce(false);
      this->write_register(nRF24L01::CONFIG,
                           this->read_register(nRF24L01::CONFIG) & ~nRF24L01::PRIM_RX);
      if (this->ack_payloads_enabled_)
        this->flush_tx();
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
      return !(this->read_register(nRF24L01::FIFO_STATUS) & nRF24L01::RX_EMPTY);
    }

    bool NRF24Component::available(uint8_t *pipe_num)
    {
      uint8_t status = this->read_register(nRF24L01::STATUS);
      if (pipe_num)
        *pipe_num = (status >> nRF24L01::RX_P_NO) & 0x07;
      return (status & nRF24L01::RX_DR);
    }

    void NRF24Component::read(void *buf, uint8_t len)
    {
      this->read_register(nRF24L01::R_RX_PAYLOAD, (uint8_t *)buf, len);
      this->write_register(nRF24L01::STATUS, nRF24L01::RX_DR);
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
      }
      this->ce(false);
      uint8_t status = this->read_register(nRF24L01::STATUS);
      this->write_register(nRF24L01::STATUS, nRF24L01::TX_DS | nRF24L01::MAX_RT);
      return status & nRF24L01::TX_DS;
    }

    void NRF24Component::start_write(const void *buf, uint8_t len, bool multicast)
    {
      this->write_register(nRF24L01::STATUS, nRF24L01::TX_DS | nRF24L01::MAX_RT);
      this->write_payload(buf, len, multicast ? nRF24L01::W_TX_PAYLOAD_NO_ACK : nRF24L01::W_TX_PAYLOAD);
      this->ce(true);
      esp_rom_delay_us(10);
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

    bool NRF24Component::writeFast(const void *buf, uint8_t len)
    {
      return this->writeFast(buf, len, false);
    }

    bool NRF24Component::writeFast(const void *buf, uint8_t len, bool multicast)
    {
      this->start_write(buf, len, multicast);
      return true; // caller should check txStandBy()
    }

    bool NRF24Component::writeBlocking(const void *buf, uint8_t len, uint32_t timeout)
    {
      uint32_t start = millis();
      while (!this->writeFast(buf, len))
      {
        if (millis() - start > timeout)
          return false;
      }
      return this->txStandBy(timeout - (millis() - start));
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
      if (number < 2)
      {
        this->write_register(nRF24L01::RX_ADDR_P0 + number, address, 5);
      }
      else
      {
        this->write_register(nRF24L01::RX_ADDR_P0 + number, address, 1);
      }
      this->write_register(nRF24L01::EN_RXADDR,
                           this->read_register(nRF24L01::EN_RXADDR) | (1 << number));
    }

    void NRF24Component::set_address_width(uint8_t a_width)
    {
      // nRF24L01 supports 3, 4, or 5 bytes.
      // SETUP_AW bits: 01=3bytes, 10=4bytes, 11=5bytes
      if (a_width < 3)
        a_width = 3;
      if (a_width > 5)
        a_width = 5;

      // The register value is width - 2
      // 3 bytes -> 1, 4 bytes -> 2, 5 bytes -> 3
      this->write_register(nRF24L01::SETUP_AW, (uint8_t)(a_width - 2));
      this->addr_width_ = a_width; // Store this for use in open_reading_pipe
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

    void NRF24Component::setPALevel(rf24_pa_dbm_e level, bool lna_enable)
    {
      uint8_t setup = this->read_register(nRF24L01::RF_SETUP) & 0xF8;
      setup |= (level << 1) | (lna_enable ? 1 : 0);
      this->write_register(nRF24L01::RF_SETUP, setup);
    }

    rf24_pa_dbm_e NRF24Component::getPALevel()
    {
      return (rf24_pa_dbm_e)((this->read_register(nRF24L01::RF_SETUP) & 0x06) >> 1);
    }

    bool NRF24Component::set_rf_data_rate(rf24_datarate_e speed)
    {
      uint8_t setup = this->read_register(nRF24L01::RF_SETUP) & ~(0x28);
      if (speed == RF24_250KBPS)
        setup |= (1 << nRF24L01::RF_DR_LOW);
      else if (speed == RF24_2MBPS)
        setup |= (1 << nRF24L01::RF_DR_HIGH);
      this->write_register(nRF24L01::RF_SETUP, setup);
      return true;
    }

    rf24_datarate_e NRF24Component::getDataRate()
    {
      uint8_t setup = this->read_register(nRF24L01::RF_SETUP);
      if (setup & (1 << nRF24L01::RF_DR_LOW))
        return RF24_250KBPS;
      if (setup & (1 << nRF24L01::RF_DR_HIGH))
        return RF24_2MBPS;
      return RF24_1MBPS;
    }

    void NRF24Component::setCRCLength(rf24_crclength_e length)
    {
      uint8_t config = this->read_register(nRF24L01::CONFIG) & ~(0x0C);
      if (length == RF24_CRC_8)
        config |= 0x08;
      else if (length == RF24_CRC_16)
        config |= 0x0C;
      this->write_register(nRF24L01::CONFIG, config);
    }

    rf24_crclength_e NRF24Component::getCRCLength()
    {
      uint8_t config = this->read_register(nRF24L01::CONFIG) & 0x0C;
      if (config == 0x0C)
        return RF24_CRC_16;
      if (config == 0x08)
        return RF24_CRC_8;
      return RF24_CRC_DISABLED;
    }

    void NRF24Component::disableCRC()
    {
      this->setCRCLength(RF24_CRC_DISABLED);
    }

    void NRF24Component::setRetries(uint8_t delay, uint8_t count)
    {
      this->write_register(nRF24L01::SETUP_RETR, (delay & 0x0F) << 4 | (count & 0x0F));
    }

    void NRF24Component::setChannel(uint8_t channel)
    {
      this->write_register(nRF24L01::RF_CH, channel);
    }

    uint8_t NRF24Component::getChannel()
    {
      return this->read_register(nRF24L01::RF_CH);
    }

    void NRF24Component::setPayloadSize(uint8_t size)
    {
      this->payload_size_ = std::min((uint8_t)size, (uint8_t)32);
    }

    uint8_t NRF24Component::getPayloadSize()
    {
      return this->payload_size_;
    }

    uint8_t NRF24Component::getDynamicPayloadSize()
    {
      return this->read_register(nRF24L01::R_RX_PL_WID);
    }

    // ====================== Features ======================

    void NRF24Component::enableAckPayload()
    {
      this->ack_payloads_enabled_ = true;
      this->write_register(nRF24L01::FEATURE,
                           this->read_register(nRF24L01::FEATURE) | nRF24L01::EN_ACK_PAY);
    }

    void NRF24Component::disableAckPayload()
    {
      this->ack_payloads_enabled_ = false;
      this->write_register(nRF24L01::FEATURE,
                           this->read_register(nRF24L01::FEATURE) & ~nRF24L01::EN_ACK_PAY);
    }

    void NRF24Component::enableDynamicPayloads()
    {
      this->dynamic_payloads_enabled_ = true;
      this->write_register(nRF24L01::FEATURE,
                           this->read_register(nRF24L01::FEATURE) | nRF24L01::EN_DPL);
      this->write_register(nRF24L01::DYNPD, 0x3F);
    }

    void NRF24Component::disableDynamicPayloads()
    {
      this->dynamic_payloads_enabled_ = false;
      this->write_register(nRF24L01::FEATURE,
                           this->read_register(nRF24L01::FEATURE) & ~nRF24L01::EN_DPL);
      this->write_register(nRF24L01::DYNPD, 0);
    }

    bool NRF24Component::isPVariant()
    {
      return this->_is_p_variant;
    }

    void NRF24Component::setAutoAck(bool enable)
    {
      this->write_register(nRF24L01::EN_AA, enable ? 0x3F : 0);
    }

    void NRF24Component::setAutoAck(uint8_t pipe, bool enable)
    {
      uint8_t en_aa = this->read_register(nRF24L01::EN_AA);
      if (enable)
        en_aa |= (1 << pipe);
      else
        en_aa &= ~(1 << pipe);
      this->write_register(nRF24L01::EN_AA, en_aa);
    }

    void NRF24Component::writeAckPayload(uint8_t pipe, const void *buf, uint8_t len)
    {
      begin_transaction_();
      this->transfer_byte(nRF24L01::W_ACK_PAYLOAD | (pipe & 0x07));
      this->write_array((const uint8_t *)buf, len);
      end_transaction_();
    }

    bool NRF24Component::isAckPayloadAvailable()
    {
      return this->read_register(nRF24L01::FIFO_STATUS) & nRF24L01::TX_EMPTY ? false : true;
    }

    // ====================== Power ======================

    void NRF24Component::powerDown()
    {
      this->ce(false);
      this->write_register(nRF24L01::CONFIG,
                           this->read_register(nRF24L01::CONFIG) & ~nRF24L01::PWR_UP);
    }

    void NRF24Component::powerUp()
    {
      this->write_register(nRF24L01::CONFIG,
                           this->read_register(nRF24L01::CONFIG) | nRF24L01::PWR_UP);
    }

    // ====================== Status & Debug ======================

    uint8_t NRF24Component::whatHappened()
    {
      uint8_t status = this->read_register(nRF24L01::STATUS);
      this->write_register(nRF24L01::STATUS, status & 0x70);
      return status & 0x70;
    }

    void NRF24Component::maskIRQ(bool tx_ok, bool tx_fail, bool rx_ready)
    {
      uint8_t config = this->read_register(nRF24L01::CONFIG);
      config |= (tx_fail ? 0 : nRF24L01::MASK_TX_DS) |
                (tx_ok ? 0 : nRF24L01::MASK_MAX_RT) |
                (rx_ready ? 0 : nRF24L01::MASK_RX_DR);
      this->write_register(nRF24L01::CONFIG, config);
    }

    rf24_fifo_state_e NRF24Component::isFifo(bool tx)
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

    bool NRF24Component::testCarrier()
    {
      return this->read_register(nRF24L01::RPD) & 1;
    }

    bool NRF24Component::testRPD()
    {
      return this->read_register(nRF24L01::RPD) & 1;
    }

    bool NRF24Component::txStandBy()
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

    bool NRF24Component::txStandBy(uint32_t timeout, bool startTx)
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

    void NRF24Component::printPrettyDetails()
    {
      ESP_LOGI(TAG, "nRF24L01+ Details:");
      ESP_LOGI(TAG, "  STATUS          = 0x%02X", this->read_register(nRF24L01::STATUS));
      ESP_LOGI(TAG, "  CONFIG          = 0x%02X", this->read_register(nRF24L01::CONFIG));
      ESP_LOGI(TAG, "  RF_SETUP        = 0x%02X", this->read_register(nRF24L01::RF_SETUP));
      ESP_LOGI(TAG, "  RF_CH           = %d", this->getChannel());
      ESP_LOGI(TAG, "  Data Rate       = %s", this->getDataRate() == RF24_250KBPS ? "250kbps" : this->getDataRate() == RF24_2MBPS ? "2Mbps"
                                                                                                                                  : "1Mbps");
      // Add more registers if desired
    }

  } // namespace nrf24
} // namespace esphome