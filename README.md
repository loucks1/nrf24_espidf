# ESPHome nRF24L01 Component

This component provides a bridge between **ESPHome** and the **nRF24L01+** 2.4GHz wireless transceiver. It abstracts the SPI communication and provides a high-level C++ API for sending and receiving data packets while staying integrated with the ESPHome lifecycle.

## Features
* **Full SPI Integration:** Uses the standard ESPHome SPI bus.
* **Configurable Power Levels:** Supports `MIN`, `LOW`, `HIGH`, and `MAX` power settings.
* **Variable Data Rates:** Supports 250Kbps, 1Mbps, and 2Mbps.
* **Payload Management:** Support for dynamic payloads and ACK payloads.
* **Callback System:** Easily hook into incoming data events via `add_on_data_callback`.

---

## Hardware Setup

| nRF24L01 Pin | ESP32/ESP8266 Pin | Notes |
| :--- | :--- | :--- |
| **VCC** | 3.3V | **Important:** Requires a stable 3.3V source (use a capacitor). |
| **GND** | GND | |
| **CE** | Any GPIO | Configured via `ce_pin`. |
| **CSN** | Any GPIO | Configured via `cs_pin` in the SPI block. |
| **SCK** | SPI SCK | Hardware SPI Clock. |
| **MOSI** | SPI MOSI | Hardware SPI Master Out Slave In. |
| **MISO** | SPI MISO | Hardware SPI Master In Slave Out. |

---

## YAML Configuration

Add the following to your ESPHome configuration file:

```yaml
spi:
  clk_pin: GPIO18
  mosi_pin: GPIO23
  miso_pin: GPIO19

nrf24:
  id: my_nrf24
  cs_pin: GPIO5
  ce_pin: GPIO17
  channel: 76             # Optional: 0-125
  rf_data_rate: 1MBPS    # Optional: 250KBPS, 1MBPS, 2MBPS
  pa_level: MAX          # Optional: MIN, LOW, HIGH, MAX