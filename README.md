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
  high_frequency_polling: false
```

## Receive polling and transmission

The component reads up to three queued packets per loop. Set
`high_frequency_polling: true` to request ESPHome's high-frequency loop mode while
the radio is initialized, listening, and has data callbacks. This option defaults
to `false`; enabling it increases CPU activity and power consumption. The request
is released on the next component loop when callback-based reception is inactive,
including after `power_down()`, or the option is disabled. Batches are bounded by
the hardware FIFO depth so continuous traffic cannot monopolize a loop iteration.

`write()` waits for the TX-complete or maximum-retry interrupt, with a 95 ms
timeout. Failure lowers CE and flushes the unsent TX payload without discarding
queued RX packets. Multicast writes enable `EN_DYN_ACK` before using the NO_ACK
payload command. Successful NO_ACK transmission does not confirm reception by
the destination.

Both `tx_standby()` overloads wait for the TX FIFO to become empty, not merely
for one packet's TX-complete interrupt. The no-argument overload uses a 95 ms
timeout; the other uses the supplied timeout in milliseconds. With `startTx:
true`, it switches out of RX mode and starts the queued transmission. Both
overloads lower CE and clear TX interrupts before returning. On maximum retries
or timeout, they return `false` and flush unsent TX data, preserving queued RX
data. Neither overload retries a payload after `MAX_RT`. `write_blocking()`
deducts the time spent loading the payload from its remaining wait budget.

`mask_irq(tx_ok, tx_fail, rx_ready)` uses `true` to mask the corresponding IRQ
and `false` to unmask it, preserving the other CONFIG bits. `is_fifo(tx)` selects
TX with `true` or RX with `false` and reports empty, occupied, full, or invalid.
`is_ack_payload_available()` checks for a payload in the RX FIFO.

Existing configurations remain valid. Starting a transmission sets the driver's
`is_listening_` state to `false`, so receive callbacks are inactive until
`start_listening()` is called again. Applications must explicitly return the
half-duplex radio to listening mode between transmissions when they need RX.