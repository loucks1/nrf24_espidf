# ESPHome nRF24L01 Component

This component provides a bridge between **ESPHome** and the **nRF24L01+** 2.4GHz wireless transceiver. It abstracts the SPI communication and provides a high-level C++ API for sending and receiving data packets while staying integrated with the ESPHome lifecycle.

## Features
* **Full SPI Integration:** Uses the standard ESPHome SPI bus.
* **Configurable Power Levels:** Supports `MIN`, `LOW`, `HIGH`, and `MAX` power settings.
* **Variable Data Rates:** Supports 250Kbps, 1Mbps, and 2Mbps.
* **Payload Management:** Support for dynamic payloads and ACK payloads.
* **Callback System:** Easily hook into incoming data events via `add_on_data_callback`.

## Interoperability with Arduino (TMRh20 RF24)

This component has been tested on hardware against the [TMRh20 RF24 library](https://github.com/nRF24/RF24) running on Arduino, in both directions:

- **RF24 (Arduino) transmitting → this component (ESPHome) receiving**
- **This component (ESPHome) transmitting → RF24 (Arduino) receiving**

An Arduino↔Arduino run with the same settings served as a control. All runs used one shared configuration, so the two libraries were compared on identical terms:

| Setting | Value |
| :--- | :--- |
| Channel | 76 (2476 MHz) |
| Data rate | 1 Mbps |
| PA level | LOW |
| CRC | 16-bit |
| Address | 5 bytes, `E1 F0 F0 F0 F0` |
| Payload | Fixed 32 bytes, dynamic payloads off |
| Auto-ack | On, retries (5, 15) |

Payload format for the test and the examples: a little-endian `uint32` sequence number followed by the text `HELLO_NRF24`, zero-padded to 32 bytes.

Results (2026-11-23, ESP32 nodes ~5 ft apart):

- Control, RF24 ↔ RF24: **187 consecutive packets, zero loss**
- RF24 TX → ESPHome RX: **131 consecutive packets with no sequence gaps** (the only losses in the run were one brief cluster matching TX-side failures, consistent with interference — channel 76 overlaps the Wi-Fi band)
- ESPHome TX → RF24 RX: **184+ consecutive packets, every transmission acknowledged, zero failures**

Both directions were re-run successfully on 2026-11-26 against the current `main`, with the same two nodes reflashed into the opposite roles between runs — the ESPHome node transmitted in one run and received in the other, and the Arduino node did the reverse. Paired, ready-to-flash examples for both platforms are in [`examples/`](examples/). [`examples/README.md`](examples/README.md) has the pairing table, how to judge a run, and the note about power-cycling a radio whose registers survived a reflash.

Scope of this testing: fixed 32-byte payloads with dynamic payloads off, and single-role nodes (a node is a dedicated transmitter or receiver for the whole boot; the role changes above were done by reflashing between runs). Other data rates, dynamic payloads, and listen-then-transmit role swapping within one boot have not been hardware-tested for interop. The examples use the byte-array address form so the on-air byte order is visible in the code. The `uint64_t` overloads write those same register bytes for the little-endian value of that array (`0xF0F0F0F0E1ULL` for `E1 F0 F0 F0 F0`); see [Addresses](#addresses).

---

## Addresses

`open_writing_pipe` and `open_reading_pipe` each have a byte-array overload and a `uint64_t` overload.

The byte-array overloads write bytes in array order. `open_writing_pipe(const uint8_t *)` writes five bytes to `TX_ADDR` and `RX_ADDR_P0`. `open_reading_pipe` writes `addr_width_` bytes (default 5) for pipes 0 and 1, and the first byte only for pipes 2–5. Pipes 2–5 share the remaining address bytes with pipe 1.

The `uint64_t` overloads expand the integer into a 5-byte buffer, low byte at index 0, then call the byte-array overload. That is the same LSB-first order as Arduino RF24. `open_writing_pipe(0xF0F0F0F0E1ULL)` and the byte array `{0xE1, 0xF0, 0xF0, 0xF0, 0xF0}` both program `E1 F0 F0 F0 F0`. For pipes 2–5 the stored byte is index 0, the low byte of the integer.

### `uint64_t` addresses written before the LSB-first change

Those overloads used to fill the same 5-byte buffer from the other end: the low byte of the integer landed at index 4. A `uint64_t` that depended on that order now programs a different on-air address. Two nodes that both run this component and pass the same `uint64_t` still agree with each other. A node still on the old firmware does not agree with current `main` when the address bytes are not a palindrome (`0x5555555555ULL` is the same either way). A constant that was byte-reversed to compensate for the old order should be written in the RF24 order above.

The byte-array overloads write the same bytes they always did. To keep the old on-air bytes from a `uint64_t` call, pass that byte sequence to the byte-array overload, or pass the `uint64_t` whose LSB-first expansion is the old 5-byte image. That integer is the old five bytes reversed: the old index 0 becomes the new low byte, through old index 4 as the new high byte. Pipes 2–5 follow the same reversal, because they store index 0.

`0x060406ULL` used to be written `00 00 06 04 06`. `0x0604060000ULL` writes those bytes now. The three data bytes `06 04 06` are a palindrome, so reversing them leaves them in place and the two leading `00` bytes move to the low end of the integer, which is a 16-bit left shift. A 3-byte value that is not a palindrome does not survive that shift. `0x010203ULL` used to be written `00 00 01 02 03`; `0x0102030000ULL` writes `00 00 03 02 01`, and the reversed integer `0x0302010000ULL` writes the old bytes. The same reversal covers a full 5-byte value: `0xE1F0F0F0F0ULL` used to be written `E1 F0 F0 F0 F0`, and `0xF0F0F0F0E1ULL` writes that sequence now.

---

## Hardware Setup

| nRF24L01 Pin | ESP32/ESP8266 Pin | Notes |
| :--- | :--- | :--- |
| **VCC** | 3.3V | **Important:** Requires a stable 3.3V source (use a capacitor). |
| **GND** | GND | |
| **CE** | Any GPIO | Configured via `ce_pin`. |
| **CSN** | Any GPIO | Configured via `cs_pin` on the `nrf24:` block. |
| **SCK** | SPI SCK | Hardware SPI Clock. |
| **MOSI** | SPI MOSI | Hardware SPI Master Out Slave In. |
| **MISO** | SPI MISO | Hardware SPI Master In Slave Out. |

---

## YAML Configuration

Load the component from this repository at ref `main`:

```yaml
external_components:
  - source: github://loucks1/nrf24_espidf@main
    components: [nrf24]
```

`github://loucks1/nrf24_espidf@main` is ESPHome's shorthand for that GitHub repository and ref. The component is `components/nrf24`, the folder ESPHome searches when the repository has a top-level `components` directory.

Add the bus and the radio:

```yaml
spi:
  clk_pin: GPIO18
  mosi_pin: GPIO23
  miso_pin: GPIO19

nrf24:
  id: my_nrf24
  cs_pin: GPIO5                       # Required
  ce_pin: GPIO17                      # Required
  channel: 76                         # Optional, default 76, range 0-125
  rf_data_rate: 1MBPS                 # Optional, default 1MBPS: 250KBPS, 1MBPS, 2MBPS
  pa_level: MAX                       # Optional, default MAX: MIN, LOW, HIGH, MAX
  high_frequency_polling: false       # Optional, default false
```

`spi_id` selects the bus when the configuration has more than one `spi:` entry. `data_rate` is the SPI clock and `spi_mode` is the SPI mode; with both omitted the device stays at 1 MHz, mode 0, MSB first. `rf_data_rate` is the on-air rate.

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

Starting a transmission sets the driver's `is_listening_` state to `false`, so
receive callbacks are inactive until `start_listening()` is called again.
Applications must explicitly return the half-duplex radio to listening mode
between transmissions when they need RX.

`stop_listening()` closes reading pipes 0 through 5. Auto-ack payloads are
received on pipe 0, so pipe 0 has to be enabled again before a transmit that
follows `stop_listening()`. `open_writing_pipe` does that after it writes
`TX_ADDR` and `RX_ADDR_P0`: it sets `ERX_P0` in `EN_RXADDR` and leaves the other
pipe-enable bits as they are. Both the byte-array and the `uint64_t` writing-pipe
overloads take this path. The `stop_listening(const uint8_t *)` overload calls
`open_writing_pipe` on the address you pass. `start_listening()` leaves
`EN_RXADDR` unchanged. With pipe 0 left closed, `write()` runs to `MAX_RT` when the
peer has already received the packet, because this node never hears the ACK.
Listen-then-transmit inside one boot is outside the hardware interop runs above.

---

## Startup

`setup()` calls `soft_reset()`. The nRF24L01+ keeps its registers across an ESP
reboot or reflash, and `power_down()` / `power_up()` leave those registers in
place, so `soft_reset()` writes the configuration the component stores:

* Clears the IRQ flags in `STATUS`, then flushes the RX and TX FIFOs.
* Channel, RF data rate, and PA level, from the YAML values (defaults 76, 1 Mbps, and `MAX`). PA level is written with the LNA bit enabled.
* `CONFIG`, as a whole register: powered up, IRQ masks clear, `PRIM_RX` clear, and CRC from `crc_length_`. The default is 16-bit CRC (`EN_CRC` and `CRCO`). 8-bit CRC sets `EN_CRC` only. `disable_crc()` leaves both bits clear. `start_listening()` is what sets `PRIM_RX`.
* Address width (default 5), payload size on all six pipes (default 32), auto-ack (default on, `EN_AA` = `0x3F`), retries (default delay 0, count 3), and dynamic payloads (default off).

`set_crc_length`, `set_retries`, `set_auto_ack(bool)`, `set_address_width`,
`set_payload_size`, `enable_dynamic_payloads`, and `disable_dynamic_payloads`
store what they were given, and a later `soft_reset()` writes those stored
values. `set_auto_ack(uint8_t pipe, bool)` does not update the stored auto-ack
flag, so the next `soft_reset()` writes the all-pipes value from the last
`set_auto_ack(bool)`, which defaults to on. `set_channel`, `set_rf_data_rate`,
and `set_pa_level` update the radio immediately and leave the stored YAML copies
alone, so the next `soft_reset()` writes the YAML values again. Pipe addresses
and `EN_RXADDR` stay as the chip already holds them; open pipes after boot as
usual.

If the chip does not answer, `soft_reset()` returns with `hardware_initialized_`
left clear. Every 15 seconds `loop()` checks the connection. While
`hardware_initialized_` is clear, that check calls `soft_reset()` first.