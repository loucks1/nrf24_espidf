# Interoperability examples

Four paired examples that reproduce the RF24 interoperability test described in the main README: pick one transmitter and one receiver from opposite platforms, flash both, and compare the logs.

|  | Transmitter | Receiver |
| :--- | :--- | :--- |
| Arduino (TMRh20 RF24) | `arduino/rf24_tx/rf24_tx.ino` | `arduino/rf24_rx/rf24_rx.ino` |
| ESPHome (this component) | `esphome/nrf24_tx.yaml` | `esphome/nrf24_rx.yaml` |

All four use the same settings (channel 76, 1 Mbps, PA_LOW, CRC-16, 5-byte address `E1 F0 F0 F0 F0`, fixed 32-byte payload, auto-ack on, retries (5, 15)) and the same payload format: a little-endian `uint32` sequence number followed by `HELLO_NRF24`, zero-padded to 32 bytes.

The receiver logs each packet with its sequence number and flags any gap; the transmitter reports success or failure per packet. A clean run is 100+ consecutive packets with no gaps on the receiver and no failures on the transmitter. Short clusters of failures can occur on channel 76, which overlaps the Wi-Fi band; moving both nodes to a channel above the Wi-Fi band (e.g. 110) avoids that.

ESPHome notes: the examples apply every radio setting explicitly in `on_boot`, so what must match the Arduino side is visible in one place. On the receiving example, reception runs through `add_on_data_callback` — the component only delivers packets when a callback is registered.

## Troubleshooting

If both sides show correct register dumps but nothing arrives, power-cycle both nodes before debugging code. The nRF24L01+ keeps its register state across ESP resets and reflashes — it is not reset when the ESP is — and a receiver can report correct registers and "listening" while not actually receiving. Both failure modes seen during testing (a transmitter that failed 100% of writes after being flashed over a receiver firmware, and a receiver that heard nothing while reporting a perfect configuration) were cleared by a reset or power cycle of the affected radio's node, with no code change.
