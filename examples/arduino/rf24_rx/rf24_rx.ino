// Arduino (TMRh20 RF24) RECEIVER - interoperability example
// Pair with either transmitter: arduino/rf24_tx (RF24) or
// esphome/nrf24_tx.yaml (ESPHome, nrf24_espidf component).
// All four examples share the same settings and payload format -
// see examples/README.md.
//
// Based on the TMRh20 RF24 "GettingStarted" example, role hard-coded to RX
// so 100+ packets can be counted unattended (no serial T/R switching).
//
// Board: ESP32-WROOM-32 DevKit (esp32dev), Arduino core, RF24 library.
// Wiring (same both nodes): SCK=GPIO18 MOSI=GPIO23 MISO=GPIO19 (default VSPI)
//                            CE=GPIO22 CSN=GPIO21, IRQ not connected.
// Settings: channel 76, 1 Mbps, PA_LOW, fixed 32-byte payload,
// dynamic payloads OFF, auto-ack ON, CRC-16, retries (5,15),
// pipe 1 address bytes (written in this order): E1 F0 F0 F0 F0

#include <SPI.h>
// NOTE: no #include <nRF24L01.h> here on purpose - RF24.h pulls it in from
// inside the RF24 library. Including it directly makes the Arduino Cloud
// compiler pick NRFLite's identically-named header instead, which conflicts.
#include <RF24.h>

RF24 radio(22, 21);  // CE, CSN

const uint8_t address[5] = {0xE1, 0xF0, 0xF0, 0xF0, 0xF0};

struct Payload {
  uint32_t seq;
  char text[28];
};
static_assert(sizeof(Payload) == 32, "Payload must be exactly 32 bytes");

uint32_t rxCount = 0;
uint32_t expectedSeq = 0;  // 0 = not yet synced to the TX sequence

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println(F("NRF24 INTEROP RX - Arduino RF24 example"));

  if (!radio.begin()) {
    Serial.println(F("FATAL: radio.begin() failed - wiring/power problem"));
    while (1) { delay(1000); }
  }
  if (!radio.isChipConnected()) {
    Serial.println(F("FATAL: chip not connected - check SPI wiring and 3.3V"));
    while (1) { delay(1000); }
  }

  radio.setChannel(76);
  radio.setDataRate(RF24_1MBPS);
  radio.setPALevel(RF24_PA_LOW);
  radio.setPayloadSize(sizeof(Payload));
  radio.disableDynamicPayloads();
  radio.setCRCLength(RF24_CRC_16);
  radio.setAutoAck(true);
  radio.setRetries(5, 15);

  radio.openReadingPipe(1, address);
  radio.startListening();

  radio.printPrettyDetails();
  Serial.println(F("RX ready - listening"));
}

void loop() {
  if (radio.available()) {
    Payload p{};
    radio.read(&p, sizeof(p));
    rxCount++;

    if (expectedSeq != 0 && p.seq != expectedSeq) {
      Serial.printf("*** GAP: expected seq=%lu, got seq=%lu (missed %ld) ***\n",
                    (unsigned long)expectedSeq, (unsigned long)p.seq,
                    (long)(p.seq - expectedSeq));
    }
    expectedSeq = p.seq + 1;

    Serial.printf("RX count=%lu seq=%lu text=%.28s\n",
                  (unsigned long)rxCount, (unsigned long)p.seq, p.text);

    if (rxCount == 100) {
      Serial.println(F("*** 100 PACKETS RECEIVED - check log above for GAP lines ***"));
    }
  }
}
