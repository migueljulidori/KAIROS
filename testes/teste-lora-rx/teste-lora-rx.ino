/*
 * Kairós — teste-lora-rx (Arduino Uno)
 *   So escuta o LoRa e imprime QUALQUER pacote que chegar.
 *   Sem motores, sem servos. Serve para separar problema de radio
 *   de problema do kairos-receptor.
 *
 * Ligacao (igual kairos-receptor):
 *   LoRa: VCC 3V3, GND, SCK 13, MISO 12, MOSI 11, NSS 10, RST 9, DIO0 solto
 *
 * Serial 9600
 *   PKT ... = chegou pacote (mostra bytes, RSSI, SNR)
 *   "sem pacote" a cada 2 s = radio ligado, mas nada chegando
 */

#include <SPI.h>
#include <LoRa.h>

#define LORA_SS   10
#define LORA_RST  9
#define LORA_DIO0 -1
const long LORA_FREQ = 115E6;

long pacotes = 0;
unsigned long ultimoPacoteMs = 0;
unsigned long ultimoAvisoMs = 0;

void setup() {
  Serial.begin(9600);
  Serial.println();
  Serial.println(F("=== KAIROS teste-lora-rx ==="));

  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println(F("LoRa FALHOU — confira fios SPI / 3V3 / NSS 10 / RST 9"));
    while (true) delay(1000);
  }
  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(250E3);
  LoRa.setCodingRate4(5);
  LoRa.enableCrc();

  Serial.println(F("LoRa RX OK — 433 MHz SF7 BW250 CR4/5 CRC. Escutando..."));
}

void loop() {
  int tam = LoRa.parsePacket();
  if (tam > 0) {
    pacotes++;
    ultimoPacoteMs = millis();
    Serial.print(F("PKT #")); Serial.print(pacotes);
    Serial.print(F("  tam=")); Serial.print(tam);
    Serial.print(F("  bytes="));
    while (LoRa.available()) {
      uint8_t b = LoRa.read();
      if (b < 0x10) Serial.print('0');
      Serial.print(b, HEX);
      Serial.print(' ');
    }
    Serial.print(F(" RSSI=")); Serial.print(LoRa.packetRssi());
    Serial.print(F(" SNR="));  Serial.println(LoRa.packetSnr());
  }

  if (millis() - ultimoPacoteMs > 2000 && millis() - ultimoAvisoMs > 2000) {
    ultimoAvisoMs = millis();
    Serial.print(F("sem pacote (total ")); Serial.print(pacotes);
    Serial.println(F(")"));
  }
}
