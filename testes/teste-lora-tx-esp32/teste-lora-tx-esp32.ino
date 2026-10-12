/*
 * Kairós — teste-lora-tx-esp32 (luva / ESP32 DevKit)
 *   So transmite. Sem flex, sem MPU. Par do teste-lora-rx-uno.
 *
 * Pacote (texto, facil de ver no RX):  "KAIROS <contador>"
 * Envia 1 pacote a cada 500 ms.
 *
 * LoRa SX1278 433MHz (igual kairos-luva-tx):
 *   VCC 3V3  GND  SCK 18  MISO 19  MOSI 23  NSS 5  RST 2  DIO0 15
 *
 * Radio (TEM que ser igual no RX):
 *   433 MHz | SF7 | BW 125 kHz | CR 4/5 | CRC | sync 0x12 | 17 dBm
 *
 * Serial 115200
 *   d = dump dos registradores do SX1278 (conferir se a config gravou)
 *
 * Valores esperados no dump (registrador: valor):
 *   0x01: 0x81 ou 0x83 (LoRa standby/TX)   0x06 0x07 0x08: 0x6C 0x40 0x00 (433 MHz)
 *   0x1D: 0x72 (BW125 CR4/5 explicito)     0x1E: 0x74 (SF7 + CRC)
 *   0x39: 0x12 (sync word)                 0x42: 0x12 (versao SX1278)
 */

#include <WiFi.h>
#include <SPI.h>
#include <LoRa.h>

#define LORA_SS   5
#define LORA_RST  2
#define LORA_DIO0 15

const long LORA_FREQ  = 433E6;
const int  LORA_SF    = 7;
const long LORA_BW    = 125E3;
const int  LORA_CR    = 5;
const int  LORA_SYNC  = 0x12;
const int  LORA_POWER = 17;

const unsigned long PERIODO_MS = 500;

long contador = 0;
long falhas = 0;
unsigned long ultimoTxMs = 0;

void setup() {
  Serial.begin(115200);
  delay(400);
  WiFi.mode(WIFI_OFF);

  Serial.println();
  Serial.println(F("=== KAIROS teste-lora-tx-esp32 ==="));

  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  LoRa.setSPIFrequency(1E6);
  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println(F("LoRa FALHOU no begin (SPI / 3V3 / NSS 5 / RST 2)"));
    while (true) delay(1000);
  }
  LoRa.setSpreadingFactor(LORA_SF);
  LoRa.setSignalBandwidth(LORA_BW);
  LoRa.setCodingRate4(LORA_CR);
  LoRa.setSyncWord(LORA_SYNC);
  LoRa.enableCrc();
  LoRa.setTxPower(LORA_POWER);

  Serial.println(F("LoRa TX OK — 433 MHz SF7 BW125 CR4/5 CRC sync 0x12 17 dBm"));
  Serial.println(F("Tecla d = dump de registradores."));
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'd' || c == 'D') {
      Serial.println(F("--- dump SX1278 ---"));
      LoRa.dumpRegisters(Serial);
      Serial.println(F("--- fim ---"));
    }
  }

  if (millis() - ultimoTxMs < PERIODO_MS) return;
  ultimoTxMs = millis();

  contador++;
  unsigned long t0 = micros();
  bool ok = LoRa.beginPacket();
  if (ok) {
    LoRa.print("KAIROS ");
    LoRa.print(contador);
    ok = LoRa.endPacket();
  }
  unsigned long dur = micros() - t0;
  if (!ok) falhas++;

  Serial.print(F("TX #")); Serial.print(contador);
  Serial.print(ok ? F("  ok") : F("  FALHOU"));
  Serial.print(F("  tempo=")); Serial.print(dur / 1000.0f, 1); Serial.print(F(" ms"));
  Serial.print(F("  falhas=")); Serial.println(falhas);
}
