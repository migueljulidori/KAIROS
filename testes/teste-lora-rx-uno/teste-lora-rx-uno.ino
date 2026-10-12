/*
 * Kairós — teste-lora-rx-uno (Arduino Uno)
 *   So recebe. Sem motores, sem servos. Par do teste-lora-tx-esp32.
 *
 * LoRa SX1278 433MHz (igual kairos-receptor):
 *   VCC 3V3  GND  SCK 13  MISO 12  MOSI 11  NSS 10  RST 9
 *   DIO0 -> D2 so se USAR_DIO0 = 1
 *
 * Radio (TEM que ser igual no TX):
 *   433 MHz | SF7 | BW 125 kHz | CR 4/5 | CRC | sync 0x12
 *
 * Dois modos de recepcao:
 *   USAR_DIO0 0 = polling com parsePacket() (DIO0 nao precisa estar ligado)
 *   USAR_DIO0 1 = recepcao continua por interrupcao (DIO0 ligado no D2)
 *
 * Serial 9600 — a cada 1 s mostra:
 *   pkts = pacotes recebidos
 *   RSSI ar min/max = energia de radio no ar (dBm), mesmo sem decodificar
 *       parado sem TX: ~ -110 a -125
 *       com o ESP transmitindo perto: o max deve subir para ~ -30 a -80
 *   Se o RSSI SOBE mas pkts=0 -> o sinal chega, mas nao decodifica (config/SPI)
 *   Se o RSSI NAO sobe          -> nenhum sinal chegando nesse radio
 *
 *   d = dump dos registradores (comparar com os valores esperados abaixo)
 *   0x06 0x07 0x08: 0x6C 0x40 0x00 | 0x1D: 0x72 | 0x1E: 0x74 | 0x39: 0x12 | 0x42: 0x12
 */

#include <SPI.h>
#include <LoRa.h>

#define USAR_DIO0 0

#define LORA_SS   10
#define LORA_RST  9
#if USAR_DIO0
#define LORA_DIO0 2
#else
#define LORA_DIO0 -1
#endif

const long LORA_FREQ = 433E6;
const int  LORA_SF   = 7;
const long LORA_BW   = 125E3;
const int  LORA_CR   = 5;
const int  LORA_SYNC = 0x12;

long pacotes = 0;
int rssiMin = 0, rssiMax = -200;
unsigned long ultimoStatusMs = 0;

#if USAR_DIO0
volatile bool chegou = false;
volatile int tamRx = 0;
char bufRx[64];

void aoReceber(int tam) {
  int n = 0;
  while (LoRa.available() && n < (int)sizeof(bufRx) - 1) bufRx[n++] = (char)LoRa.read();
  bufRx[n] = 0;
  tamRx = tam;
  chegou = true;
}
#endif

void imprimirPacote(const char* txt, int tam) {
  pacotes++;
  Serial.print(F("PKT #")); Serial.print(pacotes);
  Serial.print(F("  tam=")); Serial.print(tam);
  Serial.print(F("  \"")); Serial.print(txt); Serial.print('"');
  Serial.print(F("  RSSI=")); Serial.print(LoRa.packetRssi());
  Serial.print(F("  SNR=")); Serial.print(LoRa.packetSnr());
  Serial.print(F("  errofreq=")); Serial.print(LoRa.packetFrequencyError());
  Serial.println(F(" Hz"));
}

void setup() {
  Serial.begin(9600);
  Serial.println();
  Serial.println(F("=== KAIROS teste-lora-rx-uno ==="));

  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  LoRa.setSPIFrequency(1E6);
  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println(F("LoRa FALHOU no begin (SPI / 3V3 / NSS 10 / RST 9)"));
    while (true) delay(1000);
  }
  LoRa.setSpreadingFactor(LORA_SF);
  LoRa.setSignalBandwidth(LORA_BW);
  LoRa.setCodingRate4(LORA_CR);
  LoRa.setSyncWord(LORA_SYNC);
  LoRa.enableCrc();

#if USAR_DIO0
  LoRa.onReceive(aoReceber);
  LoRa.receive();
  Serial.println(F("LoRa RX OK — modo CONTINUO (DIO0 no D2)"));
#else
  Serial.println(F("LoRa RX OK — modo POLLING (parsePacket)"));
#endif
  Serial.println(F("433 MHz SF7 BW125 CR4/5 CRC sync 0x12 | tecla d = dump"));
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'd' || c == 'D') {
      Serial.println(F("--- dump SX1278 ---"));
      LoRa.dumpRegisters(Serial);
      Serial.println(F("--- fim ---"));
#if USAR_DIO0
      LoRa.receive();
#endif
    }
  }

#if USAR_DIO0
  if (chegou) {
    noInterrupts();
    chegou = false;
    int tam = tamRx;
    interrupts();
    imprimirPacote(bufRx, tam);
  }
#else
  int tam = LoRa.parsePacket();
  if (tam > 0) {
    char txt[64];
    int n = 0;
    while (LoRa.available() && n < (int)sizeof(txt) - 1) txt[n++] = (char)LoRa.read();
    txt[n] = 0;
    imprimirPacote(txt, tam);
  }
#endif

  int r = LoRa.rssi();
  if (r < rssiMin) rssiMin = r;
  if (r > rssiMax) rssiMax = r;

  if (millis() - ultimoStatusMs >= 1000) {
    ultimoStatusMs = millis();
    Serial.print(F("status  pkts=")); Serial.print(pacotes);
    Serial.print(F("  RSSI ar min/max=")); Serial.print(rssiMin);
    Serial.print('/'); Serial.println(rssiMax);
    rssiMin = 0;
    rssiMax = -200;
  }
}
