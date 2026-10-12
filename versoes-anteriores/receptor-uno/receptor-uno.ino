/*
   Kairós — receptor-uno (Arduino Uno RX — 5 servos + LoRa)

   LoRa SX1278 433MHz:
     3V3, GND, SCK=13, MISO=12, MOSI=11
     NSS=10, RST=9, DIO0=2, ANT=433MHz

   Servos MG996R (sinal) — NAO usar D2, D9, D10 (LoRa) nem D11-D13 (SPI):
     D1: D3 | D2: D4 | D3: D5 | D4: D6 | D5: D7
     +5V fonte externa, GND comum

   Protocolo: 6 bytes [0xB5, ang0..ang4]
   Parametros iguais ao TX (433 MHz, SF7, BW250)
*/

#include <SPI.h>
#include <LoRa.h>
#include <Servo.h>

const int NUM_CANAIS = 5;

#define LORA_SS   10
#define LORA_RST  9
#define LORA_DIO0 2

const int PIN_SERVO[NUM_CANAIS] = { 8, 4, 5, 6, 7 };

const long LORA_FREQ = 433E6;
const uint8_t LORA_HEADER = 0xB5;

Servo servos[NUM_CANAIS];
int anguloServo[NUM_CANAIS];

void configurarLoRa() {
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);

  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println("ERRO: LoRa nao iniciou.");
    while (true) delay(1000);
  }

  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(250E3);
  LoRa.setCodingRate4(5);
  LoRa.enableCrc();

  Serial.println("LoRa RX 5ch (433 MHz, SF7, BW250)");
}

void setup() {
  Serial.begin(9600);

  configurarLoRa();

  for (int ch = 0; ch < NUM_CANAIS; ch++) {
    servos[ch].attach(PIN_SERVO[ch]);
    servos[ch].write(0);
    anguloServo[ch] = 0;
  }

  Serial.println("Kairos receptor-uno 5 servos aguardando...");
}

void loop() {
  int packetSize = LoRa.parsePacket();

  if (packetSize < (NUM_CANAIS + 1)) {
    return;
  }

  uint8_t header = LoRa.read();
  if (header != LORA_HEADER) {
    while (LoRa.available()) LoRa.read();
    return;
  }

  for (int ch = 0; ch < NUM_CANAIS; ch++) {
    int angulo = (int)LoRa.read();
    angulo = constrain(angulo, 0, 180);
    anguloServo[ch] = angulo;
    servos[ch].write(angulo);
  }

  Serial.print("RX:");
  for (int ch = 0; ch < NUM_CANAIS; ch++) {
    Serial.print(" D");
    Serial.print(ch + 1);
    Serial.print("=");
    Serial.print(anguloServo[ch]);
  }
  Serial.print(" | RSSI:");
  Serial.println(LoRa.packetRssi());
}
