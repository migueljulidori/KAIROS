/*
 * Kairós — teste do pulso no braço (Arduino Uno).
 * Par do teste-pulso-tx na luva.
 *
 * O ângulo chega por LoRa, vindo do giroscópio. O motor segue com a
 * rampa suave (a aceleração sobe aos poucos).
 *
 * Pinos:
 *   STEP A3   DIR A4
 *   LoRa: NSS 10, RST 9, SCK 13, MISO 12, MOSI 11, VCC 3V3
 *   ENABLE do A4988 no GND, RESET e SLEEP no 5 V
 *
 * Pacote: [0xD1, pulso_lo, pulso_hi]  400 MHz SF7 BW250
 * Sem pacote por 800 ms, o motor para onde está.
 *
 * Serial 115200:
 *   s = para e ignora o rádio
 *   g = volta a seguir o rádio
 *   + / - = velocidade
 */

#include <SPI.h>
#include <LoRa.h>
#include <math.h>

#define LORA_SS   10
#define LORA_RST  9
#define LORA_DIO0 -1
const long LORA_FREQ = 400E6;
const uint8_t HDR = 0xD1;
const unsigned long TIMEOUT_MS = 800;

const int PIN_STEP = A3;
const int PIN_DIR  = A4;

const float PASSOS_POR_GRAU = (200.0f * 38.4f) / 360.0f;  // 21,33
const float LIM_TRAS   = -180.0f;
const float LIM_FRENTE =  110.0f;

float vmax = 750.0f;    // passos/s  (~35 °/s; 180° em cerca de 6 s)
float amax = 2200.0f;   // passos/s²
float jerk = 18000.0f;  // passos/s³

long pos = 0;
long alvo = 0;
float vel = 0;
float acc = 0;
unsigned long proxStepUs = 0;
unsigned long intervaloUs = 0;
unsigned long ultimoCtrlUs = 0;

bool seguir = true;
bool loraOk = false;
unsigned long ultimoPacoteMs = 0;
int rssi = 0;
float grausRadio = 0;

long grausParaPassos(float graus) {
  return (long)(graus * PASSOS_POR_GRAU + (graus >= 0 ? 0.5f : -0.5f));
}

float passosParaGraus(long passos) {
  return passos / PASSOS_POR_GRAU;
}

void irPara(float graus) {
  if (graus < LIM_TRAS) graus = LIM_TRAS;
  if (graus > LIM_FRENTE) graus = LIM_FRENTE;
  alvo = grausParaPassos(graus);
}

void lerRadio() {
  if (!loraOk) return;
  int tam = LoRa.parsePacket();
  if (tam < 3) {
    if (tam > 0) while (LoRa.available()) LoRa.read();
    return;
  }
  uint8_t h = LoRa.read();
  uint8_t lo = LoRa.read();
  uint8_t hi = LoRa.read();
  while (LoRa.available()) LoRa.read();
  if (h != HDR) return;
  ultimoPacoteMs = millis();
  rssi = LoRa.packetRssi();
  grausRadio = (int16_t)((uint16_t)lo | ((uint16_t)hi << 8));
  if (seguir) irPara(grausRadio);
}

void passo(bool frente) {
  digitalWrite(PIN_DIR, frente ? HIGH : LOW);
  delayMicroseconds(5);
  digitalWrite(PIN_STEP, HIGH);
  delayMicroseconds(4);
  digitalWrite(PIN_STEP, LOW);
  pos += frente ? 1 : -1;
}

void controle(float dt) {
  if (dt <= 0 || dt > 0.05f) dt = 0.001f;
  long erro = alvo - pos;

  float vDes;
  if (erro == 0) {
    vDes = 0;
  } else {
    float dist = fabsf((float)erro);
    // A aceleração demora um pouco para atingir o máximo. Freia antes.
    float reserva = (amax * amax) / (6.0f * jerk);
    if (dist > reserva) dist -= reserva;
    float vParar = sqrtf(2.0f * amax * dist);
    float v = vmax;
    if (vParar < v) v = vParar;
    vDes = (erro > 0) ? v : -v;
  }

  float aAlvo = 0;
  if (vel < vDes) aAlvo = amax;
  else if (vel > vDes) aAlvo = -amax;

  float dj = jerk * dt;
  if (acc < aAlvo) acc = min(acc + dj, aAlvo);
  else if (acc > aAlvo) acc = max(acc - dj, aAlvo);

  vel += acc * dt;
  if (vel > vmax)  { vel = vmax;  acc = 0; }
  if (vel < -vmax) { vel = -vmax; acc = 0; }
  if (aAlvo > 0 && vel > vDes) { vel = vDes; acc = 0; }
  if (aAlvo < 0 && vel < vDes) { vel = vDes; acc = 0; }
  if (erro == 0 && fabsf(vel) < 30.0f) { vel = 0; acc = 0; }

  float mag = fabsf(vel);
  if (mag < 25.0f) {
    intervaloUs = 0;
    if (erro == 0) vel = 0;
    else intervaloUs = 30000;  // fecha os últimos passos sem ficar 50 ms parado
  } else {
    intervaloUs = (unsigned long)(1000000.0f / mag);
  }
}

void imprimir() {
  Serial.print(F("pos="));
  Serial.print(passosParaGraus(pos), 1);
  Serial.print(F("°  alvo="));
  Serial.print(passosParaGraus(alvo), 1);
  Serial.print(F("°  radio="));
  Serial.print(grausRadio, 0);
  Serial.print(F("°  rssi="));
  Serial.print(rssi);
  Serial.print(seguir ? F("  seguindo") : F("  pausado"));
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_STEP, OUTPUT);
  pinMode(PIN_DIR, OUTPUT);
  digitalWrite(PIN_STEP, LOW);
  digitalWrite(PIN_DIR, LOW);

  Serial.println();
  Serial.println(F("=== TESTE PULSO (LoRa + giroscopio) ==="));
  Serial.println(F("Zero do motor = posicao de agora."));
  Serial.println(F("s = parar   g = seguir o radio   +/- velocidade"));

  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  LoRa.setSPIFrequency(1E6);
  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println(F("LoRa FALHOU — confira NSS 10 / RST 9 / 3V3"));
  } else {
    LoRa.setSpreadingFactor(7);
    LoRa.setSignalBandwidth(250E3);
    LoRa.setCodingRate4(5);
    LoRa.enableCrc();
    loraOk = true;
    Serial.println(F("LoRa RX OK 400 MHz. Esperando a luva."));
  }
  ultimoCtrlUs = micros();
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 's' || c == 'S') {
      seguir = false;
      alvo = pos;
      vel = 0;
      acc = 0;
      Serial.println(F("parado, radio ignorado"));
    }
    if (c == 'g' || c == 'G') {
      seguir = true;
      Serial.println(F("seguindo o radio"));
    }
    if (c == '+' && vmax < 1200) { vmax += 50; imprimir(); }
    if (c == '-' && vmax > 200)  { vmax -= 50; imprimir(); }
  }

  lerRadio();
  if (seguir && ultimoPacoteMs != 0 && millis() - ultimoPacoteMs > TIMEOUT_MS) {
    alvo = pos;
  }

  unsigned long agora = micros();
  if ((unsigned long)(agora - ultimoCtrlUs) >= 1000) {
    float dt = (agora - ultimoCtrlUs) / 1000000.0f;
    ultimoCtrlUs = agora;
    controle(dt);
  }

  if (intervaloUs && (long)(agora - proxStepUs) >= 0) {
    long erro = alvo - pos;
    if (erro != 0) {
      bool frente = (fabsf(vel) >= 25.0f) ? (vel > 0) : (erro > 0);
      passo(frente);
      proxStepUs = agora + intervaloUs;
    }
  }

  static unsigned long ultimoPrint = 0;
  if (millis() - ultimoPrint >= 200) {
    ultimoPrint = millis();
    imprimir();
  }
}
