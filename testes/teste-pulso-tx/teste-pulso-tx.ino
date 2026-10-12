/*
 * Kairós — teste do pulso na luva (ESP32).
 * Par do teste-pulso no Arduino Uno.
 *
 * Lê o giroscópio da MÃO e do ANTEBRAÇO, tira o giro comum e manda
 * só o ângulo do pulso por LoRa. Sem flex e sem os outros eixos.
 *
 * Pinos iguais ao kairos-luva-tx:
 *   Mão:       SDA 27  SCL 14
 *   Antebraço: SDA 4   SCL 13
 *   LoRa: NSS 5  RST 2  DIO0 15  SCK 18  MISO 19  MOSI 23
 *   MPU em 3V3, AD0 no GND (endereço 0x68)
 *
 * Rádio igual ao projeto: 400 MHz, SF7, BW 250 kHz, CR 4/5, CRC.
 * Pacote: [0xD1, pulso_lo, pulso_hi]  int16 graus, little-endian.
 *   0 = pose em que a calibração terminou
 *   negativo = para trás, até -180
 *
 * Serial 115200. Tecla i inverte o sentido. Tecla z zera o ângulo.
 */

#include <WiFi.h>
#include <SPI.h>
#include <LoRa.h>
#include <math.h>

const int PIN_SDA[2] = { 27, 4 };
const int PIN_SCL[2] = { 14, 13 };
const char* NOME[2]  = { "Mao", "Antebraco" };
#define I_MAO  0
#define I_ANTE 1

#define LORA_SS   5
#define LORA_RST  2
#define LORA_DIO0 15
const long LORA_FREQ = 400E6;
const uint8_t HDR = 0xD1;

const float ZONA = 3.0f;          // °/s abaixo disso não integra
const float LIM_TRAS = -180.0f;
const float LIM_FRENTE = 110.0f;
const unsigned long TX_MS = 40;

int gSda, gScl;
uint8_t gAddr = 0x68;
float bias[2][3];
float eixo[2][3];
bool anteOk = false;
float sinal = 1.0f;
float angulo = 0;
bool pronto = false;

void softDelay()   { delayMicroseconds(5); }
void sdaH() { pinMode(gSda, INPUT_PULLUP); }
void sdaL() { pinMode(gSda, OUTPUT); digitalWrite(gSda, LOW); }
void sclH() { pinMode(gScl, INPUT_PULLUP); softDelay(); }
void sclL() { pinMode(gScl, OUTPUT); digitalWrite(gScl, LOW); softDelay(); }
void i2cStart() { sdaH(); sclH(); sdaL(); sclL(); }
void i2cStop()  { sdaL(); sclH(); sdaH(); softDelay(); }

bool i2cWrite(uint8_t d) {
  for (int i = 7; i >= 0; i--) {
    if (d & (1 << i)) sdaH(); else sdaL();
    sclH(); sclL();
  }
  sdaH(); sclH();
  bool ack = digitalRead(gSda) == LOW;
  sclL();
  return ack;
}

uint8_t i2cRead(bool ack) {
  uint8_t d = 0;
  sdaH();
  for (int i = 7; i >= 0; i--) {
    sclH();
    if (digitalRead(gSda)) d |= (1 << i);
    sclL();
  }
  if (ack) sdaL(); else sdaH();
  sclH(); sclL(); sdaH();
  return d;
}

void barramento(int i) {
  gSda = PIN_SDA[i];
  gScl = PIN_SCL[i];
  sdaH(); sclH();
}

bool escreverReg(uint8_t reg, uint8_t val) {
  i2cStart();
  bool ok = i2cWrite((gAddr << 1) | 0) && i2cWrite(reg) && i2cWrite(val);
  i2cStop();
  return ok;
}

bool lerReg(uint8_t reg, uint8_t* buf, int n) {
  i2cStart();
  if (!i2cWrite((gAddr << 1) | 0) || !i2cWrite(reg)) { i2cStop(); return false; }
  i2cStart();
  if (!i2cWrite((gAddr << 1) | 1)) { i2cStop(); return false; }
  for (int i = 0; i < n; i++) buf[i] = i2cRead(i < n - 1);
  i2cStop();
  return true;
}

bool lerMpu(int i, float a[3], float g[3]) {
  barramento(i);
  uint8_t b[14];
  if (!lerReg(0x3B, b, 14)) return false;
  for (int k = 0; k < 3; k++) {
    a[k] = (int16_t)((b[k * 2] << 8) | b[k * 2 + 1]) / 16384.0f;
    g[k] = (int16_t)((b[8 + k * 2] << 8) | b[9 + k * 2]) / 65.5f;
  }
  return (fabsf(a[0]) + fabsf(a[1]) + fabsf(a[2])) > 0.1f;
}

bool iniciarMpu(int i) {
  barramento(i);
  if (!escreverReg(0x6B, 0x00)) return false;
  delay(20);
  escreverReg(0x1B, 0x08);
  escreverReg(0x1C, 0x00);
  escreverReg(0x1A, 0x03);
  escreverReg(0x19, 0x04);
  float a[3], g[3];
  return lerMpu(i, a, g);
}

float dot3(const float a[3], const float b[3]) {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

float eixoPrincipal(double M[3][3], float out[3]) {
  float v[3] = { 0, 0, 1 };
  for (int it = 0; it < 40; it++) {
    float n[3];
    for (int r = 0; r < 3; r++)
      n[r] = M[r][0] * v[0] + M[r][1] * v[1] + M[r][2] * v[2];
    float nn = sqrtf(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
    if (nn < 1e-6f) return 0;
    v[0] = n[0] / nn; v[1] = n[1] / nn; v[2] = n[2] / nn;
  }
  out[0] = v[0]; out[1] = v[1]; out[2] = v[2];
  double tr = M[0][0] + M[1][1] + M[2][2];
  double lam = 0;
  for (int r = 0; r < 3; r++)
    lam += v[r] * (M[r][0] * v[0] + M[r][1] * v[1] + M[r][2] * v[2]);
  return tr > 0 ? (float)(lam / tr) : 0;
}

void esperar(const __FlashStringHelper* msg, int segundos) {
  Serial.println(msg);
  for (int s = segundos; s > 0; s--) {
    Serial.println(s);
    delay(1000);
  }
}

bool calibrar() {
  float a[3], g[3];
  esperar(F("Palma na posicao INICIAL, antebraco parado."), 3);
  Serial.println(F("medindo o bias..."));
  double sg[2][3] = {{0, 0, 0}, {0, 0, 0}};
  int n[2] = {0, 0};
  unsigned long t0 = millis();
  while (millis() - t0 < 1500) {
    for (int i = 0; i < 2; i++) {
      if (!lerMpu(i, a, g)) continue;
      for (int k = 0; k < 3; k++) sg[i][k] += g[k];
      n[i]++;
    }
    delay(5);
  }
  if (n[I_MAO] < 40) {
    Serial.println(F("Mao sem leitura. Confira SDA 27 SCL 14."));
    return false;
  }
  for (int i = 0; i < 2; i++) {
    if (n[i] < 10) continue;
    for (int k = 0; k < 3; k++) bias[i][k] = sg[i][k] / n[i];
  }
  Serial.printf("bias mao [%.1f %.1f %.1f]\n", bias[0][0], bias[0][1], bias[0][2]);

  double Mm[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
  double Ma[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
  int nm = 0, na = 0;
  esperar(F("GIRE A PALMA para um lado e para o outro. Segure o ombro."), 1);
  Serial.println(F("girando..."));
  t0 = millis();
  while (millis() - t0 < 5000) {
    float gm[3], ga[3];
    bool okM = lerMpu(I_MAO, a, gm);
    bool okA = lerMpu(I_ANTE, a, ga);
    if (okM) {
      float w[3];
      for (int k = 0; k < 3; k++) w[k] = gm[k] - bias[I_MAO][k];
      float mag = sqrtf(dot3(w, w));
      if (mag > 40.0f) {
        for (int r = 0; r < 3; r++)
          for (int c = 0; c < 3; c++) Mm[r][c] += (double)w[r] * w[c];
        nm++;
      }
    }
    if (okA) {
      float w[3];
      for (int k = 0; k < 3; k++) w[k] = ga[k] - bias[I_ANTE][k];
      if (sqrtf(dot3(w, w)) > 25.0f) {
        for (int r = 0; r < 3; r++)
          for (int c = 0; c < 3; c++) Ma[r][c] += (double)w[r] * w[c];
        na++;
      }
    }
    delay(2);
  }
  if (nm < 30) {
    Serial.println(F("Pouca giro da mao. Repetindo."));
    return false;
  }
  float pureza = eixoPrincipal(Mm, eixo[I_MAO]);
  Serial.printf("eixo da mao [%.2f %.2f %.2f]  amostras %d  pureza %.0f%%\n",
                eixo[I_MAO][0], eixo[I_MAO][1], eixo[I_MAO][2], nm, pureza * 100);
  anteOk = na >= 20;
  if (anteOk) {
    eixoPrincipal(Ma, eixo[I_ANTE]);
    Serial.printf("eixo do antebraco [%.2f %.2f %.2f]\n",
                  eixo[I_ANTE][0], eixo[I_ANTE][1], eixo[I_ANTE][2]);
  } else {
    Serial.println(F("Antebraco quase parado: o pulso usa so a mao."));
    eixo[I_ANTE][0] = eixo[I_ANTE][1] = eixo[I_ANTE][2] = 0;
  }

  esperar(F("VOLTE a palma para a posicao inicial e segure."), 2);
  angulo = 0;
  Serial.println(F("Zero gravado. Pode girar. i = inverte sentido, z = zera."));
  return true;
}

void enviar(int16_t graus) {
  uint16_t u = (uint16_t)graus;
  LoRa.beginPacket();
  LoRa.write(HDR);
  LoRa.write((uint8_t)(u & 0xFF));
  LoRa.write((uint8_t)(u >> 8));
  LoRa.endPacket();
}

void setup() {
  Serial.begin(115200);
  delay(300);
  WiFi.mode(WIFI_OFF);
  Serial.println();
  Serial.println(F("=== TESTE PULSO TX (giroscopio + LoRa) ==="));

  bool okM = iniciarMpu(I_MAO);
  bool okA = iniciarMpu(I_ANTE);
  Serial.printf("Mao %s | Antebraco %s\n", okM ? "OK" : "FALHOU", okA ? "OK" : "FALHOU");
  if (!okM) {
    Serial.println(F("Sem a mao nao tem pulso."));
    while (true) delay(1000);
  }

  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  LoRa.setSPIFrequency(1E6);
  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println(F("LoRa FALHOU"));
    while (true) delay(1000);
  }
  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(250E3);
  LoRa.setCodingRate4(5);
  LoRa.enableCrc();
  LoRa.setTxPower(14);
  Serial.println(F("LoRa 400 MHz OK"));

  while (!calibrar()) delay(500);
  pronto = true;
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'i' || c == 'I') {
      sinal = -sinal;
      angulo = 0;
      Serial.println(sinal > 0 ? F("sentido normal, zero") : F("sentido invertido, zero"));
    }
    if (c == 'z' || c == 'Z') {
      angulo = 0;
      Serial.println(F("zero"));
    }
    if (c == 'r' || c == 'R') {
      pronto = false;
      while (!calibrar()) delay(500);
      pronto = true;
    }
  }
  if (!pronto) return;

  static unsigned long tAnt = 0;
  unsigned long agora = micros();
  float dt = tAnt ? (agora - tAnt) * 1e-6f : 0.02f;
  tAnt = agora;
  if (dt <= 0 || dt > 0.1f) dt = 0.02f;

  float a[3], gm[3], ga[3];
  float wm[3] = {0, 0, 0};
  float wa[3] = {0, 0, 0};
  if (lerMpu(I_MAO, a, gm)) {
    for (int k = 0; k < 3; k++) wm[k] = gm[k] - bias[I_MAO][k];
  }
  if (anteOk && lerMpu(I_ANTE, a, ga)) {
    for (int k = 0; k < 3; k++) wa[k] = ga[k] - bias[I_ANTE][k];
  }

  float taxa = sinal * (dot3(wm, eixo[I_MAO]) - dot3(wa, eixo[I_ANTE]));
  if (fabsf(taxa) < ZONA) taxa = 0;
  angulo += taxa * dt;
  if (angulo < LIM_TRAS) angulo = LIM_TRAS;
  if (angulo > LIM_FRENTE) angulo = LIM_FRENTE;

  static unsigned long ultimoTx = 0;
  if (millis() - ultimoTx >= TX_MS) {
    ultimoTx = millis();
    int16_t g = (int16_t)(angulo + (angulo >= 0 ? 0.5f : -0.5f));
    enviar(g);
  }

  static unsigned long ultimoPrint = 0;
  if (millis() - ultimoPrint >= 200) {
    ultimoPrint = millis();
    Serial.printf("pulso %+4d°  taxa %+5.0f °/s\n", (int)angulo, taxa);
  }
}
