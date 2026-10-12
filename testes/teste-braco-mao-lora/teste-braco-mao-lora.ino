/*
 * Kairós — teste-braco-mao-lora (ESP32)
 *   So 2 MPU6050 (Braco + Mao), sem Antebraco e sem flex.
 *   Envia o mesmo pacote do kairos-luva-tx -> kairos-receptor mexe o braco.
 *
 * Pacote LoRa 10 bytes:
 *   [0xC2, cotovelo, ombro, pulso, base, d0..d4]   (dedos fixos em DEDO_FIXO)
 *
 * Cada segmento tem um EIXO (vetor fixo no referencial do sensor):
 *   Braco: eixo longo = gravidade medida com o braco PARA CIMA
 *   Mao:   eixo de PRONACAO = eixo em torno do qual a mao gira quando voce
 *          vira a palma, medido pelo gyro na pose 4 (componente principal).
 *   Girar a palma e' uma rotacao EM TORNO desse eixo, entao a inclinacao dele
 *   nao muda -> o cotovelo nao mexe quando o pulso gira. Com o eixo "da mao"
 *   (pose para cima) o sensor torto nas costas da mao fazia o giro vazar.
 *
 * Juntas (mesma convencao do kairos-luva-tx):
 *   ombro    = inclinacao do eixo do Braco (0 = cima, 90 = horizontal)
 *   cotovelo = inclinacao do Braco - inclinacao do eixo de pronacao
 *              (dobrar o PULSO para cima/baixo ainda entra aqui: sem sensor
 *               no antebraco nao ha como separar — mantenha o pulso reto)
 *   pulso    = giro da Mao no eixo de pronacao - giro do Braco no proprio eixo
 *              (esse desconto cai com o cos do cotovelo)
 *   base     = rumo horizontal do Braco vs a frente da calibracao; com o braco
 *              quase vertical nao ha rumo e a base segura o ultimo valor
 *   pulso e a frente usada na base vem do gyro: derivam devagar -> tecla z zera.
 *
 * LIGACOES (iguais ao kairos-luva-tx):
 *   Braco: SDA 25  SCL 26      Mao: SDA 27  SCL 14
 *   Se o OMBRO mexer quando voce mexe o antebraco, o Braco esta no 4/13:
 *   troque PIN_SDA/PIN_SCL do Braco abaixo.
 *   LoRa: NSS 5  RST 2  DIO0 15  SCK 18  MISO 19  MOSI 23
 *
 * CALIBRACAO (no boot e na tecla r) — 4 poses; as paradas esperam voce PARAR:
 *   1. braco ESTICADO PARA CIMA, cotovelo reto, pulso reto
 *      -> eixo do Braco, zero do ombro, bias dos gyros
 *   2. braco ESTICADO A FRENTE na horizontal, cotovelo reto
 *      -> ganho do ombro (90° / medido) e "frente" do mundo
 *   3. braco na horizontal a frente, antebraco APONTANDO PARA CIMA (cotovelo 90°)
 *      -> ganho do cotovelo
 *   4. braco de novo na horizontal a frente, cotovelo reto:
 *      GIRE A PALMA para baixo e para cima (como virar uma chave) por ~6 s,
 *      sem mexer cotovelo nem ombro; no fim pare com a palma NEUTRA.
 *      -> eixo de pronacao da Mao, zero do cotovelo e zero do pulso
 *   Pose errada repete so aquela pose.
 *
 * Serial 115200:  r = recalibrar   z = zerar pulso e base
 */

#include <SPI.h>
#include <LoRa.h>
#include <WiFi.h>
#include <math.h>

// ---------------------------------------------------------------- pinos
const int NUM_MPU = 2;
#define S_BRACO 0
#define S_MAO   1
const int PIN_SDA[NUM_MPU]  = { 25, 27 };
const int PIN_SCL[NUM_MPU]  = { 26, 14 };
const char* NOME[NUM_MPU]   = { "Braco", "Mao" };

#define LORA_SS   5
#define LORA_RST  2
#define LORA_DIO0 15
const long LORA_FREQ = 433E6;
const uint8_t LORA_HEADER = 0xC2;
const unsigned long TX_PERIODO_MS = 40;

// ---------------------------------------------------------------- ajustes
// Inverte o sentido:                  C      O      P      B
const bool  INV_TX[4]           = { false, false,  true, false };
const float GANHO[4]            = {  1.0f,  1.0f,  1.0f,  1.0f };
const uint8_t DEDO_FIXO = 0;       // angulo mandado para os 5 servos

const float LIM_COT_MAX  = 150;    // mesmos limites do receptor
const float LIM_OMB_MAX  = 130;
const float LIM_PUL      = 110;
const float LIM_BASE     = 150;

const float ACC_PESO     = 0.02f;  // quanto o acelerometro corrige a gravidade
const float ACC_TOL_G    = 0.15f;  // |a| fora de 1 g +- isso = aceleracao, ignora
// Girando rapido, o sensor fora do eixo sente forca centripeta: accel mente.
const float ACC_GIRO_MAX = 150.0f; // °/s (soma |w|) acima disso so o gyro manda
const float ZONA_GIRO    = 2.5f;   // °/s abaixo disso nao integra o pulso
const int   N_CAL        = 300;

const unsigned long PARADO_MS   = 1500;   // quieto por isso = pose aceita
const unsigned long POSE_MAX_MS = 30000;  // sem ficar parado: mede assim mesmo
const float PARADO_GYRO  = 10.0f;  // °/s bruto (bias ainda desconhecido)
const float PARADO_ACC_G = 0.08f;
const float SPAN_MIN     = 55.0f;  // 90° medido fora de 55..125 = pose errada
const float SPAN_MAX     = 125.0f;

// Pose 4 (eixo de pronacao)
const unsigned long PRON_MS = 6000;
const float PRON_W_MIN      = 40.0f;  // °/s na mao: amostra de giro de verdade
const float PRON_BRACO_MAX  = 25.0f;  // °/s no braco: acima disso o braco mexeu
const int   PRON_AMOSTRAS   = 150;
const float PRON_PUREZA_MIN = 0.85f;  // fracao da energia do gyro no eixo achado
const float PRON_DESVIO_MAX = 45.0f;  // eixo x direcao da mao: acima = pose errada

const uint8_t MPU_ADDR_GND = 0x68;
const uint8_t MPU_ADDR_VCC = 0x69;
const int SOFT_DELAY_US = 5;

// ---------------------------------------------------------------- estado
struct Sensor {
  bool ok;
  uint8_t addr;
  float g[3];      // gravidade estimada (unitaria, referencial do sensor)
  float g0[3];     // gravidade com o braco PARA CIMA
  float eixo[3];   // eixo do segmento (Braco: g0 | Mao: eixo de pronacao)
  float f[3];      // "frente" do mundo no referencial do sensor (horizontal)
  bool  temFrente;
  float bias[3];   // bias do gyro (°/s)
  float w[3];      // gyro sem bias (°/s)
  uint16_t falhasSeguidas;
  unsigned long ultimaRecupMs;
  uint32_t leituras, falhas;
};
Sensor s[NUM_MPU];

float pulso = 0, base = 0, baseOffset = 0;
float jCot = 0, jOmb = 0;
float ganhoOmb = 1.0f, cotZero = 0.0f, ganhoCot = 1.0f;
unsigned long ultimoUs = 0, ultimoTxMs = 0;
uint32_t pacotesTx = 0, falhasTx = 0;
int loraFalhasSeguidas = 0;

// ---------------------------------------------------------------- soft I2C
// Mesmo metodo do teste-mpu-verifica (funcionou com todos os MPUs).
int gSda, gScl;
uint8_t gAddr = MPU_ADDR_GND;

void softDelay()   { delayMicroseconds(SOFT_DELAY_US); }
void softSdaHigh() { pinMode(gSda, INPUT_PULLUP); }
void softSdaLow()  { pinMode(gSda, OUTPUT); digitalWrite(gSda, LOW); }
void softSclHigh() { pinMode(gScl, INPUT_PULLUP); softDelay(); }
void softSclLow()  { pinMode(gScl, OUTPUT); digitalWrite(gScl, LOW); softDelay(); }
void softStart()   { softSdaHigh(); softSclHigh(); softSdaLow(); softSclLow(); }
void softStop()    { softSdaLow(); softSclHigh(); softSdaHigh(); softDelay(); }

bool softWriteByte(uint8_t d) {
  for (int i = 7; i >= 0; i--) {
    if (d & (1 << i)) softSdaHigh(); else softSdaLow();
    softSclHigh(); softSclLow();
  }
  softSdaHigh(); softSclHigh();
  bool ack = (digitalRead(gSda) == LOW);
  softSclLow();
  return ack;
}

uint8_t softReadByte(bool ack) {
  uint8_t d = 0;
  softSdaHigh();
  for (int i = 7; i >= 0; i--) {
    softSclHigh();
    if (digitalRead(gSda)) d |= (1 << i);
    softSclLow();
  }
  if (ack) softSdaLow(); else softSdaHigh();
  softSclHigh(); softSclLow(); softSdaHigh();
  return d;
}

void usarBarramento(int i) {
  gSda = PIN_SDA[i];
  gScl = PIN_SCL[i];
  gAddr = s[i].addr;
  softSdaHigh();
  softSclHigh();
}

void softClockStretch() {
  softSdaHigh();
  for (int k = 0; k < 9; k++) { softSclHigh(); softSclLow(); }
  softStop();
}

bool softAck(uint8_t addr) {
  softStart();
  bool ok = softWriteByte((addr << 1) | 0);
  softStop();
  return ok;
}

bool softWriteReg(uint8_t reg, uint8_t val) {
  softStart();
  bool ok = softWriteByte((gAddr << 1) | 0) && softWriteByte(reg) && softWriteByte(val);
  softStop();
  return ok;
}

bool softRead(uint8_t reg, uint8_t* buf, int n) {
  softStart();
  if (!softWriteByte((gAddr << 1) | 0)) { softStop(); return false; }
  if (!softWriteByte(reg))              { softStop(); return false; }
  softStart();
  if (!softWriteByte((gAddr << 1) | 1)) { softStop(); return false; }
  for (int i = 0; i < n; i++) buf[i] = softReadByte(i < n - 1);
  softStop();
  return true;
}

// a em g, gyro em °/s. Falha = I2C sem ACK ou |a| ~ 0 (MPU mudo devolve zeros).
bool lerSensor(int i, float a[3], float gy[3]) {
  usarBarramento(i);
  uint8_t b[14];
  if (!softRead(0x3B, b, 14)) {
    softClockStretch();
    if (!softRead(0x3B, b, 14)) return false;
  }
  for (int k = 0; k < 3; k++) {
    a[k]  = (int16_t)((b[k * 2] << 8) | b[k * 2 + 1]) / 16384.0f;
    gy[k] = (int16_t)((b[8 + k * 2] << 8) | b[9 + k * 2]) / 65.5f;
  }
  return (fabsf(a[0]) + fabsf(a[1]) + fabsf(a[2])) > 0.1f;
}

bool iniciarSensor(int i) {
  gSda = PIN_SDA[i];
  gScl = PIN_SCL[i];
  softClockStretch();
  delay(5);
  for (int t = 0; t < 3; t++) {
    uint8_t addr = 0;
    if (softAck(MPU_ADDR_GND))      addr = MPU_ADDR_GND;
    else if (softAck(MPU_ADDR_VCC)) addr = MPU_ADDR_VCC;
    if (!addr) { softClockStretch(); delay(10); continue; }
    s[i].addr = addr;
    gAddr = addr;
    if (!softWriteReg(0x6B, 0x00)) { softClockStretch(); delay(10); continue; }
    delay(10);
    softWriteReg(0x1B, 0x08);   // gyro ±500 °/s
    softWriteReg(0x1C, 0x00);   // accel ±2 g
    softWriteReg(0x1A, 0x03);   // DLPF 44 Hz
    softWriteReg(0x19, 0x04);   // 200 Hz
    float a[3], gy[3];
    if (lerSensor(i, a, gy)) return true;
    softClockStretch();
    delay(10);
  }
  return false;
}

// ---------------------------------------------------------------- math
float dot3(const float a[3], const float b[3]) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }

void copiar3(float dst[3], const float src[3]) { dst[0] = src[0]; dst[1] = src[1]; dst[2] = src[2]; }

bool normalizar3(float v[3]) {
  float n = sqrtf(dot3(v, v));
  if (n < 1e-4f) return false;
  v[0] /= n; v[1] /= n; v[2] /= n;
  return true;
}

float anguloEntre(const float a[3], const float b[3]) {
  float c = dot3(a, b);
  if (c > 1) c = 1;
  if (c < -1) c = -1;
  return acosf(c) * 180.0f / PI;
}

// Inclinacao COM SINAL de um eixo: 0 = para cima, +90 = a frente/lado, 180 = baixo,
// negativo = para tras. O sinal vem de f ("frente" do MUNDO rastreada pelo gyro),
// nunca de um vetor fixo no sensor: esse gira junto com o pulso e invertia o sinal.
float inclinacaoDe(const float g[3], const float eixo[3], const float f[3], bool temF) {
  float c = dot3(g, eixo);
  if (c > 1) c = 1;
  if (c < -1) c = -1;
  float magP = sqrtf(1.0f - c * c);
  float sgn = 1.0f;
  // Elevacao lateral fica quase perpendicular a f: conta como positiva.
  if (temF && dot3(eixo, f) < -0.5f * magP) sgn = -1.0f;
  float r = atan2f(sgn * magP, c) * 180.0f / PI;
  // Faixa -90..270 em vez de -180..180: com o braco CAIDO (~180) um tremor para
  // tras virava -179 e o ombro/cotovelo saltavam de ponta a ponta. Passar de -90
  // (para cima E para tras) o corpo nao faz, entao o corte fica ali.
  if (r < -90.0f) r += 360.0f;
  return r;
}

float inclinacao(const Sensor& S) { return inclinacaoDe(S.g, S.eixo, S.f, S.temFrente); }

// f = parte horizontal do eixo, com o segmento apontando para a frente.
bool frenteDe(const float g[3], const float eixo[3], float fOut[3]) {
  float c = dot3(eixo, g);
  for (int k = 0; k < 3; k++) fOut[k] = eixo[k] - c * g[k];
  return normalizar3(fOut);
}

void ancorarFrente(Sensor& S) { S.temFrente = frenteDe(S.g, S.eixo, S.f); }

// Rumo horizontal do eixo relativo a frente da calibracao (graus, + = esquerda).
// f e g sao do mundo, entao girar o braco no PROPRIO eixo (rotacao interna do
// ombro) nao muda o rumo — integrar w·g fazia a base girar com esse giro.
bool azimute(const Sensor& S, float& az) {
  if (!S.temFrente) return false;
  float c = dot3(S.eixo, S.g);
  float h[3] = { S.eixo[0] - c * S.g[0], S.eixo[1] - c * S.g[1], S.eixo[2] - c * S.g[2] };
  if (sqrtf(dot3(h, h)) < 0.35f) return false;   // eixo quase vertical: sem rumo
  float esq[3] = { S.g[1] * S.f[2] - S.g[2] * S.f[1],
                   S.g[2] * S.f[0] - S.g[0] * S.f[2],
                   S.g[0] * S.f[1] - S.g[1] * S.f[0] };
  az = atan2f(dot3(h, esq), dot3(h, S.f)) * 180.0f / PI;
  return true;
}

float wrap180(float a) {
  while (a >  180.0f) a -= 360.0f;
  while (a < -180.0f) a += 360.0f;
  return a;
}

// Gira v junto com o mundo visto pelo sensor: dv/dt = -w x v.
void girarNoMundo(float v[3], const float wr[3], float dt) {
  float cx = wr[1] * v[2] - wr[2] * v[1];
  float cy = wr[2] * v[0] - wr[0] * v[2];
  float cz = wr[0] * v[1] - wr[1] * v[0];
  v[0] -= cx * dt;
  v[1] -= cy * dt;
  v[2] -= cz * dt;
}

// Gyro prediz g e f; acelerometro corrige g so quando |a| ~ 1 g e sem giro rapido.
void atualizarGravidade(Sensor& S, const float a[3], float dt) {
  float wr[3] = { S.w[0] * DEG_TO_RAD, S.w[1] * DEG_TO_RAD, S.w[2] * DEG_TO_RAD };
  girarNoMundo(S.g, wr, dt);
  normalizar3(S.g);

  float magA = sqrtf(dot3(a, a));
  float magW = fabsf(S.w[0]) + fabsf(S.w[1]) + fabsf(S.w[2]);
  if (fabsf(magA - 1.0f) < ACC_TOL_G && magW < ACC_GIRO_MAX) {
    for (int k = 0; k < 3; k++)
      S.g[k] = (1.0f - ACC_PESO) * S.g[k] + ACC_PESO * (a[k] / magA);
    normalizar3(S.g);
  }

  if (S.temFrente) {
    girarNoMundo(S.f, wr, dt);
    float c = dot3(S.f, S.g);
    for (int k = 0; k < 3; k++) S.f[k] -= c * S.g[k];
    if (!normalizar3(S.f)) S.temFrente = false;
  }
}

// Autovetor dominante de uma matriz 3x3 simetrica (iteracao de potencia).
// Retorna a fracao da energia nesse eixo (1.0 = giro puro em um eixo so).
float eixoPrincipal(const double M[3][3], const float chute[3], float out[3]) {
  float v[3];
  copiar3(v, chute);
  if (!normalizar3(v)) { v[0] = 0; v[1] = 0; v[2] = 1; }
  for (int it = 0; it < 50; it++) {
    float n[3];
    for (int r = 0; r < 3; r++) n[r] = M[r][0] * v[0] + M[r][1] * v[1] + M[r][2] * v[2];
    if (!normalizar3(n)) return 0;
    copiar3(v, n);
  }
  double lambda = 0;
  for (int r = 0; r < 3; r++)
    lambda += v[r] * (M[r][0] * v[0] + M[r][1] * v[1] + M[r][2] * v[2]);
  double traco = M[0][0] + M[1][1] + M[2][2];
  copiar3(out, v);
  return traco > 0 ? (float)(lambda / traco) : 0;
}

// ---------------------------------------------------------------- calibracao
bool todosParados() {
  for (int i = 0; i < NUM_MPU; i++) {
    if (!s[i].ok) continue;
    float a[3], gy[3];
    if (!lerSensor(i, a, gy)) return false;
    float magA = sqrtf(dot3(a, a));
    float magG = fabsf(gy[0]) + fabsf(gy[1]) + fabsf(gy[2]);
    if (fabsf(magA - 1.0f) > PARADO_ACC_G || magG > PARADO_GYRO) return false;
  }
  return true;
}

void tituloPose(int n, const __FlashStringHelper* titulo, const __FlashStringHelper* dica) {
  Serial.println();
  Serial.println(F("----------------------------------------"));
  Serial.printf("POSE %d/4 — ", n);
  Serial.println(titulo);
  Serial.println(dica);
}

void esperarParado() {
  Serial.println(F("  entre na pose e FIQUE PARADO..."));
  delay(1500);   // tempo de entrar na pose antes de comecar a vigiar
  unsigned long t0 = millis(), desde = 0;
  bool avisou = false;
  while (millis() - t0 < POSE_MAX_MS) {
    if (todosParados()) {
      if (!desde) desde = millis();
      if (!avisou) { Serial.println(F("  segure...")); avisou = true; }
      if (millis() - desde >= PARADO_MS) {
        Serial.println(F("  parado — medindo..."));
        return;
      }
    } else {
      desde = 0;
      avisou = false;
    }
    delay(10);
  }
  Serial.println(F("  (nao ficou parado — medindo assim mesmo)"));
}

// Media da gravidade (unitaria) de um sensor; bias opcional (so na pose 1).
bool medirGravidade(int i, float gOut[3], float* biasOut) {
  double sa[3] = { 0, 0, 0 }, sg[3] = { 0, 0, 0 };
  int n = 0;
  for (int k = 0; k < N_CAL; k++) {
    float a[3], gy[3];
    if (lerSensor(i, a, gy)) {
      for (int j = 0; j < 3; j++) { sa[j] += a[j]; sg[j] += gy[j]; }
      n++;
    }
    delay(3);
  }
  if (n < N_CAL / 3) {
    Serial.printf("  %s poucas leituras (%d/%d) — confira os fios\n", NOME[i], n, N_CAL);
    return false;
  }
  for (int j = 0; j < 3; j++) gOut[j] = sa[j] / n;
  if (biasOut) for (int j = 0; j < 3; j++) biasOut[j] = sg[j] / n;
  return normalizar3(gOut);
}

// Pose 4: acumula w*w^T do gyro da mao enquanto a palma gira. O autovetor
// dominante e' o eixo real de pronacao no referencial do sensor da mao.
bool medirEixoPronacao(float eixoOut[3]) {
  double M[3][3] = { { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 } };
  int usadas = 0, rejBraco = 0;
  float giroMin = 0, giroMax = 0, giroAcum = 0;
  unsigned long t0 = millis(), tAntUs = micros(), ultimoPrint = 0;

  Serial.println(F("  GIRE A PALMA agora (para baixo e para cima, sem parar)..."));
  while (millis() - t0 < PRON_MS) {
    float aM[3], gM[3], aB[3], gB[3];
    bool okM = lerSensor(S_MAO, aM, gM);
    bool okB = s[S_BRACO].ok ? lerSensor(S_BRACO, aB, gB) : false;
    unsigned long agora = micros();
    float dt = (agora - tAntUs) / 1000000.0f;
    tAntUs = agora;
    if (dt > 0.05f) dt = 0.05f;
    if (!okM) continue;

    float w[3];
    for (int k = 0; k < 3; k++) w[k] = gM[k] - s[S_MAO].bias[k];
    float magW = sqrtf(dot3(w, w));

    // Amplitude aproximada: giro em torno da direcao da mao (so para o aviso).
    giroAcum += dot3(w, s[S_MAO].g0) * dt;
    if (giroAcum < giroMin) giroMin = giroAcum;
    if (giroAcum > giroMax) giroMax = giroAcum;

    if (okB) {
      float wb[3];
      for (int k = 0; k < 3; k++) wb[k] = gB[k] - s[S_BRACO].bias[k];
      if (sqrtf(dot3(wb, wb)) > PRON_BRACO_MAX) { rejBraco++; continue; }
    }
    if (magW < PRON_W_MIN) continue;

    for (int r = 0; r < 3; r++)
      for (int c = 0; c < 3; c++) M[r][c] += (double)w[r] * w[c];
    usadas++;

    if (millis() - ultimoPrint >= 1000) {
      ultimoPrint = millis();
      Serial.printf("  %lus | amostras %d | giro %.0f°..%.0f°\n",
                    (millis() - t0) / 1000, usadas, giroMin, giroMax);
    }
    delay(2);
  }

  if (usadas < PRON_AMOSTRAS) {
    Serial.printf("  poucas amostras de giro (%d/%d)", usadas, PRON_AMOSTRAS);
    if (rejBraco > usadas) Serial.print(F(" — o BRACO mexeu demais, segure o ombro"));
    else Serial.print(F(" — gire a palma mais rapido e mais amplo"));
    Serial.println();
    return false;
  }

  float eixo[3];
  float pureza = eixoPrincipal(M, s[S_MAO].g0, eixo);
  if (dot3(eixo, s[S_MAO].g0) < 0) for (int k = 0; k < 3; k++) eixo[k] = -eixo[k];
  float desvio = anguloEntre(eixo, s[S_MAO].g0);
  Serial.printf("  eixo de pronacao [% .2f % .2f % .2f] | pureza %.0f%% | desvio da mao %.0f°\n",
                eixo[0], eixo[1], eixo[2], pureza * 100, desvio);

  if (pureza < PRON_PUREZA_MIN) {
    Serial.println(F("  giro misturado (cotovelo/ombro mexeram?) — so a palma deve girar"));
    return false;
  }
  if (desvio > PRON_DESVIO_MAX) {
    Serial.println(F("  eixo muito longe da mao — pulso dobrado ou pose errada"));
    return false;
  }
  copiar3(eixoOut, eixo);
  return true;
}

void calibrar() {
  Serial.println();
  Serial.println(F("=== CALIBRACAO (4 poses, comeca com o braco PARA CIMA) ==="));

  for (int i = 0; i < NUM_MPU; i++) {
    if (!s[i].ok) s[i].ok = iniciarSensor(i);
    if (!s[i].ok) Serial.printf("  %s FALHOU — calibrando sem ele\n", NOME[i]);
    s[i].temFrente = false;
  }
  bool temB = s[S_BRACO].ok, temM = s[S_MAO].ok;

  // ---- POSE 1: para cima -> eixo do braco + bias
  while (true) {
    tituloPose(1, F("BRACO PARA CIMA"),
               F("  braco esticado na vertical, cotovelo reto, pulso reto."));
    esperarParado();
    bool ok = true;
    for (int i = 0; i < NUM_MPU; i++) {
      if (!s[i].ok) continue;
      if (!medirGravidade(i, s[i].g0, s[i].bias)) { ok = false; continue; }
      copiar3(s[i].eixo, s[i].g0);
      copiar3(s[i].g, s[i].g0);
      s[i].w[0] = s[i].w[1] = s[i].w[2] = 0;
      s[i].falhasSeguidas = 0;
      Serial.printf("  %-5s bias=[% .2f % .2f % .2f] °/s\n",
                    NOME[i], s[i].bias[0], s[i].bias[1], s[i].bias[2]);
    }
    if (ok) break;
    Serial.println(F("  repetindo SO a POSE 1"));
  }

  // ---- POSE 2: horizontal -> ganho do ombro + frente do braco
  float gH[NUM_MPU][3];
  bool temH[NUM_MPU] = { false, false };
  ganhoOmb = 1.0f;
  while (temB) {
    tituloPose(2, F("BRACO HORIZONTAL A FRENTE"),
               F("  braco esticado a frente, na altura do ombro, cotovelo reto."));
    esperarParado();
    for (int i = 0; i < NUM_MPU; i++)
      temH[i] = s[i].ok && medirGravidade(i, gH[i], nullptr);
    if (!temH[S_BRACO]) {
      Serial.println(F("  leitura do Braco falhou — repetindo SO a POSE 2"));
      continue;
    }
    float spanB = anguloEntre(gH[S_BRACO], s[S_BRACO].eixo);
    Serial.printf("  ombro: cima -> horizontal = %.1f° (esperado ~90)\n", spanB);
    if (spanB < SPAN_MIN || spanB > SPAN_MAX) {
      Serial.println(F("  amplitude errada — repetindo SO a POSE 2"));
      continue;
    }
    ganhoOmb = 90.0f / spanB;
    copiar3(s[S_BRACO].g, gH[S_BRACO]);
    ancorarFrente(s[S_BRACO]);
    Serial.printf("  ganho ombro=%.2f\n", ganhoOmb);
    break;
  }

  // ---- POSE 3: cotovelo 90° (guarda as gravidades; o ganho sai depois da pose 4,
  //      quando o eixo de pronacao da mao ja for conhecido)
  float g3[NUM_MPU][3];
  bool tem3 = false;
  while (temB && temM) {
    tituloPose(3, F("COTOVELO 90 GRAUS"),
               F("  braco na horizontal a frente, antebraco e mao APONTANDO PARA CIMA."));
    esperarParado();
    if (!medirGravidade(S_BRACO, g3[S_BRACO], nullptr) ||
        !medirGravidade(S_MAO, g3[S_MAO], nullptr)) {
      Serial.println(F("  leitura falhou — repetindo SO a POSE 3"));
      continue;
    }
    float incB3 = inclinacaoDe(g3[S_BRACO], s[S_BRACO].eixo, s[S_BRACO].f, s[S_BRACO].temFrente);
    if (fabsf(incB3 - 90.0f / ganhoOmb) > 30.0f) {
      Serial.printf("  braco saiu da horizontal (%.0f°) — repetindo SO a POSE 3\n", incB3);
      continue;
    }
    tem3 = true;
    break;
  }

  // ---- POSE 4: girar a palma -> eixo de pronacao, depois palma neutra parada
  while (temM) {
    tituloPose(4, F("GIRAR A PALMA"),
               F("  braco horizontal a frente, cotovelo RETO, ombro parado.\n"
                 "  Vire a palma para baixo e para cima (como girar uma chave)."));
    Serial.println(F("  comeca em 3 s..."));
    delay(3000);
    float eixo[3];
    if (!medirEixoPronacao(eixo)) {
      Serial.println(F("  repetindo SO a POSE 4"));
      continue;
    }
    copiar3(s[S_MAO].eixo, eixo);

    Serial.println(F("  agora PARE com a palma NEUTRA (braco ainda horizontal)."));
    esperarParado();
    for (int i = 0; i < NUM_MPU; i++)
      if (s[i].ok) medirGravidade(i, s[i].g, nullptr);
    ancorarFrente(s[S_MAO]);
    if (temB) ancorarFrente(s[S_BRACO]);
    break;
  }

  // ---- zero e ganho do cotovelo com o eixo de pronacao
  cotZero = 0.0f;
  ganhoCot = 1.0f;
  if (temB && temM) {
    // Zero: agora (fim da pose 4) o cotovelo esta reto.
    cotZero = inclinacao(s[S_BRACO]) - inclinacao(s[S_MAO]);
    if (tem3) {
      // Na pose 3 a mao aponta para cima: a frente dela quase nao importa (magP ~ 0).
      float fM[3];
      bool temFM = frenteDe(gH[S_MAO], s[S_MAO].eixo, fM) && temH[S_MAO];
      float incB3 = inclinacaoDe(g3[S_BRACO], s[S_BRACO].eixo, s[S_BRACO].f, s[S_BRACO].temFrente);
      float incM3 = inclinacaoDe(g3[S_MAO], s[S_MAO].eixo, fM, temFM);
      float cot3 = incB3 - incM3 - cotZero;
      Serial.printf("  cotovelo na pose 3 = %.1f° (esperado ~90)\n", cot3);
      if (cot3 >= SPAN_MIN && cot3 <= SPAN_MAX) ganhoCot = 90.0f / cot3;
      else Serial.println(F("  fora da faixa — ganho do cotovelo fica 1.00 (tecla r refaz)"));
    }
  }

  pulso = 0;
  base = 0;
  baseOffset = 0;
  ultimoUs = micros();
  Serial.println();
  Serial.println(F("========================================"));
  Serial.printf("ombro ganho %.2f | cotovelo zero %.1f° ganho %.2f\n", ganhoOmb, cotZero, ganhoCot);
  if (temM) {
    Serial.printf("eixo pronacao da mao a %.0f° da direcao da mao (corrigido)\n",
                  anguloEntre(s[S_MAO].eixo, s[S_MAO].g0));
  }
  Serial.println(F("PRONTO — pode mexer. r = recalibrar | z = zerar pulso/base"));
  Serial.println(F("========================================"));
}

// ---------------------------------------------------------------- LoRa
void aplicarConfigLoRa() {
  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(250E3);
  LoRa.setCodingRate4(5);
  LoRa.enableCrc();
  LoRa.setTxPower(14);
  LoRa.setOCP(140);
}

bool iniciarLoRa() {
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  LoRa.setSPIFrequency(1E6);
  if (!LoRa.begin(LORA_FREQ)) return false;
  aplicarConfigLoRa();
  return true;
}

uint8_t lerRegLoRa(uint8_t reg) {
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(LORA_SS, LOW);
  SPI.transfer(reg & 0x7F);
  uint8_t v = SPI.transfer(0x00);
  digitalWrite(LORA_SS, HIGH);
  SPI.endTransaction();
  return v;
}

// Reset por queda de tensao volta o SX1278 em FSK sem SF/BW: TX "funciona" e ninguem ouve.
bool loraConfigOk() {
  if (lerRegLoRa(0x42) != 0x12) return false;
  if ((lerRegLoRa(0x01) & 0x80) == 0) return false;
  if (lerRegLoRa(0x06) != 0x6C) return false;
  if ((lerRegLoRa(0x1D) & 0xF0) != 0x80) return false;
  if ((lerRegLoRa(0x1E) & 0xF4) != 0x74) return false;
  if (lerRegLoRa(0x39) != 0x12) return false;
  return true;
}

void reiniciarLoRa(const __FlashStringHelper* motivo) {
  Serial.print(F("  !! LoRa ")); Serial.print(motivo); Serial.println(F(" — resetando"));
  LoRa.end();
  delay(20);
  Serial.println(iniciarLoRa() ? F("  !! LoRa reset OK") : F("  !! LoRa reset FALHOU"));
  loraFalhasSeguidas = 0;
}

void enviarLoRa(int8_t c, int8_t o, int8_t p, int8_t b) {
  bool ok = LoRa.beginPacket();
  if (ok) {
    LoRa.write(LORA_HEADER);
    LoRa.write((uint8_t)c);
    LoRa.write((uint8_t)o);
    LoRa.write((uint8_t)p);
    LoRa.write((uint8_t)b);
    for (int i = 0; i < 5; i++) LoRa.write(DEDO_FIXO);
    ok = LoRa.endPacket();
  }
  if (ok) {
    pacotesTx++;
    loraFalhasSeguidas = 0;
  } else {
    falhasTx++;
    if (++loraFalhasSeguidas >= 3) reiniciarLoRa(F("travou"));
  }

  static unsigned long ultimoCheckMs = 0;
  if (millis() - ultimoCheckMs >= 1000) {
    ultimoCheckMs = millis();
    if (!loraConfigOk()) reiniciarLoRa(F("perdeu config"));
  }
}

int8_t paraByte(int j, float graus) {
  graus *= GANHO[j];
  if (INV_TX[j]) graus = -graus;
  graus = constrain(graus, -127.0f, 127.0f);
  return (int8_t)(graus + (graus >= 0 ? 0.5f : -0.5f));
}

// ---------------------------------------------------------------- setup / loop
void setup() {
  Serial.begin(115200);
  delay(300);
  WiFi.mode(WIFI_OFF);
  btStop();

  Serial.println();
  Serial.println(F("=== KAIROS teste-braco-mao-lora (2 MPU + LoRa 0xC2) ==="));

  for (int i = 0; i < NUM_MPU; i++) {
    s[i].addr = MPU_ADDR_GND;
    s[i].eixo[0] = 0; s[i].eixo[1] = 0; s[i].eixo[2] = 1;
    s[i].ok = iniciarSensor(i);
    Serial.printf("  %-5s SDA %d SCL %d  %s", NOME[i], PIN_SDA[i], PIN_SCL[i],
                  s[i].ok ? "OK" : "FALHOU");
    if (s[i].ok) Serial.printf("  addr=0x%02X", s[i].addr);
    Serial.println();
  }

  if (!iniciarLoRa()) {
    Serial.println(F("ERRO LoRa — confira NSS 5 / RST 2 / SPI 18-19-23"));
    while (true) delay(1000);
  }
  Serial.println(F("LoRa TX OK (433 MHz SF7 BW250)"));

  calibrar();
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'r' || c == 'R') calibrar();
    if (c == 'z' || c == 'Z') {
      pulso = 0;
      float az;
      if (azimute(s[S_BRACO], az)) baseOffset = az;
      base = 0;
      Serial.println(F("  pulso e base zerados (base = direcao atual do braco)"));
    }
  }

  unsigned long agora = micros();
  float dt = (agora - ultimoUs) / 1000000.0f;
  ultimoUs = agora;
  if (dt <= 0) dt = 0.001f;
  if (dt > 0.05f) dt = 0.05f;

  bool leuOk[NUM_MPU] = { false, false };
  for (int i = 0; i < NUM_MPU; i++) {
    if (!s[i].ok) continue;
    float a[3], gy[3];
    s[i].leituras++;
    if (!lerSensor(i, a, gy)) {
      s[i].falhas++;
      if (s[i].falhasSeguidas < 60000) s[i].falhasSeguidas++;
      // Sem leitura a velocidade antiga NAO vale: senao a junta sai andando.
      s[i].w[0] = s[i].w[1] = s[i].w[2] = 0;
      if (s[i].falhasSeguidas >= 25) s[i].ok = false;
      continue;
    }
    s[i].falhasSeguidas = 0;
    for (int k = 0; k < 3; k++) s[i].w[k] = gy[k] - s[i].bias[k];

    float magA = sqrtf(dot3(a, a));
    float magW = fabsf(s[i].w[0]) + fabsf(s[i].w[1]) + fabsf(s[i].w[2]);
    if (fabsf(magA - 1.0f) < 0.05f && magW < 3.0f) {
      for (int k = 0; k < 3; k++) s[i].bias[k] += 0.002f * (gy[k] - s[i].bias[k]);
    }
    atualizarGravidade(s[i], a, dt);
    leuOk[i] = true;
  }

  Sensor& B = s[S_BRACO];
  Sensor& M = s[S_MAO];

  // Junta so atualiza com os sensores dela lidos; senao congela no ultimo valor.
  if (leuOk[S_BRACO]) {
    float incB = inclinacao(B);
    jOmb = constrain(incB * ganhoOmb, 0.0f, LIM_OMB_MAX);

    // Braco quase vertical nao tem rumo: a base segura o ultimo valor.
    float az;
    if (azimute(B, az)) base = constrain(wrap180(az - baseOffset), -LIM_BASE, LIM_BASE);

    if (leuOk[S_MAO]) {
      float cot = (incB - inclinacao(M) - cotZero) * ganhoCot;
      jCot = constrain(cot, 0.0f, LIM_COT_MAX);

      // O giro do braco no proprio eixo so vira giro da mao com o cotovelo RETO.
      // Com o cotovelo a 90° ele balanca o antebraco em vez de girar a palma.
      float acoplamento = cosf(jCot * DEG_TO_RAD);
      float taxaPulso = dot3(M.w, M.eixo) - acoplamento * dot3(B.w, B.eixo);
      if (fabsf(taxaPulso) > ZONA_GIRO) pulso += taxaPulso * dt;
      pulso = constrain(pulso, -LIM_PUL, LIM_PUL);
    }
  }

  if (millis() - ultimoTxMs >= TX_PERIODO_MS) {
    ultimoTxMs = millis();
    int8_t c = paraByte(0, jCot);
    int8_t o = paraByte(1, jOmb);
    int8_t p = paraByte(2, pulso);
    int8_t b = paraByte(3, base);
    enviarLoRa(c, o, p, b);

    // Recupera sensor morto so depois do TX, 1 por pacote (init trava ~50 ms).
    for (int i = 0; i < NUM_MPU; i++) {
      if (s[i].ok || millis() - s[i].ultimaRecupMs < 1000) continue;
      s[i].ultimaRecupMs = millis();
      if (iniciarSensor(i)) {
        s[i].ok = true;
        s[i].falhasSeguidas = 0;
        float a[3], gy[3];
        if (lerSensor(i, a, gy)) {
          copiar3(s[i].g, a);
          normalizar3(s[i].g);
        }
        // Orientacao perdida enquanto estava morto: a "frente" antiga nao vale mais.
        s[i].temFrente = false;
        Serial.printf("  !! %s voltou (tecla r recalibra a frente)\n", NOME[i]);
      }
      break;
    }

    static uint8_t div = 0;
    if (++div >= 5) {
      div = 0;
      Serial.printf("C=%4d O=%4d P=%4d B=%4d | Braco %s %3lu%% | Mao %s %3lu%% | tx=%lu falha=%lu\n",
                    c, o, p, b,
                    B.ok ? "ok" : "--", B.leituras ? 100UL * (B.leituras - B.falhas) / B.leituras : 0UL,
                    M.ok ? "ok" : "--", M.leituras ? 100UL * (M.leituras - M.falhas) / M.leituras : 0UL,
                    (unsigned long)pacotesTx, (unsigned long)falhasTx);
      for (int i = 0; i < NUM_MPU; i++) { s[i].leituras = 0; s[i].falhas = 0; }
    }
  }
}
