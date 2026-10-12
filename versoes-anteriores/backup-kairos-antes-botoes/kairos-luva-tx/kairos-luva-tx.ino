/*
 * Kairós — kairos-luva-tx (ESP32, oficial)
 *   3 MPU6050 (Braco + Antebraco +  Mao) + 5 flex (dedos) + LoRa -> kairos-receptor
 *   Ombro e base = sensor do Braco.
 *   Cotovelo = angulo entre Braco e Antebraco (a mao nao entra).
 *   Pulso = giro da Mao em relacao ao Antebraco.
 *
 * Pacote LoRa 11 bytes:
 *   [0xC2, cotovelo, ombro, pulso_lo, pulso_hi, base, d0..d4]
 *   cotovelo, ombro e base = int8 escalado (fundo 150, 130 e 150 graus)
 *   pulso = int16 graus, little-endian (0 na partida, ±180: palma baixo ↔ palma cima)
 *   dedos = uint8 0 (aberto) ou 180 (fechado)
 * Teste de conexao antes da calibracao: envia [0xA1, seq], receptor responde
 *   [0xA2, seq|, rssi]. Em uso, a cada ~1,2 s a luva repete esse PING. Sem PONG
 *   3 vezes seguidas, reseta o radio e fica tentando ate reconectar, sem
 *   parar de mandar a posicao do braco nos intervalos.
 *
 * FLEX (divisor 66kΩ, 3V3):  3V3 --[FLEX]--+-- GPIO / 66k -- GND
 *   D1:34  D2:35  D3:32  D4:33  D5:36 (VP)
 *
 * Cada segmento tem um EIXO (vetor fixo no referencial do sensor):
 *   Braco:     eixo longo = gravidade com tudo PARA CIMA
 *   Antebraco: eixo longo = a mesma gravidade (osso do antebraco)
 *   Mao:       eixo de PRONACAO, medido girando a palma
 *   Cotovelo usa Braco x Antebraco. Dobrar ou girar a mao nao muda esse angulo.
 *   Pulso e' so o giro da Mao em torno do Antebraco.
 *
 * Juntas, no mesmo mundo (cima = acelerometro, frente = gyro desde a calibracao):
 *   ombro    = inclinacao do osso do Braco (0 = cima, 90 = horizontal)
 *   cotovelo = angulo ENTRE o osso do Braco e o osso do Antebraco.
 *              A mao pode dobrar e girar sem mudar esse valor.
 *   pulso    = giro da palma em torno do Antebraco (Mao relativa ao Antebraco).
 *   base     = rumo horizontal do Braco, ate ±150 da frente da calibracao
 *              (90 a esquerda e 90 a direita). Quase vertical, segura o valor.
 *   Ombro, cotovelo e pulso zerados na pose em que o Arduino esta parado:
 *   ombro 0 = braco para cima, cotovelo 0 = reto, pulso 0 = palma neutra.
 *
 * LIGACOES (iguais ao kairos-luva-tx):
 *   Braco: SDA 25 SCL 26    Antebraco: SDA 4 SCL 13    Mao: SDA 27 SCL 14
 *   LoRa: NSS 5  RST 2  DIO0 15  SCK 18  MISO 19  MOSI 23
 *
 * INICIO (Arduino e luva): todos os membros PARA CIMA, palma para TRAS.
 * Motores, sentido "frente" (positivo):
 *   base  (haste p/ cima)    + = esquerda,  - = direita, limite ±150
 *   ombro (haste p/ esquerda)+ = frente,     - = tras
 *   cotovelo (haste p/ direita) + = tras,    - = frente
 *   pulso (haste p/ cima)    + = esquerda,   - = tras, ±180 (palma baixo ↔ palma cima)
 *
 * CALIBRACAO — 7 poses:
 *   1. tudo PARA CIMA, palma para TRAS (zero do ombro)
 *   2. braco A FRENTE na horizontal -> ombro +90, base 0
 *   3. 90° a ESQUERDA -> base +90
 *   4. 90° a DIREITA  -> base -90
 *   5. a frente, antebraco PARA CIMA (dobra para tras) -> cotovelo +90
 *   6. gire a palma uns 6 s (basta um giro claro; sem meta de 150 amostras)
 *   7. VOLTAR ao inicio: tudo para cima, palma para tras
 *      -> cotovelo 0 e pulso 0
 *   Pose errada repete so aquela pose.
 *
 * COMO COMECAR (monitor serial 115200, Arduino receptor ligado):
 *   1. O boot SO testa: 3 MPU (OK/FALHOU) e ping/pong.
 *   2. Nao calibra e nao entra no manual sozinho. A luva nao comanda ainda.
 *   3. r = calibracao completa (dedos, 7 poses e o giro da palma). So depois a luva comanda.
 *      n = entrar no manual. So depois: c escolhe o cotovelo, t vai para a frente,
 *          f nesse eixo vai para tras, s para, n sai.
 *      h = pose do boot do Arduino (manda 0x02/0x03). Sempre o mesmo ponto.
 *      z = zerar pulso e base onde o braco esta (nao mexe no home do Arduino).
 * Comando manual no LoRa: [0xA4, tecla]. 0x01 entra, 0x00 sai, 0x02 home, 0x03 sai do home.
 * Zeros: cotovelo e ombro vem da gravidade. O pulso comeca em 0 na pose
 * inicial e so anda quando a palma gira (gyro da mao). Parado, fica parado.
 * A base nao e' zerada so porque o braco subiu.
 * Ordem do r: dedos, poses 1-6, palma baixo / palma cima, pose 7 (tudo para cima).
 */

#include <SPI.h>
#include <LoRa.h>
#include <WiFi.h>
#include <math.h>

// ---------------------------------------------------------------- pinos
const int NUM_MPU = 3;
#define S_BRACO 0
#define S_MAO   1
#define S_ANTE  2
const int PIN_SDA[NUM_MPU]  = { 25, 27, 4 };
const int PIN_SCL[NUM_MPU]  = { 26, 14, 13 };
const char* NOME[NUM_MPU]   = { "Braco", "Mao", "Antebraco" };

#define LORA_SS   5
#define LORA_RST  2
#define LORA_DIO0 15
const long LORA_FREQ = 400E6;   // SX1278 cobre 137–525 MHz
const uint8_t LORA_HEADER = 0xC2;
const uint8_t LORA_PING   = 0xA1;
const uint8_t LORA_PONG   = 0xA2;
const uint8_t LORA_CMD    = 0xA4;  // [0xA4, tecla] — manual do receptor
const uint8_t CMD_ENTRA   = 0x01;
const uint8_t CMD_SAI     = 0x00;
const uint8_t CMD_HOME_ON  = 0x02;
const uint8_t CMD_HOME_OFF = 0x03;
const int PING_POR_RODADA = 10;
const unsigned long PING_ESPERA_MS = 300;
const unsigned long TX_PERIODO_MS = 40;
const unsigned long LINK_PROBE_MS = 1200;   // PING com o link no ar
const unsigned long LINK_RETRY_MS = 400;    // PING enquanto reconecta
const unsigned long LINK_PONG_MS = 100;     // janela de escuta (link no ar)
const unsigned long LINK_PONG_DOWN_MS = 160;
const int LINK_MISSES_MAX = 3;
const unsigned long LORA_TX_PRESO_MS = 120;  // 11 bytes levam ~20 ms; passou disso, travou

// ---------------------------------------------------------------- flex
const int NUM_FLEX = 5;
const int PIN_FLEX[NUM_FLEX] = { 34, 32, 32, 33, 33 };
const char* NOME_DEDO[NUM_FLEX] = {
  "Polegar", "Indicador", "Medio", "Anelar", "Mindinho"
};

// Dedos so ABERTO (0) ou FECHADO (180), igual ao leitura-dedos.
// Cadeia: miolo do burst -> rejeita pico -> mediana -> filtro 1 euro
//         -> limiares fixos da calibracao (com histerese) -> estado.
// O fechado da calibracao e' um punho normal (~4 s). Segurar mais tempo faz
// a leitura cair alem disso; por isso o limiar nao persegue o valor sozinho.
const int FLEX_ATIVOS = 4;          // mindinho copia o anelar (mesmo GPIO 33)
const uint8_t FLEX_GRAU_ABERTO  = 0;
const uint8_t FLEX_GRAU_FECHADO = 180;

const int FLEX_CONTAGEM_S = 5;      // mao aberta, antes de medir
const int FLEX_TROCA_S = 6;          // abrir de novo e deixar o flex voltar
const int FLEX_FECHA_S = 4;          // punho normal; 20 s media o creep, nao o punho
const unsigned long FLEX_MEDE_MS = 1500;
const float FLEX_SPAN_MIN = 12.0f;  // |dobra| menor que isso = sensor nao respondeu
const int FLEX_TENTATIVAS = 3;

// So dois estados. No meio da dobra o dedo fica como estava.
// Fecha so com punho claro (65%). Abre so com a mao claramente aberta (25%).
const float FLEX_FRAC_ABRE = 0.25f;
const float FLEX_FRAC_FECHA = 0.65f;
const float FLEX_FAIXA_MIN = 0.30f;
const float FLEX_FRAC_FECHA_MAX = 0.85f;
const float FLEX_SIGMAS = 3.0f;
const unsigned long FLEX_CONFIRMA_MS = 350;

const int FLEX_DESCARTA = 24;       // divisor 66k: o ADC segura o pino anterior por mais tempo
const int FLEX_BURST = 64;
const int FLEX_GAP_US = 100;
const int FLEX_MED_N = 5;
const int FLEX_PICO = 140;
const int FLEX_PICO_SEGUIDOS = 3;

// Filtro 1 euro (Casiez 2012)
const float FLEX_MIN_CUTOFF = 2.5f;
const float FLEX_BETA = 0.03f;
const float FLEX_D_CUTOFF = 1.0f;

float flexX[NUM_FLEX];
float flexDx[NUM_FLEX];
float flexAberto[NUM_FLEX];
float flexSpan[NUM_FLEX];
float flexSinal[NUM_FLEX];          // +1: leitura cai ao dobrar, -1: sobe
float flexFracAbre[NUM_FLEX];
float flexFracFecha[NUM_FLEX];
bool flexFechado[NUM_FLEX];
bool flexValido[NUM_FLEX];
bool flexOk[NUM_FLEX];
int flexRejeitados[NUM_FLEX];
unsigned long flexTAnt[NUM_FLEX];
unsigned long flexCondDesde[NUM_FLEX];
int flexMedBuf[NUM_FLEX][FLEX_MED_N];
int flexMedIdx[NUM_FLEX];
int flexMedCheio[NUM_FLEX];
int flexProximo = 0;
uint8_t angulosFlex[NUM_FLEX];

// ---------------------------------------------------------------- ajustes
// Positivo no fio = sentido "frente" do motor (DIR sem inverter no receptor):
//   cotovelo + = antebraco para TRAS     (haste do motor para a direita)
//   ombro    + = braco para a FRENTE     (haste para a esquerda)
//   pulso    + = palma para a ESQUERDA   (haste para cima)
//            - = para TRAS
//            ±180 cobre palma para baixo e palma para cima, antebraco na horizontal
//   base     + = braco para a ESQUERDA   (haste para cima)
// Pulso invertido: com o giro vindo do gyro da mao, sem negar o motor girava
// ao contrario da palma.
//                                  cotovelo ombro  pulso  base
const bool  INV_TX[4]           = { false,   false, true,  false };
const float GANHO[4]            = {  1.0f,  1.0f,  1.0f,  1.0f };

const float LIM_COT_MIN  = -90;    // antebraco para a frente
const float LIM_COT_MAX  = 150;    // antebraco para tras
const float LIM_OMB_MIN  = -90;    // braco para tras
const float LIM_OMB_MAX  = 130;    // braco para a frente / baixo
const float LIM_PUL_TRAS   = -180;  // palma virada num sentido, antebraco na horizontal
const float LIM_PUL_FRENTE =  180;  // o outro sentido: palma baixo ↔ palma cima
const float LIM_BASE     = 150;    // a calibracao mede ±90; o giro pode seguir ate ±150
// O fio manda int8. Cotovelo 150 e base 150 nao cabem em ±127, entao vao
// escalados. O receptor desfaz com os mesmos fundos.
const float FUNDO_COT  = 150.0f;
const float FUNDO_OMB  = 130.0f;
const float FUNDO_BASE = 150.0f;
const float FIO_CHEIO  = 127.0f;

const float ACC_PESO     = 0.02f;  // quanto o acelerometro corrige a gravidade
const float ACC_TOL_G    = 0.15f;  // |a| fora de 1 g +- isso = aceleracao, ignora
// Girando rapido, o sensor fora do eixo sente forca centripeta: accel mente.
const float ACC_GIRO_MAX = 150.0f; // °/s (soma |w|) acima disso so o gyro manda
const float ZONA_GIRO     = 2.5f;  // °/s abaixo disso nao integra o pulso
// Giro da palma abaixo disso e' bias/ruido. Mao humana girando passa de 20 °/s.
const float ZONA_GIRO_PULSO = 4.0f;
// Soma |wx|+|wy|+|wz|. Abaixo disso a frente e a torcao nao integram:
// o resto e' ruido/bias, e integrar vira a base andando sozinha.
const float ZONA_INTEGRAR = 0.6f;
// Parado de verdade = a gravidade nao gira (~2°). O giro sozinho mente:
// um movimento lento de 3 °/s tem a mesma cara de bias.
const float QUIETO_COS    = 0.9994f;
const unsigned long QUIETO_US = 400000UL;
// |gyro sem bias| acima disso nunca e' parado (giro em torno da vertical).
// Bias do MPU6050 depois da pose 1 fica bem abaixo de 1 °/s.
const float QUIETO_GIRO   = 3.0f;
// Cotovelo e' dobradica: o antebraco fica no plano perpendicular ao eixo.
// Folga para o angulo de carregamento (~15°) e erro de colocacao.
const float COT_FOLGA_SEN = 0.40f;   // sen(~24°)
const float COT_PUXA      = 0.05f;   // fracao do excesso corrigida por amostra
const float COT_FORA_MAX  = 0.70f;   // >~45° fora do plano: leitura nao vale
const float BIAS_K        = 0.005f;   // so com o braco parado
const float BIAS_COEF_MAX = 0.30f;    // °/s por °C, teto do que se aprende
const int   N_CAL        = 300;
const float INICIO_INC_BRACO = 8.0f;    // so quase na vertical conta como inicio
const float INICIO_INC_ANTE  = 10.0f;
const float INICIO_INC_MAO   = 12.0f;
const unsigned long INICIO_QUIETO_MS = 1500;
const float INICIO_PUXA = 0.30f;        // 1/s: so o drift, sem puxar a pose

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
const int   PRON_AMOSTRAS   = 40;
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
  float ref[3];    // "cima" do segmento, fixa no sensor, perpendicular ao osso
  bool  temRef;
  float bias[3];   // bias do gyro (°/s) na temperatura biasTemp
  float w[3];      // gyro sem bias (°/s)
  float tempC;
  float biasTemp;
  float biasCoef[3];    // °/s por °C, aprendido parado
  float biasInicio[3];
  float tempInicio;
  bool  temBiasTemp;
  bool  biasAssentado;  // zero ja acompanhou o giro; daqui o que muda e' temperatura
  float aQuieto[3];
  bool  temQuieto;
  unsigned long quietoDesdeUs;
  bool  quieto;         // gravidade parada ha QUIETO_US — nao e' "giro pequeno"
  uint16_t falhasSeguidas;
  unsigned long ultimaRecupMs;
  uint32_t leituras, falhas;
};
Sensor s[NUM_MPU];

float pulso = 0, base = 0, baseOffset = 0;
bool gravPulsoOk = false;
bool sinalPrumoOk = false;
float sinalPrumo = 1.0f;
float acordoPrumo = 0;
float gravPulsoBase = 0;
float pulsoAnterior = 0;
float jCot = 0, jOmb = 0;
float ganhoOmb = 1.0f, cotZero = 0.0f, ganhoCot = 1.0f, pulsoZero = 0.0f;
float ganhoBase = 1.0f;
float ganhoPulso = 1.0f;       // 180° / o que o sensor viu entre palma baixo e palma cima
float pulsoGravZero = 0.0f;    // gravidade com a palma para baixo
float sinalGiroPulso = 1.0f;   // gyro no mesmo sentido dessa gravidade
bool temCalPulso = false;
bool emHome = false;   // h: pacote fica em zero ate apertar h de novo
bool manualRemoto = false;  // n: teclas do receptor vao pelo LoRa
bool bracoCalibrado = false;  // r: so entao o 0xC2 leva a pose da luva
// Rba: antebraco no braco (cotovelo). Rmf: mao no antebraco (pulso).
// Sem o antebraco, a mao entra direto no braco.
float Rba[3][3];
float Rmf[3][3];
bool temRel = false;
bool temAnteRel = false;
bool temMaoRel = false;
float eixoCot[3];
bool temEixoCot = false;
unsigned long ultimoUs = 0, ultimoTxMs = 0;
uint32_t pacotesTx = 0, falhasTx = 0;
int loraFalhasSeguidas = 0;
bool loraConectado = true;
int linkMisses = 0;
unsigned long ultimoProbeMs = 0;
unsigned long probeDeadline = 0;
bool esperandoPong = false;
uint8_t probeSeq = 0;

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
  // Bytes 6-7: TEMP_OUT. 340 LSB/°C, 0 LSB = 36,53 °C (datasheet).
  int16_t tr = (int16_t)((b[6] << 8) | b[7]);
  s[i].tempC = tr / 340.0f + 36.53f;
  return (fabsf(a[0]) + fabsf(a[1]) + fabsf(a[2])) > 0.1f;
}

// bias na temperatura de agora. Parado, bias[] e' reescrito e biasTemp acompanha.
float gyroSemBias(const Sensor& S, int k, float gy) {
  float b = S.bias[k];
  if (S.temBiasTemp)
    b += S.biasCoef[k] * (S.tempC - S.biasTemp);
  return gy - b;
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

void zerarPrumoPulso() {
  gravPulsoOk = false;
  sinalPrumoOk = false;
  sinalPrumo = 1.0f;
  acordoPrumo = 0;
  gravPulsoBase = 0;
  pulsoAnterior = pulso;
}

// Projecao da gravidade no plano do pulso. Falha com o braco na vertical.
bool projetaRelogio(const float v[3], const float ax[3], float out[3]) {
  float c = dot3(v, ax);
  for (int k = 0; k < 3; k++) out[k] = v[k] - c * ax[k];
  float n = sqrtf(dot3(out, out));
  if (n < 0.50f) return false;   // abaixo de ~30° da vertical o angulo e' ruido
  for (int k = 0; k < 3; k++) out[k] /= n;
  return true;
}

bool pulsoPelaGravidade(float& graus) {
  Sensor& M = s[S_MAO];
  Sensor& A = s[S_ANTE];
  if (!M.ok || !A.ok || !M.temRef || !A.temRef) return false;
  float pm[3], pa[3];
  if (!projetaRelogio(M.g, M.eixo, pm)) return false;
  if (!projetaRelogio(A.g, A.eixo, pa)) return false;
  float am = anguloEmTorno(M.eixo, M.ref, pm);
  float aa = anguloEmTorno(A.eixo, A.ref, pa);
  graus = sinalPrumo * wrap180(am - aa);
  return true;
}

// Giro da palma em torno do antebraco, lido pela gravidade. Com o antebraco
// deitado isso e' a pronacao inteira (palma baixo ↔ palma cima), nao um pedaco.
bool pulsoGravidadeBruta(float& graus) {
  Sensor& M = s[S_MAO];
  Sensor& A = s[S_ANTE];
  if (!M.ok || !A.ok || !M.temRef || !A.temRef) return false;
  float pm[3], pa[3];
  if (!projetaRelogio(M.g, M.eixo, pm)) return false;
  if (!projetaRelogio(A.g, A.eixo, pa)) return false;
  float am = anguloEmTorno(M.eixo, M.ref, pm);
  float aa = anguloEmTorno(A.eixo, A.ref, pa);
  graus = wrap180(am - aa);
  return true;
}

float taxaPulso() {
  // So a mao. Tirar o antebraco anulava o giro: os dois sensores giram juntos.
  return dot3(s[S_MAO].w, s[S_MAO].eixo);
}

// Quanto a palma girou em torno do proprio eixo, vista pela gravidade.
// Nao usa o antebraco: senao o giro da palma se cancela.
bool pulsoGravidadeMao(float& graus) {
  Sensor& M = s[S_MAO];
  if (!M.ok || !M.temRef) return false;
  float pm[3];
  if (!projetaRelogio(M.g, M.eixo, pm)) return false;
  graus = anguloEmTorno(M.eixo, M.ref, pm);
  return true;
}

// Filtro complementar do giro do pulso. O gyro integra; a gravidade corrige
// devagar, e so quando o antebraco nao esta na vertical. Ganho alto puxava
// o pulso no meio do gesto e o zero andava para o lado errado.
void segurarPrumoPulso(float tw, float dt) {
  Sensor& M = s[S_MAO];
  Sensor& A = s[S_ANTE];
  float taxa = dot3(M.w, M.eixo) - dot3(A.w, A.eixo);
  float pulsoGyro = wrap180(tw - pulsoZero);
  float dP = pulsoGyro - pulsoAnterior;
  pulsoAnterior = pulsoGyro;

  if (!sinalPrumoOk && fabsf(dP) > 1.2f && fabsf(taxa) > 20.0f) {
    acordoPrumo += (dP > 0 ? 1.0f : -1.0f) * (taxa > 0 ? 1.0f : -1.0f);
    if (acordoPrumo >= 10.0f) { sinalPrumo = 1.0f; sinalPrumoOk = true; }
    if (acordoPrumo <= -10.0f) { sinalPrumo = -1.0f; sinalPrumoOk = true; }
    if (sinalPrumoOk) {
      gravPulsoOk = false;
      Serial.println(F("  prumo do pulso ativo"));
    }
  }
  if (!sinalPrumoOk || fabsf(taxa) > 40.0f) return;

  float gRel;
  if (!pulsoPelaGravidade(gRel)) return;
  if (!gravPulsoOk) {
    gravPulsoBase = wrap180(gRel - pulsoGyro);
    gravPulsoOk = true;
    return;
  }
  float pulsoGrav = wrap180(gRel - gravPulsoBase);
  float err = wrap180(pulsoGyro - pulsoGrav);
  bool parado = M.quieto && A.quieto;
  if (fabsf(err) > 18.0f && !parado) return;
  float ganho = parado ? 0.35f : 0.12f;
  if (fabsf(err) > 18.0f) ganho = 0.08f;
  float k = ganho * dt;
  if (k > 0.02f) k = 0.02f;
  pulsoZero = wrap180(pulsoZero + err * k);
}

void produtoVetorial(const float a[3], const float b[3], float o[3]) {
  o[0] = a[1] * b[2] - a[2] * b[1];
  o[1] = a[2] * b[0] - a[0] * b[2];
  o[2] = a[0] * b[1] - a[1] * b[0];
}

// v (no sensor) -> mundo: frente, esquerda, cima. g e f estao no sensor.
bool mundoDe(const float g[3], const float f[3], const float v[3], float w[3]) {
  float up[3] = { g[0], g[1], g[2] };
  if (!normalizar3(up)) return false;
  float frente[3];
  float c = dot3(f, up);
  for (int k = 0; k < 3; k++) frente[k] = f[k] - c * up[k];
  if (!normalizar3(frente)) return false;
  float esq[3];
  produtoVetorial(up, frente, esq);
  if (!normalizar3(esq)) return false;
  w[0] = dot3(v, frente);
  w[1] = dot3(v, esq);
  w[2] = dot3(v, up);
  return true;
}

bool vetorNoMundo(const Sensor& S, const float v[3], float w[3]) {
  if (!S.temFrente) return false;
  return mundoDe(S.g, S.f, v, w);
}

bool projetarPerp(const float v[3], const float eixo[3], float out[3]) {
  float c = dot3(v, eixo);
  for (int k = 0; k < 3; k++) out[k] = v[k] - c * eixo[k];
  return normalizar3(out);
}

void rodrigues(const float v[3], const float axis[3], float ang, float out[3]) {
  float c = cosf(ang), s = sinf(ang);
  float cx[3];
  produtoVetorial(axis, v, cx);
  float d = dot3(axis, v);
  for (int k = 0; k < 3; k++)
    out[k] = v[k] * c + cx[k] * s + axis[k] * d * (1.0f - c);
}

// Gira a "cima" do braco ate o antebraco, sem enrolar em torno do osso.
bool referenciaSemTorcao(const float ossoB[3], const float ossoM[3],
                         const float refBw[3], float esperado[3]) {
  float n[3];
  produtoVetorial(ossoB, ossoM, n);
  float sen = sqrtf(dot3(n, n));
  float cosA = dot3(ossoB, ossoM);
  if (cosA > 1.0f) cosA = 1.0f;
  if (cosA < -1.0f) cosA = -1.0f;
  float girado[3];
  if (sen < 0.05f) copiar3(girado, refBw);
  else {
    for (int k = 0; k < 3; k++) n[k] /= sen;
    rodrigues(refBw, n, atan2f(sen, cosA), girado);
  }
  return projetarPerp(girado, ossoM, esperado);
}

float anguloEmTorno(const float eixo[3], const float de[3], const float para[3]) {
  float cruz[3];
  produtoVetorial(de, para, cruz);
  return atan2f(dot3(eixo, cruz), dot3(de, para)) * 180.0f / PI;
}

// "Cima" gravada no sensor, perpendicular ao osso. Palma neutra = as duas cimas
// apontam para o mesmo lado.
bool capturarRef(Sensor& S) {
  float c = dot3(S.g, S.eixo);
  for (int k = 0; k < 3; k++) S.ref[k] = S.g[k] - c * S.eixo[k];
  S.temRef = normalizar3(S.ref);
  return S.temRef;
}

void aplicarR(const float R[3][3], const float v[3], float o[3]) {
  for (int i = 0; i < 3; i++)
    o[i] = R[i][0] * v[0] + R[i][1] * v[1] + R[i][2] * v[2];
}

void ortoR(float R[3][3]) {
  float c0[3] = { R[0][0], R[1][0], R[2][0] };
  float c1[3] = { R[0][1], R[1][1], R[2][1] };
  if (!normalizar3(c0)) return;
  float d = dot3(c1, c0);
  for (int k = 0; k < 3; k++) c1[k] -= d * c0[k];
  if (!normalizar3(c1)) return;
  float c2[3];
  produtoVetorial(c0, c1, c2);
  for (int k = 0; k < 3; k++) {
    R[k][0] = c0[k];
    R[k][1] = c1[k];
    R[k][2] = c2[k];
  }
}

bool eixosDoSensor(const Sensor& S, float fr[3], float esq[3], float up[3]) {
  if (!S.temFrente) return false;
  copiar3(up, S.g);
  if (!normalizar3(up)) return false;
  float c = dot3(S.f, up);
  for (int k = 0; k < 3; k++) fr[k] = S.f[k] - c * up[k];
  if (!normalizar3(fr)) return false;
  produtoVetorial(up, fr, esq);
  return normalizar3(esq);
}

bool montarR(const Sensor& pai, const Sensor& filho, float R[3][3]) {
  float frP[3], esqP[3], upP[3], frF[3], esqF[3], upF[3];
  if (!eixosDoSensor(pai, frP, esqP, upP)) return false;
  if (!eixosDoSensor(filho, frF, esqF, upF)) return false;
  for (int j = 0; j < 3; j++) {
    float w0 = frF[j], w1 = esqF[j], w2 = upF[j];
    for (int i = 0; i < 3; i++)
      R[i][j] = w0 * frP[i] + w1 * esqP[i] + w2 * upP[i];
  }
  ortoR(R);
  return true;
}

bool iniciarRelativo() {
  temAnteRel = false;
  temMaoRel = false;
  temEixoCot = false;
  temRel = false;
  Sensor& B = s[S_BRACO];
  Sensor& A = s[S_ANTE];
  Sensor& M = s[S_MAO];
  float fr[3], esq[3], up[3];
  if (!eixosDoSensor(B, fr, esq, up)) return false;
  for (int k = 0; k < 3; k++) eixoCot[k] = -esq[k];
  temEixoCot = true;

  if (A.ok && A.temFrente && montarR(B, A, Rba)) {
    temAnteRel = true;
    if (M.ok && M.temFrente && montarR(A, M, Rmf)) temMaoRel = true;
  } else if (M.ok && M.temFrente && montarR(B, M, Rba)) {
    Serial.println(F("  Antebraco ausente — cotovelo volta a usar a Mao"));
  } else {
    return false;
  }
  temRel = true;
  return true;
}

void integrarUm(float R[3][3], const Sensor& pai, const Sensor& filho,
                const float eixoTrava[3], float dt) {
  float wP[3], wF[3];
  float somaP = fabsf(pai.w[0]) + fabsf(pai.w[1]) + fabsf(pai.w[2]);
  float somaF = fabsf(filho.w[0]) + fabsf(filho.w[1]) + fabsf(filho.w[2]);
  bool usaP = !pai.quieto && somaP >= ZONA_INTEGRAR;
  bool usaF = !filho.quieto && somaF >= ZONA_INTEGRAR;
  for (int k = 0; k < 3; k++) {
    wP[k] = (usaP ? pai.w[k] : 0) * DEG_TO_RAD;
    wF[k] = (usaF ? filho.w[k] : 0) * DEG_TO_RAD;
  }
  for (int j = 0; j < 3; j++) {
    float col[3] = { R[0][j], R[1][j], R[2][j] };
    float wPc[3];
    produtoVetorial(wP, col, wPc);
    float wx[3];
    if (j == 0)      { wx[0] = 0;       wx[1] = wF[2];  wx[2] = -wF[1]; }
    else if (j == 1) { wx[0] = -wF[2];  wx[1] = 0;      wx[2] = wF[0]; }
    else             { wx[0] = wF[1];   wx[1] = -wF[0]; wx[2] = 0; }
    for (int i = 0; i < 3; i++) {
      float rw = R[i][0] * wx[0] + R[i][1] * wx[1] + R[i][2] * wx[2];
      R[i][j] += (-wPc[i] + rw) * dt;
    }
  }

  float magW = 0;
  for (int k = 0; k < 3; k++) magW += fabsf(pai.w[k]) + fabsf(filho.w[k]);
  if (magW < 40.0f) {
    float gP[3], gF[3], gFp[3];
    copiar3(gP, pai.g);
    copiar3(gF, filho.g);
    if (normalizar3(gP) && normalizar3(gF)) {
      aplicarR(R, gF, gFp);
      float err[3];
      produtoVetorial(gFp, gP, err);
      float c = dot3(err, eixoTrava);
      for (int k = 0; k < 3; k++) err[k] -= c * eixoTrava[k];
      float me = sqrtf(dot3(err, err));
      if (me > 1e-4f && me < 0.4f) {
        for (int j = 0; j < 3; j++) {
          float col[3] = { R[0][j], R[1][j], R[2][j] };
          float cx[3];
          produtoVetorial(err, col, cx);
          for (int i = 0; i < 3; i++) R[i][j] += 0.08f * cx[i];
        }
      }
    }
  }
  ortoR(R);
}

// Sem magnetometro, o giro entre os dois sensores em torno da vertical nao
// tem correcao pela gravidade e acumula. O cotovelo so dobra num plano, entao
// o osso do antebraco tem que ficar perto do plano perpendicular a eixoCot.
// O que passar da folga e' drift: gira R de volta, um pouco por amostra.
void travarDobradica(float R[3][3], const float ossoFilho[3]) {
  if (!temEixoCot) return;
  float u[3];
  aplicarR(R, ossoFilho, u);
  if (!normalizar3(u)) return;
  float c = dot3(u, eixoCot);
  if (fabsf(c) <= COT_FOLGA_SEN) return;
  float alvoC = c > 0 ? COT_FOLGA_SEN : -COT_FOLGA_SEN;
  float uP[3];
  for (int k = 0; k < 3; k++) uP[k] = u[k] - (c - alvoC) * eixoCot[k];
  if (!normalizar3(uP)) return;
  float d[3];
  produtoVetorial(u, uP, d);
  for (int k = 0; k < 3; k++) d[k] *= COT_PUXA;
  for (int j = 0; j < 3; j++) {
    float col[3] = { R[0][j], R[1][j], R[2][j] };
    float cx[3];
    produtoVetorial(d, col, cx);
    for (int i = 0; i < 3; i++) R[i][j] += cx[i];
  }
  ortoR(R);
}

void integrarRelativo(float dt) {
  if (!temRel || dt <= 0) return;
  if (dt > 0.05f) dt = 0.05f;
  if (temAnteRel) {
    float u[3];
    aplicarR(Rba, s[S_ANTE].eixo, u);
    if (!normalizar3(u)) copiar3(u, s[S_BRACO].eixo);
    integrarUm(Rba, s[S_BRACO], s[S_ANTE], u, dt);
    travarDobradica(Rba, s[S_ANTE].eixo);
  } else {
    float u[3];
    aplicarR(Rba, s[S_MAO].eixo, u);
    if (!normalizar3(u)) copiar3(u, s[S_BRACO].eixo);
    integrarUm(Rba, s[S_BRACO], s[S_MAO], u, dt);
    travarDobradica(Rba, s[S_MAO].eixo);
  }
}

bool calcularAntebracoPulso(float& cotBruto, float& torcaoGraus) {
  Sensor& B = s[S_BRACO];
  Sensor& A = s[S_ANTE];
  Sensor& M = s[S_MAO];
  torcaoGraus = 0;
  if (temRel && temEixoCot && temAnteRel) {
    float u[3], ossoB[3];
    aplicarR(Rba, A.eixo, u);
    copiar3(ossoB, B.eixo);
    if (!normalizar3(u) || !normalizar3(ossoB)) return false;
    // Antebraco quase em cima do eixo do cotovelo: o atan2 vira ruido e
    // pulava para ±90 (o L). Segura o ultimo valor.
    if (fabsf(dot3(u, eixoCot)) > COT_FORA_MAX) return false;
    cotBruto = anguloEmTorno(eixoCot, ossoB, u);
    if (temMaoRel && A.temRef && M.temRef) {
      float ossoM[3], refM[3], esperado[3], palma[3], ossoA[3];
      aplicarR(Rmf, M.eixo, ossoM);
      aplicarR(Rmf, M.ref, refM);
      copiar3(ossoA, A.eixo);
      if (normalizar3(ossoM) && normalizar3(ossoA) &&
          referenciaSemTorcao(ossoA, ossoM, A.ref, esperado) &&
          projetarPerp(refM, ossoM, palma))
        torcaoGraus = anguloEmTorno(ossoM, esperado, palma);
    }
    return true;
  }
  if (temRel && temEixoCot) {
    float u[3], ossoB[3];
    aplicarR(Rba, M.eixo, u);
    copiar3(ossoB, B.eixo);
    if (!normalizar3(u) || !normalizar3(ossoB)) return false;
    if (fabsf(dot3(u, eixoCot)) > COT_FORA_MAX) return false;
    cotBruto = anguloEmTorno(eixoCot, ossoB, u);
    if (B.temRef && M.temRef) {
      float refM[3], esperado[3], palma[3];
      aplicarR(Rba, M.ref, refM);
      if (referenciaSemTorcao(ossoB, u, B.ref, esperado) && projetarPerp(refM, u, palma))
        torcaoGraus = anguloEmTorno(u, esperado, palma);
    }
    return true;
  }
  float ossoBw[3], ossoMw[3];
  if (!vetorNoMundo(B, B.eixo, ossoBw) || !vetorNoMundo(M, M.eixo, ossoMw)) return false;
  if (!normalizar3(ossoBw) || !normalizar3(ossoMw)) return false;
  float direito[3] = { 0.0f, -1.0f, 0.0f };
  cotBruto = anguloEmTorno(direito, ossoBw, ossoMw);
  if (!B.temRef || !M.temRef) return true;
  float refBw[3], refMw[3], esperado[3], palma[3];
  if (!vetorNoMundo(B, B.ref, refBw) || !vetorNoMundo(M, M.ref, refMw)) return true;
  if (!referenciaSemTorcao(ossoBw, ossoMw, refBw, esperado)) return true;
  if (!projetarPerp(refMw, ossoMw, palma)) return true;
  torcaoGraus = anguloEmTorno(ossoMw, esperado, palma);
  return true;
}

// Quase na vertical e parado. Longe disso, "puxar o zero" comia a pose de verdade.
bool poseInicioQuieta() {
  if (!s[S_BRACO].ok || !s[S_BRACO].quieto) return false;
  if (fabsf(inclinacao(s[S_BRACO])) > INICIO_INC_BRACO) return false;
  if (s[S_ANTE].ok) {
    if (!s[S_ANTE].quieto) return false;
    if (fabsf(inclinacao(s[S_ANTE])) > INICIO_INC_ANTE) return false;
  }
  if (s[S_MAO].ok) {
    if (!s[S_MAO].quieto) return false;
    if (fabsf(inclinacao(s[S_MAO])) > INICIO_INC_MAO) return false;
  }
  return true;
}

// Dois ossos na vertical: o cotovelo real e' ~0, entao o drift desse angulo
// pode sair. A base nao entra: no alto nao ha rumo, e zera-la girava o robo.
// O pulso so perde um resto pequeno; palma girada de proposito fica.
void puxarZerosInicio(float cotBruto, float tw, float dt) {
  static unsigned long t0 = 0;
  if (!bracoCalibrado || !poseInicioQuieta()) {
    t0 = 0;
    return;
  }
  if (!t0) t0 = millis();
  if (millis() - t0 < INICIO_QUIETO_MS) return;

  float k = INICIO_PUXA * dt;
  if (k > 0.02f) k = 0.02f;
  cotZero += (cotBruto - cotZero) * k;
  (void)tw;
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
// Parado, ou com giro menor que o ruido, f nao integra: esse resto e' o que
// fazia a base girar sozinha. g continua sendo puxado pela gravidade.
void atualizarGravidade(Sensor& S, const float a[3], float dt) {
  float magW = fabsf(S.w[0]) + fabsf(S.w[1]) + fabsf(S.w[2]);
  bool integra = !S.quieto && magW >= ZONA_INTEGRAR && dt > 0;
  float wr[3] = { S.w[0] * (float)DEG_TO_RAD, S.w[1] * (float)DEG_TO_RAD,
                  S.w[2] * (float)DEG_TO_RAD };
  if (integra) {
    girarNoMundo(S.g, wr, dt);
    normalizar3(S.g);
  }

  float magA = sqrtf(dot3(a, a));
  if (fabsf(magA - 1.0f) < ACC_TOL_G && magW < ACC_GIRO_MAX) {
    float peso = S.quieto ? 0.05f : ACC_PESO;
    for (int k = 0; k < 3; k++)
      S.g[k] = (1.0f - peso) * S.g[k] + peso * (a[k] / magA);
    normalizar3(S.g);
  }

  if (S.temFrente) {
    if (integra) girarNoMundo(S.f, wr, dt);
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
  Serial.printf("POSE %d/7 — ", n);
  Serial.println(titulo);
  Serial.println(dica);
}

// Entre uma pose e outra o braco muda de lugar. Sem isso a "frente" do mundo
// fica presa na pose anterior e o cotovelo/pulso nascem tortos.
void seguirSensores(float dt) {
  if (dt <= 0 || dt > 0.05f) dt = 0.02f;
  for (int i = 0; i < NUM_MPU; i++) {
    if (!s[i].ok) continue;
    float a[3], gy[3];
    if (!lerSensor(i, a, gy)) continue;
    s[i].quieto = false;
    for (int k = 0; k < 3; k++) s[i].w[k] = gyroSemBias(s[i], k, gy[k]);
    atualizarGravidade(s[i], a, dt);
  }
}

void integrarPor(unsigned long ms) {
  unsigned long t0 = millis(), tUs = micros();
  while (millis() - t0 < ms) {
    unsigned long ag = micros();
    float dt = (ag - tUs) / 1000000.0f;
    tUs = ag;
    seguirSensores(dt);
    delay(5);
  }
}

void esperarParado() {
  Serial.println(F("  entre na pose. Leitura em 10 s."));
  for (int t = 10; t >= 1; t--) {
    Serial.println(t);
    integrarPor(1000);
  }
  Serial.println(F("  medindo..."));
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
    float aM[3], gM[3], aB[3], gB[3], aA[3], gA[3];
    bool okM = lerSensor(S_MAO, aM, gM);
    bool okB = s[S_BRACO].ok ? lerSensor(S_BRACO, aB, gB) : false;
    bool okA = s[S_ANTE].ok ? lerSensor(S_ANTE, aA, gA) : false;
    unsigned long agora = micros();
    float dt = (agora - tAntUs) / 1000000.0f;
    tAntUs = agora;
    if (dt > 0.05f) dt = 0.05f;
    if (okM) {
      s[S_MAO].quieto = false;
      for (int k = 0; k < 3; k++) s[S_MAO].w[k] = gyroSemBias(s[S_MAO], k, gM[k]);
      atualizarGravidade(s[S_MAO], aM, dt);
    }
    if (okB) {
      s[S_BRACO].quieto = false;
      for (int k = 0; k < 3; k++) s[S_BRACO].w[k] = gyroSemBias(s[S_BRACO], k, gB[k]);
      atualizarGravidade(s[S_BRACO], aB, dt);
    }
    if (okA) {
      s[S_ANTE].quieto = false;
      for (int k = 0; k < 3; k++) s[S_ANTE].w[k] = gyroSemBias(s[S_ANTE], k, gA[k]);
      atualizarGravidade(s[S_ANTE], aA, dt);
    }
    if (!okM) continue;

    float w[3];
    for (int k = 0; k < 3; k++) w[k] = s[S_MAO].w[k];
    float magW = sqrtf(dot3(w, w));

    // Amplitude aproximada: giro em torno da direcao da mao (so para o aviso).
    giroAcum += dot3(w, s[S_MAO].g0) * dt;
    if (giroAcum < giroMin) giroMin = giroAcum;
    if (giroAcum > giroMax) giroMax = giroAcum;

    if (okB) {
      float wb[3];
      for (int k = 0; k < 3; k++) wb[k] = s[S_BRACO].w[k];
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

// Azimute do braco em relacao a frente (baseOffset), sem reconstruir a frente.
bool lerAzimuteBraco(float& azRel) {
  float g[3];
  if (!medirGravidade(S_BRACO, g, nullptr)) return false;
  copiar3(s[S_BRACO].g, g);
  if (s[S_BRACO].temFrente) {
    float c = dot3(s[S_BRACO].f, s[S_BRACO].g);
    for (int k = 0; k < 3; k++) s[S_BRACO].f[k] -= c * s[S_BRACO].g[k];
    if (!normalizar3(s[S_BRACO].f)) s[S_BRACO].temFrente = false;
  }
  float az;
  if (!azimute(s[S_BRACO], az)) {
    Serial.println(F("  braco quase vertical — deixe na horizontal"));
    return false;
  }
  azRel = wrap180(az - baseOffset);
  return true;
}

// Parado = gravidade parada ~2° por QUIETO_US E gyro quase zero.
// So a gravidade nao basta: girar o braco esticado para o lado e' giro em
// torno da vertical, a gravidade nao muda no sensor, e esse giro era jogado
// fora e aprendido como bias. Um sensor "parado" e o outro nao desmontava o
// cotovelo (formava o L sozinho) e a base nao seguia.
void marcarQuieto(Sensor& S, const float a[3], const float gy[3], unsigned long agoraUs) {
  float magA = sqrtf(dot3(a, a));
  if (fabsf(magA - 1.0f) > 0.08f) {
    S.temQuieto = false;
    S.quieto = false;
    return;
  }
  float w2 = 0;
  for (int k = 0; k < 3; k++) {
    float w = gyroSemBias(S, k, gy[k]);
    w2 += w * w;
  }
  if (w2 > QUIETO_GIRO * QUIETO_GIRO) {
    S.temQuieto = false;
    S.quieto = false;
    S.biasAssentado = false;
    return;
  }
  float au[3] = { a[0] / magA, a[1] / magA, a[2] / magA };
  if (!S.temQuieto) {
    copiar3(S.aQuieto, au);
    S.temQuieto = true;
    S.quietoDesdeUs = agoraUs;
    S.quieto = false;
    return;
  }
  if (dot3(au, S.aQuieto) < QUIETO_COS) {
    copiar3(S.aQuieto, au);
    S.quietoDesdeUs = agoraUs;
    S.quieto = false;
    return;
  }
  S.quieto = (unsigned long)(agoraUs - S.quietoDesdeUs) >= QUIETO_US;
  if (!S.quieto) S.biasAssentado = false;
}

void puxarBias(Sensor& S, const float gy[3]) {
  if (!S.quieto) return;
  for (int k = 0; k < 3; k++) {
    float pred = S.bias[k];
    if (S.temBiasTemp)
      pred += S.biasCoef[k] * (S.tempC - S.biasTemp);
    pred += BIAS_K * (gy[k] - pred);
    S.bias[k] = pred;
  }
  S.biasTemp = S.tempC;
  S.temBiasTemp = true;

  // So aprende °/s por °C depois que o zero ja alcancou o giro. Senao o
  // primeiro acerto (que nao e' temperatura) vira um coeficiente e a base anda mais.
  float resid = 0;
  for (int k = 0; k < 3; k++) resid += fabsf(gy[k] - S.bias[k]);
  if (resid > 0.20f) {
    S.biasAssentado = false;
    return;
  }
  if (!S.biasAssentado) {
    copiar3(S.biasInicio, S.bias);
    S.tempInicio = S.tempC;
    S.biasAssentado = true;
    return;
  }
  float dT = S.tempC - S.tempInicio;
  if (fabsf(dT) < 0.5f) return;
  for (int k = 0; k < 3; k++) {
    float slope = (S.bias[k] - S.biasInicio[k]) / dT;
    if (slope > BIAS_COEF_MAX) slope = BIAS_COEF_MAX;
    if (slope < -BIAS_COEF_MAX) slope = -BIAS_COEF_MAX;
    S.biasCoef[k] = 0.9f * S.biasCoef[k] + 0.1f * slope;
  }
  copiar3(S.biasInicio, S.bias);
  S.tempInicio = S.tempC;
}

// O zero eletrico assenta em milissegundos; o zero termico, nao.
// Se a temperatura ainda sobe, espera. Se ja estabilizou, sai em ~3 s.
void esperarAquecer() {
  bool algum = false;
  for (int i = 0; i < NUM_MPU; i++) if (s[i].ok) algum = true;
  if (!algum) return;

  Serial.println(F("Aquecendo os MPU — o zero do giro muda com a temperatura."));
  float tAnt[NUM_MPU];
  bool temAnt[NUM_MPU] = { false, false, false };
  unsigned long t0 = millis();
  unsigned long janela = millis();
  while (millis() - t0 < 20000UL) {
    delay(40);
    for (int i = 0; i < NUM_MPU; i++) {
      if (!s[i].ok) continue;
      float a[3], gy[3];
      lerSensor(i, a, gy);
    }
    if (millis() - janela < 1500UL) continue;
    janela = millis();
    bool sobe = false;
    float mostra = 0;
    bool temMostra = false;
    for (int i = 0; i < NUM_MPU; i++) {
      if (!s[i].ok) continue;
      if (!temMostra) { mostra = s[i].tempC; temMostra = true; }
      if (!temAnt[i]) {
        tAnt[i] = s[i].tempC;
        temAnt[i] = true;
        sobe = true;
        continue;
      }
      if (s[i].tempC - tAnt[i] > 0.08f) sobe = true;
      tAnt[i] = s[i].tempC;
    }
    if (!sobe) {
      Serial.println(F("  temperatura estavel."));
      return;
    }
    Serial.printf("  %.1f C\n", mostra);
  }
  Serial.println(F("  seguindo — ainda esquentando um pouco."));
}

bool mediaPulsoGrav(float& g) {
  float acc = 0;
  int n = 0;
  float ref = 0;
  bool temRefA = false;
  unsigned long t0 = millis(), tUs = micros();
  while (millis() - t0 < 800) {
    unsigned long ag = micros();
    float dt = (ag - tUs) / 1000000.0f;
    tUs = ag;
    if (dt <= 0 || dt > 0.05f) dt = 0.02f;
    seguirSensores(dt);
    float a;
    if (pulsoGravidadeMao(a)) {
      if (!temRefA) { ref = a; temRefA = true; }
      acc += wrap180(a - ref);
      n++;
    }
    delay(5);
  }
  if (n < 8) return false;
  g = wrap180(ref + acc / n);
  return true;
}

// Palma para baixo e palma para cima, antebraco deitado. O que o sensor
// medir nesse giro vira 180° no motor, no mesmo sentido da mao.
void calibrarFaixaPulso() {
  if (!s[S_MAO].ok || !s[S_ANTE].ok || !s[S_MAO].temRef || !s[S_ANTE].temRef) {
    Serial.println(F("Pulso sem antebraco e mao — fica no giro do sensor, ganho 1."));
    temCalPulso = false;
    return;
  }
  Serial.println();
  Serial.println(F("=== PULSO — o robo copia o giro da palma ==="));
  Serial.println(F("Deixe o antebraco DEITADO, apontando para a frente."));
  Serial.println(F("p = pular estas duas poses."));

  while (true) {
    if (Serial.available()) {
      char c = Serial.read();
      if (c == 'p' || c == 'P') {
        temCalPulso = false;
        Serial.println(F("  pulso sem as duas poses."));
        return;
      }
    }
    Serial.println(F("POSE A — palma para BAIXO, paralela ao chao. Segure."));
    integrarPor(4000);
    float gBaixo;
    if (!mediaPulsoGrav(gBaixo)) {
      Serial.println(F("  antebraco muito em pe — deite ele e repita."));
      continue;
    }

    Serial.println(F("POSE B — GIRE a palma para CIMA agora. O antebraco continua deitado."));
    float giro = 0;
    unsigned long t0 = millis(), tUs = micros();
    while (millis() - t0 < 5000) {
      if (Serial.available()) {
        char c = Serial.read();
        if (c == 'p' || c == 'P') {
          temCalPulso = false;
          Serial.println(F("  pulso sem as duas poses."));
          return;
        }
      }
      unsigned long ag = micros();
      float dt = (ag - tUs) / 1000000.0f;
      tUs = ag;
      if (dt <= 0 || dt > 0.05f) dt = 0.02f;
      seguirSensores(dt);
      giro += taxaPulso() * dt;
      delay(5);
    }
    float gCima;
    if (!mediaPulsoGrav(gCima)) {
      Serial.println(F("  sem leitura da palma para cima — repita."));
      continue;
    }
    float delta = wrap180(gCima - gBaixo);
    Serial.printf("  sensor viu %.0f° entre palma baixo e palma cima\n", delta);
    if (fabsf(delta) < 50.0f) {
      Serial.println(F("  girou pouco — a palma precisa ir de baixo ate cima."));
      continue;
    }
    ganhoPulso = 180.0f / fabsf(delta);
    if (ganhoPulso > 3.0f) {
      Serial.println(F("  o sensor quase nao viu o giro — repita, so a palma."));
      continue;
    }
    pulsoGravZero = gBaixo;
    sinalGiroPulso = (fabsf(giro) < 25.0f || giro * delta >= 0.0f) ? 1.0f : -1.0f;
    temCalPulso = true;
    pulso = 0;
    Serial.printf("  palma baixo = 0 | palma cima = %.0f° | ganho %.2f\n",
                  (delta >= 0.0f ? 180.0f : -180.0f), ganhoPulso);
    break;
  }
}

void calibrar() {
  Serial.println();
  Serial.println(F("=== CALIBRACAO (7 poses) ==="));
  Serial.println(F("Inicio = todos os membros PARA CIMA, palma para TRAS."));
  Serial.println(F("O Arduino liga nessa mesma pose (motores em zero)."));

  for (int i = 0; i < NUM_MPU; i++) {
    if (!s[i].ok) s[i].ok = iniciarSensor(i);
    if (!s[i].ok) Serial.printf("  %s FALHOU — calibrando sem ele\n", NOME[i]);
    s[i].temFrente = false;
    s[i].temRef = false;
    s[i].quieto = false;
    s[i].temQuieto = false;
  }
  esperarAquecer();
  temRel = false;
  temAnteRel = false;
  temMaoRel = false;
  temEixoCot = false;
  bool temB = s[S_BRACO].ok, temM = s[S_MAO].ok;

  // ---- POSE 1: para cima -> eixo do braco + bias
  while (true) {
    tituloPose(1, F("TUDO PARA CIMA, PALMA PARA TRAS"),
               F("  braco, antebraco e mao PARA CIMA.\n"
                 "  Palma virada para TRAS. Cotovelo reto.\n"
                 "  Igual a posicao em que o Arduino liga."));
    esperarParado();
    bool ok = true;
    for (int i = 0; i < NUM_MPU; i++) {
      if (!s[i].ok) continue;
      if (!medirGravidade(i, s[i].g0, s[i].bias)) { ok = false; continue; }
      copiar3(s[i].eixo, s[i].g0);
      copiar3(s[i].g, s[i].g0);
      s[i].w[0] = s[i].w[1] = s[i].w[2] = 0;
      s[i].biasTemp = s[i].tempC;
      s[i].temBiasTemp = true;
      s[i].biasCoef[0] = s[i].biasCoef[1] = s[i].biasCoef[2] = 0;
      s[i].falhasSeguidas = 0;
      Serial.printf("  %-5s bias=[% .2f % .2f % .2f] °/s  %.1f C\n",
                    NOME[i], s[i].bias[0], s[i].bias[1], s[i].bias[2], s[i].tempC);
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
    // Os dois sensores passam a dividir o mesmo "mundo" antes de dobrar o cotovelo.
    for (int i = 0; i < NUM_MPU; i++) {
      if (!temH[i]) continue;
      copiar3(s[i].g, gH[i]);
      ancorarFrente(s[i]);
    }
    Serial.printf("  ganho ombro=%.2f\n", ganhoOmb);
    ganhoBase = 1.0f;
    float az0;
    if (azimute(s[S_BRACO], az0)) baseOffset = az0;
    else baseOffset = 0;
    Serial.println(F("  base 0 = esta frente"));
    break;
  }

  // ---- POSES 3 e 4: limites da base (±90). A frente NAO e' refeita.
  float azEsq = 90, azDir = -90;
  bool temEsq = false, temDir = false;
  while (temB) {
    tituloPose(3, F("BASE — 90 GRAUS A ESQUERDA"),
               F("  braco na HORIZONTAL, cotovelo reto.\n"
                 "  Gire o corpo/braco 90° para a SUA ESQUERDA (nao abaixe o braco)."));
    esperarParado();
    float az;
    if (!lerAzimuteBraco(az)) {
      Serial.println(F("  repetindo SO a POSE 3"));
      continue;
    }
    float omb = inclinacao(s[S_BRACO]) * ganhoOmb;
    if (fabsf(omb - 90.0f) > 35.0f) {
      Serial.printf("  braco saiu da horizontal (ombro %.0f°) — repetindo\n", omb);
      continue;
    }
    if (fabsf(az) < 50.0f || fabsf(az) > 130.0f) {
      Serial.printf("  giro de %.0f° (precisa ficar perto de 90) — repetindo\n", az);
      continue;
    }
    azEsq = az;
    temEsq = true;
    Serial.printf("  esquerda medida %.1f° -> vai valer +90\n", azEsq);
    break;
  }
  while (temB) {
    tituloPose(4, F("BASE — 90 GRAUS A DIREITA"),
               F("  braco na HORIZONTAL, cotovelo reto.\n"
                 "  Gire 90° para a SUA DIREITA, espelhando a pose anterior."));
    esperarParado();
    float az;
    if (!lerAzimuteBraco(az)) {
      Serial.println(F("  repetindo SO a POSE 4"));
      continue;
    }
    float omb = inclinacao(s[S_BRACO]) * ganhoOmb;
    if (fabsf(omb - 90.0f) > 35.0f) {
      Serial.printf("  braco saiu da horizontal (ombro %.0f°) — repetindo\n", omb);
      continue;
    }
    if (fabsf(az) < 50.0f || fabsf(az) > 130.0f) {
      Serial.printf("  giro de %.0f° (precisa ficar perto de 90) — repetindo\n", az);
      continue;
    }
    if (temEsq && az * azEsq > 0) {
      Serial.println(F("  este lado e' o mesmo da esquerda — vire para o outro"));
      continue;
    }
    azDir = az;
    temDir = true;
    Serial.printf("  direita medida %.1f° -> vai valer -90\n", azDir);
    break;
  }
  if (temEsq && temDir) {
    float gE = 90.0f / azEsq;
    float gD = -90.0f / azDir;
    ganhoBase = 0.5f * (gE + gD);
    Serial.printf("  ganho base=%.2f (esquerda +90, direita -90, limite ±150)\n", ganhoBase);
  } else {
    Serial.println(F("  base sem os dois lados — ganho fica 1.00, limite ainda e' ±150"));
  }

  // ---- POSE 5: cotovelo 90° (guarda o quadro; o ganho sai depois da pose 6)
  float g3[NUM_MPU][3], f3[NUM_MPU][3];
  bool tem3 = false, temF3[NUM_MPU] = { false, false };
  while (temB && temM) {
    tituloPose(5, F("ANTEBRACO A 90 GRAUS"),
               F("  VOLTE o braco para a FRENTE, na horizontal.\n"
                 "  Cotovelo em L: antebraco e mao APONTANDO PARA CIMA."));
    esperarParado();
    bool leuB = medirGravidade(S_BRACO, g3[S_BRACO], nullptr);
    bool leuM = medirGravidade(S_MAO, g3[S_MAO], nullptr);
    bool leuA = !s[S_ANTE].ok || medirGravidade(S_ANTE, g3[S_ANTE], nullptr);
    if (!leuB || !leuM || !leuA) {
      Serial.println(F("  leitura falhou — repetindo SO a POSE 5"));
      continue;
    }
    float incB3 = inclinacaoDe(g3[S_BRACO], s[S_BRACO].eixo, s[S_BRACO].f, s[S_BRACO].temFrente);
    if (fabsf(incB3 - 90.0f / ganhoOmb) > 30.0f) {
      Serial.printf("  braco saiu da horizontal (%.0f°) — repetindo SO a POSE 5\n", incB3);
      continue;
    }
    for (int i = 0; i < NUM_MPU; i++) {
      if (!s[i].ok || !s[i].temFrente) continue;
      copiar3(s[i].g, g3[i]);
      float c = dot3(s[i].f, g3[i]);
      for (int k = 0; k < 3; k++) f3[i][k] = s[i].f[k] - c * g3[i][k];
      temF3[i] = normalizar3(f3[i]);
      if (temF3[i]) copiar3(s[i].f, f3[i]);
    }
    tem3 = temF3[S_BRACO] && (temF3[S_ANTE] || temF3[S_MAO]);
    if (!tem3) Serial.println(F("  sem frente do mundo — o ganho do cotovelo fica 1.00"));
    break;
  }

  // ---- POSE 6: giro da palma acha o eixo do pulso. 40 amostras bastam.
  while (temM) {
    tituloPose(6, F("PULSO — GIRAR A PALMA"),
               F("  braco A FRENTE na horizontal, cotovelo RETO.\n"
                 "  Gire a palma para um lado e para o outro, sem parar.\n"
                 "  Segure o ombro. O antebraco pode acompanhar a palma."));
    Serial.println(F("  comeca em 3 s..."));
    integrarPor(3000);
    float eixo[3];
    if (!medirEixoPronacao(eixo)) {
      Serial.println(F("  repetindo SO a POSE 6"));
      continue;
    }
    copiar3(s[S_MAO].eixo, eixo);

    Serial.println(F("  agora PARE com a palma NEUTRA, braco a frente, cotovelo reto."));
    esperarParado();
    for (int i = 0; i < NUM_MPU; i++)
      if (s[i].ok) medirGravidade(i, s[i].g, nullptr);
    ancorarFrente(s[S_MAO]);
    if (s[S_ANTE].ok) ancorarFrente(s[S_ANTE]);
    if (temB) ancorarFrente(s[S_BRACO]);
    if (temB && s[S_BRACO].temFrente) {
      float azF;
      if (azimute(s[S_BRACO], azF)) baseOffset = azF;
    }
    break;
  }

  // Referencias com o braco na horizontal (no alto a gravidade fica no osso).
  cotZero = 0.0f;
  pulsoZero = 0.0f;
  ganhoCot = 1.0f;
  float cotPose5 = 0;
  bool temCot5 = false;
  if (temB && temM) {
    for (int i = 0; i < NUM_MPU; i++) {
      if (!s[i].ok) continue;
      if (!capturarRef(s[i]))
        Serial.printf("  %s sem referencia de cima (osso alinhado com a gravidade?)\n", NOME[i]);
    }
    if (tem3 && temF3[S_BRACO] && (temF3[S_ANTE] || temF3[S_MAO])) {
      int iCot = temF3[S_ANTE] ? S_ANTE : S_MAO;
      float ossoB[3], ossoC[3];
      bool okB3 = mundoDe(g3[S_BRACO], f3[S_BRACO], s[S_BRACO].eixo, ossoB);
      bool okC3 = mundoDe(g3[iCot], f3[iCot], s[iCot].eixo, ossoC);
      float direito[3] = { 0.0f, -1.0f, 0.0f };
      if (okB3 && okC3 && normalizar3(ossoB) && normalizar3(ossoC)) {
        cotPose5 = anguloEmTorno(direito, ossoB, ossoC);
        temCot5 = true;
      }
    }
  }

  // Palma baixo / palma cima antes da pose 7: a calibracao termina com o
  // braco para cima e o pulso em 0, igual ao Arduino. Terminar com a palma
  // para cima e gravar 0 fazia o pulso girar sozinho depois.
  if (temM) calibrarFaixaPulso();

  // ---- POSE 7: zero real = a pose em que o Arduino liga
  while (temB && temM) {
    tituloPose(7, F("VOLTAR AO INICIO"),
               F("  TODOS os membros PARA CIMA, palma virada para TRAS.\n"
                 "  E o zero do cotovelo e do pulso (motores parados)."));
    esperarParado();
    bool leu = true;
    for (int i = 0; i < NUM_MPU; i++) {
      if (!s[i].ok) continue;
      float g[3];
      if (!medirGravidade(i, g, nullptr)) { leu = false; continue; }
      copiar3(s[i].g, g);
      if (s[i].temFrente) {
        float c = dot3(s[i].f, s[i].g);
        for (int k = 0; k < 3; k++) s[i].f[k] -= c * s[i].g[k];
        if (!normalizar3(s[i].f)) s[i].temFrente = false;
      }
    }
    if (!leu) {
      Serial.println(F("  leitura falhou — repetindo SO a POSE 7"));
      continue;
    }
    float inc = inclinacao(s[S_BRACO]);
    if (fabsf(inc) > 35.0f) {
      Serial.printf("  braco nao esta para cima (%.0f°) — repetindo\n", inc);
      continue;
    }
    if (!iniciarRelativo()) {
      Serial.println(F("  sem referencial comum — repetindo SO a POSE 7"));
      continue;
    }
    float cotI = 0, twI = 0;
    if (!calcularAntebracoPulso(cotI, twI)) {
      Serial.println(F("  sem leitura das juntas — repetindo SO a POSE 7"));
      continue;
    }
    cotZero = cotI;
    pulsoZero = twI;
    Serial.printf("  inicio: cotovelo %.1f° = 0 | pulso %.1f° = 0\n", cotZero, pulsoZero);
    break;
  }
  if (temCot5) {
    float cot3 = cotPose5 - cotZero;
    Serial.printf("  cotovelo na pose 5 = %.1f° (esperado ~+90, antebraco para tras)\n", cot3);
    if (cot3 >= SPAN_MIN && cot3 <= SPAN_MAX) ganhoCot = 90.0f / cot3;
    else Serial.println(F("  fora da faixa — ganho do cotovelo fica 1.00 (tecla r refaz)"));
  }

  pulso = 0;
  zerarPrumoPulso();
  base = 0;
  ultimoUs = micros();
  Serial.println();
  Serial.println(F("========================================"));
  Serial.printf("ombro ganho %.2f | cotovelo zero %.1f° ganho %.2f | base ganho %.2f\n",
                ganhoOmb, cotZero, ganhoCot, ganhoBase);
  Serial.println(F("Zero = tudo para cima, palma para tras."));
  Serial.println(F("+ombro frente | +cotovelo tras | +pulso esquerda | +base esquerda"));
  if (temM) {
    Serial.printf("eixo pronacao da mao a %.0f° da direcao da mao (corrigido)\n",
                  anguloEntre(s[S_MAO].eixo, s[S_MAO].g0));
  }
  Serial.println(F("PRONTO — pode mexer. r = recalibrar | z = zerar pulso/base | h = home do boot"));
  Serial.println(F("Pulso em 0 agora. So gira quando a palma girar."));
  Serial.println(F("========================================"));
  bracoCalibrado = true;
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
  if (lerRegLoRa(0x06) != 0x64) return false;   // 400 MHz
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
  esperandoPong = false;
}

// Espera o pacote anterior (assincrono) sair. beginPacket falha enquanto o
// SX1278 ainda esta em TX.
bool comecarPacote(unsigned long esperaMs) {
  unsigned long t0 = millis();
  while (!LoRa.beginPacket()) {
    if (millis() - t0 > esperaMs) return false;
    delayMicroseconds(200);
  }
  return true;
}

// O pacote de 11 bytes leva ~20 ms no ar (SF7, BW 250k). Esperar o TX_DONE
// parava os sensores metade do tempo; agora o radio transmite sozinho.
bool enviarLoRa(int8_t c, int8_t o, int16_t p, int8_t b) {
  static unsigned long txInicioMs = 0;
  if (!LoRa.beginPacket()) {
    if (millis() - txInicioMs < LORA_TX_PRESO_MS) return true;
    LoRa.idle();
    falhasTx++;
    loraFalhasSeguidas++;
    return false;
  }
  uint16_t pulsoU = (uint16_t)p;
  LoRa.write(LORA_HEADER);
  LoRa.write((uint8_t)c);
  LoRa.write((uint8_t)o);
  LoRa.write((uint8_t)(pulsoU & 0xFF));
  LoRa.write((uint8_t)(pulsoU >> 8));
  LoRa.write((uint8_t)b);
  for (int i = 0; i < NUM_FLEX; i++) LoRa.write(angulosFlex[i]);
  LoRa.endPacket(true);
  txInicioMs = millis();
  pacotesTx++;
  loraFalhasSeguidas = 0;
  return true;
}

void declararQueda(const __FlashStringHelper* motivo) {
  bool ja = !loraConectado;
  loraConectado = false;
  linkMisses = 0;
  ultimoProbeMs = 0;
  if (!ja) Serial.println(F("!! LoRa desconectou — tentando reconectar"));
  reiniciarLoRa(motivo);
}

bool enviarCmdManual(uint8_t cmd) {
  if (!comecarPacote(LORA_TX_PRESO_MS)) return false;
  LoRa.write(LORA_CMD);
  LoRa.write(cmd);
  if (!LoRa.endPacket()) return false;
  LoRa.receive();
  return true;
}

bool teclaDoReceptor(char c) {
  switch (c) {
    case '1': case '2': case '3': case '4':
    case 'c': case 'C': case 'o': case 'O': case 'p': case 'P': case 'b': case 'B':
    case 'f': case 'F': case 't': case 'T': case 's': case 'S':
    case 'm': case 'M': case 'h': case 'H': case 'z': case 'Z':
    case '+': case '-':
      return true;
  }
  return false;
}

bool enviarPingLink(uint8_t seq) {
  if (!comecarPacote(LORA_TX_PRESO_MS)) return false;
  LoRa.write(LORA_PING);
  LoRa.write(seq);
  if (!LoRa.endPacket()) return false;
  LoRa.receive();   // escuta antes do receptor responder
  return true;
}

bool lerPongLink(uint8_t seq, int& rssiLuva) {
  int tam = LoRa.parsePacket();
  if (tam <= 0) return false;
  if (tam < 3) {
    while (LoRa.available()) LoRa.read();
    return false;
  }
  uint8_t h = LoRa.read();
  uint8_t sq = LoRa.read();
  (void)LoRa.read();
  while (LoRa.available()) LoRa.read();
  if (h != LORA_PONG || sq != seq) return false;
  rssiLuva = LoRa.packetRssi();
  return true;
}

void onLinkUp(int rssi) {
  esperandoPong = false;
  linkMisses = 0;
  ultimoProbeMs = millis();
  if (!loraConectado) {
    loraConectado = true;
    Serial.printf("!! LoRa RECONECTADO (rssi %d dBm)\n", rssi);
  }
}

void onLinkMiss() {
  esperandoPong = false;
  linkMisses++;
  if (loraConectado) {
    if (linkMisses >= LINK_MISSES_MAX) {
      // Nao reseta na hora: o chip pode estar sao e so o PONG se perdeu.
      // Os PINGs seguintes reconectam; o reset entra se eles tambem falharem.
      loraConectado = false;
      linkMisses = 0;
      ultimoProbeMs = 0;
      Serial.println(F("!! LoRa desconectou — tentando reconectar"));
    }
    return;
  }
  if (linkMisses >= 4) {
    linkMisses = 0;
    reiniciarLoRa(F("reconexao"));
  }
}

// Manda a pose o tempo todo. De vez em quando pede PONG. Sem resposta,
// reseta o SX1278 e continua tentando ate o receptor voltar.
void cuidarLinkLoRa(int8_t c, int8_t o, int16_t p, int8_t b) {
  unsigned long agora = millis();

  if (!esperandoPong) {
    static unsigned long ultimoCheckMs = 0;
    if (agora - ultimoCheckMs >= 1000) {
      ultimoCheckMs = agora;
      if (!loraConfigOk()) declararQueda(F("perdeu config"));
    }
  }

  if (esperandoPong) {
    int rssi = 0;
    if (lerPongLink(probeSeq, rssi)) onLinkUp(rssi);
    else if ((long)(agora - probeDeadline) >= 0) onLinkMiss();
    else return;
  }

  unsigned long intervalo = loraConectado ? LINK_PROBE_MS : LINK_RETRY_MS;
  if (agora - ultimoProbeMs >= intervalo) {
    ultimoProbeMs = agora;
    probeSeq++;
    if (!enviarPingLink(probeSeq)) {
      onLinkMiss();
      return;
    }
    esperandoPong = true;
    probeDeadline = millis() + (loraConectado ? LINK_PONG_MS : LINK_PONG_DOWN_MS);
    return;
  }

  if ((bracoCalibrado || manualRemoto || emHome) && agora - ultimoTxMs >= TX_PERIODO_MS) {
    ultimoTxMs = agora;
    if (!enviarLoRa(c, o, p, b) && loraFalhasSeguidas >= 3) {
      loraFalhasSeguidas = 0;
      declararQueda(F("travou"));
    }
  }

  if (!loraConectado) {
    static unsigned long ultimoAvisoMs = 0;
    if (agora - ultimoAvisoMs >= 1000) {
      ultimoAvisoMs = agora;
      Serial.println(F("!! LoRa reconectando..."));
    }
  }
}

int8_t paraByte(int j, float graus) {
  graus *= GANHO[j];
  if (INV_TX[j]) graus = -graus;
  float fundo = (j == 0) ? FUNDO_COT : (j == 1) ? FUNDO_OMB : FUNDO_BASE;
  graus = constrain(graus, -fundo, fundo);
  float fio = graus * FIO_CHEIO / fundo;
  return (int8_t)(fio + (fio >= 0 ? 0.5f : -0.5f));
}

// Pulso passa de ±127, entao vai em int16. 0 = pose da partida; tras = negativo.
int16_t paraPulso(float graus) {
  graus *= GANHO[2];
  if (INV_TX[2]) graus = -graus;
  graus = constrain(graus, LIM_PUL_TRAS, LIM_PUL_FRENTE);
  return (int16_t)(graus + (graus >= 0 ? 0.5f : -0.5f));
}

// ---------------------------------------------------------------- flex
// Mesma leitura e decisao do leitura-dedos (so aberto/fechado).
int mediaDoMioloFlex(int ch) {
  int v[FLEX_BURST];
  for (int i = 0; i < FLEX_DESCARTA; i++) analogRead(PIN_FLEX[ch]);
  for (int i = 0; i < FLEX_BURST; i++) {
    v[i] = analogRead(PIN_FLEX[ch]);
    delayMicroseconds(FLEX_GAP_US);
  }
  for (int i = 1; i < FLEX_BURST; i++) {
    int key = v[i];
    int j = i - 1;
    while (j >= 0 && v[j] > key) {
      v[j + 1] = v[j];
      j--;
    }
    v[j + 1] = key;
  }
  const int a = FLEX_BURST / 4;
  const int b = FLEX_BURST - FLEX_BURST / 4;
  long soma = 0;
  for (int i = a; i < b; i++) soma += v[i];
  return (int)(soma / (b - a));
}

int medianaFlex(int ch) {
  int tmp[FLEX_MED_N];
  int n = flexMedCheio[ch] ? FLEX_MED_N : flexMedIdx[ch];
  if (n < 1) n = 1;
  for (int i = 0; i < n; i++) tmp[i] = flexMedBuf[ch][i];
  for (int i = 1; i < n; i++) {
    int key = tmp[i];
    int j = i - 1;
    while (j >= 0 && tmp[j] > key) {
      tmp[j + 1] = tmp[j];
      j--;
    }
    tmp[j + 1] = key;
  }
  return tmp[n / 2];
}

void empurraMedFlex(int ch, int v) {
  flexMedBuf[ch][flexMedIdx[ch]] = v;
  flexMedIdx[ch]++;
  if (flexMedIdx[ch] >= FLEX_MED_N) {
    flexMedIdx[ch] = 0;
    flexMedCheio[ch] = 1;
  }
}

float alphaLpFlex(float cutoff, float dt) {
  float tau = 1.0f / (2.0f * PI * cutoff);
  return 1.0f / (1.0f + tau / dt);
}

void filtrarFlex(int ch) {
  unsigned long agora = micros();
  float dt = flexOk[ch] ? (agora - flexTAnt[ch]) * 1e-6f : 0.04f;
  if (dt <= 0.0f || dt > 0.5f) dt = 0.04f;
  flexTAnt[ch] = agora;

  int limpo = mediaDoMioloFlex(ch);
  if (flexOk[ch] && abs(limpo - (int)flexX[ch]) > FLEX_PICO &&
      flexRejeitados[ch] < FLEX_PICO_SEGUIDOS) {
    flexRejeitados[ch]++;
  } else {
    flexRejeitados[ch] = 0;
    empurraMedFlex(ch, limpo);
  }
  float med = medianaFlex(ch);

  if (!flexOk[ch]) {
    flexX[ch] = med;
    flexDx[ch] = 0;
    flexOk[ch] = true;
    return;
  }

  float dx = (med - flexX[ch]) / dt;
  flexDx[ch] += alphaLpFlex(FLEX_D_CUTOFF, dt) * (dx - flexDx[ch]);
  float cutoff = FLEX_MIN_CUTOFF + FLEX_BETA * fabsf(flexDx[ch]);
  flexX[ch] += alphaLpFlex(cutoff, dt) * (med - flexX[ch]);
}

void filtrarTodosFlex() {
  for (int ch = 0; ch < FLEX_ATIVOS; ch++) filtrarFlex(ch);
}

void contagemFlex(const __FlashStringHelper* pose, int segundos) {
  Serial.print(pose);
  Serial.print(F(" em "));
  for (int t = segundos; t > 0; t--) {
    Serial.print(t);
    Serial.print(' ');
    unsigned long t0 = millis();
    while (millis() - t0 < 1000) filtrarTodosFlex();
  }
  Serial.println();
}

void medirPoseFlex(float media[], float desvio[]) {
  double soma[NUM_FLEX] = { 0 };
  double soma2[NUM_FLEX] = { 0 };
  long n = 0;
  unsigned long t0 = millis();
  while (millis() - t0 < FLEX_MEDE_MS) {
    filtrarTodosFlex();
    for (int ch = 0; ch < FLEX_ATIVOS; ch++) {
      soma[ch] += flexX[ch];
      soma2[ch] += (double)flexX[ch] * flexX[ch];
    }
    n++;
  }
  for (int ch = 0; ch < FLEX_ATIVOS; ch++) {
    if (n < 2) {
      media[ch] = flexX[ch];
      desvio[ch] = 0;
      continue;
    }
    double m = soma[ch] / n;
    double var = soma2[ch] / n - m * m;
    media[ch] = m;
    desvio[ch] = var > 0 ? sqrt(var) : 0;
  }
  Serial.println(F("  ok, medido."));
}

void zerarFiltroFlex() {
  for (int ch = 0; ch < NUM_FLEX; ch++) {
    flexOk[ch] = false;
    flexMedIdx[ch] = 0;
    flexMedCheio[ch] = 0;
    flexRejeitados[ch] = 0;
  }
}

int aplicarCalibracaoFlex(const float ab1[], const float fe[], const float ab2[],
                          const float dAb1[], const float dFe[], const float dAb2[]) {
  Serial.println(F("Dedo       aberto fechado dobra ruido  abre fecha"));
  int bons = 0;
  for (int ch = 0; ch < FLEX_ATIVOS; ch++) {
    float dobra = ab2[ch] - fe[ch];
    flexSinal[ch] = dobra >= 0 ? 1.0f : -1.0f;
    flexSpan[ch] = fabsf(dobra);
    flexAberto[ch] = ab2[ch];
    flexValido[ch] = flexSpan[ch] >= FLEX_SPAN_MIN;

    float ruido = max(dAb2[ch], max(dAb1[ch], dFe[ch]));
    float fa = FLEX_FRAC_ABRE;
    if (flexValido[ch] && FLEX_SIGMAS * ruido / flexSpan[ch] > fa)
      fa = FLEX_SIGMAS * ruido / flexSpan[ch];
    float ff = max(FLEX_FRAC_FECHA, fa + FLEX_FAIXA_MIN);
    bool apertado = ff > FLEX_FRAC_FECHA_MAX;
    if (apertado) {
      ff = FLEX_FRAC_FECHA_MAX;
      fa = min(fa, ff - FLEX_FAIXA_MIN);
    }
    flexFracAbre[ch] = fa;
    flexFracFecha[ch] = ff;

    const char* aviso = "";
    if (!flexValido[ch]) aviso = "<- NAO MUDOU, fica aberto";
    else if (apertado) aviso = "<- ruido alto p/ a dobra, pode oscilar";
    else bons++;
    Serial.printf("%-9s  %6d  %6d  %4d  %4.1f  %3d%%  %3d%% %s\n", NOME_DEDO[ch],
                  (int)ab2[ch], (int)fe[ch], (int)dobra, ruido,
                  (int)(fa * 100), (int)(ff * 100), aviso);
  }
  return bons;
}

void calibrarFlex() {
  float ab1[NUM_FLEX], fe[NUM_FLEX], ab2[NUM_FLEX];
  float dAb1[NUM_FLEX], dFe[NUM_FLEX], dAb2[NUM_FLEX];

  Serial.println();
  Serial.println(F("=== CALIBRACAO FLEX (mao inteira) ==="));
  Serial.println(F("Punho normal, sem esmagar. O flex continua caindo se ficar"));
  Serial.println(F("fechado por muito tempo, e esse valor o dedo nunca alcanca depois."));
  Serial.println(F("Mindinho acompanha o anelar."));

  int bons = 0;
  for (int tentativa = 1; tentativa <= FLEX_TENTATIVAS; tentativa++) {
    if (tentativa > 1) {
      Serial.println();
      Serial.println(F("A dobra ficou fraca. Repetindo os dedos."));
      Serial.println(F("Abra de verdade, depois feche todos no punho, depois abra de novo."));
    }
    zerarFiltroFlex();
    contagemFlex(F("MAO ABERTA, esticada e parada."), FLEX_CONTAGEM_S);
    medirPoseFlex(ab1, dAb1);
    contagemFlex(F("FECHE num punho normal e segure."), FLEX_FECHA_S);
    medirPoseFlex(fe, dFe);
    // Depois de dobrar o flex nao volta na hora; este aberto e' o que vale.
    contagemFlex(F("ABRA a mao de novo e segure."), FLEX_TROCA_S);
    medirPoseFlex(ab2, dAb2);

    bons = aplicarCalibracaoFlex(ab1, fe, ab2, dAb1, dFe, dAb2);
    Serial.printf("%d de %d dedos com dobra boa.\n", bons, FLEX_ATIVOS);
    if (bons >= 3) break;
  }
  if (bons < 3)
    Serial.println(F("Poucos dedos responderam. f repete so os dedos."));
  else
    Serial.println(F("=== FLEX OK ==="));
}

void iniciarFiltrosFlex() {
  for (int ch = 0; ch < NUM_FLEX; ch++) {
    flexFechado[ch] = false;
    flexCondDesde[ch] = millis();
    angulosFlex[ch] = FLEX_GRAU_ABERTO;
  }
  flexProximo = 0;
}

float fracDobraFlex(int ch) {
  return flexSinal[ch] * (flexAberto[ch] - flexX[ch]) / flexSpan[ch];
}

void decidirFlex(int ch) {
  if (!flexValido[ch]) {
    flexFechado[ch] = false;
    return;
  }
  unsigned long ms = millis();
  float f = fracDobraFlex(ch);
  // Schmitt: punho fecha, mao aberta abre. Entre os dois, nao muda.
  bool querFechado = flexFechado[ch] ? (f >= flexFracAbre[ch]) : (f > flexFracFecha[ch]);
  bool querTrocar = querFechado != flexFechado[ch];
  if (!querTrocar) {
    flexCondDesde[ch] = ms;
    return;
  }
  if (ms - flexCondDesde[ch] < FLEX_CONFIRMA_MS) return;
  flexCondDesde[ch] = ms;
  flexFechado[ch] = !flexFechado[ch];
}

// Um dedo por volta do loop: o burst leva ~7 ms e o braco nao pode esperar os 4.
void processarFlexProximo() {
  int ch = flexProximo;
  flexProximo = (flexProximo + 1) % FLEX_ATIVOS;
  filtrarFlex(ch);
  decidirFlex(ch);
  angulosFlex[ch] = flexFechado[ch] ? FLEX_GRAU_FECHADO : FLEX_GRAU_ABERTO;
  angulosFlex[NUM_FLEX - 1] = angulosFlex[FLEX_ATIVOS - 1];
}

// ---------------------------------------------------------------- teste LoRa
bool pingLoRa(uint8_t seq, int& rssiLuva, int& rssiArduino) {
  LoRa.beginPacket();
  LoRa.write(LORA_PING);
  LoRa.write(seq);
  if (!LoRa.endPacket()) return false;

  unsigned long t0 = millis();
  while (millis() - t0 < PING_ESPERA_MS) {
    int tam = LoRa.parsePacket();
    if (tam >= 3) {
      uint8_t h = LoRa.read();
      uint8_t sq = LoRa.read();
      int8_t r = (int8_t)LoRa.read();
      while (LoRa.available()) LoRa.read();
      if (h == LORA_PONG && sq == seq) {
        rssiLuva = LoRa.packetRssi();
        rssiArduino = r;
        return true;
      }
    } else if (tam > 0) {
      while (LoRa.available()) LoRa.read();
    }
    delay(2);
  }
  return false;
}

// Precisa do kairos-receptor ligado (ele responde PONG). Sem Serial aberto,
// repete sozinho a cada 5 s ate conectar.
void testarConexaoLoRa() {
  Serial.println();
  Serial.println(F("=== TESTE DE CONEXAO LoRa (antes da calibracao) ==="));
  Serial.println(F("  Ligue o Arduino receptor. A luva manda PING e espera PONG."));
  uint8_t seq = 0;
  while (true) {
    int ok = 0;
    long somaLuva = 0, somaArduino = 0;
    for (int k = 0; k < PING_POR_RODADA; k++) {
      int rLuva = 0, rArduino = 0;
      seq++;
      if (pingLoRa(seq, rLuva, rArduino)) {
        ok++;
        somaLuva += rLuva;
        somaArduino += rArduino;
        Serial.printf("  ping %3u  PONG  rssi luva %4d dBm | rssi arduino %4d dBm\n",
                      seq, rLuva, rArduino);
      } else {
        Serial.printf("  ping %3u  sem resposta\n", seq);
      }
      delay(100);
    }

    Serial.printf("  resultado: %d/%d respostas", ok, PING_POR_RODADA);
    if (ok > 0) Serial.printf("  (media: luva %ld dBm, arduino %ld dBm)", somaLuva / ok, somaArduino / ok);
    Serial.println();

    if (ok >= PING_POR_RODADA / 2) {
      Serial.println(F("=== LoRa CONECTADO — seguindo para a calibracao ==="));
      return;
    }

    Serial.println(ok > 0 ? F("  Conexao FRACA (muitos pacotes perdidos).")
                          : F("  SEM CONEXAO com o Arduino."));
    Serial.println(F("  Confira: receptor ligado com o kairos-receptor, antenas, mesma BW/SF."));
    Serial.println(F("  ENTER = testar de novo | p = pular e calibrar mesmo assim"));
    while (Serial.available()) Serial.read();
    unsigned long t0 = millis();
    while (millis() - t0 < 5000 && !Serial.available()) delay(20);
    if (Serial.available()) {
      char c = Serial.read();
      while (Serial.available()) Serial.read();
      if (c == 'p' || c == 'P') {
        Serial.println(F("  Pulando teste LoRa."));
        return;
      }
    }
    Serial.println(F("  Testando de novo..."));
  }
}

// ---------------------------------------------------------------- setup / loop
void setup() {
  // Sem buffer, a linha de status (~150 bytes) trava o loop ~13 ms esperando a UART.
  Serial.setTxBufferSize(1024);
  Serial.begin(115200);
  delay(300);
  WiFi.mode(WIFI_OFF);
  btStop();

  Serial.println();
  Serial.println(F("=== KAIROS luva-tx — 2 MPU + flex + LoRa (0xC2) ==="));
  Serial.println(F("COMO COMECAR"));
  Serial.println(F("1. Agora so teste: MPU OK/FALHOU e ping/pong."));
  Serial.println(F("2. Nao calibra e nao entra no manual sozinho."));
  Serial.println(F("3. r = calibracao completa (dedos, poses, palma baixo/cima, tudo para cima)."));
  Serial.println(F("   n = manual. Depois: c cotovelo, t para frente, f para tras, s para, n sai."));
  Serial.println(F("   h = pose do boot do Arduino. z = zerar pulso/base (nao mexe no home)."));
  Serial.println(F("   Pulso: parado fica parado; so gira quando a palma gira."));

  analogReadResolution(12);
  for (int ch = 0; ch < NUM_FLEX; ch++) {
    analogSetPinAttenuation(PIN_FLEX[ch], ADC_11db);
    flexOk[ch] = false;
    flexRejeitados[ch] = 0;
    flexMedIdx[ch] = 0;
    flexMedCheio[ch] = 0;
    flexValido[ch] = false;
    flexSinal[ch] = 1.0f;
    flexSpan[ch] = FLEX_SPAN_MIN;
    flexFechado[ch] = false;
    angulosFlex[ch] = FLEX_GRAU_ABERTO;
  }

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
  Serial.println(F("LoRa TX OK (400 MHz SF7 BW250)"));

  testarConexaoLoRa();
  iniciarFiltrosFlex();
  ultimoUs = micros();
  ultimoProbeMs = millis();
  loraConectado = true;
  Serial.println(F("Testes prontos. A luva ainda nao comanda o braco."));
  Serial.println(F("r = calibracao completa | n = manual | h = pose do boot"));
}

// Depois de uma pausa longa (calibracao flex) a frente do mundo ficou velha.
void reancorarFrente() {
  Serial.println();
  Serial.println(F("Reancorando a frente: braco ESTICADO A FRENTE na horizontal."));
  esperarParado();
  for (int i = 0; i < NUM_MPU; i++) {
    if (!s[i].ok) continue;
    if (medirGravidade(i, s[i].g, nullptr)) ancorarFrente(s[i]);
  }
  iniciarRelativo();
  float cotBruto, tw;
  if (calcularAntebracoPulso(cotBruto, tw)) {
    cotZero = cotBruto;
    pulsoZero = tw;
  }
  pulso = 0;
  zerarPrumoPulso();
  base = 0;
  baseOffset = 0;
  ultimoUs = micros();
  Serial.println(F("Frente OK — pode mexer."));
}

void calibrarCompleta() {
  if (manualRemoto) {
    manualRemoto = false;
    enviarCmdManual(CMD_SAI);
  }
  if (emHome) {
    emHome = false;
    enviarCmdManual(CMD_HOME_OFF);
  }
  Serial.println(F("Calibracao completa: dedos, poses do braco, palma, e por fim tudo para cima."));
  calibrarFlex();
  iniciarFiltrosFlex();
  calibrar();
}

// O giro da palma entra pelo gyro da mao, so quando ela gira de verdade.
// Parado, nada integra: o pulso fica onde estava ate voce mexer.
// A gravidade so tira um resto pequeno de drift, com a mao parada e o
// antebraco deitado. Puxar para um angulo longe fazia o robo girar sozinho.
void atualizarPulso(float dt) {
  Sensor& M = s[S_MAO];
  float taxa = taxaPulso();
  if (!M.quieto && fabsf(taxa) > ZONA_GIRO_PULSO)
    pulso += taxa * ganhoPulso * sinalGiroPulso * dt;

  float gRel;
  bool anteDeitado = !s[S_ANTE].ok ||
                     fabsf(inclinacao(s[S_ANTE]) - 90.0f) < 35.0f;
  if (temCalPulso && M.quieto && anteDeitado && pulsoGravidadeMao(gRel)) {
    float alvo = ganhoPulso * wrap180(gRel - pulsoGravZero);
    float err = wrap180(alvo - pulso);
    if (fabsf(err) < 20.0f) {
      float k = 0.4f * dt;
      if (k > 0.02f) k = 0.02f;
      pulso += err * k;
    }
  }
  if (pulso > LIM_PUL_FRENTE) pulso = LIM_PUL_FRENTE;
  if (pulso < LIM_PUL_TRAS) pulso = LIM_PUL_TRAS;
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'n' || c == 'N') {
      manualRemoto = !manualRemoto;
      emHome = false;
      bool ok = enviarCmdManual(manualRemoto ? CMD_ENTRA : CMD_SAI);
      if (!ok) Serial.println(F("  LoRa nao enviou o modo."));
      Serial.println(manualRemoto
        ? F("Manual. 1/c cotovelo  2/o ombro  3/p pulso  4/b base | f frente  t tras | s para | h home | z zero | m teste | +/- vel | n sai")
        : F("Modo luva — sensores no comando."));
    } else if (manualRemoto && teclaDoReceptor(c)) {
      if (!enviarCmdManual((uint8_t)c)) Serial.println(F("  comando nao enviado"));
      else { Serial.print(F("  -> ")); Serial.println(c); }
    } else if (c == 'r' || c == 'R') calibrarCompleta();
    else if (!manualRemoto && (c == 'f' || c == 'F')) {
      calibrarFlex();
      iniciarFiltrosFlex();
      reancorarFrente();
    } else if (!manualRemoto && (c == 'z' || c == 'Z')) {
      float gRel;
      if (pulsoGravidadeMao(gRel)) pulsoGravZero = gRel;
      pulso = 0;
      zerarPrumoPulso();
      float az;
      if (azimute(s[S_BRACO], az)) baseOffset = az;
      base = 0;
      Serial.println(F("  pulso e base zerados (base = direcao atual do braco)"));
    } else if (!manualRemoto && (c == 'h' || c == 'H')) {
      emHome = !emHome;
      if (!enviarCmdManual(emHome ? CMD_HOME_ON : CMD_HOME_OFF))
        Serial.println(F("  LoRa nao enviou o home — o pacote ainda vai zerado."));
      Serial.println(emHome
        ? F("Home — braco no ponto em que o Arduino ligou. h de novo segue a luva.")
        : F("Seguindo a luva."));
    }
  }

  unsigned long agora = micros();
  float dt = (agora - ultimoUs) / 1000000.0f;
  ultimoUs = agora;
  if (dt <= 0) dt = 0.001f;
  if (dt > 0.05f) dt = 0.05f;
  static uint32_t voltas = 0;
  voltas++;

  processarFlexProximo();

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
    marcarQuieto(s[i], a, gy, agora);
    if (s[i].quieto) puxarBias(s[i], gy);
    for (int k = 0; k < 3; k++) s[i].w[k] = gyroSemBias(s[i], k, gy[k]);
    atualizarGravidade(s[i], a, dt);
    leuOk[i] = true;
  }

  if (leuOk[S_BRACO] && leuOk[S_ANTE]) integrarRelativo(dt);
  else if (leuOk[S_BRACO] && leuOk[S_MAO] && !temAnteRel) integrarRelativo(dt);
  if (leuOk[S_ANTE] && leuOk[S_MAO] && temMaoRel) {
    float dtPul = dt > 0.05f ? 0.05f : dt;
    float trava[3];
    copiar3(trava, s[S_ANTE].eixo);
    if (!normalizar3(trava)) { trava[0] = 0; trava[1] = 0; trava[2] = 1; }
    integrarUm(Rmf, s[S_ANTE], s[S_MAO], trava, dtPul);
  }

  Sensor& B = s[S_BRACO];
  Sensor& M = s[S_MAO];

  // Junta so atualiza com os sensores dela lidos; senao congela no ultimo valor.
  if (leuOk[S_BRACO]) {
    float incB = inclinacao(B);
    jOmb = constrain(incB * ganhoOmb, LIM_OMB_MIN, LIM_OMB_MAX);

    // Braco quase vertical nao tem rumo: a base segura o ultimo valor.
    float az;
    if (azimute(B, az)) {
      float b = wrap180(az - baseOffset) * ganhoBase;
      base = constrain(b, -LIM_BASE, LIM_BASE);
    }

    if (leuOk[S_MAO] || leuOk[S_ANTE]) {
      float cotBruto, tw;
      if (calcularAntebracoPulso(cotBruto, tw)) {
        bool temTw = temMaoRel ? (s[S_ANTE].temRef && M.temRef) : (B.temRef && M.temRef);
        puxarZerosInicio(cotBruto, tw, dt);
        jCot = constrain((cotBruto - cotZero) * ganhoCot, LIM_COT_MIN, LIM_COT_MAX);
        (void)temTw;
      }
    }
  }

  if (leuOk[S_MAO]) atualizarPulso(dt);

  int8_t c = paraByte(0, jCot);
  int8_t o = paraByte(1, jOmb);
  int16_t p = paraPulso(pulso);
  int8_t b = paraByte(3, base);
  if (emHome || !bracoCalibrado) {
    c = 0;
    o = 0;
    p = 0;
    b = 0;
  }
  if (emHome) {
    for (int i = 0; i < NUM_FLEX; i++) angulosFlex[i] = FLEX_GRAU_ABERTO;
  }
  cuidarLinkLoRa(c, o, p, b);

  static unsigned long ultimoCasaMs = 0;
  if (millis() - ultimoCasaMs >= TX_PERIODO_MS) {
    ultimoCasaMs = millis();

    // Recupera sensor morto 1 por vez (init trava ~50 ms).
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
        s[i].quieto = false;
        s[i].temQuieto = false;
        Serial.printf("  !! %s voltou (tecla r recalibra a frente)\n", NOME[i]);
      }
      break;
    }

    static uint8_t div = 0;
    static unsigned long ultimoStatusMs = 0;
    if (++div >= 5) {
      div = 0;
      unsigned long janelaMs = millis() - ultimoStatusMs;
      ultimoStatusMs = millis();
      unsigned long hz = janelaMs ? (1000UL * voltas) / janelaMs : 0;
      voltas = 0;
      Serial.printf("C=%4d O=%4d P=%4d B=%4d | D=%d %d %d %d %d | %luHz | Braco %s %3lu%% | Ante %s %3lu%% | Mao %s %3lu%% | tx=%lu falha=%lu | %s\n",
                    c, o, p, b,
                    angulosFlex[0], angulosFlex[1], angulosFlex[2], angulosFlex[3], angulosFlex[4],
                    hz,
                    B.ok ? "ok" : "--", B.leituras ? 100UL * (B.leituras - B.falhas) / B.leituras : 0UL,
                    s[S_ANTE].ok ? "ok" : "--",
                    s[S_ANTE].leituras ? 100UL * (s[S_ANTE].leituras - s[S_ANTE].falhas) / s[S_ANTE].leituras : 0UL,
                    M.ok ? "ok" : "--", M.leituras ? 100UL * (M.leituras - M.falhas) / M.leituras : 0UL,
                    (unsigned long)pacotesTx, (unsigned long)falhasTx,
                    emHome ? "HOME" : (loraConectado ? "link ok" : "RECONECTANDO"));
      for (int i = 0; i < NUM_MPU; i++) { s[i].leituras = 0; s[i].falhas = 0; }
    }
  }
}
