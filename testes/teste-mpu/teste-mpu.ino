/*
 * Kairós — teste-mpu v3 (ESP32 TX)  —  cinemática esférica do braço
 * ============================================================================
 *
 * MÉTODO (baseado na literatura de captura de movimento com IMU):
 *
 *  1) Cada MPU roda um AHRS Mahony 6DOF (accel + gyro) -> quaternion.
 *     Mahony e' recomendado para MCU (barato, convergencia rapida).
 *     Sem magnetometro o YAW deriva -> compensado com bias + ZUPT.
 *
 *  2) Calibracao sensor->segmento SEM supor como o MPU esta colado:
 *     na pose ZERO (braco esticado PARA CIMA) o acelerometro de cada MPU
 *     aponta exatamente ao longo do segmento. Guardamos esse vetor no
 *     referencial do sensor (eixoBraco_s, eixoAnte_s, eixoMao_s).
 *     Isso elimina o erro de montagem, que e' a maior fonte de erro
 *     em estimativa de angulo de ombro com IMU.
 *
 *  3) DESACOPLAMENTO (a causa do movimento "estranho" era misturar o
 *     rumo 6D dos 3 MPUs). Sem magnetometro o yaw de cada Mahony e'
 *     INOBSERVAVEL e cada sensor inventa um norte proprio. Angulo entre
 *     vetores de dois quaternions vira lixo e uma junta puxa a outra.
 *
 *     Cada junta usa SO as grandezas que ela realmente observa:
 *
 *       OMBRO + BASE  — so o MPU do BRACO, em esferico:
 *         ombro = inclinacao do eixo contra a gravidade (gF)
 *         base  = azimute do mesmo eixo no mundo DESSE sensor
 *                 (Mahony do braco, zero = frente do passo 2)
 *         Os outros dois MPUs so entram na base quando os 3 giros
 *         em torno da vertical CONCORDAM (voce girou o tronco).
 *         Se discordam, so o braco manda — articular nao vaza.
 *
 *       COTOVELO — Seel 2014, braco + antebraco, SEM quaternion:
 *         eixo da dobradica identificado no frame de CADA sensor
 *         (giro durante o passo 3 + eixo x gravidade)
 *         angulo = pitch da gravidade no plano da dobradica
 *         velocidade = ω_ante·j_ante − ω_braco·j_braco
 *         filtro complementar (eq. 15 do paper)
 *
 *       PULSO — PRONACAO (gira a palma), antebraco + mao.
 *         O motor do robo gira a palma, entao medimos torcao em torno
 *         do eixo do antebraco: ω_mao·eixo − ω_ante·eixo.
 *         Com antebraco inclinado, a gravidade no plano transversal
 *         corrige a deriva (dois "relogios" de baixo calibrados no zero).
 *         Flexao do pulso (tchauzinho) NAO tem motor — e' ignorada.
 *
 *     Abaixar o braco esticado NAO mexe o cotovelo.
 *     Girar o tronco NAO mexe o ombro.
 *     Abrir o braco para o LADO mexe a BASE (esferico: yaw+pitch).
 *
 * ============================================================================
 * POSE ZERO: braco ESTICADO PARA CIMA, cotovelo RETO, parado ~3 s.
 * Tecla 'r' no Serial 115200 recalibra.
 *
 * LIGACOES (AD0 -> GND, VCC -> 3V3 em todos)
 *   Mao:       SDA 27  SCL 14
 *   Antebraco: SDA 25  SCL 26
 *   Braco:     SDA 4   SCL 13
 *   LoRa:      NSS 5  RST 2  DIO0 15  SCK 18  MISO 19  MOSI 23
 *
 * Pacote LoRa: [0xC1, cotovelo, ombro, pulso, base]  int8 graus
 * ============================================================================
 */

#include <WiFi.h>
#include <SPI.h>
#include <LoRa.h>
#include <math.h>

// ---------------------------------------------------------------- hardware
const uint8_t MPU_ADDR = 0x68;
const int NUM_MPU = 3;
const int SOFT_DELAY_US = 5;

const int PIN_SDA[NUM_MPU] = { 27, 25, 4 };
const int PIN_SCL[NUM_MPU] = { 14, 26, 13 };
const char* NOME_MPU[NUM_MPU] = { "Mao", "Antebraco", "Braco" };

#define IDX_MAO   0
#define IDX_ANTE  1
#define IDX_BRACO 2

#define LORA_SS   5
#define LORA_RST  2
#define LORA_DIO0 15
const long LORA_FREQ = 433E6;
const uint8_t LORA_HEADER = 0xC1;

enum { J_COTOVELO = 0, J_OMBRO = 1, J_PULSO = 2, J_BASE = 3, NUM_JUNTAS = 4 };

// ---------------------------------------------------------------- ajustes
// Inverte o sentido de uma junta:      C      O      P      B
const bool  INV_TX[NUM_JUNTAS]  = { false, false, false, false };
// Ganho humano -> robo (1.0 = copia 1:1; 0.5 = metade do angulo)
const float GANHO[NUM_JUNTAS]   = {  1.0f,  1.0f,  1.0f,  1.0f };

// ---------------------------------------------------------------------------
// MODO DE MAPEAMENTO — mude aqui e recompile para comparar os dois
//
//  0 = ESFERICO (azimute + elevacao). Reproduz a POSE exata do seu braco.
//      Como o robo e' yaw(base) + pitch(ombro), essa e' a unica decomposicao
//      que faz o braco robotico apontar para onde o seu aponta.
//      Efeito colateral: braco para o LADO exige base 90 + ombro 90, entao
//      abrir o braco lateralmente gira bastante a base. Isso e' da mecanica.
//
//  1 = PLANAR (desacoplado). Frente/tras move SO o ombro, lado move SO a base.
//      Cada eixo responde a um movimento so, que e' o mais intuitivo de pilotar,
//      mas nao reproduz a pose exata em movimentos diagonais.
//      Nao tem singularidade na vertical.
// ---------------------------------------------------------------------------
#define MODO_MAPEAMENTO 0

// AHRS Mahony — so backup. Kp baixo: Kp=4 puxava aceleracao linear para
// a orientacao e as 3 juntas erravam juntas.
const float MAHONY_KP = 1.2f;
const float MAHONY_KI = 0.005f;

// Rastreador de gravidade no frame do sensor (nao precisa de rumo)
const float GRAV_KP_CAL = 8.0f;
const float GRAV_KP     = 2.0f;

// Rejeicao de aceleracao (metodo da biblioteca Fusion / x-io).
// Ao mover o braco rapido o acelerometro le gravidade + aceleracao do movimento.
// Se o filtro acreditar nisso, a orientacao inteira inclina errado e os TRES
// angulos erram juntos. Aqui o peso do acelerometro cai conforme |a| se afasta
// de 1 g, e chega a zero fora da janela.
const float ACC_JANELA   = 0.30f;   // g de desvio onde o peso chega a zero
const float ACC_RECUP_S  = 1.5f;    // rejeitando por mais que isso, volta fraco
const float ACC_PESO_MIN = 0.15f;   // peso residual durante a recuperacao

// Deteccao de repouso (ZUPT) -> corrige bias do gyro e mata a deriva de yaw
const float REPOUSO_GYRO   = 2.5f;   // °/s
const float REPOUSO_ACC    = 0.06f;  // |a| - 1g
const float BIAS_ADAPT     = 0.01f;  // quanto o bias acompanha quando parado

// BASE — filtro complementar de rumo.
//
// Sem magnetometro o giro em torno da vertical e' inobservavel pelo
// acelerometro: a gravidade nao carrega informacao de rumo. E o azimute do
// braco, que seria a referencia, e' indefinido com o braco na vertical.
//
// Por isso a base usa DUAS fontes:
//   GIROSCOPIO  -> integra a velocidade angular do braco em torno da vertical.
//                  Responde na hora e funciona em QUALQUER pose, sem
//                  singularidade. Sozinho, deriva.
//   AZIMUTE     -> referencia sem deriva, porem so confiavel com o braco
//                  inclinado. Entra como correcao lenta, com peso sin(elev).
//
// Resultado: resposta imediata sempre, e a deriva e' zerada toda vez que voce
// baixa o braco. Nada de base congelada.
const float ELEV_MIN_BASE  = 20.0f;   // abaixo disso o azimute nao corrige
const float ELEV_MAX_BASE  = 160.0f;
const float BASE_KP        = 2.5f;    // 1/s — puxa ao azimute do MPU do braco (1 sensor)
const float BASE_MAX_DEG_S = 250.0f;
const float BASE_ZONA_MORTA = 1.2f;   // °/s — ω·g residual parado

// Saida
const float EMA_SAIDA      = 0.35f;  // 0..1 — maior = mais responsivo
const float DEADZONE_GRAUS = 1.5f;
const unsigned long TX_PERIODO_MS = 40;   // 25 Hz (airtime LoRa ~15 ms)

// ---------------------------------------------------------------- estado
struct Quat { float w, x, y, z; };
struct Vec3 { float x, y, z; };

struct Imu {
  Quat q;
  float bx, by, bz;       // bias do gyro (°/s)
  float ix, iy, iz;       // integral Mahony
  Vec3 eixo;              // eixo longitudinal do segmento (frame do sensor)
  float tempoRejeitado;   // ha quanto tempo o acelerometro esta sendo ignorado
  float pesoAcc;          // confianca atual no acelerometro (diagnostico)
  uint16_t leituras;      // estatistica desde o ultimo print
  uint16_t falhas;
  uint8_t falhasSeguidas;
  unsigned long ultimaRecupMs;
  Vec3 giroMundo;         // nao usado nas juntas (rumo do quat e' invisivel)
  Vec3 w;                 // gyro no frame do sensor (°/s), ja sem bias
  Vec3 gF;                // gravidade estimada no frame do sensor (unitaria)
  bool parado;
  bool ok;
};

float cotZero = 0;
float pulsoZero = 0;
float sinalCot = 1.0f;
float sinalPulso = 1.0f;

// Um sensor mudo congela o quaternion dele. Como o ombro e a base saem do
// vetor do braco e o cotovelo e' o angulo entre braco e antebraco, um unico
// sensor morto trava ombro + base e faz o cotovelo responder a tudo.
// Por isso a falha agora e' detectada, recuperada e mostrada no serial.
const uint8_t FALHAS_P_RECUPERAR = 5;
const unsigned long RECUP_INTERVALO_MS = 500;

bool calibrando = false;  // durante a calibracao o acelerometro e' confiavel

// ---------------------------------------------------------------------------
// RESTRICAO DE DOBRADICA (Laidig, Schauer & Seel)
//
// Com fusao 6D cada IMU estima orientacao num referencial global PROPRIO,
// girado em torno da vertical em relacao aos outros. Como o angulo do cotovelo
// vem de dois sensores, essa diferenca de rumo entra direto no resultado.
//
// O cotovelo e' (aproximadamente) uma dobradica: o eixo de flexao visto pelo
// sensor do braco e o mesmo eixo visto pelo sensor do antebraco TEM que
// apontar para a mesma direcao no mundo. A diferenca de azimute entre os dois
// e' exatamente o erro de rumo, e da para medi-la e corrigi-la em tempo real.
// No paper original isso derruba o erro de 25,8° para 2,6°.
//
// A restricao nao funciona com o eixo da dobradica na vertical (o azimute some),
// por isso a correcao e' ponderada pela componente horizontal do eixo.
// ---------------------------------------------------------------------------
Vec3 eixoCotBraco;        // eixo da dobradica no frame do sensor do braco
Vec3 eixoCotAnte;         // o mesmo eixo no frame do sensor do antebraco
bool eixoCotOk = false;
float rumoCorrigido = 0;  // quanto ja foi corrigido no antebraco (diagnostico)

// Eixo do pulso / refs de gravidade transversal (torcao da palma).
// Em modo torcao: eixoPulsoAnte/Mao guardam o "baixo" na pose neutra.
Vec3 eixoPulsoAnte = { 0, 0, 0 };
Vec3 eixoPulsoMao  = { 0, 0, 0 };
bool eixoPulsoOk = false;
float rumoPulso = 0;

// Ombro: pose 0 (para cima) + pose 90 (horizontal a frente).
// ganho faz a horizontal medida virar 90° no robo; zero tira o residuo
// da pose inicial; o eixo funcional define o plano sagital.
Vec3  eixoOmbroSensor = { 0, 1, 0 };
bool  ombroCalOk = false;
float ganhoOmbro = 1.0f;
float zeroOmbro  = 0.0f;

const float HEAD_KP      = 1.5f;   // 1/s — forca da correcao de rumo
const float HEAD_MIN_HOR = 0.20f;  // eixo mais vertical que isso: nao corrige
const float FLEX_MIN_EIXO = 22.0f; // flexao minima para a amostra valer
const float FLEX_MAX_EIXO = 158.0f;
const unsigned long CAL_OMBRO_MS = 25000;  // teto; termina antes se a pose fechar
const unsigned long CAL_FLEX_MS  = 25000;
const unsigned long CAL_PULSO_MS = 20000;
const int   CAL_GYRO_MIN_N   = 50;
const int   CAL_GRAV_MIN_N   = 80;
const float CAL_GYRO_MIN_W   = 22.0f;
const float CAL_PROX_MAX_W   = 35.0f;  // se o proximal girar mais que isso, a amostra e' lixo
const float CAL_SPAN_COT     = 50.0f;  // flexao minima no passo 3 (graus)
const float CAL_SPAN_PUL     = 30.0f;

// Punho: limite fisiologico da pronossupinacao. Leitura alem disso e' deriva.
const float PULSO_ROM    = 100.0f;  // graus
const float PULSO_ROM_KP = 0.5f;    // 1/s — velocidade de devolucao

// Filtro complementar por junta (Seel eq. 15): gyro prediz, gravidade corrige.
// Kp=2 -> constante de tempo ~0.5 s. Kp alto faz aceleracao linear virar angulo.
const float JUNTA_KP     = 2.0f;    // 1/s — cotovelo e pulso
const float OMBRO_KP     = 6.0f;    // elevacao tem gravidade confiavel

// Consenso dos 3 giros para a BASE. Se os world-Z concordam, o corpo
// esta girando e a media e' melhor que um sensor so. Se discordam,
// voce esta articulando e so o MPU do braco manda.
const float CONSENSUS_MAX = 18.0f;  // °/s de espalhamento maximo
const float CADEIA_RETA   = 22.0f;  // graus — abaixo disso os 3 votam no ombro

Imu imu[NUM_MPU];
Quat qRelPulso0;          // mao vs antebraco na calibracao
float azimuteRef = 0;
bool  azimuteTravado = false;
bool  avisouBaseTravada = false;
float baseMemoria = 0;    // ultimo azimute valido

float saida[NUM_JUNTAS];
bool  saidaInit = false;
bool  juntaInit = false;   // filtro complementar por junta ja inicializado

unsigned long ultimoUs = 0;
unsigned long ultimoTxMs = 0;
int gSda, gScl;

// ---------------------------------------------------------------- soft I2C
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
  gSda = PIN_SDA[i]; gScl = PIN_SCL[i];
  softSdaHigh(); softSclHigh();
}

bool softWriteReg(uint8_t reg, uint8_t val) {
  softStart();
  if (!softWriteByte((MPU_ADDR << 1) | 0)) { softStop(); return false; }
  if (!softWriteByte(reg))                 { softStop(); return false; }
  if (!softWriteByte(val))                 { softStop(); return false; }
  softStop();
  return true;
}

bool softRead(uint8_t reg, uint8_t* buf, int n) {
  softStart();
  if (!softWriteByte((MPU_ADDR << 1) | 0)) { softStop(); return false; }
  if (!softWriteByte(reg))                 { softStop(); return false; }
  softStart();
  if (!softWriteByte((MPU_ADDR << 1) | 1)) { softStop(); return false; }
  for (int i = 0; i < n; i++) buf[i] = softReadByte(i < n - 1);
  softStop();
  return true;
}

bool iniciarSensor(int i) {
  usarBarramento(i);
  if (!softWriteReg(0x6B, 0x00)) return false;   // acorda
  delay(10);
  softWriteReg(0x1B, 0x08);   // gyro ±500 °/s
  softWriteReg(0x1C, 0x00);   // accel ±2 g
  softWriteReg(0x1A, 0x03);   // DLPF 44 Hz — corta vibracao do braco
  softWriteReg(0x19, 0x04);   // sample rate 200 Hz
  return true;
}

// Destrava um barramento preso. Se um pacote for cortado no meio (ruido dos
// motores, fio comprido), o MPU pode ficar segurando o SDA em nivel baixo e o
// barramento morre ate alguem mandar 9 pulsos de clock para liberar o escravo.
void recuperarBarramento(int i) {
  usarBarramento(i);
  softSdaHigh();
  for (int k = 0; k < 9; k++) { softSclHigh(); softSclLow(); }
  softStop();
  iniciarSensor(i);
}

// Le accel (g) e gyro (°/s) num unico burst de 14 bytes
bool lerSensor(int i, Vec3& a, Vec3& g) {
  usarBarramento(i);
  uint8_t b[14];
  if (!softRead(0x3B, b, 14)) return false;
  int16_t ax = (b[0]  << 8) | b[1];
  int16_t ay = (b[2]  << 8) | b[3];
  int16_t az = (b[4]  << 8) | b[5];
  int16_t gx = (b[8]  << 8) | b[9];
  int16_t gy = (b[10] << 8) | b[11];
  int16_t gz = (b[12] << 8) | b[13];
  a.x = ax / 16384.0f; a.y = ay / 16384.0f; a.z = az / 16384.0f;
  g.x = gx / 65.5f;    g.y = gy / 65.5f;    g.z = gz / 65.5f;
  return true;
}

// ---------------------------------------------------------------- math
float wrap180(float a) {
  while (a >  180.0f) a -= 360.0f;
  while (a < -180.0f) a += 360.0f;
  return a;
}

bool normalizar(Vec3& v) {
  float n = sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
  if (n < 1e-4f) return false;
  v.x /= n; v.y /= n; v.z /= n;
  return true;
}

float dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }

Vec3 cross(const Vec3& a, const Vec3& b) {
  return { a.y * b.z - a.z * b.y,
           a.z * b.x - a.x * b.z,
           a.x * b.y - a.y * b.x };
}

float anguloEntre(const Vec3& a, const Vec3& b) {
  float d = constrain(dot(a, b), -1.0f, 1.0f);
  return acosf(d) * 180.0f / PI;
}

float mag3(const Vec3& v) {
  return sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
}

// Acumula uma direcao unitaria invertendo o sinal se estiver no hemisferio oposto.
void acumularDir(Vec3& soma, int& n, Vec3 d) {
  if (!normalizar(d)) return;
  if (n > 0 && dot(d, soma) < 0) { d.x = -d.x; d.y = -d.y; d.z = -d.z; }
  soma.x += d.x; soma.y += d.y; soma.z += d.z;
  n++;
}

bool fecharMedia(Vec3 soma, int n, int minimo, Vec3& out) {
  if (n < minimo) return false;
  out = soma;
  return normalizar(out);
}

// Remove a componente ao longo do osso (fica so o plano transversal).
Vec3 perpAoEixo(const Vec3& v, const Vec3& z) {
  float p = dot(v, z);
  return { v.x - p * z.x, v.y - p * z.y, v.z - p * z.z };
}

// Angulo com sinal de a -> b em torno do eixo (graus). a e b unitarios.
float anguloEmTorno(const Vec3& a, const Vec3& b, const Vec3& eixo) {
  float c = constrain(dot(a, b), -1.0f, 1.0f);
  float s = dot(cross(a, b), eixo);
  return atan2f(s, c) * 180.0f / PI;
}

Quat qMul(const Quat& a, const Quat& b) {
  Quat r;
  r.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
  r.x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y;
  r.y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x;
  r.z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w;
  return r;
}

Quat qConj(const Quat& q) { return { q.w, -q.x, -q.y, -q.z }; }

// Roda um vetor do frame do sensor para o mundo:  v_mundo = q * v * q'
Vec3 qRotate(const Quat& q, const Vec3& v) {
  float tx = 2.0f * (q.y * v.z - q.z * v.y);
  float ty = 2.0f * (q.z * v.x - q.x * v.z);
  float tz = 2.0f * (q.x * v.y - q.y * v.x);
  Vec3 r;
  r.x = v.x + q.w * tx + (q.y * tz - q.z * ty);
  r.y = v.y + q.w * ty + (q.z * tx - q.x * tz);
  r.z = v.z + q.w * tz + (q.x * ty - q.y * tx);
  return r;
}

// AHRS Mahony 6DOF (accel + gyro). Alinha o eixo Z do mundo com a gravidade.
// Com rejeicao de aceleracao: o acelerometro so e' levado a serio quando a
// leitura tem magnitude de gravidade pura.
void mahony(Imu& s, Vec3 a, Vec3 g, float dt) {
  float gx = (g.x - s.bx) * PI / 180.0f;
  float gy = (g.y - s.by) * PI / 180.0f;
  float gz = (g.z - s.bz) * PI / 180.0f;

  float magA = sqrtf(a.x * a.x + a.y * a.y + a.z * a.z);
  float peso;
  if (calibrando) {
    peso = 1.0f;
    s.tempoRejeitado = 0;
  } else {
    peso = 1.0f - fabsf(magA - 1.0f) / ACC_JANELA;
    if (peso < 0) peso = 0;
    if (peso > 1) peso = 1;

    // Rejeitar para sempre faria o yaw e o pitch derivarem sem volta.
    // Depois de um tempo o filtro reaceita o acelerometro com peso baixo.
    if (peso <= 0.0f) {
      s.tempoRejeitado += dt;
      if (s.tempoRejeitado > ACC_RECUP_S) peso = ACC_PESO_MIN;
    } else {
      s.tempoRejeitado = 0;
    }
  }
  s.pesoAcc = peso;

  if (peso > 0.0f && normalizar(a)) {
    // gravidade estimada pelo quaternion atual (linha 3 da matriz de rotacao)
    float vx = 2.0f * (s.q.x * s.q.z - s.q.w * s.q.y);
    float vy = 2.0f * (s.q.w * s.q.x + s.q.y * s.q.z);
    float vz = s.q.w * s.q.w - s.q.x * s.q.x - s.q.y * s.q.y + s.q.z * s.q.z;

    float ex = a.y * vz - a.z * vy;
    float ey = a.z * vx - a.x * vz;
    float ez = a.x * vy - a.y * vx;

    // A integral so acumula com leitura confiavel, senao o erro de aceleracao
    // vira bias permanente e o sensor fica torto mesmo depois de parar.
    if (peso > 0.8f) {
      s.ix += MAHONY_KI * ex * dt;
      s.iy += MAHONY_KI * ey * dt;
      s.iz += MAHONY_KI * ez * dt;
    }

    gx += peso * MAHONY_KP * ex + s.ix;
    gy += peso * MAHONY_KP * ey + s.iy;
    gz += peso * MAHONY_KP * ez + s.iz;
  }

  Quat q = s.q;
  s.q.w += 0.5f * (-q.x * gx - q.y * gy - q.z * gz) * dt;
  s.q.x += 0.5f * ( q.w * gx + q.y * gz - q.z * gy) * dt;
  s.q.y += 0.5f * ( q.w * gy - q.x * gz + q.z * gx) * dt;
  s.q.z += 0.5f * ( q.w * gz + q.x * gy - q.y * gx) * dt;

  float n = sqrtf(s.q.w * s.q.w + s.q.x * s.q.x + s.q.y * s.q.y + s.q.z * s.q.z);
  if (n > 1e-6f) { s.q.w /= n; s.q.x /= n; s.q.y /= n; s.q.z /= n; }
}

// Gravidade no frame do sensor: g' = -ω × g, corrigida pelo acelerometro
// quando |a| ~ 1g. NAO usa rumo. e' o que observa inclinacao de verdade.
void atualizarGravidade(Imu& s, Vec3 a, Vec3 wDeg, float dt) {
  s.w = wDeg;
  float wx = wDeg.x * PI / 180.0f;
  float wy = wDeg.y * PI / 180.0f;
  float wz = wDeg.z * PI / 180.0f;
  Vec3 g = s.gF;
  s.gF.x = g.x + dt * ( wz * g.y - wy * g.z);
  s.gF.y = g.y + dt * ( wx * g.z - wz * g.x);
  s.gF.z = g.z + dt * ( wy * g.x - wx * g.y);

  float magA = sqrtf(a.x * a.x + a.y * a.y + a.z * a.z);
  float peso = 1.0f - fabsf(magA - 1.0f) / ACC_JANELA;
  if (calibrando) peso = 1.0f;
  if (peso < 0) peso = 0;
  if (peso > 1) peso = 1;
  s.pesoAcc = peso;
  if (peso > 0.0f && magA > 0.2f) {
    float inv = 1.0f / magA;
    float k = calibrando ? GRAV_KP_CAL : GRAV_KP;
    s.gF.x += peso * k * (a.x * inv - s.gF.x) * dt;
    s.gF.y += peso * k * (a.y * inv - s.gF.y) * dt;
    s.gF.z += peso * k * (a.z * inv - s.gF.z) * dt;
  }
  normalizar(s.gF);
}

// Pitch da gravidade no plano da dobradica (Seel). 0 = segmento alinhado
// com a gravidade no plano (braco para cima na pose zero).
float pitchHinge(const Vec3& g, const Vec3& z, const Vec3& j) {
  Vec3 x = cross(z, j);
  if (!normalizar(x)) return 0;
  return atan2f(dot(g, x), dot(g, z)) * 180.0f / PI;
}

// Leva um vetor do mundo para o frame do sensor (transposta da rotacao)
Vec3 qRotateInv(const Quat& q, const Vec3& v) {
  return qRotate(qConj(q), v);
}

// Quaternion de rotacao de 'graus' em torno do eixo Z do mundo
Quat quatZ(float graus) {
  float m = graus * PI / 360.0f;   // metade do angulo, em rad
  return { cosf(m), 0, 0, sinf(m) };
}

// Quaternion de rotacao de 'graus' em torno de um eixo qualquer (unitario)
Quat quatEixo(const Vec3& n, float graus) {
  float m = graus * PI / 360.0f;
  float s = sinf(m);
  return { cosf(m), n.x * s, n.y * s, n.z * s };
}

float mediana3(float a, float b, float c) {
  if (a > b) { float t = a; a = b; b = t; }
  if (b > c) { float t = b; b = c; c = t; }
  if (a > b) { float t = a; a = b; b = t; }
  return b;
}

// Laidig: o eixo da junta visto pelos dois sensores tem que ter o mesmo
// azimute. A diferenca e' o erro de rumo do sensor distal. Devolve o
// erro (graus) e aplica a correcao no quaternion distal. horizOut e'
// a menor projecao horizontal (0 = eixo vertical, restricao cega).
float alinharRumo(Imu& distal, const Vec3& eixoProxSensor, const Quat& qProx,
                  const Vec3& eixoDistSensor, float dt, float& horizOut) {
  Vec3 a1 = qRotate(qProx, eixoProxSensor);
  Vec3 a2 = qRotate(distal.q, eixoDistSensor);
  float h1 = sqrtf(a1.x * a1.x + a1.y * a1.y);
  float h2 = sqrtf(a2.x * a2.x + a2.y * a2.y);
  horizOut = (h1 < h2) ? h1 : h2;
  if (horizOut <= HEAD_MIN_HOR) return 0;
  float erro = wrap180((atan2f(a1.y, a1.x) - atan2f(a2.y, a2.x)) * 180.0f / PI);
  float corr = HEAD_KP * horizOut * erro * dt;
  distal.q = qMul(quatZ(corr), distal.q);
  return erro;
}

// Swing-twist: quanto q girou em torno do eixo n (graus)
float torcaoEmTorno(const Quat& q, const Vec3& n) {
  float proj = q.x * n.x + q.y * n.y + q.z * n.z;
  float tw = q.w, tx = n.x * proj, ty = n.y * proj, tz = n.z * proj;
  float mag = sqrtf(tw * tw + tx * tx + ty * ty + tz * tz);
  if (mag < 1e-6f) return 0;
  tw /= mag; proj /= mag;
  return 2.0f * atan2f(proj, tw) * 180.0f / PI;
}

// ---------------------------------------------------------------- LoRa
void configurarLoRa() {
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println("ERRO LoRa");
    while (true) delay(1000);
  }
  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(250E3);
  LoRa.setCodingRate4(5);
  LoRa.enableCrc();
  LoRa.setTxPower(17);
  Serial.println("LoRa TX OK");
}

int8_t paraByte(int j, float graus) {
  graus *= GANHO[j];
  if (INV_TX[j]) graus = -graus;
  graus = constrain(graus, -127.0f, 127.0f);
  return (int8_t)(graus + (graus >= 0 ? 0.5f : -0.5f));
}

void enviarLoRa(int8_t c, int8_t o, int8_t p, int8_t b) {
  LoRa.beginPacket();
  LoRa.write(LORA_HEADER);
  LoRa.write((uint8_t)c);
  LoRa.write((uint8_t)o);
  LoRa.write((uint8_t)p);
  LoRa.write((uint8_t)b);
  LoRa.endPacket();
}

// ---------------------------------------------------------------- calibracao
void linha() {
  Serial.println(F("------------------------------------------------"));
}

void tituloPasso(int n, int total, const char* nome) {
  Serial.println();
  linha();
  Serial.printf("PASSO %d/%d — %s\n", n, total, nome);
  linha();
}

void contagemRegressiva(int segundos) {
  for (int s = segundos; s > 0; s--) {
    Serial.printf("  comeca em %d...\n", s);
    delay(1000);
  }
  Serial.println(F("  AGORA"));
}

void imprimirGuiaCalibracao() {
  Serial.println();
  linha();
  Serial.println(F("CALIBRACAO — 5 passos, voce controla o ritmo"));
  linha();
  Serial.println(F("  Cada passo ESPERA voce entrar na pose e apertar ENTER."));
  Serial.println(F("  Nao precisa ter pressa. r so recomeça DEPOIS que acabar."));
  Serial.println();
  Serial.println(F("  1. ZERO: braco ESTICADO PARA CIMA, parado."));
  Serial.println(F("  2. OMBRO em duas poses paradas (nao precisa ficar"));
  Serial.println(F("     subindo e descendo o tempo todo):"));
  Serial.println(F("       2a. para CIMA, parado"));
  Serial.println(F("       2b. HORIZONTAL a FRENTE, parado"));
  Serial.println(F("  3. COTOVELO: braco horizontal, ombro parado,"));
  Serial.println(F("     dobre ate o L e estique. Depois estique e fique parado."));
  Serial.println(F("  4. PULSO: antebraco parado na horizontal,"));
  Serial.println(F("     GIRE A PALMA (pronacao/supinacao)."));
  Serial.println(F("     Depois palma NEUTRA e parado."));
  Serial.println(F("  5. BASE: de novo horizontal a FRENTE, parado."));
  linha();
}

void drenarSerial() {
  while (Serial.available()) Serial.read();
}

// Espera o usuario entrar na pose. Qualquer tecla avanca (ENTER no Monitor).
void esperarEnter(const char* dica) {
  drenarSerial();
  Serial.println();
  if (dica) Serial.println(dica);
  Serial.println(F("  >>> Pose pronta? Clique no Serial e aperte ENTER."));
  unsigned long t0 = millis();
  while (millis() - t0 < 120000UL) {
    if (Serial.available()) {
      drenarSerial();
      Serial.println(F("  ok."));
      delay(250);
      return;
    }
    delay(20);
  }
  Serial.println(F("  (sem tecla — seguindo)"));
}

// True se os 3 MPUs estao quietos (gyro baixo e |a| ~ 1 g).
bool sensoresParados() {
  for (int i = 0; i < NUM_MPU; i++) {
    if (!imu[i].ok) continue;
    Vec3 a, g;
    if (!lerSensor(i, a, g)) return false;
    float magA = sqrtf(a.x * a.x + a.y * a.y + a.z * a.z);
    float magG = fabsf(g.x) + fabsf(g.y) + fabsf(g.z);
    if (fabsf(magA - 1.0f) > 0.08f) return false;
    if (magG > 4.0f) return false;
  }
  return true;
}

void esperarParado(unsigned long quietoMs, unsigned long timeoutMs) {
  Serial.println(F("  esperando voce ficar PARADO..."));
  unsigned long t0 = millis();
  unsigned long desde = 0;
  int ultimo = -1;
  while (millis() - t0 < timeoutMs) {
    if (sensoresParados()) {
      if (!desde) desde = millis();
      unsigned long ok = millis() - desde;
      int falta = (int)((quietoMs > ok) ? (quietoMs - ok + 999) / 1000 : 0);
      if (falta != ultimo) {
        ultimo = falta;
        if (falta > 0) Serial.printf("  continue parado... %d s\n", falta);
      }
      if (ok >= quietoMs) {
        Serial.println(F("  parado confirmado."));
        return;
      }
    } else {
      desde = 0;
      if (ultimo != -2) {
        Serial.println(F("  ainda tem movimento — trave o braco."));
        ultimo = -2;
      }
    }
    delay(25);
  }
  Serial.println(F("  nao ficou totalmente parado. vou medir assim mesmo."));
}

// Le os 3 sensores uma vez e atualiza a fusao. Usado pelos lacos de calibracao.
void passoFusao(float dt) {
  for (int i = 0; i < NUM_MPU; i++) {
    if (!imu[i].ok) continue;
    Vec3 a, g;
    if (!lerSensor(i, a, g)) continue;
    if (sqrtf(a.x * a.x + a.y * a.y + a.z * a.z) < 0.1f) continue;
    Vec3 w = { g.x - imu[i].bx, g.y - imu[i].by, g.z - imu[i].bz };
    atualizarGravidade(imu[i], a, w, dt);
    mahony(imu[i], a, g, dt);
  }
}

float elevacaoBraco() {
  return anguloEntre(imu[IDX_BRACO].eixo, imu[IDX_BRACO].gF);
}

void fusaoTick(unsigned long& ant) {
  unsigned long ag = micros();
  float dt = (ag - ant) / 1000000.0f;
  ant = ag;
  if (dt <= 0 || dt > 0.05f) dt = 0.01f;
  passoFusao(dt);
}

bool esperarFaixaElevacao(float lo, float hi, unsigned long timeoutMs, const char* nome) {
  Serial.printf("  esperando ombro entre %.0f° e %.0f° (%s)\n", lo, hi, nome);
  unsigned long t0 = millis();
  unsigned long ant = micros();
  unsigned long lastPrint = 0;
  unsigned long okDesde = 0;
  while (millis() - t0 < timeoutMs) {
    fusaoTick(ant);
    float el = elevacaoBraco();
    bool naFaixa = (el >= lo && el <= hi);
    bool quieto = mag3(imu[IDX_BRACO].w) < 10.0f;
    if (millis() - lastPrint > 500) {
      lastPrint = millis();
      if (el < lo)      Serial.printf("  ombro %.0f°  — abaixe mais (alvo %.0f-%.0f)\n", el, lo, hi);
      else if (el > hi) Serial.printf("  ombro %.0f°  — suba um pouco\n", el);
      else if (!quieto) Serial.printf("  ombro %.0f°  — boa, agora FIQUE PARADO\n", el);
      else              Serial.printf("  ombro %.0f°  — parado, medindo...\n", el);
    }
    if (naFaixa && quieto) {
      if (!okDesde) okDesde = millis();
      if (millis() - okDesde >= 1200) {
        Serial.printf("  pose OK (%.0f°).\n", el);
        return true;
      }
    } else {
      okDesde = 0;
    }
  }
  Serial.println(F("  nao segurou a pose o suficiente. vou medir assim mesmo."));
  return false;
}

float medirElevacaoMedia(unsigned long ms) {
  float s = 0;
  int n = 0;
  unsigned long t0 = millis();
  unsigned long ant = micros();
  while (millis() - t0 < ms) {
    fusaoTick(ant);
    s += elevacaoBraco();
    n++;
  }
  return n ? s / n : elevacaoBraco();
}

void medirAzimuteMedia(unsigned long ms, float& azOut, float& elOut) {
  float ssin = 0, scos = 0, sel = 0;
  int n = 0;
  unsigned long t0 = millis();
  unsigned long ant = micros();
  while (millis() - t0 < ms) {
    fusaoTick(ant);
    Vec3 u = qRotate(imu[IDX_BRACO].q, imu[IDX_BRACO].eixo);
    if (!normalizar(u)) continue;
    float az = atan2f(u.y, u.x);
    ssin += sinf(az);
    scos += cosf(az);
    sel += elevacaoBraco();
    n++;
  }
  azOut = (n > 0) ? atan2f(ssin, scos) * 180.0f / PI : 0;
  elOut = (n > 0) ? sel / n : elevacaoBraco();
}

float medirHingeMedia(int idxP, int idxD, const Vec3& jP, const Vec3& jD, unsigned long ms) {
  float s = 0;
  int n = 0;
  unsigned long t0 = millis();
  unsigned long ant = micros();
  while (millis() - t0 < ms) {
    fusaoTick(ant);
    s += wrap180(
      pitchHinge(imu[idxD].gF, imu[idxD].eixo, jD) -
      pitchHinge(imu[idxP].gF, imu[idxP].eixo, jP));
    n++;
  }
  return n ? s / n : 0;
}

// Eixo de uma dobradica 1-DoF no FRAME DE CADA SENSOR (Seel 2012 / Stancin).
// NAO usa quaternion: dois Mahony 6D nao compartilham rumo, e u×v no "mundo"
// mistura as juntas. Fontes, as duas no proprio IMU:
//   1) direcao de ω quando |ω| e' grande  (eixo real do movimento)
//   2) eixo_longitudinal × gravidade      (dobradica no plano vertical)
// O proximal parado nao gera ω — cai no (2), por isso o passo 3 pede o
// braco na HORIZONTAL.
bool calibrarEixoJunta(int idxP, int idxD, Vec3& eixoP, Vec3& eixoD,
                       unsigned long durMs, const char* dica, float* sinalOut,
                       float minSpan, float proxMaxW) {
  Vec3 sumWp = { 0, 0, 0 }, sumWd = { 0, 0, 0 };
  Vec3 sumGp = { 0, 0, 0 }, sumGd = { 0, 0, 0 };
  int nWp = 0, nWd = 0, nGp = 0, nGd = 0, nRuim = 0;
  float minDiff = 0, maxDiff = 0;
  bool temDiff = false;
  unsigned long t0 = millis();
  unsigned long ant = micros();
  unsigned long lastPrint = 0;
  bool avisouProx = false;

  while (millis() - t0 < durMs) {
    fusaoTick(ant);

    float wp = mag3(imu[idxP].w);

    // Proximal mexendo = ombro/antebraco vazando para esta junta. Descarta.
    if (wp > proxMaxW) {
      nRuim++;
      if (!avisouProx) {
        Serial.println(F("  o segmento de cima esta mexendo — trave ele."));
        avisouProx = true;
      }
      continue;
    }

    // So a componente PERPENDICULAR ao osso: flexao. Torcao (ω · eixo) e' ignorada.
    Vec3 wdFlex = perpAoEixo(imu[idxD].w, imu[idxD].eixo);
    Vec3 wpFlex = perpAoEixo(imu[idxP].w, imu[idxP].eixo);
    if (mag3(wdFlex) > CAL_GYRO_MIN_W) acumularDir(sumWd, nWd, wdFlex);
    if (mag3(wpFlex) > CAL_GYRO_MIN_W) acumularDir(sumWp, nWp, wpFlex);

    Vec3 np = cross(imu[idxP].eixo, imu[idxP].gF);
    Vec3 nd = cross(imu[idxD].eixo, imu[idxD].gF);
    if (mag3(np) > 0.35f) acumularDir(sumGp, nGp, np);
    if (mag3(nd) > 0.35f) acumularDir(sumGd, nGd, nd);

    Vec3 jp, jd;
    bool pOk = fecharMedia(nWp >= CAL_GYRO_MIN_N ? sumWp : sumGp,
                           nWp >= CAL_GYRO_MIN_N ? nWp  : nGp, 8, jp);
    bool dOk = fecharMedia(nWd >= CAL_GYRO_MIN_N ? sumWd : sumGd,
                           nWd >= CAL_GYRO_MIN_N ? nWd  : nGd, 8, jd);
    float d = 0;
    if (pOk && dOk) {
      d = wrap180(
        pitchHinge(imu[idxD].gF, imu[idxD].eixo, jd) -
        pitchHinge(imu[idxP].gF, imu[idxP].eixo, jp));
      if (!temDiff) { minDiff = maxDiff = d; temDiff = true; }
      else {
        if (d < minDiff) minDiff = d;
        if (d > maxDiff) maxDiff = d;
      }
    }

    if (millis() - lastPrint > 600) {
      lastPrint = millis();
      int restam = (int)((durMs - (millis() - t0) + 999) / 1000);
      if (temDiff)
        Serial.printf("  %2ds | flex ~%+.0f°  (faixa %.0f..%.0f) | %s\n",
                      restam, d, minDiff, maxDiff, dica);
      else
        Serial.printf("  %2ds | gyro distais:%d  grav:%d  (%s)\n",
                      restam, nWd, nGd, dica);
    }

    bool eixoCheio = (nWd >= CAL_GYRO_MIN_N || nGd >= CAL_GRAV_MIN_N) &&
                     (nWp >= CAL_GYRO_MIN_N || nGp >= CAL_GRAV_MIN_N);
    bool spanOk = temDiff && (maxDiff - minDiff) >= minSpan;
    if (eixoCheio && spanOk && (millis() - t0) > 5000) {
      Serial.println(F("  amostras suficientes — pode parar de dobrar."));
      break;
    }
  }

  bool pOk = fecharMedia(sumWp, nWp, CAL_GYRO_MIN_N, eixoP)
          || fecharMedia(sumGp, nGp, CAL_GRAV_MIN_N, eixoP);
  bool dOk = fecharMedia(sumWd, nWd, CAL_GYRO_MIN_N, eixoD)
          || fecharMedia(sumGd, nGd, CAL_GRAV_MIN_N, eixoD);
  if (!pOk || !dOk) {
    Serial.printf("  poucas amostras (gyro %d/%d grav %d/%d, %d sujas).\n",
                  nWp, nWd, nGp, nGd, nRuim);
    Serial.println(F("  Proximal na HORIZONTAL e PARADO, dobre o distal com amplitude."));
    return false;
  }

  // Garante dobradica de FLEXAO (perpendicular ao osso), nao torcao.
  eixoP = perpAoEixo(eixoP, imu[idxP].eixo);
  eixoD = perpAoEixo(eixoD, imu[idxD].eixo);
  if (!normalizar(eixoP)) {
    eixoP = cross(imu[idxP].eixo, imu[idxP].gF);
    if (!normalizar(eixoP)) return false;
  }
  if (!normalizar(eixoD)) {
    eixoD = cross(imu[idxD].eixo, imu[idxD].gF);
    if (!normalizar(eixoD)) return false;
  }

  float span = temDiff ? (maxDiff - minDiff) : 0;
  if (span < minSpan) {
    Serial.printf("  amplitude fraca (%.0f°, precisa >= %.0f°). Dobre mais, ate o L.\n",
                  span, minSpan);
    return false;
  }

  if (sinalOut) {
    if (temDiff && fabsf(minDiff) > fabsf(maxDiff) && minDiff < -20.0f)
      *sinalOut = -1.0f;
    else
      *sinalOut = 1.0f;
  }

  Serial.printf("  eixo OK | gyro %d/%d  grav %d/%d  faixa %.0f..%.0f°  sinal=%+.0f\n",
                nWp, nWd, nGp, nGd, minDiff, maxDiff, sinalOut ? *sinalOut : 1.0f);
  return true;
}

void calibrarOmbro() {
  tituloPasso(2, 5, "OMBRO — duas poses PARADAS");
  Serial.println(F("Cotovelo RETO o tempo todo. So frente e cima, nao para o lado."));
  ombroCalOk = false;
  ganhoOmbro = 1.0f;
  zeroOmbro = 0.0f;

  for (int tentativa = 1; tentativa <= 2 && !ombroCalOk; tentativa++) {
    if (tentativa > 1) Serial.println(F("  repetindo o passo 2..."));

    Serial.println(F("2a) Braco ESTICADO PARA CIMA, como o passo 1."));
    esperarEnter("  Suba o braco e fique parado.");
    esperarFaixaElevacao(0.0f, 22.0f, 40000, "para cima");
    esperarParado(1000, 8000);
    Serial.println(F("  medindo o zero do ombro (1.5 s)..."));
    zeroOmbro = medirElevacaoMedia(1500);

    Serial.println();
    Serial.println(F("2b) Abaixe o braco ESTICADO a FRENTE ate ficar"));
    Serial.println(F("    PARALELO ao chao. Cotovelo reto. Tronco de frente."));
    esperarEnter("  Horizontal a frente, parado.");
    esperarFaixaElevacao(78.0f, 105.0f, 45000, "horizontal a frente");
    esperarParado(1000, 8000);
    Serial.println(F("  medindo 90° e a frente da base (1.5 s)..."));
    float az = 0, el90 = 0;
    medirAzimuteMedia(1500, az, el90);

    float span = el90 - zeroOmbro;
    Serial.printf("  cima=%.1f°  horizontal=%.1f°  span=%.1f°\n", zeroOmbro, el90, span);
    if (span < 50.0f) {
      Serial.println(F("  span curto — na 2b o braco tem que ficar paralelo ao chao."));
      continue;
    }

    ganhoOmbro = 90.0f / span;
    if (ganhoOmbro < 0.75f) ganhoOmbro = 0.75f;
    if (ganhoOmbro > 1.35f) ganhoOmbro = 1.35f;
    azimuteRef = az;
    azimuteTravado = true;
    baseMemoria = 0;
    ombroCalOk = true;
    Serial.printf("PASSO 2 OK — zero=%.1f°  horizontal=%.1f°  ganho=%.2f\n",
                  zeroOmbro, el90, ganhoOmbro);
    Serial.println(F("  frente da BASE anotada nesta horizontal."));
  }

  if (!ombroCalOk)
    Serial.println(F("PASSO 2 FALHOU — o ombro vai usar 1:1 sem ganho. Tecla r no fim."));
}

void calibrarEixoCotovelo() {
  tituloPasso(3, 5, "EIXO DO COTOVELO");
  Serial.println(F("  - braco na HORIZONTAL a FRENTE (como o fim do passo 2)"));
  Serial.println(F("  - do ombro ate o cotovelo PARADO"));
  Serial.println(F("  - dobre SO o cotovelo ate um L, estique, repita DEVAGAR"));
  Serial.println(F("  O Serial mostra a flexao ao vivo. Pare quando pedir."));

  eixoCotOk = false;
  rumoCorrigido = 0;

  for (int tentativa = 1; tentativa <= 2 && !eixoCotOk; tentativa++) {
    if (tentativa > 1) Serial.println(F("  repetindo o passo 3..."));
    sinalCot = 1.0f;
    cotZero = 0;
    esperarEnter("  Horizontal, ombro travado, pronto para dobrar o cotovelo.");
    esperarFaixaElevacao(70.0f, 110.0f, 30000, "braco horizontal");

    if (!calibrarEixoJunta(IDX_BRACO, IDX_ANTE, eixoCotBraco, eixoCotAnte,
                           CAL_FLEX_MS, "dobre e estique o cotovelo",
                           &sinalCot, CAL_SPAN_COT, CAL_PROX_MAX_W)) {
      continue;
    }
    eixoCotOk = true;
  }

  if (!eixoCotOk) {
    Serial.println(F("PASSO 3 FALHOU — cotovelo no modo simples. Tecla r no fim."));
    return;
  }

  Serial.println();
  Serial.println(F("Agora ESTIQUE o cotovelo (braco horizontal, parado)."));
  Serial.println(F("Isso marca o ZERO do cotovelo — nao fique em L."));
  esperarEnter("  Cotovelo esticado, ombro ainda horizontal.");
  esperarParado(1000, 8000);
  cotZero = medirHingeMedia(IDX_BRACO, IDX_ANTE, eixoCotBraco, eixoCotAnte, 1200);
  Serial.printf("PASSO 3 OK — zero do cotovelo = %.1f°  sinal=%+.0f\n", cotZero, sinalCot);
  Serial.printf("  no Braco     [% .2f % .2f % .2f]\n",
                eixoCotBraco.x, eixoCotBraco.y, eixoCotBraco.z);
  Serial.printf("  no Antebraco [% .2f % .2f % .2f]\n",
                eixoCotAnte.x, eixoCotAnte.y, eixoCotAnte.z);
}

void calibrarEixoPulso() {
  tituloPasso(4, 5, "PULSO — TORCAO DA PALMA");
  Serial.println(F("  O motor do robo GIRA A PALMA. Por isso este passo e'"));
  Serial.println(F("  pronacao/supinacao — NAO dobre o pulso (tchauzinho)."));
  Serial.println(F("  - antebraco PARADO na HORIZONTAL"));
  Serial.println(F("  - gire a palma para dentro e para fora, umas 6 vezes"));

  eixoPulsoOk = false;
  rumoPulso = 0;
  sinalPulso = 1.0f;
  pulsoZero = 0;

  for (int tentativa = 1; tentativa <= 2 && !eixoPulsoOk; tentativa++) {
    if (tentativa > 1) Serial.println(F("  repetindo o passo 4..."));
    esperarEnter("  Antebraco travado, pronto para GIRAR A PALMA.");

    float acum = 0, minA = 0, maxA = 0;
    int nGiro = 0;
    unsigned long t0 = millis();
    unsigned long ant = micros();
    unsigned long lastPrint = 0;
    bool avisouProx = false;

    while (millis() - t0 < CAL_PULSO_MS) {
      unsigned long ag = micros();
      float dt = (ag - ant) / 1000000.0f;
      ant = ag;
      if (dt <= 0 || dt > 0.05f) dt = 0.01f;
      passoFusao(dt);

      float wp = mag3(imu[IDX_ANTE].w);
      if (wp > CAL_PROX_MAX_W) {
        if (!avisouProx) {
          Serial.println(F("  antebraco mexendo — trave o cotovelo/antebraco."));
          avisouProx = true;
        }
        continue;
      }

      float w = dot(imu[IDX_MAO].w, imu[IDX_MAO].eixo)
              - dot(imu[IDX_ANTE].w, imu[IDX_ANTE].eixo);
      if (fabsf(w) > 15.0f) {
        acum = wrap180(acum + w * dt);
        if (nGiro == 0) { minA = maxA = acum; }
        else {
          if (acum < minA) minA = acum;
          if (acum > maxA) maxA = acum;
        }
        nGiro++;
      }

      if (millis() - lastPrint > 600) {
        lastPrint = millis();
        int restam = (int)((CAL_PULSO_MS - (millis() - t0) + 999) / 1000);
        Serial.printf("  %2ds | torcao ~%+.0f°  faixa %.0f..%.0f  (gire a palma)\n",
                      restam, acum, minA, maxA);
      }

      if (nGiro > 80 && (maxA - minA) >= CAL_SPAN_PUL && (millis() - t0) > 5000) {
        Serial.println(F("  amostras suficientes — pode parar de girar."));
        break;
      }
    }

    float span = maxA - minA;
    if (nGiro < 40 || span < CAL_SPAN_PUL) {
      Serial.printf("  amplitude fraca (%.0f°, precisa >= %.0f°). Gire mais a palma.\n",
                    span, CAL_SPAN_PUL);
      continue;
    }
    if (fabsf(minA) > fabsf(maxA) && minA < -20.0f) sinalPulso = -1.0f;
    else sinalPulso = 1.0f;

    Serial.println();
    Serial.println(F("Agora deixe a PALMA NA POSE NEUTRA (a que o robo chama de zero),"));
    Serial.println(F("antebraco ainda horizontal, e fique PARADO."));
    esperarEnter("  Palma neutra, parado.");
    esperarParado(800, 6000);

    // "Relogios" de gravidade no plano transversal = referencia absoluta
    Vec3 ra = { 0, 0, 0 }, rm = { 0, 0, 0 };
    int na = 0, nm = 0;
    unsigned long tM = millis();
    ant = micros();
    while (millis() - tM < 1200) {
      fusaoTick(ant);
      Vec3 ga = perpAoEixo(imu[IDX_ANTE].gF, imu[IDX_ANTE].eixo);
      Vec3 gm = perpAoEixo(imu[IDX_MAO].gF,  imu[IDX_MAO].eixo);
      if (mag3(ga) > 0.35f) acumularDir(ra, na, ga);
      if (mag3(gm) > 0.35f) acumularDir(rm, nm, gm);
    }
    if (!fecharMedia(ra, na, 30, eixoPulsoAnte) ||
        !fecharMedia(rm, nm, 30, eixoPulsoMao)) {
      Serial.println(F("  nao deu para travar a gravidade — antebraco mais horizontal."));
      continue;
    }

    pulsoZero = 0;
    saida[J_PULSO] = 0;
    eixoPulsoOk = true;
    Serial.printf("PASSO 4 OK — torcao da palma  sinal=%+.0f  faixa %.0f..%.0f°\n",
                  sinalPulso, minA, maxA);
  }

  if (!eixoPulsoOk)
    Serial.println(F("PASSO 4 FRACO — pulso so no giro (pode derivar). Tecla r no fim."));
}

void calibrar() {
  imprimirGuiaCalibracao();
  calibrando = true;   // aqui o braco esta parado: acelerometro 100% confiavel
  avisouBaseTravada = false;

  tituloPasso(1, 5, "POSE ZERO (braco para cima)");
  Serial.println(F("Coloque o braco assim e NAO MEXA:"));
  Serial.println(F("  - braco ESTICADO PARA CIMA (vertical)"));
  Serial.println(F("  - cotovelo RETO"));
  Serial.println(F("  - pulso reto, palma na pose que voce usa"));
  Serial.println(F("  - tronco de frente"));
  esperarEnter("  Quando estiver parado para cima, aperte ENTER.");
  esperarParado(2000, 20000);
  Serial.println(F("  medindo bias e eixos (fique parado, ~12 s)..."));

  // 1) zera estado e mede bias do gyro
  for (int i = 0; i < NUM_MPU; i++) {
    imu[i].q = { 1, 0, 0, 0 };
    imu[i].bx = imu[i].by = imu[i].bz = 0;
    imu[i].ix = imu[i].iy = imu[i].iz = 0;
    imu[i].tempoRejeitado = 0;
    imu[i].pesoAcc = 1.0f;
    imu[i].leituras = 0;
    imu[i].falhas = 0;
    imu[i].falhasSeguidas = 0;
    imu[i].giroMundo = { 0, 0, 0 };
    imu[i].w = { 0, 0, 0 };
    imu[i].gF = { 0, 0, 1 };
    imu[i].parado = true;
  }

  const int N_BIAS = 400;
  for (int i = 0; i < NUM_MPU; i++) {
    if (!imu[i].ok) continue;
    float gx[400], gy[400], gz[400];
    int n = 0;
    for (int k = 0; k < N_BIAS; k++) {
      Vec3 a, g;
      if (lerSensor(i, a, g) && n < N_BIAS) {
        gx[n] = g.x; gy[n] = g.y; gz[n] = g.z;
        n++;
      }
      delay(4);
    }
    if (n < 40) continue;
    float sx = 0, sy = 0, sz = 0;
    for (int k = 0; k < n; k++) { sx += gx[k]; sy += gy[k]; sz += gz[k]; }
    sx /= n; sy /= n; sz /= n;
    float mx = 0, my = 0, mz = 0;
    int k2 = 0;
    for (int k = 0; k < n; k++) {
      if (fabsf(gx[k] - sx) + fabsf(gy[k] - sy) + fabsf(gz[k] - sz) > 3.0f) continue;
      mx += gx[k]; my += gy[k]; mz += gz[k];
      k2++;
    }
    if (k2 > 20) { imu[i].bx = mx / k2; imu[i].by = my / k2; imu[i].bz = mz / k2; }
    else { imu[i].bx = sx; imu[i].by = sy; imu[i].bz = sz; }
  }

  // Mahony converge mais tempo: pose zero precisa de gravidade bem alinhada
  unsigned long t0 = millis();
  unsigned long ant = micros();
  while (millis() - t0 < 3500) {
    unsigned long ag = micros();
    float dt = (ag - ant) / 1000000.0f;
    ant = ag;
    if (dt <= 0 || dt > 0.05f) dt = 0.01f;
    for (int i = 0; i < NUM_MPU; i++) {
      if (!imu[i].ok) continue;
      Vec3 a, g;
      if (lerSensor(i, a, g)) {
        Vec3 w = { g.x - imu[i].bx, g.y - imu[i].by, g.z - imu[i].bz };
        atualizarGravidade(imu[i], a, w, dt);
        mahony(imu[i], a, g, dt);
      }
    }
  }

  // Eixo longitudinal: media da gravidade no sensor, com rejeicao de outlier
  const int N_EIXO = 200;
  bool algumMudo = false;
  bool mudo[NUM_MPU] = { false, false, false };
  for (int i = 0; i < NUM_MPU; i++) {
    if (!imu[i].ok) { imu[i].eixo = { 0, 0, 1 }; mudo[i] = true; algumMudo = true; continue; }
    Vec3 am[200];
    int n = 0;
    for (int k = 0; k < N_EIXO; k++) {
      Vec3 a, g;
      if (lerSensor(i, a, g) && n < N_EIXO) {
        float mag = sqrtf(a.x * a.x + a.y * a.y + a.z * a.z);
        if (mag > 0.7f && mag < 1.3f) { am[n++] = a; }
      }
      delay(5);
    }
    if (n < N_EIXO / 2) {
      imu[i].eixo = { 0, 0, 1 };
      mudo[i] = true;
      algumMudo = true;
      continue;
    }
    Vec3 s = { 0, 0, 0 };
    for (int k = 0; k < n; k++) { s.x += am[k].x; s.y += am[k].y; s.z += am[k].z; }
    s.x /= n; s.y /= n; s.z /= n;
    if (!normalizar(s)) { mudo[i] = true; algumMudo = true; imu[i].eixo = { 0, 0, 1 }; continue; }
    Vec3 s2 = { 0, 0, 0 };
    int k2 = 0;
    for (int k = 0; k < n; k++) {
      Vec3 t = am[k];
      if (!normalizar(t)) continue;
      if (dot(t, s) < 0.97f) continue;   // descarta > ~14°
      s2.x += t.x; s2.y += t.y; s2.z += t.z;
      k2++;
    }
    if (k2 < n / 2 || !normalizar(s2)) {
      imu[i].eixo = s;
    } else {
      imu[i].eixo = s2;
    }
    imu[i].gF = imu[i].eixo;
  }

  // 4) referencia de torcao do pulso
  qRelPulso0 = qMul(qConj(imu[IDX_ANTE].q), imu[IDX_MAO].q);

  azimuteRef = 0;
  azimuteTravado = false;
  avisouBaseTravada = false;
  baseMemoria = 0;
  saidaInit = false;
  juntaInit = false;
  eixoPulsoOk = false;
  rumoPulso = 0;
  cotZero = 0;
  pulsoZero = 0;
  sinalCot = 1.0f;
  sinalPulso = 1.0f;
  eixoPulsoAnte = { 0, 0, 0 };
  eixoPulsoMao  = { 0, 0, 0 };
  ombroCalOk = false;
  ganhoOmbro = 1.0f;
  zeroOmbro = 0.0f;
  eixoOmbroSensor = { 0, 1, 0 };
  for (int j = 0; j < NUM_JUNTAS; j++) saida[j] = 0;

  Serial.println();
  bool passo1Ok = !algumMudo;
  for (int i = 0; i < NUM_MPU; i++) {
    Serial.printf("  %-10s  eixo=[% .2f % .2f % .2f]  bias=[% .2f % .2f % .2f]  %s\n",
                  NOME_MPU[i], imu[i].eixo.x, imu[i].eixo.y, imu[i].eixo.z,
                  imu[i].bx, imu[i].by, imu[i].bz,
                  imu[i].ok && !mudo[i] ? "OK" : "FALHOU");
  }

  if (!passo1Ok) {
    Serial.println();
    Serial.println(F("PASSO 1 FALHOU — algum MPU nao respondeu."));
    Serial.println(F("Eixo [0.00 0.00 1.00] = sensor mudo."));
    Serial.println(F("  Mao        SDA 27  SCL 14"));
    Serial.println(F("  Antebraco  SDA 25  SCL 26"));
    Serial.println(F("  Braco      SDA  4  SCL 13"));
    Serial.println(F("Confira ligacao e aperte r para tentar de novo."));
  } else {
    Serial.println(F("PASSO 1 OK — os 3 MPUs leram a pose zero."));
  }

  if (imu[IDX_BRACO].ok && !algumMudo) {
    calibrarOmbro();
  } else {
    ombroCalOk = false;
    Serial.println();
    Serial.println(F("PASSO 2 PULADO — falta o sensor do braco."));
  }

  if (imu[IDX_BRACO].ok && imu[IDX_ANTE].ok && !algumMudo) {
    calibrarEixoCotovelo();
  } else {
    eixoCotOk = false;
    Serial.println();
    Serial.println(F("PASSO 3 PULADO — falta sensor do braco ou do antebraco."));
  }

  if (imu[IDX_ANTE].ok && imu[IDX_MAO].ok && !algumMudo) {
    calibrarEixoPulso();
  } else {
    eixoPulsoOk = false;
    Serial.println();
    Serial.println(F("PASSO 4 PULADO — falta sensor do antebraco ou da mao."));
  }

  Serial.println();
  tituloPasso(5, 5, "ZERO DA BASE");
  Serial.println(F("Braco ESTICADO a FRENTE, horizontal, cotovelo reto, parado."));
  Serial.println(F("Essa direcao e' o zero da base do robo."));
  esperarEnter("  Horizontal a frente.");
  esperarFaixaElevacao(75.0f, 105.0f, 30000, "horizontal a frente");
  esperarParado(800, 6000);
  {
    float az = 0, el = 0;
    medirAzimuteMedia(1200, az, el);
    azimuteRef = az;
    azimuteTravado = true;
    baseMemoria = 0;
    saida[J_BASE] = 0;
  }
  avisouBaseTravada = true;
  Serial.printf("PASSO 5 OK — base zerada (ombro nesta pose = %.0f°).\n", elevacaoBraco());

  Serial.println();
  linha();
  Serial.println(F("RESUMO"));
  Serial.printf("  ombro    %s\n", ombroCalOk ? "OK" : "FALHOU");
  Serial.printf("  cotovelo %s  (zero %.1f°)\n", eixoCotOk ? "OK" : "FALHOU", cotZero);
  Serial.printf("  pulso    %s  (zero %.1f°)\n", eixoPulsoOk ? "OK" : "FALHOU", pulsoZero);
  Serial.println(F("  base     OK"));
  Serial.println(F("Pode usar o braco. Tecla r recomeça os 5 passos."));
  linha();
  calibrando = false;
  ultimoUs = micros();
}

// Percentual de leituras I2C bem sucedidas desde a ultima chamada
int taxaOk(int i) {
  if (imu[i].leituras == 0) return imu[i].ok ? 100 : 0;
  int pct = 100 - (100L * imu[i].falhas) / imu[i].leituras;
  imu[i].leituras = 0;
  imu[i].falhas = 0;
  return pct;
}

void processarSerial() {
  if (!Serial.available()) return;
  char c = Serial.read();
  if (c == 'r' || c == 'R') calibrar();
}

// ---------------------------------------------------------------- setup
void setup() {
  Serial.begin(115200);
  delay(300);
  WiFi.mode(WIFI_OFF);

  Serial.println();
  Serial.println(F("=== KAIROS teste-mpu v5 — juntas desacopladas (Seel) ==="));
  Serial.println(F("Serial 115200. Calibracao: ENTER avanca cada pose."));
  Serial.println(F("Tecla r = recomeçar DEPOIS que a calibracao acabar."));

  for (int i = 0; i < NUM_MPU; i++) {
    imu[i].q = { 1, 0, 0, 0 };
    imu[i].eixo = { 0, 0, 1 };
    imu[i].gF = { 0, 0, 1 };
    imu[i].w = { 0, 0, 0 };
    imu[i].ok = iniciarSensor(i);
    Serial.printf("%-10s %s\n", NOME_MPU[i], imu[i].ok ? "OK" : "FALHOU");
  }

  configurarLoRa();
  calibrar();
}

// ---------------------------------------------------------------- loop
void loop() {
  processarSerial();

  unsigned long agora = micros();
  float dt = (agora - ultimoUs) / 1000000.0f;
  ultimoUs = agora;
  if (dt <= 0 || dt > 0.05f) dt = 0.01f;

  // ---- le e funde os 3 sensores
  for (int i = 0; i < NUM_MPU; i++) {
    if (!imu[i].ok) continue;
    Vec3 a, g;
    imu[i].leituras++;

    // MPU mudo devolve zeros. Em repouso ele SEMPRE le ~1 g, entao magnitude
    // nula e' dado invalido e nao uma leitura valida de queda livre.
    bool lido = lerSensor(i, a, g);
    float magA = sqrtf(a.x * a.x + a.y * a.y + a.z * a.z);
    if (!lido || magA < 0.1f) {
      imu[i].falhas++;
      if (imu[i].falhasSeguidas < 255) imu[i].falhasSeguidas++;
      if (imu[i].falhasSeguidas >= FALHAS_P_RECUPERAR &&
          millis() - imu[i].ultimaRecupMs > RECUP_INTERVALO_MS) {
        imu[i].ultimaRecupMs = millis();
        recuperarBarramento(i);
      }
      continue;
    }
    imu[i].falhasSeguidas = 0;

    Vec3 wBody = { g.x - imu[i].bx, g.y - imu[i].by, g.z - imu[i].bz };
    float magG = fabsf(wBody.x) + fabsf(wBody.y) + fabsf(wBody.z);
    imu[i].parado = (fabsf(magA - 1.0f) < REPOUSO_ACC && magG < REPOUSO_GYRO * 3);
    if (imu[i].parado) {
      imu[i].bx += BIAS_ADAPT * (g.x - imu[i].bx);
      imu[i].by += BIAS_ADAPT * (g.y - imu[i].by);
      imu[i].bz += BIAS_ADAPT * (g.z - imu[i].bz);
      wBody = { g.x - imu[i].bx, g.y - imu[i].by, g.z - imu[i].bz };
    }

    atualizarGravidade(imu[i], a, wBody, dt);
    mahony(imu[i], a, g, dt);
  }

  Imu& br = imu[IDX_BRACO];
  Imu& an = imu[IDX_ANTE];
  Imu& ma = imu[IDX_MAO];

  // ---- OMBRO: inclinacao do BRACO contra a gravidade (1 sensor, sem rumo)
  float gOmbAcc = anguloEntre(br.eixo, br.gF);
  if (ombroCalOk) {
    gOmbAcc = (gOmbAcc - zeroOmbro) * ganhoOmbro;
    if (gOmbAcc < 0) gOmbAcc = 0;
  }
  float wOmb = 0;
  Vec3 nElev = cross(br.eixo, br.gF);
  if (normalizar(nElev)) wOmb = dot(br.w, nElev);
  if (ombroCalOk) wOmb *= ganhoOmbro;

  // ---- BASE: azimute esferico SO do MPU do braco.
  // Giro em torno da vertical (os 3 votam se concordam = tronco).
  // Azimute do quaternion DESTE sensor puxa o zero quando o braco nao
  // esta vertical — e' o que distingue frente vs lado.
  float tB = dot(br.w, br.gF);
  float tA = an.ok ? dot(an.w, an.gF) : tB;
  float tM = ma.ok ? dot(ma.w, ma.gF) : tB;
  float tMin = tB, tMax = tB;
  if (tA < tMin) tMin = tA; if (tA > tMax) tMax = tA;
  if (tM < tMin) tMin = tM; if (tM > tMax) tMax = tM;
  bool consenso = (tMax - tMin) < CONSENSUS_MAX;
  float taxa = consenso ? mediana3(tB, tA, tM) : tB;
  taxa = constrain(taxa, -BASE_MAX_DEG_S, BASE_MAX_DEG_S);
  if (fabsf(taxa) > BASE_ZONA_MORTA)
    baseMemoria = wrap180(baseMemoria + taxa * dt);

  float elev = gOmbAcc;
  bool baseValida = (elev > ELEV_MIN_BASE && elev < ELEV_MAX_BASE);
  Vec3 uBr = qRotate(br.q, br.eixo);
  if (baseValida && mag3(uBr) > 0.2f) {
    float azRaw = atan2f(uBr.y, uBr.x) * 180.0f / PI;
    if (!azimuteTravado) {
      azimuteRef = azRaw;
      azimuteTravado = true;
      baseMemoria = 0;
      if (!avisouBaseTravada) {
        avisouBaseTravada = true;
        Serial.println();
        linha();
        Serial.println(F("PASSO 5 OK — zero da base e' a frente atual."));
        Serial.println(F("Pode usar o braco. Tecla r = calibrar de novo."));
        linha();
      }
    }
    float azAlvo = wrap180(azRaw - azimuteRef);
    float wAz = br.pesoAcc;   // so corrige com accel limpo
    if (wAz < 0) wAz = 0;
    baseMemoria = wrap180(baseMemoria + BASE_KP * wAz * wrap180(azAlvo - baseMemoria) * dt);
  }
  float gBase = baseMemoria;

  // ---- COTOVELO: Seel 2014 — pitch no plano da dobradica + ω·j
  float gCotAcc, wCot = 0;
  if (eixoCotOk) {
    float p1 = pitchHinge(br.gF, br.eixo, eixoCotBraco);
    float p2 = pitchHinge(an.gF, an.eixo, eixoCotAnte);
    gCotAcc = sinalCot * wrap180(wrap180(p2 - p1) - cotZero);
    wCot = sinalCot * (dot(an.w, eixoCotAnte) - dot(br.w, eixoCotBraco));
  } else {
    gCotAcc = anguloEntre(br.gF, an.gF);
    wCot = 0;
  }
  if (gCotAcc < -5) gCotAcc = -5;     // folga pequena; hiperextensao nao
  if (gCotAcc > 150) gCotAcc = 150;

  // ---- PULSO: PRONACAO (gira a palma) — o motor do robo e' esse eixo.
  // Velocidade = ω relativo no eixo longitudinal (Seel).
  // Absoluto: com antebraco inclinado, cada sensor ve "baixo" no plano
  // transversal; a diferenca dos angulos vs a pose neutra = torcao.
  // Flexao do pulso (tchauzinho) nao tem motor e e' ignorada.
  float wPul = sinalPulso * (dot(ma.w, ma.eixo) - dot(an.w, an.eixo));
  float gPulAcc = saidaInit ? saida[J_PULSO] : 0;
  float wAccPul = 0;
  if (an.parado && ma.parado) wPul = 0;

  if (eixoPulsoOk) {
    Vec3 ga = perpAoEixo(an.gF, an.eixo);
    Vec3 gm = perpAoEixo(ma.gF, ma.eixo);
    if (mag3(ga) > 0.35f && mag3(gm) > 0.35f &&
        normalizar(ga) && normalizar(gm)) {
      float angA = anguloEmTorno(eixoPulsoAnte, ga, an.eixo);
      float angM = anguloEmTorno(eixoPulsoMao,  gm, ma.eixo);
      gPulAcc = sinalPulso * wrap180(angM - angA - pulsoZero);
      wAccPul = an.pesoAcc * ma.pesoAcc;
    }
  }
  if (gPulAcc >  PULSO_ROM) gPulAcc =  PULSO_ROM;
  if (gPulAcc < -PULSO_ROM) gPulAcc = -PULSO_ROM;
  // Sem gravidade transversal: devolve devagar se estourou o ROM (bias do gyro)
  if (wAccPul < 0.05f && fabsf(saidaInit ? saida[J_PULSO] : 0) > PULSO_ROM * 0.9f) {
    float cur = saidaInit ? saida[J_PULSO] : 0;
    float excesso = cur - ((cur > 0) ? PULSO_ROM : -PULSO_ROM);
    gPulAcc = cur - PULSO_ROM_KP * excesso * 0.2f;
    wAccPul = 0.15f;
  }

  // Filtro complementar: gyro prediz, gravidade corrige (peso cai se |a| != 1g)
  float wAccCot = br.pesoAcc * an.pesoAcc;
  float wAccOmb = br.pesoAcc;
  if (!juntaInit) {
    saida[J_COTOVELO] = gCotAcc < 0 ? 0 : gCotAcc;
    saida[J_OMBRO]    = gOmbAcc;
    saida[J_PULSO]    = gPulAcc;
    saida[J_BASE]     = gBase;
    juntaInit = true;
  } else {
    saida[J_COTOVELO] += wCot * dt + JUNTA_KP * wAccCot * (gCotAcc - saida[J_COTOVELO]) * dt;
    if (saida[J_COTOVELO] < 0) saida[J_COTOVELO] = 0;
    if (saida[J_COTOVELO] > 150) saida[J_COTOVELO] = 150;
    saida[J_OMBRO]    += wOmb * dt + OMBRO_KP * wAccOmb * (gOmbAcc - saida[J_OMBRO]) * dt;
    if (saida[J_OMBRO] < 0) saida[J_OMBRO] = 0;
    saida[J_PULSO]    += wPul * dt + JUNTA_KP * wAccPul * wrap180(gPulAcc - saida[J_PULSO]) * dt;
    saida[J_PULSO]     = wrap180(saida[J_PULSO]);
    if (saida[J_PULSO] >  PULSO_ROM) saida[J_PULSO] =  PULSO_ROM;
    if (saida[J_PULSO] < -PULSO_ROM) saida[J_PULSO] = -PULSO_ROM;
  }

  if (!saidaInit) {
    saida[J_BASE] = gBase;
    saidaInit = true;
  } else {
    float e = wrap180(gBase - saida[J_BASE]);
    if (fabsf(e) > 0.4f)
      saida[J_BASE] = wrap180(saida[J_BASE] + 0.55f * e);
  }

  // ---- envia no ritmo do LoRa
  if (millis() - ultimoTxMs >= TX_PERIODO_MS) {
    ultimoTxMs = millis();

    int8_t c = paraByte(J_COTOVELO, saida[J_COTOVELO]);
    int8_t o = paraByte(J_OMBRO,    saida[J_OMBRO]);
    int8_t p = paraByte(J_PULSO,    saida[J_PULSO]);
    int8_t b = paraByte(J_BASE,     saida[J_BASE]);
    enviarLoRa(c, o, p, b);

    static uint8_t divPrint = 0;
    if (++divPrint >= 5) {
      divPrint = 0;
      int okB = taxaOk(IDX_BRACO), okA = taxaOk(IDX_ANTE), okM = taxaOk(IDX_MAO);
      Serial.printf("C=%4d O=%4d P=%4d B=%4d | elev=%5.0f %s | ok B/A/M=%3d/%3d/%3d%% | acc=%.2f/%.2f/%.2f\n",
                    c, o, p, b, elev, consenso ? "3imu" : "braco",
                    okB, okA, okM,
                    br.pesoAcc, an.pesoAcc, ma.pesoAcc);
      if (okB < 90 || okA < 90 || okM < 90)
        Serial.println("  !! SENSOR FALHANDO — confira os fios I2C");
    }
  }
}
