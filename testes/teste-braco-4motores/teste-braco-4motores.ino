/*
 * Kairós — teste-braco-4motores v3 (Arduino Uno RX)
 * ============================================================================
 *
 * O QUE MUDOU E POR QUE
 *
 *  Problema 1 — TRAVAMENTO:
 *    a versao anterior dava ate 80 passos dentro de delayMicroseconds(),
 *    bloqueando o loop por 153 ms (eixos normais) a 329 ms (ombro).
 *    Durante esse tempo o LoRa nao era lido e os pacotes se perdiam.
 *    Agora NAO existe delay bloqueante: os passos sao agendados por micros()
 *    e o LoRa e' lido em TODA volta do loop.
 *
 *  Problema 2 — LENTIDAO:
 *    com delay fixo o ombro fazia 11 graus/s e os demais 24 graus/s.
 *    Um braco humano faz 100+ graus/s, entao o robo nunca acompanhava.
 *    Motor de passo partindo direto trava acima de ~400 passos/s, mas COM
 *    RAMPA DE ACELERACAO chega a 1000-2000 passos/s em full step.
 *    Implementado perfil de velocidade com aceleracao limitada:
 *      ombro 42 graus/s, base 56, cotovelo 66, pulso 75.
 *
 *  Problema 3 — TORQUE DO OMBRO:
 *    em vez de um delay fixo maior, o ombro agora tem vmax/aceleracao
 *    proprios e frenagem calculada (v = sqrt(2*a*erro)), entao ele nunca
 *    e' mandado parar de repente — que era o que fazia perder passos.
 *
 * ============================================================================
 * CINEMATICA — o ESP32 envia, em graus, com zero = braco ESTICADO PARA CIMA:
 *   COTOVELO = flexao (0 = esticado)
 *   OMBRO    = elevacao a partir da vertical (0 = para cima)
 *   PULSO    = torcao da palma (pronacao/supinacao)
 *   BASE     = azimute (giro em torno da vertical; inclui abrir o braco pro lado)
 *
 * Reducao 38.4:1 -> 21.333 passos de motor por grau de junta (full step).
 * 90 graus de junta = 1920 passos.
 * ============================================================================
 *
 * LIGACOES
 *   LoRa: VCC 3V3, GND, SCK 13, MISO 12, MOSI 11, NSS 10, RST 9, DIO0 solto
 *   A4988 (ENABLE -> GND, RESET+SLEEP -> 5V):
 *     Cotovelo  STEP D3  DIR A0   (medio)
 *     Ombro     STEP A1  DIR A2   (grande)
 *     Pulso     STEP A3  DIR A4   (medio)
 *     Base      STEP A5  DIR D2   (grande)
 *
 * Serial 9600:
 *   1/2/3/4 ou c/o/p/b = escolhe eixo | f/t = frente/tras | s = parar
 *   m = teste dos motores | h = home (zero) | z = redefinir zero aqui
 *   + / - = aumenta/diminui velocidade global 10%
 * ============================================================================
 */

#include <SPI.h>
#include <LoRa.h>
#include <math.h>

const int NUM_EIXOS = 4;
#define EIXO_COTOVELO 0
#define EIXO_OMBRO    1
#define EIXO_PULSO    2
#define EIXO_BASE     3

const int PIN_STEP[NUM_EIXOS] = { 3, A1, A3, A5 };
const int PIN_DIR[NUM_EIXOS]  = { A0, A2, A4, 2 };

const char* NOME_EIXO[NUM_EIXOS] = { "Cotovelo", "Ombro", "Pulso", "Base" };

// true = inverte o sentido desse eixo
const bool INV_DIR[NUM_EIXOS] = { false, false, false, false };

#define LORA_SS   10
#define LORA_RST  9
#define LORA_DIO0 -1
const long LORA_FREQ = 433E6;
const uint8_t LORA_HEADER = 0xC1;

// ----------------------------------------------------------------- mecanica
const int   PASSOS_MOTOR = 200;                                  // full step
const float REDUCAO = 38.4f;                                     // planetaria
const float PASSOS_POR_GRAU = (PASSOS_MOTOR * REDUCAO) / 360.0f; // = 21.333

// Velocidade maxima (passos/s) e aceleracao (passos/s^2) por eixo.
//
// DIMENSIONADO PARA FONTE DE 12 V. O torque de um motor de passo despenca
// depois do "joelho" da curva, em V/(2*pi*L*I) ~ 546 passos/s com 12 V.
// Estes valores ficam em ~70-90% do joelho, que e' a regiao com torque cheio.
// Os valores anteriores (900-1600) estavam 2 a 3 vezes acima disso: dai o
// movimento rapido demais e a perda de passos.
//
// Com fonte de 24 V o joelho dobra (~1114 passos/s): basta apertar '+' ate
// escalaVel = 2.0, sem recompilar.
//                                    Cotovelo  Ombro   Pulso    Base
const float VMAX[NUM_EIXOS]  = {         500,    380,    550,    450 };
const float ACEL[NUM_EIXOS]  = {        2000,   1500,   2200,   1800 };

// Limites de curso da junta (graus) — protege a mecanica
const float LIM_MIN[NUM_EIXOS] = {   0,   0, -110, -150 };
const float LIM_MAX[NUM_EIXOS] = { 150, 130,  110,  150 };

const float DEADZONE_GRAUS = 1.5f;
const unsigned long TIMEOUT_LORA_MS = 800;   // sem pacote -> congela o alvo
const unsigned long PERIODO_CTRL_US = 2000;  // malha de velocidade a 500 Hz

// ----------------------------------------------------------------- estado
long  pos[NUM_EIXOS];          // posicao atual (passos)
long  alvo[NUM_EIXOS];         // alvo (passos)
float vel[NUM_EIXOS];          // velocidade atual com sinal (passos/s)
unsigned long proxStepUs[NUM_EIXOS];
unsigned long intervaloUs[NUM_EIXOS];
float escalaVel = 1.0f;

unsigned long ultimoCtrlUs = 0;
unsigned long ultimoPacoteMs = 0;
unsigned long ultimoPrintMs = 0;

int  eixoManual = 0;
int  dirManual = 0;            // 0 parado, +1 frente, -1 tras
bool modoManual = false;
bool loraOk = false;
long pacotesOk = 0;

// ----------------------------------------------------------------- util
long grausParaPassos(float graus) {
  return (long)(graus * PASSOS_POR_GRAU + (graus >= 0 ? 0.5f : -0.5f));
}

float passosParaGraus(long passos) {
  return passos / PASSOS_POR_GRAU;
}

inline void pulsoStep(int i, bool frente) {
  bool d = INV_DIR[i] ? !frente : frente;
  digitalWrite(PIN_DIR[i], d ? HIGH : LOW);
  delayMicroseconds(2);              // DIR setup (A4988 pede 200 ns)
  digitalWrite(PIN_STEP[i], HIGH);
  delayMicroseconds(3);              // STEP high minimo do A4988 = 1 us
  digitalWrite(PIN_STEP[i], LOW);
  pos[i] += frente ? 1 : -1;
}

// ----------------------------------------------------------------- controle
/*
 * Malha de velocidade com aceleracao limitada.
 * Para cada eixo:
 *   vParar = sqrt(2 * a * |erro|)   -> velocidade maxima que ainda permite
 *                                      frear a tempo sem estourar o alvo
 *   vDes   = sinal(erro) * min(vmax, vParar)
 *   vel converge para vDes respeitando a aceleracao
 * Isso da uma rampa trapezoidal mesmo com o alvo mudando o tempo todo,
 * que e' exatamente o caso da teleoperacao.
 */
void atualizarControle(float dt) {
  if (dt <= 0 || dt > 0.05f) dt = 0.002f;
  for (int i = 0; i < NUM_EIXOS; i++) {
    long erro;
    if (modoManual && i == eixoManual && dirManual != 0) {
      // manual: empurra o alvo a frente para o eixo andar continuamente
      alvo[i] = pos[i] + dirManual * 100000L;
      erro = alvo[i] - pos[i];
    } else {
      erro = alvo[i] - pos[i];
    }

    float vDes;
    if (erro == 0) {
      vDes = 0;
    } else {
      float vParar = sqrtf(2.0f * ACEL[i] * fabsf((float)erro));
      float v = VMAX[i] * escalaVel;
      if (vParar < v) v = vParar;
      vDes = (erro > 0) ? v : -v;
    }

    float dv = ACEL[i] * dt;
    if (vel[i] < vDes)      vel[i] = min(vel[i] + dv, vDes);
    else if (vel[i] > vDes) vel[i] = max(vel[i] - dv, vDes);

    float mag = fabsf(vel[i]);
    if (mag < 20.0f) {
      intervaloUs[i] = 0;            // praticamente parado
      if (erro == 0) vel[i] = 0;
      else intervaloUs[i] = 50000;   // arranque lento (20 passos/s)
    } else {
      intervaloUs[i] = (unsigned long)(1000000.0f / mag);
    }
  }
}

void executarPassos(unsigned long agora) {
  for (int i = 0; i < NUM_EIXOS; i++) {
    if (intervaloUs[i] == 0) continue;
    if ((long)(agora - proxStepUs[i]) < 0) continue;

    // Em movimento o sentido vem da VELOCIDADE, nao do erro: se o alvo for
    // cruzado a alta velocidade o eixo desacelera e volta, em vez de parar
    // de repente (parada brusca = passo perdido).
    bool frente;
    if (fabsf(vel[i]) >= 20.0f) frente = (vel[i] > 0);
    else {
      long erro = alvo[i] - pos[i];
      if (erro == 0) continue;
      frente = (erro > 0);
    }

    pulsoStep(i, frente);

    // reagenda a partir de agora (evita acumular atraso apos um pico de carga)
    proxStepUs[i] = agora + intervaloUs[i];
  }
}

// ----------------------------------------------------------------- LoRa
void configurarLoRa() {
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println(F("LoRa FALHOU — so modo manual"));
    loraOk = false;
    return;
  }
  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(250E3);
  LoRa.setCodingRate4(5);
  LoRa.enableCrc();
  LoRa.receive();
  loraOk = true;
  Serial.println(F("LoRa RX OK"));
}

void receberLoRa() {
  if (!loraOk) return;
  int tam = LoRa.parsePacket();
  if (tam < 5) return;

  uint8_t buf[5];
  for (int i = 0; i < 5 && LoRa.available(); i++) buf[i] = LoRa.read();
  while (LoRa.available()) LoRa.read();
  if (buf[0] != LORA_HEADER) return;

  ultimoPacoteMs = millis();
  pacotesOk++;
  if (modoManual) return;

  float graus[NUM_EIXOS];
  graus[EIXO_COTOVELO] = (int8_t)buf[1];
  graus[EIXO_OMBRO]    = (int8_t)buf[2];
  graus[EIXO_PULSO]    = (int8_t)buf[3];
  graus[EIXO_BASE]     = (int8_t)buf[4];

  static float alvoG[NUM_EIXOS] = { 0, 0, 0, 0 };
  for (int i = 0; i < NUM_EIXOS; i++) {
    float g = constrain(graus[i], LIM_MIN[i], LIM_MAX[i]);
    alvoG[i] += 0.40f * (g - alvoG[i]);
    long novo = grausParaPassos(alvoG[i]);
    if (labs(novo - alvo[i]) >= grausParaPassos(DEADZONE_GRAUS))
      alvo[i] = novo;
  }
}

// ----------------------------------------------------------------- comandos
void testarMotores() {
  Serial.println(F("=== Teste: 200 passos ida/volta por eixo ==="));
  modoManual = false;
  dirManual = 0;
  for (int i = 0; i < NUM_EIXOS; i++) {
    Serial.print(F("  ")); Serial.println(NOME_EIXO[i]);
    alvo[i] = pos[i] + 200;
    unsigned long t = millis();
    while (pos[i] != alvo[i] && millis() - t < 4000) {
      unsigned long ag = micros();
      atualizarControle((ag - ultimoCtrlUs) / 1000000.0f);
      ultimoCtrlUs = ag;
      executarPassos(ag);
    }
    delay(250);
    alvo[i] = pos[i] - 200;
    t = millis();
    while (pos[i] != alvo[i] && millis() - t < 4000) {
      unsigned long ag = micros();
      atualizarControle((ag - ultimoCtrlUs) / 1000000.0f);
      ultimoCtrlUs = ag;
      executarPassos(ag);
    }
    delay(250);
  }
  Serial.println(F("Teste concluido"));
}

void pararTudo() {
  modoManual = false;
  dirManual = 0;
  for (int i = 0; i < NUM_EIXOS; i++) alvo[i] = pos[i];
}

void processarComando(char c) {
  switch (c) {
    case '1': case 'c': case 'C': eixoManual = EIXO_COTOVELO; modoManual = true; dirManual = 0; break;
    case '2': case 'o': case 'O': eixoManual = EIXO_OMBRO;    modoManual = true; dirManual = 0; break;
    case '3': case 'p': case 'P': eixoManual = EIXO_PULSO;    modoManual = true; dirManual = 0; break;
    case '4': case 'b': case 'B': eixoManual = EIXO_BASE;     modoManual = true; dirManual = 0; break;

    case 'f': case 'F': modoManual = true; dirManual = +1; break;
    case 't': case 'T': modoManual = true; dirManual = -1; break;
    case 's': case 'S': pararTudo(); Serial.println(F("Parado")); return;

    case 'm': case 'M': testarMotores(); return;

    case 'h': case 'H':
      modoManual = false; dirManual = 0;
      for (int i = 0; i < NUM_EIXOS; i++) alvo[i] = 0;
      Serial.println(F("Home — voltando ao zero (esticado para cima)"));
      return;

    case 'z': case 'Z':
      for (int i = 0; i < NUM_EIXOS; i++) { pos[i] = 0; alvo[i] = 0; vel[i] = 0; }
      Serial.println(F("Zero mecanico redefinido na pose atual"));
      return;

    case '+': escalaVel = min(escalaVel + 0.1f, 2.0f); break;
    case '-': escalaVel = max(escalaVel - 0.1f, 0.2f); break;

    default: return;
  }
  Serial.print(F("Eixo ")); Serial.print(NOME_EIXO[eixoManual]);
  Serial.print(F("  dir=")); Serial.print(dirManual);
  Serial.print(F("  vel x")); Serial.println(escalaVel);
}

// ----------------------------------------------------------------- setup
void setup() {
  Serial.begin(9600);

  for (int i = 0; i < NUM_EIXOS; i++) {
    pinMode(PIN_STEP[i], OUTPUT);
    pinMode(PIN_DIR[i], OUTPUT);
    digitalWrite(PIN_STEP[i], LOW);
    digitalWrite(PIN_DIR[i], LOW);
    pos[i] = 0; alvo[i] = 0; vel[i] = 0;
    proxStepUs[i] = 0; intervaloUs[i] = 0;
  }

  Serial.println();
  Serial.println(F("=== KAIROS 4 motores v3 — passo nao bloqueante + rampa ==="));
  Serial.print(F("passos/grau = ")); Serial.println(PASSOS_POR_GRAU);
  Serial.println(F("Pose inicial: braco ESTICADO PARA CIMA"));
  Serial.println(F("c/o/p/b + f/t/s | m teste | h home | z zero | +/- velocidade"));

  configurarLoRa();
  ultimoCtrlUs = micros();
}

// ----------------------------------------------------------------- loop
void loop() {
  // LoRa lido em toda volta — o loop nunca bloqueia mais que ~5 us
  receberLoRa();

  if (Serial.available()) processarComando(Serial.read());

  unsigned long agora = micros();

  if ((unsigned long)(agora - ultimoCtrlUs) >= PERIODO_CTRL_US) {
    float dt = (agora - ultimoCtrlUs) / 1000000.0f;
    ultimoCtrlUs = agora;
    if (dt > 0.05f) dt = 0.05f;

    // perdeu o sinal: congela o alvo onde esta (nao solta o braco)
    if (loraOk && !modoManual && millis() - ultimoPacoteMs > TIMEOUT_LORA_MS) {
      for (int i = 0; i < NUM_EIXOS; i++) alvo[i] = pos[i];
    }

    atualizarControle(dt);
  }

  executarPassos(agora);

  if (millis() - ultimoPrintMs >= 500) {
    ultimoPrintMs = millis();
    Serial.print(F("C=")); Serial.print(passosParaGraus(pos[0]), 0);
    Serial.print(F(" O=")); Serial.print(passosParaGraus(pos[1]), 0);
    Serial.print(F(" P=")); Serial.print(passosParaGraus(pos[2]), 0);
    Serial.print(F(" B=")); Serial.print(passosParaGraus(pos[3]), 0);
    Serial.print(F(" | pkt=")); Serial.print(pacotesOk);
    Serial.print(F(" | vel x")); Serial.println(escalaVel);
  }
}
