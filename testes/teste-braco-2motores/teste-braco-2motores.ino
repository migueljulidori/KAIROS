/*
 * Kairós — teste-braco-2motores (Arduino Uno RX)
 *
 * ESP32 com teste-mpu.ino (3 MPU) envia C,O,P,B via LoRa.
 * Dois motores do par movem em PARALELO — cada um no seu ritmo, sem esperar o outro.
 *
 * ESCOLHA O PAR (descomente UM):
 *   PAR_BASE_OMBRO      → Base + Ombro
 *   PAR_OMBRO_COTOVELO  → Ombro + Cotovelo
 *   PAR_COTOVELO_PULSO  → Cotovelo + Pulso
 *
 * Serial 9600 — m=teste | h=home | z=zero | s=para
 */

#include <SPI.h>
#include <LoRa.h>
#include <math.h>

#define EIXO_COTOVELO 0
#define EIXO_OMBRO    1
#define EIXO_PULSO    2
#define EIXO_BASE     3

#define PAR_BASE_OMBRO     0
#define PAR_OMBRO_COTOVELO 1
#define PAR_COTOVELO_PULSO 2

// ---- DESCOMENTE APENAS UM ----
#define PAR_ATIVO PAR_BASE_OMBRO
//#define PAR_ATIVO PAR_OMBRO_COTOVELO
// #define PAR_ATIVO PAR_COTOVELO_PULSO

const int NUM_EIXOS = 4;

const int PIN_STEP[NUM_EIXOS] = { 3, A1, A3, A5 };
const int PIN_DIR[NUM_EIXOS]  = { A0, A2, A4, 2 };

const char* NOME_EIXO[NUM_EIXOS] = {
  "Cotovelo", "Ombro", "Pulso", "Base"
};

const bool INV_DIR[NUM_EIXOS] = { false, false, false, false };

bool eixoHabilitado[NUM_EIXOS];
int parEixoA = -1;
int parEixoB = -1;

#define LORA_SS   10
#define LORA_RST  9
#define LORA_DIO0 -1

const long LORA_FREQ = 433E6;
const uint8_t LORA_HEADER = 0xC1;

const int PASSOS_MOTOR = 200;
const float REDUCAO = 38.4f;
const float PASSOS_POR_GRAU = (PASSOS_MOTOR * REDUCAO) / 360.0f;

// Velocidade — ombro mais lento = mais torque (sobretudo ao SUBIR)
const int DELAY_US = 1500;
const int DELAY_US_OMBRO = 3500;       // descer / geral
const int DELAY_US_OMBRO_SUBIR = 5500; // voltar para cima (contra gravidade)
const int STEP_HIGH_US = 80;
const int DEADZONE_GRAUS = 3;
const int PASSOS_POR_LOOP = 100;
const float FILTRO_ALVO = 0.82f;
const float SLEW_GRAUS = 8.0f;  // max mudança de alvo por pacote LoRa (~suave)

long posPassos[NUM_EIXOS];
long alvoPassos[NUM_EIXOS];
float alvoFiltradoGraus[NUM_EIXOS];
bool alvoFiltradoInit = false;

unsigned long ultimoPrintMs = 0;

void configurarParAtivo() {
  for (int i = 0; i < NUM_EIXOS; i++) eixoHabilitado[i] = false;
  parEixoA = -1;
  parEixoB = -1;

  switch (PAR_ATIVO) {
    case PAR_BASE_OMBRO:
      eixoHabilitado[EIXO_BASE] = true;
      eixoHabilitado[EIXO_OMBRO] = true;
      parEixoA = EIXO_BASE;
      parEixoB = EIXO_OMBRO;
      break;
    case PAR_OMBRO_COTOVELO:
      eixoHabilitado[EIXO_OMBRO] = true;
      eixoHabilitado[EIXO_COTOVELO] = true;
      parEixoA = EIXO_OMBRO;
      parEixoB = EIXO_COTOVELO;
      break;
    case PAR_COTOVELO_PULSO:
      eixoHabilitado[EIXO_COTOVELO] = true;
      eixoHabilitado[EIXO_PULSO] = true;
      parEixoA = EIXO_COTOVELO;
      parEixoB = EIXO_PULSO;
      break;
  }
}

const char* nomeParAtivo() {
  switch (PAR_ATIVO) {
    case PAR_BASE_OMBRO:     return "Base+Ombro";
    case PAR_OMBRO_COTOVELO: return "Ombro+Cotovelo";
    case PAR_COTOVELO_PULSO: return "Cotovelo+Pulso";
    default:                 return "?";
  }
}

long grausParaPassos(int graus) {
  return (long)(graus * PASSOS_POR_GRAU + (graus >= 0 ? 0.5f : -0.5f));
}

int passosParaGraus(long passos) {
  return (int)(passos / PASSOS_POR_GRAU + (passos >= 0 ? 0.5f : -0.5f));
}

int delayEixo(int eixo) {
  if (eixo != EIXO_OMBRO) return DELAY_US;
  // alvo < pos => voltando ao zero (braco para cima) — precisa mais torque
  long erro = alvoPassos[eixo] - posPassos[eixo];
  if (erro < 0) return DELAY_US_OMBRO_SUBIR;
  return DELAY_US_OMBRO;
}

void setDir(int eixo, bool frente) {
  bool dir = frente;
  if (INV_DIR[eixo]) dir = !dir;
  digitalWrite(PIN_DIR[eixo], dir ? HIGH : LOW);
}

bool eixoPrecisaMover(int eixo) {
  return eixoHabilitado[eixo] && posPassos[eixo] != alvoPassos[eixo];
}

// Um ciclo: todos os motores que precisam andar dao STEP no mesmo instante
bool passoParalelo() {
  bool algum = false;
  int maxDelay = 0;

  for (int i = 0; i < NUM_EIXOS; i++) {
    if (!eixoPrecisaMover(i)) continue;
    long erro = alvoPassos[i] - posPassos[i];
    setDir(i, erro > 0);
    algum = true;
    int d = delayEixo(i);
    if (d > maxDelay) maxDelay = d;
  }

  if (!algum) return false;

  delayMicroseconds(15);

  for (int i = 0; i < NUM_EIXOS; i++) {
    if (!eixoPrecisaMover(i)) continue;
    digitalWrite(PIN_STEP[i], HIGH);
  }

  delayMicroseconds(STEP_HIGH_US);

  for (int i = 0; i < NUM_EIXOS; i++) {
    if (!eixoPrecisaMover(i)) continue;
    digitalWrite(PIN_STEP[i], LOW);
    long erro = alvoPassos[i] - posPassos[i];
    posPassos[i] += (erro > 0) ? 1 : -1;
  }

  delayMicroseconds(maxDelay);
  return true;
}

bool loraOk = false;

void configurarLoRa() {
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println("LoRa FALHOU — use m/h/z/s no Serial");
    loraOk = false;
    return;
  }
  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(250E3);
  LoRa.setCodingRate4(5);
  LoRa.enableCrc();
  loraOk = true;
  Serial.println("LoRa RX OK");
}

void receberLoRa() {
  if (!loraOk) return;

  int packetSize = LoRa.parsePacket();
  if (packetSize < 5) return;

  uint8_t header = LoRa.read();
  if (header != LORA_HEADER) {
    while (LoRa.available()) LoRa.read();
    return;
  }

  int graus[NUM_EIXOS];
  for (int i = 0; i < NUM_EIXOS; i++) {
    graus[i] = (int)(int8_t)LoRa.read();
  }

  for (int i = 0; i < NUM_EIXOS; i++) {
    if (!eixoHabilitado[i]) continue;

    if (!alvoFiltradoInit) {
      alvoFiltradoGraus[i] = graus[i];
      continue;
    }

    float alvo = (float)graus[i];
    float diff = alvo - alvoFiltradoGraus[i];
    if (fabs(diff) < DEADZONE_GRAUS) {
      // ainda permite voltar ao zero lentamente
      if (fabs(alvo) < DEADZONE_GRAUS && fabs(alvoFiltradoGraus[i]) > 0.5f) {
        alvoFiltradoGraus[i] *= 0.9f;
      }
      continue;
    }

    // slew: não salta de uma vez
    if (diff > SLEW_GRAUS) alvo = alvoFiltradoGraus[i] + SLEW_GRAUS;
    if (diff < -SLEW_GRAUS) alvo = alvoFiltradoGraus[i] - SLEW_GRAUS;

    alvoFiltradoGraus[i] = FILTRO_ALVO * alvoFiltradoGraus[i]
                         + (1.0f - FILTRO_ALVO) * alvo;
  }

  if (!alvoFiltradoInit) alvoFiltradoInit = true;

  for (int i = 0; i < NUM_EIXOS; i++) {
    if (!eixoHabilitado[i]) continue;
    long novoAlvo = grausParaPassos((int)(alvoFiltradoGraus[i] + (alvoFiltradoGraus[i] >= 0 ? 0.5f : -0.5f)));
    if (novoAlvo != alvoPassos[i]) {
      alvoPassos[i] = novoAlvo;
    }
  }

  unsigned long agora = millis();
  if (agora - ultimoPrintMs >= 300) {
    ultimoPrintMs = agora;
    Serial.print("RX C=");
    Serial.print(graus[0]);
    Serial.print(" O=");
    Serial.print(graus[1]);
    Serial.print(" P=");
    Serial.print(graus[2]);
    Serial.print(" B=");
    Serial.print(graus[3]);
    Serial.print(" | ");
    Serial.print(nomeParAtivo());
    Serial.print(" posA=");
    Serial.print(passosParaGraus(posPassos[parEixoA]));
    Serial.print(" posB=");
    Serial.println(passosParaGraus(posPassos[parEixoB]));
  }
}

void seguirAlvos() {
  for (int n = 0; n < PASSOS_POR_LOOP; n++) {
    if (!passoParalelo()) break;
  }
}

bool parEmMovimento() {
  return eixoPrecisaMover(parEixoA) || eixoPrecisaMover(parEixoB);
}

void testarParAtivo() {
  const int PASSOS_TESTE = 50;
  Serial.println("Teste paralelo (2 motores independentes)...");
  alvoPassos[parEixoA] = PASSOS_TESTE;
  alvoPassos[parEixoB] = PASSOS_TESTE;
  while (parEmMovimento()) seguirAlvos();
  delay(300);
  alvoPassos[parEixoA] = 0;
  alvoPassos[parEixoB] = 0;
  while (parEmMovimento()) seguirAlvos();
  Serial.println("Teste concluido");
}

void processarComando(char c) {
  switch (c) {
    case 's': case 'S':
      for (int i = 0; i < NUM_EIXOS; i++) {
        if (eixoHabilitado[i]) alvoPassos[i] = posPassos[i];
      }
      Serial.println("Parado");
      break;
    case 'm': case 'M':
      testarParAtivo();
      break;
    case 'h': case 'H':
      for (int i = 0; i < NUM_EIXOS; i++) {
        if (eixoHabilitado[i]) alvoPassos[i] = 0;
      }
      Serial.println("Home");
      break;
    case 'z': case 'Z':
      for (int i = 0; i < NUM_EIXOS; i++) {
        if (eixoHabilitado[i]) {
          posPassos[i] = 0;
          alvoPassos[i] = 0;
          alvoFiltradoGraus[i] = 0;
        }
      }
      alvoFiltradoInit = false;
      Serial.println("Zero");
      break;
  }
}

void setup() {
  Serial.begin(9600);
  configurarParAtivo();

  for (int i = 0; i < NUM_EIXOS; i++) {
    pinMode(PIN_STEP[i], OUTPUT);
    pinMode(PIN_DIR[i], OUTPUT);
    digitalWrite(PIN_STEP[i], LOW);
    digitalWrite(PIN_DIR[i], LOW);
    posPassos[i] = 0;
    alvoPassos[i] = 0;
  }

  configurarLoRa();

  Serial.println("=== KAIROS teste-braco-2motores (paralelo) ===");
  Serial.print("Par: ");
  Serial.println(nomeParAtivo());
  Serial.println("2 motores independentes — comecam juntos, cada um no seu ritmo");
  Serial.println("m=teste | h=home | z=zero | s=para");
}

void loop() {
  if (Serial.available()) {
    processarComando((char)Serial.read());
  }

  receberLoRa();
  seguirAlvos();
}
