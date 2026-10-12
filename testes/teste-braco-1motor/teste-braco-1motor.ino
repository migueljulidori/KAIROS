/*
 * Kairós — teste-braco-1motor (Arduino Uno RX)
 *
 * ESP32 com teste-mpu.ino (3 MPU) envia C,O,P,B via LoRa.
 * Este sketch move APENAS UM motor por vez — escolha abaixo.
 *
 * ============================================================
 * ESCOLHA O EIXO (descomente UM, comente os outros)
 * ============================================================
 *
 *   EIXO_BASE      → gira braco horizontal     (TX: B)  mova: base
 *   EIXO_OMBRO     → levanta/abaixa braco      (TX: O)  mova: braco reto
 *   EIXO_COTOVELO  → dobra cotovelo             (TX: C)  mova: so cotovelo
 *   EIXO_PULSO     → gira mao                   (TX: P)  mova: pulso
 *
 * ============================================================
 * ROTEIRO DE TESTE (ESP32 + este sketch)
 * ============================================================
 *
 *  1) EIXO_BASE     → calibra zero → gira braco → so Base mexe
 *  2) EIXO_OMBRO    → levanta/abaixa braco (cotovelo reto)
 *  3) EIXO_COTOVELO → dobra so cotovelo
 *  4) EIXO_PULSO    → gira so mao
 *
 * Depois use teste-braco-2motores.ino para pares de eixos.
 *
 * Ligacoes iguais a teste-braco-4motores (LoRa, A4988, pinos).
 * Serial 9600 — h=home | z=zero | s=para | m=teste eixo ativo
 */

#include <SPI.h>
#include <LoRa.h>

#define EIXO_COTOVELO 0
#define EIXO_OMBRO    1
#define EIXO_PULSO    2
#define EIXO_BASE     3

// ---- DESCOMENTE APENAS UM ----
// #define EIXO_ATIVO EIXO_BASE
 #define EIXO_ATIVO EIXO_OMBRO
// #define EIXO_ATIVO EIXO_COTOVELO
// #define EIXO_ATIVO EIXO_PULSO

const int NUM_EIXOS = 4;

const int PIN_STEP[NUM_EIXOS] = { 3, A1, A3, A5 };
const int PIN_DIR[NUM_EIXOS]  = { A0, A2, A4, 2 };

const char* NOME_EIXO[NUM_EIXOS] = {
  "Cotovelo", "Ombro", "Pulso", "Base"
};

const char LETRA_EIXO[NUM_EIXOS] = { 'C', 'O', 'P', 'B' };

const bool INV_DIR[NUM_EIXOS] = { false, false, false, false };

bool eixoHabilitado[NUM_EIXOS];

#define LORA_SS   10
#define LORA_RST  9
#define LORA_DIO0 -1

const long LORA_FREQ = 433E6;
const uint8_t LORA_HEADER = 0xC1;

const int PASSOS_MOTOR = 200;
const float REDUCAO = 38.4f;
const float PASSOS_POR_GRAU = (PASSOS_MOTOR * REDUCAO) / 360.0f;

const int DELAY_US = 3500;
const int DELAY_US_OMBRO = 6000;
const int STEP_HIGH_US = 100;
const int DEADZONE_GRAUS = 3;

long posPassos[NUM_EIXOS];
long alvoPassos[NUM_EIXOS];

int eixoManual = 0;
long passosManuais = 0;
bool modoManual = false;

long grausParaPassos(int graus) {
  return (long)(graus * PASSOS_POR_GRAU + (graus >= 0 ? 0.5f : -0.5f));
}

int passosParaGraus(long passos) {
  return (int)(passos / PASSOS_POR_GRAU + (passos >= 0 ? 0.5f : -0.5f));
}

void darPasso(int eixo, bool frente) {
  if (!eixoHabilitado[eixo]) return;
  bool dir = frente;
  if (INV_DIR[eixo]) dir = !dir;
  digitalWrite(PIN_DIR[eixo], dir ? HIGH : LOW);
  delayMicroseconds(20);
  digitalWrite(PIN_STEP[eixo], HIGH);
  delayMicroseconds(STEP_HIGH_US);
  digitalWrite(PIN_STEP[eixo], LOW);
  delayMicroseconds(eixo == EIXO_OMBRO ? DELAY_US_OMBRO : DELAY_US);
}

bool loraOk = false;

void configurarLoRa() {
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println("LoRa FALHOU — seguindo so com Serial (f/t/s/m)");
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
  if (!loraOk || modoManual) return;

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
    long novoAlvo = grausParaPassos(graus[i]);
    int diffGraus = abs(passosParaGraus(novoAlvo - posPassos[i]));
    if (diffGraus < DEADZONE_GRAUS) continue;
    alvoPassos[i] = novoAlvo;
  }

  Serial.print("RX C=");
  Serial.print(graus[0]);
  Serial.print(" O=");
  Serial.print(graus[1]);
  Serial.print(" P=");
  Serial.print(graus[2]);
  Serial.print(" B=");
  Serial.print(graus[3]);
  Serial.print(" | ativo ");
  Serial.print(LETRA_EIXO[EIXO_ATIVO]);
  Serial.print(" alvoG=");
  Serial.print(passosParaGraus(alvoPassos[EIXO_ATIVO]));
  Serial.print(" RSSI=");
  Serial.println(LoRa.packetRssi());
}

void seguirAlvos() {
  int i = EIXO_ATIVO;
  if (!eixoHabilitado[i]) return;

  long erro = alvoPassos[i] - posPassos[i];
  if (erro == 0) return;

  if (erro > 0) {
    darPasso(i, true);
    posPassos[i]++;
  } else {
    darPasso(i, false);
    posPassos[i]--;
  }
}

void testarMotorAtivo() {
  int i = EIXO_ATIVO;
  const int PASSOS_TESTE = 40;
  Serial.print("Teste ");
  Serial.println(NOME_EIXO[i]);
  modoManual = false;

  for (int p = 0; p < PASSOS_TESTE; p++) {
    darPasso(i, true);
    posPassos[i]++;
    alvoPassos[i] = posPassos[i];
  }
  delay(500);
  for (int p = 0; p < PASSOS_TESTE; p++) {
    darPasso(i, false);
    posPassos[i]--;
    alvoPassos[i] = posPassos[i];
  }
  Serial.println("Teste concluido");
}

void processarComando(char c) {
  switch (c) {
    case 'f': case 'F':
      modoManual = true;
      passosManuais = -1;
      eixoManual = EIXO_ATIVO;
      Serial.println("Manual frente");
      break;
    case 't': case 'T':
      modoManual = true;
      passosManuais = -2;
      eixoManual = EIXO_ATIVO;
      Serial.println("Manual tras");
      break;
    case 's': case 'S':
      modoManual = false;
      passosManuais = 0;
      alvoPassos[EIXO_ATIVO] = posPassos[EIXO_ATIVO];
      Serial.println("Parado");
      break;
    case 'm': case 'M':
      testarMotorAtivo();
      break;
    case 'h': case 'H':
      modoManual = false;
      alvoPassos[EIXO_ATIVO] = 0;
      Serial.println("Home eixo ativo");
      break;
    case 'z': case 'Z':
      posPassos[EIXO_ATIVO] = 0;
      alvoPassos[EIXO_ATIVO] = 0;
      Serial.println("Zero eixo ativo");
      break;
  }
}

void setup() {
  Serial.begin(9600);

  for (int i = 0; i < NUM_EIXOS; i++) {
    eixoHabilitado[i] = (i == EIXO_ATIVO);
    pinMode(PIN_STEP[i], OUTPUT);
    pinMode(PIN_DIR[i], OUTPUT);
    digitalWrite(PIN_STEP[i], LOW);
    digitalWrite(PIN_DIR[i], LOW);
    posPassos[i] = 0;
    alvoPassos[i] = 0;
  }

  configurarLoRa();

  Serial.println("=== KAIROS teste-braco-1motor ===");
  Serial.print("Eixo ATIVO: ");
  Serial.print(NOME_EIXO[EIXO_ATIVO]);
  Serial.print(" (");
  Serial.print(LETRA_EIXO[EIXO_ATIVO]);
  Serial.println(" no pacote LoRa)");
  Serial.println("ESP32: teste-mpu.ino | Zero = braco esticado PARA CIMA");
  Serial.println("Comandos: m=teste | f/t/s | h=home | z=zero");
}

void loop() {
  if (Serial.available()) {
    processarComando((char)Serial.read());
  }

  receberLoRa();

  if (modoManual) {
    int i = EIXO_ATIVO;
    if (passosManuais == -1) {
      darPasso(i, true);
      posPassos[i]++;
      alvoPassos[i] = posPassos[i];
    } else if (passosManuais == -2) {
      darPasso(i, false);
      posPassos[i]--;
      alvoPassos[i] = posPassos[i];
    }
  } else {
    seguirAlvos();
  }
}
