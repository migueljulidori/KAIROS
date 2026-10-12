/*
 * Kairós — kairos-receptor (Arduino Uno RX unificado)
 *   4x NEMA17 (braco) + 5x servo MG996R (dedos) + LoRa
 *
 * Backup intacto: teste-braco-4motores/  e  receptor-uno/
 *
 * Pacote LoRa 11 bytes (mesmo do kairos-luva-tx):
 *   [0xC2, cotovelo, ombro, pulso_lo, pulso_hi, base, d0, d1, d2, d3, d4]
 *   cotovelo, ombro e base = int8 escalado (fundo 150, 130 e 150 graus)
 *   pulso = int16 graus (0 na partida, ate -180 para tras)
 *   dedos = uint8 0 (aberto) ou 180 (fechado)
 * Teste de conexao (luva, antes da calibracao e se o link cair):
 *   recebe [0xA1, seq] -> responde [0xA2, seq, rssi]
 * Manual vindo da luva: [0xA4, tecla]. 0x01 entra, 0x00 sai, 0x02 home, 0x03 sai do home.
 *   No modo manual o pacote 0xC2 continua atualizando os dedos,
 *   e nao mexe nas juntas ate a luva sair do modo.
 *
 * LIGACOES
 *   LoRa: VCC 3V3, GND, SCK 13, MISO 12, MOSI 11, NSS 10, RST 9, DIO0 solto
 *   A4988 (ENABLE->GND, RESET+SLEEP->5V):
 *     Cotovelo  STEP D3  DIR A0
 *     Ombro     STEP A1  DIR A2
 *     Pulso     STEP A3  DIR A4
 *     Base      STEP A5  DIR D2
 *   Servos MG996R (sinal) — pins livres (nao batem com LoRa/SPI/step):
 *     D1:D8  D2:D4  D3:D5  D4:D6  D5:D7
 *     +5V fonte externa, GND comum com o Uno
 *
 * Serial 115200:
 *   1/2/3/4 ou c/o/p/b = eixo | f/t = frente/tras | s = parar
 *   m = teste motores + servos | h = home do boot | z nao redefine o home | +/- velocidade
 * Home: sempre o ponto em que o Arduino ligou. A volta nao fica lenta: em
 * passo inteiro, 100–300 passos/s e' a ressonancia do NEMA17 e ele perde passo.
 * Cotovelo, ombro e base no fio sao int8 escalados (fundo 150, 130 e 150).
 * ============================================================================
 */

#include <SPI.h>
#include <LoRa.h>
#include <Servo.h>
#include <math.h>

const int NUM_EIXOS = 4;
#define EIXO_COTOVELO 0
#define EIXO_OMBRO    1
#define EIXO_PULSO    2
#define EIXO_BASE     3

const int NUM_FLEX = 5;

const int PIN_STEP[NUM_EIXOS] = { 3, A1, A3, A5 };
const int PIN_DIR[NUM_EIXOS]  = { A0, A2, A4, 2 };
const int PIN_SERVO[NUM_FLEX] = { 8, 4, 5, 6, 7 };

const char* NOME_EIXO[NUM_EIXOS] = { "Cotovelo", "Ombro", "Pulso", "Base" };
// Positivo (DIR "frente") ja e' o sentido mecanico:
//   cotovelo = antebraco para tras | ombro = braco para a frente
//   pulso = esquerda no positivo, tras no negativo (ate -180) | base = esquerda
const bool INV_DIR[NUM_EIXOS] = { false, false, false, false };

#define LORA_SS   10
#define LORA_RST  9
#define LORA_DIO0 -1
const long LORA_FREQ = 400E6;   // igual a luva
const uint8_t LORA_HEADER = 0xC2;
const int LORA_PKT_LEN = 11;  // pulso ocupa 2 bytes (int16)
const uint8_t LORA_PING = 0xA1;   // teste de conexao da luva: [0xA1, seq]
const uint8_t LORA_PONG = 0xA2;   // resposta: [0xA2, seq, rssi]
const uint8_t LORA_CMD  = 0xA4;   // [0xA4, tecla] vindo do serial do ESP
const uint8_t CMD_ENTRA = 0x01;
const uint8_t CMD_SAI   = 0x00;
const uint8_t CMD_HOME_ON  = 0x02;
const uint8_t CMD_HOME_OFF = 0x03;
const int LORA_PING_LEN = 2;

const int   PASSOS_MOTOR = 200;
const float REDUCAO = 38.4f;
const float PASSOS_POR_GRAU = (PASSOS_MOTOR * REDUCAO) / 360.0f;

// VMAX em passos/s, nao em °/s. Passo inteiro: a ressonancia do NEMA17 fica
// entre ~100 e ~300 passos/s. Cruzar essa faixa devagar perde passo e o
// home deixa de ser o ponto do boot. A velocidade de cruzeiro fica acima.
//                                    Cotovelo  Ombro   Pulso    Base
const float VMAX[NUM_EIXOS]  = {         500,    420,    750,    480 };
const float ACEL[NUM_EIXOS]  = {         700,    600,   2200,    900 };
// So o pulso: a aceleracao sobe aos poucos (igual ao teste-pulso).
const float JERK_PULSO = 18000.0f;

// Iguais a luva. Cotovelo -90..150, ombro -90..130, pulso -180..110, base ±150.
// Cotovelo, ombro e base chegam escalados de int8 (fundo / 127).
const float FUNDO_EIXO[NUM_EIXOS] = { 150.0f, 130.0f, 1.0f, 150.0f };
const float LIM_MIN[NUM_EIXOS] = { -90, -90, -180, -150 };
const float LIM_MAX[NUM_EIXOS] = { 150, 130,  110,  150 };

const float DEADZONE_GRAUS = 0.4f;
const unsigned long TIMEOUT_LORA_MS = 800;
const unsigned long PERIODO_CTRL_US = 2000;
const int DEADBAND_SERVO = 4;

long  pos[NUM_EIXOS];
long  alvo[NUM_EIXOS];
long  homeBoot[NUM_EIXOS];   // pose do boot: nunca muda com z
float alvoG[NUM_EIXOS];
float vel[NUM_EIXOS];
float accPulso = 0;
unsigned long proxStepUs[NUM_EIXOS];
unsigned long intervaloUs[NUM_EIXOS];
float escalaVel = 1.0f;

Servo servos[NUM_FLEX];
int anguloServo[NUM_FLEX];
int anguloServoEnv[NUM_FLEX];

unsigned long ultimoCtrlUs = 0;
unsigned long ultimoPacoteMs = 0;
unsigned long ultimoPrintMs = 0;

int  eixoManual = 0;
int  dirManual = 0;
bool modoManual = false;
bool loraOk = false;
bool emHome = false;   // h: ignora o pacote e fica no zero ate h de novo
bool manualRemoto = false;  // luva pediu o modo manual: juntas nao seguem o 0xC2
long pacotesOk = 0;

long grausParaPassos(float graus) {
  return (long)(graus * PASSOS_POR_GRAU + (graus >= 0 ? 0.5f : -0.5f));
}

float passosParaGraus(long passos) {
  return passos / PASSOS_POR_GRAU;
}

inline void pulsoStep(int i, bool frente) {
  bool d = INV_DIR[i] ? !frente : frente;
  digitalWrite(PIN_DIR[i], d ? HIGH : LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_STEP[i], HIGH);
  delayMicroseconds(3);
  digitalWrite(PIN_STEP[i], LOW);
  pos[i] += frente ? 1 : -1;
}

void atualizarControle(float dt) {
  if (dt <= 0 || dt > 0.05f) dt = 0.002f;
  for (int i = 0; i < NUM_EIXOS; i++) {
    long erro;
    if (modoManual && i == eixoManual && dirManual != 0) {
      alvo[i] = pos[i] + dirManual * 100000L;
      erro = alvo[i] - pos[i];
    } else {
      erro = alvo[i] - pos[i];
    }

    // Atravessa 90–300 passos/s mais rapido. O pulso ja cruza acima disso
    // e o jerk limita o tranco; acelera-lo aqui fazia perder passo.
    float acel = ACEL[i];
    float magV = fabsf(vel[i]);
    if (i != EIXO_PULSO && magV > 90.0f && magV < 300.0f) acel *= 2.5f;
    float jerk = JERK_PULSO;

    float vDes;
    if (erro == 0) {
      vDes = 0;
    } else {
      float dist = fabsf((float)erro);
      if (i == EIXO_PULSO) {
        float reserva = (acel * acel) / (6.0f * jerk);
        if (dist > reserva) dist -= reserva;
      }
      float vParar = sqrtf(2.0f * acel * dist);
      float v = VMAX[i] * escalaVel;
      if (vParar < v) v = vParar;
      vDes = (erro > 0) ? v : -v;
    }

    if (i == EIXO_PULSO) {
      float aAlvo = 0;
      if (vel[i] < vDes) aAlvo = acel;
      else if (vel[i] > vDes) aAlvo = -acel;
      float dj = jerk * dt;
      if (accPulso < aAlvo) accPulso = min(accPulso + dj, aAlvo);
      else if (accPulso > aAlvo) accPulso = max(accPulso - dj, aAlvo);
      vel[i] += accPulso * dt;
      float teto = VMAX[i] * escalaVel;
      if (vel[i] > teto)  { vel[i] = teto;  accPulso = 0; }
      if (vel[i] < -teto) { vel[i] = -teto; accPulso = 0; }
      if (aAlvo > 0 && vel[i] > vDes) { vel[i] = vDes; accPulso = 0; }
      if (aAlvo < 0 && vel[i] < vDes) { vel[i] = vDes; accPulso = 0; }
      if (erro == 0 && fabsf(vel[i]) < 30.0f) { vel[i] = 0; accPulso = 0; }
    } else {
      float dv = acel * dt;
      if (vel[i] < vDes)      vel[i] = min(vel[i] + dv, vDes);
      else if (vel[i] > vDes) vel[i] = max(vel[i] - dv, vDes);
    }

    float mag = fabsf(vel[i]);
    float piso = (i == EIXO_PULSO) ? 25.0f : 20.0f;
    if (mag < piso) {
      intervaloUs[i] = 0;
      if (erro == 0) vel[i] = 0;
      else intervaloUs[i] = (i == EIXO_PULSO) ? 30000 : 50000;
    } else {
      intervaloUs[i] = (unsigned long)(1000000.0f / mag);
    }
  }
}

void executarPassos(unsigned long agora) {
  for (int i = 0; i < NUM_EIXOS; i++) {
    if (intervaloUs[i] == 0) continue;
    if ((long)(agora - proxStepUs[i]) < 0) continue;

    bool frente;
    if (fabsf(vel[i]) >= 20.0f) frente = (vel[i] > 0);
    else {
      long erro = alvo[i] - pos[i];
      if (erro == 0) continue;
      frente = (erro > 0);
    }

    pulsoStep(i, frente);
    proxStepUs[i] = agora + intervaloUs[i];
  }
}

void atualizarServos() {
  for (int ch = 0; ch < NUM_FLEX; ch++) {
    int a = constrain(anguloServo[ch], 0, 180);
    if (anguloServoEnv[ch] < 0 || abs(a - anguloServoEnv[ch]) >= DEADBAND_SERVO) {
      servos[ch].write(a);
      anguloServoEnv[ch] = a;
    }
  }
}

void configurarLoRa() {
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  LoRa.setSPIFrequency(1E6);   // Uno 5 V -> SX1278 3.3 V: 8 MHz corrompe escritas
  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println(F("LoRa FALHOU — so modo manual"));
    loraOk = false;
    return;
  }
  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(250E3);
  LoRa.setCodingRate4(5);
  LoRa.enableCrc();
  LoRa.setTxPower(10);  // o PONG sai do 3V3 do Uno (~150 mA max); 17 dBm derruba
  LoRa.setOCP(140);   // padrao 100 mA corta o PA do PONG quando esquenta
  loraOk = true;
  Serial.println(F("LoRa RX OK (0xC2 braco+flex)"));
}

// Leitura direta: a lib nao expoe readRegister().
uint8_t lerRegLoRa(uint8_t reg) {
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(LORA_SS, LOW);
  SPI.transfer(reg & 0x7F);
  uint8_t v = SPI.transfer(0x00);
  digitalWrite(LORA_SS, HIGH);
  SPI.endTransaction();
  return v;
}

// Pico de corrente dos motores/servos reseta o SX1278: ele volta em FSK,
// 434 MHz, sem SF/BW, e fica mudo para sempre sem ninguem perceber.
bool loraConfigOk() {
  if (lerRegLoRa(0x42) != 0x12) return false;            // versao
  if ((lerRegLoRa(0x01) & 0x80) == 0) return false;      // modo LoRa
  if (lerRegLoRa(0x06) != 0x64) return false;            // 400 MHz
  if ((lerRegLoRa(0x1D) & 0xF0) != 0x80) return false;   // BW 250 kHz
  if ((lerRegLoRa(0x1E) & 0xF4) != 0x74) return false;   // SF7 + CRC
  if (lerRegLoRa(0x39) != 0x12) return false;            // sync word
  return true;
}

const unsigned long LORA_CHECK_MS   = 1000;
const unsigned long LORA_SILENCIO_MS = 3000;
unsigned long ultimoCheckLoRaMs = 0;
unsigned long ultimoReinitLoRaMs = 0;
int loraReinits = 0;

void vigiarLoRa() {
  unsigned long agora = millis();
  if (agora - ultimoCheckLoRaMs < LORA_CHECK_MS) return;
  ultimoCheckLoRaMs = agora;

  bool configRuim = !loraOk || !loraConfigOk();
  bool silencio = (agora - ultimoPacoteMs > LORA_SILENCIO_MS) &&
                  (agora - ultimoReinitLoRaMs > LORA_SILENCIO_MS);
  if (!configRuim && !silencio) return;

  Serial.println(configRuim ? F("!! LoRa perdeu config — reiniciando")
                            : F("!! LoRa em silencio — reiniciando"));
  LoRa.end();
  configurarLoRa();
  ultimoReinitLoRaMs = agora;
  loraReinits++;
}

void responderPingSeq(uint8_t seq) {
  int rssi = LoRa.packetRssi();
  delay(5);   // da tempo da luva voltar para RX depois do endPacket
  LoRa.beginPacket();
  LoRa.write(LORA_PONG);
  LoRa.write(seq);
  LoRa.write((uint8_t)(int8_t)rssi);
  LoRa.endPacket();

  Serial.print(F("PING ")); Serial.print(seq);
  Serial.print(F(" rssi=")); Serial.print(rssi);
  Serial.println(F(" -> PONG"));
}

void entrarHome() {
  emHome = true;
  modoManual = false;
  dirManual = 0;
  for (int i = 0; i < NUM_EIXOS; i++) {
    alvo[i] = homeBoot[i];
    alvoG[i] = 0;
  }
  for (int ch = 0; ch < NUM_FLEX; ch++) anguloServo[ch] = 0;
  atualizarServos();
  Serial.println(F("Home — voltando ao ponto em que o Arduino ligou."));
}

void sairHome() {
  emHome = false;
  Serial.println(F("Seguindo a luva."));
}

void aplicarCmdRemoto(uint8_t b) {
  ultimoPacoteMs = millis();
  pacotesOk++;
  if (b == CMD_ENTRA) {
    manualRemoto = true;
    emHome = false;
    modoManual = false;
    dirManual = 0;
    Serial.println(F("Manual remoto — juntas nao seguem a luva"));
    return;
  }
  if (b == CMD_SAI) {
    manualRemoto = false;
    emHome = false;
    modoManual = false;
    dirManual = 0;
    Serial.println(F("Seguindo a luva"));
    return;
  }
  if (b == CMD_HOME_ON) { entrarHome(); return; }
  if (b == CMD_HOME_OFF) { sairHome(); return; }
  processarComando((char)b);
}

void receberLoRa() {
  if (!loraOk) return;
  int tam = LoRa.parsePacket();
  if (tam == 0) return;
  if (tam == LORA_PING_LEN) {
    uint8_t h = LoRa.read();
    uint8_t b = LoRa.read();
    while (LoRa.available()) LoRa.read();
    if (h == LORA_PING) responderPingSeq(b);
    else if (h == LORA_CMD) aplicarCmdRemoto(b);
    return;
  }
  if (tam < LORA_PKT_LEN) {
    while (LoRa.available()) LoRa.read();
    return;
  }

  uint8_t buf[LORA_PKT_LEN];
  for (int i = 0; i < LORA_PKT_LEN && LoRa.available(); i++) buf[i] = LoRa.read();
  while (LoRa.available()) LoRa.read();
  if (buf[0] != LORA_HEADER) return;

  ultimoPacoteMs = millis();
  pacotesOk++;

  // Dedos sempre acompanham (mesmo em modo manual do braco).
  // Pulso ocupa buf[3] e buf[4], entao os dedos comecam em buf[6].
  for (int ch = 0; ch < NUM_FLEX; ch++) {
    anguloServo[ch] = (int)buf[6 + ch];
  }
  atualizarServos();

  if (emHome) {
    for (int ch = 0; ch < NUM_FLEX; ch++) anguloServo[ch] = 0;
    atualizarServos();
    for (int i = 0; i < NUM_EIXOS; i++) alvo[i] = homeBoot[i];
    return;
  }

  if (modoManual || manualRemoto) return;

  float graus[NUM_EIXOS];
  graus[EIXO_COTOVELO] = (int8_t)buf[1] * (FUNDO_EIXO[EIXO_COTOVELO] / 127.0f);
  graus[EIXO_OMBRO]    = (int8_t)buf[2] * (FUNDO_EIXO[EIXO_OMBRO] / 127.0f);
  graus[EIXO_PULSO]    = (int16_t)((uint16_t)buf[3] | ((uint16_t)buf[4] << 8));
  graus[EIXO_BASE]     = (int8_t)buf[5] * (FUNDO_EIXO[EIXO_BASE] / 127.0f);

  for (int i = 0; i < NUM_EIXOS; i++) {
    float g = constrain(graus[i], LIM_MIN[i], LIM_MAX[i]);
    alvoG[i] += 0.75f * (g - alvoG[i]);
    long novo = grausParaPassos(alvoG[i]);
    if (labs(novo - alvo[i]) >= grausParaPassos(DEADZONE_GRAUS))
      alvo[i] = novo;
  }
}

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

  Serial.println(F("=== Teste: servos 0 -> 90 -> 180 -> 0 por dedo ==="));
  const int POSES[] = { 0, 90, 180, 0 };
  for (int ch = 0; ch < NUM_FLEX; ch++) {
    Serial.print(F("  Dedo ")); Serial.print(ch + 1);
    Serial.print(F(" (D")); Serial.print(PIN_SERVO[ch]); Serial.println(F(")"));
    for (int k = 0; k < 4; k++) {
      servos[ch].write(POSES[k]);
      anguloServo[ch] = POSES[k];
      anguloServoEnv[ch] = POSES[k];
      delay(600);
    }
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
      if (emHome) sairHome();
      else entrarHome();
      return;

    case 'z': case 'Z':
      Serial.println(F("z nao redefine o home. h volta ao ponto do boot. Religue o Arduino na pose desejada para mudar esse ponto."));
      return;

    case '+': escalaVel = min(escalaVel + 0.1f, 2.0f); break;
    case '-': escalaVel = max(escalaVel - 0.1f, 0.2f); break;

    default: return;
  }
  Serial.print(F("Eixo ")); Serial.print(NOME_EIXO[eixoManual]);
  Serial.print(F("  dir=")); Serial.print(dirManual);
  Serial.print(F("  vel x")); Serial.println(escalaVel);
}

void setup() {
  Serial.begin(115200);

  for (int i = 0; i < NUM_EIXOS; i++) {
    pinMode(PIN_STEP[i], OUTPUT);
    pinMode(PIN_DIR[i], OUTPUT);
    digitalWrite(PIN_STEP[i], LOW);
    digitalWrite(PIN_DIR[i], LOW);
    pos[i] = 0; alvo[i] = 0; vel[i] = 0;
    homeBoot[i] = 0; alvoG[i] = 0;
    proxStepUs[i] = 0; intervaloUs[i] = 0;
  }

  for (int ch = 0; ch < NUM_FLEX; ch++) {
    servos[ch].attach(PIN_SERVO[ch]);
    servos[ch].write(0);
    anguloServo[ch] = 0;
    anguloServoEnv[ch] = 0;
  }

  Serial.println();
  Serial.println(F("=== KAIROS receptor — 4 motores + 5 servos ==="));
  Serial.print(F("passos/grau = ")); Serial.println(PASSOS_POR_GRAU);
  Serial.println(F("Servos: D8 D4 D5 D6 D7 | c/o/p/b + f/t/s | m teste | h home do boot | +/-"));
  Serial.println(F("Home = pose em que o Arduino ligou. Nao perde esse ponto com z."));

  configurarLoRa();
  ultimoCtrlUs = micros();
}

void loop() {
  receberLoRa();

  if (Serial.available()) processarComando(Serial.read());
  vigiarLoRa();

  unsigned long agora = micros();

  if ((unsigned long)(agora - ultimoCtrlUs) >= PERIODO_CTRL_US) {
    float dt = (agora - ultimoCtrlUs) / 1000000.0f;
    ultimoCtrlUs = agora;
    if (dt > 0.05f) dt = 0.05f;

    if (loraOk && !modoManual && !emHome && millis() - ultimoPacoteMs > TIMEOUT_LORA_MS) {
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
    Serial.print(F(" | D="));
    for (int ch = 0; ch < NUM_FLEX; ch++) {
      Serial.print(anguloServo[ch]);
      if (ch < NUM_FLEX - 1) Serial.print(' ');
    }
    Serial.print(F(" | pkt=")); Serial.print(pacotesOk);
    Serial.print(F(" | reinit=")); Serial.print(loraReinits);
    Serial.print(F(" | vel x")); Serial.print(escalaVel);
    if (emHome) {
      bool chegou = true;
      for (int i = 0; i < NUM_EIXOS; i++) {
        if (labs(pos[i] - homeBoot[i]) > grausParaPassos(2.0f)) chegou = false;
      }
      Serial.print(chegou ? F(" | HOME ok") : F(" | HOME"));
    }
    Serial.println();
  }
}
