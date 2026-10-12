/*
 * Kairós — teste-mpu-verifica
 *   Só verifica se os 3 MPU6050 respondem no Soft-I2C.
 *   Sem calibracao, sem LoRa, sem flex.
 *
 * Serial 115200
 *   - boot: diagnostico (niveis, ACK 0x68/0x69, WHO_AM_I)
 *   - loop: accel (g) + gyro (°/s) a cada ~250 ms
 *   - tecla r = re-escanear / reiniciar sensores
 *
 * Ligacoes (iguais ao kairos-luva-tx):
 *   Mao:       SDA 27  SCL 14
 *   Antebraco: SDA 25  SCL 26
 *   Braco:     SDA  4  SCL 13
 *   VCC->3V3   GND->GND
 *   AD0->GND = 0x68   |   AD0->3V3 = 0x69   (nao deixe flutuando)
 */

#include <math.h>

const uint8_t MPU_ADDR_GND = 0x68;
const uint8_t MPU_ADDR_VCC = 0x69;
const int NUM_MPU = 3;
const int SOFT_DELAY_US = 5;

const int PIN_SDA[NUM_MPU] = { 27, 4, 25 };
const int PIN_SCL[NUM_MPU] = { 14, 13, 26 };
const char* NOME[NUM_MPU]  = { "Mao", "Antebraco", "Braco" };

uint8_t mpuAddr[NUM_MPU];
bool    mpuOk[NUM_MPU];

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
  gAddr = mpuAddr[i];
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
  if (!softWriteByte((gAddr << 1) | 0)) { softStop(); return false; }
  if (!softWriteByte(reg))              { softStop(); return false; }
  if (!softWriteByte(val))              { softStop(); return false; }
  softStop();
  return true;
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

bool softReadRegOnce(uint8_t addr, uint8_t reg, uint8_t* val) {
  softStart();
  if (!softWriteByte((addr << 1) | 0)) { softStop(); return false; }
  if (!softWriteByte(reg))             { softStop(); return false; }
  softStart();
  if (!softWriteByte((addr << 1) | 1)) { softStop(); return false; }
  *val = softReadByte(false);
  softStop();
  return true;
}

bool lerRaw(int i, float& ax, float& ay, float& az,
            float& gx, float& gy, float& gz) {
  usarBarramento(i);
  uint8_t b[14];
  if (!softRead(0x3B, b, 14)) return false;
  int16_t rax = (b[0]  << 8) | b[1];
  int16_t ray = (b[2]  << 8) | b[3];
  int16_t raz = (b[4]  << 8) | b[5];
  int16_t rgx = (b[8]  << 8) | b[9];
  int16_t rgy = (b[10] << 8) | b[11];
  int16_t rgz = (b[12] << 8) | b[13];
  ax = rax / 16384.0f; ay = ray / 16384.0f; az = raz / 16384.0f;
  gx = rgx / 65.5f;    gy = rgy / 65.5f;    gz = rgz / 65.5f;
  return true;
}

void imprimirDiag(int i) {
  gSda = PIN_SDA[i];
  gScl = PIN_SCL[i];
  softSdaHigh();
  softSclHigh();
  delay(2);

  int nivSda = digitalRead(gSda);
  int nivScl = digitalRead(gScl);

  Serial.printf("%-10s  SDA GPIO%-2d  SCL GPIO%-2d  |  niveis SDA=%d SCL=%d",
                NOME[i], PIN_SDA[i], PIN_SCL[i], nivSda, nivScl);

  if (!nivSda || !nivScl) {
    Serial.println(F("  |  BARRAMENTO PRESO (curto / solda)"));
    mpuOk[i] = false;
    return;
  }

  softClockStretch();
  bool a68 = softAck(MPU_ADDR_GND);
  bool a69 = softAck(MPU_ADDR_VCC);
  Serial.printf("  |  ACK 0x68=%s  0x69=%s",
                a68 ? "sim" : "nao", a69 ? "sim" : "nao");

  uint8_t who = 0;
  uint8_t addr = 0;
  if (a68 && softReadRegOnce(MPU_ADDR_GND, 0x75, &who)) {
    addr = MPU_ADDR_GND;
  } else if (a69 && softReadRegOnce(MPU_ADDR_VCC, 0x75, &who)) {
    addr = MPU_ADDR_VCC;
  }

  if (!addr) {
    Serial.println(F("  |  FALHOU — SEM ACK (AD0/solda/modulo)"));
    mpuOk[i] = false;
    return;
  }

  mpuAddr[i] = addr;
  gAddr = addr;
  Serial.printf("  |  WHO=0x%02X @0x%02X", who, addr);

  softClockStretch();
  if (!softWriteReg(0x6B, 0x00)) {
    Serial.println(F("  |  FALHOU ao acordar (PWR_MGMT)"));
    mpuOk[i] = false;
    return;
  }
  delay(10);
  softWriteReg(0x1B, 0x08);  // gyro ±500 °/s
  softWriteReg(0x1C, 0x00);  // accel ±2 g
  softWriteReg(0x1A, 0x03);  // DLPF 44 Hz
  softWriteReg(0x19, 0x04);  // ~200 Hz

  float ax, ay, az, gx, gy, gz;
  if (!lerRaw(i, ax, ay, az, gx, gy, gz)) {
    Serial.println(F("  |  FALHOU na leitura"));
    mpuOk[i] = false;
    return;
  }

  float mag = sqrtf(ax * ax + ay * ay + az * az);
  Serial.printf("  |  |a|=%.2f g  OK\n", mag);
  mpuOk[i] = true;
}

void escanearTodos() {
  Serial.println();
  Serial.println(F("=== KAIROS teste-mpu-verifica ==="));
  Serial.println(F("Sem calibracao. Tecla r = re-escanear."));
  Serial.println(F("Parado: |a| deve ficar perto de 1.00 g."));
  Serial.println();

  int ok = 0;
  for (int i = 0; i < NUM_MPU; i++) {
    mpuAddr[i] = MPU_ADDR_GND;
    mpuOk[i] = false;
    imprimirDiag(i);
    if (mpuOk[i]) ok++;
  }

  Serial.println();
  Serial.printf("Resultado: %d / %d OK\n", ok, NUM_MPU);
  if (ok < NUM_MPU) {
    Serial.println(F("Dica Braco: AD0 soldado em GND (0x68) ou 3V3 (0x69)."));
  }
  Serial.println(F("------------------------------------------------"));
}

void setup() {
  Serial.begin(115200);
  delay(400);
  escanearTodos();
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'r' || c == 'R') escanearTodos();
  }

  static unsigned long ultimo = 0;
  if (millis() - ultimo < 250) return;
  ultimo = millis();

  for (int i = 0; i < NUM_MPU; i++) {
    if (!mpuOk[i]) {
      Serial.printf("%-10s  --- FALHOU ---\n", NOME[i]);
      continue;
    }
    float ax, ay, az, gx, gy, gz;
    if (!lerRaw(i, ax, ay, az, gx, gy, gz)) {
      Serial.printf("%-10s  leitura I2C falhou (tecla r)\n", NOME[i]);
      mpuOk[i] = false;
      continue;
    }
    float mag = sqrtf(ax * ax + ay * ay + az * az);
    Serial.printf("%-10s  a=[% .2f % .2f % .2f] |a|=%.2f  g=[% .1f % .1f % .1f]\n",
                  NOME[i], ax, ay, az, mag, gx, gy, gz);
  }
  Serial.println();
}
