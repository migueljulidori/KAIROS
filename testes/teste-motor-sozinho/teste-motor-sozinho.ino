/*
 * Kairós — teste-motor-sozinho (diagnostico ombro)
 *
 * SEM LoRa. Pinos IGUAIS ao projeto:
 *   Cotovelo D3/A0 | Ombro A1/A2 | Pulso A3/A4 | Base A5/D2
 *
 * Serial 9600
 * No boot: pulsa STEP bem LENTO (da para medir no multimetro).
 * Comandos: f t s m
 */

#define EIXO_COTOVELO 0
#define EIXO_OMBRO    1
#define EIXO_PULSO    2
#define EIXO_BASE     3

// ---- DESCOMENTE APENAS UM ----
// #define EIXO_ATIVO EIXO_COTOVELO
#define EIXO_ATIVO EIXO_OMBRO
// #define EIXO_ATIVO EIXO_PULSO
// #define EIXO_ATIVO EIXO_BASE

const int PIN_STEP[4] = { 3, A1, A3, A5 };
const int PIN_DIR[4]  = { A0, A2, A4, 2 };
const char* NOME[4]   = { "Cotovelo", "Ombro", "Pulso", "Base" };

// Lento de proposito para medir STEP no multimetro
const int DELAY_US = 5000;

int modo = 0;
unsigned long ultimoPrint = 0;
long contadorPassos = 0;

void darPasso(bool frente) {
  digitalWrite(PIN_DIR[EIXO_ATIVO], frente ? HIGH : LOW);
  delayMicroseconds(50);
  digitalWrite(PIN_STEP[EIXO_ATIVO], HIGH);
  delayMicroseconds(200);
  digitalWrite(PIN_STEP[EIXO_ATIVO], LOW);
  delayMicroseconds(DELAY_US);
  contadorPassos++;
}

void setup() {
  Serial.begin(9600);
  delay(300);

  pinMode(PIN_STEP[EIXO_ATIVO], OUTPUT);
  pinMode(PIN_DIR[EIXO_ATIVO], OUTPUT);
  digitalWrite(PIN_STEP[EIXO_ATIVO], LOW);
  digitalWrite(PIN_DIR[EIXO_ATIVO], LOW);

  Serial.println("=== DIAGNOSTICO MOTOR ===");
  Serial.print("Eixo: ");
  Serial.println(NOME[EIXO_ATIVO]);
  Serial.println("Ombro => STEP no pino A1 | DIR no pino A2");
  Serial.println();
  Serial.println("TESTE 1 — multimetro no pino A1 (STEP):");
  Serial.println("  deve alternar ~0V e ~5V a cada meio segundo");
  Serial.println();

  // Pulso LENTO visivel no multimetro (10 vezes)
  for (int i = 0; i < 10; i++) {
    digitalWrite(PIN_STEP[EIXO_ATIVO], HIGH);
    Serial.println("STEP = 5V (A1)");
    delay(500);
    digitalWrite(PIN_STEP[EIXO_ATIVO], LOW);
    Serial.println("STEP = 0V (A1)");
    delay(500);
  }

  Serial.println();
  Serial.println("TESTE 2 — 100 passos (ouvir motor/vibrar)");
  for (int i = 0; i < 100; i++) darPasso(true);
  delay(400);
  for (int i = 0; i < 100; i++) darPasso(false);

  Serial.println("Fim auto. Digite f / t / s / m");
  Serial.println("Se A1 NAO oscilar no teste 1 => problema no Uno/cabo STEP");
  Serial.println("Se A1 oscilar e motor parado => A4988/bobinas/Vref/ENABLE");
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'f' || c == 'F') { modo = 1; Serial.println("frente"); }
    if (c == 't' || c == 'T') { modo = 2; Serial.println("tras"); }
    if (c == 's' || c == 'S') { modo = 0; Serial.println("parado"); }
    if (c == 'm' || c == 'M') {
      Serial.println("teste m");
      for (int i = 0; i < 100; i++) darPasso(true);
      delay(300);
      for (int i = 0; i < 100; i++) darPasso(false);
      modo = 0;
      Serial.println("ok");
    }
  }

  if (modo == 1) darPasso(true);
  if (modo == 2) darPasso(false);

  if (modo != 0 && millis() - ultimoPrint > 1000) {
    ultimoPrint = millis();
    Serial.print("passos=");
    Serial.println(contadorPassos);
  }
}
