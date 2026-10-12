/*
 * Kairós — teste-braco-motor (Arduino Uno)
 *
 * Teste ISOLADO de 1 eixo (A4988 + NEMA17).
 * Comece pelo cotovelo. Depois repita a logica para ombro, pulso e base.
 *
 * ============================================================
 * LIGACOES A4988 <-> Arduino Uno
 * ============================================================
 *
 *   A4988          Uno
 *   -----          ---
 *   STEP    ---->  D3
 *   DIR     ---->  A0
 *   ENABLE  ---->  A1   (LOW = motor ligado)
 *   VDD     ---->  5V
 *   GND     ---->  GND  (comum com a fonte do motor)
 *   VMOT    ---->  +12V fonte externa do motor
 *   GND(VM) ---->  GND da fonte (mesmo GND do Uno)
 *   1A/1B/2A/2B -> bobinas do NEMA17
 *
 * IMPORTANTE:
 *   - Nunca alimentar o motor pelo 5V do Arduino
 *   - Fonte tipica: 12V, 2A ou mais
 *   - Ajustar o potenciometro de corrente do A4988
 *   - Dissipador no A4988 recomendado
 *
 * ============================================================
 * COMANDOS NO MONITOR SERIAL (9600)
 * ============================================================
 *
 *   f  = gira para frente  (ex.: cotovelo dobra)
 *   t  = gira para tras    (ex.: cotovelo estende)
 *   s  = para
 *   +  = aumenta velocidade
 *   -  = diminui velocidade
 *   1  = 45 graus (aprox.)
 *   2  = 90 graus (aprox.)
 *   3  = 180 graus (aprox.)
 *
 * Ajuste PASSOS_POR_VOLTA conforme microstepping do A4988:
 *   full step  = 200
 *   1/2        = 400
 *   1/4        = 800
 *   1/8        = 1600
 *   1/16       = 3200
 */

const int PIN_STEP   = 3;
const int PIN_DIR    = A0;
const int PIN_ENABLE = A1;

const int PASSOS_POR_VOLTA = 200;   // full step (MS1/MS2/MS3 em GND)
const int DELAY_US_MIN = 400;       // mais rapido
const int DELAY_US_MAX = 3000;      // mais lento

int delayUs = 1200;
int sentido = HIGH;                 // HIGH = frente, LOW = tras
bool motorLigado = false;
long passosRestantes = 0;

void setup() {
  Serial.begin(9600);

  pinMode(PIN_STEP, OUTPUT);
  pinMode(PIN_DIR, OUTPUT);
  pinMode(PIN_ENABLE, OUTPUT);

  digitalWrite(PIN_ENABLE, HIGH);   // desligado no inicio
  digitalWrite(PIN_DIR, sentido);
  digitalWrite(PIN_STEP, LOW);

  Serial.println("=== KAIROS teste-braco-motor ===");
  Serial.println("Comandos: f t s + - 1 2 3");
  Serial.println("Ligue a fonte 12V do motor e digite f ou t");
}

void darPasso() {
  digitalWrite(PIN_STEP, HIGH);
  delayMicroseconds(delayUs);
  digitalWrite(PIN_STEP, LOW);
  delayMicroseconds(delayUs);
}

void ligarMotor() {
  digitalWrite(PIN_ENABLE, LOW);    // A4988: LOW = ativo
  motorLigado = true;
}

void desligarMotor() {
  digitalWrite(PIN_ENABLE, HIGH);
  motorLigado = false;
  passosRestantes = 0;
}

void girarGraus(int graus) {
  ligarMotor();
  passosRestantes = (long)PASSOS_POR_VOLTA * graus / 360;
  Serial.print("Movendo ~");
  Serial.print(graus);
  Serial.print(" graus | passos=");
  Serial.println(passosRestantes);
}

void processarComando(char c) {
  switch (c) {
    case 'f':
    case 'F':
      sentido = HIGH;
      digitalWrite(PIN_DIR, sentido);
      passosRestantes = -1;   // continuo
      ligarMotor();
      Serial.println("Frente (continuo)");
      break;

    case 't':
    case 'T':
      sentido = LOW;
      digitalWrite(PIN_DIR, sentido);
      passosRestantes = -1;
      ligarMotor();
      Serial.println("Tras (continuo)");
      break;

    case 's':
    case 'S':
      desligarMotor();
      Serial.println("Parado");
      break;

    case '+':
      delayUs = constrain(delayUs - 100, DELAY_US_MIN, DELAY_US_MAX);
      Serial.print("Mais rapido | delayUs=");
      Serial.println(delayUs);
      break;

    case '-':
      delayUs = constrain(delayUs + 100, DELAY_US_MIN, DELAY_US_MAX);
      Serial.print("Mais lento | delayUs=");
      Serial.println(delayUs);
      break;

    case '1':
      girarGraus(45);
      break;

    case '2':
      girarGraus(90);
      break;

    case '3':
      girarGraus(180);
      break;
  }
}

void loop() {
  if (Serial.available()) {
    processarComando((char)Serial.read());
  }

  if (!motorLigado) return;

  if (passosRestantes == 0) {
    desligarMotor();
    Serial.println("Movimento concluido");
    return;
  }

  darPasso();

  if (passosRestantes > 0) {
    passosRestantes--;
  }
}
