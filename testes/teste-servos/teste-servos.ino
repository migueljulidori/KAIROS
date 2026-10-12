/*
 * Kairós — teste-servos (Arduino Uno)
 *
 * Teste dos 5 servos: abre e fecha os dedos em loop.
 *
 * Servos MG996R (sinal):
 *   D1: D8 | D2: D4 | D3: D5 | D4: D6 | D5: D7
 *   +5V fonte externa, GND comum com Arduino
 *
 * 0   = dedo reto (aberto)
 * 180 = dedo dobrado (fechado)
 */

#include <Servo.h>

const int NUM_SERVOS = 5;
const int PIN_SERVO[NUM_SERVOS] = { 8, 4, 5, 6, 7 };

const int ANGULO_ABERTO  = 0;
const int ANGULO_FECHADO = 180;
const int PASSO          = 3;
const int DELAY_MS       = 15;

Servo servos[NUM_SERVOS];
int anguloAtual[NUM_SERVOS];

void moverTodos(int destino) {
  bool chegou = false;

  while (!chegou) {
    chegou = true;

    for (int i = 0; i < NUM_SERVOS; i++) {
      if (anguloAtual[i] < destino) {
        anguloAtual[i] += PASSO;
        if (anguloAtual[i] > destino) anguloAtual[i] = destino;
        chegou = false;
      } else if (anguloAtual[i] > destino) {
        anguloAtual[i] -= PASSO;
        if (anguloAtual[i] < destino) anguloAtual[i] = destino;
        chegou = false;
      }

      servos[i].write(anguloAtual[i]);
    }

    delay(DELAY_MS);
  }
}

void setup() {
  Serial.begin(9600);
  Serial.println("Kairos teste-servos — abrindo e fechando dedos");

  for (int i = 0; i < NUM_SERVOS; i++) {
    servos[i].attach(PIN_SERVO[i]);
    anguloAtual[i] = ANGULO_ABERTO;
    servos[i].write(ANGULO_ABERTO);
  }

  delay(500);
}

void loop() {
  Serial.println("Fechando dedos...");
  moverTodos(ANGULO_FECHADO);
  delay(2000);

  Serial.println("Abrindo dedos...");
  moverTodos(ANGULO_ABERTO);
  delay(400);
}
