/*
 * Kair�s � teste-flex-5 (5 flex + 5 servos)
 *
 * LIGACOES (repetir para cada dedo 1 a 5):
 *
 * Flex Sensor 2.2"
 *   5V ----[FLEX]----+---- A0..A4  (C1=A0, C2=A1, C3=A2, C4=A3, C5=A4)
 *                    |
 *                 [66k?]
 *                    |
 *                   GND
 *
 * Servo MG996R
 *   laranja (sinal) -> D3, D5, D6, D9, D10  (C1..C5)
 *   vermelho        -> 5V (fonte externa recomendada)
 *   marrom/preto    -> GND
 *
 * IMPORTANTE: 5 servos consomem muita corrente.
 *   Use fonte 5V externa para os servos + GND comum com o Arduino.
 *
 * Calibre flexMin[] (dobrado) e flexMax[] (reto) para cada canal.
 */

#include <Servo.h>

const int NUM_CANAIS = 5;

const int PIN_FLEX[NUM_CANAIS]  = { A0, A1, A2, A3, A4 };
const int PIN_SERVO[NUM_CANAIS] = {  3,  5,  6,  9, 10 };

const int OVERSAMPLE_N = 16;
const int BUF_N = 15;

// Calibrar cada dedo (valores iniciais iguais ao teste de 1 flex)
int flexMin[NUM_CANAIS] = { 950, 950, 950, 950, 950 };
int flexMax[NUM_CANAIS] = { 1019, 1019, 1019, 1019, 1019 };

Servo servos[NUM_CANAIS];

int buffer[NUM_CANAIS][BUF_N];
int idxBuffer[NUM_CANAIS];

float valorFiltrado[NUM_CANAIS];
float anguloSuave[NUM_CANAIS];
float filtroDeriv[NUM_CANAIS];
bool  filtroInit[NUM_CANAIS];

int anguloEnviado[NUM_CANAIS];

unsigned long tempoAnteriorUs = 0;

const float LIMIAR_VAR = 8.0f;

const float minCutoffNormal   = 1.20f;
const float minCutoffInstavel = 0.55f;
const float beta              = 0.050f;
const float dCutoff           = 1.0f;
const float alphaAngulo       = 0.45f;

const int deadbandServo = 2;

int lerFlex(int pino) {
  long soma = 0;
  for (int i = 0; i < OVERSAMPLE_N; i++) {
    soma += analogRead(pino);
    delayMicroseconds(150);
  }
  return (int)(soma / OVERSAMPLE_N);
}

void sortArray(int arr[], int n) {
  for (int i = 1; i < n; i++) {
    int key = arr[i];
    int j = i - 1;
    while (j >= 0 && arr[j] > key) {
      arr[j + 1] = arr[j];
      j--;
    }
    arr[j + 1] = key;
  }
}

float mediaTruncada(int arr[], int n) {
  int tmp[BUF_N];
  for (int i = 0; i < n; i++) tmp[i] = arr[i];
  sortArray(tmp, n);

  int cortar = n / 5;
  if (cortar < 1) cortar = 1;

  long soma = 0;
  int cnt = 0;
  for (int i = cortar; i < n - cortar; i++) {
    soma += tmp[i];
    cnt++;
  }
  return (float)soma / cnt;
}

float calcularVariancia(int arr[], int n) {
  long soma = 0;
  for (int i = 0; i < n; i++) soma += arr[i];
  float media = (float)soma / n;
  float var = 0;
  for (int i = 0; i < n; i++) {
    float d = arr[i] - media;
    var += d * d;
  }
  return var / n;
}

float filtroOneEuro(int ch, float x, float dt, float minCutoff) {
  if (dt <= 0.0f) dt = 0.001f;

  float dx = (x - valorFiltrado[ch]) / dt;
  float alphaD = dt / (dt + 1.0f / (2.0f * 3.14159265f * dCutoff));
  filtroDeriv[ch] = alphaD * dx + (1.0f - alphaD) * filtroDeriv[ch];

  float cutoff = minCutoff + beta * fabs(filtroDeriv[ch]);
  float alpha  = dt / (dt + 1.0f / (2.0f * 3.14159265f * cutoff));

  if (!filtroInit[ch]) {
    valorFiltrado[ch] = x;
    filtroInit[ch] = true;
  } else {
    valorFiltrado[ch] = alpha * x + (1.0f - alpha) * valorFiltrado[ch];
  }

  return valorFiltrado[ch];
}

void processarCanal(int ch, float dt) {
  int raw = lerFlex(PIN_FLEX[ch]);

  buffer[ch][idxBuffer[ch]] = raw;
  idxBuffer[ch] = (idxBuffer[ch] + 1) % BUF_N;

  float truncada  = mediaTruncada(buffer[ch], BUF_N);
  float variancia = calcularVariancia(buffer[ch], BUF_N);

  float minCutoff = (variancia > LIMIAR_VAR) ? minCutoffInstavel : minCutoffNormal;
  filtroOneEuro(ch, truncada, dt, minCutoff);

  int flexFiltrado = (int)(valorFiltrado[ch] + 0.5f);

  int anguloAlvo = map(flexFiltrado, flexMax[ch], flexMin[ch], 0, 180);
  anguloAlvo = constrain(anguloAlvo, 0, 180);

  anguloSuave[ch] = anguloSuave[ch] * (1.0f - alphaAngulo) + anguloAlvo * alphaAngulo;
  int angulo = (int)(anguloSuave[ch] + 0.5f);

  if (abs(angulo - anguloEnviado[ch]) >= deadbandServo) {
    servos[ch].write(angulo);
    anguloEnviado[ch] = angulo;
  }

  Serial.print("C");
  Serial.print(ch + 1);
  Serial.print(" F:");
  Serial.print(flexFiltrado);
  Serial.print(" R:");
  Serial.print(raw);
  Serial.print(" S:");
  Serial.print(angulo);

  if (ch < NUM_CANAIS - 1) {
    Serial.print(" | ");
  }
}

void setup() {
  Serial.begin(9600);

  for (int ch = 0; ch < NUM_CANAIS; ch++) {
    servos[ch].attach(PIN_SERVO[ch]);

    int inicial = lerFlex(PIN_FLEX[ch]);
    for (int i = 0; i < BUF_N; i++) buffer[ch][i] = inicial;

    valorFiltrado[ch] = inicial;
    filtroInit[ch] = true;
    anguloEnviado[ch] = -1;

    anguloSuave[ch] = constrain(map(inicial, flexMax[ch], flexMin[ch], 0, 180), 0, 180);
    servos[ch].write((int)anguloSuave[ch]);
  }

  tempoAnteriorUs = micros();

  Serial.println("Kairos teste-flex-5 iniciado");
}

void loop() {
  unsigned long agoraUs = micros();
  float dt = (agoraUs - tempoAnteriorUs) / 1000000.0f;
  tempoAnteriorUs = agoraUs;

  for (int ch = 0; ch < NUM_CANAIS; ch++) {
    processarCanal(ch, dt);
  }

  Serial.println();
}
