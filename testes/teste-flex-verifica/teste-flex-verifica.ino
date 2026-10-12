/*
 * Kairós — teste-flex-verifica
 *   Mesma leitura + filtro do teste-flex (que funcionou).
 *   Envia os dedos por LoRa para o kairos-receptor (sem mudar o Arduino).
 *
 * Pacote LoRa 10 bytes (formato do kairos-receptor):
 *   [0xC2, cotovelo, ombro, pulso, base, d0, d1, d2, d3, d4]
 *   Juntas vao sempre 0: o braco fica no zero em que ligou (so os dedos mexem).
 *
 * LoRa SX1278 433MHz:
 *   NSS 5  RST 2  DIO0 15  SCK 18  MISO 19  MOSI 23
 *
 * Pipeline (igual teste-flex):
 *   oversample 16 -> buffer 15 -> media truncada -> variancia
 *   -> One Euro (cutoff por variancia) -> deadband -> map 0..180
 *
 * Serial 115200 | tecla r = recalibra
 *
 * Ligacao:
 *   3V3 --[FLEX]--+-- GPIO
 *                 |
 *               66 kΩ
 *                 |
 *                GND
 *   D1:34  D2:35  D3:32  D4:33  D5:36
 */

#include <WiFi.h>
#include <SPI.h>
#include <LoRa.h>

const int NUM_CANAIS = 5;
const int PIN_FLEX[NUM_CANAIS] = { 34, 35, 32, 33, 36 };

#define LORA_SS   5
#define LORA_RST  2
#define LORA_DIO0 15
const long LORA_FREQ = 115E6;
const uint8_t LORA_HEADER = 0xC2;
const unsigned long TX_PERIODO_MS = 40;   // 25 Hz, igual luva-tx
unsigned long ultimoTxMs = 0;

const int OVERSAMPLE_N = 16;
const int CAL_AMOSTRAS = 25;
const int FAIXA_MINIMA = 10;
const int BUF_N = 15;

int flexMin[NUM_CANAIS];
int flexMax[NUM_CANAIS];

int buffer[NUM_CANAIS][BUF_N];
int idxBuffer[NUM_CANAIS];

float valorFiltrado[NUM_CANAIS];
float flexEstavel[NUM_CANAIS];
float filtroDeriv[NUM_CANAIS];
bool  filtroInit[NUM_CANAIS];

int flexPublicado[NUM_CANAIS];

unsigned long tempoAnteriorUs = 0;

const float LIMIAR_VAR = 6.0f;
const float minCutoffNormal   = 0.90f;
const float minCutoffInstavel = 0.35f;
const float beta              = 0.040f;
const float dCutoff           = 1.0f;
const float alphaExtraParado  = 0.12f;
const int deadbandFlex = 4;

const char* NOME_DEDO[NUM_CANAIS] = {
  "Polegar", "Indicador", "Medio", "Anelar", "Mindinho"
};

const int TOTAL_PASSOS_CAL = NUM_CANAIS * 2;

void configurarLoRa() {
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println(F("ERRO: LoRa nao iniciou."));
    while (true) delay(1000);
  }
  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(250E3);
  LoRa.setCodingRate4(5);
  LoRa.enableCrc();
  LoRa.setTxPower(14);
  Serial.println(F("LoRa TX OK (0xC2, so dedos)"));
}

long pacotesTx = 0;
long falhasTx = 0;

void enviarLoRa(const int ang[NUM_CANAIS]) {
  if (!LoRa.beginPacket()) { falhasTx++; return; }
  LoRa.write(LORA_HEADER);
  for (int j = 0; j < 4; j++) LoRa.write((uint8_t)0);   // cotovelo ombro pulso base
  for (int ch = 0; ch < NUM_CANAIS; ch++) LoRa.write((uint8_t)ang[ch]);
  if (LoRa.endPacket()) pacotesTx++;
  else falhasTx++;
}

int lerFlex(int ch) {
  long soma = 0;
  for (int i = 0; i < OVERSAMPLE_N; i++) {
    soma += analogRead(PIN_FLEX[ch]);
    delayMicroseconds(100);
  }
  return (int)(soma / OVERSAMPLE_N);
}

int medirMedia(int ch) {
  long soma = 0;
  for (int i = 0; i < CAL_AMOSTRAS; i++) {
    soma += lerFlex(ch);
    delay(40);
  }
  return (int)(soma / CAL_AMOSTRAS);
}

void aguardarContagem() {
  Serial.println(F("Preparando leitura em..."));
  for (int s = 3; s >= 1; s--) {
    Serial.println(s);
    delay(1000);
  }
}

void previewLeitura(int ch) {
  Serial.println(F("Leitura ao vivo (somente este dedo):"));
  for (int i = 0; i < 8; i++) {
    Serial.printf("  %d\n", lerFlex(ch));
    delay(200);
  }
}

int calibrarPasso(int ch, int numeroPasso, const char* posicao) {
  Serial.println();
  Serial.println(F("----------------------------------------"));
  Serial.printf("PASSO %d de %d\n", numeroPasso, TOTAL_PASSOS_CAL);
  Serial.printf("Dedo %d — %s\n", ch + 1, NOME_DEDO[ch]);
  Serial.printf("Posicao: %s\n", posicao);
  Serial.println(F("Calibre SOMENTE este dedo. Mantenha os outros parados."));
  Serial.println();

  previewLeitura(ch);
  aguardarContagem();

  int valor = medirMedia(ch);
  Serial.printf("Valor salvo para D%d (%s): %d\n", ch + 1, posicao, valor);
  Serial.println(F("----------------------------------------"));
  return valor;
}

bool calibrarSistema() {
  Serial.println();
  Serial.println(F("=== KAIROS — CALIBRACAO DEDO A DEDO (igual teste-flex) ==="));
  Serial.println(F("5 dedos, um por vez: primeiro RETO, depois DOBRADO."));
  Serial.println();
  delay(1000);

  int passo = 1;
  for (int ch = 0; ch < NUM_CANAIS; ch++) {
    Serial.println();
    Serial.printf(">>> Dedo %d (%s) <<<\n", ch + 1, NOME_DEDO[ch]);
    delay(800);

    flexMax[ch] = calibrarPasso(ch, passo++, "RETO");
    flexMin[ch] = calibrarPasso(ch, passo++, "DOBRADO NO MAXIMO");

    if (flexMax[ch] <= flexMin[ch]) {
      Serial.printf("ERRO D%d: RETO deve ser MAIOR que DOBRADO.\n", ch + 1);
      return false;
    }
    if ((flexMax[ch] - flexMin[ch]) < FAIXA_MINIMA) {
      Serial.printf("ERRO D%d: diferenca muito pequena.\n", ch + 1);
      return false;
    }
    Serial.printf("Dedo %d OK\n", ch + 1);
    delay(500);
  }

  Serial.println();
  Serial.println(F("=== CALIBRACAO OK ==="));
  for (int ch = 0; ch < NUM_CANAIS; ch++) {
    Serial.printf("D%d %-10s | reto=%4d | dobrado=%4d | faixa=%d\n",
                  ch + 1, NOME_DEDO[ch], flexMax[ch], flexMin[ch],
                  flexMax[ch] - flexMin[ch]);
  }
  Serial.println(F("Movimente os dedos. Tecla r = recalibrar."));
  Serial.println();
  return true;
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

void iniciarFiltros() {
  for (int ch = 0; ch < NUM_CANAIS; ch++) {
    int inicial = flexMax[ch];
    for (int i = 0; i < BUF_N; i++) buffer[ch][i] = inicial;
    idxBuffer[ch] = 0;
    valorFiltrado[ch] = inicial;
    flexEstavel[ch]   = inicial;
    flexPublicado[ch] = -1;
    filtroInit[ch]    = true;
    filtroDeriv[ch]   = 0;
  }
  tempoAnteriorUs = micros();
}

void processarCanal(int ch, float dt, int& rawOut, int& anguloOut) {
  int raw = lerFlex(ch);
  rawOut = raw;

  buffer[ch][idxBuffer[ch]] = raw;
  idxBuffer[ch] = (idxBuffer[ch] + 1) % BUF_N;

  float truncada  = mediaTruncada(buffer[ch], BUF_N);
  float variancia = calcularVariancia(buffer[ch], BUF_N);
  float minCutoff = (variancia > LIMIAR_VAR) ? minCutoffInstavel : minCutoffNormal;
  filtroOneEuro(ch, truncada, dt, minCutoff);

  bool instavel = (variancia > LIMIAR_VAR);
  if (instavel) {
    flexEstavel[ch] = flexEstavel[ch] * (1.0f - alphaExtraParado) + valorFiltrado[ch] * alphaExtraParado;
  } else {
    flexEstavel[ch] = valorFiltrado[ch];
  }

  int flexFiltrado = (int)(flexEstavel[ch] + 0.5f);
  if (flexPublicado[ch] < 0) {
    flexPublicado[ch] = flexFiltrado;
  } else if (!instavel || abs(flexFiltrado - flexPublicado[ch]) >= deadbandFlex) {
    flexPublicado[ch] = flexFiltrado;
  }

  anguloOut = map(flexPublicado[ch], flexMax[ch], flexMin[ch], 0, 180);
  anguloOut = constrain(anguloOut, 0, 180);
}

void setup() {
  Serial.begin(115200);
  delay(300);
  WiFi.mode(WIFI_OFF);

  analogReadResolution(12);
  for (int ch = 0; ch < NUM_CANAIS; ch++) {
    analogSetPinAttenuation(PIN_FLEX[ch], ADC_11db);
  }

  Serial.println();
  Serial.println(F("=== KAIROS teste-flex-verifica ==="));
  Serial.println(F("Filtro = teste-flex (oversample + media truncada + OneEuro)."));

  while (!calibrarSistema()) {
    Serial.println(F("Reiniciando calibracao em 3s..."));
    delay(3000);
  }
  configurarLoRa();
  iniciarFiltros();
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'r' || c == 'R') {
      while (!calibrarSistema()) {
        Serial.println(F("Reiniciando calibracao em 3s..."));
        delay(3000);
      }
      iniciarFiltros();
    }
  }

  unsigned long agoraUs = micros();
  float dt = (agoraUs - tempoAnteriorUs) / 1000000.0f;
  tempoAnteriorUs = agoraUs;
  if (dt <= 0 || dt > 0.05f) dt = 0.01f;

  int raw[NUM_CANAIS];
  int ang[NUM_CANAIS];
  for (int ch = 0; ch < NUM_CANAIS; ch++) {
    processarCanal(ch, dt, raw[ch], ang[ch]);
  }

  if (millis() - ultimoTxMs >= TX_PERIODO_MS) {
    ultimoTxMs = millis();
    enviarLoRa(ang);
  }

  static unsigned long ultimoPrint = 0;
  if (millis() - ultimoPrint < 50) return;  // ~20 Hz no Serial
  ultimoPrint = millis();

  for (int ch = 0; ch < NUM_CANAIS; ch++) {
    Serial.printf("D%d:%3d(r%4d) ", ch + 1, ang[ch], raw[ch]);
  }
  Serial.printf("| tx=%ld falha=%ld\n", pacotesTx, falhasTx);
}
