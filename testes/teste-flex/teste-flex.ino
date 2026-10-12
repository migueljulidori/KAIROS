/*
 * Kairós — teste-flex (ESP32 TX — 5 flex + LoRa)
 *
 * FLEX (repetir para dedos 1-5, divisor 66kΩ, 3V3):
 *   D1: GPIO 34 | D2: GPIO 35 | D3: GPIO 32
 *   D4: GPIO 33 | D5: GPIO 36
 *
 * LoRa SX1278 433MHz:
 *   3V3, GND, SCK=18, MISO=19, MOSI=23
 *   NSS=5, RST=2, DIO0=15, ANT=433MHz
 *
 * Protocolo LoRa: 6 bytes [0xB5, ang0..ang4]
 * Calibracao semiautomatica por dedo ao ligar.
 */

#include <WiFi.h>
#include <SPI.h>
#include <LoRa.h>

const int NUM_CANAIS = 5;

const int PIN_FLEX[NUM_CANAIS] = { 34, 35, 32, 33, 36 };

#define LORA_SS   5
#define LORA_RST  2
#define LORA_DIO0 15

const long LORA_FREQ = 433E6;
const uint8_t LORA_HEADER = 0xB5;

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
  "Polegar",
  "Indicador",
  "Medio",
  "Anelar",
  "Mindinho"
};

const int TOTAL_PASSOS_CAL = NUM_CANAIS * 2;

void configurarLoRa() {
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);

  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println("ERRO: LoRa nao iniciou.");
    while (true) delay(1000);
  }

  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(250E3);
  LoRa.setCodingRate4(5);
  LoRa.enableCrc();
  LoRa.setTxPower(17);

  Serial.println("LoRa TX 5ch (433 MHz, SF7, BW250)");
}

void enviarAngulosLoRa(const uint8_t angulos[NUM_CANAIS]) {
  LoRa.beginPacket();
  LoRa.write(LORA_HEADER);
  for (int ch = 0; ch < NUM_CANAIS; ch++) {
    LoRa.write(angulos[ch]);
  }
  LoRa.endPacket();
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
  Serial.println("Preparando leitura em...");
  for (int s = 3; s >= 1; s--) {
    Serial.println(s);
    delay(1000);
  }
}

void previewLeitura(int ch) {
  Serial.println("Leitura ao vivo (somente este dedo):");
  for (int i = 0; i < 8; i++) {
    Serial.print("  ");
    Serial.println(lerFlex(ch));
    delay(200);
  }
}

int calibrarPasso(int ch, int numeroPasso, const char* posicao) {
  Serial.println();
  Serial.println("----------------------------------------");
  Serial.print("PASSO ");
  Serial.print(numeroPasso);
  Serial.print(" de ");
  Serial.println(TOTAL_PASSOS_CAL);
  Serial.print("Dedo ");
  Serial.print(ch + 1);
  Serial.print(" — ");
  Serial.println(NOME_DEDO[ch]);
  Serial.print("Posicao: ");
  Serial.println(posicao);
  Serial.println("Calibre SOMENTE este dedo. Mantenha os outros parados.");
  Serial.println();

  previewLeitura(ch);
  aguardarContagem();

  int valor = medirMedia(ch);

  Serial.print("Valor salvo para D");
  Serial.print(ch + 1);
  Serial.print(" (");
  Serial.print(posicao);
  Serial.print("): ");
  Serial.println(valor);
  Serial.println("----------------------------------------");

  return valor;
}

bool calibrarSistema() {
  Serial.println();
  Serial.println("=== KAIROS — CALIBRACAO DEDO A DEDO ===");
  Serial.println("Serao 5 dedos, um por vez.");
  Serial.println("Cada dedo: primeiro RETO, depois DOBRADO.");
  Serial.println();
  delay(1000);

  int passo = 1;

  for (int ch = 0; ch < NUM_CANAIS; ch++) {
    Serial.println();
    Serial.print(">>> Iniciando calibracao do dedo ");
    Serial.print(ch + 1);
    Serial.print(" (");
    Serial.print(NOME_DEDO[ch]);
    Serial.println(") <<<");
    delay(1500);

    flexMax[ch] = calibrarPasso(ch, passo++, "RETO");

    flexMin[ch] = calibrarPasso(ch, passo++, "DOBRADO NO MAXIMO");

    if (flexMax[ch] <= flexMin[ch]) {
      Serial.print("ERRO no dedo ");
      Serial.print(ch + 1);
      Serial.println(": valor RETO deve ser MAIOR que DOBRADO.");
      return false;
    }

    if ((flexMax[ch] - flexMin[ch]) < FAIXA_MINIMA) {
      Serial.print("ERRO no dedo ");
      Serial.print(ch + 1);
      Serial.println(": diferenca muito pequena entre reto e dobrado.");
      return false;
    }

    Serial.print("Dedo ");
    Serial.print(ch + 1);
    Serial.println(" calibrado com sucesso!");
    delay(1000);
  }

  Serial.println();
  Serial.println("=== CALIBRACAO OK — TODOS OS DEDOS ===");
  for (int ch = 0; ch < NUM_CANAIS; ch++) {
    Serial.print("D");
    Serial.print(ch + 1);
    Serial.print(" ");
    Serial.print(NOME_DEDO[ch]);
    Serial.print(" | reto=");
    Serial.print(flexMax[ch]);
    Serial.print(" | dobrado=");
    Serial.println(flexMin[ch]);
  }
  Serial.println();
  Serial.println("Luva pronta! Movimente os dedos.");
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
    valorFiltrado[ch] = inicial;
    flexEstavel[ch]   = inicial;
    flexPublicado[ch] = -1;
    filtroInit[ch]    = true;
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
  Serial.begin(9600);
  delay(300);

  WiFi.mode(WIFI_OFF);

  analogReadResolution(12);
  for (int ch = 0; ch < NUM_CANAIS; ch++) {
    analogSetPinAttenuation(PIN_FLEX[ch], ADC_11db);
  }

  while (!calibrarSistema()) {
    Serial.println("Reiniciando calibracao em 3s...");
    delay(3000);
  }

  configurarLoRa();
  iniciarFiltros();
}

void loop() {
  unsigned long agoraUs = micros();
  float dt = (agoraUs - tempoAnteriorUs) / 1000000.0f;
  tempoAnteriorUs = agoraUs;

  uint8_t angulos[NUM_CANAIS];
  int raw[NUM_CANAIS];
  int ang;

  for (int ch = 0; ch < NUM_CANAIS; ch++) {
    processarCanal(ch, dt, raw[ch], ang);
    angulos[ch] = (uint8_t)ang;
  }

  enviarAngulosLoRa(angulos);

  for (int ch = 0; ch < NUM_CANAIS; ch++) {
    Serial.print("D");
    Serial.print(ch + 1);
    Serial.print(":");
    Serial.print(angulos[ch]);
    if (ch < NUM_CANAIS - 1) Serial.print(" ");
  }
  Serial.println();
}
