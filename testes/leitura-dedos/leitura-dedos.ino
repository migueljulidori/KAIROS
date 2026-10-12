#include <WiFi.h>

// Teste dos flex: so ABERTO ou FECHADO, com calibracao guiada no boot.
// Mande 'c' no serial para calibrar de novo.
// Cadeia: miolo do burst -> rejeita pico -> mediana -> filtro 1 euro
//         -> limiares fixos da calibracao (com histerese) -> estado.
// Os limiares nao se ajustam sozinhos: o flex volta devagar enquanto fica
// dobrado, e um ajuste automatico acabaria aprendendo "fechado" como "aberto".

const bool DEBUG = true;     // true mostra os numeros de cada dedo

const int NUM_FLEX = 5;
const int DEDOS_ATIVOS = 4;  // mindinho fora por enquanto
const int PIN_FLEX[NUM_FLEX] = { 34, 35, 32, 33, 36 };
const char* NOME_DEDO[NUM_FLEX] = { "Polegar", "Indicador", "Medio", "Anelar", "Mindinho" };

// Calibracao
const int CONTAGEM_S = 10;         // antes da primeira pose
const int TROCA_S = 5;             // entre uma pose e outra
const unsigned long MEDE_MS = 2000;
const float SPAN_MIN = 8.0f;       // |dobra| menor que isso = sensor nao responde

// Fracao da dobra calibrada (0 = aberto, 1 = fechado da calibracao).
// ABRE fica perto do aberto porque o flex segurado dobrado volta sozinho.
const float FRAC_ABRE = 0.30f;
const float FRAC_FECHA = 0.55f;
const float FAIXA_MIN = 0.20f;     // distancia minima entre os dois limiares
const float FRAC_FECHA_MAX = 0.85f;
const float SIGMAS = 3.0f;         // limiar fica a 3 desvios do ruido do aberto
const unsigned long CONFIRMA_MS = 200;

// Leitura
const int DESCARTA = 8;      // o ADC mistura o pino anterior nas primeiras leituras
const int BURST = 64;
const int GAP_US = 100;
const int MED_N = 5;
const int PICO = 140;
const int PICO_SEGUIDOS = 3;

// Filtro 1 euro (Casiez 2012)
const float MIN_CUTOFF = 0.6f;
const float BETA = 0.03f;
const float D_CUTOFF = 1.0f;

float x[NUM_FLEX];
float dxHat[NUM_FLEX];
float aberto[NUM_FLEX];
float span[NUM_FLEX];
float sinal[NUM_FLEX];    // +1: leitura cai ao dobrar, -1: leitura sobe ao dobrar
float fracAbre[NUM_FLEX];
float fracFecha[NUM_FLEX];
bool fechado[NUM_FLEX];
bool valido[NUM_FLEX];
bool ok[NUM_FLEX];
int rejeitados[NUM_FLEX];
unsigned long tAnt[NUM_FLEX];
unsigned long condDesde[NUM_FLEX];

int medBuf[NUM_FLEX][MED_N];
int medIdx[NUM_FLEX];
int medCheio[NUM_FLEX];

int mediaDoMiolo(int ch) {
  int s[BURST];
  for (int i = 0; i < DESCARTA; i++) analogRead(PIN_FLEX[ch]);
  for (int i = 0; i < BURST; i++) {
    s[i] = analogRead(PIN_FLEX[ch]);
    delayMicroseconds(GAP_US);
  }
  for (int i = 1; i < BURST; i++) {
    int key = s[i];
    int j = i - 1;
    while (j >= 0 && s[j] > key) {
      s[j + 1] = s[j];
      j--;
    }
    s[j + 1] = key;
  }
  const int a = BURST / 4;
  const int b = BURST - BURST / 4;
  long soma = 0;
  for (int i = a; i < b; i++) soma += s[i];
  return (int)(soma / (b - a));
}

int medianaDe(int ch) {
  int tmp[MED_N];
  int n = medCheio[ch] ? MED_N : medIdx[ch];
  if (n < 1) n = 1;
  for (int i = 0; i < n; i++) tmp[i] = medBuf[ch][i];
  for (int i = 1; i < n; i++) {
    int key = tmp[i];
    int j = i - 1;
    while (j >= 0 && tmp[j] > key) {
      tmp[j + 1] = tmp[j];
      j--;
    }
    tmp[j + 1] = key;
  }
  return tmp[n / 2];
}

void empurraMed(int ch, int v) {
  medBuf[ch][medIdx[ch]] = v;
  medIdx[ch]++;
  if (medIdx[ch] >= MED_N) {
    medIdx[ch] = 0;
    medCheio[ch] = 1;
  }
}

float alphaLp(float cutoff, float dt) {
  float tau = 1.0f / (2.0f * PI * cutoff);
  return 1.0f / (1.0f + tau / dt);
}

void filtrar(int ch) {
  unsigned long agora = micros();
  float dt = ok[ch] ? (agora - tAnt[ch]) * 1e-6f : 0.04f;
  if (dt <= 0.0f || dt > 0.5f) dt = 0.04f;
  tAnt[ch] = agora;

  int limpo = mediaDoMiolo(ch);
  if (ok[ch] && abs(limpo - (int)x[ch]) > PICO && rejeitados[ch] < PICO_SEGUIDOS) {
    rejeitados[ch]++;
  } else {
    rejeitados[ch] = 0;
    empurraMed(ch, limpo);
  }
  float med = medianaDe(ch);

  if (!ok[ch]) {
    x[ch] = med;
    dxHat[ch] = 0;
    ok[ch] = true;
    return;
  }

  float dx = (med - x[ch]) / dt;
  dxHat[ch] += alphaLp(D_CUTOFF, dt) * (dx - dxHat[ch]);
  float cutoff = MIN_CUTOFF + BETA * fabsf(dxHat[ch]);
  x[ch] += alphaLp(cutoff, dt) * (med - x[ch]);
}

void filtrarTodos() {
  for (int ch = 0; ch < DEDOS_ATIVOS; ch++) filtrar(ch);
}

void contagem(const __FlashStringHelper* pose, int segundos) {
  Serial.print(pose);
  Serial.print(F(" em "));
  for (int s = segundos; s > 0; s--) {
    Serial.print(s);
    Serial.print(' ');
    unsigned long t0 = millis();
    while (millis() - t0 < 1000) filtrarTodos();
  }
  Serial.println();
}

void medirPose(float media[], float desvio[]) {
  double soma[NUM_FLEX] = { 0 };
  double soma2[NUM_FLEX] = { 0 };
  long n = 0;
  unsigned long t0 = millis();
  while (millis() - t0 < MEDE_MS) {
    filtrarTodos();
    for (int ch = 0; ch < DEDOS_ATIVOS; ch++) {
      soma[ch] += x[ch];
      soma2[ch] += (double)x[ch] * x[ch];
    }
    n++;
  }
  for (int ch = 0; ch < DEDOS_ATIVOS; ch++) {
    if (n < 2) {
      media[ch] = x[ch];
      desvio[ch] = 0;
      continue;
    }
    double m = soma[ch] / n;
    double var = soma2[ch] / n - m * m;
    media[ch] = m;
    desvio[ch] = var > 0 ? sqrt(var) : 0;
  }
  Serial.println(F("  ok, medido."));
}

void calibrar() {
  float ab1[NUM_FLEX], fe[NUM_FLEX], ab2[NUM_FLEX];
  float dAb1[NUM_FLEX], dFe[NUM_FLEX], dAb2[NUM_FLEX];

  Serial.println();
  Serial.println(F("=== CALIBRACAO ==="));
  contagem(F("Deixe a MAO ABERTA, esticada e parada."), CONTAGEM_S);
  medirPose(ab1, dAb1);
  contagem(F("FECHE a mao e segure."), TROCA_S);
  medirPose(fe, dFe);
  // O flex para mais baixo depois de dobrar; este e o aberto que vale.
  contagem(F("ABRA a mao de novo e segure."), TROCA_S);
  medirPose(ab2, dAb2);

  Serial.println(F("Dedo       aberto fechado dobra ruido  abre fecha"));
  int bons = 0;
  for (int ch = 0; ch < DEDOS_ATIVOS; ch++) {
    float dobra = ab2[ch] - fe[ch];
    sinal[ch] = dobra >= 0 ? 1.0f : -1.0f;
    span[ch] = fabsf(dobra);
    aberto[ch] = ab2[ch];
    valido[ch] = span[ch] >= SPAN_MIN;
    fechado[ch] = false;
    condDesde[ch] = millis();

    // Ruido de referencia: o maior entre as poses paradas.
    float ruido = max(dAb2[ch], max(dAb1[ch], dFe[ch]));
    float fa = FRAC_ABRE;
    if (valido[ch] && SIGMAS * ruido / span[ch] > fa) fa = SIGMAS * ruido / span[ch];
    float ff = max(FRAC_FECHA, fa + FAIXA_MIN);
    bool apertado = ff > FRAC_FECHA_MAX;
    if (apertado) {
      ff = FRAC_FECHA_MAX;
      fa = min(fa, ff - FAIXA_MIN);
    }
    fracAbre[ch] = fa;
    fracFecha[ch] = ff;

    const char* aviso = "";
    if (!valido[ch]) aviso = "<- NAO DOBRA, ignorado";
    else if (apertado) aviso = "<- ruido alto p/ a dobra, pode oscilar";
    else bons++;
    Serial.printf("%-9s  %6d  %6d  %4d  %4.1f  %3d%%  %3d%% %s\n", NOME_DEDO[ch],
                  (int)ab2[ch], (int)fe[ch], (int)dobra, ruido,
                  (int)(fa * 100), (int)(ff * 100), aviso);
  }
  Serial.printf("%d de %d dedos com boa margem. Mande 'c' para calibrar de novo.\n\n",
                bons, DEDOS_ATIVOS);
}

float fracDobra(int ch) {
  return sinal[ch] * (aberto[ch] - x[ch]) / span[ch];
}

// Retorna true se o dedo trocou de estado nesta chamada.
bool decidir(int ch) {
  if (!valido[ch]) return false;
  unsigned long ms = millis();
  float f = fracDobra(ch);

  bool querTrocar = fechado[ch] ? (f < fracAbre[ch]) : (f > fracFecha[ch]);
  if (!querTrocar) {
    condDesde[ch] = ms;
    return false;
  }
  if (ms - condDesde[ch] < CONFIRMA_MS) return false;
  condDesde[ch] = ms;
  fechado[ch] = !fechado[ch];
  return true;
}

void imprime() {
  for (int ch = 0; ch < DEDOS_ATIVOS; ch++) {
    const char* txt = !valido[ch] ? "---    " : (fechado[ch] ? "FECHADO" : "aberto ");
    if (DEBUG) {
      Serial.printf("%s x=%4d dobra=%3d%% %s", NOME_DEDO[ch], (int)x[ch],
                    (int)(fracDobra(ch) * 100), txt);
    } else {
      Serial.printf("%-9s %s", NOME_DEDO[ch], txt);
    }
    if (ch < DEDOS_ATIVOS - 1) Serial.print(F(" | "));
  }
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  delay(300);
  WiFi.mode(WIFI_OFF);
  analogReadResolution(12);
  for (int ch = 0; ch < NUM_FLEX; ch++) {
    analogSetPinAttenuation(PIN_FLEX[ch], ADC_11db);
    ok[ch] = false;
    rejeitados[ch] = 0;
    medIdx[ch] = 0;
    medCheio[ch] = 0;
    valido[ch] = false;
    sinal[ch] = 1.0f;
    span[ch] = SPAN_MIN;
  }
  calibrar();
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'c' || c == 'C') calibrar();
  }

  filtrarTodos();
  bool mudou = false;
  for (int ch = 0; ch < DEDOS_ATIVOS; ch++) {
    if (decidir(ch)) mudou = true;
  }

  static unsigned long ultimoPrint = 0;
  if (mudou || millis() - ultimoPrint >= 500) {
    ultimoPrint = millis();
    imprime();
  }
}
