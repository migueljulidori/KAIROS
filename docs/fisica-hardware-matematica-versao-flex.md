# Hardware

> ⚙️ Página técnica do braço teleoperado — do movimento da mão até o motor no robô.

---

**Ideia geral:**

A pessoa veste uma luva com sensores. A luva mede como o braço e os dedos estão, transforma isso em números, e manda esses números por rádio LoRa. Do outro lado, um Arduino Uno recebe o pacote e move motores de passo (braço) e servos (dedos) para copiar o movimento.

A linha de raciocínio da página é esta:

1. O que existe em cada ponta (luva e braço)
2. Como o flex vira ângulo de dedo
3. Como o MPU vira ângulo de junta
4. Como o ângulo vira passo de motor
5. Como o rádio carrega tudo isso
6. Glossário dos símbolos

---

**Visão do sistema:**

Na luva (transmissor) usamos um ESP32. Nele ficam três MPU6050 (braço, antebraço e mão) e cinco sensores flex (um por dedo). O ESP32 lê tudo, calcula os ângulos e envia um pacote LoRa a cada 40 ms.

No braço (receptor) usamos um Arduino Uno. Ele recebe o pacote, manda quatro motores NEMA17 (cotovelo, ombro, pulso, base) via drivers A4988, e cinco servos MG996R nos dedos.

Não usamos magnetômetro. Por isso o “norte” absoluto de cada MPU deriva com o tempo. A solução é simples: cada junta usa só o que ela consegue observar bem — gravidade, giro relativo, dobradiça entre dois sensores — sem misturar o rumo 6D dos três MPUs.

**Resumo em tópicos:**

- Luva = ESP32 + 3× MPU6050 + 5× flex + LoRa SX1278
- Braço = Uno + 4× NEMA17/A4988 + 5× MG996R + LoRa SX1278
- Fluxo = sensor → filtro → ângulo → pacote 0xC2 → atuador
- Cada junta tem a própria conta; não há um “IMU único” do corpo inteiro

---

**Luva — sensores de dedo (flex):**

O flex é um sensor cuja resistência muda quando o dedo dobra. Para o microcontrolador ler isso, montamos um divisor de tensão: o flex fica em série com um resistor fixo de 66 kΩ.

Importante: no ESP32 a alimentação do divisor é **3,3 V**. O ADC do ESP32 não tolera 5 V — se ligar em 5 V, pode queimar o pino.

Ligação de cada canal:

- Um terminal do flex → 3V3
- O outro terminal do flex → GPIO do ESP32 e também no resistor 66 kΩ
- O outro lado do resistor → GND
- A junção (GPIO) é o que o ADC lê

Pinos usados:

- D1 → GPIO 34
- D2 → GPIO 35
- D3 → GPIO 32
- D4 → GPIO 33
- D5 → GPIO 36 (VP)

Quando o dedo está reto, a resistência do flex é menor (~25 kΩ). Quando dobra, sobe (~100 kΩ). Como o resistor fixo está embaixo (ligado ao GND), ao dobrar a tensão no GPIO cai.

**Conta do divisor:**

Vadc = 3,3 × R / (Rf + R)

- Vadc = tensão que o ESP32 lê no GPIO (0 a 3,3 V)
- 3,3 = alimentação do circuito, em volts
- Rf = resistência do flex (muda ao dobrar)
- R = resistor fixo de 66 kΩ

Em palavras: a fração R / (Rf + R) da alimentação aparece no ADC. Se Rf sobe, essa fração desce, e Vadc desce.

Exemplos típicos no ESP32 (ADC de 12 bits, 0…4095):

- Reto: Rf ≈ 25 kΩ → Vadc ≈ 2,39 V → contagem ≈ 2970
- Dobrado: Rf ≈ 100 kΩ → Vadc ≈ 1,31 V → contagem ≈ 1630

Se a leitura fica entre 0 e 200 com picos aleatórios, o pino está flutuando: falta 3V3, GND ou a junção do divisor. Não é erro de calibração.

**Do valor bruto ao ângulo do dedo:**

ângulo = map(raw, flexMax, flexMin, 0, 180)

- ângulo = posição do dedo em graus (0° aberto, 180° fechado)
- raw = leitura do ADC depois da filtragem
- flexMax = mediana com dedo reto (tensão alta)
- flexMin = mediana com dedo dobrado (tensão baixa)
- map = escala linear de um intervalo para outro (função do Arduino)

Antes do map, o firmware limpa o sinal nesta ordem:

raw → média de 16 leituras → média truncada → filtro One Euro → deadband → ângulo

- Oversample: reduz ruído elétrico do ADC
- Média truncada: joga fora picos isolados
- One Euro: suave quando parado, rápido quando o dedo realmente mexe
- Deadband: ignora mudanças pequenas demais (evita tremor no servo)

---

**Luva — sensores de pose (MPU6050):**

Cada MPU6050 é uma IMU: entrega aceleração (a) e velocidade angular (ω, em °/s). A partir da aceleração estimamos a direção da gravidade no sensor (ĝ). Com o giroscópio sabemos quão rápido o segmento está girando.

Temos três MPUs:

- Braço → ombro e base
- Antebraço → cotovelo
- Mão → pulso (pronação/supinação)

**Símbolos que aparecem nas contas:**

- a = aceleração (em g; 1 g ≈ 9,8 m/s²)
- ω (ômega) = velocidade angular (°/s) — “quão rápido gira”
- ĝ (g chapéu) = direção da gravidade (vetor unitário)
- û (u chapéu) = eixo longitudinal do segmento (calibrado na pose zero)
- ĵ (j chapéu) = eixo da dobradiça Seel entre dois segmentos
- · = produto escalar (“quanto dois vetores apontam na mesma direção”)
- θ (teta) = ângulo da junta em graus
- ψ (psi) = azimute da base (giro em torno da vertical)
- ψ̇ (psi ponto) = velocidade desse azimute (°/s)
- Δt (delta t) = intervalo de tempo entre duas leituras

**Ombro:**

O ombro é o ângulo entre o eixo do braço e a gravidade.

θombro = arccos(û · ĝ)

- arccos transforma o produto escalar (−1…1) em ângulo (0°…180°)
- Braço vertical para cima → ≈ 0°
- Braço horizontal → ≈ 90°

Na calibração medimos θ com o braço em cima e na horizontal e calculamos um ganho:

G = 90° / (θhoriz − θcima)

Esse G só escala o ângulo bruto para bater com o mundo real.

**Cotovelo:**

O cotovelo é uma dobradiça entre braço e antebraço (modelo Seel). Pegamos o eixo da dobradiça em cada sensor e comparamos os giros:

ωcot = ĵante · ωante − ĵbraco · ωbraco

A subtração remove o giro comum (ombro/tronco) e deixa só a dobradiça.

Depois integramos no tempo e corrigimos com o acelerômetro (filtro complementar):

θ(t+Δt) = θ(t) + ωcot·Δt + Kp·wacc·(θacc − θ)·Δt

- θ(t) = ângulo agora
- ωcot·Δt = quanto o giro acrescentou neste intervalo
- θacc = ângulo estimado só pelo acelerômetro (referência absoluta, mais lenta)
- Kp = quão forte puxamos o ângulo em direção ao accel
- wacc = confiança no accel (0 a 1); cai quando |a| ≠ 1 g, porque aí tem aceleração linear misturada

**Pulso:**

Pronação/supinação = giro relativo no eixo longitudinal da mão vs antebraço:

ωpulso = ûmao · ωmao − ûante · ωante

Com o antebraço inclinado, a projeção da gravidade no plano transversal ainda dá uma referência absoluta de torção.

**Base:**

A base gira em torno da vertical. Aproximamos a velocidade por:

ψ̇ ≈ ωbraco · ĝ

Ou seja: a parte do giro do braço que aponta na direção da gravidade. Os três MPUs só entram juntos se concordarem que o tronco está girando; senão usamos só o braço. Perto da vertical o azimute fica singular — aí a correção fica mais cuidadosa.

---

**Braço — motores de passo:**

Cada junta do braço (exceto os dedos) usa um NEMA17 com caixa de redução 38,4:1, comandado por um A4988 em full step.

Conta dos passos:

- 200 passos por volta do rotor (full step)
- Redução 38,4:1 → 200 × 38,4 = 7680 passos por volta da junta
- passos/grau = 7680 / 360 = 21,333…

Exemplo: 90° de cotovelo → 90 × 21,333 = 1920 passos.

A fonte dos motores é 12 V. Acima de certa velocidade (“joelho” da curva de torque) o motor perde força; trabalhamos em ~70–90% desse limite:

- Cotovelo: vmax 500 passos/s, accel 2000
- Ombro: 380 / 1500
- Pulso: 550 / 2200
- Base: 450 / 1800

**Perfil com frenagem:**

O alvo muda o tempo todo (teleoperação). Para não ultrapassar o alvo, calculamos a velocidade máxima ainda capaz de frear a tempo:

e = alvo − posição
vparar = √(2 · a · |e|)
vdes = sinal(e) · min(vmax, vparar)

- e = quanto falta (erro)
- a = aceleração/desaceleração do perfil
- vparar = teto de velocidade para conseguir parar no alvo
- vdes = velocidade desejada neste instante
- sinal(e) = sentido do movimento
- min = não passa nem de vmax nem de vparar

Limites mecânicos típicos (graus): cotovelo 0…150, ombro 0…130, pulso −110…110, base −150…150. Deadzone ~1,5° evita tremor parado.

**Pinos A4988 no Uno:**

ENABLE no GND; RESET e SLEEP no 5 V.

- Cotovelo: STEP 3, DIR A0
- Ombro: STEP A1, DIR A2
- Pulso: STEP A3, DIR A4
- Base: STEP A5, DIR 2

**Dedos — servos MG996R:**

Cinco servos, PWM de 0° a 180°, alimentados com 5 V externo (GND comum com o Uno).

- D1 → pino 8
- D2 → pino 4
- D3 → pino 5
- D4 → pino 6
- D5 → pino 7

Não usar 9, 10, 11–13 (LoRa/SPI) nem os pinos dos A4988.

---

**Rádio LoRa:**

O elo entre luva e braço é o SX1278 em 433 MHz (SF7, BW 250 kHz, CR 4/5, CRC ligado). O TX manda a cada 40 ms (25 Hz). Airtime típico ~15 ms → o rádio fica no ar cerca de 37% do tempo.

Pacote unificado **0xC2** (10 bytes):

- Byte 0 = header 0xC2
- Bytes 1…4 = cotovelo, ombro, pulso, base (int8, graus)
- Bytes 5…9 = dedos D1…D5 (uint8, 0…180)

Backups antigos: 0xC1 = só braço; 0xB5 = só flex. No RX, se passar 800 ms sem pacote, o alvo congela (não “voa” sozinho).

Pinos LoRa:

- ESP32: NSS 5, RST 2, DIO0 15, SCK/MISO/MOSI 18/19/23
- Uno: NSS 10, RST 9, DIO0 solto, SCK/MISO/MOSI 13/12/11
- VCC do módulo: 3V3 nos dois lados

**Soft-I2C dos MPUs no ESP32:**

Cada MPU tem o próprio barramento (mesmo endereço 0x68, AD0 no GND, VCC 3V3):

- Mão: SDA 27, SCL 14
- Antebraço: SDA 25, SCL 26
- Braço: SDA 4, SCL 13

---

**Exemplos numéricos:**

1. Tempo do ombro em 90° a 380 passos/s  
   t = (90 × 21,333) / 380 ≈ 5,05 s  
   (sem rampa; na prática um pouco mais)

2. Resolução do flex com faixa útil de 800 counts  
   180° / 800 = 0,225° por count

3. Duty cycle LoRa  
   15 ms de airtime em período de 40 ms → ~37% no ar a 25 Hz

---

**Glossário rápido:**

Letras gregas

- θ (teta) = ângulo da junta
- ω (ômega) = velocidade angular
- ψ (psi) = azimute da base
- ψ̇ = velocidade do azimute
- Δ = variação / intervalo (Δt = passo de tempo)

Chapéus e vetores

- â = vetor unitário (só direção)
- · = produto escalar
- |…| = módulo / valor absoluto

Flex e ADC

- Rf, R, Vadc, raw, flexMax, flexMin — ver seção dos dedos

Motores

- e, vmax, vparar, vdes, a, G, Kp, wacc — ver seções de ombro/cotovelo/perfil

Termos

- Divisor = flex + 66 kΩ; a junção é o que o ADC lê
- ADC = conversor analógico-digital (Uno 10-bit; ESP32 12-bit)
- Seel = junta de 1 eixo (giro + gravidade)
- Complementar = giro rápido + correção pelo accel
- One Euro = filtro suave parado e rápido em movimento
- Deadzone = erro pequeno demais para atualizar o alvo

---

**Sketches:**

- TX unificado → pasta kairos-luva-tx (header 0xC2)
- RX unificado → pasta kairos-receptor (header 0xC2)
- Backup MPU → teste-mpu (0xC1)
- Backup flex → teste-flex (0xB5)
- Backup braço → teste-braco-4motores (0xC1)
