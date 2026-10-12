# Detalhes técnicos

## Protocolo LoRa

Configuração igual nos dois lados:

| Parâmetro | Valor |
|---|---|
| Frequência | 400 MHz (o SX1278 cobre de 137 a 525 MHz) |
| Spreading factor | SF7 |
| Largura de banda | 250 kHz |
| Coding rate | 4/5 |
| CRC | ligado |
| SPI | 1 MHz (no Uno, a 8 MHz as escritas no SX1278 de 3,3 V corrompiam) |
| Potência | luva 14 dBm, Uno 10 dBm (o 3V3 do Uno só fornece ~150 mA) |
| OCP | 140 mA |

Com essa configuração, um pacote de 11 bytes leva cerca de 20 ms no ar.

### Pacotes

| Tipo | Bytes | Sentido |
|---|---|---|
| Pose | `[0xC2, cot, omb, pulso_lo, pulso_hi, base, d0, d1, d2, d3, d4]` | luva → braço, a cada 40 ms |
| PING | `[0xA1, seq]` | luva → braço |
| PONG | `[0xA2, seq, rssi]` | braço → luva |
| Comando | `[0xA4, tecla]` | luva → braço |

Codificação da pose:
- `cot`, `omb` e `base` são `int8` escalados: `valor = graus / fundo × 127`. Os fundos são
  150° (cotovelo), 130° (ombro) e 150° (base). Assim cabem em ±127.
- `pulso` é `int16` em graus, little-endian, de −180 a +180.
- `d0..d4` são 0 (aberto) ou 180 (fechado).

Comandos especiais em `[0xA4, x]`: `0x01` entra no manual, `0x00` sai do manual, `0x02`
liga o home e `0x03` desliga o home. Os demais valores são teclas ASCII do modo manual.

### Link

Com o link no ar, a luva repete o PING a cada ~1,2 s. Se faltarem 3 PONGs seguidos, ela
reinicia o rádio e tenta reconectar a cada 400 ms, sem parar de enviar a pose entre os
PINGs. No receptor, se não chegar pose por 800 ms, os motores param.

Os envios usam `endPacket(true)` (assíncrono). Antes de começar outro pacote, o firmware
verifica no registrador `RegOpMode` (`0x01 & 0x07 == 0x03`) se ainda há transmissão em
curso. Se o rádio ficar mais de 120 ms em TX, ele é tratado como travado e reiniciado.

No receptor, o PONG também é assíncrono (`pongPendente` / `pongNoAr`). Enquanto o PONG
está pendente ou no ar, o receptor não tenta ler pacotes.

## Como cada junta é calculada (luva)

Não há magnetômetro. Por isso, o "norte" de cada MPU deriva com o tempo. Cada junta usa
só o que ela consegue observar bem: a gravidade, o giro relativo entre dois sensores e a
restrição de dobradiça do cotovelo.

### Fusão de cada sensor
Filtro complementar: o giroscópio gira o vetor gravidade estimado, e o acelerômetro
corrige com peso 0,02. A correção é ignorada quando |a| foge de 1 g ± 0,15 g (o braço
está acelerando) ou quando o giro passa de 150 °/s (a força centrípeta engana o
acelerômetro).

### Ombro
Inclinação do osso do braço em relação à vertical medida na pose 1: 0° = para cima, 90° =
horizontal.

### Base
Rumo horizontal do braço, medido a partir da frente da pose 2, até ±150°. Com o braço
quase vertical o rumo não é definido; nesse caso, a base segura o último valor.

### Cotovelo
Ângulo entre o osso do braço e o osso do antebraço. A mão não entra na conta, então
dobrar ou girar a mão não muda o cotovelo.

**Restrição de dobradiça (`travarDobradica`).** O cotovelo é uma dobradiça: o antebraço
só se move no plano perpendicular ao eixo da articulação. Sem essa restrição, ao esticar
o braço à frente e girar para a esquerda, o erro de deriva entre os dois sensores
aparecia como cotovelo dobrando ("formava um L"). Agora, a cada amostra, a componente
do antebraço fora do plano é puxada de volta (5% do excesso por amostra), com folga de
sen(24°) para o ângulo de carregamento natural (~15°) e para o erro de colocação. Leituras
mais de ~45° fora do plano são descartadas. A ideia vem da literatura de IMU em
membros (restrições cinemáticas de junta, ver Laidig e Seel).

### Pulso
Giro da palma em torno do eixo do antebraço (mão relativa ao antebraço), de −180° a
+180°:
- Integra o giroscópio da mão no eixo de pronação medido na pose 6.
- Abaixo de 4 °/s, nada é integrado (é bias ou ruído; uma mão girando passa de 20 °/s).
- Com o braço parado e o antebraço deitado, a gravidade vista pela mão corrige
  lentamente a deriva (no máximo 2% do erro por amostra, e só se o erro for menor que
  20°).
- As poses A e B da calibração definem o zero (palma para baixo), o ganho e o sentido.

O motor do pulso tem `INV_TX[pulso] = true`: sem essa inversão, o robô girava ao
contrário da palma.

### Detecção de "parado" (ZUPT)
Só o acelerômetro não basta: um giro em torno da vertical não muda a gravidade. O
sensor só é considerado parado quando a gravidade não gira (cos > 0,9994, ~2°) **e** o
giro sem bias fica abaixo de 3 °/s durante 400 ms (como em Skog et al., 2010). Parado,
o bias do giroscópio é reaprendido aos poucos (`BIAS_K = 0.005`), com compensação de
temperatura.

## Controle dos motores (receptor)

- Passo inteiro, 200 passos/volta, redução 38,4:1 → 21,33 passos por grau.
- Perfil trapezoidal por eixo, com velocidade máxima `VMAX` e aceleração `ACEL` em
  passos/s e passos/s².
- O pulso tem limite de jerk (18 000 passos/s³), para a aceleração subir aos poucos.
- **Ressonância:** em passo inteiro, o NEMA17 ressoa entre ~100 e ~300 passos/s e perde
  passo. Os outros eixos atravessam essa faixa com 2,5× a aceleração, e a velocidade de
  cruzeiro fica acima dela.
- Laço de controle a cada 2 ms; zona morta de 0,4°.
- Servos com zona morta de 4°, para não ficarem tremendo.

| Eixo | VMAX (passos/s) | ACEL (passos/s²) | Limites |
|---|---|---|---|
| Cotovelo | 500 | 700 | −90° a 150° |
| Ombro | 420 | 600 | −90° a 130° |
| Pulso | 750 | 2200 | −180° a 180° |
| Base | 480 | 900 | −150° a 150° |

## Desempenho e configuração do ESP32

- O Wi-Fi e o Bluetooth são desligados no `setup` (`WiFi.mode(WIFI_OFF)`, `btStop()`).
- `Serial.setTxBufferSize(1024)`: com o buffer padrão, imprimir o status travava o loop.
- No Uno, o buffer de TX da serial tem só 64 bytes. Por isso, o status é montado com
  `snprintf` e enviado aos poucos (`availableForWrite`).
- **Dois núcleos:** a opção *Arduino Runs On* só muda o `loop()` de núcleo; continua sendo
  um núcleo só. Dividir em duas tarefas (sensores num núcleo, LoRa no outro) não deixa
  o braço mais preciso: o rádio já envia a cada 40 ms e os sensores são lidos bem mais
  rápido que isso. Além disso, dois núcleos mexendo no mesmo ângulo podem fazer a junta
  pular. Deixe *Arduino Runs On* e *Events Run On* em **Core 1**.

## Referências

- Skog, I. et al. *Zero-Velocity Detection — An Algorithm Evaluation.* IEEE Trans. Biomed. Eng., 2010.
- Seel, T.; Raisch, J.; Schauer, T. *IMU-Based Joint Angle Measurement for Gait Analysis.* Sensors, 2014.
- Laidig, D.; Schauer, T.; Seel, T. *Exploiting kinematic constraints to compensate magnetic disturbances when calculating joint angles of approximate hinge joints from orientation estimates of inertial sensors.* ICORR, 2017.
- Allegro A4988 datasheet (Vref = 8 · I · Rcs) e página do driver A4988 da Pololu.
- Semtech SX1276/77/78 datasheet (registradores e tempo no ar).
- [sandeepmistry/arduino-LoRa](https://github.com/sandeepmistry/arduino-LoRa)
