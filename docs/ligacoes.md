# Ligações

## Luva — ESP32 (transmissor)

### MPU6050 (cada sensor no seu próprio barramento I²C)

Os três sensores usam o mesmo endereço (0x68, AD0 no GND). Por isso, cada um tem seu
próprio par SDA/SCL, criado em software.

| Sensor | SDA | SCL | VCC | GND |
|---|---|---|---|---|
| Braço | GPIO 25 | GPIO 26 | 3V3 | GND |
| Mão | GPIO 27 | GPIO 14 | 3V3 | GND |
| Antebraço | GPIO 4 | GPIO 13 | 3V3 | GND |

Posição na pessoa:
- **Braço:** na parte de cima do braço (úmero)
- **Antebraço:** no antebraço, perto do pulso
- **Mão:** nas costas da mão

### LoRa SX1278 (Ra-02)

| SX1278 | ESP32 |
|---|---|
| VCC | 3V3 (nunca 5 V) |
| GND | GND |
| NSS | GPIO 5 |
| RST | GPIO 2 |
| DIO0 | GPIO 15 |
| SCK | GPIO 18 |
| MISO | GPIO 19 |
| MOSI | GPIO 23 |

### Botões dos dedos (botão táctil de 4 terminais)

O botão de 4 terminais tem dois pares de pinos que já vêm ligados entre si. Ligue o
GPIO num terminal e o GND no terminal **diagonalmente oposto**. Assim você sempre pega
um terminal de cada par, seja qual for a orientação do botão. Os outros dois terminais
ficam soltos.

Não é preciso resistor: o firmware liga o pull-up interno (`INPUT_PULLUP`). Botão solto
lê HIGH (dedo aberto, 0°). Botão apertado lê LOW (dedo fechado, 180°). O debounce é de
25 ms.

| Dedo | GPIO | Outro terminal |
|---|---|---|
| Polegar | 32 | GND |
| Indicador | 33 | GND |
| Médio | 21 | GND |
| Anelar | 22 | GND |
| Mindinho | 16 | GND |

> Os GPIOs 34, 35, 36 e 39 do ESP32 **não têm pull-up interno**. Por isso, os pinos
> antigos dos flex (34, 35, 36) não servem para os botões.

```
 GPIO 32 ──┤ ┌─────┐ ├── (solto)
           │ │  ●  │ │
 (solto) ──┤ └─────┘ ├── GND
```

## Braço — Arduino Uno (receptor)

### LoRa SX1278

| SX1278 | Uno |
|---|---|
| VCC | 3V3 |
| GND | GND |
| SCK | D13 |
| MISO | D12 |
| MOSI | D11 |
| NSS | D10 |
| RST | D9 |
| DIO0 | não ligado (o receptor faz polling) |

### Drivers A4988

Em cada driver: ENABLE no GND, RESET e SLEEP ligados juntos em 5 V, VDD em 5 V, VMOT na
fonte dos motores (12 V) e GND comum com o Uno. MS1/MS2/MS3 soltos dão **passo inteiro**.

| Junta | STEP | DIR |
|---|---|---|
| Cotovelo | D3 | A0 |
| Ombro | A1 | A2 |
| Pulso | A3 | A4 |
| Base | A5 | D2 |

Sentido positivo (DIR em "frente"):
- **Base** (haste para cima): + = esquerda, − = direita, limite ±150°
- **Ombro** (haste para a esquerda): + = frente, − = trás, de −90° a 130°
- **Cotovelo** (haste para a direita): + = trás, − = frente, de −90° a 150°
- **Pulso** (haste para cima): + = esquerda, − = trás, ±180° (de palma para baixo a palma para cima)

### Servos MG996R (dedos)

| Dedo | Sinal |
|---|---|
| D1 (polegar) | D8 |
| D2 (indicador) | D4 |
| D3 (médio) | D5 |
| D4 (anelar) | D6 |
| D5 (mindinho) | D7 |

Alimente os servos com uma **fonte externa de 5 V** (cada MG996R chega a 2,5 A travado). O
GND da fonte precisa estar ligado ao GND do Uno.

## Corrente dos motores (Vref do A4988)

A corrente máxima por bobina é ajustada no potenciômetro do A4988:

\[ V_{ref} = 8 \cdot I_{max} \cdot R_{cs} \]

`R_cs` é o resistor de sensoriamento da placa. Nos módulos genéricos ele vem marcado
`R100` (0,1 Ω). Nos da Pololu, vem marcado `R068` (0,068 Ω).

| Corrente | Vref com R100 | Vref com R068 |
|---|---|---|
| 1,0 A | 0,80 V | 0,54 V |
| 1,2 A | 0,96 V | 0,65 V |
| 1,5 A | 1,20 V | 0,82 V |

Sem dissipador e ventilação, o A4988 aguenta cerca de 1 A por bobina. Para chegar perto
de 1,5 A (torque máximo de um NEMA17 comum), use dissipador e um ventilador. Meça o Vref
entre o potenciômetro e o GND, com a lógica alimentada e o motor parado.

## Melhorias de hardware recomendadas (pendentes)

- **Capacitor de 100 µF** (eletrolítico, ≥25 V) entre VMOT e GND, junto de cada A4988.
  Ele protege o driver dos picos de tensão da fonte.
- **Micropasso:** colocar MS2 em 5 V (1/4 de passo) reduz a vibração e a ressonância. Se
  fizer isso, multiplique `PASSOS_MOTOR` no receptor pelo mesmo fator e reajuste `VMAX`.
- **Fim de curso ou sensor Hall** em cada junta, para um home absoluto. Hoje o home é a
  pose em que o Uno ligou (malha aberta): se o motor perder passo, o zero se perde.
