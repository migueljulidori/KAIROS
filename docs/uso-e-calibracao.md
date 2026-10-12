# Uso e calibração

## Gravação

### Bibliotecas
- `LoRa` (sandeepmistry) 0.8.0. Use a versão em `bibliotecas/LoRa`, que tem o timeout no `endPacket`.
- `Servo` (já vem com o Arduino IDE)
- `SPI`, `WiFi` e `math.h` (já vêm com os cores)

Os MPU6050 são lidos por um I²C em software dentro do próprio sketch. Não é preciso
instalar biblioteca de MPU.

### Placas
- Luva: core **esp32** da Espressif, placa *ESP32 Dev Module*, 240 MHz
- Braço: *Arduino Uno*

### Pela linha de comando (opcional)

```bash
A=/snap/arduino/current
P=$HOME/snap/arduino/current/.arduino15/packages
LIB=$HOME/snap/arduino/current/Arduino/libraries

# Luva (ESP32)
$A/arduino-builder -compile -hardware $A/hardware -hardware $P \
  -tools $A/tools-builder -tools $P -built-in-libraries $A/libraries -libraries $LIB \
  -fqbn esp32:esp32:esp32 -build-path /tmp/kairos-tx firmware/kairos-luva-tx/kairos-luva-tx.ino

# Braço (Uno)
$A/arduino-builder -compile -hardware $A/hardware \
  -tools $A/tools-builder -tools $A/hardware/tools/avr -built-in-libraries $A/libraries -libraries $LIB \
  -fqbn arduino:avr:uno -build-path /tmp/kairos-rx firmware/kairos-receptor/kairos-receptor.ino
```

Tamanhos da última compilação: luva com cerca de 939 KB de flash e receptor com 16 928
bytes (52% do Uno).

## Ligando o sistema

1. **Braço primeiro.** Antes de ligar o Uno, coloque o braço robótico com **todas as
   juntas para cima e a palma para trás**. A pose em que o Uno liga é o *home*: `h`
   sempre volta para ela, e nenhum comando a redefine.
2. **Luva.** Abra o monitor serial a 115200. O boot faz só dois testes:
   - os 3 MPU (OK ou FALHOU);
   - o link LoRa (ping/pong com RSSI dos dois lados).
3. No boot, a luva não calibra e não entra no modo manual sozinha. Os botões dos dedos
   já funcionam, mas as juntas ficam paradas até a calibração.

## Calibração (`r`)

Vista a luva. Em cada pose, fique parado: a pose é aceita depois de ~1,5 s sem
movimento. Se uma pose ficar fora do esperado (90° medido fora de 55° a 125°), o
firmware repete só aquela pose.

| Pose | Posição | O que mede |
|---|---|---|
| 1 | Tudo **para cima**, palma para **trás** | Zero do ombro, gravidade de referência e bias dos giroscópios |
| 2 | Braço **à frente**, na horizontal | Ombro +90°, frente da base (0°) |
| 3 | Braço 90° à **esquerda** | Base +90° |
| 4 | Braço 90° à **direita** | Base −90° |
| 5 | Braço à frente, **antebraço para cima** (dobrado) | Cotovelo +90° |
| 6 | **Gire a palma** por ~6 s | Eixo de pronação da mão |
| A | Antebraço deitado, **palma para baixo** | Zero do pulso |
| B | **Gire a palma para cima** (5 s) | Faixa e sentido do pulso (±180°) |
| 7 | Volte ao início: tudo para cima, palma para trás | Cotovelo 0 e pulso 0 |

Nas poses A e B, a tecla `p` pula a calibração do pulso. Sem ela, o pulso segue só o
giroscópio, com ganho 1.

Depois da pose 7, a luva passa a comandar o braço.

## Teclas no monitor serial da luva

| Tecla | Ação |
|---|---|
| `r` | Calibração completa (as poses acima) |
| `h` | Liga/desliga o **home**: o braço vai para o ponto em que o Uno ligou e ignora a luva. `h` de novo volta a seguir a luva |
| `z` | Zera pulso e base onde o braço está agora. Não muda o home do Uno |
| `n` | Entra/sai do **modo manual remoto** |

### Modo manual remoto (`n`)

As teclas são repassadas para o Uno pelo LoRa (`[0xA4, tecla]`). Enquanto isso, os
dedos continuam seguindo os botões.

| Tecla | Ação |
|---|---|
| `1` / `c` | Escolhe o cotovelo |
| `2` / `o` | Escolhe o ombro |
| `3` / `p` | Escolhe o pulso |
| `4` / `b` | Escolhe a base |
| `f` | Gira o eixo escolhido no sentido "frente" (positivo) |
| `t` | Gira no sentido "trás" (negativo) |
| `s` | Para |
| `h` | Home |
| `m` | Teste de motores e servos |
| `+` / `-` | Velocidade (de 0,2× a 2×) |
| `n` | Sai do manual (o braço volta a seguir a luva) |

## Teclas no monitor serial do receptor (Uno)

As mesmas do manual: `1`–`4` ou `c`/`o`/`p`/`b`, `f`, `t`, `s`, `m`, `h`, `+` e `-`. A
tecla `z` não redefine o home.

## Dedos

Botão apertado fecha o dedo (servo em 180°). Botão solto abre o dedo (servo em 0°).
Os dedos funcionam desde o boot, sem calibrar.

## Status

A luva imprime uma linha de status com os ângulos (cotovelo, ombro, pulso e base), os
dedos, o estado do link e a taxa do loop em Hz. O receptor imprime posição/alvo de cada
eixo, pacotes recebidos e PONGs, sem travar o loop: o texto sai aos poucos, conforme há
espaço no buffer da serial.
