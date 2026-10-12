# Kairós — braço robótico teleoperado por luva

O Kairós é um braço robótico que copia o movimento do braço e da mão de uma pessoa.
A pessoa veste uma **luva** com três sensores de movimento (MPU6050) e cinco botões
(um por dedo). A luva calcula os ângulos das juntas e envia esses valores por rádio
**LoRa** para um **Arduino Uno**. O Uno move quatro motores de passo NEMA17 (base,
ombro, cotovelo e pulso) e cinco servos MG996R (os dedos).

```
 LUVA (ESP32)                                        BRAÇO (Arduino Uno)
 ┌──────────────────────────┐   LoRa 400 MHz      ┌────────────────────────────┐
 │ MPU6050 Braço            │   pacote 11 bytes   │ 4x A4988 + NEMA17          │
 │ MPU6050 Antebraço        │ ──────────────────► │   cotovelo, ombro,         │
 │ MPU6050 Mão              │   a cada 40 ms      │   pulso, base              │
 │ 5 botões (dedos)         │ ◄────────────────── │ 5x servo MG996R (dedos)    │
 │ SX1278                   │   PONG (link vivo)  │ SX1278                     │
 └──────────────────────────┘                     └────────────────────────────┘
```

## Estrutura do repositório

| Pasta | Conteúdo |
|---|---|
| `firmware/kairos-luva-tx/` | Firmware oficial da luva (ESP32, transmissor) |
| `firmware/kairos-receptor/` | Firmware oficial do braço (Arduino Uno, receptor) |
| `bibliotecas/LoRa/` | Biblioteca LoRa 0.8.0 (sandeepmistry) **com uma correção local** ([ver abaixo](#biblioteca-lora-modificada)) |
| `versoes-anteriores/` | Backups: luva com flex, antes do pulso novo, antes dos botões, receptor antigo |
| `testes/` | Sketches de teste usados durante o desenvolvimento (motores, MPU, LoRa, flex, servos, pulso) |
| `docs/` | Documentação detalhada e diagrama (draw.io) |

## Documentação

- [`docs/ligacoes.md`](docs/ligacoes.md) — ligação de cada pino (luva e braço), botões, A4988 e Vref
- [`docs/uso-e-calibracao.md`](docs/uso-e-calibracao.md) — como gravar, ligar, calibrar e usar (teclas `r`, `n`, `h`, `z`...)
- [`docs/tecnico.md`](docs/tecnico.md) — protocolo LoRa, como cada junta é calculada, controle dos motores, decisões e pesquisa
- [`docs/historico.md`](docs/historico.md) — problemas encontrados e como foram resolvidos
- [`docs/fisica-hardware-matematica-versao-flex.md`](docs/fisica-hardware-matematica-versao-flex.md) — página técnica (versão anterior, ainda com sensores flex)
- [`docs/diagramas/kairos-esquema-e-blocos.drawio`](docs/diagramas/kairos-esquema-e-blocos.drawio) — esquema e diagrama de blocos (abrir em [diagrams.net](https://app.diagrams.net))

## Hardware

**Luva (TX)**
- ESP32 DevKit
- 3× MPU6050, cada um no seu próprio barramento I²C: Braço, Antebraço, Mão
- 5× botão táctil de 4 terminais (dedos)
- LoRa SX1278 (Ra-02), 400 MHz

**Braço (RX)**
- Arduino Uno
- 4× NEMA17 + driver A4988 (passo inteiro, redução 38,4:1 → 21,33 passos por grau)
- 5× servo MG996R (alimentação externa de 5 V, GND comum)
- LoRa SX1278 (Ra-02), 400 MHz

## Início rápido

1. Instale a biblioteca `bibliotecas/LoRa` na pasta `libraries` do Arduino (substituindo a original).
2. Grave `firmware/kairos-receptor` no Uno (placa **Arduino Uno**).
3. Grave `firmware/kairos-luva-tx` no ESP32 (placa **ESP32 Dev Module**).
4. Ligue o braço **com todas as juntas para cima e a palma para trás**. Essa pose vira o home.
5. Abra o monitor serial da luva a **115200**. O boot testa os 3 MPU e o link LoRa (ping/pong).
6. Envie `r` e siga as 7 poses da calibração. Depois disso, a luva comanda o braço.
7. Use `h` para voltar o braço ao home e `n` para o modo manual remoto.

Os detalhes estão em [`docs/uso-e-calibracao.md`](docs/uso-e-calibracao.md).

## Biblioteca LoRa modificada

Em `bibliotecas/LoRa/src/LoRa.cpp`, o `endPacket()` síncrono original fica preso em um
`while` até o SX1278 sinalizar TX_DONE. Se o SPI falhar ou o rádio sofrer uma queda de
tensão (brownout), a placa trava para sempre, mas continua "viva" para o watchdog. A
versão deste repositório desiste depois de 100 ms, coloca o rádio em idle e limpa as
flags:

```cpp
unsigned long t0 = millis();
while ((readRegister(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK) == 0) {
  if (millis() - t0 > 100) {
    idle();
    writeRegister(REG_IRQ_FLAGS, 0xFF);
    return 0;
  }
  yield();
}
```

O envio normal dos dois firmwares já usa o modo assíncrono (`endPacket(true)`). O timeout
protege os envios síncronos que ainda existem.

## Aviso sobre o Arduino IDE

Durante o desenvolvimento, o Arduino IDE aberto **sobrescreveu os arquivos `.ino` com
versões antigas** que estavam em abas abertas. Se editar o código fora do IDE, feche as
abas sem salvar ou ative *Arquivo → Preferências → Usar editor externo*.
