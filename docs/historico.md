# Histórico de problemas e soluções

## Evolução do projeto

1. **Testes isolados** (`testes/`): um motor, dois motores, quatro motores, MPU, flex,
   servos, LoRa TX (ESP32) e RX (Uno).
2. **Luva com 3 MPU + 5 flex** (`versoes-anteriores/kairos-luva-tx-3mpu-backup`,
   `versoes-anteriores/backup-kairos-com-flex`).
3. **Receptor unificado** (`firmware/kairos-receptor`), que substituiu `receptor-uno` e
   `teste-braco-4motores`.
4. **Pulso novo**, seguindo o giro da palma (antes dele: `versoes-anteriores/backup-kairos-antes-pulso`).
5. **Botões no lugar dos flex** (antes deles: `versoes-anteriores/backup-kairos-antes-botoes`).

## Problemas resolvidos

| Problema | Causa | Solução |
|---|---|---|
| Pulso girava sozinho | Bias do giroscópio sendo integrado com a mão parada | Zona morta de 4 °/s no giro da palma, detecção de parado com gyro + gravidade, bias reaprendido só com o braço parado |
| Pulso girava invertido | Sinal do giro da mão oposto ao sentido do motor | `INV_TX[pulso] = true` na luva |
| Braço esticado virando para a esquerda "formava um L" no cotovelo | Deriva entre os sensores do braço e do antebraço aparecia como dobra | Restrição de dobradiça no cotovelo (`travarDobradica`) |
| Luva travava depois de um tempo | `endPacket()` síncrono preso esperando TX_DONE | Envio assíncrono (`endPacket(true)`), checagem de TX preso por 120 ms e timeout de 100 ms na biblioteca |
| Receptor travava ao responder | PONG síncrono bloqueava o loop dos motores | PONG assíncrono (`pongPendente` / `pongNoAr` / `cuidarPong`) |
| Loop travando ao imprimir | Buffer serial cheio (64 bytes no Uno) | Status com `snprintf` enviado aos poucos (`availableForWrite`); no ESP32, `setTxBufferSize(1024)` |
| Home se perdia | Motor perdendo passo na faixa de ressonância (100–300 passos/s) | Velocidade de cruzeiro acima da faixa e aceleração 2,5× ao atravessá-la; home fixo na pose do boot |
| Escritas no LoRa do Uno corrompidas | SPI de 8 MHz com o Uno em 5 V e o SX1278 em 3,3 V | SPI a 1 MHz |
| PONG do Uno falhava | 17 dBm puxava corrente demais do 3V3 do Uno; OCP de 100 mA cortava o PA | 10 dBm no Uno e OCP de 140 mA |
| Flex instáveis | Ruído analógico e pinos flutuando | Troca por botões com pull-up interno e debounce de 25 ms |
| Arquivos `.ino` voltando para versões antigas | O Arduino IDE aberto salvava as abas por cima | Restauração a partir dos backups; recomendação de usar editor externo |

## Decisões

- O boot **só testa** (MPU e ping/pong). Calibrar e entrar no manual são sempre ações
  explícitas (`r` e `n`).
- A calibração cobre só o braço: poses 1 a 6, palma para baixo e para cima, e pose 7.
  A tecla `f` (reancorar a frente) foi removida.
- O home é sempre o ponto em que o Uno ligou. Nenhuma tecla redefine esse ponto.
- Enquanto o braço não está calibrado, ou está em home, a luva envia as juntas zeradas.
  Os dedos são enviados sempre.
- Os motores ficam com a corrente máxima (Vref) que o driver aguenta.
