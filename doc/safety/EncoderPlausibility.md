# EncoderPlausibility

> `include/safety/EncoderPlausibility.h` · `src/safety/EncoderPlausibility.cpp`

## Visão geral

Verifica se a **roda traseira** (motriz) e as **rodas dianteiras** (realimentação do controle de velocidade) contam a mesma história. Avaliação pura, chamada pelo [SafetyManager](SafetyManager.md), que retém a falha.

O [DriveController](../control/DriveController.md) controla a velocidade pela média das rodas dianteiras. Se essa medição não enxergar o movimento, o controlador vê velocidade zero e leva a tração ao máximo (PWM 38). Isso acontece em dois casos:

- **encoder dianteiro sem sinal**, lendo zero (é uma leitura "válida": finita e recente);
- **veículo suspenso na bancada:** só a roda traseira gira.

## Regra

Falha quando, **continuamente** por `ENCODER_DISAGREE_TIME_MS` (0,5 s):

- há tração aplicada (PWM de tração > 0);
- as velocidades são válidas;
- `v_traseira − v_média_dianteiras > ENCODER_DISAGREE_MARGIN_M_S` (0,8 m/s).

Só esse sentido é verificado:

| Situação | Diferença | Dispara? |
|---|---|---|
| Frenagem (traseira desacelera ou trava) | traseira **mais lenta** | não |
| Curva (dianteiras percorrem mais) | dianteiras **mais rápidas** | não |
| Patinagem curta na arrancada | traseira mais rápida por < 0,5 s | não |
| Encoder dianteiro sem sinal / bancada | traseira muito mais rápida, sustentado | **sim** |

Nos testes de 12/10 e 16/08/2026 a maior diferença com tração foi de 0,49 m/s sustentada e 1,35 m/s por 0,18 s (patinagem na arrancada), sem disparo com os valores atuais.

## Retenção e rearme

No [SafetyManager](SafetyManager.md), a falha fica **retida** (flag `INHIBIT_ENCODERS_DISAGREE` = 4096) e bloqueia o movimento até o operador rearmar:

- desligando o **enable físico**; ou
- publicando o **enable lógico** como `false`.

## Interface

| Método | Descrição |
|---|---|
| `update(input)` | `true` quando a discordância persistiu pelo tempo configurado |
| `reset()` | Zera a contagem de tempo |

`Input`: `now_us`, `traction_applied`, `speeds_valid`, `rear_velocity_m_s`, `front_velocity_m_s`.

## Parâmetros (`config.h`)

| Parâmetro | Valor |
|---|---|
| `ENCODER_DISAGREE_MARGIN_M_S` | 0,8 m/s |
| `ENCODER_DISAGREE_TIME_MS` | 500 ms |

## Pontos de atenção

- **Impede testar o controle de velocidade na bancada:** com o veículo suspenso, a falha dispara em ~0,5 s. É intencional: sem ela, a traseira giraria com tração máxima. Um modo bancada está em discussão.
- Não cobre a falha de **um só** encoder dianteiro: a média cai pela metade, e a diferença só passa de 0,8 m/s acima de ~1,6 m/s.
