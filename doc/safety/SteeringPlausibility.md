# SteeringPlausibility

> `include/safety/SteeringPlausibility.h` · `src/safety/SteeringPlausibility.cpp`

## Visão geral

Verifica se o esterçamento **responde ao comando**: compara o PWM efetivamente aplicado ao motor de esterçamento com o ângulo medido pelo LVDT. Avaliação pura, chamada pelo [SafetyManager](SafetyManager.md), que retém as falhas.

Detecta:

| Falha | Causas típicas |
|---|---|
| **Travado** | Motor ou mecanismo preso, obstáculo, batente mecânico antes do limite de software, **LVDT sem sinal** (inclusive com a entrada flutuando, que a validação do [SteeringSensor](../sensors/SteeringSensor.md) não pega) |
| **Sentido invertido** | Motor com fios trocados, sinal do sensor ou de `STEERING_INVERT_SIGN` errado |

## Regras

Só são avaliados comandos **fortes**: \|comando\| ≥ `STEERING_PLAUSIBILITY_MIN_COMMAND` (0,3, PWM 77). O comando é o PWM aplicado (`direita − esquerda`), convertido para a convenção do ângulo e dividido por 255.

| Falha | Condição |
|---|---|
| Travado | Comando forte no mesmo sentido por **1 s** sem o ângulo avançar **0,5°** nesse sentido |
| Sentido invertido | Depois de **0,2 s** do início do comando, o ângulo **recua 2°** contra ele |

### Por que só comandos fortes

No teste de 12/10/2026:

| PWM aplicado | Maior tempo sem avançar 0,5° | Maior recuo contra o comando |
|---|---|---|
| ≥ 38 (piso de 0,15) | 2,07 s (atrito estático) | 0,59° |
| ≥ 77 (0,3) | 0,42 s | 0,00° |

No piso de atuação a roda pode ficar parada por atrito sem nenhum defeito. Se ela estiver presa de verdade, a integral do controlador aumenta o comando até passar de 0,3, e a falha é detectada.

Repetindo a regra sobre esse teste: nenhum disparo, com folga de 2,3× no tempo. O teste teve só 3,7 s de comando forte, então os valores devem ser confirmados nos próximos ensaios.

## Retenção e rearme

O [SafetyManager](SafetyManager.md) retém as falhas e bloqueia o movimento:

| Flag | Valor |
|---|---|
| `INHIBIT_STEERING_STALLED` | 512 |
| `INHIBIT_STEERING_WRONG_DIRECTION` | 1024 |

Rearme: enable físico desligado ou enable lógico `false`. A verificação continua ativa em modo bancada.

## Parâmetros (`config.h`)

| Parâmetro | Valor |
|---|---|
| `STEERING_PLAUSIBILITY_MIN_COMMAND` | 0,3 |
| `STEERING_STALL_MIN_PROGRESS_DEG` | 0,5° |
| `STEERING_STALL_TIME_MS` | 1000 ms |
| `STEERING_WRONG_DIRECTION_DEG` | 2,0° |
| `STEERING_WRONG_DIRECTION_GRACE_MS` | 200 ms |

## Pontos de atenção

- Na proteção de fim de curso de software o controlador zera o comando, então não há disparo. Se o **batente mecânico** vier antes do limite de software, a falha "travado" dispara, e isso é desejado.
- Uma entrada flutuante pode gerar ruído que, por acaso, pareça avanço. A detecção desse caso é provável, não garantida.
