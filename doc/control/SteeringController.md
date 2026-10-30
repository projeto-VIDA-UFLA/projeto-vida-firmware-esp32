# SteeringController

> `include/control/SteeringController.h` · `src/control/SteeringController.cpp`

## Visão geral

Controle de **posição do esterçamento**: recebe o ângulo de referência (ROS) e o ângulo medido pelo LVDT e calcula o comando do motor de esterçamento. É um **PI** (o termo D existe, mas hoje Kd = 0) com zona morta com histerese, anti-windup, saturação, piso de atuação e proteção de fim de curso.

Diagramas detalhados (malha, fluxograma, característica estática e fragilidades): [`doc/arquitetura/controle_estercamento/`](../arquitetura/controle_estercamento/).

## Onde é usada

| Quem | Método | Frequência |
|---|---|---|
| `TaskControl` | `update(referência, snapshot do LVDT, permissão)` | 200 Hz |
| [SafetyManager](../safety/SafetyManager.md) | `getLastOutput()` (idade e validade da saída) | 200 Hz |

## Interface pública

| Método | Descrição |
|---|---|
| `update(reference, sensor, motion_permitted)` | Calcula a saída de um ciclo (ver abaixo) |
| `getLastOutput()` | Última saída e seu instante (`timestamp_us = 0` se inválida) |
| `reset()` | Zera integral, erro anterior e estado da zona morta |

### Output

| Campo | Descrição |
|---|---|
| `target_angle_deg` | Referência efetivamente usada (já limitada) |
| `measured_angle_deg` | Ângulo medido (NaN se o sensor estava inválido ou velho) |
| `angle_error_deg` | Referência − medido |
| `proportional_term`, `integral_term`, `derivative_term` | Parcelas do PID |
| `normalized_command` | Comando na convenção do ângulo `[-1, 1]` |
| `motor_command` | Mesmo comando na convenção do motor (`> 0` = PWM da direita) |
| `inside_deadband` | Dentro da zona morta (comando 0) |
| `saturated` | O PID passou de ±1 antes da saturação |
| `target_clamped` | Referência limitada à faixa de trabalho |
| `position_limit_active` | Proteção de fim de curso bloqueou o comando |
| `valid` | Saída utilizável |

## Funcionamento

```mermaid
flowchart LR
    R["θref"] --> CL["Limite ±20°"]
    CL --> SUM(("Σ"))
    M["θmedido (LVDT)"] --> SUM
    SUM -- erro --> DB["Zona morta<br/>entra ≤ 0,4° · sai > 1,2°"]
    DB --> PID["P + I (+ D)<br/>anti-windup"]
    PID --> SAT["Saturação ±1"]
    SAT --> FL["Piso ±0,15"]
    FL --> LIM["Fim de curso ±25°"]
    LIM --> SG["× sinal → motor_command"]
```

Ordem das etapas em cada ciclo (`update()` → `compute()`):

1. **Idade do sensor:** leitura inválida ou com mais de `STEERING_SENSOR_SNAPSHOT_TIMEOUT_MS` (20 ms) vira `NaN`.
2. **dt:** tempo desde a chamada anterior (no primeiro ciclo, `TASK_CONTROL_PERIOD_MS`).
3. **Validação:** referência, medição ou dt não finitos, ou dt fora de `(0, 100 ms]` → `reset()` e **saída inválida**.
4. **Limite da referência:** `[STEERING_REFERENCE_MIN_DEG, STEERING_REFERENCE_MAX_DEG]` = ±20° (`target_clamped`).
   **Limite de taxa:** a referência usada caminha até a pedida a no máximo `STEERING_REFERENCE_MAX_RATE_DEG_S` (40°/s); sem permissão de movimento (ou após um reset) ela parte do ângulo medido. Degraus do analógico viram rampas (no chão, em 30/10, os degraus acumulavam integral e davam tranco).
5. **Erro** e **derivada do erro** (zero no primeiro ciclo após um reset).
6. **Zona morta com histerese:** fora dela, só entra com \|erro\| ≤ 0,4°; dentro dela, só sai com \|erro\| > 1,2°. Dentro: comando 0 e integral zerada.
7. **Integral** (só com movimento permitido):
   - **cruzou a referência** (erro trocou de sinal): integral zerada, para não empurrar além dela;
   - integral candidata = `clamp(∫ + erro·dt, ±5 °·s)` (parcela integral até ±0,5);
   - **anti-windup condicional:** a candidata é rejeitada se o comando já passa de ±1 e ela aumentaria a saturação. Uma integral que ajuda a sair da saturação é aceita.
   - **sem permissão de movimento**, a integral fica em zero (o motor está parado; integrar acumularia erro e o esterçamento partiria saturado ao habilitar).
8. **Saturação** em `±MAX_ABS_STEER_COMMAND` (`saturated`).
9. **Piso de atuação:** `0 < |u| < 0,15` vira `±0,15`, para vencer o atrito estático.
10. **Fim de curso:** com o medido ≥ +25° e comando positivo (ou ≤ −25° e negativo), o comando vai a 0 e a integral é zerada. O comando no sentido de volta continua permitido.
11. **Saída:** `motor_command = STEERING_ANGLE_SIGN × normalized_command`, publicada com timestamp.

### Limites de ângulo

| Faixa | Valor | Quem aplica |
|---|---|---|
| Trabalho (referência) | ±20° | `SteeringController` (passo 4) |
| Segurança (fim de curso) | ±25° | `SteeringController` (passo 10) |
| Fundo de escala do LVDT | ~−28,5° / ~+34,5° | [SteeringSensor](../sensors/SteeringSensor.md) (invalida a leitura) |
| Fim físico do lado mais curto | ~28,9° | mecânica |

## Parâmetros (`config.h`)

| Parâmetro | Valor | Uso |
|---|---|---|
| `STEERING_DIRECT_KP` | 0,007 /° | Ganho proporcional |
| `STEERING_DIRECT_KI` | 0,1 /(°·s) | Ganho integral |
| `STEERING_DIRECT_KD` | 0 | Ganho derivativo (desligado) |
| `STEERING_DIRECT_INTEGRAL_MIN/MAX` | ±5 °·s | Limite da integral (±0,5 de comando) |
| `STEERING_REFERENCE_MAX_RATE_DEG_S` | 40°/s | Taxa máxima da referência |
| `STEERING_DEADBAND_ENTER_DEG` | 0,4° | Entrada na zona morta |
| `STEERING_DEADBAND_EXIT_DEG` | 1,2° | Saída da zona morta |
| `STEERING_MIN_ACTUATION` | 0,15 | Piso de atuação |
| `MAX_ABS_STEER_COMMAND` | 1,0 | Saturação |
| `STEERING_REFERENCE_MIN/MAX_DEG` | ±20° | Faixa de trabalho |
| `STEERING_MIN/MAX_DEG` | ±25° | Limite de segurança |
| `STEERING_CONTROL_MAX_DT_S` | 0,1 s | dt máximo antes de resetar |
| `STEERING_SENSOR_SNAPSHOT_TIMEOUT_MS` | 20 ms | Idade máxima da leitura do LVDT |

## Concorrência

O estado interno (integral, erro anterior, zona morta) só é usado pela `TaskControl`. A última saída é publicada sob `portMUX` e lida pelo `SafetyManager` via `getLastOutput()`.

## Pontos de atenção

- **Kp baixo:** com Kp = 0,007/°, o termo P só passa do piso de 0,15 com erros acima de ~21°. Na prática o controle é um relé de ±0,15 mais a integral. Revisar na sintonia.
- **Integral zerada na zona morta:** o redutor não trava a posição. Se o veículo em movimento empurrar as rodas para fora da zona morta, o controlador corrige com pulsos curtos. Observar em teste.
- **Referência sem limite de taxa:** o campo `angular_velocity_deg_s` da referência é recebido mas ignorado; degraus de referência viram velocidade máxima do motor.
- **D sobre o erro, sem filtro:** se Kd for ativado, degraus de referência geram pico (derivative kick). Preferir derivar a medição.
