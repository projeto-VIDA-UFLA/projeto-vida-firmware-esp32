# SafetyWatchdogs

> `include/safety/SafetyWatchdogs.h` · `src/safety/SafetyWatchdogs.cpp`

## Visão geral

Função **pura** (sem estado e sem hardware) que avalia os watchdogs de tempo usados pelo [SafetyManager](SafetyManager.md). Recebe instantes e flags e devolve o que está recente ou expirado. Fica separada para poder ser verificada isoladamente.

## Interface

`static Result evaluate(const Input &input)`

### Input

| Campo | Descrição |
|---|---|
| `now_us` | Instante atual |
| `initialized_at_us` | Instante do `SafetyManager::begin()` |
| `sensor_task_heartbeat_us` | Último ciclo válido da `TaskSensors` (0 = nunca) |
| `steering_snapshot_valid`, `steering_snapshot_us` | Validade e instante da leitura do LVDT |
| `control_required` | Demais condições ok: a saída do controlador precisa existir |
| `control_output_valid`, `control_command`, `control_output_us` | Última saída do [SteeringController](../control/SteeringController.md) |

### Result

| Campo | Regra |
|---|---|
| `sensor_task_timed_out` | Heartbeat com mais de 20 ms; ou nenhum heartbeat 500 ms depois do `begin()` |
| `sensor_task_healthy` | Já houve heartbeat e não expirou |
| `steering_snapshot_fresh` | Leitura válida com idade ≤ 20 ms |
| `control_output_fresh` | Saída válida, comando finito e idade ≤ 20 ms |
| `control_output_timed_out` | `control_required` e a saída não está recente |

"Recente" significa: instante maior que zero, não no futuro e com idade dentro do limite.

## Parâmetros (`config.h`)

`SENSOR_TASK_WATCHDOG_TIMEOUT_MS`, `SENSOR_TASK_STARTUP_GRACE_MS`, `STEERING_SENSOR_SNAPSHOT_TIMEOUT_MS`, `CONTROL_OUTPUT_WATCHDOG_TIMEOUT_MS` (valores em [SafetyManager](SafetyManager.md#parâmetros-configh)).
