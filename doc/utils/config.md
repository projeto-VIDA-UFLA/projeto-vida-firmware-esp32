# config.h — parâmetros do firmware

> `include/utils/config.h`

Todos os parâmetros ajustáveis do firmware ficam neste arquivo, agrupados por módulo. A pinagem fica em [pinout.h](pinout.md). Mudanças aqui exigem recompilar e regravar o ESP32.

## Comunicação

| Parâmetro | Valor | Descrição |
|---|---|---|
| `SERIAL_BAUDRATE` | 921600 | Serial do micro-ROS; o agente no Odroid precisa usar o mesmo valor |

## PWM e canais LEDC

| Parâmetro | Valor | Descrição |
|---|---|---|
| `PWM_FREQUENCY_HZ` | 9000 | PWM dos motores de tração e esterçamento |
| `PWM_RESOLUTION_BITS` | 8 | Duty de 0 a 255 |
| `PWM_CH_DRIVE` | 0 | Canal da tração |
| `PWM_CH_STEER_RIGHT` / `PWM_CH_STEER_LEFT` | 1 / 2 | Canais do esterçamento |
| `PWM_CH_BRAKE` | 4 | Canal do servo (timer próprio, 50 Hz) |

## Tasks

| Parâmetro | Valor | Descrição |
|---|---|---|
| `TASK_SENSOR_PERIOD_MS` | 2 | `TaskSensors` (500 Hz) |
| `TASK_SAFETY_PERIOD_MS` | 5 | `TaskSafety` (200 Hz) |
| `TASK_CONTROL_PERIOD_MS` | 5 | `TaskControl` (200 Hz) |
| `TASK_COMUNICATION_PERIOD_MS` | 20 | Período de publicação ROS (50 Hz) |
| `PRIORITY_SAFETY` / `PRIORITY_CONTROL` / `PRIORITY_SENSORS` | 4 / 3 / 2 | Prioridades FreeRTOS |

## Segurança e watchdogs → [SafetyManager](../safety/SafetyManager.md)

| Parâmetro | Valor | Descrição |
|---|---|---|
| `LOGICAL_ENABLE_WATCHDOG_TIMEOUT_MS` | 300 | Idade máxima do enable lógico |
| `DRIVE_REFERENCE_WATCHDOG_TIMEOUT_MS` | 200 | Idade máxima da referência de tração |
| `STEERING_REFERENCE_WATCHDOG_TIMEOUT_MS` | 200 | Idade máxima da referência de esterçamento |
| `SENSOR_TASK_WATCHDOG_TIMEOUT_MS` | 20 | Tempo máximo entre ciclos válidos da `TaskSensors` |
| `SENSOR_TASK_STARTUP_GRACE_MS` | 500 | Tolerância para o primeiro ciclo após o boot |
| `STEERING_SENSOR_SNAPSHOT_TIMEOUT_MS` | 20 | Idade máxima da leitura do LVDT |
| `CONTROL_OUTPUT_WATCHDOG_TIMEOUT_MS` | 20 | Idade máxima da saída do controlador / do comando |
| `ENCODER_DISAGREE_MARGIN_M_S` | 0,8 | Traseira acima das dianteiras que caracteriza discordância ([EncoderPlausibility](../safety/EncoderPlausibility.md)) |
| `ENCODER_DISAGREE_TIME_MS` | 500 | Tempo contínuo para a falha |
| `STEERING_PLAUSIBILITY_MIN_COMMAND` | 0,3 | Comando mínimo avaliado ([SteeringPlausibility](../safety/SteeringPlausibility.md)) |
| `STEERING_STALL_MIN_PROGRESS_DEG` / `STEERING_STALL_TIME_MS` | 0,5° / 1000 | Travado |
| `STEERING_WRONG_DIRECTION_DEG` / `STEERING_WRONG_DIRECTION_GRACE_MS` | 2,0° / 200 | Sentido invertido |
| `TASK_SUPERVISOR_PERIOD_MS` / `TIMEOUT_MS` / `STARTUP_GRACE_MS` | 10 / 50 / 500 | [TaskSupervisor](../safety/TaskSupervisor.md) |
| `TASK_WDT_TIMEOUT_S` | 5 | Watchdog do ESP-IDF (vigia o supervisor) |
| `BENCH_MODE_REQUEST_TIMEOUT_MS` | 1000 | Sem pedido recente, sai do modo bancada |
| `BENCH_MODE_MAX_SPEED_M_S` | 3,0 | Velocidade máxima em modo bancada |

## Sensor de esterçamento → [SteeringSensor](../sensors/SteeringSensor.md)

| Parâmetro | Valor | Descrição |
|---|---|---|
| `ADC_RESOLUTION_BITS` | 12 | ADC de 0 a 4095 |
| `STEERING_MIN_DEG` / `STEERING_MAX_DEG` | −25° / +25° | **Limite de segurança** (proteção de fim de curso) |
| `STEERING_REFERENCE_MIN_DEG` / `MAX_DEG` | −20° / +20° | **Faixa de trabalho** (limite da referência) |
| `STEERING_ADC_CENTER` | 2046 | Centro do polinômio de calibração |
| `STEERING_POLY_A` / `B` / `C` | −2,14862e-2 / −3,09559e-7 / 1,42370e-9 | Coeficientes do polinômio cúbico |
| `STEERING_INVERT_SIGN` | 1 | `1` = REP-103 (esquerda +); `0` = convenção da calibração |
| `STEERING_ZERO_OFFSET_DEG` | 1,6° | Offset de zero (teste de 12/10/2026) |
| `STEERING_FILTER_ALPHA` | 0,05 | Filtro do ângulo (τ ≈ 39 ms) |
| `STEERING_VELOCITY_FILTER_ALPHA` | 0,01 | Filtro da velocidade angular |
| `STEERING_ADC_VALID_MIN` / `MAX` | 40 / 4055 | Faixa válida do ADC |
| `STEERING_ADC_RAIL_TIME_MS` | 10 | Persistência fora da faixa para invalidar |
| `STEERING_ADC_STUCK_TOLERANCE` | 3 | Variação máxima (counts) de "leitura travada" |
| `STEERING_ADC_STUCK_TIME_MS` | 500 | Tempo para considerar a leitura travada |

## Encoders e movimento → [EncoderReader](../sensors/EncoderReader.md), [VehicleMotion](../sensors/VehicleMotion.md), [DriveMotor](../actuators/DriveMotor.md)

| Parâmetro | Valor | Descrição |
|---|---|---|
| `ENC_REAR_TICKS_PER_REV` | 1440 | Ticks x4 por volta (traseiro) |
| `ENC_FRONT_TICKS_PER_REV` | 730 | Ticks x4 por volta (dianteiros) |
| `DRIVE_WHEEL_DIAMETER_M` | 0,366 | Diâmetro das rodas |
| `ENCODER_VELOCITY_WINDOW_MS` | 20 | Duração mínima da janela M/T |
| `ENCODER_ZERO_VELOCITY_TIMEOUT_MS` | 200 | Sem bordas por este tempo: velocidade zero |
| `DRIVE_VELOCITY_FILTER_ALPHA` | 0,5 | Filtro após o M/T |
| `DRIVE_ACCEL_FILTER_ALPHA` | 0,01 | Filtro da aceleração |
| `DRIVE_STOP_SPEED_THRESHOLD_MPS` | 0,05 | Limiar de "parado" |
| `DRIVE_STOP_CONFIRMATION_TIME_MS` | 200 | Tempo contínuo parado antes de trocar DIR |
| `DRIVE_VELOCITY_MEASUREMENT_TIMEOUT_MS` | 50 | Idade máxima da medição de velocidade |
| `DRIVE_DIRECTION_DEADTIME_MS` | 20 | Sem torque após trocar DIR |

## Freio → [BrakeServo](../actuators/BrakeServo.md)

| Parâmetro | Valor | Descrição |
|---|---|---|
| `BRAKE_SERVO_FREQUENCY_HZ` / `BRAKE_SERVO_RESOLUTION_BITS` | 50 / 16 | PWM do servo |
| `BRAKE_CALIBRATED` | 1 | `0` desliga os pulsos (freio inoperante) |
| `BRAKE_RELEASED_US` / `BRAKE_APPLIED_US` | 2500 / 1600 | Pulso do freio solto / total |
| `BRAKE_HOLD_US` | 1700 | Pulso máximo com o veículo parado |
| `BRAKE_HOLD_CONFIRMATION_MS` | 1000 | Tempo parado antes do alívio |
| `BRAKE_FAULT_RATE_PER_S` | 1,0 | Velocidade da rampa de falha |
| `BRAKE_DRIVE_CUTOFF` | 0,05 | Posição do freio que zera a tração |

## Controle de esterçamento → [SteeringController](../control/SteeringController.md), [SteeringMotor](../actuators/SteeringMotor.md)

| Parâmetro | Valor | Descrição |
|---|---|---|
| `STEERING_DEADBAND_ENTER_DEG` / `EXIT_DEG` | 0,4° / 1,2° | Zona morta com histerese |
| `STEERING_MIN_ACTUATION` | 0,15 | Piso de atuação |
| `MAX_ABS_STEER_COMMAND` | 1,0 | Saturação do comando |
| `STEERING_CONTROL_MAX_DT_S` | 0,1 | dt máximo antes de resetar |
| `STEERING_DIRECTION_DEADTIME_US` | 1000 | Tempo mínimo com os dois PWMs em zero na inversão |
| `STEERING_DIRECT_KP` / `KI` / `KD` | 0,007 / 0,1 / 0 | Ganhos do PID |
| `STEERING_DIRECT_INTEGRAL_MIN` / `MAX` | ±10 °·s | Limite da integral |

As constantes `STEERING_POS_*`/`STEERING_VEL_*` comentadas são do controlador em cascata antigo e não são usadas.

## Controle longitudinal → [DriveController](../control/DriveController.md)

| Parâmetro | Valor | Descrição |
|---|---|---|
| `DRIVE_MAX_TARGET_SPEED_M_S` | 4,0 m/s | Limite da referência |
| `DRIVE_DEFAULT_MAX_ACCEL_M_S2` | 0,4 m/s² | Rampa de subida quando `acceleration_m_s2` = 0 |
| `DRIVE_MAX_ACCEL_M_S2` | 0,8 m/s² | Teto da rampa de subida |
| `DRIVE_MAX_DECEL_M_S2` | 1,0 m/s² | Rampa de descida |
| `DRIVE_VEL_KP` / `DRIVE_VEL_KI` | 0,10 / 0,025 | Ganhos do PI de velocidade |
| `DRIVE_CONTROL_MAX_DT_S` | 0,1 s | `dt` máximo antes de resetar |
| `DRIVE_MAX_TRACTION_COMMAND` | 0,15 | Tração máxima (PWM 38) |
| `DRIVE_BRAKE_COAST_BAND` | 0,02 | Faixa de roda livre |
| `DRIVE_BRAKE_EFFORT_GAIN` | 3,0 | Escala esforço → freio (calibrar) |
| `DRIVE_BRAKE_ENGAGE_POSITION` | 0,2 | Posição em que o freio começa a agir (calibrar) |
| `DRIVE_STOP_REFERENCE_M_S` | 0,05 m/s | Referência considerada "parar" |
| `DRIVE_STOP_BRAKE_POSITION` | 0,6 | Freio de parada |
| `DRIVE_HOLD_BRAKE_POSITION` | 1,0 | Freio de estacionamento |
| `DRIVE_BRAKE_OVERRIDE_THRESHOLD` | 0,05 | `brake_command` que ativa o freio manual (L2) |
