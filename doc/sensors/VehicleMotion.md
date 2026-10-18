# VehicleMotion

> `include/sensors/VehicleMotion.h` · `src/sensors/VehicleMotion.cpp`

## Visão geral

Combina os encoders para responder três perguntas:

1. **Qual a velocidade do veículo?** Realimentação do [DriveController](../control/DriveController.md).
2. **O veículo está parado?** Usado pelo freio para aliviar o servo.
3. **Qual velocidade usar para autorizar a troca de sentido da tração?** Usado pelo [DriveMotor](../actuators/DriveMotor.md).

O freio atua só na roda traseira. Com ela travada, o encoder traseiro marca zero mesmo com o veículo deslizando, e são as rodas dianteiras que confirmam a parada. Por isso o critério sempre exige os três encoders.

## Onde é usada

| Quem | Método | Frequência |
|---|---|---|
| `TaskControl` → `DriveController::update()` | `getVehicleSpeed()` | 200 Hz |
| `TaskSafety` → `ActuatorManager::updateBrake()` | `isStopped()` | 200 Hz |
| `TaskControl` → `ActuatorManager::applyCommands()` | `getStopDetectionVelocity()` | 200 Hz |

## Interface pública

| Método | Descrição |
|---|---|
| `VehicleMotion(rear, right, left)` | Referências aos três [EncoderReader](EncoderReader.md) |
| `getVehicleSpeed()` | Média das velocidades das rodas dianteiras |
| `isStopped()` | `true` se os três encoders têm leitura recente, finita e com \|v\| ≤ limiar |
| `getStopDetectionVelocity()` | Medição de pior caso para o `DriveMotor` |

## Funcionamento

### `getVehicleSpeed()`

Média das velocidades filtradas dos encoders **dianteiros**. As rodas dianteiras não são tracionadas nem freadas: não patinam sob tração nem travam ao frear, então medem a velocidade real do veículo. O `timestamp_us` é o da amostra mais antiga, e a medição é inválida se algum dos dois não for finito ou não tiver medição.

Sem compensação de curva: em curva, a média das dianteiras fica ~1/cos δ acima da velocidade do eixo traseiro (3,5% a 15°, 6% a 20°).

### `isStopped()`

Cada encoder precisa ter, no instante atual:

- leitura com idade ≤ `DRIVE_VELOCITY_MEASUREMENT_TIMEOUT_MS` (50 ms);
- velocidade filtrada finita;
- \|velocidade\| ≤ `DRIVE_STOP_SPEED_THRESHOLD_MPS` (0,05 m/s).

Leitura inválida ou velha **conta como movimento**.

### `getStopDetectionVelocity()`

Monta uma única `DriveMotor::VelocityMeasurement` pelo **pior caso**:

| Campo | Regra |
|---|---|
| `velocity_m_s` | A de **maior módulo** entre os três |
| `timestamp_us` | A amostra **mais antiga** entre os três |
| `valid` | `false` se qualquer encoder tiver velocidade não finita ou sem medição |

Assim, um único encoder sem sinal (lendo 0 m/s) não libera a inversão com o veículo em movimento. O `DriveMotor` ainda exige que essa medição fique abaixo do limiar por `DRIVE_STOP_CONFIRMATION_TIME_MS` (200 ms) antes de trocar o pino DIR.

## Parâmetros (`config.h`)

| Parâmetro | Valor | Uso |
|---|---|---|
| `DRIVE_STOP_SPEED_THRESHOLD_MPS` | 0,05 m/s | Limiar de "parado" |
| `DRIVE_VELOCITY_MEASUREMENT_TIMEOUT_MS` | 50 ms | Idade máxima da leitura |

## Concorrência

Não guarda estado: cada chamada lê snapshots dos encoders (protegidos por `portMUX`) e calcula o resultado na hora.
