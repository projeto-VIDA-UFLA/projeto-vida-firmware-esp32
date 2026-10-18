# DriveController

> `include/control/DriveController.h` · `src/control/DriveController.cpp`

## Visão geral

Controle **longitudinal** do veículo. Hoje funciona em **malha aberta**: a referência recebida do ROS é tratada como um PWM e convertida diretamente em comando de tração. É nesta classe que entra o controle de velocidade em malha fechada.

## Onde é usada

| Quem | Método | Frequência |
|---|---|---|
| `TaskControl` | `update(referência de tração)` | 200 Hz |

## Interface pública

| Método | Descrição |
|---|---|
| `update(reference)` | Converte a referência de tração em comando e repassa o freio |

### Output

| Campo | Descrição |
|---|---|
| `normalized_command` | Comando de tração `[-1, 1]` (positivo = frente) |
| `brake_command` | Freio repassado da referência (`0` solto, `1` total) |
| `valid` | Referência recebida, finita e com idade ≤ `DRIVE_REFERENCE_WATCHDOG_TIMEOUT_MS` |

## Funcionamento (malha aberta)

O campo `velocity_m_s` da mensagem `DriveReference` carrega, **temporariamente**, um PWM assinado em `[-255, 255]` (o gamepad publica de 0 a 255 pelo gatilho R2):

```
comando = clamp(referência × DRIVE_OPEN_LOOP_PROPORTIONAL_GAIN / DRIVE_OPEN_LOOP_REFERENCE_MAX_PWM, −1, 1)
```

Com ganho 0,15, a referência máxima (255) gera comando 0,15, ou seja, PWM 38 de 255.

O freio (`brake_command`) é repassado sem tratamento. O intertravamento freio → tração é feito pelo [ActuatorManager](../actuators/ActuatorManager.md): com o freio acima de 0,05 a tração é zerada.

## Parâmetros (`config.h`)

| Parâmetro | Valor | Uso |
|---|---|---|
| `DRIVE_OPEN_LOOP_REFERENCE_MAX_PWM` | 255 | Escala da referência |
| `DRIVE_OPEN_LOOP_PROPORTIONAL_GAIN` | 0,15 | Fração do PWM máximo liberada |
| `DRIVE_REFERENCE_WATCHDOG_TIMEOUT_MS` | 200 ms | Idade máxima da referência |

## Pontos de atenção

- Os campos `velocity_m_s` (como velocidade) e `acceleration_m_s2` da referência ainda não são usados.
- Para a malha fechada, a medição de velocidade dos encoders precisa melhorar (ver [EncoderReader](../sensors/EncoderReader.md)); o [VehicleMotion](../sensors/VehicleMotion.md) é o lugar previsto para o estimador.
