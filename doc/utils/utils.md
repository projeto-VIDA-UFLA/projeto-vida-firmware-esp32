# utils.h — funções auxiliares

> `include/utils/utils.h`

| Função | Descrição |
|---|---|
| `clampFloat(value, min, max)` | Limita `value` ao intervalo `[min, max]` |
| `mapFloat(X, A, B)` | Reta `A·X + B` (sem uso hoje) |
| `floatCommandToPwm(command)` | Converte o **módulo** de um comando `[-1, 1]` em duty de 8 bits (0–255). O sentido é tratado por quem chama |

`clampFloat` e `floatCommandToPwm` são usadas por [DriveMotor](../actuators/DriveMotor.md) e [SteeringMotor](../actuators/SteeringMotor.md); `clampFloat` também pelo [DriveController](../control/DriveController.md).
