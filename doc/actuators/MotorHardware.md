# MotorHardware

> `include/actuators/MotorHardware.h` · `src/actuators/MotorHardware.cpp`

## Visão geral

Interface abstrata com todo o acesso a hardware que os atuadores precisam: configuração de pinos e canais LEDC, escrita de PWM e do pino de direção, e o relógio.

[DriveMotor](DriveMotor.md), [SteeringMotor](SteeringMotor.md) e [BrakeServo](BrakeServo.md) recebem um `MotorHardware*` opcional no construtor:

- **`nullptr` (padrão):** usa `defaultMotorHardware()`, implementada com as funções do Arduino/ESP32;
- **implementação falsa:** permite verificar todas as escritas de PWM e DIR sem hardware, e controlar o tempo.

## Interface

| Método | Implementação padrão |
|---|---|
| `configureOutput(pin)` | `pinMode(pin, OUTPUT)` |
| `configurePwm(pin, channel)` | `ledcSetup(channel, PWM_FREQUENCY_HZ, PWM_RESOLUTION_BITS)` + `ledcAttachPin` |
| `configureServo(pin, channel)` | `ledcSetup(channel, BRAKE_SERVO_FREQUENCY_HZ, BRAKE_SERVO_RESOLUTION_BITS)` + `ledcAttachPin` |
| `writeDirection(pin, high)` | `digitalWrite` |
| `writePwm(channel, duty)` | `ledcWrite` |
| `nowUs()` | `esp_timer_get_time()` |

## Canais LEDC

| Canal | Uso | Timer |
|---|---|---|
| 0 | Tração | compartilhado com o canal 1 (9 kHz) |
| 1 | Esterçamento direita | compartilhado com o canal 0 |
| 2 | Esterçamento esquerda | compartilhado com o canal 3 |
| 4 | Servo do freio (50 Hz) | timer próprio |

O freio usa o canal 4 porque os canais 2 e 3 compartilham timer: usar o 3 mudaria a frequência do esterçamento para 50 Hz.
