# pinout.h — pinagem

> `include/utils/pinout.h`

Placa: **ESP32 DevKit de 38 pinos** (`esp32dev`).

| Função | Constante | GPIO | Observação |
|---|---|---|---|
| Encoder traseiro A / B | `ENC_REAR_A` / `ENC_REAR_B` | 19 / 18 | |
| Encoder dianteiro direito A / B | `ENC_RIGHT_A` / `ENC_RIGHT_B` | 35 / 34 | Só entrada, sem pull-up interno |
| Encoder dianteiro esquerdo A / B | `ENC_LEFT_A` / `ENC_LEFT_B` | 33 / 32 | |
| LVDT (esterçamento) | `PIN_LVDT` | 36 | ADC1_CH0, só entrada |
| Esterçamento — PWM direita | `PIN_STEER_RIGHT_PWM` | 25 | Canal LEDC 1 |
| Esterçamento — PWM esquerda | `PIN_STEER_LEFT_PWM` | 26 | Canal LEDC 2 |
| Tração — PWM | `PIN_DRIVE_PWM` | 27 | Canal LEDC 0 |
| Tração — direção (DIR) | `PIN_DRIVE_REVERSE` | 14 | `HIGH` = ré; emite sinal durante o boot |
| Servo do freio | `PIN_BRAKE_SERVO` | 4 | Canal LEDC 4 (50 Hz) |
| Enable físico de segurança | `PIN_ENABLE_SAFETY` | 13 | `HIGH` = painel habilitado; `LOW` bloqueia o movimento |
| I2C SDA / SCL (INA228, bateria) | `PIN_I2C_SDA` / `PIN_I2C_SCL` | 21 / 22 | Endereço `INA228_I2C_ADDRESS` = 0x40 |

O ADC usado é o ADC1, que continua funcionando com o Wi-Fi ligado (o firmware não usa Wi-Fi).
