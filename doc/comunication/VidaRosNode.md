# VidaRosNode

> `include/comunication/VidaRosNode.h` · `src/comunication/VidaRosNode.cpp`

## Visão geral

Nó micro-ROS do VIDA: define **todos os tópicos** do firmware sobre a infraestrutura do [RosCommunication](RosCommunication.md).

- **Recebe** referências e o enable lógico e os guarda no [CommandMailbox](CommandMailbox.md).
- **Publica** sensores, atuadores e estado de segurança a cada 20 ms.

| Item | Valor |
|---|---|
| Nó | `esp32_controller` |
| Namespace | `/vida` |
| Domain ID | 0 |
| Pacote de mensagens | `vida_interfaces` (em `interfaces/vida_interfaces/`) |

## Interface pública

| Método | Descrição |
|---|---|
| `VidaRosNode(encoders, steering_sensor, safety, actuators, mailbox)` | Objetos que fornecem e recebem os dados |
| `begin(transport)` | Registra as entidades e inicia a `TaskRosCommunication` (núcleo 0) |

Os callbacks do rclc são funções livres; elas acessam o objeto pela instância única `instance_`.

## Tópicos assinados

| Tópico | Mensagem | Destino | Validação |
|---|---|---|---|
| `/vida/control/steering/reference` | `SteeringReference` | `updateSteeringReference` | descarta se `angle_deg` ou `angular_velocity_deg_s` não forem finitos |
| `/vida/control/drive/reference` | `DriveReference` | `updateDriveReference` | descarta se algum campo não for finito |
| `/vida/safety/logical_enable` | `LogicalEnable` | `updateLogicalEnable` | — |
| `/vida/safety/bench_mode_request` | `std_msgs/Bool` | `updateBenchModeRequest` | — |

Hoje quem publica as referências é o gamepad (`control_pkg`, no notebook, a 25 Hz), e quem publica o enable lógico é o nó `security` do Odroid.

## Tópicos publicados (a cada 20 ms)

| Tópico | Mensagem | Conteúdo |
|---|---|---|
| `/vida/sensors/encoders` | `Encoders` | Medição dos três encoders (só publica se os três tiverem timestamp) |
| `/vida/sensors/steering` | `Steering` | ADC bruto e ângulos em REP-103 (esquerda +). Publicado também com a leitura inválida, para diagnóstico; a invalidade aparece em `/vida/safety/status` (flag 128) |
| `/vida/actuators/steering` | `Comands` | PWM do esterçamento, **positivo = esterçar para ângulo positivo (esquerda)** |
| `/vida/actuators/drive` | `Comands` | PWM da tração, negativo = ré |
| `/vida/actuators/brake` | `Comands` | **Largura de pulso do servo em µs** (0 = sem pulsos) |
| `/vida/safety/status` | `SafetyStatus` | Estado, condições e `inhibit_flags` do [SafetyManager](../safety/SafetyManager.md) |
| `/vida/safety/bench_mode` | `std_msgs/Bool` | Modo bancada efetivo (sem timestamp) |

O timestamp de cada mensagem é o **instante da amostra** convertido para o relógio do agente. Nada é publicado antes da sincronização de relógio.

## Mensagens (`vida_interfaces/msg`)

```
# SteeringReference.msg
builtin_interfaces/Time timestamp
float32 angle_deg               # REP-103, esquerda +
float32 angular_velocity_deg_s  # ainda nao usado

# DriveReference.msg
builtin_interfaces/Time timestamp
float32 velocity_m_s            # velocidade desejada (m/s), limitada a [0, 3]
float32 acceleration_m_s2       # aceleracao maxima da rampa (0 = padrao 0,4 m/s2)
float32 brake_command           # > 0,05: freio manual (zera a velocidade, freio total)

# LogicalEnable.msg
builtin_interfaces/Time timestamp
bool enabled

# Encoders.msg
EncoderMeasurement left
EncoderMeasurement right
EncoderMeasurement rear

# EncoderMeasurement.msg
builtin_interfaces/Time timestamp
int64 count
float32 raw_velocity_m_s
float32 filtered_velocity_m_s
float32 raw_acceleration_m_s2
float32 filtered_acceleration_m_s2

# Steering.msg
builtin_interfaces/Time timestamp
uint16 raw_adc
float32 raw_angle_deg
float32 filtered_angle_deg
float32 filtered_angular_velocity_deg_s

# Comands.msg
builtin_interfaces/Time timestamp
int16 pwm

# SafetyStatus.msg
uint8 state                     # 0 DISABLED, 1 READY
bool physical_enable
bool logical_enable
bool logical_enable_received
bool logical_enable_fresh
bool drive_reference_fresh
bool steering_reference_fresh
bool motion_permitted
uint32 inhibit_flags            # ver SafetyManager (1..256, 512, 1024, 4096, 8192)
```

`SteeringControllerOutput.msg` ainda existe no pacote, mas não é mais publicado.

**Limite de entidades** da biblioteca micro-ROS (`lib/micro_ros_platformio/metas/colcon.meta`): 10 publishers (7 usados) e **5 subscriptions (4 usadas)**. Um novo tópico assinado ocupa a última vaga; além disso é preciso aumentar `RMW_UXRCE_MAX_SUBSCRIPTIONS` e recompilar a biblioteca.

## Pontos de atenção

- Ao mudar qualquer `.msg`, a biblioteca micro-ROS precisa ser recompilada (`pio run -t clean_microros` e depois `pio run`). Os pacotes `vida_interfaces` do Odroid e do notebook precisam ter a mesma definição.
