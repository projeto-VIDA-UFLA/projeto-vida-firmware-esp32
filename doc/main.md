# main.cpp — instâncias, tasks e inicialização

> `main/main.cpp`

## Visão geral

O `main.cpp` cria todos os objetos do firmware como variáveis globais, liga as interrupções dos encoders e cria as tasks FreeRTOS. **As tasks não contêm lógica**: cada ciclo apenas chama métodos dos objetos e repassa snapshots por parâmetro.

## Objetos globais

| Objeto | Classe | Construído com |
|---|---|---|
| `right_encoder`, `left_encoder` | [EncoderReader](sensors/EncoderReader.md) | pinos `ENC_RIGHT_*`/`ENC_LEFT_*`, `ENC_FRONT_TICKS_PER_REV` |
| `rear_encoder` | [EncoderReader](sensors/EncoderReader.md) | pinos `ENC_REAR_*`, `ENC_REAR_TICKS_PER_REV` |
| `vehicle_motion` | [VehicleMotion](sensors/VehicleMotion.md) | os três encoders |
| `steering_sensor` | [SteeringSensor](sensors/SteeringSensor.md) | `PIN_LVDT` |
| `steering_motor` | [SteeringMotor](actuators/SteeringMotor.md) | `PIN_STEER_*_PWM`, `PWM_CH_STEER_*` |
| `drive_motor` | [DriveMotor](actuators/DriveMotor.md) | `PIN_DRIVE_PWM`, `PIN_DRIVE_REVERSE`, `PWM_CH_DRIVE` |
| `brake_servo` | [BrakeServo](actuators/BrakeServo.md) | `PIN_BRAKE_SERVO`, `PWM_CH_BRAKE` |
| `actuator_manager` | [ActuatorManager](actuators/ActuatorManager.md) | os três atuadores |
| `steering_controller` | [SteeringController](control/SteeringController.md) | — |
| `drive_controller` | [DriveController](control/DriveController.md) | — |
| `command_mailbox` | [CommandMailbox](comunication/CommandMailbox.md) | — |
| `safety_manager` | [SafetyManager](safety/SafetyManager.md) | `PIN_ENABLE_SAFETY`, mailbox, actuator manager, steering controller |
| `ros_node` | [VidaRosNode](comunication/VidaRosNode.md) | encoders, sensor, safety, atuadores, mailbox |

## Tasks

| Task | Núcleo | Prioridade | Período | Stack |
|---|---|---|---|---|
| `TaskSafety` | 1 | 4 (`PRIORITY_SAFETY`) | 5 ms (`TASK_SAFETY_PERIOD_MS`) | 4096 |
| `TaskControl` | 1 | 3 (`PRIORITY_CONTROL`) | 5 ms (`TASK_CONTROL_PERIOD_MS`) | 4096 |
| `TaskSensors` | 1 | 2 (`PRIORITY_SENSORS`) | 2 ms (`TASK_SENSOR_PERIOD_MS`) | 4096 |
| `TaskRosCommunication` | 0 | 2 | laço de 1 ms; publica a cada 20 ms | 8192 |

As três primeiras usam `vTaskDelayUntil` (período fixo). A `TaskRosCommunication` é criada pelo [`VidaRosNode`](comunication/VidaRosNode.md).

### TaskSensors (500 Hz)

```cpp
steering_sensor.update();
rear_encoder.update(); right_encoder.update(); left_encoder.update();
safety_manager.reportSensorTaskCycle(snapshot LVDT, snapshots dos 3 encoders);
```

O heartbeat só é registrado depois do ciclo completo e se todas as leituras forem válidas.

### TaskSafety (200 Hz, maior prioridade)

```cpp
safety_manager.update();                                  // permissão de movimento
actuator_manager.updateBrake(vehicle_motion.isStopped()); // rampa de falha / alívio parado
```

### TaskControl (200 Hz)

```cpp
steering = steering_controller.update(ref. esterçamento, snapshot LVDT,
                                      actuator_manager.isMotionPermitted());
drive    = drive_controller.update(ref. tração);
command  = TimedCommand{drive.normalized_command, steering.motor_command,
                        drive.brake_command, agora, steering.valid && drive.valid};
actuator_manager.applyCommands(command, vehicle_motion.getStopDetectionVelocity());
```

## Interrupções

`EncoderReader::begin(isr)` liga a mesma ISR aos dois canais de cada encoder (`CHANGE`). As ISRs (`isrRearEncoder`, `isrRightEncoder`, `isrLeftEncoder`) ficam em IRAM e só chamam `handleInterrupt()`.

## Sequência de inicialização (`setup()`)

1. `brake_servo.begin()` — **primeiro de tudo**: freio total imediato, para o servo não ficar sem pulsos (solto) durante o boot.
2. `Serial.begin(SERIAL_BAUDRATE)` e espera de 1 s.
3. Encoders (com as interrupções) e sensor de esterçamento.
4. `drive_motor`, `steering_motor` e `actuator_manager` — movimento bloqueado.
5. `safety_manager.begin()` — estado `DISABLED`.
6. Criação das tasks de sensores, controle e segurança.
7. `ros_node.begin(Serial)` — registra os tópicos e cria a `TaskRosCommunication`.

O `loop()` do Arduino fica vazio.
