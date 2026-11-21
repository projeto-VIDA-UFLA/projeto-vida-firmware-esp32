# Projeto VIDA: firmware do ESP32

Firmware de **controle de baixo nível** do **VIDA (Veículo Inteligente de Desenvolvimento
Aplicado)**, protótipo de veículo autônomo do Laboratório de Mobilidade Terrestre (LMT) da UFLA.

O ESP32 lê os sensores, controla os atuadores de tração, esterçamento e freio, garante a
segurança do veículo e troca dados com o ROS 2 via **micro-ROS** (serial).

- **Documentação classe por classe:** [`doc/README.md`](doc/README.md)
- **Visão geral do projeto e como operar o veículo:** [projeto-VIDA-UFLA](https://github.com/projeto-VIDA-UFLA)

## Lugar na arquitetura

| Nível | Hardware | Repositório |
|---|---|---|
| 1 | **ESP32** | **Este repositório:** sensores, controle de baixo nível, segurança e ponte micro-ROS |
| 2 | Odroid N2 | [projeto-vida-controle-baixo-nivel-odroid](https://github.com/projeto-VIDA-UFLA/projeto-vida-controle-baixo-nivel-odroid): agente micro-ROS, GNSS RTK, EKF e enable lógico |
| 3 | Notebook | [projeto-vida-controle-alto-nivel](https://github.com/projeto-VIDA-UFLA/projeto-vida-controle-alto-nivel): Stanley, gerenciador de modos e gamepad; [projeto-vida-painel](https://github.com/projeto-VIDA-UFLA/projeto-vida-painel): painel de operação |

## Hardware

**Atuadores**

| Atuador | Acionamento |
|---|---|
| Motor de tração DC 36 V, 1000 W (roda traseira) | Driver de potência: 1 PWM e pino de direção |
| Motor de esterçamento DC 24 V | Ponte H com 2 PWMs (um por sentido) |
| Freio a disco traseiro | Servo puxando o cabo de aço |

**Sensores**

| Sensor | Uso |
|---|---|
| LVDT (ADC) | Ângulo de esterçamento |
| Encoder em quadratura no eixo traseiro (motriz) | Velocidade em modo bancada e verificação de patinação |
| Encoder em quadratura em cada roda dianteira | Velocidade do veículo e detecção de veículo parado |
| INA228 (I2C) | Tensão e corrente do banco de baterias de tração |

Pinagem: [`doc/utils/pinout.md`](doc/utils/pinout.md).

## Estrutura do repositório

```text
main/main.cpp                Instâncias, tasks FreeRTOS e setup()
include/, src/
  sensors/                   EncoderReader, SteeringSensor, VehicleMotion, BatteryMonitor
  control/                   SteeringController, DriveController
  actuators/                 ActuatorManager, DriveMotor, SteeringMotor, BrakeServo, MotorHardware
  safety/                    SafetyManager, SafetyWatchdogs, EncoderPlausibility,
                             SteeringPlausibility, TaskSupervisor
  comunication/              CommandMailbox, RosCommunication, VidaRosNode
  utils/                     config.h (parâmetros), pinout.h, utils.h
interfaces/vida_interfaces/  Mensagens ROS 2 compiladas na biblioteca micro-ROS
lib/micro_ros_platformio/    Biblioteca micro-ROS para PlatformIO (terceiros)
test/                        Testes unitários (ambiente vida_tests)
doc/                         Documentação (mesma estrutura das classes) e diagramas
```

## Funcionamento

**Tasks FreeRTOS** (detalhes em [`doc/main.md`](doc/main.md)):

| Task | Núcleo | Prioridade | Período | Função |
|---|---|---|---|---|
| `TaskSafety` | 1 | 4 | 5 ms | Permissão de movimento e freio de segurança |
| `TaskControl` | 1 | 3 | 5 ms | Controladores de esterçamento e de velocidade |
| `TaskSensors` | 1 | 2 | 2 ms | LVDT e encoders |
| `TaskRosCommunication` | 0 | 2 | 1 ms (publica a cada 20 ms) | micro-ROS |
| `TaskBattery` | 0 | 1 | 100 ms | Tensão e corrente do banco (INA228) |

As tasks do núcleo 1 trocam dados por *snapshots* protegidos, e só o `ActuatorManager` acessa
os atuadores. Um supervisor por temporizador corta os atuadores se alguma task do núcleo 1
deixar de executar.

**Segurança:** o veículo só se move com o **enable físico** ligado, o **enable lógico** do ROS
recebido e recente, as **referências** recentes (até 200 ms), os **sensores** válidos e o
**controlador** respondendo. Qualquer falha para os motores e leva o freio a total em rampa.
Detalhes em [`doc/safety/SafetyManager.md`](doc/safety/SafetyManager.md).

**Controle:**
- **Esterçamento:** PID de posição a 200 Hz, com derivativo sobre a medição, zona morta com
  histerese, integral condicionada com *anti-windup*, compensação da zona morta do motor,
  limitador de taxa da referência (40°/s) e proteção de fim de curso. Faixa de trabalho de
  ±20° e limite de segurança de ±25°.
- **Velocidade:** PI de velocidade a 200 Hz sobre a média das rodas dianteiras (velocidade
  estimada pelo método M/T), com rampa de aceleração, *anti-windup* e coordenação entre
  tração, roda livre e freio. Velocidade máxima de 2,5 m/s.

**Convenções:** ângulos em graus na convenção **REP-103 (esquerda positivo)**, comandos
normalizados em `[-1, 1]` e freio de `0` (solto) a `1` (total).

## Tópicos ROS 2

| Direção | Tópico | Mensagem |
|---|---|---|
| Assina | `/vida/control/steering/reference` | `SteeringReference` |
| Assina | `/vida/control/drive/reference` | `DriveReference` |
| Assina | `/vida/safety/logical_enable` | `LogicalEnable` |
| Publica | `/vida/sensors/encoders` | `Encoders` |
| Publica | `/vida/sensors/steering` | `Steering` |
| Publica | `/vida/sensors/battery` | `BatteryStatus` |
| Publica | `/vida/actuators/{steering,drive,brake}` | `Comands` |
| Publica | `/vida/safety/status` | `SafetyStatus` |

Definição e semântica de cada campo: [`doc/comunication/VidaRosNode.md`](doc/comunication/VidaRosNode.md).

## Compilar e gravar

Requer o [PlatformIO](https://platformio.org/). No VIDA, o firmware é gravado a partir da
Odroid, onde o ESP32 está conectado. Na raiz do repositório:

```bash
pio run -e vida_firmware                    # compila
pio run -e vida_firmware --target upload    # grava no ESP32
pio test -e vida_tests                      # testes unitários (executados no ESP32)
```

O `platformio.ini` usa a porta `/dev/vida_esp32` (regra udev da Odroid). Em outro computador,
comente `upload_port` e `monitor_port` para detectar a porta automaticamente.

A primeira compilação constrói a biblioteca micro-ROS (ROS 2 Humble) com as mensagens de
`interfaces/vida_interfaces/`, o que demora alguns minutos e precisa de internet. **Depois de
alterar qualquer `.msg`**, limpe e recompile:

```bash
pio run -e vida_firmware --target clean_microros
pio run -e vida_firmware
```

As mesmas mensagens precisam existir, com a mesma definição, no pacote `vida_interfaces` da
Odroid e do notebook.

## Conexão com o ROS 2

A serial do ESP32 é usada pelo micro-ROS, então não há monitor serial de texto. O agente
micro-ROS roda em container na Odroid e sobe sozinho com ela. Ao conectar, o ESP32 cria o nó
`/vida/esp32_controller`, sincroniza o relógio com o agente e começa a publicar.

## Licença

MIT. Veja [`LICENSE`](LICENSE).
