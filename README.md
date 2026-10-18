# Projeto VIDA — Firmware ESP32

Firmware de **controle de baixo nível** do **VIDA (Veículo Inteligente de Desenvolvimento Aplicado)**, protótipo de veículo autônomo do Laboratório de Mobilidade Terrestre (LMT) da UFLA, usado como plataforma de pesquisa em veículos inteligentes.

O ESP32 lê os sensores, controla os atuadores, garante a segurança do veículo e troca dados com o ROS 2 via **micro-ROS** (serial).

📖 **Documentação completa, classe por classe:** [`doc/README.md`](doc/README.md)

## Arquitetura do sistema

| Camada | Hardware | Responsabilidade |
|---|---|---|
| 1 | **ESP32 (este repositório)** | Controle de baixo nível (esterçamento, tração, freio), segurança, leitura de sensores, ponte micro-ROS |
| 2 | Odroid | Agente micro-ROS, EKF, GNSS/IMU (u-blox), enable lógico, referências de esterçamento e velocidade |
| 3 | Notebook | Controle de trajetória (Stanley, futuro); hoje, gamepad para testes |

## Hardware

**Atuadores**

| Atuador | Acionamento |
|---|---|
| Motor de tração DC 36 V, 1000 W | Driver de potência: 1 PWM + pino de direção (a ré ainda depende de um circuito de relés) |
| Motor de esterçamento DC 24 V | Ponte H com 2 PWMs (um por sentido) |
| Freio a disco traseiro | Servo de hobby puxando o cabo de aço |

**Sensores**

| Sensor | Uso |
|---|---|
| LVDT (ADC) | Ângulo de esterçamento |
| Encoder em quadratura no eixo traseiro (motriz, sem diferencial) | Velocidade e distância |
| Encoder em quadratura em cada roda dianteira | Velocidade e confirmação de veículo parado |

Pinagem: [`doc/utils/pinout.md`](doc/utils/pinout.md).

## Estrutura do repositório

```text
main/main.cpp            Instâncias, tasks FreeRTOS e setup()
include/, src/
  sensors/               EncoderReader, SteeringSensor, VehicleMotion
  control/               SteeringController, DriveController
  actuators/             ActuatorManager, DriveMotor, SteeringMotor, BrakeServo, MotorHardware
  safety/                SafetyManager, SafetyWatchdogs
  comunication/          CommandMailbox, RosCommunication, VidaRosNode
  utils/                 config.h (parâmetros), pinout.h, utils.h
interfaces/vida_interfaces/   Mensagens ROS 2 compiladas na biblioteca micro-ROS
lib/micro_ros_platformio/     Biblioteca micro-ROS para PlatformIO
doc/                     Documentação (mesma estrutura de classes) e diagramas
TODO.md                  Pendências
```

## Funcionamento resumido

Tasks FreeRTOS (detalhes em [`doc/main.md`](doc/main.md)):

| Task | Núcleo | Prioridade | Período | Função |
|---|---|---|---|---|
| `TaskSafety` | 1 | 4 | 5 ms | Permissão de movimento e freio |
| `TaskControl` | 1 | 3 | 5 ms | Controladores de esterçamento e tração |
| `TaskSensors` | 1 | 2 | 2 ms | LVDT e encoders |
| `TaskRosCommunication` | 0 | 2 | 1 ms (publica a cada 20 ms) | micro-ROS |

**Segurança:** o veículo só se move com o **enable físico** ligado, o **enable lógico** do ROS recebido e recente, as **referências** recentes, os **sensores** válidos e o **controlador** respondendo. Qualquer falha para os motores e leva o freio a total em rampa. Detalhes em [`doc/safety/SafetyManager.md`](doc/safety/SafetyManager.md).

**Controle:**
- **Esterçamento:** PI de posição com zona morta com histerese, anti-windup e proteção de fim de curso. Faixa de trabalho de ±20° e limite de segurança de ±25°.
- **Tração:** PI de **velocidade** em malha fechada sobre a média das rodas dianteiras (velocidade estimada pelo método M/T). O ESP32 comanda tração e freio; até 3 m/s, com rampa de aceleração e freio manual pelo ROS.

**Convenções:** ângulos em graus na convenção **REP-103 (esquerda positivo)**, comandos normalizados em `[-1, 1]`, freio de `0` (solto) a `1` (total).

## Tópicos ROS 2

| Direção | Tópico | Mensagem |
|---|---|---|
| Assina | `/vida/control/steering/reference` | `SteeringReference` |
| Assina | `/vida/control/drive/reference` | `DriveReference` |
| Assina | `/vida/safety/logical_enable` | `LogicalEnable` |
| Publica | `/vida/sensors/encoders` | `Encoders` |
| Publica | `/vida/sensors/steering` | `Steering` |
| Publica | `/vida/actuators/{steering,drive,brake}` | `Comands` |
| Publica | `/vida/safety/status` | `SafetyStatus` |

Definições e semântica de cada campo: [`doc/comunication/VidaRosNode.md`](doc/comunication/VidaRosNode.md).

## Build e gravação

Requer o [PlatformIO](https://platformio.org/). Na raiz do repositório:

```bash
pio run -e vida_firmware                    # compila
pio run -e vida_firmware --target upload    # grava no ESP32
```

O `platformio.ini` usa a porta `/dev/vida_esp32` (regra udev). Comente `upload_port`/`monitor_port` para detectar a porta automaticamente.

A primeira compilação constrói a biblioteca micro-ROS (ROS 2 Humble) com as mensagens de `interfaces/vida_interfaces/`, o que demora alguns minutos. **Depois de alterar qualquer `.msg`**, limpe e recompile:

```bash
pio run -e vida_firmware --target clean_microros
pio run -e vida_firmware
```

As mesmas mensagens precisam existir, com a mesma definição, nos pacotes `vida_interfaces` do Odroid e do notebook.

## Conexão com o ROS 2

A serial do ESP32 é usada pelo micro-ROS, então não há monitor serial de texto. No Odroid, o agente roda em container (repositório `projeto-vida-controle-baixo-nivel-odroid`):

```bash
docker run -it --rm --net=host --device /dev/vida_esp32 \
  microros/micro-ros-agent:humble serial --dev /dev/vida_esp32 -b 921600
```

Ao conectar, o ESP32 cria o nó `/vida/esp32_controller`, sincroniza o relógio com o agente e começa a publicar.

## Licença

MIT. Veja [`LICENSE`](LICENSE).
