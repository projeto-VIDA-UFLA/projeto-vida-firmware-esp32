# Projeto VIDA - Firmware ESP32

Firmware embarcado para o ESP32 do Projeto VIDA. Esta versao controla, em fase de teste local, os atuadores de tracao e estercamento, le encoders em quadratura e sensor analogico de estercamento, aplica logica de seguranca e possui infraestrutura para comandos pela serial.

O projeto usa PlatformIO com framework Arduino para ESP32.

## Visao geral

O firmware esta organizado em modulos pequenos:

| Modulo | Responsabilidade |
| --- | --- |
| `main.cpp` | Inicializacao do hardware, objetos globais, interrupcoes dos encoders e tasks FreeRTOS. |
| `SerialCommand` | Parser de comandos seriais, handshake com o computador de bordo e alimentacao do watchdog. |
| `SafetyManager` | Maquina de estados de seguranca: boot, aguardando Odroid, desabilitado, habilitado, parada segura e falha. |
| `Watchdog` | Timeout dos comandos recebidos pela serial. |
| `VehicleController` | Fachada que coordena os controladores de tracao e estercamento. |
| `DriveVelocityController` | Controle longitudinal por velocidade, com rampa de aceleracao e malha de aceleracao. |
| `SteeringController` | Controle de estercamento em cascata: posicao para velocidade e velocidade para comando do motor. |
| `DriveMotor` | Saida PWM do motor longitudinal e pino de direcao/re. |
| `SteeringMotor` | Saidas PWM independentes para estercar para direita ou esquerda. |
| `EncoderReader` | Leitura de encoders em quadratura x4 via interrupcao. |
| `SteeringSensor` | Leitura ADC do sensor de estercamento e conversao para graus. |
| `PidController` | Controlador PID generico com saturacao e anti-windup simples. |

## Hardware alvo

- Placa: ESP32 DevKit 38 pinos (`esp32dev`)
- Framework: Arduino
- Comunicacao serial: `115200` baud
- PWM: 9 kHz, 8 bits
- Sensor de estercamento: entrada analogica ADC de 12 bits
- Encoders: tres encoders em quadratura

## Pinagem

A pinagem fica centralizada em [`include/pinout.h`](include/pinout.h).

| Funcao | Pino |
| --- | --- |
| Encoder traseiro A/B | GPIO 19 / GPIO 18 |
| Encoder dianteiro direito A/B | GPIO 35 / GPIO 34 |
| Encoder dianteiro esquerdo A/B | GPIO 33 / GPIO 32 |
| Sensor analogico de estercamento | GPIO 36 |
| PWM estercamento direita | GPIO 26 |
| PWM estercamento esquerda | GPIO 25 |
| PWM tracao | GPIO 27 |
| Direcao/re da tracao | GPIO 14 |
| Enable fisico de seguranca | GPIO 23 |

O pino `PIN_ENABLE_SAFETY` deve estar em nivel alto para permitir atuacao. Nivel baixo leva o sistema para estado de falha.

## Parametros principais

Os parametros ficam em [`include/config.h`](include/config.h). Alguns valores importantes:

| Parametro | Valor |
| --- | --- |
| Watchdog de comando | 300 ms |
| Periodo da task de seguranca | 10 ms |
| Periodo da task de controle | 10 ms |
| Periodo da task de sensores | 20 ms |
| Limite de velocidade longitudinal | +/- 2.0 m/s |
| Aceleracao padrao | 0.40 m/s2 |
| Aceleracao maxima segura | 0.80 m/s2 |
| Faixa de estercamento | -30 a +30 graus |
| Velocidade maxima padrao de estercamento | 20 graus/s |
| Velocidade maxima segura de estercamento | 45 graus/s |

## Estados de seguranca

O `SafetyManager` permite atuacao somente no estado `ENABLED`.

Fluxo esperado:

1. `BOOT`: estado inicial.
2. `WAITING_ODROID`: aguardando handshake `odroid_ready`.
3. `DISABLED`: Odroid pronto e watchdog OK, mas software enable desligado.
4. `ENABLED`: enable fisico alto, handshake OK, watchdog OK e `enable 1`.
5. `SAFE_STOP`: watchdog expirou.
6. `FAULT`: falha critica ou enable fisico em nivel baixo.

Quando a atuacao nao esta permitida, os motores sao parados.

## Comandos seriais

Os comandos implementados em `SerialCommand` usam linhas terminadas por `\n` ou `\r`.

| Comando | Descricao |
| --- | --- |
| `odroid_ready` | Marca o computador de bordo como pronto, alimenta o watchdog e responde `esp_ready`. |
| `odroid_ready 0` | Remove o estado de pronto, desabilita o software enable e zera os alvos. |
| `enable 1` | Habilita atuacao por software, desde que a seguranca permita. |
| `enable 0` | Desabilita atuacao por software. |
| `speed <m_s> <max_accel_m_s2>` | Define velocidade alvo e limite de aceleracao longitudinal. |
| `steer_target <deg> <max_vel_deg_s>` | Define angulo alvo de estercamento e limite de velocidade angular. |
| `reset_enc` | Zera os contadores e deltas dos encoders. |
| `status` | Solicita status. Atualmente apenas registra a requisicao; o status periodico ja e impresso pela task de debug. |

Exemplo de sequencia:

```text
odroid_ready
enable 1
speed 1.0 0.4
steer_target 10 20
```

Com `REQUIRE_ODROID_HANDSHAKE` habilitado, os comandos `enable`, `speed` e `steer_target` sao ignorados antes do handshake.

## Controle longitudinal

O controle de tracao usa os tres encoders para estimar a velocidade media do veiculo. O alvo de velocidade passa por uma rampa limitada por aceleracao. Em seguida:

1. Um PID de velocidade gera um setpoint de aceleracao.
2. Um PID de aceleracao gera comando normalizado para o motor.
3. O comando e saturado entre `-1.0` e `1.0` e convertido para PWM.

Comando positivo indica frente; comando negativo aciona o pino de re.

## Controle de estercamento

O sensor analogico e mapeado linearmente de `0.0 V` a `3.3 V` para a faixa `-30` a `+30` graus, com filtro passa-baixas simples.

O controle de estercamento funciona em cascata:

1. PID de posicao calcula a velocidade angular alvo.
2. A velocidade angular alvo e limitada.
3. PID de velocidade calcula o comando normalizado do motor.
4. Comando positivo aciona o PWM de direita; comando negativo aciona o PWM de esquerda.

Ha uma zona morta de posicao para evitar atuacao quando o erro angular ja esta pequeno.

## Build, upload e monitor serial

Instale o PlatformIO e execute na raiz do repositorio:

```bash
pio run
```

Para gravar no ESP32:

```bash
pio run --target upload
```

Para abrir o monitor serial:

```bash
pio device monitor
```

O `platformio.ini` esta configurado com:

```ini
upload_port = /dev/vida_esp32
monitor_port = /dev/vida_esp32
```

Comente essas linhas caso queira que o PlatformIO detecte automaticamente a porta USB.

## Testes

Existe um teste de leitura do sensor de estercamento em:

```text
test/test_steering_sensor/test_steering_sensor.cpp
```

Para executar:

```bash
pio test -e esp32dev
```

O teste imprime amostras de ADC, tensao e angulo em graus, alem de validar se a leitura ADC esta dentro da faixa esperada.

## Estrutura do repositorio

```text
include/    Headers, configuracao e pinagem
src/        Implementacao do firmware
test/       Testes PlatformIO/Unity
lib/        Bibliotecas locais opcionais do PlatformIO
```

## Observacoes do estado atual

- O banner de inicializacao identifica esta versao como "Fase 1", sem micro-ROS e voltada para teste local.
- O parser `SerialCommand::update()` esta implementado, mas no estado atual do `src/main.cpp` ele nao e chamado por nenhuma task nem pelo `loop()`.
- O `loop()` atual aplica continuamente `steering_motor.setCommand(0.1)` com `delay(200)`, comportamento tipico de teste manual do motor de estercamento.
- Antes de uso integrado com o computador de bordo, e recomendavel trocar esse teste manual por uma chamada periodica de `serial_command.update()` ou por uma task dedicada de comunicacao serial.

## Licenca

Este projeto esta licenciado sob a licenca MIT. Veja [`LICENSE`](LICENSE).
