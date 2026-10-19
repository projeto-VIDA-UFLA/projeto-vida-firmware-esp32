# TODO - Firmware ESP32

## Proximos passos

- [x] **Estimador de velocidade dos encoders:** metodo M/T (janela de 20 ms).
- [x] **Controle longitudinal em malha fechada:** PI de velocidade com tracao
  e freio (`DriveController`).
- [ ] **Ensaios do controle longitudinal** (em trecho plano):
  - validar a velocidade M/T contra o GPS;
  - degraus de referencia 0,5 -> 1,0 m/s para ajustar Kp/Ki;
  - calibrar o freio: a velocidade constante, posicoes fixas de freio e
    desaceleracao medida -> `DRIVE_BRAKE_ENGAGE_POSITION` e
    `DRIVE_BRAKE_EFFORT_GAIN`;
  - identificacao em malha aberta com degraus de PWM (o modelo atual cobre
    so PWM 0-38 e tem baixa precisao).
- [x] **Criterio de "parado" unico.** A troca de DIR agora exige os 3 encoders
  parados (`VehicleMotion::getStopDetectionVelocity`), como o alivio do freio.
- [x] **Encoders discordantes** (`EncoderPlausibility`): traseira > dianteiras
  com tracao, falha retida.
- [x] **Modo bancada:** X por 5 s no gamepad (O por 5 s para sair), entra so
  parado, vibracao de confirmacao.
- [ ] Validar a vibracao no controle real (depende do joy_node conseguir
  inicializar o haptic do PS4: procurar "No haptic" no log).
- [x] **Plausibilidade do esterçamento** (`SteeringPlausibility`): travado e
  sentido invertido. Validar os limites nos ensaios (o teste de 12/10 teve so
  3,7 s de comando forte).
- [x] **Supervisor de tasks** (`TaskSupervisor`): corta os atuadores se uma
  task do nucleo 1 travar.
- [ ] Hardware: conferir pull-downs nas entradas da ponte H e do driver de
  tracao (durante um reset do ESP32 os pinos ficam sem sinal e o freio solta).

## Melhorias menores

- [ ] Limitar a parcela integral do esterçamento a +-0,5 (hoje ela sozinha
  pode comandar 100%).
- [ ] Limite de taxa da referencia de esterçamento usando
  `angular_velocity_deg_s` (recebido e ignorado). Util com o Stanley.
- [ ] Derivada do esterçamento sobre a medicao filtrada, nao sobre o erro
  (hoje Kd = 0, entao inofensivo).
- [ ] Telemetria opcional do controle longitudinal (referencia apos a rampa,
  esforco, P, I), ligada por `#define` para nao pesar na serial.
- [ ] Plausibilidade da tracao sem movimento (PWM aplicado e veiculo parado).
- [ ] Validar no veiculo a zona morta com histerese (0,4 / 1,2 grau) e a
  deteccao de LVDT travado / fundo de escala.

## Calibracao (teste de 12/10/2026)

- [ ] Validar no veiculo o offset de zero `STEERING_ZERO_OFFSET_DEG` (1,6 graus).
- [ ] Ganho do esterçamento: o modelo preve ~16% mais yaw rate (L efetivo
  1,38 m contra 1,125 m). Medir o entre-eixos real antes de mexer no polinomio.
- [ ] Encoders medem ~7% mais distancia que o GPS: conferir `DRIVE_WHEEL_DIAMETER_M`,
  se as rodas dianteiras tem o mesmo diametro da traseira e se
  `ENC_*_TICKS_PER_REV` sao contagens x4 (730 nao e multiplo de 4).

## Adiado

- [ ] `EncoderReader::handleInterrupt()` fora da IRAM (so importa com escrita
  em flash durante a operacao).

## Limpeza

- [ ] Envs `steering_sensor`, `steering_motor` e `encoder_teste` do `platformio.ini`
  apontam para arquivos que nao existem mais.
- [ ] `monitor_speed` (460800) diferente de `SERIAL_BAUDRATE` (921600).
- [ ] `vida_interfaces` divergente entre os 3 repositorios (`SafetyStatus.msg`);
  usar uma fonte unica. `SteeringControllerOutput.msg` nao e mais publicado e
  pode sair na proxima recompilacao da libmicroros.
