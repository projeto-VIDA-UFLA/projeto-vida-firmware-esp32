# TODO - Firmware ESP32

## Proximos passos

- [ ] **Estimador de velocidade dos encoders.** Com Δt de 2 ms, um tick vale
  ~0,4 m/s no traseiro e ~0,8 m/s nos dianteiros. Medir por janela maior ou
  por periodo entre pulsos (`last_timestamp_` ja e gravado na ISR e nao e usado).
- [ ] **Controle longitudinal em malha fechada.** Hoje e malha aberta: o campo
  `velocity_m_s` de `DriveReference` carrega PWM em [-255, 255].
- [x] **Criterio de "parado" unico.** A troca de DIR agora exige os 3 encoders
  parados (`VehicleMotion::getStopDetectionVelocity`), como o alivio do freio.
- [ ] **Plausibilidade comando x movimento** (conceito da branch Whilker):
  esterçamento travado/invertido, tracao sem movimento, encoders discordantes.
  Tratar a roda traseira travada pelo freio para nao gerar falso "encoders
  discordam". Cobre o LVDT desconectado com entrada flutuante (le ruido, nao
  fica travado nem no extremo), que a validacao do SteeringSensor nao pega.
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

- [ ] README descreve uma versao antiga (SerialCommand, PidController, pinos, baud).
- [ ] Envs `steering_sensor`, `steering_motor` e `encoder_teste` do `platformio.ini`
  apontam para arquivos que nao existem mais.
- [ ] `monitor_speed` (460800) diferente de `SERIAL_BAUDRATE` (921600).
- [ ] `vida_interfaces` divergente entre os 3 repositorios (`SafetyStatus.msg`);
  usar uma fonte unica. `SteeringControllerOutput.msg` nao e mais publicado e
  pode sair na proxima recompilacao da libmicroros.
