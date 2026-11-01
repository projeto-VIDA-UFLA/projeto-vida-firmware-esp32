# TODO - Firmware ESP32

Pendencias abertas (atualizado em 30/10/2026, depois dos ensaios no chao e do
1o Stanley). O que ja foi feito e validado esta no historico do git e no
ROTEIRO_TESTES.md.

## Prioridade

- [ ] **Esterçamento mais responsivo e sem sobressinal.** No 1o Stanley
  (30/10) a direcao chegou ~0,3 s atrasada e passou do comando, e o Stanley
  oscilou +-10 graus a 0,7 Hz. Causa (bags de 30/10): o motor quase nao se
  move abaixo de PWM ~80, e o controlador ficava no piso de PWM 38 esperando
  a integral subir. Implementado em 31/10, falta testar no chao: compensacao
  da zona morta do motor (0,25), Kp 0,05, D sobre a velocidade medida (0,002),
  integral com |erro| <= 4 graus ou com a direcao parada (Ki 0,1).
  31/10, 2o Stanley: andou bem, mas parou 2x no mesmo ponto com flag 512 (a
  integral so com |erro| <= 4 graus nao vencia a carga na curva de raio 2,3 m;
  corrigido com a integral tambem com a direcao parada, falta testar). Na simulacao com o motor
  medido: degrau de 10 graus em 0,30 s com 0,5 grau de sobressinal (era 0,67 s
  e 5,3). Teste: andando devagar em reta, analogico de um lado ao outro e de
  volta ao centro (bag com `/vida/control/steering/reference`,
  `/vida/sensors/steering`, `/vida/actuators/steering`). Depois: refazer o
  modelo do esterçamento no simulador do alto-nivel e reajustar o Stanley
  (`gain_k`, `yaw_damping_s`).
- [ ] **Mensagens atrasando na entrada do ESP32.** Notebook e Odroid publicam
  a 25 Hz sem falhas, mas de vez em quando o ESP32 fica >300 ms sem ver uma
  mensagem: o pedido do modo bancada (bancada, 29/10) e o enable logico (1o
  Stanley, 30/10, bloqueou no meio da volta). Implementado em 31/10, falta
  testar:
  - assinaturas best-effort (reliable segurava tudo ate a retransmissao);
  - buffer de recepcao da UART de 256 para 2048 bytes;
  - publish reliable espera no maximo 20 ms pela confirmacao (era 1000 ms).

  31/10, 2o e 3o Stanley: ainda perdia so o enable logico (flag 8, ate ~1 s,
  so no automatico; no notebook ele chega sem falhas). Hipotese: reserva de 4
  vagas de recepcao do micro-ROS cheia, e o enable chega sempre por ultimo.
  Feito: `RMW_UXRCE_MAX_HISTORY` 4 -> 10 (libmicroros recompilada) e
  `LOGICAL_ENABLE_WATCHDOG_TIMEOUT_MS` 500 -> 1500 ms. Com 1500 ms um buraco
  de ~1 s fica invisivel: para confirmar a hipotese, rodar uma volta no
  automatico com o watchdog em 500 ms; sem flag 8, deixar em 500 ms.
  Teste: no bag, `logical_enable_fresh`, `drive_reference_fresh` e
  `steering_reference_fresh` de `/vida/safety/status` nao caem com o gamepad
  ligado (comparar com os bags de 30/10); `ros2 topic hz` dos topicos do
  ESP32 continua em 50 Hz. Se der certo, voltar a pensar nos contornos
  (`BENCH_MODE_REQUEST_TIMEOUT_MS`, `LOGICAL_ENABLE_WATCHDOG_TIMEOUT_MS`,
  tolerancia do mode_manager).
- [ ] **Sessao micro-ROS "zumbi" apos reinicio rapido do agente** (bancada,
  29/10; campo, 30/10): implementado em 31/10, falta testar. Firmware: 3 s
  sem nenhuma mensagem recebida (depois da 1a da sessao) recria a sessao.
  Odroid: o agente espera 2 s antes de abrir a serial. Teste: com o notebook
  rodando, `docker compose restart micro_ros_agent` (e tambem
  `docker compose up -d --force-recreate`); os topicos do ESP32 voltam sem
  reset em poucos segundos.

## Ensaios e ajustes

- [ ] **Freio (uso no plano).** Freio total no plano: 2,5 m/s -> 0 em 2,5 m
  (~1,5-1,7 m/s2, servo em 1450 us). Falta: esticar o cabo (ferramenta) e, se
  o servo continuar sem aquecer/zumbir, testar o passo seguinte (1400 us),
  medindo a distancia de parada no plano. Descidas nao sao o foco: o veiculo
  e usado no plano.
- [ ] **Entre-eixos efetivo, com a chaveta definitiva** (instalada em 31/10).
  Em 30/10 a roda traseira patinava no eixo por falta da chaveta (flag 4096
  pegou certo). No trajeto pista -> laboratorio de 31/10 (bag 16:51) a
  traseira chegou a 5,3 m/s com as dianteiras a 1,1-1,5 m/s, mas foi uma
  valeta, nao a chaveta; rajadas de ate 0,37 s, sem 4096. Falta:
  - confirmar que a 4096 nao aparece em uso normal (o teto agora e PWM 64);
  - medir de novo o entre-eixos efetivo: em 31/10 as curvas deram ~1,17 m
    (geometrico 1,125); os 1,42 m de 30/10 foram medidos com a roda
    patinando. Se confirmar, ajustar `feedforward_wheelbase_m` do Stanley,
    o raio minimo do `path_tools` (4 m) e o `wheelbase_m` do simulador.
- [ ] **Escala dos encoders:** medem ~3,5% mais que o GPS (eram ~7%, mas parte
  vinha das curvas). Medir numa reta de 20 m e ajustar `DRIVE_WHEEL_DIAMETER_M`;
  conferir se `ENC_*_TICKS_PER_REV` sao contagens x4 (730 nao e multiplo de 4).

- [ ] **Monitor de bateria (INA228), 31/10:** trazido da branch `Alexandre`
  para o firmware atual (`BatteryMonitor`, `/vida/sensors/battery`). Falta:
  - preencher `BATTERY_SHUNT_RESISTANCE_OHM` com o shunt instalado (hoje 0:
    corrente NaN) e conferir o sinal com o veiculo tracionando;
  - `pio run -t clean_microros` (mensagem nova) e testar no hardware;
  - conferir a faixa do SoC (35,8-40 V) com a curva de descarga do banco.

## Outros repositorios

- [ ] **Documentar o `.env` do Odroid** (a definir o melhor lugar na revisao da
  documentacao): ele precisa ser atualizado no proprio Odroid, ao lado do
  `docker-compose.yml`, e nao vem com o `git pull` (esta no `.gitignore`). O
  `docker-compose.yml` le `VIDA_NTRIP_USERNAME` e `VIDA_NTRIP_PASSWORD`. Sem valores
  aqui: credenciais nao entram no repositorio.
- [ ] `robot_state_ekf` e privado: pedir ao Danilo acesso de leitura para a
  conta `Nucleo-LMT`, para o Odroid atualizar o submodulo sem token pessoal.
- [ ] Navegacao (alto-nivel): regravar a rota com o `path_recorder` aceitando
  FLOAT (`require_rtk_fixed:=false`).
- [ ] GPS a 2 Hz no ZED-F9R: configurado e confirmado no Odroid em 31/10
  (`measurement_rate_ms` = 500, `/vida/sensors/gps/fix` a 2,00 Hz). Falta so
  conferir em campo que o RTK FIXED se mantem como a 1 Hz.
- [ ] Depois de validar na pista: migrar o `vida_navigation` para C++ usando a
  versao Python como referencia.

## Limpeza

- [ ] `vida_interfaces` Fonte unica de verdade (submodulo)
