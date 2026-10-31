# TODO - Firmware ESP32

Pendencias abertas (atualizado em 30/10/2026, depois dos ensaios no chao e do
1o Stanley). O que ja foi feito e validado esta no historico do git e no
ROTEIRO_TESTES.md.

## Prioridade

- [ ] **Esterçamento mais responsivo e sem sobressinal.** No 1o Stanley
  (30/10) a direcao chegou ~0,3 s atrasada e passou do comando (atraso puro
  ~0,15 s + resposta subamortecida), e o Stanley oscilou +-10 graus a 0,7 Hz.
  Hoje o controlador e quase so integral (Kp 0,007/grau). Proposta: subir o
  Kp (~0,03-0,05/grau), derivada sobre a medicao filtrada (nao sobre o erro)
  para frear a aproximacao, e integral so para o erro final. Ajustar no chao
  com degraus de +-10 graus andando devagar em reta (bag para analise).
- [ ] **Mensagens atrasando na entrada do ESP32.** Notebook e Odroid publicam
  a 25 Hz sem falhas, mas de vez em quando o ESP32 fica >300 ms sem ver uma
  mensagem: o pedido do modo bancada (bancada, 29/10) e o enable logico (1o
  Stanley, 30/10, bloqueou no meio da volta). Contornado com
  `BENCH_MODE_REQUEST_TIMEOUT_MS` = 1 s e `LOGICAL_ENABLE_WATCHDOG_TIMEOUT_MS`
  = 500 ms (e o mode_manager tolera 0,3 s de bloqueio). Investigar: contar
  mensagens recebidas por topico no ESP32; suspeita na fila de entrada do
  micro-ROS (`RMW_UXRCE_MAX_HISTORY` = 4 com 4 mensagens chegando juntas a cada
  40 ms) e no publish reliable que bloqueia a task do ROS.
- [ ] **Sessao micro-ROS "zumbi" apos reinicio rapido do agente** (bancada,
  29/10; repetiu em campo ao subir o docker compose do Odroid): o agente
  volta em <600 ms, o ESP32 nao percebe (3 pings perdidos) e o agente novo
  descarta tudo ate o reset. Contorno atual: resetar o ESP32 depois de
  reiniciar o Odroid. Proposta: em `AgentConnected`, sem mensagem do enable
  logico por ~2 s, ir para `AgentDisconnected` e recriar as entidades.

## Ensaios e ajustes

- [ ] **Freio em descida.** Freio total no plano: 2,5 m/s -> 0 em 2,5 m
  (~1,5-1,7 m/s2, servo em 1450 us); em descida de ~10% mal segura. Falta:
  esticar o cabo (ferramenta), testar o passo seguinte do servo (1400 us)
  se ele continuar sem aquecer/zumbir, e decidir `DRIVE_BRAKE_EFFORT_GAIN`
  3 -> 6 (freio automatico do PI segurando a velocidade em descida).
- [ ] **Eixo traseiro girando em falso** (flag 4096, 30/10): nao era a grama;
  o eixo motriz gira dentro do tarugo usado como enxerto e a tracao nao chega
  a roda. O encoder traseiro fica no eixo, entao marcou 2,2-2,7 m/s com o
  carro a ~0,9 m/s, e a falha de encoders discordantes pegou certo. Correcao
  mecanica em andamento (usuario); depois, confirmar que a 4096 nao aparece
  mais.
- [ ] **Escala dos encoders:** medem ~3,5% mais que o GPS (eram ~7%, mas parte
  vinha das curvas). Medir numa reta de 20 m e ajustar `DRIVE_WHEEL_DIAMETER_M`;
  conferir se `ENC_*_TICKS_PER_REV` sao contagens x4 (730 nao e multiplo de 4).

## Outros repositorios

- [ ] `robot_state_ekf` e privado: pedir ao Danilo acesso de leitura para a
  conta `Nucleo-LMT`, para o Odroid atualizar o submodulo sem token pessoal.
- [ ] Navegacao (alto-nivel): regravar a rota com o `path_recorder` aceitando
  FLOAT (`require_rtk_fixed:=false`).
- [ ] GPS a 2 Hz no ZED-F9R (Odroid): o driver ja tem `measurement_rate_ms`
  (31/10, padrao 1000). Falta testar: 500 no `zed_f9r.yaml`, conferir o ACK no
  log e `ros2 topic hz /vida/sensors/gps/fix` = 2 Hz.
- [ ] Depois de validar na pista: migrar o `vida_navigation` para C++ usando a
  versao Python como referencia.

## Limpeza

- [ ] `vida_interfaces` em 3 copias: as mensagens em comum estao identicas e o
  Odroid foi sincronizado com o alto-nivel (31/10), mas cada repositorio tem a
  sua copia. Fonte unica de verdade (submodulo) fica para depois.
