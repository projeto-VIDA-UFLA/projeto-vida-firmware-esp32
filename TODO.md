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
- [ ] GPS a 2 Hz no ZED-F9R: configurado e confirmado no Odroid em 31/10
  (`measurement_rate_ms` = 500, `/vida/sensors/gps/fix` a 2,00 Hz). Falta so
  conferir em campo que o RTK FIXED se mantem como a 1 Hz.
- [ ] Depois de validar na pista: migrar o `vida_navigation` para C++ usando a
  versao Python como referencia.

## Limpeza

- [ ] `vida_interfaces` Fonte unica de verdade (submodulo)
