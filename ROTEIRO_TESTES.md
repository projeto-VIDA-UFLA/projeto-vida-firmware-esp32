# Roteiro de testes - VIDA

Testes do mais simples ao mais complexo. Cada fase só começa quando a anterior
passou.

Em **todo** teste:
- alguém com a emergência por rádio na mão;
- gravar um rosbag: `ros2 bag record -a -o teste_<fase>_<data>`;
- anotar o que deu diferente do esperado (e o horário, para achar no bag).

Versões em teste:
- ESP32: branch `pendente-teste/ajustes-pre-merge`
- Odroid: branch `pendente-teste/navegacao` (submódulo `robot_state_ekf` em `eaadcd3`)
- Notebook: branch `pendente-teste/navegacao` do alto-nível

---

## Fase 0 - Preparação (motores desligados)

- [x] 0.1 Firmware gravado no ESP32.
- [x] 0.2 Odroid: `git submodule status` mostra `eaadcd3` sem `-` na frente;
  `docker compose build`; o executável do EKF existe na imagem.
- [x] 0.3 Notebook: `docker compose build` do alto-nível.
- [x] 0.4 Rede: `ping` entre notebook (192.168.10.2) e Odroid (192.168.10.1).
- [x] 0.5 `ros2 topic list` mostra os mesmos tópicos no notebook e no Odroid
  (mesmo `ROS_DOMAIN_ID`).

## Fase 1 - Bancada, rodas suspensas

- [ ] **1.1 micro-ROS:** o agente conecta e os tópicos aparecem:
  `ros2 topic hz /vida/sensors/encoders`, `/vida/sensors/steering`,
  `/vida/safety/status`. Reiniciar o agente: o ESP32 reconecta sozinho.
  *29/10: taxas OK (~50 Hz). Reconexão FALHOU com reinício rápido do agente
  (0,17 s fora do ar): sessão "zumbi", só volta com reset. Ver TODO.md.*
- [x] **1.2 Freio no boot:** ao ligar o ESP32, o freio aplica antes de qualquer
  outra coisa.
- [x] **1.3 Bloqueios de segurança** (`inhibit_flags` em `/vida/safety/status`):
  - enable físico desligado → flag 1;
  - sem gamepad → enable lógico ausente (flag 2 ou 8);
  - tudo OK → `motion_permitted: true`.
- [x] **1.4 Sensor de esterçamento** (`/vida/sensors/steering`):
  - rodas retas → perto de 0° (confere o offset de 1,6°);
  - rodas para a esquerda → ângulo **positivo** (REP-103);
  - anotar os extremos físicos de cada lado.
- [x] **1.5 Encoders:** girar cada roda à mão para frente → os três com
  velocidade positiva.
- [x] **1.6 Controle de esterçamento pelo gamepad:**
  - analógico para a esquerda → rodas para a esquerda;
  - saturação em ±20°;
  - parado, o motor não fica "caçando" (zona morta 0,4/1,2°).
- [x] **1.7 Gamepad perdido:** desligar o controle → referências a zero, freio
  total, enable lógico cai. Com o controle ligado, nenhuma queda falsa
  (timeout de 250 ms).
- [x] **1.8 Modo bancada:**
  - X por 5 s com as rodas paradas → vibração contínua e
    `/vida/safety/bench_mode` true;
  - O por 5 s → sai, com 3 pulsos;
  - tentar entrar com uma roda girando → recusado.
- [x] **1.9 Tração em modo bancada:**
  - R2 → a roda traseira gira e a velocidade M/T acompanha a referência;
  - L2 → freio total;
  - soltar o R2 → para e segura com o freio de parada.
  *29/10: achados e correções no firmware: corrida entre núcleos derrubava o
  modo bancada (corrigido); zona morta da controladora ~PWM 20-23
  (compensação: tração começa em PWM 21); atraso de ~0,25 s entre PWM e roda;
  em vazio a roda não gira abaixo de ~3 m/s (PWM 20); ganhos próprios da
  bancada (Kp = Ki = 0,008); sem freio automático na bancada; sem tração
  desacelerando. PlotJuggler aberto atrapalha o ESP32 (ver TODO.md).*
- [x] **1.10 Encoders discordantes** (fora do modo bancada): um pouco de R2 com a
  traseira suspensa → flag 4096, que fica travada. Rearmar: desligar e
  religar o enable físico.
- [ ] **1.11 Sem falsos positivos:** durante toda a bancada, nenhuma flag 512
  (esterçamento travado), 1024 (sentido invertido) ou 8192 (supervisor de
  tasks).

## Fase 2 - GPS e EKF com o carro parado, ao ar livre

- [ ] **2.1 GNSS:**
  - `/vida/sensors/gps/status`: fix, RTK FIXED, `correction_age_s` baixo;
  - `ros2 topic hz /vida/sensors/gps/fix` (esperado: 1 Hz).
- [ ] **2.2 EKF:**
  - `ros2 service call /robot_state_ekf_node/reset std_srvs/srv/Trigger`;
  - `/robot_state/status` passa por WAITING_ORIGIN → WAITING_HEADING;
  - `stationary: true` e bias do giroscópio estável (`/robot_state/gyro_bias`);
  - parado por 2 minutos, o rumo não deriva.

## Fase 3 - No chão, modo manual, na pista

Antes de sair:
- gravar o firmware mais recente (zona morta 21, sem tração desacelerando);
- velocidade máxima do R2: hoje o container do notebook está com 3 m/s (teste
  de bancada). Para as primeiras voltas, sugestão: 2 m/s
  (`docker exec ros_ws sed -i 's/max_speed_m_s: .*/max_speed_m_s: 2.0/'
  /vida_ws/install/control_pkg/share/control_pkg/config/gamepad.yaml && docker restart ros_ws`).
  Volta ao original (1 m/s) com `docker compose up -d --force-recreate`;
- **sem PlotJuggler aberto**; um bag por teste, com nome.

Bag sugerido (baixo nível + GPS/EKF):
`ros2 bag record -o pista_<teste> /vida/actuators/drive /vida/actuators/brake
/vida/actuators/steering /vida/sensors/encoders /vida/sensors/steering
/vida/control/drive/reference /vida/control/steering/reference
/vida/safety/status /vida/sensors/gps/fix /vida/sensors/gps/status
/vida/sensors/imu/data_raw /robot_state/odom /robot_state/status /gamepad/joy`

- [ ] **3.1 Velocidade mínima contínua** (o que define a velocidade do
  automático): reta plana, R2 só um pouco (~0,5 a 1 m/s pedido); o PI vai
  encostar em PWM 21-22. Anotar a velocidade estável das rodas dianteiras e se
  a controladora corta (trancos). Repetir no sentido contrário da reta.
- [ ] **3.2 Controle de velocidade:** R2 em patamares de ~10 s (1,5 → 2,0 →
  2,5 m/s, conforme a mínima do 3.1): sobressinal, erro em regime, trancos;
  PWM alternando 0/21 = PI na borda da zona morta.
- [ ] **3.3 Parada e frenagem:** soltar o R2 a 2 e a 2,5 m/s (freio de parada,
  sem tranco depois de soltar); L2 a 2 e a 2,5 m/s → **distância de parada**
  (marcar no chão ou pelo GPS). Define a margem dos abortos do automático.
- [ ] **3.4 Raio de giro:** círculo devagar com o analógico no fundo para cada
  lado; medir o diâmetro (trena ou GPS). Atenção: o gamepad limita a ±15°
  (o firmware aceita ±20°): o raio a 20° sai do entre-eixos efetivo medido a
  15°. Confirma (ou não) o raio mínimo de 3,8 m usado nos caminhos.
- [ ] **3.5 Escala dos encoders:** reta medida de 20 m; comparar a distância dos
  encoders com a do GPS (hoje os encoders medem ~7% a mais) e calibrar
  `DRIVE_WHEEL_DIAMETER_M`.
- [ ] **3.6 Plausibilidades sem falsos positivos:** em todas as voltas, nenhuma
  flag 512, 1024, 4096 ou 8192 em `/vida/safety/status`.
- [ ] **3.7 EKF andando:** depois de 2 m com RTK FIXED → RUNNING com heading do
  GPS; `/robot_state/odom` suave.
- [ ] **3.8 Gravar caminhos** (`path_recorder`): uma reta de ~20 m e uma volta no
  traçado; passar as duas por `path_tools from-recording`.
  *Antes: corrigir o Dockerfile (scipy) e o docker-compose do notebook, ver
  TODO.md.*
- [ ] **3.9 Velocidade do automático:** com a mínima do 3.1 e a distância de
  parada do 3.3, decidir a velocidade do modo automático (planejado 1 m/s;
  provável 2 a 2,5 m/s) e revalidar Stanley e manobra no simulador.

## Fase 4 - Modo automático, aumentando aos poucos

Lançar com `ros2 launch vida_navigation navigation.launch.py path_file:=<caminho.csv>`.

| Gamepad (5 s) | Ação | Vibração |
|---|---|---|
| △ | entra no automático | 1 pulso: aceito; 2 longos: Stanley no comando |
| □ | sai do automático | 4 curtos: de volta ao manual |
| L2 | aborta e freia (a qualquer momento) | contínua: aborto |
| — | pedido recusado | 6 pulsos rápidos (motivo em `/navigation/mode`) |

- [ ] **4.1 Manual pelo mode_manager:** dirigir normalmente; a resposta continua
  igual à do gamepad direto.
- [ ] **4.2 Recusas:** △ com o carro andando, e △ longe do início do caminho
  (mais de 3 m) → 6 pulsos.
- [ ] **4.3 Entrada até a manobra:** △ → PREPARANDO → CALIBRANDO → MANOBRA.
  Abortar com □; repetir abortando com L2. Tem que parar e só devolver a
  tração ao gamepad depois de soltar o R2.
- [ ] **4.4 Primeiro automático de verdade:** na reta gravada (caminho aberto),
  do início ao fim.
- [ ] **4.5 Uma curva larga:** caminho com raio de 6 m ou mais.
- [ ] **4.6 Abortos em movimento:** L2, □, emergência por rádio, perda de RTK
  (parar o cliente NTRIP).
- [ ] **4.7 Volta completa no circuito**, e só então ajustar `gain_k`
  (`config/navigation.yaml`).
