#pragma once

// Parametros do firmware. Pinos ficam em pinout.h.
// Documentacao de cada parametro por modulo: doc/utils/config.md.

// ============================================================
// Comunicacao serial
// ============================================================

// Transporte do micro-ROS; o agente no Odroid deve usar o mesmo baud rate.
#define SERIAL_BAUDRATE 921600
// Buffer de recepcao da UART (padrao do Arduino: 256 bytes, cheio em ~3 ms a
// 921600). As 4 mensagens de entrada chegam juntas a cada 40 ms e a task do
// ROS pode ficar alguns ms ocupada publicando: com 256 bytes perdia quadros.
#define SERIAL_RX_BUFFER_SIZE 2048

// ============================================================
// PWM e canais LEDC
// ============================================================

#define PWM_FREQUENCY_HZ 9000
#define PWM_RESOLUTION_BITS 8

#define PWM_CH_DRIVE 0
#define PWM_CH_STEER_RIGHT 1
#define PWM_CH_STEER_LEFT 2
// Canal 4: timer LEDC proprio. Os canais 0/1 e 2/3 compartilham timer, entao
// usar o 3 mudaria a frequencia do estercamento para 50 Hz.
#define PWM_CH_BRAKE 4

// Sem uso: a conversao de comando para duty nao consulta esta constante.
// #define PWM_MAX_DUTY 255

// ============================================================
// Periodos das tarefas
// ============================================================

#define TASK_SENSOR_PERIOD_MS 2        // 500 Hz
#define TASK_SAFETY_PERIOD_MS 5        // 200 Hz
#define TASK_CONTROL_PERIOD_MS 5      // 200 Hz
#define TASK_COMUNICATION_PERIOD_MS 20 // 50 Hz (periodo de publicacao ROS)

// ============================================================
// Prioridades das tarefas
// ============================================================

#define PRIORITY_SAFETY 4
#define PRIORITY_CONTROL 3
#define PRIORITY_SENSORS 2

// Sem uso enquanto as respectivas tarefas nao existirem.
// #define PRIORITY_DEBUG 1

// ============================================================
// Seguranca e watchdogs
// ============================================================

// O enable chega a 25 Hz, mas no modo automatico o ESP32 chegou a ficar ~1 s
// sem ve-lo (31/10: flag 8 duas vezes na manobra; no notebook ele saiu sem
// falhas). Era 300 ate 30/10 e 500 ate 31/10. Perder o gamepad continua
// parando o veiculo em 250 ms: o gamepad publica enable falso (joy_timeout) e
// as referencias param (watchdog de 200 ms). Investigacao: TODO.md.
#define LOGICAL_ENABLE_WATCHDOG_TIMEOUT_MS 1500
#define DRIVE_REFERENCE_WATCHDOG_TIMEOUT_MS 200
#define STEERING_REFERENCE_WATCHDOG_TIMEOUT_MS 200

// Tempo maximo entre ciclos validos da task de sensores.
#define SENSOR_TASK_WATCHDOG_TIMEOUT_MS 20
// Janela concedida na inicializacao para a primeira conclusao da task.
#define SENSOR_TASK_STARTUP_GRACE_MS 500
// Idade maxima da amostra do sensor usada pelo controle de estercamento.
#define STEERING_SENSOR_SNAPSHOT_TIMEOUT_MS 20
// Idade maxima de uma saida concluida e valida do controlador.
#define CONTROL_OUTPUT_WATCHDOG_TIMEOUT_MS 20

// Encoders discordantes: com tracao aplicada, roda traseira mais rapida que a
// media das dianteiras por mais que MARGIN durante TIME -> falha retida
// (encoder dianteiro sem sinal ou roda traseira girando no ar). Rearme:
// enable fisico desligado ou enable logico false.
// Nos testes de 12/10 e 16/08/2026 a diferenca chegou a 0,49 m/s e, por
// 0,18 s, a 1,35 m/s (patinagem na arrancada): sem disparo com 0,8 m/s/0,5 s.
#define ENCODER_DISAGREE_MARGIN_M_S 0.8f
#define ENCODER_DISAGREE_TIME_MS 500

// Plausibilidade do esterçamento (comando aplicado x angulo medido). So
// avalia |comando| >= MIN_COMMAND: no teste de 12/10/2026, com o piso de 0,15
// (PWM 38) a roda ficou ate 2,07 s sem avancar 0,5 grau (atrito estatico);
// com comando >= 0,3 (PWM 77), no maximo 0,42 s, e o maior recuo contra o
// comando foi 0,59 grau. Falhas retidas ate o rearme (como encoders).
#define STEERING_PLAUSIBILITY_MIN_COMMAND 0.3f
// Travado so com erro grande: parado no chao, um erro de 0,6 grau fora da zona
// morta fez a integral subir ate o comando forte sem a direcao mover (atrito),
// e a falha acusava travamento (flag 512, 30/10).
#define STEERING_STALL_MIN_ERROR_DEG 2.0f
// E so com o veiculo andando: parado na grama (30/10) o motor nao venceu o
// atrito do pneu com 6 graus de erro e a falha acusava travamento.
#define STEERING_STALL_MIN_SPEED_M_S 0.2f
// Travado: sem avancar STALL_MIN_PROGRESS no sentido comandado por STALL_TIME.
#define STEERING_STALL_MIN_PROGRESS_DEG 0.5f
#define STEERING_STALL_TIME_MS 1000
// Sentido invertido: recuo de WRONG_DIRECTION contra o comando, depois de GRACE.
#define STEERING_WRONG_DIRECTION_DEG 2.0f
#define STEERING_WRONG_DIRECTION_GRACE_MS 200

// Supervisor de tasks: timer de TASK_SUPERVISOR_PERIOD_MS no nucleo 0 (fora
// das tasks vigiadas). TaskSensors, TaskControl ou TaskSafety sem sinal de
// vida por TASK_SUPERVISOR_TIMEOUT_MS -> PWMs zerados e freio total escritos
// direto no hardware, falha retida ate reiniciar. Nao reinicia sozinho: no
// reboot o servo fica sem pulsos e o freio solta.
#define TASK_SUPERVISOR_PERIOD_MS 10
#define TASK_SUPERVISOR_TIMEOUT_MS 50
#define TASK_SUPERVISOR_STARTUP_GRACE_MS 500
// Watchdog do ESP-IDF (segundos inteiros): vigia o proprio supervisor e a
// idle task do nucleo 0. Estourou -> reinicio (falha dupla). Mantido no
// padrao do framework (5 s): o transporte serial do micro-ROS pode ocupar o
// nucleo 0 por ~1 s ao criar entidades, e um disparo falso reinicia o ESP32
// com o freio solto durante o boot.
#define TASK_WDT_TIMEOUT_S 5

// Modo bancada (veiculo suspenso): pedido pelo gamepad em
// /vida/safety/bench_mode_request. Entra so com o veiculo parado; sai assim
// que o pedido for false ou deixar de chegar por REQUEST_TIMEOUT. Sempre
// inicia em modo normal. Na bancada: velocidade realimentada pelo encoder
// traseiro, verificacao de encoders discordantes desligada e velocidade
// maxima reduzida.
// O pedido chega a 25 Hz. No teste de 29/10 ele atrasou mais de 300 ms de vez
// em quando entre o agente e o ESP32 (o enable logico nao), e o modo bancada
// piscava; 1 s tolera isso. Perder o gamepad continua bloqueando o veiculo
// pelo enable logico (LOGICAL_ENABLE_WATCHDOG_TIMEOUT_MS).
#define BENCH_MODE_REQUEST_TIMEOUT_MS 1000
// 3 m/s: com a roda em vazio a controladora ja passa de 2 m/s logo acima da
// zona morta, entao 1 m/s nao da para manter na bancada (teste de 29/10).
#define BENCH_MODE_MAX_SPEED_M_S 4.0f

// Modo manobra (empurrar o veiculo a mao): freio solto e tracao bloqueada.
// Pedido pelo gamepad (R1 por 3 s) com DriveReference.brake_command <=
// MANEUVER_BRAKE_REQUEST. Funciona tambem com o enable fisico desligado, mas:
// - so entra com o veiculo parado e o gamepad vivo (enable logico true);
// - desligar o enable fisico (emergencia) cancela; para soltar de novo o
//   pedido precisa ser desfeito e refeito;
// - passar de MANEUVER_MAX_SPEED_M_S (rolando sozinho) cancela;
// - sem referencia de velocidade recente (DRIVE_REFERENCE_WATCHDOG_TIMEOUT_MS)
//   ou sem enable logico, cancela.
#define MANEUVER_BRAKE_REQUEST -0.5f
// 2,5 m/s desde 30/10: com 1,0 m/s empurrar o veiculo a mao ja cancelava.
#define MANEUVER_MAX_SPEED_M_S 2.5f

// ============================================================
// Sensor de estercamento
// ============================================================

#define ADC_RESOLUTION_BITS 12

// Limite de seguranca: passando dele o controlador bloqueia o comando que
// afastaria ainda mais. O lado mais curto chega ao fim fisico em ~28,9 graus
// (ADC 0 no polinomio).
#define STEERING_MIN_DEG -25.0f
#define STEERING_MAX_DEG 25.0f

// Faixa de trabalho: referencias fora dela sao limitadas. Fica abaixo do
// limite de seguranca para deixar margem (gamepad hoje: 15; Stanley: ate 20).
#define STEERING_REFERENCE_MIN_DEG -20.0f
#define STEERING_REFERENCE_MAX_DEG 20.0f

// Conversao ADC -> angulo da roda virtual do modelo de bicicleta (graus).
// Polinomio cubico ancorado no centro, com d = adc - STEERING_ADC_CENTER:
//   delta = A*d + B*d^2 + C*d^3
// A curva e assimetrica: nao assumir simetria em torno do centro.
#define STEERING_ADC_CENTER 2046
#define STEERING_POLY_A -2.14862e-2f
#define STEERING_POLY_B -3.09559e-7f
#define STEERING_POLY_C 1.42370e-9f

// Convencao de sinal do angulo de estercamento em todo o sistema.
// 0: convencao dos dados de calibracao (esquerda negativo, direita positivo).
// 1: REP-103 (esquerda positivo, direita negativo).
// A mesma flag inverte o angulo medido e o sentido do comando do motor, de
// modo que referencia ROS, medicao e atuacao fiquem sempre na mesma
// convencao. STEERING_MIN_DEG/STEERING_MAX_DEG seguem a convencao escolhida.
#define STEERING_INVERT_SIGN 1
#define STEERING_ANGLE_SIGN (STEERING_INVERT_SIGN ? -1.0f : 1.0f)

// Offset de zero, na convencao dos dados de calibracao (direita positivo):
// angulo que o polinomio indica com o veiculo andando reto. E subtraido do
// polinomio antes da troca de sinal. Teste de 12/10/2026: 1,6. Teste no chao
// de 30/10 (curvatura pelo giroscopio x angulo medido): com 1,6 o veiculo
// andava reto com +2,2 graus medidos (REP-103), curvando para a direita com 0;
// novo valor 1,6 - 2,2 = -0,6. Incerteza ~1 grau (inclinacao lateral da via):
// confirmar com reta de ida e volta.
#define STEERING_ZERO_OFFSET_DEG -0.6f

#define STEERING_FILTER_ALPHA 0.05f
#define STEERING_VELOCITY_FILTER_ALPHA 0.01f

// Validacao da leitura do LVDT (amostras a 500 Hz).
// Fundo/fim de escala: em operacao o ADC nunca chega aos extremos (no teste
// de 12/10/2026 ficou entre 1079 e 2815; ADC 40 ~ -28,5 graus, 4055 ~ +34,5).
// Leitura fora da faixa por mais que RAIL_TIME indica sensor desconectado,
// em curto ou alem do curso.
#define STEERING_ADC_VALID_MIN 40
#define STEERING_ADC_VALID_MAX 4055
#define STEERING_ADC_RAIL_TIME_MS 10
// Leitura travada: o LVDT sempre oscila (desvio-padrao ~25 counts parado; a
// menor variacao em 500 ms no teste foi de 30 counts). Ficar dentro de
// +-STUCK_TOLERANCE por STUCK_TIME indica sinal perdido. Rever se o ruido do
// sensor for reduzido no hardware.
#define STEERING_ADC_STUCK_TOLERANCE 3
#define STEERING_ADC_STUCK_TIME_MS 500

// Sem uso: permanecem documentadas para uma futura conversao ADC/tensao.
// #define ADC_MAX_VALUE 4095.0f
// #define ADC_REF_VOLTAGE 3.3f

// ============================================================
// Encoders e estimativa de movimento longitudinal
// ============================================================

#define ENC_REAR_TICKS_PER_REV 1440.0f
#define ENC_FRONT_TICKS_PER_REV 730.0f
#define DRIVE_WHEEL_DIAMETER_M 0.366f

// Velocidade pelo metodo M/T: ticks contados numa janela de pelo menos
// ENCODER_VELOCITY_WINDOW_MS, divididos pelo tempo exato entre a primeira e a
// ultima borda (instante gravado na ISR). Atraso ~metade da janela.
#define ENCODER_VELOCITY_WINDOW_MS 20
// Sem nenhuma borda por este tempo a velocidade e zero (1 tick em 200 ms:
// ~8 mm/s nos dianteiros, ~4 mm/s no traseiro).
#define ENCODER_ZERO_VELOCITY_TIMEOUT_MS 200

// Filtro apos o M/T: so suaviza os degraus entre janelas. Em simulacao
// (bordas de quadratura desiguais e jitter na ISR), janela de 20 ms + alpha
// 0,5 da atraso de 25-28 ms, contra 39 ms do estimador antigo (delta de ticks
// em 2 ms + alpha 0,05), sem ruido de quantizacao.
#define DRIVE_VELOCITY_FILTER_ALPHA 0.5f
#define DRIVE_ACCEL_FILTER_ALPHA 0.01f

// Velocidade maxima realimentada para considerar o veiculo parado.
#define DRIVE_STOP_SPEED_THRESHOLD_MPS 0.05f
// Tempo continuo abaixo do limiar exigido antes de trocar DIR.
#define DRIVE_STOP_CONFIRMATION_TIME_MS 200
// Idade maxima aceita para a velocidade usada na autorizacao de inversao.
#define DRIVE_VELOCITY_MEASUREMENT_TIMEOUT_MS 50
// Tempo sem torque depois de alterar DIR antes de reaplicar PWM.
#define DRIVE_DIRECTION_DEADTIME_MS 20

// ============================================================
// Freio (servo de hobby que puxa o cabo de aco)
// ============================================================

#define BRAKE_SERVO_FREQUENCY_HZ 50
#define BRAKE_SERVO_RESOLUTION_BITS 16

// 0: freio nao calibrado. Nenhum pulso e enviado ao servo (freio inoperante).
// So passar para 1 depois de medir BRAKE_APPLIED_US no veiculo.
#define BRAKE_CALIBRATED 1

// Largura de pulso para freio solto e freio total, medidas no veiculo.
// O pulso diminui para frear: o servo puxa o cabo no sentido de 2500 -> 1450.
// 30/10: freio fraco no chao (0,36 m/s2 na descida); sem ferramenta para
// esticar o cabo, o freio total desce em passos (1600 -> 1500 -> 1450),
// conferindo a cada teste se o servo aguenta (sem aquecer nem zumbir em
// 1500). O antigo total (1600) virou o HOLD.
#define BRAKE_RELEASED_US 2500
#define BRAKE_APPLIED_US 1450

// Com o veiculo parado o servo puxa no maximo ate este pulso, para nao forcar
// o servo mantendo o freio. Deve ficar entre BRAKE_RELEASED_US e
// BRAKE_APPLIED_US. Volta ao freio total assim que houver movimento.
#define BRAKE_HOLD_US 1600
// Tempo continuo com os tres encoders abaixo de DRIVE_STOP_SPEED_THRESHOLD_MPS
// antes de aliviar para BRAKE_HOLD_US. Os dianteiros nao sao freados, entao
// detectam o veiculo deslizando com a roda traseira travada.
#define BRAKE_HOLD_CONFIRMATION_MS 1000

// Rampa usada somente em falhas: fracao do curso por segundo (1.0 = 1 s de
// solto a freio total). Na operacao normal o freio segue a referencia.
#define BRAKE_FAULT_RATE_PER_S 1.0f

// Acima desta posicao do freio (0 a 1) o PWM de tracao e zerado.
#define BRAKE_DRIVE_CUTOFF 0.05f

// ============================================================
// Controle direto de posicao do estercamento
// ============================================================

// Zona morta com histerese. Dentro dela o comando e zero e a integral e
// zerada. O controlador entra quando |erro| <= ENTER e so volta a atuar
// quando |erro| > EXIT. A faixa entre os dois (0,8 grau) cobre o ruido do
// angulo filtrado parado (pico a pico ~0,76 grau no teste de 12/10/2026),
// evitando o liga-desliga na borda. Valores iniciais: ajustar em ensaio.
#define STEERING_DEADBAND_ENTER_DEG 0.4f
#define STEERING_DEADBAND_EXIT_DEG 1.2f
// Compensacao da zona morta do motor: fora da zona morta do erro, o comando
// do PID e somado a +-STEERING_DEADZONE_COMPENSATION. Nos bags de 30/10 (no
// chao, andando) a direcao quase nao se move abaixo de PWM ~75-80 (0-7 graus/s
// ate PWM 80, ~45 graus/s em PWM 115, ~70 em PWM 150). O antigo piso de 0,15
// (PWM 38) deixava o motor parado ate a integral subir: ~0,3 s de atraso e,
// ao arrancar, a integral acumulada passava 3-7 graus da referencia.
// 0,25 = PWM 64: logo abaixo do arranque, o P completa o resto.
#define STEERING_DEADZONE_COMPENSATION 0.25f
#define MAX_ABS_STEER_COMMAND 1.0f
// dt acima disto (task atrasada) reseta o controlador.
#define STEERING_CONTROL_MAX_DT_S 0.1f
// Intervalo sem PWM entre sentidos opostos do motor de estercamento.
#define STEERING_DIRECTION_DEADTIME_US 1000

// Kp: comando normalizado por grau.
// Ki: comando normalizado por grau-segundo.
// Kd: comando normalizado por grau por segundo, sobre a velocidade MEDIDA
// (nao sobre o erro): freia a aproximacao sem chutar nos degraus da referencia.
// INTEGRAL_MIN/MAX limitam a integral do erro (grau-segundo).
// Ganhos de 31/10 escolhidos simulando este codigo com o motor medido nos bags
// de 30/10 (degrau de 10 graus: 0,30 s e 0,5 grau de sobressinal; seno de
// 8 graus a 0,7 Hz: 92 ms de atraso. O anterior, Kp 0,007 / Ki 0,1 com piso
// 0,15: 0,67 s, 5,3 graus e 342 ms). Ki de 0,05 para 0,1 em 31/10 junto com a
// integral com a direcao parada: vence uma carga de ~90 PWM em < 0,9 s, antes
// da plausibilidade de travamento (1 s).
#define STEERING_DIRECT_KP 0.05f
#define STEERING_DIRECT_KI 0.1f
#define STEERING_DIRECT_KD 0.002f
// A integral acumula com |erro| <= INTEGRAL_ZONE (erro final que o atrito
// segura) ou com a direcao parada longe da referencia (|velocidade medida| <
// INTEGRAL_STALL_RATE): carga que o P sozinho nao vence. Aproximando rapido,
// fica zerada (era a causa do sobressinal). Em 31/10 (1o Stanley com estes
// ganhos) so a zona: na curva fechada a direita a direcao parou em -15,5 graus
// com 4,5 graus de erro e PWM 120, sem integral, e deu flag 512.
#define STEERING_INTEGRAL_ZONE_DEG 4.0f
#define STEERING_INTEGRAL_STALL_RATE_DEG_S 5.0f
// Constante de tempo do filtro da velocidade medida usada pelo D (a velocidade
// do SteeringSensor tem ~0,2 s de filtro, lenta demais para isto).
#define STEERING_DERIVATIVE_FILTER_TAU_S 0.02f
// +-5 grau-segundo = +-0,5 de comando pela integral (era +-10 = 100%). No chao
// (30/10) o atrito estatico segurava a direcao enquanto a integral subia ate
// PWM 130-160; ao soltar, passava 3 a 7 graus da referencia.
#define STEERING_DIRECT_INTEGRAL_MIN -5.0f
#define STEERING_DIRECT_INTEGRAL_MAX 5.0f
// Taxa maxima da referencia usada pelo controlador: o analogico manda degraus
// (0 -> 15 graus de uma vez), que acumulavam integral e davam tranco.
#define STEERING_REFERENCE_MAX_RATE_DEG_S 40.0f

// ============================================================
// Controle de estercamento em cascata (inativo)
// ============================================================

// Estas constantes pertencem ao controlador posicao-velocidade antigo e nao
// sao consultadas pelo controlador direto atualmente compilado.
// #define STEERING_MIN_TARGET_VEL_DEG_S 1.0f
// #define STEERING_DEFAULT_MAX_VEL_DEG_S 20.0f
// #define STEERING_MAX_TARGET_VEL_DEG_S 45.0f
// #define STEERING_POS_KP 2.0f
// #define STEERING_POS_KI 0.0f
// #define STEERING_POS_KD 0.0f
// #define STEERING_VEL_KP 0.025f
// #define STEERING_VEL_KI 0.0f
// #define STEERING_VEL_KD 0.0f

// ============================================================
// Controle longitudinal (velocidade em malha fechada)
// ============================================================

// Referencia: DriveReference.velocity_m_s em m/s (sem re: negativo vira 0),
// limitada e passada por uma rampa de aceleracao/desaceleracao.
// 30/10: 2 m/s nos primeiros ensaios no chao; 2,5 m/s em seguida, para ficar
// acima da velocidade minima da controladora (~2 m/s com PWM 20). Subir para
// 3 m/s so depois de medir a frenagem no plano (o freio e fraco em descida).
#define DRIVE_MAX_TARGET_SPEED_M_S 2.5f
// Rampa de subida quando DriveReference.acceleration_m_s2 = 0.
#define DRIVE_DEFAULT_MAX_ACCEL_M_S2 0.4f
// Teto da rampa de subida pedida pelo ROS.
#define DRIVE_MAX_ACCEL_M_S2 0.8f
// Rampa de descida da referencia (o PI freia para acompanha-la).
#define DRIVE_MAX_DECEL_M_S2 1.0f

// PI de velocidade. Realimentacao: media das rodas dianteiras.
// Ganhos iniciais pelo cancelamento do polo da planta identificada no ensaio
// de 16/08/2026 (tau ~4 s, ganho ~0,15 (m/s)/PWM = ~39 m/s por unidade de
// comando, atraso ~0,2 s) para constante de tempo em malha fechada de ~1 s
// (5x o atraso):
//   Kp = tau / (K * Tmf) = 0,10 [comando/(m/s)]   Ki = Kp / tau = 0,025 [comando/m]
// Em simulacao com essa planta: degrau de 1 m/s sem sobressinal (90% em
// 3,6 s, limitado pela rampa); descida de 3% com ~11% acima da referencia.
// Ajustar em ensaio.
#define DRIVE_VEL_KP 0.10f
#define DRIVE_VEL_KI 0.025f

// Ganhos do modo bancada: a roda em vazio e outra planta. Bag de 29/10:
// tau ~1 s (roda livre), atraso ~0,25 s (filtro RC + controladora), ganho
// ~1 m/s por passo de PWM (~7x a 30x o do chao). SIMC com tau_c = atraso:
// Kp = tau / (K (tau_c + L)) ~ 0,008, Ki = Kp / tau ~ 0,008. Com 1 passo de
// PWM ~ 1 m/s, sobra uma oscilacao de ~0,5 m/s (resolucao de 8 bits).
#define DRIVE_BENCH_VEL_KP 0.008f
#define DRIVE_BENCH_VEL_KI 0.008f
// dt acima disto (task atrasada) reseta o controlador.
#define DRIVE_CONTROL_MAX_DT_S 0.1f

// Esforco do PI: > 0 tracao, < 0 freio. Tracao maxima = PWM 50 (0,197 * 255 =
// 50,2): no chao (30/10) a subida com curva saturou o antigo teto de PWM 38.
#define DRIVE_MAX_TRACTION_COMMAND 0.197f

// Zona morta da controladora da scooter (PWM -> filtro RC -> entrada de
// acelerador). Bancada de 29/10: parada parte com PWM 22-23, girando segura
// com PWM 20 e desarma com 19 (sem histerese abaixo disso). Esforco de tracao
// > 0 vira DEADZONE + esforco: o PI atua de PWM 20 ao teto sem gastar segundos
// integrando ate a zona morta; para partir, a integral sobe os 2-3 passos que
// faltam. 0,08 * 255 = 20,4 -> PWM 20 (a conversao trunca).
#define DRIVE_TRACTION_DEADZONE_COMMAND 0.08f

// Desacelerando (referencia pedida abaixo da rampa por mais que isto), o PI
// nao traciona: so roda livre ou freio. Sem isso, ao soltar o R2 a 3 m/s a
// rampa de descida ainda pedia tracao e a controladora religava com tranco.
#define DRIVE_DECEL_NO_TRACTION_MARGIN_M_S 0.1f

// Freio automatico (esforco negativo do PI):
// - entre -COAST_BAND e 0 o veiculo segue em roda livre (nem tracao nem freio);
// - abaixo disso, esforco de freio = (-esforco - COAST_BAND) * EFFORT_GAIN em
//   [0, 1], convertido em posicao ENGAGE + (1 - ENGAGE) * esforco de freio.
// ENGAGE compensa a folga do cabo; EFFORT_GAIN compensa a diferenca entre a
// eficacia do freio e a do motor. Valores provisorios: calibrar em ensaio.
#define DRIVE_BRAKE_COAST_BAND 0.02f
#define DRIVE_BRAKE_EFFORT_GAIN 3.0f
#define DRIVE_BRAKE_ENGAGE_POSITION 0.2f

// Parada: com a referencia (apos a rampa) abaixo de STOP_REFERENCE o PI e
// desligado: freio em STOP_BRAKE ate o veiculo parar
// (DRIVE_STOP_SPEED_THRESHOLD_MPS) e depois HOLD_BRAKE (o BrakeServo alivia o
// pulso apos 1 s parado). Sem isso o PI freia cada vez menos perto de zero e a
// parada se arrasta (simulacao: 11,6 s contra 2,2 s para parar de 1,5 m/s).
#define DRIVE_STOP_REFERENCE_M_S 0.05f
// 1,0 desde 30/10: com 0,6 o veiculo acelerou numa descida de ~9% (2,3 ->
// 3,0 m/s) com o R2 solto.
#define DRIVE_STOP_BRAKE_POSITION 1.0f
#define DRIVE_HOLD_BRAKE_POSITION 1.0f

// Freio manual pelo ROS (gatilho L2): DriveReference.brake_command acima
// disto zera a referencia e aplica o freio total imediatamente.
#define DRIVE_BRAKE_OVERRIDE_THRESHOLD 0.05f
