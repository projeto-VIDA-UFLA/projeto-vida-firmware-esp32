#pragma once

// Parametros do firmware. Pinos ficam em pinout.h.
// Documentacao de cada parametro por modulo: doc/utils/config.md.

// ============================================================
// Comunicacao serial
// ============================================================

// Transporte do micro-ROS; o agente no Odroid deve usar o mesmo baud rate.
#define SERIAL_BAUDRATE 921600

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

#define LOGICAL_ENABLE_WATCHDOG_TIMEOUT_MS 300
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
// polinomio antes da troca de sinal. Teste de 12/10/2026: curvatura nula com
// -1,5 a -1,7 graus em REP-103, isto e, +1,6 graus na convencao da calibracao.
#define STEERING_ZERO_OFFSET_DEG 1.6f

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
#define DRIVE_VELOCITY_FILTER_ALPHA 0.05f
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
// O pulso diminui para frear: o servo puxa o cabo no sentido de 2500 -> 1600.
#define BRAKE_RELEASED_US 2500
#define BRAKE_APPLIED_US 1600

// Com o veiculo parado o servo puxa no maximo ate este pulso, para nao forcar
// o servo mantendo o freio. Deve ficar entre BRAKE_RELEASED_US e
// BRAKE_APPLIED_US. Volta ao freio total assim que houver movimento.
#define BRAKE_HOLD_US 1700
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
// Piso de atuacao: fora da zona morta, |comando| menor que isto vira
// +-STEERING_MIN_ACTUATION (vence o atrito estatico do mecanismo).
#define STEERING_MIN_ACTUATION 0.15f
#define MAX_ABS_STEER_COMMAND 1.0f
// dt acima disto (task atrasada) reseta o controlador.
#define STEERING_CONTROL_MAX_DT_S 0.1f
// Intervalo sem PWM entre sentidos opostos do motor de estercamento.
#define STEERING_DIRECTION_DEADTIME_US 1000

// Kp: comando normalizado por grau.
// Ki: comando normalizado por grau-segundo.
// Kd: comando normalizado por grau por segundo.
// INTEGRAL_MIN/MAX limitam a integral do erro (grau-segundo).
#define STEERING_DIRECT_KP 0.007f
#define STEERING_DIRECT_KI 0.1f
#define STEERING_DIRECT_KD 0.0f
#define STEERING_DIRECT_INTEGRAL_MIN -10.0f
#define STEERING_DIRECT_INTEGRAL_MAX 10.0f

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
// Controle longitudinal (malha aberta)
// ============================================================

// A referencia recebida em velocity_m_s e tratada temporariamente como um
// comando PWM assinado na faixa [-255, 255]. Comando de tracao =
// referencia * GAIN / MAX_PWM (com ganho 0,15 o PWM maximo e 38).
#define DRIVE_OPEN_LOOP_REFERENCE_MAX_PWM 255.0f
#define DRIVE_OPEN_LOOP_PROPORTIONAL_GAIN 0.15f

// Sem uso: constantes do controlador em cascata antigo (velocidade ->
// aceleracao), mantidas como referencia para a malha fechada.
// #define DRIVE_MAX_TARGET_SPEED_M_S 2.0f
// #define DRIVE_MIN_ACCEL_M_S2 0.05f
// #define DRIVE_DEFAULT_MAX_ACCEL_M_S2 0.40f
// #define DRIVE_MAX_SAFE_ACCEL_M_S2 0.80f
// #define DRIVE_MIN_ACTUATION 0.05f
// #define DRIVE_VEL_KP 0.45f
// #define DRIVE_VEL_KI 0.0f
// #define DRIVE_VEL_KD 0.0f
// #define DRIVE_ACCEL_KP 0.30f
// #define DRIVE_ACCEL_KI 0.0f
// #define DRIVE_ACCEL_KD 0.0f
// #define MAX_ABS_DRIVE_COMMAND 1.0f
