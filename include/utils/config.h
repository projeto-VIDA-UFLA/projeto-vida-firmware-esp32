#pragma once

// ============================================================
// Comunicacao serial
// ============================================================

//#define SERIAL_BAUDRATE 460800
#define SERIAL_BAUDRATE 921600

// ============================================================
// PWM e canais LEDC
// ============================================================

#define PWM_FREQUENCY_HZ 9000
#define PWM_RESOLUTION_BITS 8

#define PWM_CH_DRIVE 0
#define PWM_CH_STEER_RIGHT 1
#define PWM_CH_STEER_LEFT 2

// Sem uso: a conversao de comando para duty nao consulta esta constante.
// #define PWM_MAX_DUTY 255

// ============================================================
// Periodos das tarefas
// ============================================================

#define TASK_SENSOR_PERIOD_MS 2        // 500 Hz
#define TASK_SAFETY_PERIOD_MS 5        // 200 Hz
#define TASK_CONTROL_PERIOD_MS 5      // 200 Hz
#define TASK_COMUNICATION_PERIOD_MS 20 // 50 Hz

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

#define STEERING_MIN_DEG -30.0f
#define STEERING_MAX_DEG 30.0f


#define STEERING_CALIBRATE_OFFSET 150

#define STEERING_CALIBRATE_DEG_1 34.35f
#define STEERING_CALIBRATE_DEG_2 -40.01f

#define STEERING_CALIBRATE_ADC_1 150
#define STEERING_CALIBRATE_ADC_2 4095 



#define STEERING_FILTER_ALPHA 0.25f
#define STEERING_VELOCITY_FILTER_ALPHA 0.25f

// Sem uso: permanecem documentadas para uma futura conversao ADC/tensao.
// #define ADC_MAX_VALUE 4095.0f
// #define ADC_REF_VOLTAGE 3.3f

// ============================================================
// Encoders e estimativa de movimento longitudinal
// ============================================================

#define ENC_REAR_TICKS_PER_REV 100.0f
#define ENC_FRONT_TICKS_PER_REV 100.0f
#define DRIVE_WHEEL_DIAMETER_M 0.30f
#define DRIVE_VELOCITY_FILTER_ALPHA 0.25f
#define DRIVE_ACCEL_FILTER_ALPHA 0.20f

// Velocidade maxima realimentada para considerar o veiculo parado.
#define DRIVE_STOP_SPEED_THRESHOLD_MPS 0.05f
// Tempo continuo abaixo do limiar exigido antes de trocar DIR.
#define DRIVE_STOP_CONFIRMATION_TIME_MS 200
// Idade maxima aceita para a velocidade usada na autorizacao de inversao.
#define DRIVE_VELOCITY_MEASUREMENT_TIMEOUT_MS 50
// Tempo sem torque depois de alterar DIR antes de reaplicar PWM.
#define DRIVE_DIRECTION_DEADTIME_MS 20

// ============================================================
// Controle direto de posicao do estercamento
// ============================================================

#define STEERING_POSITION_DEADBAND_DEG 0.7f
#define STEERING_MIN_ACTUATION 0.08f
#define MAX_ABS_STEER_COMMAND 1.0f
#define STEERING_CONTROL_MAX_DT_S 0.1f
// Intervalo sem PWM entre sentidos opostos do motor de estercamento.
#define STEERING_DIRECTION_DEADTIME_US 1000

// Kp: comando normalizado por grau.
// Ki: comando normalizado por grau-segundo.
// Kd: comando normalizado por grau por segundo.
#define STEERING_DIRECT_KP 0.03f
#define STEERING_DIRECT_KI 0.0f
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
// Controle longitudinal (ainda nao integrado)
// ============================================================

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
