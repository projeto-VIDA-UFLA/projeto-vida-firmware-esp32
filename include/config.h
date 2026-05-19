#pragma once

// ============================================================
// Configurações gerais
// ============================================================

#define SERIAL_BAUDRATE 115200

// micro-ROS
#define ENABLE_MICRO_ROS       1
#define ENABLE_MICRO_ROS_COMMAND_NODE 0
#define MICRO_ROS_NODE_NAME    "vida_esp32"
#define MICRO_ROS_CMD_TOPIC    "/cmd_vel"
#define MICRO_ROS_SPIN_PERIOD_MS 10

// Quando o transporte micro-ROS usa Serial, logs e comandos ASCII na mesma UART
// podem corromper o protocolo XRCE-DDS.
#define ENABLE_SERIAL_COMMANDS 0
#define ENABLE_DEBUG_STATUS    0

// PWM
#define PWM_FREQUENCY_HZ       9000
#define PWM_RESOLUTION_BITS    8
#define PWM_MAX_DUTY           255

// Canais LEDC do ESP32
#define PWM_CH_DRIVE           0
#define PWM_CH_STEER_RIGHT     1
#define PWM_CH_STEER_LEFT      2

// Controle / tarefas
#define TASK_SAFETY_PERIOD_MS  10    // 100 Hz
#define TASK_CONTROL_PERIOD_MS 10    // 100 Hz
#define TASK_SENSOR_PERIOD_MS  20    // 50 Hz
#define TASK_DEBUG_PERIOD_MS   500   // 2 Hz

// Watchdog de comando
#define COMMAND_WATCHDOG_TIMEOUT_MS 300

// Handshake com o computador de bordo
#define REQUIRE_ODROID_HANDSHAKE 1

// Sensor de esterçamento
#define ADC_RESOLUTION_BITS    12
#define ADC_MAX_VALUE          4095.0f
#define ADC_REF_VOLTAGE        3.3f

#define STEERING_MIN_DEG      -30.0f
#define STEERING_MAX_DEG       30.0f

// Controle de esterçamento em cascata
#define STEERING_POSITION_DEADBAND_DEG   0.7f
#define STEERING_MIN_ACTUATION           0.08f
#define STEERING_MIN_TARGET_VEL_DEG_S    1.0f
#define STEERING_DEFAULT_MAX_VEL_DEG_S   20.0f
#define STEERING_MAX_TARGET_VEL_DEG_S    45.0f
#define STEERING_VELOCITY_FILTER_ALPHA   0.25f

#define STEERING_POS_KP                  2.0f
#define STEERING_POS_KI                  0.0f
#define STEERING_POS_KD                  0.0f

#define STEERING_VEL_KP                  0.025f
#define STEERING_VEL_KI                  0.0f
#define STEERING_VEL_KD                  0.0f

// Controle longitudinal por velocidade
#define DRIVE_ENCODER_TICKS_PER_REV      100.0f
#define DRIVE_WHEEL_DIAMETER_M           0.30f
#define DRIVE_MAX_TARGET_SPEED_M_S       2.0f
#define DRIVE_MIN_ACCEL_M_S2             0.05f
#define DRIVE_DEFAULT_MAX_ACCEL_M_S2     0.40f
#define DRIVE_MAX_SAFE_ACCEL_M_S2        0.80f
#define DRIVE_MIN_ACTUATION              0.05f
#define DRIVE_VELOCITY_FILTER_ALPHA      0.25f

#define DRIVE_VEL_KP                     0.45f
#define DRIVE_VEL_KI                     0.0f
#define DRIVE_VEL_KD                     0.0f

#define DRIVE_ACCEL_KP                   0.30f
#define DRIVE_ACCEL_KI                   0.0f
#define DRIVE_ACCEL_KD                   0.0f

// Saturações dos comandos
#define MAX_ABS_DRIVE_COMMAND  1.0f
#define MAX_ABS_STEER_COMMAND  1.0f

// Prioridade das tasks
#define PRIORITY_SAFETY        4
#define PRIORITY_CONTROL       3
#define PRIORITY_SENSORS       2
#define PRIORITY_DEBUG         1
#define PRIORITY_MICRO_ROS     2
