//======================================================================
// BIBLIOTECAS
//======================================================================

// Inclui bibliotecas ESP32
#include <Arduino.h>
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cmath>

// Inclui classes utilitárias
#include "utils/pinout.h"
#include "utils/config.h"
#include "utils/utils.h"

// Inclui classes de sensores
#include "sensors/EncoderReader.h"
#include "sensors/SteeringSensor.h"

// Inclui classes de atuadores
#include "actuators/SteeringMotor.h"
#include "actuators/DriveMotor.h"
#include "actuators/BrakeServo.h"
#include "actuators/ActuatorManager.h"

// Inclui classes de segurança
#include "safety/SafetyManager.h"

// Inclui classes de controle
#include "control/SteeringController.h"

// Inclui classes de comunicação
#include "comunication/CommandMailbox.h"
#include "comunication/VidaRosNode.h"

//======================================================================
// INSTANCIA DE OBJETOS
//======================================================================

// Instanciando objetos de encoders
EncoderReader right_encoder(ENC_RIGHT_A, ENC_RIGHT_B, ENC_FRONT_TICKS_PER_REV, "front_right");
EncoderReader left_encoder(ENC_LEFT_A, ENC_LEFT_B, ENC_FRONT_TICKS_PER_REV, "front_left");
EncoderReader rear_encoder(ENC_REAR_A, ENC_REAR_B, ENC_REAR_TICKS_PER_REV, "rear");

// Instanciando objeto do sensor de esterçamento
SteeringSensor steering_sensor(PIN_LVDT);

// Instanciando objetos que representam os atuadores
SteeringMotor steering_motor(PIN_STEER_RIGHT_PWM, PIN_STEER_LEFT_PWM, PWM_CH_STEER_RIGHT, PWM_CH_STEER_LEFT);
DriveMotor drive_motor(PIN_DRIVE_PWM, PIN_DRIVE_REVERSE, PWM_CH_DRIVE);
BrakeServo brake_servo(PIN_BRAKE_SERVO, PWM_CH_BRAKE);
ActuatorManager actuator_manager(drive_motor, steering_motor, brake_servo);

// Instanciando Controladores
SteeringController steering_controller;

CommandMailbox command_mailbox;

// Instanciando Objeto de gerenciamento de segurança
SafetyManager safety_manager(PIN_ENABLE_SAFETY, command_mailbox,
                             actuator_manager, steering_controller);

// Instanciando nó do ROS
VidaRosNode ros_node(rear_encoder, right_encoder, left_encoder, steering_sensor, safety_manager, actuator_manager, command_mailbox, steering_controller);

//======================================================================
// DEFINIÇÃO DE ISRs DOS ENCODERS
//======================================================================

void IRAM_ATTR isrRearEncoder()
{
  rear_encoder.handleInterrupt();
}

void IRAM_ATTR isrRightEncoder()
{
  right_encoder.handleInterrupt();
}

void IRAM_ATTR isrLeftEncoder()
{
  left_encoder.handleInterrupt();
}

//======================================================================
// TASKs
//======================================================================

void taskSensors(void *parameter)
{
  (void)parameter;
  TickType_t last_wake_time = xTaskGetTickCount();

  while (true)
  {
    steering_sensor.update();
    rear_encoder.update();
    right_encoder.update();
    left_encoder.update();

    const SteeringSensor::Snapshot steering = steering_sensor.getSnapshot();
    const EncoderReader::Snapshot rear = rear_encoder.getMeasurementSnapshot();
    const EncoderReader::Snapshot right = right_encoder.getMeasurementSnapshot();
    const EncoderReader::Snapshot left = left_encoder.getMeasurementSnapshot();

    // Validando leitura dos sensores
    const bool cycle_valid =
        steering.valid &&
        std::isfinite(rear.filtered_velocity_m_s) &&
        std::isfinite(right.filtered_velocity_m_s) &&
        std::isfinite(left.filtered_velocity_m_s) &&
        rear.timestamp_us > 0 &&
        right.timestamp_us > 0 &&
        left.timestamp_us > 0;

    // Heartbeat registrado somente depois do ciclo completo.
    safety_manager.reportSensorTaskCycle(cycle_valid, steering);

    vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(TASK_SENSOR_PERIOD_MS));
  }
}

void taskSafety(void *parameter)
{
  (void)parameter;

  TickType_t last_wake_time = xTaskGetTickCount();

  while (true)
  {
    safety_manager.update();
    actuator_manager.updateBrake();
    vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(TASK_SAFETY_PERIOD_MS));
  }
}

void taskControl(void *parameter)
{
  (void)parameter;

  TickType_t last_wake_time = xTaskGetTickCount();
  int64_t previous_update_us = 0;

  while (true)
  {
    const int64_t now_us = esp_timer_get_time();
    const CommandMailbox::SteeringReferenceSnapshot steering_reference =
        command_mailbox.getSteeringReferenceSnapshot();
    const CommandMailbox::DriveReferenceSnapshot drive_reference =
        command_mailbox.getDriveReferenceSnapshot();

    const SteeringSensor::Snapshot steering = steering_sensor.getSnapshot();
    constexpr int64_t steering_snapshot_timeout_us =
        static_cast<int64_t>(STEERING_SENSOR_SNAPSHOT_TIMEOUT_MS) * 1000LL;
    const bool steering_is_fresh =
        steering.valid &&
        steering.timestamp_us > 0 &&
        now_us >= steering.timestamp_us &&
        (now_us - steering.timestamp_us) <= steering_snapshot_timeout_us;
    const float dt_s = previous_update_us > 0
                           ? static_cast<float>(now_us - previous_update_us) / 1000000.0f
                           : static_cast<float>(TASK_CONTROL_PERIOD_MS) / 1000.0f;

    const SteeringController::Input input{
        steering_reference.angle_deg,
        steering_is_fresh ? steering.filtered_angle_deg : NAN,
        dt_s};

    (void)steering_controller.update(input);

    const SteeringController::Snapshot output =
        steering_controller.getLastOutput();

    // Malha aberta longitudinal: a referencia ROS e um PWM assinado em
    // [-255, 255]. O DriveMotor recebe comando normalizado em [-1, 1].
    // Exemplo: referencia 255 -> 255 * 0.8 = 204 de PWM.
    const float drive_command =
        clampFloat(drive_reference.velocity_m_s *
                       DRIVE_OPEN_LOOP_PROPORTIONAL_GAIN /
                       DRIVE_OPEN_LOOP_REFERENCE_MAX_PWM,
                   -1.0f,
                   1.0f);
    constexpr int64_t drive_reference_timeout_us =
        static_cast<int64_t>(DRIVE_REFERENCE_WATCHDOG_TIMEOUT_MS) * 1000LL;
    const bool drive_reference_is_fresh =
        drive_reference.received &&
        std::isfinite(drive_reference.velocity_m_s) &&
        drive_reference.received_at_us > 0 &&
        now_us >= drive_reference.received_at_us &&
        (now_us - drive_reference.received_at_us) <=
            drive_reference_timeout_us;

    // O encoder traseiro e a fonte longitudinal: ele mede a roda do eixo
    // motriz diretamente, sem combinar velocidades das rodas esterçantes.
    const EncoderReader::Snapshot rear =
        rear_encoder.getMeasurementSnapshot();
    // O controlador opera na convencao de STEERING_INVERT_SIGN; o motor
    // aciona o PWM da direita com comando positivo. O sinal converte de volta.
    const ActuatorManager::TimedCommand command{
        drive_command,
        STEERING_ANGLE_SIGN * output.output.normalized_command,
        drive_reference.brake_command,
        now_us,
        output.output.valid && drive_reference_is_fresh};
    const DriveMotor::VelocityMeasurement drive_velocity{
        rear.filtered_velocity_m_s,
        rear.timestamp_us,
        std::isfinite(rear.filtered_velocity_m_s)};
    (void)actuator_manager.applyCommands(command, drive_velocity);

    previous_update_us = now_us;
    vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(TASK_CONTROL_PERIOD_MS));
  }
}

//======================================================================
// VOID SETUP
//======================================================================

void setup()
{
  // Inicia comunicação serial
  Serial.begin(SERIAL_BAUDRATE);
  delay(1000);

  // Inicializa sensores
  rear_encoder.begin();
  right_encoder.begin();
  left_encoder.begin();
  steering_sensor.begin();

  // Inicializa atuadores. O freio vem primeiro: parte acionado.
  brake_servo.begin();
  drive_motor.begin();
  steering_motor.begin();
  actuator_manager.begin();

  // Inicializa gerenciador de segurança
  safety_manager.begin();

  // Inicializa interrupções
  attachInterrupt(digitalPinToInterrupt(ENC_REAR_A), isrRearEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_REAR_B), isrRearEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_A), isrRightEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_B), isrRightEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_LEFT_A), isrLeftEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_LEFT_B), isrLeftEncoder, CHANGE);

  // Inicializa TASKs
  xTaskCreatePinnedToCore(taskSensors, "TaskSensors", 4096, nullptr, PRIORITY_SENSORS, nullptr, 1);
  xTaskCreatePinnedToCore(taskControl, "TaskControl", 4096, nullptr, PRIORITY_CONTROL, nullptr, 1);
  xTaskCreatePinnedToCore(taskSafety, "TaskSafety", 4096, nullptr, PRIORITY_SAFETY, nullptr, 1);

  ros_node.begin(Serial);
}

void loop()
{
}
