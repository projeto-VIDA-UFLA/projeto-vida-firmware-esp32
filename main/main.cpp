//======================================================================
// BIBLIOTECAS
//======================================================================

// Inclui bibliotecas ESP32
#include <Arduino.h>
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Inclui classes utilitárias
#include "utils/pinout.h"
#include "utils/config.h"

// Inclui classes de sensores
#include "sensors/EncoderReader.h"
#include "sensors/SteeringSensor.h"

// Inclui classes de atuadores
#include "actuators/SteeringMotor.h"
#include "actuators/DriveMotor.h"
#include "actuators/ActuatorManager.h"

// Inclui classes de controle
#include "control/SteeringController.h"

// Inclui classes de segurança
#include "safety/SafetyManager.h"

// Inclui classes de comunicação
#include "comunication/CommandMailbox.h"
#include "comunication/VidaRosNode.h"

//======================================================================
// INSTANCIA DE OBJETOS
//======================================================================

// Instanciando objetos de encoders
EncoderReader rear_encoder(ENC_REAR_A, ENC_REAR_B, ENC_REAR_TICKS_PER_REV, "rear");
EncoderReader right_encoder(ENC_RIGHT_A, ENC_RIGHT_B, ENC_FRONT_TICKS_PER_REV, "front_right");
EncoderReader left_encoder(ENC_LEFT_A, ENC_LEFT_B, ENC_FRONT_TICKS_PER_REV, "front_left");

// Instanciando objeto do sensor de esterçamento
SteeringSensor steering_sensor(PIN_LVDT);

// Instanciando atuadores
SteeringMotor steering_motor(PIN_STEER_RIGHT_PWM, PIN_STEER_LEFT_PWM, PWM_CH_STEER_RIGHT, PWM_CH_STEER_LEFT);
DriveMotor drive_motor(PIN_DRIVE_PWM, PIN_DRIVE_REVERSE, PWM_CH_DRIVE);
ActuatorManager actuator_manager(drive_motor, steering_motor);

SteeringController steering_controller;

CommandMailbox command_mailbox;
SafetyManager safety_manager(PIN_ENABLE_SAFETY, command_mailbox, actuator_manager);

// Instanciando nó do ROS
VidaRosNode ros_node(actuator_manager, safety_manager, steering_sensor,
                     rear_encoder, right_encoder, left_encoder,
                     command_mailbox, steering_controller);

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
    const CommandMailbox::SteeringReferenceSnapshot reference =
        command_mailbox.getSteeringReferenceSnapshot();

    const SteeringSensor::Snapshot steering = steering_sensor.getSnapshot();
    const float dt_s = previous_update_us > 0
                           ? static_cast<float>(now_us - previous_update_us) / 1000000.0f
                           : static_cast<float>(TASK_CONTROL_PERIOD_MS) / 1000.0f;

    const SteeringController::Input input{
        reference.angle_deg,
        steering.filtered_angle_deg,
        dt_s};

    (void)steering_controller.update(input);

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

  // Inicializa atuadores
  drive_motor.begin();
  steering_motor.begin();
  actuator_manager.begin();

  const bool safety_initialized = safety_manager.begin();
  if (!safety_initialized)
  {
    actuator_manager.setMotionPermit(false);
  }

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

  if (safety_initialized && xTaskCreatePinnedToCore(taskSafety, "TaskSafety", 4096, nullptr, PRIORITY_SAFETY, nullptr, 1) != pdPASS)
  {
    actuator_manager.setMotionPermit(false);
  }

  ros_node.begin(Serial);
}

void loop()
{
}
