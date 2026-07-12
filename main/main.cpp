//======================================================================
// BIBLIOTECAS
//======================================================================

// Inclui bibliotecas ESP32
#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Inclui classes utilitárias
#include "utils/pinout.h"
#include "utils/config.h"

// Iclui classes de sensores
#include "sensors/EncoderReader.h"
#include "sensors/SteeringSensor.h"

// Iclui classes de sensores
#include "actuators/SteeringMotor.h"
#include "actuators/DriveMotor.h"

// Iclui classes de comunicação
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

// Instanciando nó do ROS
VidaRosNode ros_node(steering_motor, drive_motor, steering_sensor,
                     rear_encoder, right_encoder, left_encoder);

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
  steering_motor.begin();
  drive_motor.begin();

  // Inicializa interrupções
  attachInterrupt(digitalPinToInterrupt(ENC_REAR_A), isrRearEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_REAR_B), isrRearEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_A), isrRightEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_B), isrRightEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_LEFT_A), isrLeftEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_LEFT_B), isrLeftEncoder, CHANGE);

  // Inicializa TASKs
  xTaskCreatePinnedToCore(taskSensors, "TaskSensors", 4096, nullptr, PRIORITY_SENSORS, nullptr, 1);
  ros_node.begin(Serial);
}

void loop()
{
}
