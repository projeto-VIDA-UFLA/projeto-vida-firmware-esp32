#include <Arduino.h>

#include "pinout.h"
#include "config.h"

#include "EncoderReader.h"
#include "SteeringSensor.h"
#include "DriveMotor.h"
#include "SteeringMotor.h"
#include "Watchdog.h"
#include "SafetyManager.h"
#include "VehicleController.h"


// ============================================================
// Objetos globais
// ============================================================

EncoderReader rear_encoder(ENC_REAR_A, ENC_REAR_B, "rear");
EncoderReader right_encoder(ENC_RIGHT_A, ENC_RIGHT_B, "front_right");
EncoderReader left_encoder(ENC_LEFT_A, ENC_LEFT_B, "front_left");

SteeringSensor steering_sensor(PIN_LVDT);

DriveMotor drive_motor(PIN_DRIVE_PWM, PIN_DRIVE_REVERSE, PWM_CH_DRIVE);
SteeringMotor steering_motor(PIN_STEER_RIGHT_PWM, PIN_STEER_LEFT_PWM, PWM_CH_STEER_RIGHT, PWM_CH_STEER_LEFT);

Watchdog command_watchdog(COMMAND_WATCHDOG_TIMEOUT_MS);
SafetyManager safety_manager(PIN_ENABLE_SAFETY);

VehicleController vehicle_controller(
    drive_motor,
    steering_motor,
    steering_sensor,
    rear_encoder,
    left_encoder,
    right_encoder,
    safety_manager);

SerialCommand serial_command(vehicle_controller, command_watchdog, rear_encoder, left_encoder, right_encoder);

// ============================================================
// ISRs dos encoders
// ============================================================

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

// ============================================================
// Tasks FreeRTOS
// ============================================================

void taskSafety(void *parameter)
{
  (void)parameter;

  TickType_t last_wake_time = xTaskGetTickCount();

  while (true)
  {
    bool software_enable = serial_command.getSoftwareEnable();
    bool odroid_ready = serial_command.isOdroidReady();
    bool watchdog_ok = command_watchdog.isOk();
    bool critical_fault = false;

    safety_manager.update(odroid_ready, software_enable, watchdog_ok, critical_fault);

    if (!safety_manager.canActuate())
    {
      drive_motor.stop();
      steering_motor.stop();
    }

    vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(TASK_SAFETY_PERIOD_MS));
  }
}

void taskSensors(void *parameter)
{
  (void)parameter;

  TickType_t last_wake_time = xTaskGetTickCount();

  while (true)
  {
    steering_sensor.update();
    vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(TASK_SENSOR_PERIOD_MS));
  }
}

void taskControl(void *parameter)
{
  (void)parameter;

  TickType_t last_wake_time = xTaskGetTickCount();

  while (true)
  {
    vehicle_controller.update();
    vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(TASK_CONTROL_PERIOD_MS));
  }
}

void taskDebug(void *parameter)
{
  (void)parameter;

  TickType_t last_wake_time = xTaskGetTickCount();

  while (true)
  {
    long rear_count = rear_encoder.getCount();
    long left_count = left_encoder.getCount();
    long right_count = right_encoder.getCount();

    Serial.printf(
        "[STATUS] state=%s odroid_ready=%d physical_enable=%d watchdog=%d dt=%lu ms | enc rear=%ld left=%ld right=%ld | speed target=%.2f ramp=%.2f meas=%.2f accel_sp=%.2f accel=%.2f cmd=%.2f | steer angle=%.2f target=%.2f vel_sp=%.2f vel=%.2f cmd=%.2f\n",
        safety_manager.getStateName(),
        serial_command.isOdroidReady(),
        safety_manager.isPhysicalEnableOk(),
        command_watchdog.isOk(),
        command_watchdog.timeSinceLastFeedMs(),
        rear_count,
        left_count,
        right_count,
        vehicle_controller.getTargetVelocityMps(),
        vehicle_controller.getRampedVelocityMps(),
        vehicle_controller.getMeasuredVelocityMps(),
        vehicle_controller.getAccelSetpointMps2(),
        vehicle_controller.getMeasuredAccelMps2(),
        vehicle_controller.getDriveCommand(),
        steering_sensor.getAngleDeg(),
        vehicle_controller.getSteeringTargetDeg(),
        vehicle_controller.getSteeringVelocitySetpointDegS(),
        vehicle_controller.getSteeringMeasuredVelocityDegS(),
        vehicle_controller.getSteeringControlCommand());

    vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(TASK_DEBUG_PERIOD_MS));
  }
}

// ============================================================
// Setup / loop
// ============================================================

void setup()
{
  Serial.begin(SERIAL_BAUDRATE);
  delay(1000);

  Serial.println();
  Serial.println("============================================");
  Serial.println("ESP32 Vehicle Firmware - Fase 1");
  Serial.println("Projeto VIDA");
  Serial.println("Sem micro-ROS - teste local");
  Serial.println("============================================");

  rear_encoder.begin();
  right_encoder.begin();
  left_encoder.begin();

  attachInterrupt(digitalPinToInterrupt(ENC_REAR_A), isrRearEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_REAR_B), isrRearEncoder, CHANGE);

  attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_A), isrRightEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_B), isrRightEncoder, CHANGE);

  attachInterrupt(digitalPinToInterrupt(ENC_LEFT_A), isrLeftEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_LEFT_B), isrLeftEncoder, CHANGE);

  steering_sensor.begin();
  drive_motor.begin();
  steering_motor.begin();

  command_watchdog.begin();
  safety_manager.begin();
  vehicle_controller.begin();

  serial_command.begin();

  xTaskCreatePinnedToCore(taskSafety, "TaskSafety", 4096, nullptr, PRIORITY_SAFETY, nullptr, 1);
  xTaskCreatePinnedToCore(taskSensors, "TaskSensors", 4096, nullptr, PRIORITY_SENSORS, nullptr, 1);
  xTaskCreatePinnedToCore(taskControl, "TaskControl", 4096, nullptr, PRIORITY_CONTROL, nullptr, 1);
  xTaskCreatePinnedToCore(taskDebug, "TaskDebug", 4096, nullptr, PRIORITY_DEBUG, nullptr, 1);

  Serial.println("Sistema inicializado.");
  Serial.println("Aguardando handshake: odroid_ready");
  Serial.println("Use comandos: odroid_ready, enable 1, speed 1.0 0.4, steer_target 10 20, reset_enc");
}

void loop()
{
  steering_motor.setCommand(0.1);
  delay(200);
 
}
