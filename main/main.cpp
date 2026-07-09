#include <Arduino.h>

#include "pinout.h"
#include "config.h"

#include "EncoderReader.h"
#include "SteeringSensor.h"

EncoderReader rear_encoder(ENC_REAR_A, ENC_REAR_B, "rear");
EncoderReader right_encoder(ENC_RIGHT_A, ENC_RIGHT_B, "front_right");
EncoderReader left_encoder(ENC_LEFT_A, ENC_LEFT_B, "front_left");

SteeringSensor steering_sensor(PIN_LVDT);

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

void taskSensors(void *parameter)
{
  (void)parameter;

  TickType_t last_wake_time = xTaskGetTickCount();

  while (true)
  {
    steering_sensor.update();

    Serial.print(millis());
    Serial.print(" :   ");

    Serial.print(steering_sensor.getRawAdc());
    Serial.print("  |   ");

    Serial.print(rear_encoder.getCount());
    Serial.print("  |   ");

    Serial.print(right_encoder.getCount());
    Serial.print("  |   ");

    Serial.print(left_encoder.getCount());
    Serial.println();

    vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(TASK_SENSOR_PERIOD_MS));
  }
}

void setup()
{
  Serial.begin(SERIAL_BAUDRATE);
  delay(1000);

  rear_encoder.begin();
  right_encoder.begin();
  left_encoder.begin();
  steering_sensor.begin();

  attachInterrupt(digitalPinToInterrupt(ENC_REAR_A), isrRearEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_REAR_B), isrRearEncoder, CHANGE);

  attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_A), isrRightEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_B), isrRightEncoder, CHANGE);

  attachInterrupt(digitalPinToInterrupt(ENC_LEFT_A), isrLeftEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_LEFT_B), isrLeftEncoder, CHANGE);

  xTaskCreatePinnedToCore(taskSensors, "TaskSensors", 4096, nullptr, PRIORITY_SENSORS, nullptr, 1);

  Serial.println("Sistema inicializado.");
}

void loop()
{
}