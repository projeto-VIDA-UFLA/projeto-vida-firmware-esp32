#include <Arduino.h>
#include <unity.h>

#include "pinout.h"
#include "config.h"
#include "SteeringSensor.h"

// Compila somente a implementacao do sensor neste teste, sem puxar src/main.cpp.
#include "../../src/SteeringSensor.cpp"

SteeringSensor steering_sensor(PIN_LVDT);

void test_print_steering_sensor_adc_and_degrees()
{
    const uint16_t sample_count = 200;
    const uint16_t sample_period_ms = 100;

    Serial.println();
    Serial.println("Teste do sensor de estercamento");
    Serial.printf("Pino ADC: GPIO %u\n", PIN_LVDT);
    Serial.println("tempo_ms,adc,voltage_v,angle_deg");

    for (uint16_t i = 0; i < sample_count; i++)
    {
        steering_sensor.update();

        const int raw_adc = steering_sensor.getRawAdc();
        const float voltage = steering_sensor.getVoltage();
        const float angle_deg = steering_sensor.getAngleDeg();

        Serial.printf(
            "%lu,%d,%.3f,%.2f\n",
            millis(),
            raw_adc,
            voltage,
            angle_deg);

        TEST_ASSERT_GREATER_OR_EQUAL_INT(0, raw_adc);
        TEST_ASSERT_LESS_OR_EQUAL_INT(static_cast<int>(ADC_MAX_VALUE), raw_adc);

        delay(sample_period_ms);
    }
}

void setup()
{
    Serial.begin(SERIAL_BAUDRATE);
    delay(2000);

    steering_sensor.begin();

    UNITY_BEGIN();
    RUN_TEST(test_print_steering_sensor_adc_and_degrees);
    UNITY_END();
}

void loop()
{
}
