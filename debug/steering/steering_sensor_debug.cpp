#include <Arduino.h>

#include "config.h"
#include "pinout.h"
#include "SteeringSensor.h"

namespace
{
constexpr uint32_t READ_PERIOD_MS = 100;

SteeringSensor steering_sensor(PIN_LVDT);

void printHeader()
{
    Serial.println();
    Serial.println("============================================");
    Serial.println("Debug do sensor de estercamento - Projeto VIDA");
    Serial.println("============================================");
    Serial.printf("Pino ADC: GPIO %u\n", PIN_LVDT);
    Serial.printf("Resolucao ADC: %u bits\n", ADC_RESOLUTION_BITS);
    Serial.printf("Faixa estimada: %.1f V -> %.1f V | %.1f deg -> %.1f deg\n",
                  0.0f,
                  ADC_REF_VOLTAGE,
                  STEERING_MIN_DEG,
                  STEERING_MAX_DEG);
    Serial.println();
    Serial.println("tempo_ms,adc_bruto,tensao_v,angulo_deg");
}

void printSample()
{
    steering_sensor.update();

    Serial.printf("%lu,%d,%.3f,%.2f\n",
                  millis(),
                  steering_sensor.getRawAdc(),
                  steering_sensor.getVoltage(),
                  steering_sensor.getAngleDeg());
}
}

void setup()
{
    Serial.begin(SERIAL_BAUDRATE);
    delay(1000);

    steering_sensor.begin();
    printHeader();
}

void loop()
{
    printSample();
    delay(READ_PERIOD_MS);
}
