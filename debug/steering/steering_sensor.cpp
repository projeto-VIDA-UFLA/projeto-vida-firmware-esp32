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
        Serial.println("=====================================================");
        Serial.println("Debug do sensor de estercamento - Projeto VIDA");
        Serial.println("=====================================================");
        Serial.printf("Pino ADC: GPIO %u\n", PIN_LVDT);
        Serial.printf("Resolucao ADC: %u bits (0 - 4095)\n", ADC_RESOLUTION_BITS);
        Serial.println();
    }

    void printSample()
    {
        steering_sensor.update();

        Serial.printf("Leitura ADC: %d    ?    Angulo(graus): %.2f\n",
                      steering_sensor.getRawAdc(),
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
