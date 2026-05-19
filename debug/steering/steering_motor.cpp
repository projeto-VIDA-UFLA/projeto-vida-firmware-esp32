#include <Arduino.h>

#include "config.h"
#include "pinout.h"
#include "SteeringMotor.h"

namespace
{
    constexpr uint32_t READ_PERIOD_MS = 100;

    SteeringMotor steering_motor(PIN_STEER_RIGHT_PWM, PIN_STEER_LEFT_PWM, PWM_CH_STEER_RIGHT, PWM_CH_STEER_LEFT);

    void printHeader()
    {
        Serial.println();
        Serial.println("=====================================================");
        Serial.println("Debug do motor de estercamento - Projeto VIDA");
        Serial.println("=====================================================");
        Serial.printf("Pino ADC: GPIO %u\n", PIN_LVDT);
        Serial.printf("Resolucao ADC: %u bits (0 - 4095)\n", ADC_RESOLUTION_BITS);
        Serial.println();
    }
   
}

void setup()
{
    Serial.begin(SERIAL_BAUDRATE);
    delay(1000);

    steering_motor.begin();
    printHeader();
}

void loop()
{
    steering_motor.setCommand(0.4f);
    delay(400);
    steering_motor.stop();
    delay(20000);
}
