#include <Arduino.h>

#include "config.h"
#include "pinout.h"
#include "EncoderReader.h"

namespace
{
    EncoderReader encoderLeft(ENC_LEFT_A, ENC_LEFT_B, "Encoder Esquerdo");
    EncoderReader encoderRight(ENC_RIGHT_A, ENC_RIGHT_B, "Encoder Direito");
    EncoderReader encoderRear(ENC_REAR_A, ENC_REAR_B, "Encoder Traseiro");

    constexpr uint32_t READ_PERIOD_MS = 100;

    void printHeader()
    {
        Serial.println();
        Serial.println("=====================================================");
        Serial.println("Debug dos encoders - Projeto VIDA");
        Serial.println("=====================================================");
        Serial.printf("Pino ADC: GPIO %u\n", PIN_LVDT);                            // TODO: corrigir
        Serial.printf("Resolucao ADC: %u bits (0 - 4095)\n", ADC_RESOLUTION_BITS); // TODO: corrigir
        Serial.println();
    }

}

void IRAM_ATTR isrRearEncoder()
{
    encoderRear.handleInterrupt();
}

void IRAM_ATTR isrRightEncoder()
{
    encoderRight.handleInterrupt();
}

void IRAM_ATTR isrLeftEncoder()
{
    encoderLeft.handleInterrupt();
}

void setup()
{
    Serial.begin(SERIAL_BAUDRATE);
    delay(1000);

    encoderRear.begin();
    encoderRight.begin();
    encoderLeft.begin();

    attachInterrupt(digitalPinToInterrupt(ENC_REAR_A), isrRearEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_REAR_B), isrRearEncoder, CHANGE);

    attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_A), isrRightEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_B), isrRightEncoder, CHANGE);

    attachInterrupt(digitalPinToInterrupt(ENC_LEFT_A), isrLeftEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_LEFT_B), isrLeftEncoder, CHANGE);

    printHeader();
}

void loop()
{
    
}
