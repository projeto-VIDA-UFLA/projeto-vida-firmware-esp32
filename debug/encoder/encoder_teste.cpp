#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "config.h"
#include "pinout.h"
#include "EncoderReader.h"

namespace
{
    EncoderReader encoderLeft(ENC_LEFT_A, ENC_LEFT_B, "Encoder Esquerdo");
    EncoderReader encoderRight(ENC_RIGHT_A, ENC_RIGHT_B, "Encoder Direito");
    EncoderReader encoderRear(ENC_REAR_A, ENC_REAR_B, "Encoder Traseiro");

    static void TaskProcessaOdometria(void *pv);

    constexpr uint32_t READ_PERIOD_MS = 100;

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

    static void TaskProcessaOdometria(void *pv)
    {
        (void)pv;

        TickType_t xLastWakeTime = xTaskGetTickCount();
        const TickType_t xPeriod = pdMS_TO_TICKS(1000 / 2000); // 50Hz (1000ms / 50 = 20ms)

        for (;;)
        {
            Serial.printf("Contagens dos Encoders:\n");
            Serial.printf(" - Esquerdo: %d\n", encoderLeft.getCount());
            Serial.printf(" - Direito: %d\n", encoderRight.getCount());
            Serial.printf(" - Traseiro: %d\n", encoderRear.getCount());
            Serial.print("=================================================");
            Serial.println();

            // Espera até o próximo ciclo de 20ms
            vTaskDelayUntil(&xLastWakeTime, xPeriod);
        }
    }

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

    xTaskCreatePinnedToCore(TaskProcessaOdometria, "Odometria", 4096, nullptr, 2, nullptr, 0);

    printHeader();
}

void loop()
{
    // Nada a fazer aqui, tudo é processado na task TaskProcessaOdometria
}
