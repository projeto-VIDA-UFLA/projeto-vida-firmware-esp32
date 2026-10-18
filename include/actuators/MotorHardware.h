#pragma once

#include <Arduino.h>

/**
 * Acesso ao hardware usado pelos atuadores (pinos, LEDC e relogio).
 *
 * Os atuadores recebem esta interface no construtor; o padrao
 * (defaultMotorHardware) usa o Arduino/ESP32. Os testes unitarios injetam
 * uma implementacao falsa para verificar as escritas sem hardware.
 */
class MotorHardware
{
public:
    virtual ~MotorHardware() = default;

    virtual void configureOutput(uint8_t pin) = 0;
    // PWM de motor: PWM_FREQUENCY_HZ com PWM_RESOLUTION_BITS.
    virtual void configurePwm(uint8_t pin, uint8_t channel) = 0;
    // PWM de servo: BRAKE_SERVO_FREQUENCY_HZ com BRAKE_SERVO_RESOLUTION_BITS.
    virtual void configureServo(uint8_t pin, uint8_t channel) = 0;
    virtual void writeDirection(uint8_t pin, bool high) = 0;
    virtual void writePwm(uint8_t channel, int duty) = 0;
    virtual int64_t nowUs() const = 0;
};

// Implementacao com ledcSetup/ledcWrite/digitalWrite e esp_timer.
MotorHardware &defaultMotorHardware();
