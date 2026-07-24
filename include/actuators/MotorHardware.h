#pragma once

#include <Arduino.h>

class MotorHardware
{
public:
    virtual ~MotorHardware() = default;

    virtual void configureOutput(uint8_t pin) = 0;
    virtual void configurePwm(uint8_t pin, uint8_t channel) = 0;
    virtual void writeDirection(uint8_t pin, bool high) = 0;
    virtual void writePwm(uint8_t channel, int duty) = 0;
    virtual int64_t nowUs() const = 0;
};

MotorHardware &defaultMotorHardware();
