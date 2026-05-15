#pragma once
#include <Arduino.h>

class DriveMotor
{
public:
    DriveMotor(uint8_t pwm_pin, uint8_t direction_pin, uint8_t pwm_channel);

    void begin();
    void setCommand(float normalized_command);
    void stop();

    float getLastCommand() const;
    int getLastPwm() const;
    bool isReverse() const;

private:
    uint8_t pwm_pin_;
    uint8_t direction_pin_;
    uint8_t pwm_channel_;

    float last_command_;
    int last_pwm_;
    bool reverse_;
};
