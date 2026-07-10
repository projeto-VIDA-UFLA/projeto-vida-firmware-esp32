#pragma once
#include <Arduino.h>

class SteeringMotor
{
public:
    SteeringMotor(uint8_t right_pwm_pin, uint8_t left_pwm_pin, uint8_t right_channel, uint8_t left_channel);

    void begin();

    // Comando normalizado:
    // > 0: aciona direita
    // < 0: aciona esquerda
    // = 0: parado
    void setCommand(float normalized_command);

    void stop();

    float getLastCommand() const;
    int getRightPwm() const;
    int getLeftPwm() const;

private:
    uint8_t right_pwm_pin_;
    uint8_t left_pwm_pin_;
    uint8_t right_channel_;
    uint8_t left_channel_;

    float last_command_;
    int right_pwm_;
    int left_pwm_;
};
