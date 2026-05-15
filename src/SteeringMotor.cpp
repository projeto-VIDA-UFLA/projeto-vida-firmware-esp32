#include "SteeringMotor.h"
#include "config.h"
#include "utils.h"

SteeringMotor::SteeringMotor(uint8_t right_pwm_pin, uint8_t left_pwm_pin, uint8_t right_channel, uint8_t left_channel)
    : right_pwm_pin_(right_pwm_pin),
      left_pwm_pin_(left_pwm_pin),
      right_channel_(right_channel),
      left_channel_(left_channel),
      last_command_(0.0f),
      right_pwm_(0),
      left_pwm_(0)
{
}

void SteeringMotor::begin()
{
    ledcSetup(right_channel_, PWM_FREQUENCY_HZ, PWM_RESOLUTION_BITS);
    ledcSetup(left_channel_, PWM_FREQUENCY_HZ, PWM_RESOLUTION_BITS);

    ledcAttachPin(right_pwm_pin_, right_channel_);
    ledcAttachPin(left_pwm_pin_, left_channel_);

    stop();
}

void SteeringMotor::setCommand(float normalized_command)
{
    normalized_command = clampFloat(normalized_command, -1.0f, 1.0f);
    last_command_ = normalized_command;

    right_pwm_ = 0;
    left_pwm_ = 0;

    if (normalized_command > 0.0f)
    {
        right_pwm_ = floatCommandToPwm(normalized_command);
    }
    else if (normalized_command < 0.0f)
    {
        left_pwm_ = floatCommandToPwm(normalized_command);
    }

    // Garantia: nunca acionar os dois lados ao mesmo tempo.
    ledcWrite(right_channel_, right_pwm_);
    ledcWrite(left_channel_, left_pwm_);
}

void SteeringMotor::stop()
{
    last_command_ = 0.0f;
    right_pwm_ = 0;
    left_pwm_ = 0;

    ledcWrite(right_channel_, 0);
    ledcWrite(left_channel_, 0);
}

float SteeringMotor::getLastCommand() const
{
    return last_command_;
}

int SteeringMotor::getRightPwm() const
{
    return right_pwm_;
}

int SteeringMotor::getLeftPwm() const
{
    return left_pwm_;
}
