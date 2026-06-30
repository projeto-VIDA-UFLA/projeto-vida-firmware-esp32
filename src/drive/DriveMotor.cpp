#include "DriveMotor.h"
#include "config.h"
#include "utils.h"

DriveMotor::DriveMotor(uint8_t pwm_pin, uint8_t direction_pin, uint8_t pwm_channel)
    : pwm_pin_(pwm_pin),
      direction_pin_(direction_pin),
      pwm_channel_(pwm_channel),
      last_command_(0.0f),
      last_pwm_(0),
      reverse_(false)
{
}

void DriveMotor::begin()
{
    pinMode(direction_pin_, OUTPUT);
    digitalWrite(direction_pin_, LOW);

    ledcSetup(pwm_channel_, PWM_FREQUENCY_HZ, PWM_RESOLUTION_BITS);
    ledcAttachPin(pwm_pin_, pwm_channel_);
    stop();
}

void DriveMotor::setCommand(float normalized_command)
{
    normalized_command = clampFloat(normalized_command, -1.0f, 1.0f);

    reverse_ = normalized_command < 0.0f;
    last_pwm_ = floatCommandToPwm(normalized_command);
    last_command_ = normalized_command;

    // LOW = frente; HIGH = ré
    digitalWrite(direction_pin_, reverse_ ? HIGH : LOW);
    ledcWrite(pwm_channel_, last_pwm_);
}

void DriveMotor::stop()
{
    last_command_ = 0.0f;
    last_pwm_ = 0;
    reverse_ = false;

    digitalWrite(direction_pin_, LOW);
    ledcWrite(pwm_channel_, 0);
}

float DriveMotor::getLastCommand() const
{
    return last_command_;
}

int DriveMotor::getLastPwm() const
{
    return last_pwm_;
}

bool DriveMotor::isReverse() const
{
    return reverse_;
}
