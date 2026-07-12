#include "actuators/SteeringMotor.h"
#include "utils/config.h"
#include "utils/utils.h"
#include "esp_timer.h"

SteeringMotor::SteeringMotor(uint8_t right_pwm_pin, uint8_t left_pwm_pin, uint8_t right_channel, uint8_t left_channel)
    : right_pwm_pin_(right_pwm_pin),
      left_pwm_pin_(left_pwm_pin),
      right_channel_(right_channel),
      left_channel_(left_channel),
      last_command_(0.0f),
      right_pwm_(0),
      left_pwm_(0),
      pwm_timestamp_us_(0)
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

    int right_pwm = 0;
    int left_pwm = 0;

    if (normalized_command > 0.0f)
    {
        right_pwm = floatCommandToPwm(normalized_command);
    }
    else if (normalized_command < 0.0f)
    {
        left_pwm = floatCommandToPwm(normalized_command);
    }

    // Garantia: nunca acionar os dois lados ao mesmo tempo.
    ledcWrite(right_channel_, right_pwm);
    ledcWrite(left_channel_, left_pwm);
    const int64_t timestamp_us = esp_timer_get_time();

    portENTER_CRITICAL(&snapshot_mux_);
    last_command_ = normalized_command;
    right_pwm_ = right_pwm;
    left_pwm_ = left_pwm;
    pwm_timestamp_us_ = timestamp_us;
    portEXIT_CRITICAL(&snapshot_mux_);
}

void SteeringMotor::stop()
{
    ledcWrite(right_channel_, 0);
    ledcWrite(left_channel_, 0);
    const int64_t timestamp_us = esp_timer_get_time();

    portENTER_CRITICAL(&snapshot_mux_);
    last_command_ = 0.0f;
    right_pwm_ = 0;
    left_pwm_ = 0;
    pwm_timestamp_us_ = timestamp_us;
    portEXIT_CRITICAL(&snapshot_mux_);
}

float SteeringMotor::getLastCommand() const
{
    portENTER_CRITICAL(&snapshot_mux_);
    const float last_command = last_command_;
    portEXIT_CRITICAL(&snapshot_mux_);
    return last_command;
}

int SteeringMotor::getRightPwm() const
{
    portENTER_CRITICAL(&snapshot_mux_);
    const int right_pwm = right_pwm_;
    portEXIT_CRITICAL(&snapshot_mux_);
    return right_pwm;
}

int SteeringMotor::getLeftPwm() const
{
    portENTER_CRITICAL(&snapshot_mux_);
    const int left_pwm = left_pwm_;
    portEXIT_CRITICAL(&snapshot_mux_);
    return left_pwm;
}

SteeringMotor::Snapshot SteeringMotor::getSnapshot() const
{
    Snapshot snapshot{};

    portENTER_CRITICAL(&snapshot_mux_);
    snapshot.right_pwm = right_pwm_;
    snapshot.left_pwm = left_pwm_;
    snapshot.timestamp_us = pwm_timestamp_us_;
    portEXIT_CRITICAL(&snapshot_mux_);

    return snapshot;
}
