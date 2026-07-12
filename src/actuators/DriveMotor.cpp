#include "actuators/DriveMotor.h"
#include "utils/config.h"
#include "utils/utils.h"
#include "esp_timer.h"

DriveMotor::DriveMotor(uint8_t pwm_pin, uint8_t direction_pin, uint8_t pwm_channel)
    : pwm_pin_(pwm_pin),
      direction_pin_(direction_pin),
      pwm_channel_(pwm_channel),
      last_command_(0.0f),
      last_pwm_(0),
      reverse_(false),
      pwm_timestamp_us_(0)
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

    const bool reverse = normalized_command < 0.0f;
    const int pwm = floatCommandToPwm(normalized_command);

    // LOW = frente; HIGH = ré
    digitalWrite(direction_pin_, reverse ? HIGH : LOW);
    ledcWrite(pwm_channel_, pwm);
    const int64_t timestamp_us = esp_timer_get_time();

    portENTER_CRITICAL(&snapshot_mux_);
    last_command_ = normalized_command;
    last_pwm_ = pwm;
    reverse_ = reverse;
    pwm_timestamp_us_ = timestamp_us;
    portEXIT_CRITICAL(&snapshot_mux_);
}

void DriveMotor::stop()
{
    digitalWrite(direction_pin_, LOW);
    ledcWrite(pwm_channel_, 0);
    const int64_t timestamp_us = esp_timer_get_time();

    portENTER_CRITICAL(&snapshot_mux_);
    last_command_ = 0.0f;
    last_pwm_ = 0;
    reverse_ = false;
    pwm_timestamp_us_ = timestamp_us;
    portEXIT_CRITICAL(&snapshot_mux_);
}

float DriveMotor::getLastCommand() const
{
    portENTER_CRITICAL(&snapshot_mux_);
    const float last_command = last_command_;
    portEXIT_CRITICAL(&snapshot_mux_);
    return last_command;
}

int DriveMotor::getLastPwm() const
{
    portENTER_CRITICAL(&snapshot_mux_);
    const int last_pwm = last_pwm_;
    portEXIT_CRITICAL(&snapshot_mux_);
    return last_pwm;
}

bool DriveMotor::isReverse() const
{
    portENTER_CRITICAL(&snapshot_mux_);
    const bool reverse = reverse_;
    portEXIT_CRITICAL(&snapshot_mux_);
    return reverse;
}

DriveMotor::Snapshot DriveMotor::getSnapshot() const
{
    Snapshot snapshot{};

    portENTER_CRITICAL(&snapshot_mux_);
    snapshot.pwm = last_pwm_;
    snapshot.reverse = reverse_;
    snapshot.timestamp_us = pwm_timestamp_us_;
    portEXIT_CRITICAL(&snapshot_mux_);

    return snapshot;
}
