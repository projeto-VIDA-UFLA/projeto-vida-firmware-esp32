#include "actuators/SteeringMotor.h"
#include "utils/config.h"
#include "utils/utils.h"
#include <cmath>

SteeringMotor::SteeringMotor(uint8_t right_pwm_pin, uint8_t left_pwm_pin,
                             uint8_t right_channel, uint8_t left_channel,
                             MotorHardware *hardware)
    : right_pwm_pin_(right_pwm_pin),
      left_pwm_pin_(left_pwm_pin),
      right_channel_(right_channel),
      left_channel_(left_channel),
      last_command_(0.0f),
      right_pwm_(0),
      left_pwm_(0),
      pwm_timestamp_us_(0),
      state_(State::STOPPED),
      pending_command_(0.0f),
      deadtime_started_us_(0),
      hardware_(hardware != nullptr ? *hardware : defaultMotorHardware())
{
}

void SteeringMotor::begin()
{
    hardware_.configurePwm(right_pwm_pin_, right_channel_);
    hardware_.configurePwm(left_pwm_pin_, left_channel_);

    stop();
}

void SteeringMotor::setCommand(float normalized_command)
{
    if (!std::isfinite(normalized_command))
    {
        stop();
        return;
    }

    normalized_command = clampFloat(normalized_command, -1.0f, 1.0f);
    const int64_t now_us = hardware_.nowUs();

    if (normalized_command == 0.0f)
    {
        stop();
        return;
    }

    const bool wants_right = normalized_command > 0.0f;
    const bool opposite_direction =
        (state_ == State::DRIVING_RIGHT && !wants_right) ||
        (state_ == State::DRIVING_LEFT && wants_right);

    if (opposite_direction)
    {
        // A inversao sempre passa por um estado observavel com ambos em zero.
        writeBothOff();
        pending_command_ = normalized_command;
        deadtime_started_us_ = now_us;
        storeSnapshot(0.0f, 0, 0, State::REVERSAL_DEADTIME, now_us);
        return;
    }

    if (state_ == State::REVERSAL_DEADTIME)
    {
        pending_command_ = normalized_command;
        if ((now_us - deadtime_started_us_) < STEERING_DIRECTION_DEADTIME_US)
        {
            writeBothOff();
            storeSnapshot(0.0f, 0, 0, State::REVERSAL_DEADTIME, now_us);
            return;
        }
        normalized_command = pending_command_;
    }

    const int pwm = floatCommandToPwm(normalized_command);
    const int right_pwm = normalized_command > 0.0f ? pwm : 0;
    const int left_pwm = normalized_command < 0.0f ? pwm : 0;

    // O canal inativo e zerado antes de qualquer escrita no canal ativo.
    hardware_.writePwm(normalized_command > 0.0f ? left_channel_ : right_channel_, 0);
    hardware_.writePwm(normalized_command > 0.0f ? right_channel_ : left_channel_, pwm);
    storeSnapshot(normalized_command, right_pwm, left_pwm,
                  normalized_command > 0.0f ? State::DRIVING_RIGHT
                                            : State::DRIVING_LEFT,
                  now_us);
}

void SteeringMotor::stop()
{
    writeBothOff();
    pending_command_ = 0.0f;
    deadtime_started_us_ = 0;
    storeSnapshot(0.0f, 0, 0, State::STOPPED, hardware_.nowUs());
}

void SteeringMotor::forceOutputOff()
{
    writeBothOff();
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

SteeringMotor::State SteeringMotor::getState() const
{
    portENTER_CRITICAL(&snapshot_mux_);
    const State state = state_;
    portEXIT_CRITICAL(&snapshot_mux_);
    return state;
}

SteeringMotor::Snapshot SteeringMotor::getSnapshot() const
{
    Snapshot snapshot{};

    portENTER_CRITICAL(&snapshot_mux_);
    snapshot.right_pwm = right_pwm_;
    snapshot.left_pwm = left_pwm_;
    snapshot.timestamp_us = pwm_timestamp_us_;
    snapshot.state = state_;
    portEXIT_CRITICAL(&snapshot_mux_);

    return snapshot;
}

void SteeringMotor::writeBothOff()
{
    hardware_.writePwm(right_channel_, 0);
    hardware_.writePwm(left_channel_, 0);
}

void SteeringMotor::storeSnapshot(float command, int right_pwm, int left_pwm,
                                  State state, int64_t timestamp_us)
{
    portENTER_CRITICAL(&snapshot_mux_);
    last_command_ = command;
    right_pwm_ = right_pwm;
    left_pwm_ = left_pwm;
    state_ = state;
    pwm_timestamp_us_ = timestamp_us;
    portEXIT_CRITICAL(&snapshot_mux_);
}
