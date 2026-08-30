#include "actuators/DriveMotor.h"

#include <cmath>
#include "utils/config.h"
#include "utils/utils.h"

DriveMotor::DriveMotor(uint8_t pwm_pin, uint8_t direction_pin,
                       uint8_t pwm_channel, MotorHardware *hardware)
    : pwm_pin_(pwm_pin),
      direction_pin_(direction_pin),
      pwm_channel_(pwm_channel),
      last_command_(0.0f),
      last_pwm_(0),
      reverse_(false),
      pwm_timestamp_us_(0),
      state_(State::STOPPED),
      pending_reverse_(false),
      stopped_since_us_(0),
      direction_changed_at_us_(0),
      hardware_(hardware != nullptr ? *hardware : defaultMotorHardware())
{
}

void DriveMotor::begin()
{
    hardware_.configureOutput(direction_pin_);
    // A direcao inicial e configurada antes de qualquer PWM.
    hardware_.writeDirection(direction_pin_, false);
    hardware_.configurePwm(pwm_pin_, pwm_channel_);
    stop();
}

void DriveMotor::setCommand(
    float normalized_command,
    const VelocityMeasurement &velocity_measurement)
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

    const bool requested_reverse = normalized_command < 0.0f;

    if (state_ == State::DIRECTION_DEADTIME)
    {
        writePwmZero();

        if (requested_reverse != reverse_)
        {
            state_ = State::WAITING_FOR_STOP;
            pending_reverse_ = requested_reverse;
            stopped_since_us_ = 0;
            storeSnapshot(0.0f, 0, state_, now_us);
            return;
        }

        constexpr int64_t direction_deadtime_us =
            static_cast<int64_t>(DRIVE_DIRECTION_DEADTIME_MS) * 1000LL;
        if ((now_us - direction_changed_at_us_) < direction_deadtime_us)
        {
            storeSnapshot(0.0f, 0, state_, now_us);
            return;
        }

        const int pwm = floatCommandToPwm(normalized_command);
        hardware_.writePwm(pwm_channel_, pwm);
        storeSnapshot(normalized_command, pwm, State::DRIVING, now_us);
        return;
    }

    const bool direction_change_requested = requested_reverse != reverse_;
    if (direction_change_requested || state_ == State::WAITING_FOR_STOP)
    {
        // Torque e removido imediatamente; DIR conserva o sentido atual.
        writePwmZero();
        pending_reverse_ = requested_reverse;
        state_ = State::WAITING_FOR_STOP;

        if (!isVelocityFreshAndStopped(velocity_measurement, now_us))
        {
            stopped_since_us_ = 0;
            storeSnapshot(0.0f, 0, state_, now_us);
            return;
        }

        if (stopped_since_us_ == 0)
        {
            stopped_since_us_ = now_us;
        }

        constexpr int64_t stop_confirmation_us =
            static_cast<int64_t>(DRIVE_STOP_CONFIRMATION_TIME_MS) * 1000LL;
        if ((now_us - stopped_since_us_) < stop_confirmation_us)
        {
            storeSnapshot(0.0f, 0, state_, now_us);
            return;
        }

        // PWM permanece zero no instante em que o pino DIR e alterado.
        writePwmZero();
        hardware_.writeDirection(direction_pin_, pending_reverse_);
        reverse_ = pending_reverse_;
        direction_changed_at_us_ = now_us;
        stopped_since_us_ = 0;
        state_ = State::DIRECTION_DEADTIME;
        storeSnapshot(0.0f, 0, state_, now_us);
        return;
    }

    const int pwm = floatCommandToPwm(normalized_command);
    hardware_.writePwm(pwm_channel_, pwm);
    storeSnapshot(normalized_command, pwm, State::DRIVING, now_us);
}

void DriveMotor::stop()
{
    // Parar nunca altera DIR: o veiculo pode ainda estar em movimento.
    writePwmZero();
    pending_reverse_ = reverse_;
    stopped_since_us_ = 0;
    direction_changed_at_us_ = 0;
    storeSnapshot(0.0f, 0, State::STOPPED, hardware_.nowUs());
}

float DriveMotor::getLastCommand() const
{
    portENTER_CRITICAL(&snapshot_mux_);
    const float value = last_command_;
    portEXIT_CRITICAL(&snapshot_mux_);
    return value;
}

int DriveMotor::getLastPwm() const
{
    portENTER_CRITICAL(&snapshot_mux_);
    const int value = last_pwm_;
    portEXIT_CRITICAL(&snapshot_mux_);
    return value;
}

bool DriveMotor::isReverse() const
{
    portENTER_CRITICAL(&snapshot_mux_);
    const bool value = reverse_;
    portEXIT_CRITICAL(&snapshot_mux_);
    return value;
}

DriveMotor::State DriveMotor::getState() const
{
    portENTER_CRITICAL(&snapshot_mux_);
    const State value = state_;
    portEXIT_CRITICAL(&snapshot_mux_);
    return value;
}

DriveMotor::Snapshot DriveMotor::getSnapshot() const
{
    portENTER_CRITICAL(&snapshot_mux_);
    const Snapshot snapshot{
        last_pwm_,
        reverse_,
        pwm_timestamp_us_,
        state_,
        state_ == State::WAITING_FOR_STOP};
    portEXIT_CRITICAL(&snapshot_mux_);
    return snapshot;
}

bool DriveMotor::isVelocityFreshAndStopped(
    const VelocityMeasurement &measurement, int64_t now_us) const
{
    constexpr int64_t velocity_timeout_us =
        static_cast<int64_t>(DRIVE_VELOCITY_MEASUREMENT_TIMEOUT_MS) * 1000LL;
    return measurement.valid &&
           std::isfinite(measurement.velocity_m_s) &&
           measurement.timestamp_us > 0 &&
           now_us >= measurement.timestamp_us &&
           (now_us - measurement.timestamp_us) <= velocity_timeout_us &&
           std::fabs(measurement.velocity_m_s) <= DRIVE_STOP_SPEED_THRESHOLD_MPS;
}

void DriveMotor::writePwmZero()
{
    hardware_.writePwm(pwm_channel_, 0);
}

void DriveMotor::storeSnapshot(float command, int pwm, State state,
                               int64_t timestamp_us)
{
    portENTER_CRITICAL(&snapshot_mux_);
    last_command_ = command;
    last_pwm_ = pwm;
    state_ = state;
    pwm_timestamp_us_ = timestamp_us;
    portEXIT_CRITICAL(&snapshot_mux_);
}
