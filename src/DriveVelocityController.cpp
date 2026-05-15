#include "DriveVelocityController.h"
#include "config.h"
#include "utils.h"

DriveVelocityController::DriveVelocityController(DriveMotor& motor,
                                                 EncoderReader& rear_encoder,
                                                 EncoderReader& left_encoder,
                                                 EncoderReader& right_encoder)
    : motor_(motor),
      rear_encoder_(rear_encoder),
      left_encoder_(left_encoder),
      right_encoder_(right_encoder),
      velocity_pid_(DRIVE_VEL_KP,
                    DRIVE_VEL_KI,
                    DRIVE_VEL_KD,
                    -DRIVE_MAX_SAFE_ACCEL_M_S2,
                    DRIVE_MAX_SAFE_ACCEL_M_S2),
      accel_pid_(DRIVE_ACCEL_KP,
                 DRIVE_ACCEL_KI,
                 DRIVE_ACCEL_KD,
                 -MAX_ABS_DRIVE_COMMAND,
                 MAX_ABS_DRIVE_COMMAND),
      target_velocity_m_s_(0.0f),
      ramped_velocity_m_s_(0.0f),
      measured_velocity_m_s_(0.0f),
      measured_accel_m_s2_(0.0f),
      accel_setpoint_m_s2_(0.0f),
      max_accel_m_s2_(DRIVE_DEFAULT_MAX_ACCEL_M_S2),
      motor_command_(0.0f),
      previous_velocity_m_s_(0.0f),
      last_update_ms_(0),
      has_previous_velocity_(false)
{
}

void DriveVelocityController::begin()
{
    stop();
    rear_encoder_.getAndResetDelta();
    left_encoder_.getAndResetDelta();
    right_encoder_.getAndResetDelta();
    last_update_ms_ = millis();
}

void DriveVelocityController::update(bool enabled)
{
    uint32_t now_ms = millis();
    float dt_s = (now_ms - last_update_ms_) / 1000.0f;
    last_update_ms_ = now_ms;

    long rear_delta = rear_encoder_.getAndResetDelta();
    long left_delta = left_encoder_.getAndResetDelta();
    long right_delta = right_encoder_.getAndResetDelta();

    if (dt_s > 0.0f)
    {
        float rear_velocity = encoderDeltaToDistanceM(rear_delta) / dt_s;
        float left_velocity = encoderDeltaToDistanceM(left_delta) / dt_s;
        float right_velocity = encoderDeltaToDistanceM(right_delta) / dt_s;
        float raw_velocity = (rear_velocity + left_velocity + right_velocity) / 3.0f;

        measured_velocity_m_s_ =
            DRIVE_VELOCITY_FILTER_ALPHA * raw_velocity +
            (1.0f - DRIVE_VELOCITY_FILTER_ALPHA) * measured_velocity_m_s_;

        if (has_previous_velocity_)
        {
            measured_accel_m_s2_ = (measured_velocity_m_s_ - previous_velocity_m_s_) / dt_s;
        }

        previous_velocity_m_s_ = measured_velocity_m_s_;
        has_previous_velocity_ = true;
    }

    if (!enabled || dt_s <= 0.0f)
    {
        stop();
        return;
    }

    float safe_accel = clampFloat(
        fabs(max_accel_m_s2_),
        DRIVE_MIN_ACCEL_M_S2,
        DRIVE_MAX_SAFE_ACCEL_M_S2
    );

    float max_delta_velocity = safe_accel * dt_s;
    float velocity_error = target_velocity_m_s_ - ramped_velocity_m_s_;
    ramped_velocity_m_s_ += clampFloat(
        velocity_error,
        -max_delta_velocity,
        max_delta_velocity
    );

    velocity_pid_.setOutputLimits(-safe_accel, safe_accel);
    accel_setpoint_m_s2_ = velocity_pid_.update(ramped_velocity_m_s_, measured_velocity_m_s_, dt_s);
    accel_setpoint_m_s2_ = clampFloat(accel_setpoint_m_s2_, -safe_accel, safe_accel);

    motor_command_ = accel_pid_.update(accel_setpoint_m_s2_, measured_accel_m_s2_, dt_s);
    motor_command_ = applyActuationDeadband(motor_command_);

    motor_.setCommand(motor_command_);
}

void DriveVelocityController::stop()
{
    velocity_pid_.reset();
    accel_pid_.reset();
    target_velocity_m_s_ = 0.0f;
    ramped_velocity_m_s_ = 0.0f;
    accel_setpoint_m_s2_ = 0.0f;
    motor_command_ = 0.0f;
    motor_.stop();
}

void DriveVelocityController::setTarget(float velocity_m_s, float max_accel_m_s2)
{
    target_velocity_m_s_ = clampFloat(
        velocity_m_s,
        -DRIVE_MAX_TARGET_SPEED_M_S,
        DRIVE_MAX_TARGET_SPEED_M_S
    );
    max_accel_m_s2_ = clampFloat(
        fabs(max_accel_m_s2),
        DRIVE_MIN_ACCEL_M_S2,
        DRIVE_MAX_SAFE_ACCEL_M_S2
    );
}

float DriveVelocityController::getTargetVelocityMps() const
{
    return target_velocity_m_s_;
}

float DriveVelocityController::getRampedVelocityMps() const
{
    return ramped_velocity_m_s_;
}

float DriveVelocityController::getMeasuredVelocityMps() const
{
    return measured_velocity_m_s_;
}

float DriveVelocityController::getMeasuredAccelMps2() const
{
    return measured_accel_m_s2_;
}

float DriveVelocityController::getAccelSetpointMps2() const
{
    return accel_setpoint_m_s2_;
}

float DriveVelocityController::getMaxAccelMps2() const
{
    return max_accel_m_s2_;
}

float DriveVelocityController::getMotorCommand() const
{
    return motor_command_;
}

float DriveVelocityController::encoderDeltaToDistanceM(long delta_ticks) const
{
    float wheel_circumference_m = PI * DRIVE_WHEEL_DIAMETER_M;
    return (static_cast<float>(delta_ticks) / DRIVE_ENCODER_TICKS_PER_REV) * wheel_circumference_m;
}

float DriveVelocityController::applyActuationDeadband(float command) const
{
    if (command == 0.0f)
    {
        return 0.0f;
    }

    if (fabs(command) < DRIVE_MIN_ACTUATION)
    {
        return command > 0.0f ? DRIVE_MIN_ACTUATION : -DRIVE_MIN_ACTUATION;
    }

    return command;
}
