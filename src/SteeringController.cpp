#include "SteeringController.h"
#include "config.h"
#include "utils.h"

SteeringController::SteeringController(SteeringMotor& motor, SteeringSensor& sensor)
    : motor_(motor),
      sensor_(sensor),
      position_pid_(STEERING_POS_KP,
                    STEERING_POS_KI,
                    STEERING_POS_KD,
                    -STEERING_MAX_TARGET_VEL_DEG_S,
                    STEERING_MAX_TARGET_VEL_DEG_S),
      velocity_pid_(STEERING_VEL_KP,
                    STEERING_VEL_KI,
                    STEERING_VEL_KD,
                    -MAX_ABS_STEER_COMMAND,
                    MAX_ABS_STEER_COMMAND),
      target_angle_deg_(0.0f),
      max_velocity_deg_s_(STEERING_DEFAULT_MAX_VEL_DEG_S),
      velocity_setpoint_deg_s_(0.0f),
      measured_velocity_deg_s_(0.0f),
      motor_command_(0.0f),
      previous_angle_deg_(0.0f),
      last_update_ms_(0),
      has_previous_angle_(false)
{
}

void SteeringController::begin()
{
    stop();
    previous_angle_deg_ = sensor_.getAngleDeg();
    last_update_ms_ = millis();
    has_previous_angle_ = true;
}

void SteeringController::update(bool enabled)
{
    uint32_t now_ms = millis();
    float dt_s = (now_ms - last_update_ms_) / 1000.0f;
    last_update_ms_ = now_ms;

    float current_angle_deg = sensor_.getAngleDeg();

    if (!has_previous_angle_ || dt_s <= 0.0f)
    {
        previous_angle_deg_ = current_angle_deg;
        has_previous_angle_ = true;
        motor_.stop();
        return;
    }

    float raw_velocity_deg_s = (current_angle_deg - previous_angle_deg_) / dt_s;
    measured_velocity_deg_s_ =
        STEERING_VELOCITY_FILTER_ALPHA * raw_velocity_deg_s +
        (1.0f - STEERING_VELOCITY_FILTER_ALPHA) * measured_velocity_deg_s_;
    previous_angle_deg_ = current_angle_deg;

    if (!enabled)
    {
        stop();
        return;
    }

    float position_error = target_angle_deg_ - current_angle_deg;
    if (fabs(position_error) <= STEERING_POSITION_DEADBAND_DEG)
    {
        velocity_setpoint_deg_s_ = 0.0f;
        motor_command_ = 0.0f;
        velocity_pid_.reset();
        motor_.stop();
        return;
    }

    velocity_setpoint_deg_s_ = position_pid_.update(target_angle_deg_, current_angle_deg, dt_s);
    velocity_setpoint_deg_s_ = clampFloat(
        velocity_setpoint_deg_s_,
        -fabs(max_velocity_deg_s_),
        fabs(max_velocity_deg_s_)
    );

    motor_command_ = velocity_pid_.update(velocity_setpoint_deg_s_, measured_velocity_deg_s_, dt_s);
    motor_command_ = applyActuationDeadband(motor_command_);

    motor_.setCommand(motor_command_);
}

void SteeringController::stop()
{
    position_pid_.reset();
    velocity_pid_.reset();
    velocity_setpoint_deg_s_ = 0.0f;
    motor_command_ = 0.0f;
    motor_.stop();
}

void SteeringController::setTarget(float angle_deg, float max_velocity_deg_s)
{
    target_angle_deg_ = clampFloat(angle_deg, STEERING_MIN_DEG, STEERING_MAX_DEG);
    max_velocity_deg_s_ = clampFloat(
        fabs(max_velocity_deg_s),
        STEERING_MIN_TARGET_VEL_DEG_S,
        STEERING_MAX_TARGET_VEL_DEG_S
    );
}

float SteeringController::getTargetAngleDeg() const
{
    return target_angle_deg_;
}

float SteeringController::getMaxVelocityDegS() const
{
    return max_velocity_deg_s_;
}

float SteeringController::getVelocitySetpointDegS() const
{
    return velocity_setpoint_deg_s_;
}

float SteeringController::getMeasuredVelocityDegS() const
{
    return measured_velocity_deg_s_;
}

float SteeringController::getMotorCommand() const
{
    return motor_command_;
}

float SteeringController::applyActuationDeadband(float command) const
{
    if (command == 0.0f)
    {
        return 0.0f;
    }

    if (fabs(command) < STEERING_MIN_ACTUATION)
    {
        return command > 0.0f ? STEERING_MIN_ACTUATION : -STEERING_MIN_ACTUATION;
    }

    return command;
}
