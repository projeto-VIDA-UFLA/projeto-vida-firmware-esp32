#include "VehicleController.h"
#include "config.h"
#include "utils.h"

VehicleController::VehicleController(DriveMotor& drive_motor,
                                     SteeringMotor& steering_motor,
                                     SteeringSensor& steering_sensor,
                                     EncoderReader& rear_encoder,
                                     EncoderReader& left_encoder,
                                     EncoderReader& right_encoder,
                                     SafetyManager& safety_manager)
    : drive_motor_(drive_motor),
      steering_motor_(steering_motor),
      safety_manager_(safety_manager),
      steering_controller_(steering_motor, steering_sensor),
      drive_controller_(drive_motor, rear_encoder, left_encoder, right_encoder),
      control_mode_(VehicleControlMode::ClosedLoop),
      open_loop_drive_command_(0.0f),
      open_loop_steering_command_(0.0f)
{
}

void VehicleController::begin()
{
    steering_controller_.begin();
    drive_controller_.begin();
}

void VehicleController::update()
{
    if (!safety_manager_.canActuate())
    {
        steering_controller_.update(false);
        drive_controller_.update(false);
        open_loop_drive_command_ = 0.0f;
        open_loop_steering_command_ = 0.0f;
        return;
    }

    if (control_mode_ == VehicleControlMode::OpenLoop)
    {
        drive_motor_.setCommand(open_loop_drive_command_);
        steering_motor_.setCommand(open_loop_steering_command_);
        return;
    }

    steering_controller_.update(true);
    drive_controller_.update(true);
}

void VehicleController::setSpeedTarget(float velocity_m_s, float max_accel_m_s2)
{
    control_mode_ = VehicleControlMode::ClosedLoop;
    drive_controller_.setTarget(velocity_m_s, max_accel_m_s2);
}

void VehicleController::setSteeringTarget(float target_deg, float max_velocity_deg_s)
{
    control_mode_ = VehicleControlMode::ClosedLoop;
    steering_controller_.setTarget(target_deg, max_velocity_deg_s);
}

void VehicleController::setOpenLoopCommand(float drive_command, float steering_command)
{
    control_mode_ = VehicleControlMode::OpenLoop;
    open_loop_drive_command_ = clampFloat(drive_command, -MAX_ABS_DRIVE_COMMAND, MAX_ABS_DRIVE_COMMAND);
    open_loop_steering_command_ = clampFloat(steering_command, -MAX_ABS_STEER_COMMAND, MAX_ABS_STEER_COMMAND);
}

void VehicleController::setControlMode(VehicleControlMode mode)
{
    control_mode_ = mode;
}

VehicleControlMode VehicleController::getControlMode() const
{
    return control_mode_;
}

float VehicleController::getTargetVelocityMps() const
{
    return drive_controller_.getTargetVelocityMps();
}

float VehicleController::getRampedVelocityMps() const
{
    return drive_controller_.getRampedVelocityMps();
}

float VehicleController::getMeasuredVelocityMps() const
{
    return drive_controller_.getMeasuredVelocityMps();
}

float VehicleController::getMeasuredAccelMps2() const
{
    return drive_controller_.getMeasuredAccelMps2();
}

float VehicleController::getAccelSetpointMps2() const
{
    return drive_controller_.getAccelSetpointMps2();
}

float VehicleController::getMaxAccelMps2() const
{
    return drive_controller_.getMaxAccelMps2();
}

float VehicleController::getDriveCommand() const
{
    return drive_controller_.getMotorCommand();
}

float VehicleController::getSteeringTargetDeg() const
{
    return steering_controller_.getTargetAngleDeg();
}

float VehicleController::getSteeringVelocityMaxDegS() const
{
    return steering_controller_.getMaxVelocityDegS();
}

float VehicleController::getSteeringVelocitySetpointDegS() const
{
    return steering_controller_.getVelocitySetpointDegS();
}

float VehicleController::getSteeringMeasuredVelocityDegS() const
{
    return steering_controller_.getMeasuredVelocityDegS();
}

float VehicleController::getSteeringControlCommand() const
{
    return steering_controller_.getMotorCommand();
}
