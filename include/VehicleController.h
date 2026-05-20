#pragma once
#include <Arduino.h>
#include "DriveMotor.h"
#include "SteeringMotor.h"
#include "SteeringSensor.h"
#include "SafetyManager.h"
#include "EncoderReader.h"
#include "SteeringController.h"
#include "DriveVelocityController.h"

enum class VehicleControlMode
{
    ClosedLoop,
    OpenLoop
};

class VehicleController
{
public:
    VehicleController(DriveMotor& drive_motor,
                      SteeringMotor& steering_motor,
                      SteeringSensor& steering_sensor,
                      EncoderReader& rear_encoder,
                      EncoderReader& left_encoder,
                      EncoderReader& right_encoder,
                      SafetyManager& safety_manager);

    void begin();
    void update();

    void setSpeedTarget(float velocity_m_s, float max_accel_m_s2);
    void setSteeringTarget(float target_deg, float max_velocity_deg_s);
    void setOpenLoopCommand(float drive_command, float steering_target_normalized);
    void setControlMode(VehicleControlMode mode);
    VehicleControlMode getControlMode() const;

    float getTargetVelocityMps() const;
    float getRampedVelocityMps() const;
    float getMeasuredVelocityMps() const;
    float getMeasuredAccelMps2() const;
    float getAccelSetpointMps2() const;
    float getMaxAccelMps2() const;
    float getDriveCommand() const;

    float getSteeringTargetDeg() const;
    float getSteeringVelocityMaxDegS() const;
    float getSteeringVelocitySetpointDegS() const;
    float getSteeringMeasuredVelocityDegS() const;
    float getSteeringControlCommand() const;

private:
    DriveMotor& drive_motor_;
    SteeringMotor& steering_motor_;
    SteeringSensor& steering_sensor_;
    SafetyManager& safety_manager_;

    SteeringController steering_controller_;
    DriveVelocityController drive_controller_;

    VehicleControlMode control_mode_;
    float open_loop_drive_command_;
    float open_loop_steering_target_deg_;
};
