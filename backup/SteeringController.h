#pragma once
#include <Arduino.h>
#include "PidController.h"
#include "SteeringMotor.h"
#include "SteeringSensor.h"

class SteeringController
{
public:
    SteeringController(SteeringMotor& motor, SteeringSensor& sensor);

    void begin();
    void update(bool enabled);
    void stop();

    void setTarget(float angle_deg, float max_velocity_deg_s);

    float getTargetAngleDeg() const;
    float getMaxVelocityDegS() const;
    float getVelocitySetpointDegS() const;
    float getMeasuredVelocityDegS() const;
    float getMotorCommand() const;

private:
    SteeringMotor& motor_;
    SteeringSensor& sensor_;

    PidController position_pid_;
    PidController velocity_pid_;

    float target_angle_deg_;
    float max_velocity_deg_s_;
    float velocity_setpoint_deg_s_;
    float measured_velocity_deg_s_;
    float motor_command_;
    float previous_angle_deg_;
    uint32_t last_update_ms_;
    bool has_previous_angle_;

    float applyActuationDeadband(float command) const;
};
