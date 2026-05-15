#pragma once
#include <Arduino.h>
#include "DriveMotor.h"
#include "EncoderReader.h"
#include "PidController.h"

class DriveVelocityController
{
public:
    DriveVelocityController(DriveMotor& motor,
                            EncoderReader& rear_encoder,
                            EncoderReader& left_encoder,
                            EncoderReader& right_encoder);

    void begin();
    void update(bool enabled);
    void stop();

    void setTarget(float velocity_m_s, float max_accel_m_s2);

    float getTargetVelocityMps() const;
    float getRampedVelocityMps() const;
    float getMeasuredVelocityMps() const;
    float getMeasuredAccelMps2() const;
    float getAccelSetpointMps2() const;
    float getMaxAccelMps2() const;
    float getMotorCommand() const;

private:
    DriveMotor& motor_;
    EncoderReader& rear_encoder_;
    EncoderReader& left_encoder_;
    EncoderReader& right_encoder_;

    PidController velocity_pid_;
    PidController accel_pid_;

    float target_velocity_m_s_;
    float ramped_velocity_m_s_;
    float measured_velocity_m_s_;
    float measured_accel_m_s2_;
    float accel_setpoint_m_s2_;
    float max_accel_m_s2_;
    float motor_command_;
    float previous_velocity_m_s_;
    uint32_t last_update_ms_;
    bool has_previous_velocity_;

    float encoderDeltaToDistanceM(long delta_ticks) const;
    float applyActuationDeadband(float command) const;
};
