#pragma once

#include <Arduino.h>
#include "actuators/MotorHardware.h"

class DriveMotor
{
public:
    enum class State : uint8_t
    {
        STOPPED,
        DRIVING,
        WAITING_FOR_STOP,
        DIRECTION_DEADTIME
    };

    struct VelocityMeasurement
    {
        float velocity_m_s;
        int64_t timestamp_us;
        bool valid;
    };

    struct Snapshot
    {
        int pwm;
        bool reverse;
        int64_t timestamp_us;
        State state;
        bool direction_change_pending;
    };

    DriveMotor(uint8_t pwm_pin, uint8_t direction_pin, uint8_t pwm_channel,
               MotorHardware *hardware = nullptr);

    void begin();
    void setCommand(float normalized_command,
                    const VelocityMeasurement &velocity_measurement);
    void stop();

    float getLastCommand() const;
    int getLastPwm() const;
    bool isReverse() const;
    State getState() const;
    Snapshot getSnapshot() const;

private:
    bool isVelocityFreshAndStopped(const VelocityMeasurement &measurement,
                                   int64_t now_us) const;
    void writePwmZero();
    void storeSnapshot(float command, int pwm, State state,
                       int64_t timestamp_us);

    uint8_t pwm_pin_;
    uint8_t direction_pin_;
    uint8_t pwm_channel_;

    float last_command_;
    int last_pwm_;
    bool reverse_;
    int64_t pwm_timestamp_us_;
    State state_;
    bool pending_reverse_;
    int64_t stopped_since_us_;
    int64_t direction_changed_at_us_;
    MotorHardware &hardware_;

    // Protege a leitura/escrita coerente do comando, PWM, direcao e timestamp.
    mutable portMUX_TYPE snapshot_mux_ = portMUX_INITIALIZER_UNLOCKED;
};
