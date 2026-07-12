#pragma once
#include <Arduino.h>

class DriveMotor
{
public:
    struct Snapshot
    {
        int pwm;
        bool reverse;
        int64_t timestamp_us;
    };

    DriveMotor(uint8_t pwm_pin, uint8_t direction_pin, uint8_t pwm_channel);

    void begin();
    void setCommand(float normalized_command);
    void stop();

    float getLastCommand() const;
    int getLastPwm() const;
    bool isReverse() const;
    Snapshot getSnapshot() const;

private:
    uint8_t pwm_pin_;
    uint8_t direction_pin_;
    uint8_t pwm_channel_;

    float last_command_;
    int last_pwm_;
    bool reverse_;
    int64_t pwm_timestamp_us_;

    // Protege a leitura/escrita coerente do comando, PWM, direção e timestamp.
    mutable portMUX_TYPE snapshot_mux_ = portMUX_INITIALIZER_UNLOCKED;
};
