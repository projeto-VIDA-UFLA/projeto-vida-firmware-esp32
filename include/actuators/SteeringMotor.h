#pragma once
#include <Arduino.h>
#include "actuators/MotorHardware.h"

class SteeringMotor
{
public:
    enum class State : uint8_t
    {
        STOPPED,
        DRIVING_RIGHT,
        DRIVING_LEFT,
        REVERSAL_DEADTIME
    };

    struct Snapshot
    {
        int right_pwm;
        int left_pwm;
        int64_t timestamp_us;
        State state;
    };

    SteeringMotor(uint8_t right_pwm_pin, uint8_t left_pwm_pin,
                  uint8_t right_channel, uint8_t left_channel,
                  MotorHardware *hardware = nullptr);

    void begin();

    // Comando normalizado:
    // > 0: aciona direita
    // < 0: aciona esquerda
    // = 0: parado
    void setCommand(float normalized_command);

    void stop();

    float getLastCommand() const;
    int getRightPwm() const;
    int getLeftPwm() const;
    State getState() const;
    Snapshot getSnapshot() const;

private:
    void writeBothOff();
    void storeSnapshot(float command, int right_pwm, int left_pwm,
                       State state, int64_t timestamp_us);

    uint8_t right_pwm_pin_;
    uint8_t left_pwm_pin_;
    uint8_t right_channel_;
    uint8_t left_channel_;

    float last_command_;
    int right_pwm_;
    int left_pwm_;
    int64_t pwm_timestamp_us_;
    State state_;
    float pending_command_;
    int64_t deadtime_started_us_;
    MotorHardware &hardware_;

    // Protege a leitura/escrita coerente do comando, PWMs e timestamp.
    mutable portMUX_TYPE snapshot_mux_ = portMUX_INITIALIZER_UNLOCKED;
};
