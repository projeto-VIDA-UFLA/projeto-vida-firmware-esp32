#pragma once
#include <Arduino.h>

class SteeringMotor
{
public:
    struct Snapshot
    {
        int right_pwm;
        int left_pwm;
        int64_t timestamp_us;
    };

    SteeringMotor(uint8_t right_pwm_pin, uint8_t left_pwm_pin, uint8_t right_channel, uint8_t left_channel);

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
    Snapshot getSnapshot() const;

private:
    uint8_t right_pwm_pin_;
    uint8_t left_pwm_pin_;
    uint8_t right_channel_;
    uint8_t left_channel_;

    float last_command_;
    int right_pwm_;
    int left_pwm_;
    int64_t pwm_timestamp_us_;

    // Protege a leitura/escrita coerente do comando, PWMs e timestamp.
    mutable portMUX_TYPE snapshot_mux_ = portMUX_INITIALIZER_UNLOCKED;
};
