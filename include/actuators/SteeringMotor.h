#pragma once
#include <Arduino.h>
#include "actuators/MotorHardware.h"

/**
 * Motor de estercamento 24 V em ponte H com dois PWMs independentes: um gira
 * para a direita e o outro para a esquerda.
 *
 * Os dois PWMs nunca ficam ativos juntos. Ao inverter o sentido o motor passa
 * por REVERSAL_DEADTIME, com ambos em zero por STEERING_DIRECTION_DEADTIME_US.
 * Esta classe trabalha no sentido fisico do motor; a conversao a partir da
 * convencao do angulo e feita pelo SteeringController (motor_command).
 */
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

    // Configura os dois canais PWM e parte parado.
    void begin();

    // Comando normalizado [-1, 1]:
    // > 0: aciona direita
    // < 0: aciona esquerda
    // = 0: parado
    // Comando nao finito para o motor.
    void setCommand(float normalized_command);

    // Zera os dois PWMs imediatamente.
    void stop();

    // Emergencia do supervisor de tasks: zera os dois PWMs direto no
    // hardware, sem tocar no estado (pode ser chamado sem o mutex).
    void forceOutputOff();

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
