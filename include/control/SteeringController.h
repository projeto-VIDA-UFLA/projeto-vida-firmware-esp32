#pragma once

#include <Arduino.h>

#include "comunication/CommandMailbox.h"
#include "sensors/SteeringSensor.h"

class SteeringController
{
public:
    struct Output
    {
        float target_angle_deg;
        float measured_angle_deg;
        float angle_error_deg;

        float proportional_term;
        float integral_term;
        float derivative_term;

        // Comando na convencao do angulo (STEERING_INVERT_SIGN).
        float normalized_command;
        // Mesmo comando na convencao do motor: > 0 aciona o PWM da direita.
        float motor_command;
        bool inside_deadband;
        bool saturated;
        bool target_clamped;
        bool position_limit_active;
        bool valid;
    };

    struct Snapshot
    {
        Output output;
        int64_t timestamp_us;
    };

    SteeringController();

    // Sensor com leitura invalida ou velha invalida a saida.
    // Sem permissao de movimento o motor esta parado: a integral e mantida
    // em zero para nao acumular erro (windup) ate a proxima habilitacao.
    Output update(const CommandMailbox::SteeringReferenceSnapshot &reference,
                  const SteeringSensor::Snapshot &sensor,
                  bool motion_permitted);
    Snapshot getLastOutput() const;
    void reset();

private:
    struct Input
    {
        float target_angle_deg;
        float measured_angle_deg;
        float dt_s;
        bool motion_permitted;
    };

    Output compute(const Input &input);
    Output storeOutput(const Output &output);

    float integral_;
    float previous_error_deg_;
    bool initialized_;
    int64_t previous_update_us_;

    Output last_output_;
    int64_t last_output_timestamp_us_;
    mutable portMUX_TYPE output_mux_ = portMUX_INITIALIZER_UNLOCKED;
};
