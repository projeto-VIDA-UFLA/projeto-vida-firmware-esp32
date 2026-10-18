#pragma once

#include <Arduino.h>

#include "comunication/CommandMailbox.h"
#include "sensors/SteeringSensor.h"

/**
 * Controle de posicao do esterçamento (TaskControl, a cada
 * TASK_CONTROL_PERIOD_MS).
 *
 * Malha: referencia limitada a STEERING_REFERENCE_*_DEG -> erro -> zona
 * morta com histerese -> PI (D disponivel, hoje Kd = 0) com anti-windup
 * condicional -> saturacao [-1, 1] -> piso STEERING_MIN_ACTUATION ->
 * protecao de fim de curso (STEERING_MIN/MAX_DEG) -> motor_command.
 *
 * Entradas invalidas (referencia nao finita, sensor invalido ou velho, dt
 * fora de (0, STEERING_CONTROL_MAX_DT_S]) resetam o controlador e geram saida
 * invalida, o que para o motor e bloqueia o movimento no SafetyManager.
 * Angulos em graus na convencao de STEERING_INVERT_SIGN (REP-103).
 */
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
        bool inside_deadband;       // dentro da zona morta: comando 0
        bool saturated;             // PID passou de +-1 antes da saturacao
        bool target_clamped;        // referencia limitada a faixa de trabalho
        bool position_limit_active; // fim de curso bloqueou o comando
        bool valid;
    };

    // Ultima saida e seu instante (0 se invalida), lida pelo SafetyManager.
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
    // Zera integral, erro anterior e zona morta (nao zera o relogio do dt).
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
    // Calcula motor_command e publica a saida para getLastOutput().
    Output storeOutput(const Output &output);

    float integral_;
    float previous_error_deg_;
    bool initialized_;
    bool inside_deadband_;
    int64_t previous_update_us_;

    Output last_output_;
    int64_t last_output_timestamp_us_;
    mutable portMUX_TYPE output_mux_ = portMUX_INITIALIZER_UNLOCKED;
};
