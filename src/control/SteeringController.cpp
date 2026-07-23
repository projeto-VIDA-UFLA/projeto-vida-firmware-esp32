#include "control/SteeringController.h"
#include "utils/config.h"
#include <cmath>

namespace
{
    float clampValue(float value, float minimum, float maximum)
    {
        if (value < minimum)
        {
            return minimum;
        }
        if (value > maximum)
        {
            return maximum;
        }
        return value;
    }
} // namespace

SteeringController::SteeringController()
    : integral_(0.0f),
      previous_error_deg_(0.0f),
      initialized_(false)
{
}

SteeringController::Output SteeringController::update(const Input &input)
{
    Output output{};
    output.target_angle_deg = input.target_angle_deg;
    output.measured_angle_deg = input.measured_angle_deg;

    if (!std::isfinite(input.target_angle_deg) ||
        !std::isfinite(input.measured_angle_deg) ||
        !std::isfinite(input.dt_s) ||
        input.dt_s <= 0.0f ||
        input.dt_s > STEERING_CONTROL_MAX_DT_S)
    {
        reset();
        return output;
    }

    // A referencia nunca pode solicitar uma posicao fora do curso permitido.
    // O valor limitado tambem e devolvido no Output para que a telemetria
    // represente a referencia efetivamente usada pelo controlador.
    const float target_angle_deg = clampValue(input.target_angle_deg,
                                              STEERING_MIN_DEG,
                                              STEERING_MAX_DEG);
                                              
    output.target_clamped = target_angle_deg != input.target_angle_deg;
    output.target_angle_deg = target_angle_deg;

    const float error_deg = target_angle_deg - input.measured_angle_deg;
    if (!std::isfinite(error_deg))
    {
        reset();
        return output;
    }

    output.angle_error_deg = error_deg;
    output.valid = true;

    float derivative_deg_s = 0.0f;
    if (initialized_)
    {
        derivative_deg_s = (error_deg - previous_error_deg_) / input.dt_s;
        if (!std::isfinite(derivative_deg_s))
        {
            reset();
            output.valid = false;
            return output;
        }
    }
    else
    {
        initialized_ = true;
    }

    // Ganhos e unidades:
    // Kp [comando normalizado/grau]
    // Ki [comando normalizado/(grau-segundo)]
    // Kd [comando normalizado/(grau/segundo)]
    output.proportional_term = STEERING_DIRECT_KP * error_deg;
    output.derivative_term = STEERING_DIRECT_KD * derivative_deg_s;

    if (std::fabs(error_deg) <= STEERING_POSITION_DEADBAND_DEG)
    {
        integral_ = 0.0f;
        previous_error_deg_ = error_deg;
        output.integral_term = 0.0f;
        output.normalized_command = 0.0f;
        output.inside_deadband = true;
        return output;
    }

    float candidate_integral = 0.0f;
    if (STEERING_DIRECT_KI != 0.0f)
    {
        candidate_integral = clampValue(integral_ + error_deg * input.dt_s,
                                        STEERING_DIRECT_INTEGRAL_MIN,
                                        STEERING_DIRECT_INTEGRAL_MAX);
    }

    const float candidate_integral_term = STEERING_DIRECT_KI * candidate_integral;
    const float candidate_command = output.proportional_term +
                                    candidate_integral_term +
                                    output.derivative_term;

    // Anti-windup condicional: rejeita apenas a parcela integral que empurraria
    // uma saida ja saturada ainda mais na mesma direcao. Uma parcela que ajude
    // a sair da saturacao continua permitida.
    const float integral_command_delta =
        STEERING_DIRECT_KI * (candidate_integral - integral_);
    const bool worsens_positive_saturation =
        candidate_command > MAX_ABS_STEER_COMMAND && integral_command_delta > 0.0f;
    const bool worsens_negative_saturation =
        candidate_command < -MAX_ABS_STEER_COMMAND && integral_command_delta < 0.0f;

    if (!worsens_positive_saturation && !worsens_negative_saturation)
    {
        integral_ = candidate_integral;
    }

    output.integral_term = STEERING_DIRECT_KI * integral_;
    const float command = output.proportional_term +
                          output.integral_term +
                          output.derivative_term;

    if (!std::isfinite(command))
    {
        reset();
        output.proportional_term = 0.0f;
        output.integral_term = 0.0f;
        output.derivative_term = 0.0f;
        output.valid = false;
        return output;
    }

    output.saturated = command > MAX_ABS_STEER_COMMAND ||
                       command < -MAX_ABS_STEER_COMMAND;
    output.normalized_command = clampValue(command,
                                           -MAX_ABS_STEER_COMMAND,
                                           MAX_ABS_STEER_COMMAND);

    if (output.normalized_command != 0.0f &&
        std::fabs(output.normalized_command) < STEERING_MIN_ACTUATION)
    {
        output.normalized_command = std::copysign(STEERING_MIN_ACTUATION,
                                                  output.normalized_command);
    }

    output.normalized_command = clampValue(output.normalized_command,
                                           -MAX_ABS_STEER_COMMAND,
                                           MAX_ABS_STEER_COMMAND);

    // Protecao direcional de fim de curso: ao atingir um limite, bloqueia
    // apenas o comando que afastaria ainda mais o esterçamento da faixa.
    // O comando no sentido de retorno permanece permitido.
    const bool pushing_beyond_positive_limit =
        input.measured_angle_deg >= STEERING_MAX_DEG &&
        output.normalized_command > 0.0f;
    const bool pushing_beyond_negative_limit =
        input.measured_angle_deg <= STEERING_MIN_DEG &&
        output.normalized_command < 0.0f;

    if (pushing_beyond_positive_limit || pushing_beyond_negative_limit)
    {
        output.normalized_command = 0.0f;
        output.position_limit_active = true;

        // Evita conservar uma parcela integral que tente manter o motor
        // pressionado contra o fim de curso.
        integral_ = 0.0f;
        output.integral_term = 0.0f;
    }

    previous_error_deg_ = error_deg;
    return output;
}

void SteeringController::reset()
{
    integral_ = 0.0f;
    previous_error_deg_ = 0.0f;
    initialized_ = false;
}
