#include "control/SteeringController.h"
#include "utils/config.h"
#include "esp_timer.h"
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

    static_assert(STEERING_REFERENCE_MIN_DEG >= STEERING_MIN_DEG &&
                      STEERING_REFERENCE_MAX_DEG <= STEERING_MAX_DEG,
                  "A faixa de referencia deve ficar dentro do limite de seguranca");
    static_assert(STEERING_DEADBAND_ENTER_DEG < STEERING_DEADBAND_EXIT_DEG,
                  "A entrada da zona morta deve ser menor que a saida");
} // namespace

SteeringController::SteeringController()
    : integral_(0.0f),
      previous_error_deg_(0.0f),
      initialized_(false),
      inside_deadband_(false),
      previous_update_us_(0),
      last_output_{},
      last_output_timestamp_us_(0)
{
}

SteeringController::Output SteeringController::update(
    const CommandMailbox::SteeringReferenceSnapshot &reference,
    const SteeringSensor::Snapshot &sensor,
    bool motion_permitted)
{
    const int64_t now_us = esp_timer_get_time();
    constexpr int64_t sensor_timeout_us =
        static_cast<int64_t>(STEERING_SENSOR_SNAPSHOT_TIMEOUT_MS) * 1000LL;

    const bool sensor_is_fresh =
        sensor.valid &&
        sensor.timestamp_us > 0 &&
        now_us >= sensor.timestamp_us &&
        (now_us - sensor.timestamp_us) <= sensor_timeout_us;

    // No primeiro ciclo nao ha medida anterior: usa o periodo nominal.
    const float dt_s = previous_update_us_ > 0
                           ? static_cast<float>(now_us - previous_update_us_) / 1000000.0f
                           : static_cast<float>(TASK_CONTROL_PERIOD_MS) / 1000.0f;
    previous_update_us_ = now_us;

    return compute(Input{reference.angle_deg,
                         sensor_is_fresh ? sensor.filtered_angle_deg : NAN,
                         dt_s,
                         motion_permitted});
}

SteeringController::Output SteeringController::compute(const Input &input)
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
        return storeOutput(output);
    }

    // A referencia fica na faixa de trabalho, com margem ate o limite de
    // seguranca. O valor limitado tambem e devolvido no Output para que a
    // telemetria represente a referencia efetivamente usada pelo controlador.
    const float target_angle_deg = clampValue(input.target_angle_deg,
                                              STEERING_REFERENCE_MIN_DEG,
                                              STEERING_REFERENCE_MAX_DEG);
                                              
    output.target_clamped = target_angle_deg != input.target_angle_deg;
    output.target_angle_deg = target_angle_deg;

    const float error_deg = target_angle_deg - input.measured_angle_deg;
    if (!std::isfinite(error_deg))
    {
        reset();
        return storeOutput(output);
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
            return storeOutput(output);
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

    // Histerese: dentro da zona morta so sai acima de EXIT; fora dela so
    // entra abaixo de ENTER.
    const float absolute_error_deg = std::fabs(error_deg);
    inside_deadband_ = inside_deadband_
                           ? absolute_error_deg <= STEERING_DEADBAND_EXIT_DEG
                           : absolute_error_deg <= STEERING_DEADBAND_ENTER_DEG;

    if (inside_deadband_)
    {
        integral_ = 0.0f;
        previous_error_deg_ = error_deg;
        output.integral_term = 0.0f;
        output.normalized_command = 0.0f;
        output.inside_deadband = true;
        return storeOutput(output);
    }

    if (!input.motion_permitted)
    {
        // Atuador desconectado: integrar aqui acumularia erro com o motor
        // parado e, ao habilitar, o esterçamento partiria saturado.
        integral_ = 0.0f;
    }
    else
    {
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

        // Anti-windup condicional: rejeita apenas a parcela integral que
        // empurraria uma saida ja saturada ainda mais na mesma direcao. Uma
        // parcela que ajude a sair da saturacao continua permitida.
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
        return storeOutput(output);
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
    return storeOutput(output);
}

SteeringController::Snapshot SteeringController::getLastOutput() const
{
    portENTER_CRITICAL(&output_mux_);
    const Snapshot snapshot{last_output_, last_output_timestamp_us_};
    portEXIT_CRITICAL(&output_mux_);
    return snapshot;
}

SteeringController::Output SteeringController::storeOutput(const Output &output)
{
    Output stored = output;
    stored.motor_command = STEERING_ANGLE_SIGN * output.normalized_command;

    const bool valid_output =
        stored.valid && std::isfinite(stored.normalized_command);
    const int64_t timestamp_us = valid_output ? esp_timer_get_time() : 0;

    portENTER_CRITICAL(&output_mux_);
    last_output_ = stored;
    last_output_timestamp_us_ = timestamp_us;
    portEXIT_CRITICAL(&output_mux_);
    return stored;
}

void SteeringController::reset()
{
    integral_ = 0.0f;
    previous_error_deg_ = 0.0f;
    initialized_ = false;
    inside_deadband_ = false;
}
