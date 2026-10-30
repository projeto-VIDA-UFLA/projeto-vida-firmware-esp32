#include "control/DriveController.h"

#include <cmath>
#include "esp_timer.h"
#include "utils/config.h"
#include "utils/utils.h"

namespace
{
    // Esforco minimo: freio total.
    constexpr float MIN_EFFORT =
        -(DRIVE_BRAKE_COAST_BAND + 1.0f / DRIVE_BRAKE_EFFORT_GAIN);

    static_assert(DRIVE_MAX_TRACTION_COMMAND > 0.0f &&
                      DRIVE_MAX_TRACTION_COMMAND <= 1.0f,
                  "DRIVE_MAX_TRACTION_COMMAND deve estar em (0, 1]");
    static_assert(DRIVE_TRACTION_DEADZONE_COMMAND >= 0.0f &&
                      DRIVE_TRACTION_DEADZONE_COMMAND < DRIVE_MAX_TRACTION_COMMAND,
                  "DRIVE_TRACTION_DEADZONE_COMMAND deve estar em [0, DRIVE_MAX_TRACTION_COMMAND)");
    static_assert(DRIVE_TRACTION_RUNNING_DEADZONE_COMMAND >= 0.0f &&
                      DRIVE_TRACTION_RUNNING_DEADZONE_COMMAND <= DRIVE_TRACTION_DEADZONE_COMMAND,
                  "DRIVE_TRACTION_RUNNING_DEADZONE_COMMAND deve estar em [0, DRIVE_TRACTION_DEADZONE_COMMAND]");
    static_assert(DRIVE_BRAKE_EFFORT_GAIN > 0.0f,
                  "DRIVE_BRAKE_EFFORT_GAIN deve ser positivo");
    static_assert(DRIVE_BRAKE_ENGAGE_POSITION >= 0.0f &&
                      DRIVE_BRAKE_ENGAGE_POSITION < 1.0f,
                  "DRIVE_BRAKE_ENGAGE_POSITION deve estar em [0, 1)");
    static_assert(DRIVE_BRAKE_ENGAGE_POSITION > BRAKE_DRIVE_CUTOFF,
                  "O freio automatico deve passar do corte de tracao");

    bool isFresh(int64_t timestamp_us, int64_t now_us, int64_t timeout_us)
    {
        // Um carimbo depois de now_us e o mais recente possivel: foi gravado por
        // outra task (outro nucleo) entre a leitura de now_us e a desta medida.
        // Trata-lo como velho gerava bloqueios falsos de um ciclo (bancada 29/10).
        return timestamp_us > 0 &&
               (now_us - timestamp_us) <= timeout_us;
    }
} // namespace

DriveController::DriveController()
    : integral_term_(0.0f),
      target_velocity_m_s_(0.0f),
      target_initialized_(false),
      bench_mode_(false),
      traction_offset_(DRIVE_TRACTION_DEADZONE_COMMAND),
      previous_update_us_(0)
{
}

DriveController::Output DriveController::update(
    const CommandMailbox::DriveReferenceSnapshot &reference,
    const DriveMotor::VelocityMeasurement &vehicle_speed,
    bool motion_permitted,
    bool bench_mode)
{
    constexpr int64_t reference_timeout_us =
        static_cast<int64_t>(DRIVE_REFERENCE_WATCHDOG_TIMEOUT_MS) * 1000LL;
    constexpr int64_t speed_timeout_us =
        static_cast<int64_t>(DRIVE_VELOCITY_MEASUREMENT_TIMEOUT_MS) * 1000LL;

    const int64_t now_us = esp_timer_get_time();

    // No primeiro ciclo nao ha medida anterior: usa o periodo nominal.
    const float dt_s = previous_update_us_ > 0
                           ? static_cast<float>(now_us - previous_update_us_) / 1000000.0f
                           : static_cast<float>(TASK_CONTROL_PERIOD_MS) / 1000.0f;
    previous_update_us_ = now_us;

    Output output{};
    output.measured_velocity_m_s = vehicle_speed.velocity_m_s;

    const bool reference_valid =
        reference.received &&
        std::isfinite(reference.velocity_m_s) &&
        std::isfinite(reference.acceleration_m_s2) &&
        std::isfinite(reference.brake_command) &&
        isFresh(reference.received_at_us, now_us, reference_timeout_us);
    const bool speed_valid =
        vehicle_speed.valid &&
        std::isfinite(vehicle_speed.velocity_m_s) &&
        isFresh(vehicle_speed.timestamp_us, now_us, speed_timeout_us);
    const bool dt_valid = dt_s > 0.0f && dt_s <= DRIVE_CONTROL_MAX_DT_S;

    if (!reference_valid || !speed_valid || !dt_valid)
    {
        reset();
        output.brake_command = 1.0f;
        output.valid = false;
        return output;
    }

    // Troca de modo: a fonte da velocidade muda, recomeca do zero.
    if (bench_mode != bench_mode_)
    {
        reset();
        bench_mode_ = bench_mode;
    }

    output.valid = true;
    const float measured_m_s = vehicle_speed.velocity_m_s;

    // Sem permissao o motor esta parado: integrar acumularia erro (windup).
    // A rampa acompanha a velocidade medida para retomar sem degrau.
    if (!motion_permitted || !target_initialized_)
    {
        integral_term_ = 0.0f;
        target_velocity_m_s_ = std::fmax(0.0f, measured_m_s);
        target_initialized_ = true;
    }

    if (!motion_permitted)
    {
        output.target_velocity_m_s = target_velocity_m_s_;
        output.brake_command = DRIVE_HOLD_BRAKE_POSITION;
        return output;
    }

    // Freio manual (L2): referencia zerada e freio total imediato.
    if (reference.brake_command > DRIVE_BRAKE_OVERRIDE_THRESHOLD)
    {
        // Ao soltar, a rampa recomeca na velocidade medida.
        integral_term_ = 0.0f;
        target_velocity_m_s_ = 0.0f;
        target_initialized_ = false;
        output.brake_override = true;
        output.brake_command = 1.0f;
        return output;
    }

    // Referencia limitada (sem re) e rampa de aceleracao/desaceleracao.
    const float max_speed_m_s = bench_mode ? BENCH_MODE_MAX_SPEED_M_S
                                           : DRIVE_MAX_TARGET_SPEED_M_S;
    const float requested_m_s = clampFloat(reference.velocity_m_s,
                                           0.0f,
                                           max_speed_m_s);
    const float max_accel_m_s2 = reference.acceleration_m_s2 > 0.0f
                                     ? std::fmin(reference.acceleration_m_s2,
                                                 DRIVE_MAX_ACCEL_M_S2)
                                     : DRIVE_DEFAULT_MAX_ACCEL_M_S2;
    const bool decelerating =
        requested_m_s < target_velocity_m_s_ - DRIVE_DECEL_NO_TRACTION_MARGIN_M_S;
    if (requested_m_s > target_velocity_m_s_)
    {
        target_velocity_m_s_ = std::fmin(requested_m_s,
                                         target_velocity_m_s_ + max_accel_m_s2 * dt_s);
    }
    else
    {
        target_velocity_m_s_ = std::fmax(requested_m_s,
                                         target_velocity_m_s_ - DRIVE_MAX_DECEL_M_S2 * dt_s);
    }
    output.target_velocity_m_s = target_velocity_m_s_;

    // Parada: com referencia nula o PI e desligado. Freio de parada ate o
    // veiculo parar, depois freio de estacionamento.
    if (target_velocity_m_s_ <= DRIVE_STOP_REFERENCE_M_S)
    {
        integral_term_ = 0.0f;
        output.holding = std::fabs(measured_m_s) <= DRIVE_STOP_SPEED_THRESHOLD_MPS;
        output.brake_command = output.holding ? DRIVE_HOLD_BRAKE_POSITION
                                              : DRIVE_STOP_BRAKE_POSITION;
        return output;
    }

    const float error_m_s = target_velocity_m_s_ - measured_m_s;
    const float kp = bench_mode ? DRIVE_BENCH_VEL_KP : DRIVE_VEL_KP;
    const float ki = bench_mode ? DRIVE_BENCH_VEL_KI : DRIVE_VEL_KI;
    output.proportional_term = kp * error_m_s;

    // Na bancada a roda gira em vazio e passa da referencia com facilidade: o PI
    // so corta a tracao (roda livre), sem freio automatico. L2 e parada freiam.
    const float min_effort = bench_mode ? 0.0f : MIN_EFFORT;
    // Compensacao da zona morta: parada, a controladora so parte acima de
    // DEADZONE; em movimento, RUNNING_DEADZONE. Na troca a integral absorve a
    // diferenca, entao a tracao aplicada nao da degrau.
    const float traction_offset =
        std::fabs(measured_m_s) < DRIVE_TRACTION_MOVING_SPEED_M_S
            ? DRIVE_TRACTION_DEADZONE_COMMAND
            : DRIVE_TRACTION_RUNNING_DEADZONE_COMMAND;
    if (traction_offset != traction_offset_)
    {
        integral_term_ += traction_offset_ - traction_offset;
        traction_offset_ = traction_offset;
    }

    // Desacelerando o PI nao traciona (so roda livre ou freio).
    const float max_effort =
        decelerating ? 0.0f : DRIVE_MAX_TRACTION_COMMAND - traction_offset_;
    integral_term_ = clampFloat(integral_term_, min_effort, max_effort);

    // Anti-windup condicional: rejeita a integracao que aprofundaria uma
    // saturacao (tracao maxima ou freio total) na mesma direcao.
    const float candidate_integral =
        clampFloat(integral_term_ + ki * error_m_s * dt_s,
                   min_effort, max_effort);
    const float candidate_effort = output.proportional_term + candidate_integral;
    const bool worsens_traction_saturation =
        candidate_effort > max_effort && candidate_integral > integral_term_;
    const bool worsens_brake_saturation =
        candidate_effort < min_effort && candidate_integral < integral_term_;
    if (!worsens_traction_saturation && !worsens_brake_saturation)
    {
        integral_term_ = candidate_integral;
    }
    output.integral_term = integral_term_;

    const float effort = output.proportional_term + integral_term_;
    if (!std::isfinite(effort))
    {
        reset();
        output.brake_command = 1.0f;
        output.valid = false;
        return output;
    }

    output.saturated = effort > max_effort || effort < min_effort;
    output.effort = clampFloat(effort, min_effort, max_effort);
    allocateEffort(output.effort, traction_offset_, output);
    return output;
}

void DriveController::allocateEffort(float effort, float traction_offset, Output &output)
{
    if (effort > 0.0f)
    {
        output.normalized_command = traction_offset + effort;
        output.brake_command = 0.0f;
        return;
    }

    output.normalized_command = 0.0f;

    // Pequeno excesso de velocidade: roda livre, sem freio.
    const float brake_effort =
        clampFloat((-effort - DRIVE_BRAKE_COAST_BAND) * DRIVE_BRAKE_EFFORT_GAIN,
                   0.0f, 1.0f);
    output.brake_command =
        brake_effort > 0.0f
            ? DRIVE_BRAKE_ENGAGE_POSITION +
                  (1.0f - DRIVE_BRAKE_ENGAGE_POSITION) * brake_effort
            : 0.0f;
}

void DriveController::reset()
{
    integral_term_ = 0.0f;
    target_velocity_m_s_ = 0.0f;
    target_initialized_ = false;
    traction_offset_ = DRIVE_TRACTION_DEADZONE_COMMAND;
}
