#include "safety/SteeringPlausibility.h"

#include <cmath>
#include "utils/config.h"

namespace
{
    constexpr int64_t msToUs(int64_t ms)
    {
        return ms * 1000LL;
    }

    static_assert(STEERING_PLAUSIBILITY_MIN_COMMAND > STEERING_DEADZONE_COMPENSATION,
                  "A plausibilidade nao deve avaliar a compensacao da zona morta");
    static_assert(STEERING_WRONG_DIRECTION_GRACE_MS < STEERING_STALL_TIME_MS,
                  "A tolerancia de inversao deve ser menor que o tempo de travamento");
} // namespace

SteeringPlausibility::SteeringPlausibility()
{
    reset();
}

void SteeringPlausibility::reset()
{
    direction_ = 0;
    start_angle_deg_ = 0.0f;
    start_us_ = 0;
    progress_angle_deg_ = 0.0f;
    progress_us_ = 0;
}

SteeringPlausibility::Result SteeringPlausibility::update(const Input &input)
{
    Result result{false, false};

    const bool large_error =
        std::isfinite(input.error_deg) &&
        std::fabs(input.error_deg) >= STEERING_STALL_MIN_ERROR_DEG &&
        std::isfinite(input.vehicle_speed_m_s) &&
        std::fabs(input.vehicle_speed_m_s) >= STEERING_STALL_MIN_SPEED_M_S;
    const int direction =
        !large_error                                          ? 0
        : input.command >= STEERING_PLAUSIBILITY_MIN_COMMAND  ? 1
        : input.command <= -STEERING_PLAUSIBILITY_MIN_COMMAND ? -1
                                                               : 0;

    if (direction == 0 || !input.angle_valid || !std::isfinite(input.angle_deg))
    {
        // Sem comando forte nao ha movimento exigido.
        direction_ = 0;
        return result;
    }

    if (direction != direction_)
    {
        direction_ = direction;
        start_angle_deg_ = input.angle_deg;
        start_us_ = input.now_us;
        progress_angle_deg_ = input.angle_deg;
        progress_us_ = input.now_us;
        return result;
    }

    const float progress_deg =
        static_cast<float>(direction) * (input.angle_deg - progress_angle_deg_);
    if (progress_deg >= STEERING_STALL_MIN_PROGRESS_DEG)
    {
        progress_angle_deg_ = input.angle_deg;
        progress_us_ = input.now_us;
    }

    result.stalled =
        (input.now_us - progress_us_) >= msToUs(STEERING_STALL_TIME_MS);

    // Logo apos o inicio a inercia e o filtro ainda seguem o sentido anterior.
    const float travel_deg =
        static_cast<float>(direction) * (input.angle_deg - start_angle_deg_);
    result.wrong_direction =
        (input.now_us - start_us_) >= msToUs(STEERING_WRONG_DIRECTION_GRACE_MS) &&
        travel_deg <= -STEERING_WRONG_DIRECTION_DEG;

    return result;
}
