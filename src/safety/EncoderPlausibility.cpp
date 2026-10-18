#include "safety/EncoderPlausibility.h"

#include "utils/config.h"

EncoderPlausibility::EncoderPlausibility()
    : disagree_since_us_(0)
{
}

bool EncoderPlausibility::update(const Input &input)
{
    constexpr int64_t disagree_time_us =
        static_cast<int64_t>(ENCODER_DISAGREE_TIME_MS) * 1000LL;

    const bool disagree =
        input.traction_applied &&
        input.speeds_valid &&
        (input.rear_velocity_m_s - input.front_velocity_m_s) >
            ENCODER_DISAGREE_MARGIN_M_S;

    if (!disagree)
    {
        disagree_since_us_ = 0;
        return false;
    }

    if (disagree_since_us_ == 0)
    {
        disagree_since_us_ = input.now_us;
    }

    return (input.now_us - disagree_since_us_) >= disagree_time_us;
}

void EncoderPlausibility::reset()
{
    disagree_since_us_ = 0;
}
