#include "control/DriveController.h"

#include <cmath>
#include "esp_timer.h"
#include "utils/config.h"
#include "utils/utils.h"

DriveController::Output DriveController::update(
    const CommandMailbox::DriveReferenceSnapshot &reference)
{
    const int64_t now_us = esp_timer_get_time();
    constexpr int64_t reference_timeout_us =
        static_cast<int64_t>(DRIVE_REFERENCE_WATCHDOG_TIMEOUT_MS) * 1000LL;

    const bool reference_is_fresh =
        reference.received &&
        std::isfinite(reference.velocity_m_s) &&
        reference.received_at_us > 0 &&
        now_us >= reference.received_at_us &&
        (now_us - reference.received_at_us) <= reference_timeout_us;

    // Exemplo: referencia 255 -> 255 * 0.15 = 38 de PWM.
    const float command = clampFloat(reference.velocity_m_s *
                                         DRIVE_OPEN_LOOP_PROPORTIONAL_GAIN /
                                         DRIVE_OPEN_LOOP_REFERENCE_MAX_PWM,
                                     -1.0f,
                                     1.0f);

    return Output{command, reference.brake_command, reference_is_fresh};
}
