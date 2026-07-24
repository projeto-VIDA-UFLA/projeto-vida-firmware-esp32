#include "safety/SafetyWatchdogs.h"

#include <cmath>
#include "utils/config.h"

SafetyWatchdogs::Result SafetyWatchdogs::evaluate(const Input &input)
{
    constexpr int64_t sensor_task_timeout_us =
        static_cast<int64_t>(SENSOR_TASK_WATCHDOG_TIMEOUT_MS) * 1000LL;
    constexpr int64_t sensor_startup_grace_us =
        static_cast<int64_t>(SENSOR_TASK_STARTUP_GRACE_MS) * 1000LL;
    constexpr int64_t steering_snapshot_timeout_us =
        static_cast<int64_t>(STEERING_SENSOR_SNAPSHOT_TIMEOUT_MS) * 1000LL;
    constexpr int64_t control_output_timeout_us =
        static_cast<int64_t>(CONTROL_OUTPUT_WATCHDOG_TIMEOUT_MS) * 1000LL;

    const bool sensor_task_started = input.sensor_task_heartbeat_us > 0;
    const bool sensor_task_timed_out =
        sensor_task_started
            ? !isFresh(input.sensor_task_heartbeat_us, input.now_us,
                       sensor_task_timeout_us)
            : input.initialized_at_us > 0 &&
                  input.now_us >= input.initialized_at_us &&
                  (input.now_us - input.initialized_at_us) >
                      sensor_startup_grace_us;

    const bool steering_snapshot_fresh =
        input.steering_snapshot_valid &&
        isFresh(input.steering_snapshot_us, input.now_us,
                steering_snapshot_timeout_us);

    const bool control_output_fresh =
        input.control_output_valid &&
        std::isfinite(input.control_command) &&
        isFresh(input.control_output_us, input.now_us,
                control_output_timeout_us);

    return Result{
        sensor_task_timed_out,
        sensor_task_started && !sensor_task_timed_out,
        steering_snapshot_fresh,
        control_output_fresh,
        input.control_required && !control_output_fresh};
}

bool SafetyWatchdogs::isFresh(int64_t timestamp_us, int64_t now_us,
                              int64_t timeout_us)
{
    return timestamp_us > 0 &&
           now_us >= timestamp_us &&
           (now_us - timestamp_us) <= timeout_us;
}
