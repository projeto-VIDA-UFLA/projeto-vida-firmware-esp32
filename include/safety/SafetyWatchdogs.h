#pragma once

#include <stdint.h>

class SafetyWatchdogs
{
public:
    struct Input
    {
        int64_t now_us;
        int64_t initialized_at_us;
        int64_t sensor_task_heartbeat_us;
        bool steering_snapshot_valid;
        int64_t steering_snapshot_us;
        bool control_required;
        bool control_output_valid;
        float control_command;
        int64_t control_output_us;
    };

    struct Result
    {
        bool sensor_task_timed_out;
        bool sensor_task_healthy;
        bool steering_snapshot_fresh;
        bool control_output_fresh;
        bool control_output_timed_out;
    };

    static Result evaluate(const Input &input);

private:
    static bool isFresh(int64_t timestamp_us, int64_t now_us,
                        int64_t timeout_us);
};
