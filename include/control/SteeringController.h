#pragma once

#include <Arduino.h>

class SteeringController
{
public:
    struct Input
    {
        float target_angle_deg;
        float measured_angle_deg;
        float dt_s;
    };

    struct Output
    {
        float target_angle_deg;
        float measured_angle_deg;
        float angle_error_deg;

        float proportional_term;
        float integral_term;
        float derivative_term;

        float normalized_command;
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

    Output update(const Input &input);
    Snapshot getLastOutput() const;
    void reset();

private:
    Output storeOutput(const Output &output);

    float integral_;
    float previous_error_deg_;
    bool initialized_;

    Output last_output_;
    int64_t last_output_timestamp_us_;
    mutable portMUX_TYPE output_mux_ = portMUX_INITIALIZER_UNLOCKED;
};
