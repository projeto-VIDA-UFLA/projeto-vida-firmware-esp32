#pragma once

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

    SteeringController();

    Output update(const Input &input);
    void reset();

private:
    float integral_;
    float previous_error_deg_;
    bool initialized_;
};
