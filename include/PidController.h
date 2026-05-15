#pragma once
#include <Arduino.h>

class PidController
{
public:
    PidController(float kp, float ki, float kd, float output_min, float output_max);

    void setGains(float kp, float ki, float kd);
    void setOutputLimits(float output_min, float output_max);
    void reset();

    float update(float setpoint, float measurement, float dt_s);

private:
    float kp_;
    float ki_;
    float kd_;
    float output_min_;
    float output_max_;
    float integral_;
    float previous_error_;
    bool has_previous_error_;
};
