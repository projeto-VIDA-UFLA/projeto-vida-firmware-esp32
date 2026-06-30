#include "PidController.h"
#include "utils.h"

PidController::PidController(float kp, float ki, float kd, float output_min, float output_max)
    : kp_(kp),
      ki_(ki),
      kd_(kd),
      output_min_(output_min),
      output_max_(output_max),
      integral_(0.0f),
      previous_error_(0.0f),
      has_previous_error_(false)
{
}

void PidController::setGains(float kp, float ki, float kd)
{
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

void PidController::setOutputLimits(float output_min, float output_max)
{
    output_min_ = output_min;
    output_max_ = output_max;
}

void PidController::reset()
{
    integral_ = 0.0f;
    previous_error_ = 0.0f;
    has_previous_error_ = false;
}

float PidController::update(float setpoint, float measurement, float dt_s)
{
    if (dt_s <= 0.0f)
    {
        return 0.0f;
    }

    float error = setpoint - measurement;
    integral_ += error * dt_s;

    float derivative = 0.0f;
    if (has_previous_error_)
    {
        derivative = (error - previous_error_) / dt_s;
    }

    float output = kp_ * error + ki_ * integral_ + kd_ * derivative;
    float limited_output = clampFloat(output, output_min_, output_max_);

    if (ki_ != 0.0f && output != limited_output)
    {
        integral_ -= error * dt_s;
    }

    previous_error_ = error;
    has_previous_error_ = true;

    return limited_output;
}
