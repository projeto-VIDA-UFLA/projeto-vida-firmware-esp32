#pragma once
#include <Arduino.h>

inline float clampFloat(float value, float min_value, float max_value)
{
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

inline float mapFloat(float x, float in_min, float in_max, float out_min, float out_max)
{
    if (in_max == in_min) return out_min;
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

inline int floatCommandToPwm(float normalized_command)
{
    float value = fabs(clampFloat(normalized_command, -1.0f, 1.0f));
    return static_cast<int>(value * 255.0f);
}
