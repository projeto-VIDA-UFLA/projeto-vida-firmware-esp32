#pragma once
#include <Arduino.h>

// Limita value ao intervalo [min_value, max_value].
inline float clampFloat(float value, float min_value, float max_value)
{
    if (value < min_value)
        return min_value;
    if (value > max_value)
        return max_value;
    return value;
}

// Reta A*X + B.
inline float mapFloat(float X, float A, float B)
{
    return A * X + B;
}

// Converte o modulo de um comando normalizado [-1, 1] em duty de 8 bits (0-255).
inline int floatCommandToPwm(float normalized_command)
{
    float value = fabs(clampFloat(normalized_command, -1.0f, 1.0f));
    return static_cast<int>(value * 255.0f);
}
