#pragma once

#include <stdint.h>

/**
 * Plausibilidade entre o encoder traseiro (roda motriz) e as rodas
 * dianteiras (velocidade do veiculo usada pelo controle longitudinal).
 *
 * Com tracao aplicada, a roda traseira bem mais rapida que a media das
 * dianteiras por ENCODER_DISAGREE_TIME_MS indica que a realimentacao nao
 * enxerga o movimento: encoder dianteiro sem sinal (lendo zero) ou roda
 * traseira girando no ar (veiculo suspenso). Nesses casos o controle de
 * velocidade levaria a tracao ao maximo.
 *
 * So o sentido traseira > dianteiras e verificado: ao frear a roda traseira
 * fica mais lenta (pode travar) e em curva as dianteiras ficam mais rapidas,
 * sem gerar falso alarme. Avaliacao pura; a retencao da falha fica no
 * SafetyManager.
 */
class EncoderPlausibility
{
public:
    struct Input
    {
        int64_t now_us;
        bool traction_applied;   // PWM de tracao > 0
        bool speeds_valid;       // velocidades finitas e medidas
        float rear_velocity_m_s;
        float front_velocity_m_s; // media das rodas dianteiras
    };

    EncoderPlausibility();

    // true quando a discordancia persistiu por ENCODER_DISAGREE_TIME_MS.
    bool update(const Input &input);
    void reset();

private:
    int64_t disagree_since_us_; // 0 = sem discordancia
};
