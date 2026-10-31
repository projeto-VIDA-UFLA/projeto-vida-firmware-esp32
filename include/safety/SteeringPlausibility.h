#pragma once

#include <stdint.h>

/**
 * Plausibilidade entre o comando aplicado ao motor de esterçamento e o
 * angulo medido pelo LVDT.
 *
 * So avalia comandos fortes (|comando| >= STEERING_PLAUSIBILITY_MIN_COMMAND):
 * no piso de atuacao (0,15) o atrito estatico pode segurar a roda por mais de
 * 2 s sem defeito nenhum (teste de 12/10/2026).
 *
 * Comando forte com erro abaixo de STEERING_STALL_MIN_ERROR_DEG, ou com o
 * veiculo abaixo de STEERING_STALL_MIN_SPEED_M_S, nao conta: parado no chao o
 * atrito do pneu pode segurar a direcao (30/10).
 *
 * - Travado: comando forte no mesmo sentido por STEERING_STALL_TIME_MS sem o
 *   angulo avancar STEERING_STALL_MIN_PROGRESS_DEG nesse sentido. Motor ou
 *   mecanismo preso, batente antes do limite de software, ou LVDT sem sinal.
 * - Sentido invertido: depois de STEERING_WRONG_DIRECTION_GRACE_MS do inicio
 *   do comando, o angulo recua STEERING_WRONG_DIRECTION_DEG contra ele.
 *   Motor ou sensor com sinal trocado.
 *
 * Avaliacao pura; a retencao das falhas fica no SafetyManager.
 */
class SteeringPlausibility
{
public:
    struct Input
    {
        int64_t now_us;
        float command;     // comando aplicado [-1, 1], convencao do angulo
        bool angle_valid;
        float angle_deg;   // angulo filtrado (REP-103)
        float error_deg;   // referencia (do controlador) - angulo
        float vehicle_speed_m_s; // para so avaliar o travamento andando
    };

    struct Result
    {
        bool stalled;
        bool wrong_direction;
    };

    SteeringPlausibility();

    Result update(const Input &input);
    void reset();

private:
    int direction_;               // sentido do comando forte atual (0 = nenhum)
    float start_angle_deg_;       // angulo no inicio do comando
    int64_t start_us_;
    float progress_angle_deg_;    // angulo no ultimo avanco confirmado
    int64_t progress_us_;
};
