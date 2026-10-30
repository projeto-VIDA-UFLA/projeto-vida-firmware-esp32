#pragma once

#include <Arduino.h>

#include "actuators/DriveMotor.h"
#include "comunication/CommandMailbox.h"

/**
 * Controle longitudinal de velocidade (TaskControl, a cada
 * TASK_CONTROL_PERIOD_MS).
 *
 * Malha:
 *   referencia (m/s) -> limite [0, DRIVE_MAX_TARGET_SPEED_M_S] -> rampa de
 *   aceleracao/desaceleracao -> PI com anti-windup -> esforco
 *   esforco > 0: tracao (ate DRIVE_MAX_TRACTION_COMMAND)
 *   esforco < 0: roda livre na faixa DRIVE_BRAKE_COAST_BAND, depois freio
 * Realimentacao: velocidade do veiculo (media das rodas dianteiras).
 *
 * Casos especiais:
 * - freio manual (brake_command da referencia acima de
 *   DRIVE_BRAKE_OVERRIDE_THRESHOLD): referencia zerada e freio total;
 * - parada: referencia ~0 -> PI desligado, freio de parada ate o veiculo
 *   parar e depois freio de estacionamento;
 * - sem permissao de movimento: integral zerada e rampa acompanhando a
 *   velocidade medida, para retomar sem degrau ao habilitar.
 * Referencia ou medicao invalida/velha, ou dt fora da faixa, geram saida
 * invalida (o ActuatorManager para tudo e freia em rampa).
 * Sem re: o comando de tracao nunca e negativo.
 */
class DriveController
{
public:
    struct Output
    {
        float normalized_command; // tracao: 0 ou [offset, DRIVE_MAX_TRACTION_COMMAND]
        float brake_command;      // freio [0, 1]
        bool valid;

        // Diagnostico
        float target_velocity_m_s;   // referencia apos limite e rampa
        float measured_velocity_m_s;
        float proportional_term;
        float integral_term;
        float effort;                // saida do PI: > 0 tracao, < 0 freio
        bool brake_override;         // freio manual pelo ROS
        bool holding;                // parado com freio de estacionamento
        bool saturated;
    };

    DriveController();

    // bench_mode limita a referencia a BENCH_MODE_MAX_SPEED_M_S; a troca de
    // modo reinicia o controlador (a fonte da velocidade muda).
    Output update(const CommandMailbox::DriveReferenceSnapshot &reference,
                  const DriveMotor::VelocityMeasurement &vehicle_speed,
                  bool motion_permitted,
                  bool bench_mode);

    // Zera integral e rampa (a rampa recomeca na velocidade medida).
    void reset();

private:
    // Converte o esforco do PI em tracao (offset + esforco) e freio.
    static void allocateEffort(float effort, float traction_offset, Output &output);

    float integral_term_;
    float target_velocity_m_s_;
    bool target_initialized_;
    bool bench_mode_;
    float traction_offset_; // compensacao da zona morta em uso
    int64_t previous_update_us_;
};
