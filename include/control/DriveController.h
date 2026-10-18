#pragma once

#include <Arduino.h>

#include "comunication/CommandMailbox.h"

/**
 * Controle longitudinal (TaskControl, a cada TASK_CONTROL_PERIOD_MS).
 *
 * Malha aberta: a referencia recebida em velocity_m_s e tratada como um PWM
 * assinado em [-255, 255] e convertida em comando normalizado [-1, 1] com
 * ganho DRIVE_OPEN_LOOP_PROPORTIONAL_GAIN. O freio e repassado da referencia.
 * E aqui que entra o controle de velocidade em malha fechada.
 */
class DriveController
{
public:
    struct Output
    {
        float normalized_command; // tracao [-1, 1]
        float brake_command;      // repassado da referencia [0, 1]
        bool valid; // referencia recente (DRIVE_REFERENCE_WATCHDOG_TIMEOUT_MS) e finita
    };

    Output update(const CommandMailbox::DriveReferenceSnapshot &reference);
};
