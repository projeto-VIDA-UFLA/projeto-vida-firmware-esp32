#pragma once

#include <Arduino.h>

#include "comunication/CommandMailbox.h"

/**
 * Controle longitudinal.
 *
 * Malha aberta: a referencia recebida em velocity_m_s e tratada como um PWM
 * assinado em [-255, 255] e convertida em comando normalizado [-1, 1].
 */
class DriveController
{
public:
    struct Output
    {
        float normalized_command;
        float brake_command;
        bool valid; // referencia recente e finita
    };

    Output update(const CommandMailbox::DriveReferenceSnapshot &reference);
};
