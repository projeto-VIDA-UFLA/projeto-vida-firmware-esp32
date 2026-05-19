#pragma once
#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "protocol.h"

// Estado compartilhado (UART <-> ROS)
void stateInit();

// READY/Link
void stateSetEsp1Ready(bool ready);
bool stateGetEsp1Ready();

void stateSetLastTelemOkMs(uint32_t ms);
uint32_t stateGetLastTelemOkMs();
bool stateLinkOk(uint32_t nowMs, uint32_t timeoutMs);

// CMD vindo do ROS
void stateSetCmd(const CmdPayload& cmd);
CmdPayload stateGetCmd();

// Telemetria recebida do ESP1
void stateSetTelem(const TelemFastPayload& t);
bool stateConsumeTelem(TelemFastPayload& out); // retorna true se havia "nova"
