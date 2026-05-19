#pragma once
#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "protocol.h"

struct OutFrame {
  uint8_t type;
  uint8_t len;
  uint8_t payload[80];
};

void uartLinkInit();
QueueHandle_t uartLinkGetTxQueue();

void uartLinkPollRx();              // chama dentro da TaskRx
void uartLinkSendFrame(uint8_t type, const uint8_t* payload, uint8_t len); // usado só pela TaskTx

bool uartLinkEnqueue(uint8_t type, const void* payload, uint8_t len);
