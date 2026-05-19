#pragma once

// UART entre ESP2 e ESP1
#define UART_LINK_RX 16
#define UART_LINK_TX 17

// Parâmetros
static const uint32_t UART_BAUD = 115200;

// Tarefas
static const uint32_t CMD_RATE_HZ   = 50;   // envia CMD 50 Hz
static const uint32_t HELLO_RATE_HZ = 1;    // handshake 1 Hz

// Timeout
static const uint32_t FRAME_TIMEOUT_MS = 20;
static const uint32_t LINK_TIMEOUT_MS  = 120; // sem telem válida => link down
