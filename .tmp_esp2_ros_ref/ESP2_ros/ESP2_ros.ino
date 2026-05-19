#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "pins.h"
#include "protocol.h"
#include "state.h"
#include "uart_link.h"
#include "ros_bridge.h"

// Tasks
static void TaskUartRx(void* pv);
static void TaskUartTx(void* pv);
static void TaskRos(void* pv);

void setup() {
  Serial.begin(115200);

  stateInit();
  uartLinkInit();
  rosInit();

  // Prioridades (ajuste fino):
  // RX: mais alta para não perder bytes
  // TX: média
  // ROS: pode ser menor (mas não muito, pra não travar executor)
  xTaskCreatePinnedToCore(TaskUartRx, "UartRx", 4096, nullptr, 3, nullptr, 0);
  xTaskCreatePinnedToCore(TaskUartTx, "UartTx", 4096, nullptr, 2, nullptr, 0);
  xTaskCreatePinnedToCore(TaskRos,    "ROS",    8192, nullptr, 1, nullptr, 1);

  Serial.println("ESP2 Bridge OK (FreeRTOS + módulos)");
}

void loop() {}

// ---------------- RX UART ----------------
static void TaskUartRx(void* pv) {
  (void)pv;
  for (;;) {
    uartLinkPollRx();
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

// ---------------- TX UART ----------------
static void TaskUartTx(void* pv) {
  (void)pv;

  QueueHandle_t q = uartLinkGetTxQueue();

  const TickType_t cmdPeriod = pdMS_TO_TICKS(1000 / CMD_RATE_HZ);      // 20 ms
  const TickType_t helloPeriod = pdMS_TO_TICKS(1000 / HELLO_RATE_HZ);  // 1s

  TickType_t lastWake = xTaskGetTickCount();
  TickType_t nextCmd  = lastWake;
  TickType_t nextHello = lastWake;
  TickType_t nextHb   = lastWake;

  for (;;) {
    // 1) manda frames pendentes (se algum módulo enfileirar)
    OutFrame f;
    while (xQueueReceive(q, &f, 0) == pdTRUE) {
      uartLinkSendFrame(f.type, f.payload, f.len);
    }

    TickType_t now = xTaskGetTickCount();

    // 2) Monitor link: se caiu, volta handshake
    bool link_ok = stateLinkOk(millis(), LINK_TIMEOUT_MS);
    if (!link_ok && stateGetEsp1Ready()) {
      stateSetEsp1Ready(false);
    }

    // 3) Handshake HELLO até READY
    if (!stateGetEsp1Ready() && now >= nextHello) {
      nextHello = now + helloPeriod;
      const char hello[] = "ESP2";
      uartLinkSendFrame(MSG_HELLO, (const uint8_t*)hello, (uint8_t)strlen(hello));
    }

      // 5) CMD 50 Hz (só se READY)
    if (stateGetEsp1Ready() && now >= nextCmd) {
      nextCmd = now + cmdPeriod;
      CmdPayload cmd = stateGetCmd();
      uartLinkSendFrame(MSG_CMD, (const uint8_t*)&cmd, (uint8_t)sizeof(cmd));
    }

    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(2));
  }
}

// ---------------- ROS (micro-ROS executor + publish) ----------------
static void TaskRos(void* pv) {
  (void)pv;
  for (;;) {
    rosSpinSome();
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}
