#include "state.h"

static SemaphoreHandle_t g_mutex = nullptr;

static bool g_esp1Ready = false;
static uint32_t g_lastTelemOkMs = 0;

static CmdPayload g_cmd{};
static TelemFastPayload g_telem{};
static bool g_telemNew = false;

void stateInit() {
  g_mutex = xSemaphoreCreateMutex();

  xSemaphoreTake(g_mutex, portMAX_DELAY);
  g_esp1Ready = false;
  g_lastTelemOkMs = 0;
  g_cmd = {0,0,0,0};
  memset(&g_telem, 0, sizeof(g_telem));
  g_telemNew = false;
  xSemaphoreGive(g_mutex);
}

void stateSetEsp1Ready(bool ready) {
  xSemaphoreTake(g_mutex, portMAX_DELAY);
  g_esp1Ready = ready;
  xSemaphoreGive(g_mutex);
}

bool stateGetEsp1Ready() {
  bool out;
  xSemaphoreTake(g_mutex, portMAX_DELAY);
  out = g_esp1Ready;
  xSemaphoreGive(g_mutex);
  return out;
}

void stateSetLastTelemOkMs(uint32_t ms) {
  xSemaphoreTake(g_mutex, portMAX_DELAY);
  g_lastTelemOkMs = ms;
  xSemaphoreGive(g_mutex);
}

uint32_t stateGetLastTelemOkMs() {
  uint32_t out;
  xSemaphoreTake(g_mutex, portMAX_DELAY);
  out = g_lastTelemOkMs;
  xSemaphoreGive(g_mutex);
  return out;
}

bool stateLinkOk(uint32_t nowMs, uint32_t timeoutMs) {
  uint32_t last = stateGetLastTelemOkMs();
  if (last == 0) return false;
  return (nowMs - last) < timeoutMs;
}

void stateSetCmd(const CmdPayload& cmd) {
  xSemaphoreTake(g_mutex, portMAX_DELAY);
  g_cmd = cmd;
  xSemaphoreGive(g_mutex);
}

CmdPayload stateGetCmd() {
  CmdPayload out;
  xSemaphoreTake(g_mutex, portMAX_DELAY);
  out = g_cmd;
  xSemaphoreGive(g_mutex);
  return out;
}

void stateSetTelem(const TelemFastPayload& t) {
  xSemaphoreTake(g_mutex, portMAX_DELAY);
  g_telem = t;
  g_telemNew = true;
  xSemaphoreGive(g_mutex);
}

bool stateConsumeTelem(TelemFastPayload& out) {
  bool had;
  xSemaphoreTake(g_mutex, portMAX_DELAY);
  had = g_telemNew;
  if (had) {
    out = g_telem;
    g_telemNew = false;
  }
  xSemaphoreGive(g_mutex);
  return had;
}
