#include "uart_link.h"
#include "pins.h"
#include "protocol.h"
#include "state.h"

static HardwareSerial Link(2);

static const uint8_t MAX_PAYLOAD = 80;

static QueueHandle_t g_txQueue = nullptr;
static uint8_t g_seqTx = 0;

// ---------------- TX ----------------
void uartLinkSendFrame(uint8_t type, const uint8_t* payload, uint8_t len) {
  uint8_t seq = g_seqTx++;

  uint8_t crc = 0x00;
  crc = crc8_update(crc, type);
  crc = crc8_update(crc, seq);
  crc = crc8_update(crc, len);
  if (len && payload) crc = crc8_stream(crc, payload, len);

  Link.write(SOF1);
  Link.write(SOF2);
  Link.write(type);
  Link.write(seq);
  Link.write(len);
  if (len && payload) Link.write(payload, len);
  Link.write(crc);
}

bool uartLinkEnqueue(uint8_t type, const void* payload, uint8_t len) {
  if (!g_txQueue) return false;
  if (len > MAX_PAYLOAD) return false;

  OutFrame f{};
  f.type = type;
  f.len = len;
  if (len && payload) memcpy(f.payload, payload, len);

  return (xQueueSend(g_txQueue, &f, 0) == pdTRUE);
}

QueueHandle_t uartLinkGetTxQueue() {
  return g_txQueue;
}

// ---------------- RX Parser ----------------
enum RxState {
  RX_WAIT_SOF1,
  RX_WAIT_SOF2,
  RX_READ_TYPE,
  RX_READ_SEQ,
  RX_READ_LEN,
  RX_READ_PAYLOAD,
  RX_READ_CRC
};

static RxState st = RX_WAIT_SOF1;
static uint32_t frameStartMs = 0;

static uint8_t typeRx = 0, seqRx = 0, lenRx = 0;
static uint8_t payloadRx[MAX_PAYLOAD];
static uint8_t idxRx = 0;
static uint8_t crcRx = 0;

static void resetParser() {
  st = RX_WAIT_SOF1;
  idxRx = 0;
  lenRx = 0;
}

static void handleFrame(uint8_t type, uint8_t seq, uint8_t len, uint8_t* payload) {
  (void)seq;

  if (type == MSG_READY) {
    stateSetEsp1Ready(true);
    return;
  }

  if (type == MSG_TELEM_FAST) {
    if (len != sizeof(TelemFastPayload)) return;

    TelemFastPayload t{};
    memcpy(&t, payload, sizeof(t));
    stateSetTelem(t);
    stateSetLastTelemOkMs(millis());
    return;
  }

  if (type == MSG_HEARTBEAT) {
    // opcional: processar HB do ESP1
    return;
  }
}

void uartLinkPollRx() {
  if (st != RX_WAIT_SOF1 && (millis() - frameStartMs > FRAME_TIMEOUT_MS)) {
    resetParser();
  }

  while (Link.available()) {
    uint8_t b = (uint8_t)Link.read();

    switch (st) {
      case RX_WAIT_SOF1:
        if (b == SOF1) { frameStartMs = millis(); st = RX_WAIT_SOF2; }
        break;

      case RX_WAIT_SOF2:
        st = (b == SOF2) ? RX_READ_TYPE : RX_WAIT_SOF1;
        break;

      case RX_READ_TYPE:
        typeRx = b; st = RX_READ_SEQ;
        break;

      case RX_READ_SEQ:
        seqRx = b; st = RX_READ_LEN;
        break;

      case RX_READ_LEN:
        lenRx = b;
        if (lenRx > MAX_PAYLOAD) resetParser();
        else if (lenRx == 0) st = RX_READ_CRC;
        else { idxRx = 0; st = RX_READ_PAYLOAD; }
        break;

      case RX_READ_PAYLOAD:
        payloadRx[idxRx++] = b;
        if (idxRx >= lenRx) st = RX_READ_CRC;
        break;

      case RX_READ_CRC: {
        crcRx = b;

        uint8_t crc = 0x00;
        crc = crc8_update(crc, typeRx);
        crc = crc8_update(crc, seqRx);
        crc = crc8_update(crc, lenRx);
        crc = crc8_stream(crc, payloadRx, lenRx);

        if (crc == crcRx) {
          handleFrame(typeRx, seqRx, lenRx, payloadRx);
        }
        resetParser();
        break;
      }
    }
  }
}

// ---------------- init ----------------
void uartLinkInit() {
  Link.begin(UART_BAUD, SERIAL_8N1, UART_LINK_RX, UART_LINK_TX);
  g_txQueue = xQueueCreate(16, sizeof(OutFrame));
  resetParser();
}
