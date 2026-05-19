#include "protocol.h"

const uint8_t SOF1 = 0xAA;
const uint8_t SOF2 = 0x55;

// CRC8 poly 0x07 | init 0x00
uint8_t crc8_update(uint8_t crc, uint8_t data) {
  crc ^= data;
  for (uint8_t i = 0; i < 8; i++) {
    crc = (crc & 0x80)
            ? (uint8_t)((crc << 1) ^ 0x07)
            : (uint8_t)(crc << 1);
  }
  return crc;
}

uint8_t crc8_stream(uint8_t crc,
                    const uint8_t* data,
                    size_t len) {
  for (size_t i = 0; i < len; i++) {
    crc = crc8_update(crc, data[i]);
  }
  return crc;
}
