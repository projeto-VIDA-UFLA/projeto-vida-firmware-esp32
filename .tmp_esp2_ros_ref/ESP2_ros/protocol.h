#pragma once
#include <Arduino.h>

// ================= SOF =================
extern const uint8_t SOF1;
extern const uint8_t SOF2;

// ================= Tipos =================
enum MsgType : uint8_t {
  MSG_HELLO      = 0x01,
  MSG_READY      = 0x02,
  MSG_HEARTBEAT  = 0x03,
  MSG_CMD        = 0x10,
  MSG_TELEM_FAST = 0x11
};

// ================= Payloads =================
#pragma pack(push, 1)

struct CmdPayload {
  float steer_pos_sp;
  float steer_vel_sp;
  float drive_vel_sp;
  float drive_acc_sp;
};

struct TelemFastPayload {
  uint64_t t_sample_us;   // timestamp monotônico da amostra no ESP1 (micros desde boot)
  uint32_t dt_us;         // delta-T local da amostra (debug/sanity)

  // ---- esterçamento ----
  float steer_angle_deg;      // ângulo (graus)
  float steer_rate_rad_s;     // velocidade (rad/s)

  // ---- longitudinal (média + por encoder) ----
  float v_long_mean_mps;      // velocidade longitudinal média (m/s)
  float a_long_mean_mps2;     // aceleração longitudinal média (m/s²)

  float v_enc_r_mps;          // velocidade estimada encoder direito (m/s)
  float v_enc_l_mps;          // velocidade estimada encoder esquerdo (m/s)
  float v_enc_rear_mps;       // velocidade estimada encoder traseiro (m/s)

  // (opcional agora; se não quiser, pode remover e manter só aceleração média)
  float a_enc_r_mps2;         // aceleração por encoder direito (m/s²)
  float a_enc_l_mps2;         // aceleração por encoder esquerdo (m/s²)
  float a_enc_rear_mps2;      // aceleração por encoder traseiro (m/s²)

  // ---- ticks (robustez contra perda de pacote) ----
  int32_t ticks_total_r;
  int32_t ticks_total_l;
  int32_t ticks_total_rear; 

  //controlador
  float u_omega_sterring;
  float u_steering;
};

struct HeartbeatPayload {
  uint32_t uptime_ms;
  uint32_t last_rx_ok_ms;
};

#pragma pack(pop)

// ================= CRC =================
uint8_t crc8_update(uint8_t crc, uint8_t data);
uint8_t crc8_stream(uint8_t crc, const uint8_t* data, size_t len);
