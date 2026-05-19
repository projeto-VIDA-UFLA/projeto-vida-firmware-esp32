#include <Arduino.h>
#include <micro_ros_arduino.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

// micro-ROS time sync / epoch time
#include <rmw_microros/rmw_microros.h>

// Interfaces VIDA
#include <vida_interfaces/msg/telemetry.h>
#include <vida_interfaces/msg/cmd.h>

// Projeto
#include "state.h"
#include "protocol.h"
#include "pins.h"

// ------------------------------------------------------------
// RCL/RCLC globals
// ------------------------------------------------------------
static rcl_allocator_t allocator;
static rclc_support_t support;
static rcl_node_t node;

static rcl_publisher_t pub_telem;
static rcl_subscription_t sub_cmd;
static rclc_executor_t executor;

// Mensagens (C structs)
static vida_interfaces__msg__Telemetry telem_msg;
static vida_interfaces__msg__Cmd cmd_msg;

// ------------------------------------------------------------
// Time sync helpers
// ------------------------------------------------------------
static bool g_time_synced = false;

static inline void stamp_from_epoch(std_msgs__msg__Header * h, int64_t epoch_ns)
{
  if (epoch_ns <= 0) {
    h->stamp.sec = 0;
    h->stamp.nanosec = 0;
    return;
  }

  h->stamp.sec = (int32_t)(epoch_ns / 1000000000LL);
  h->stamp.nanosec = (uint32_t)(epoch_ns % 1000000000LL);
}

static void try_sync_time()
{
  // Tenta sincronizar com o agent. Se já estiver syncado, não precisa ficar chamando.
  // Timeout em ms.
  if (!g_time_synced) {
    if (rmw_uros_sync_session(1000) == RMW_RET_OK) {
      g_time_synced = true;
    }
  }
}

// ------------------------------------------------------------
// Helpers de Header sem alocação dinâmica
// ------------------------------------------------------------
static inline void header_zero(std_msgs__msg__Header * h)
{
  h->stamp.sec = 0;
  h->stamp.nanosec = 0;

  // frame_id vazio
  h->frame_id.data = NULL;
  h->frame_id.size = 0;
  h->frame_id.capacity = 0;
}

// ------------------------------------------------------------
// Subscriber callback: ROS2 -> ESP2 -> stateSetCmd()
// ------------------------------------------------------------
static void cmd_callback(const void * msgin)
{
  const vida_interfaces__msg__Cmd * m = (const vida_interfaces__msg__Cmd *)msgin;

  CmdPayload cmd{};
  if (m->mode == 0) {
    // STOP: zera setpoints (sem precisar alterar o protocolo UART)
    cmd.steer_pos_sp  = 0.0f;
    cmd.steer_vel_sp  = 0.0f;
    cmd.drive_vel_sp  = 0.0f;
    cmd.drive_acc_sp  = 0.0f;
  } else {
    // Ativo
    cmd.steer_pos_sp  = m->steer_pos_sp_deg;     // graus
    cmd.steer_vel_sp  = m->steer_vel_sp_rad_s;   // rad/s
    cmd.drive_vel_sp  = m->drive_vel_sp_mps;     // m/s
    cmd.drive_acc_sp  = m->drive_acc_sp_mps2;    // m/s²
  }

  stateSetCmd(cmd);
}

// ------------------------------------------------------------
// API
// ------------------------------------------------------------
void rosInit()
{
  // Transporte padrão do micro_ros_arduino no ESP32
  // (é o que seu exemplo "funcionando" usa)
  set_microros_transports();

  allocator = rcl_get_default_allocator();
  delay(1500);

  // Support + node
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "vida_esp2_bridge", "", &support);

  // Publisher: Telemetry
  rclc_publisher_init_default(
    &pub_telem,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(vida_interfaces, msg, Telemetry),
    "/vida/telem"
  );

  // Subscriber: Cmd
  rclc_subscription_init_default(
    &sub_cmd,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(vida_interfaces, msg, Cmd),
    "/vida/cmd"
  );

  // Init msgs (zera + prepara structs internos)
  vida_interfaces__msg__Telemetry__init(&telem_msg);
  vida_interfaces__msg__Cmd__init(&cmd_msg);

  // Headers zerados por enquanto (sem sync)
  header_zero(&telem_msg.header);
  header_zero(&cmd_msg.header);

  // Executor (1 subscription)
  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &sub_cmd, &cmd_msg, &cmd_callback, ON_NEW_DATA);

  // Tenta sincronizar tempo com o Agent (epoch)
  try_sync_time();
}

void rosSpinSome()
{
  // 1) Processa callbacks do ROS (cmd)
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(3));

  // 2) Publica telemetria quando houver nova do ESP1
  TelemFastPayload t{};
  if (stateConsumeTelem(t))
  {
    // Tenta sync do tempo (se ainda não conseguiu)
    try_sync_time();

    // Timestamp do ROS (epoch) via micro-ROS
    const int64_t now_epoch_ns = rmw_uros_epoch_nanos();
    stamp_from_epoch(&telem_msg.header, now_epoch_ns);

    // --- Copia 1:1 do payload UART -> msg ROS ---
    telem_msg.t_sample_us = (uint64_t)t.t_sample_us;
    telem_msg.dt_us       = (uint32_t)t.dt_us;

    telem_msg.steer_angle_deg    = t.steer_angle_deg;
    telem_msg.steer_rate_rad_s   = t.steer_rate_rad_s;

    telem_msg.v_long_mean_mps    = t.u_omega_sterring;
    telem_msg.a_long_mean_mps2   = t.u_steering;

    telem_msg.v_enc_r_mps        = t.v_enc_r_mps;
    telem_msg.v_enc_l_mps        = t.v_enc_l_mps;
    telem_msg.v_enc_rear_mps     = t.v_enc_rear_mps;

    telem_msg.a_enc_r_mps2       = t.a_enc_r_mps2;
    telem_msg.a_enc_l_mps2       = t.a_enc_l_mps2;
    telem_msg.a_enc_rear_mps2    = t.a_enc_rear_mps2;

    telem_msg.ticks_total_r      = t.ticks_total_r;
    telem_msg.ticks_total_l      = t.ticks_total_l;
    telem_msg.ticks_total_rear   = t.ticks_total_rear;

    // Publica
    rcl_ret_t rc = rcl_publish(&pub_telem, &telem_msg, NULL);
    (void)rc; // se não quiser warning

    // Se quiser debugar quando "não publica", descomente:
    // if (rc != RCL_RET_OK) {
    //   Serial.printf("rcl_publish failed: %d\n", (int)rc);
    // }
  }
}
