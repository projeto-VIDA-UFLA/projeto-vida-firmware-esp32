#pragma once

#include <Arduino.h>

#include "actuators/ActuatorManager.h"
#include "comunication/CommandMailbox.h"
#include "control/SteeringController.h"
#include "safety/EncoderPlausibility.h"
#include "sensors/EncoderReader.h"
#include "sensors/SteeringSensor.h"

// Estado publicado em /vida/safety/status (mesmos valores do SafetyStatus.msg).
enum class SafetyState : uint8_t
{
    STATE_DISABLED = 0,
    STATE_READY = 1
};

// Motivos de bloqueio do movimento; varios podem estar ativos ao mesmo tempo.
enum SafetyInhibitFlag : uint32_t
{
    INHIBIT_NONE = 0,
    INHIBIT_PHYSICAL_ENABLE_OFF = 1U << 0,
    INHIBIT_LOGICAL_ENABLE_NOT_RECEIVED = 1U << 1,
    INHIBIT_LOGICAL_ENABLE_FALSE = 1U << 2,
    INHIBIT_LOGICAL_ENABLE_TIMEOUT = 1U << 3,
    INHIBIT_DRIVE_REFERENCE_NOT_FRESH = 1U << 4,
    INHIBIT_STEERING_REFERENCE_NOT_FRESH = 1U << 5,
    INHIBIT_SENSOR_TASK_TIMEOUT = 1U << 6,
    INHIBIT_STEERING_SENSOR_NOT_FRESH = 1U << 7,
    INHIBIT_CONTROL_OUTPUT_TIMEOUT = 1U << 8,
    // Bits 9-11 reservados (mesma numeracao do SafetyStatus.msg do Odroid).
    INHIBIT_ENCODERS_DISAGREE = 1U << 12 // retida ate rearme
};

/**
 * Decide se o veiculo pode se mover (TaskSafety, a cada TASK_SAFETY_PERIOD_MS).
 *
 * O movimento so e permitido (STATE_READY) com TODAS as condicoes:
 * - enable fisico (pino) em nivel alto;
 * - enable logico recebido, true e com idade <= LOGICAL_ENABLE_WATCHDOG_TIMEOUT_MS;
 * - referencias de tracao e esterçamento recentes (*_REFERENCE_WATCHDOG_TIMEOUT_MS);
 * - TaskSensors viva (heartbeat <= SENSOR_TASK_WATCHDOG_TIMEOUT_MS);
 * - leitura do LVDT valida e recente (STEERING_SENSOR_SNAPSHOT_TIMEOUT_MS);
 * - saida do SteeringController valida e recente (CONTROL_OUTPUT_WATCHDOG_TIMEOUT_MS).
 * - encoders concordantes (EncoderPlausibility).
 * Cada condicao falsa gera uma SafetyInhibitFlag. Nas transicoes de estado
 * chama ActuatorManager::setMotionPermit(). As falhas de tempo nao sao
 * retidas: o estado volta a READY assim que as condicoes voltam. A falha de
 * encoders discordantes e retida ate o rearme (enable fisico desligado ou
 * enable logico false).
 */
class SafetyManager
{
public:
    struct Snapshot
    {
        SafetyState state;
        bool physical_enable;
        bool logical_enable;
        bool logical_enable_received;
        bool logical_enable_fresh;
        bool drive_reference_fresh;
        bool steering_reference_fresh;
        bool sensor_task_healthy;
        bool steering_sensor_fresh;
        bool control_output_fresh;
        bool encoders_disagree; // falha retida
        bool motion_permitted;
        uint32_t inhibit_flags;
        int64_t timestamp_us;
    };

    SafetyManager(uint8_t physical_enable_pin,
                  CommandMailbox &command_mailbox,
                  ActuatorManager &actuator_manager,
                  SteeringController &steering_controller);

    // Configura o pino de enable e parte em STATE_DISABLED.
    void begin();
    // Reavalia as condicoes e aplica a transicao de estado, se houver.
    void update();
    // Heartbeat da task de sensores, chamado ao fim de cada ciclo completo.
    // Registrado com os tres encoders validos; a validade e o instante da
    // leitura do LVDT sao guardados junto, para o watchdog do sensor.
    void reportSensorTaskCycle(
        const SteeringSensor::Snapshot &steering_snapshot,
        const EncoderReader::Snapshot &rear_snapshot,
        const EncoderReader::Snapshot &right_snapshot,
        const EncoderReader::Snapshot &left_snapshot);

    Snapshot getSnapshot() const;

private:
    static bool encoderSnapshotValid(const EncoderReader::Snapshot &snapshot);
    static bool isFresh(bool received,
                        int64_t received_at_us,
                        int64_t now_us,
                        int64_t timeout_us);
    void applyStateTransition(SafetyState previous_state, SafetyState new_state);

    uint8_t physical_enable_pin_;
    CommandMailbox &command_mailbox_;
    ActuatorManager &actuator_manager_;
    SteeringController &steering_controller_;

    struct SensorHealthSnapshot
    {
        int64_t initialized_at_us;
        int64_t heartbeat_us;
        bool steering_snapshot_valid;
        int64_t steering_snapshot_us;
        bool encoder_speeds_valid;
        float rear_velocity_m_s;
        float front_velocity_m_s; // media das dianteiras
    };

    SensorHealthSnapshot sensor_health_;
    EncoderPlausibility encoder_plausibility_;
    bool encoders_disagree_latched_;
    mutable portMUX_TYPE health_mux_ = portMUX_INITIALIZER_UNLOCKED;

    Snapshot snapshot_;
    mutable portMUX_TYPE snapshot_mux_ = portMUX_INITIALIZER_UNLOCKED;
};
