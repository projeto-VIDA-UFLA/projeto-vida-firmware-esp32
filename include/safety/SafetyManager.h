#pragma once

#include <Arduino.h>

#include "actuators/ActuatorManager.h"
#include "comunication/CommandMailbox.h"
#include "control/SteeringController.h"
#include "sensors/SteeringSensor.h"

enum class SafetyState : uint8_t
{
    STATE_DISABLED = 0,
    STATE_READY = 1
};

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
    INHIBIT_CONTROL_OUTPUT_TIMEOUT = 1U << 8
};

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
        bool motion_permitted;
        uint32_t inhibit_flags;
        int64_t timestamp_us;
    };

    SafetyManager(uint8_t physical_enable_pin,
                  CommandMailbox &command_mailbox,
                  ActuatorManager &actuator_manager,
                  SteeringSensor &steering_sensor,
                  SteeringController &steering_controller);

    void begin();
    void update();
    void reportSensorTaskCycle(bool cycle_valid,
                               int64_t steering_snapshot_timestamp_us);

    Snapshot getSnapshot() const;

private:
    static bool isFresh(bool received,
                        int64_t received_at_us,
                        int64_t now_us,
                        int64_t timeout_us);
    void applyStateTransition(SafetyState previous_state, SafetyState new_state);

    uint8_t physical_enable_pin_;
    CommandMailbox &command_mailbox_;
    ActuatorManager &actuator_manager_;
    SteeringSensor &steering_sensor_;
    SteeringController &steering_controller_;

    int64_t initialized_at_us_;
    int64_t sensor_task_heartbeat_us_;
    int64_t reported_steering_snapshot_us_;
    mutable portMUX_TYPE health_mux_ = portMUX_INITIALIZER_UNLOCKED;

    Snapshot snapshot_;
    mutable portMUX_TYPE snapshot_mux_ = portMUX_INITIALIZER_UNLOCKED;
};
