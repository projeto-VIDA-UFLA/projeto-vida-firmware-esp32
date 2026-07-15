#pragma once

#include <Arduino.h>

#include "actuators/ActuatorManager.h"
#include "comunication/CommandMailbox.h"

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
    INHIBIT_LOGICAL_ENABLE_TIMEOUT = 1U << 3
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
        bool motion_permitted;
        uint32_t inhibit_flags;
        int64_t timestamp_us;
    };

    SafetyManager(uint8_t physical_enable_pin, CommandMailbox &command_mailbox, ActuatorManager &actuator_manager);

    bool begin();
    void update();

    Snapshot getSnapshot() const;

private:
    void applyStateTransition(SafetyState previous_state, SafetyState new_state);

    uint8_t physical_enable_pin_;
    CommandMailbox &command_mailbox_;
    ActuatorManager &actuator_manager_;

    Snapshot snapshot_;
    mutable portMUX_TYPE snapshot_mux_ = portMUX_INITIALIZER_UNLOCKED;
};
