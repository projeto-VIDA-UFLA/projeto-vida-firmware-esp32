#pragma once

#include <Arduino.h>

#include "actuators/ActuatorManager.h"
#include "comunication/CommandMailbox.h"

enum class SafetyState : uint8_t
{
    STATE_DISABLED,
    STATE_READY
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
        bool motion_permitted;
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
