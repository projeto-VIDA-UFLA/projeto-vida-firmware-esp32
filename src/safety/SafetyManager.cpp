#include "safety/SafetyManager.h"

#include "esp_timer.h"

SafetyManager::SafetyManager(uint8_t physical_enable_pin,
                             CommandMailbox &command_mailbox,
                             ActuatorManager &actuator_manager)
    : physical_enable_pin_(physical_enable_pin),
      command_mailbox_(command_mailbox),
      actuator_manager_(actuator_manager),
      snapshot_{SafetyState::STATE_DISABLED, false, false, false, false, 0}
{
}

bool SafetyManager::begin()
{
    pinMode(physical_enable_pin_, INPUT);

    // A inicializacao e sempre bloqueada, independentemente do nivel fisico.
    actuator_manager_.setMotionPermit(false);

    const bool physical_enable = digitalRead(physical_enable_pin_) == HIGH;
    const Snapshot initial_snapshot{
        SafetyState::STATE_DISABLED,
        physical_enable,
        false,
        false,
        false,
        esp_timer_get_time()};

    portENTER_CRITICAL(&snapshot_mux_);
    snapshot_ = initial_snapshot;
    portEXIT_CRITICAL(&snapshot_mux_);

    return true;
}

void SafetyManager::update()
{
    const bool physical_enable = digitalRead(physical_enable_pin_) == HIGH;
    const CommandMailbox::LogicalEnableSnapshot logical_enable =
        command_mailbox_.getLogicalEnableSnapshot();

    // TODO: Antes da TaskControl, adicionar timeout do enable logico e das
    // referencias, supervisao ROS e validacao de referencias e sensores.
    const bool motion_permitted =
        physical_enable && logical_enable.received && logical_enable.enabled;
    const SafetyState new_state = motion_permitted
                                      ? SafetyState::STATE_READY
                                      : SafetyState::STATE_DISABLED;

    SafetyState previous_state;
    portENTER_CRITICAL(&snapshot_mux_);
    previous_state = snapshot_.state;
    portEXIT_CRITICAL(&snapshot_mux_);

    if (new_state != previous_state)
    {
        applyStateTransition(previous_state, new_state);
    }

    const Snapshot new_snapshot{
        new_state,
        physical_enable,
        logical_enable.enabled,
        logical_enable.received,
        motion_permitted,
        esp_timer_get_time()};

    portENTER_CRITICAL(&snapshot_mux_);
    snapshot_ = new_snapshot;
    portEXIT_CRITICAL(&snapshot_mux_);
}

SafetyManager::Snapshot SafetyManager::getSnapshot() const
{
    portENTER_CRITICAL(&snapshot_mux_);
    const Snapshot snapshot = snapshot_;
    portEXIT_CRITICAL(&snapshot_mux_);
    return snapshot;
}

void SafetyManager::applyStateTransition(SafetyState previous_state,
                                         SafetyState new_state)
{
    (void)previous_state;
    actuator_manager_.setMotionPermit(new_state == SafetyState::STATE_READY);
}
