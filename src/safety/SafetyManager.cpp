#include "safety/SafetyManager.h"

#include "utils/config.h"
#include "esp_timer.h"

SafetyManager::SafetyManager(uint8_t physical_enable_pin,
                             CommandMailbox &command_mailbox,
                             ActuatorManager &actuator_manager)
    : physical_enable_pin_(physical_enable_pin),
      command_mailbox_(command_mailbox),
      actuator_manager_(actuator_manager),
      snapshot_()
{
    snapshot_.state = SafetyState::STATE_DISABLED;
    snapshot_.physical_enable = false;
    snapshot_.logical_enable = false;
    snapshot_.logical_enable_received = false;
    snapshot_.logical_enable_fresh = false;
    snapshot_.drive_reference_fresh = false;
    snapshot_.steering_reference_fresh = false;
    snapshot_.motion_permitted = false;
    snapshot_.inhibit_flags =
        INHIBIT_PHYSICAL_ENABLE_OFF |
        INHIBIT_LOGICAL_ENABLE_NOT_RECEIVED |
        INHIBIT_DRIVE_REFERENCE_NOT_FRESH |
        INHIBIT_STEERING_REFERENCE_NOT_FRESH;
    snapshot_.timestamp_us = 0;
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
        false,
        false,
        false,
        (physical_enable ? INHIBIT_NONE : INHIBIT_PHYSICAL_ENABLE_OFF) |
            INHIBIT_LOGICAL_ENABLE_NOT_RECEIVED |
            INHIBIT_DRIVE_REFERENCE_NOT_FRESH |
            INHIBIT_STEERING_REFERENCE_NOT_FRESH,
        esp_timer_get_time()};

    portENTER_CRITICAL(&snapshot_mux_);
    snapshot_ = initial_snapshot;
    portEXIT_CRITICAL(&snapshot_mux_);

    return true;
}

void SafetyManager::update()
{
    constexpr int64_t logical_enable_timeout_us =
        static_cast<int64_t>(LOGICAL_ENABLE_WATCHDOG_TIMEOUT_MS) * 1000LL;
    constexpr int64_t drive_reference_timeout_us =
        static_cast<int64_t>(DRIVE_REFERENCE_WATCHDOG_TIMEOUT_MS) * 1000LL;
    constexpr int64_t steering_reference_timeout_us =
        static_cast<int64_t>(STEERING_REFERENCE_WATCHDOG_TIMEOUT_MS) * 1000LL;

    const int64_t now_us = esp_timer_get_time();
    const bool physical_enable = digitalRead(physical_enable_pin_) == HIGH;
    const CommandMailbox::LogicalEnableSnapshot logical_enable =
        command_mailbox_.getLogicalEnableSnapshot();
    const CommandMailbox::DriveReferenceSnapshot drive_reference =
        command_mailbox_.getDriveReferenceSnapshot();
    const CommandMailbox::SteeringReferenceSnapshot steering_reference =
        command_mailbox_.getSteeringReferenceSnapshot();

    const bool logical_enable_fresh = isFresh(logical_enable.received,
                                              logical_enable.received_at_us,
                                              now_us,
                                              logical_enable_timeout_us);
    const bool drive_reference_fresh = isFresh(drive_reference.received,
                                               drive_reference.received_at_us,
                                               now_us,
                                               drive_reference_timeout_us);
    const bool steering_reference_fresh = isFresh(steering_reference.received,
                                                  steering_reference.received_at_us,
                                                  now_us,
                                                  steering_reference_timeout_us);

    uint32_t inhibit_flags = INHIBIT_NONE;
    if (!physical_enable)
    {
        inhibit_flags |= INHIBIT_PHYSICAL_ENABLE_OFF;
    }

    if (!logical_enable.received)
    {
        inhibit_flags |= INHIBIT_LOGICAL_ENABLE_NOT_RECEIVED;
    }
    else
    {
        if (!logical_enable.enabled)
        {
            inhibit_flags |= INHIBIT_LOGICAL_ENABLE_FALSE;
        }

        if (!logical_enable_fresh)
        {
            inhibit_flags |= INHIBIT_LOGICAL_ENABLE_TIMEOUT;
        }
    }

    if (!drive_reference_fresh)
    {
        inhibit_flags |= INHIBIT_DRIVE_REFERENCE_NOT_FRESH;
    }

    if (!steering_reference_fresh)
    {
        inhibit_flags |= INHIBIT_STEERING_REFERENCE_NOT_FRESH;
    }

    const bool motion_permitted =
        physical_enable &&
        logical_enable.received &&
        logical_enable_fresh &&
        logical_enable.enabled &&
        drive_reference_fresh &&
        steering_reference_fresh;
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
        logical_enable_fresh,
        drive_reference_fresh,
        steering_reference_fresh,
        motion_permitted,
        inhibit_flags,
        now_us};

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

bool SafetyManager::isFresh(bool received,
                            int64_t received_at_us,
                            int64_t now_us,
                            int64_t timeout_us)
{
    return received &&
           received_at_us > 0 &&
           now_us >= received_at_us &&
           (now_us - received_at_us) <= timeout_us;
}

void SafetyManager::applyStateTransition(SafetyState previous_state,
                                         SafetyState new_state)
{
    (void)previous_state;
    actuator_manager_.setMotionPermit(new_state == SafetyState::STATE_READY);
}
