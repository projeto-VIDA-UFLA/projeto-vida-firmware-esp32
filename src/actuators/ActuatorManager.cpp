#include "actuators/ActuatorManager.h"

#include <cmath>
#include "esp_timer.h"
#include "utils/config.h"

ActuatorManager::ActuatorManager(DriveMotor &drive_motor,
                                 SteeringMotor &steering_motor)
    : drive_motor_(drive_motor),
      steering_motor_(steering_motor),
      mutex_(nullptr),
      motion_permitted_(false),
      emergency_latched_(false)
{
}

bool ActuatorManager::begin()
{
    if (mutex_ == nullptr)
    {
        mutex_ = xSemaphoreCreateMutex();
    }

    if (!takeMutex())
    {
        drive_motor_.stop();
        steering_motor_.stop();
        return false;
    }

    motion_permitted_ = false;
    emergency_latched_ = false;
    stopUnlocked();
    giveMutex();
    return true;
}

bool ActuatorManager::applyCommands(
    const TimedCommand &command,
    const DriveMotor::VelocityMeasurement &drive_velocity)
{
    if (!takeMutex())
    {
        return false;
    }

    const int64_t now_us = esp_timer_get_time();
    constexpr int64_t control_output_timeout_us =
        static_cast<int64_t>(CONTROL_OUTPUT_WATCHDOG_TIMEOUT_MS) * 1000LL;
    const bool output_is_fresh =
        command.produced_at_us > 0 &&
        now_us >= command.produced_at_us &&
        (now_us - command.produced_at_us) <= control_output_timeout_us;
    const bool commands_are_valid =
        command.valid &&
        output_is_fresh &&
        std::isfinite(command.drive_command) &&
        std::isfinite(command.steering_command) &&
        command.drive_command >= -1.0f && command.drive_command <= 1.0f &&
        command.steering_command >= -1.0f && command.steering_command <= 1.0f;

    if (!commands_are_valid || !motion_permitted_ || emergency_latched_)
    {
        stopUnlocked();
        giveMutex();
        return false;
    }

    drive_motor_.setCommand(command.drive_command, drive_velocity);
    steering_motor_.setCommand(command.steering_command);
    giveMutex();
    return true;
}

void ActuatorManager::setMotionPermit(bool permitted)
{
    if (!takeMutex())
    {
        return;
    }

    if (!permitted)
    {
        motion_permitted_ = false;
        stopUnlocked();
    }
    else if (!emergency_latched_)
    {
        motion_permitted_ = true;
    }

    giveMutex();
}

void ActuatorManager::stop()
{
    if (!takeMutex())
    {
        drive_motor_.stop();
        steering_motor_.stop();
        return;
    }

    stopUnlocked();
    giveMutex();
}

void ActuatorManager::emergencyStop()
{
    if (!takeMutex())
    {
        drive_motor_.stop();
        steering_motor_.stop();
        return;
    }

    emergency_latched_ = true;
    motion_permitted_ = false;
    stopUnlocked();
    giveMutex();
}

bool ActuatorManager::isMotionPermitted() const
{
    if (!takeMutex())
    {
        return false;
    }

    const bool motion_permitted = motion_permitted_;
    giveMutex();
    return motion_permitted;
}

bool ActuatorManager::isEmergencyLatched() const
{
    if (!takeMutex())
    {
        return false;
    }

    const bool emergency_latched = emergency_latched_;
    giveMutex();
    return emergency_latched;
}

ActuatorManager::Snapshot ActuatorManager::getSnapshot() const
{
    Snapshot snapshot{};

    if (!takeMutex())
    {
        snapshot.drive = drive_motor_.getSnapshot();
        snapshot.steering = steering_motor_.getSnapshot();
        snapshot.motion_permitted = false;
        snapshot.emergency_latched = false;
        return snapshot;
    }

    snapshot.drive = drive_motor_.getSnapshot();
    snapshot.steering = steering_motor_.getSnapshot();
    snapshot.motion_permitted = motion_permitted_;
    snapshot.emergency_latched = emergency_latched_;
    giveMutex();
    return snapshot;
}

bool ActuatorManager::takeMutex() const
{
    return mutex_ != nullptr && xSemaphoreTake(mutex_, portMAX_DELAY) == pdTRUE;
}

void ActuatorManager::giveMutex() const
{
    (void)xSemaphoreGive(mutex_);
}

void ActuatorManager::stopUnlocked()
{
    drive_motor_.stop();
    steering_motor_.stop();
}
