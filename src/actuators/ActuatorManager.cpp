#include "actuators/ActuatorManager.h"

#include <cmath>
#include "esp_timer.h"
#include "utils/config.h"

ActuatorManager::ActuatorManager(DriveMotor &drive_motor,
                                 SteeringMotor &steering_motor,
                                 BrakeServo &brake_servo)
    : drive_motor_(drive_motor),
      steering_motor_(steering_motor),
      brake_servo_(brake_servo),
      mutex_(nullptr),
      motion_permitted_(false),
      emergency_latched_(false),
      maneuver_brake_release_(false),
      supervisor_tripped_(false)
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
        brake_servo_.applyFaultRamp();
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
        std::isfinite(command.brake_command) &&
        command.drive_command >= -1.0f && command.drive_command <= 1.0f &&
        command.steering_command >= -1.0f && command.steering_command <= 1.0f &&
        command.brake_command >= 0.0f && command.brake_command <= 1.0f;

    if (supervisor_tripped_)
    {
        holdSupervisorStop();
        giveMutex();
        return false;
    }

    if (maneuver_brake_release_ && !emergency_latched_)
    {
        // Manobra: freio solto, tracao parada; direcao so com permissao.
        drive_motor_.stop();
        if (commands_are_valid && motion_permitted_)
        {
            steering_motor_.setCommand(command.steering_command);
        }
        else
        {
            steering_motor_.stop();
        }
        brake_servo_.setCommand(0.0f);
        giveMutex();
        return commands_are_valid && motion_permitted_;
    }

    if (!commands_are_valid || !motion_permitted_ || emergency_latched_)
    {
        stopUnlocked();
        giveMutex();
        return false;
    }

    brake_servo_.setCommand(command.brake_command);
    steering_motor_.setCommand(command.steering_command);

    // Intertravamento: com o freio acionado a tracao nao briga com ele.
    if (brake_servo_.getPosition() > BRAKE_DRIVE_CUTOFF)
    {
        drive_motor_.stop();
    }
    else
    {
        drive_motor_.setCommand(command.drive_command, drive_velocity);
    }

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
    else if (!emergency_latched_ && !supervisor_tripped_)
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
        brake_servo_.applyFaultRamp();
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
        brake_servo_.applyFaultRamp();
        return;
    }

    emergency_latched_ = true;
    motion_permitted_ = false;
    stopUnlocked();
    giveMutex();
}

void ActuatorManager::setManeuverBrakeRelease(bool release)
{
    if (!takeMutex())
    {
        return;
    }

    const bool was_released = maneuver_brake_release_;
    maneuver_brake_release_ = release && !emergency_latched_ && !supervisor_tripped_;
    if (was_released && !maneuver_brake_release_)
    {
        // Saiu da manobra: freio de volta ja (a task de controle reassume).
        drive_motor_.stop();
        brake_servo_.setCommand(1.0f);
    }
    giveMutex();
}

bool ActuatorManager::isManeuverBrakeRelease() const
{
    if (!takeMutex())
    {
        return false;
    }

    const bool release = maneuver_brake_release_;
    giveMutex();
    return release;
}

void ActuatorManager::updateBrake(bool vehicle_stopped)
{
    if (!takeMutex())
    {
        return;
    }

    if (supervisor_tripped_)
    {
        holdSupervisorStop();
    }
    brake_servo_.update(vehicle_stopped);
    giveMutex();
}

void ActuatorManager::forceSafeOutputs()
{
    supervisor_tripped_ = true;
    drive_motor_.forceOutputOff();
    steering_motor_.forceOutputOff();
    brake_servo_.forceFullBrake();
}

bool ActuatorManager::isSupervisorTripped() const
{
    return supervisor_tripped_;
}

void ActuatorManager::holdSupervisorStop()
{
    // Mesmo estado que o supervisor escreve no hardware: motores em 0 e
    // freio total (sem rampa).
    motion_permitted_ = false;
    maneuver_brake_release_ = false;
    drive_motor_.stop();
    steering_motor_.stop();
    brake_servo_.setCommand(1.0f);
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
        snapshot.brake = brake_servo_.getSnapshot();
        snapshot.motion_permitted = false;
        snapshot.emergency_latched = false;
        snapshot.supervisor_tripped = supervisor_tripped_;
        return snapshot;
    }

    snapshot.drive = drive_motor_.getSnapshot();
    snapshot.steering = steering_motor_.getSnapshot();
    snapshot.brake = brake_servo_.getSnapshot();
    snapshot.motion_permitted = motion_permitted_;
    snapshot.emergency_latched = emergency_latched_;
    snapshot.supervisor_tripped = supervisor_tripped_;
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
    // Parar motores zera PWM; parar o freio significa aplica-lo em rampa
    // (exceto no modo manobra, em que ele fica solto).
    drive_motor_.stop();
    steering_motor_.stop();
    if (maneuver_brake_release_ && !emergency_latched_ && !supervisor_tripped_)
    {
        brake_servo_.setCommand(0.0f);
    }
    else
    {
        brake_servo_.applyFaultRamp();
    }
}
