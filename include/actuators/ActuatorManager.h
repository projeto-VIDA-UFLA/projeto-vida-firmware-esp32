#pragma once

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "actuators/BrakeServo.h"
#include "actuators/DriveMotor.h"
#include "actuators/SteeringMotor.h"

class ActuatorManager
{
public:
    struct TimedCommand
    {
        float drive_command;
        float steering_command;
        float brake_command;
        int64_t produced_at_us;
        bool valid;
    };

    struct Snapshot
    {
        DriveMotor::Snapshot drive;
        SteeringMotor::Snapshot steering;
        BrakeServo::Snapshot brake;
        bool motion_permitted;
        bool emergency_latched;
    };

    ActuatorManager(DriveMotor &drive_motor, SteeringMotor &steering_motor,
                    BrakeServo &brake_servo);

    bool begin();

    bool applyCommands(
        const TimedCommand &command,
        const DriveMotor::VelocityMeasurement &drive_velocity);
    void setMotionPermit(bool permitted);
    void stop();
    void emergencyStop();

    // Avanca a rampa de falha do freio e o alivio de veiculo parado.
    // Chamado periodicamente pela task de seguranca para nao depender da
    // task de controle.
    void updateBrake(bool vehicle_stopped);

    bool isMotionPermitted() const;
    bool isEmergencyLatched() const;
    Snapshot getSnapshot() const;

private:
    bool takeMutex() const;
    void giveMutex() const;
    void stopUnlocked();

    DriveMotor &drive_motor_;
    SteeringMotor &steering_motor_;
    BrakeServo &brake_servo_;

    mutable SemaphoreHandle_t mutex_;
    bool motion_permitted_;
    bool emergency_latched_;
};
