#pragma once

#include <Arduino.h>

#include "comunication/CommandMailbox.h"
#include "comunication/RosCommunication.h"
#include "sensors/EncoderReader.h"
#include "sensors/SteeringSensor.h"
#include "actuators/SteeringMotor.h"
#include "actuators/DriveMotor.h"

#if defined(__INTELLISENSE__) || defined(__clangd__)
#include "microros_intellisense_stub.h"
#else
#include <vida_interfaces/msg/encoders.h>
#include <vida_interfaces/msg/steering.h>
#include <vida_interfaces/msg/comands.h>
#include <vida_interfaces/msg/steering_reference.h>
#include <vida_interfaces/msg/drive_reference.h>
#include <vida_interfaces/msg/logical_enable.h>
#endif

class VidaRosNode
{
public:
    VidaRosNode(SteeringMotor &steering_motor,
                DriveMotor &drive_motor,
                SteeringSensor &steering_sensor,
                EncoderReader &rear_encoder,
                EncoderReader &right_encoder,
                EncoderReader &left_encoder,
                CommandMailbox &command_mailbox);

    bool begin(Stream &transport);

private:
    static void publishSensorsCallback(rcl_timer_t *timer, int64_t last_call_time);
    static void steeringReferenceCallback(const void *message);
    static void driveReferenceCallback(const void *message);
    static void logicalEnableCallback(const void *message);

    bool configure();
    void publishSensors(rcl_timer_t *timer);
    bool setTimestamp(builtin_interfaces__msg__Time &timestamp, int64_t local_time_us);
    bool fillEncoderMeasurement(vida_interfaces__msg__EncoderMeasurement &message,
                                const EncoderReader::Snapshot &snapshot);

    static VidaRosNode *instance_;

    RosCommunication communication_;

    rcl_publisher_t encoders_publisher_;
    rcl_publisher_t steering_publisher_;
    rcl_publisher_t cmd_steering_publisher_;
    rcl_publisher_t cmd_drive_publisher_;

    rcl_subscription_t steering_reference_subscription_;
    rcl_subscription_t drive_reference_subscription_;
    rcl_subscription_t logical_enable_subscription_;

    vida_interfaces__msg__Encoders encoders_message_;
    vida_interfaces__msg__Steering steering_message_;
    vida_interfaces__msg__Comands cmd_steering_message_;
    vida_interfaces__msg__Comands cmd_drive_message_;

    vida_interfaces__msg__SteeringReference steering_reference_message_;
    vida_interfaces__msg__DriveReference drive_reference_message_;
    vida_interfaces__msg__LogicalEnable logical_enable_message_;

    rcl_timer_t sensors_publish_timer_;

    SteeringMotor &steering_motor_;
    DriveMotor &drive_motor_;

    SteeringSensor &steering_sensor_;
    EncoderReader &rear_encoder_;
    EncoderReader &right_encoder_;
    EncoderReader &left_encoder_;
    CommandMailbox &command_mailbox_;
};
