#pragma once

#include <Arduino.h>

#include "comunication/CommandMailbox.h"
#include "comunication/RosCommunication.h"
#include "actuators/ActuatorManager.h"
#include "control/SteeringController.h"
#include "safety/SafetyManager.h"
#include "sensors/EncoderReader.h"
#include "sensors/SteeringSensor.h"

#if defined(__INTELLISENSE__) || defined(__clangd__)
#include "microros_intellisense_stub.h"
#else
#include <vida_interfaces/msg/encoders.h>
#include <vida_interfaces/msg/steering.h>
#include <vida_interfaces/msg/comands.h>
#include <vida_interfaces/msg/steering_reference.h>
#include <vida_interfaces/msg/drive_reference.h>
#include <vida_interfaces/msg/logical_enable.h>
#include <vida_interfaces/msg/safety_status.h>
#include <vida_interfaces/msg/steering_controller_output.h>
#endif

class VidaRosNode
{
public:
    VidaRosNode(EncoderReader &rear_encoder,
                EncoderReader &right_encoder,
                EncoderReader &left_encoder,
                SteeringSensor &steering_sensor,
                SafetyManager &safety_manager,
                ActuatorManager &actuator_manager,
                CommandMailbox &command_mailbox,
                SteeringController &steering_controller);

    bool begin(Stream &transport);

private:
    static void publishSensorsCallback(rcl_timer_t *timer, int64_t last_call_time);
    static void publishSteeringOutputCallback(rcl_timer_t *timer, int64_t last_call_time);
    static void steeringReferenceCallback(const void *message);
    static void driveReferenceCallback(const void *message);
    static void logicalEnableCallback(const void *message);

    bool configure();
    void publishSensors(rcl_timer_t *timer);

    bool setTimestamp(builtin_interfaces__msg__Time &timestamp, int64_t local_time_us);
    bool fillEncoderMeasurement(vida_interfaces__msg__EncoderMeasurement &message, const EncoderReader::Snapshot &snapshot);

    static VidaRosNode *instance_;

    RosCommunication communication_;

    // Publisher de sensores
    rcl_publisher_t encoders_publisher_;
    rcl_publisher_t steering_publisher_;

    // Publisher de atuadores
    rcl_publisher_t cmd_drive_publisher_;
    rcl_publisher_t cmd_steering_publisher_;
    rcl_publisher_t cmd_brake_publisher_;

    // Publisher de segurança
    rcl_publisher_t safety_status_publisher_;

    // Publisher de controle
    rcl_publisher_t steering_output_publisher_;
    rcl_publisher_t drive_output_publisher_;

    // Subscription de controle
    rcl_subscription_t steering_reference_subscription_;
    rcl_subscription_t drive_reference_subscription_;

    // Subscription de Segurança
    rcl_subscription_t logical_enable_subscription_;

    // Mensagens
    vida_interfaces__msg__Encoders encoders_message_;
    vida_interfaces__msg__Steering steering_message_;
    vida_interfaces__msg__Comands cmd_steering_message_;
    vida_interfaces__msg__Comands cmd_drive_message_;
    vida_interfaces__msg__Comands cmd_brake_message_;
    vida_interfaces__msg__SafetyStatus safety_status_message_;
    vida_interfaces__msg__SteeringReference steering_reference_message_;
    vida_interfaces__msg__DriveReference drive_reference_message_;
    vida_interfaces__msg__LogicalEnable logical_enable_message_;
    vida_interfaces__msg__SteeringControllerOutput steering_output_message_;

    // Timer
    rcl_timer_t publish_timer_;

    // Sensores
    SteeringSensor &steering_sensor_;
    EncoderReader &rear_encoder_;
    EncoderReader &right_encoder_;
    EncoderReader &left_encoder_;

    // Atudarores e segurança
    ActuatorManager &actuator_manager_;
    SafetyManager &safety_manager_;

    // Comandos recebidos
    CommandMailbox &command_mailbox_;

    // Atudarores e segurança
    SteeringController &steering_controller_;
};
