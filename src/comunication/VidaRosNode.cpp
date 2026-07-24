#include "comunication/VidaRosNode.h"
#include "utils/config.h"

#include <cmath>
#include "esp_timer.h"

namespace
{
    constexpr char NODE_NAME[] = "esp32_controller";
    constexpr char NODE_NAMESPACE[] = "/vida";
    constexpr int ROS_DOMAIN_ID = 0;

    // Tópicos de sensores
    constexpr char ENCODERS_TOPIC[] = "/vida/sensors/encoders";
    constexpr char STEERING_TOPIC[] = "/vida/sensors/steering";

    // tópicos de atuadores
    constexpr char CMD_STEERING_TOPIC[] = "/vida/actuators/steering";
    constexpr char CMD_DRIVE_TOPIC[] = "/vida/actuators/drive";

    // tópicos de segurança
    constexpr char SAFETY_STATUS_TOPIC[] = "/vida/safety/status";
    constexpr char LOGICAL_ENABLE_TOPIC[] = "/vida/safety/logical_enable";

    // tópicos de controle
    constexpr char STEERING_REFERENCE_TOPIC[] = "/vida/control/steering/reference";
    constexpr char STEERING_OUTPUT_TOPIC[] = "/vida/control/steering/output";
    constexpr char DRIVE_REFERENCE_TOPIC[] = "/vida/control/drive/reference";
    constexpr char DRIVE_OUTPUT_TOPIC[] = "/vida/control/drive/output";

    int64_t timestampToNanoseconds(const builtin_interfaces__msg__Time &timestamp)
    {
        return static_cast<int64_t>(timestamp.sec) * 1000000000LL +
               static_cast<int64_t>(timestamp.nanosec);
    }
} // namespace

VidaRosNode *VidaRosNode::instance_ = nullptr;

VidaRosNode::VidaRosNode(EncoderReader &rear_encoder,
                         EncoderReader &right_encoder,
                         EncoderReader &left_encoder,
                         SteeringSensor &steering_sensor,
                         SafetyManager &safety_manager,
                         ActuatorManager &actuator_manager,
                         CommandMailbox &command_mailbox,
                         SteeringController &steering_controller)
    : communication_(NODE_NAME, NODE_NAMESPACE, ROS_DOMAIN_ID),
      encoders_publisher_(rcl_get_zero_initialized_publisher()),
      steering_publisher_(rcl_get_zero_initialized_publisher()),
      cmd_steering_publisher_(rcl_get_zero_initialized_publisher()),
      cmd_drive_publisher_(rcl_get_zero_initialized_publisher()),
      safety_status_publisher_(rcl_get_zero_initialized_publisher()),
      steering_reference_subscription_(rcl_get_zero_initialized_subscription()),
      drive_reference_subscription_(rcl_get_zero_initialized_subscription()),
      logical_enable_subscription_(rcl_get_zero_initialized_subscription()),
      steering_output_publisher_(rcl_get_zero_initialized_publisher()),
      drive_output_publisher_(rcl_get_zero_initialized_publisher()),
      encoders_message_{},
      steering_message_{},
      steering_output_message_{},
      cmd_drive_message_{},
      safety_status_message_{},
      steering_reference_message_{},
      drive_reference_message_{},
      logical_enable_message_{},
      publish_timer_(rcl_get_zero_initialized_timer()),
      actuator_manager_(actuator_manager),
      safety_manager_(safety_manager),
      steering_sensor_(steering_sensor),
      rear_encoder_(rear_encoder),
      right_encoder_(right_encoder),
      left_encoder_(left_encoder),
      command_mailbox_(command_mailbox),
      steering_controller_(steering_controller)
{
}

bool VidaRosNode::begin(Stream &transport)
{
    if (instance_ != nullptr && instance_ != this)
    {
        return false;
    }

    instance_ = this;

    if (!configure())
    {
        instance_ = nullptr;
        return false;
    }

    communication_.begin(transport);

    if (!communication_.startTask("TaskRosCommunication", 8192, 2, 0))
    {
        instance_ = nullptr;
        return false;
    }

    return true;
}

bool VidaRosNode::configure()
{
    if (!communication_.addPublisher(
            encoders_publisher_,
            ROSIDL_GET_MSG_TYPE_SUPPORT(vida_interfaces, msg, Encoders),
            ENCODERS_TOPIC))
    {
        return false;
    }

    if (!communication_.addPublisher(
            steering_publisher_,
            ROSIDL_GET_MSG_TYPE_SUPPORT(vida_interfaces, msg, Steering),
            STEERING_TOPIC))
    {
        return false;
    }

    if (!communication_.addPublisher(
            cmd_steering_publisher_,
            ROSIDL_GET_MSG_TYPE_SUPPORT(vida_interfaces, msg, Comands),
            CMD_STEERING_TOPIC))
    {
        return false;
    }

    if (!communication_.addPublisher(
            cmd_drive_publisher_,
            ROSIDL_GET_MSG_TYPE_SUPPORT(vida_interfaces, msg, Comands),
            CMD_DRIVE_TOPIC))
    {
        return false;
    }

    if (!communication_.addPublisher(
            safety_status_publisher_,
            ROSIDL_GET_MSG_TYPE_SUPPORT(vida_interfaces, msg, SafetyStatus),
            SAFETY_STATUS_TOPIC))
    {
        return false;
    }

    if (!communication_.addSubscription(
            logical_enable_subscription_,
            ROSIDL_GET_MSG_TYPE_SUPPORT(vida_interfaces, msg, LogicalEnable),
            LOGICAL_ENABLE_TOPIC,
            &logical_enable_message_,
            logicalEnableCallback))
    {
        return false;
    }

    if (!communication_.addSubscription(
            steering_reference_subscription_,
            ROSIDL_GET_MSG_TYPE_SUPPORT(vida_interfaces, msg, SteeringReference),
            STEERING_REFERENCE_TOPIC,
            &steering_reference_message_,
            steeringReferenceCallback))
    {
        return false;
    }

    if (!communication_.addSubscription(
            drive_reference_subscription_,
            ROSIDL_GET_MSG_TYPE_SUPPORT(vida_interfaces, msg, DriveReference),
            DRIVE_REFERENCE_TOPIC,
            &drive_reference_message_,
            driveReferenceCallback))
    {
        return false;
    }

    if (!communication_.addPublisher(
            steering_output_publisher_,
            ROSIDL_GET_MSG_TYPE_SUPPORT(vida_interfaces, msg, SteeringControllerOutput),
            STEERING_OUTPUT_TOPIC))
    {
        return false;
    }

    if (!communication_.addPublisher(
            steering_output_publisher_,
            ROSIDL_GET_MSG_TYPE_SUPPORT(vida_interfaces, msg, DriveControllerOutput),
            DRIVE_OUTPUT_TOPIC))
    {
        return false;
    }

    return communication_.addTimer(
        publish_timer_,
        TASK_COMUNICATION_PERIOD_MS,
        publishSensorsCallback);
}

void VidaRosNode::steeringReferenceCallback(const void *message)
{
    if (instance_ == nullptr || message == nullptr)
    {
        return;
    }

    const auto *reference = static_cast<const vida_interfaces__msg__SteeringReference *>(message);

    if (!std::isfinite(reference->angle_deg) ||
        !std::isfinite(reference->angular_velocity_deg_s))
    {
        return;
    }

    instance_->command_mailbox_.updateSteeringReference(
        reference->angle_deg,
        reference->angular_velocity_deg_s,
        timestampToNanoseconds(reference->timestamp));
}

void VidaRosNode::driveReferenceCallback(const void *message)
{
    if (instance_ == nullptr || message == nullptr)
    {
        return;
    }

    const auto *reference =
        static_cast<const vida_interfaces__msg__DriveReference *>(message);

    if (!std::isfinite(reference->velocity_m_s) ||
        !std::isfinite(reference->acceleration_m_s2))
    {
        return;
    }

    instance_->command_mailbox_.updateDriveReference(
        reference->velocity_m_s,
        reference->acceleration_m_s2,
        timestampToNanoseconds(reference->timestamp));
}

void VidaRosNode::logicalEnableCallback(const void *message)
{
    if (instance_ == nullptr || message == nullptr)
    {
        return;
    }

    const auto *logical_enable =
        static_cast<const vida_interfaces__msg__LogicalEnable *>(message);

    instance_->command_mailbox_.updateLogicalEnable(
        logical_enable->enabled,
        timestampToNanoseconds(logical_enable->timestamp));
}

void VidaRosNode::publishSensorsCallback(rcl_timer_t *timer, int64_t last_call_time)
{
    (void)last_call_time;

    if (instance_ != nullptr)
    {
        instance_->publishSensors(timer);
    }
}

void VidaRosNode::publishSensors(rcl_timer_t *timer)
{
    if (timer == nullptr)
    {
        return;
    }

    const ActuatorManager::Snapshot actuators = actuator_manager_.getSnapshot();
    const SteeringMotor::Snapshot &steering_pwm = actuators.steering;
    const DriveMotor::Snapshot &drive_pwm = actuators.drive;

    const EncoderReader::Snapshot left = left_encoder_.getMeasurementSnapshot();
    const EncoderReader::Snapshot right = right_encoder_.getMeasurementSnapshot();
    const EncoderReader::Snapshot rear = rear_encoder_.getMeasurementSnapshot();
    const SteeringSensor::Snapshot steering = steering_sensor_.getSnapshot();
    const SafetyManager::Snapshot safety = safety_manager_.getSnapshot();

    const SteeringController::Snapshot steering_controller = steering_controller_.getLastOutput();

    const bool encoders_valid =
        fillEncoderMeasurement(encoders_message_.left, left) &&
        fillEncoderMeasurement(encoders_message_.right, right) &&
        fillEncoderMeasurement(encoders_message_.rear, rear);

    if (encoders_valid)
    {
        communication_.publish(encoders_publisher_, &encoders_message_);
    }

    if (setTimestamp(steering_message_.timestamp, steering.timestamp_us))
    {
        steering_message_.raw_adc = steering.raw_adc;
        steering_message_.raw_angle_deg = steering.raw_angle_deg;
        steering_message_.filtered_angle_deg = steering.filtered_angle_deg;
        steering_message_.filtered_angular_velocity_deg_s = steering.filtered_angular_velocity_deg_s;
        communication_.publish(steering_publisher_, &steering_message_);
    }

    if (setTimestamp(cmd_steering_message_.timestamp, steering_pwm.timestamp_us))
    {
        cmd_steering_message_.pwm = steering_pwm.right_pwm - steering_pwm.left_pwm;
        communication_.publish(cmd_steering_publisher_, &cmd_steering_message_);
    }

    if (setTimestamp(cmd_drive_message_.timestamp, drive_pwm.timestamp_us))
    {
        cmd_drive_message_.pwm = !drive_pwm.reverse ? drive_pwm.pwm : -drive_pwm.pwm;
        communication_.publish(cmd_drive_publisher_, &cmd_drive_message_);
    }

    if (setTimestamp(safety_status_message_.timestamp, safety.timestamp_us))
    {
        safety_status_message_.state = static_cast<uint8_t>(safety.state);
        safety_status_message_.physical_enable = safety.physical_enable;
        safety_status_message_.logical_enable = safety.logical_enable;
        safety_status_message_.logical_enable_received = safety.logical_enable_received;
        safety_status_message_.logical_enable_fresh = safety.logical_enable_fresh;
        safety_status_message_.drive_reference_fresh = safety.drive_reference_fresh;
        safety_status_message_.steering_reference_fresh = safety.steering_reference_fresh;
        safety_status_message_.motion_permitted = safety.motion_permitted;
        safety_status_message_.inhibit_flags = safety.inhibit_flags;
        communication_.publish(safety_status_publisher_, &safety_status_message_);
    }

    if (!setTimestamp(steering_output_message_.timestamp, steering_controller.timestamp_us))
    {
        steering_output_message_.target_angle_deg = steering_controller.output.target_angle_deg;
        steering_output_message_.measured_angle_deg = steering_controller.output.measured_angle_deg;
        steering_output_message_.angle_error_deg = steering_controller.output.angle_error_deg;
        steering_output_message_.proportional_term = steering_controller.output.proportional_term;
        steering_output_message_.integral_term = steering_controller.output.integral_term;
        steering_output_message_.derivative_term = steering_controller.output.derivative_term;
        steering_output_message_.normalized_command = steering_controller.output.normalized_command;
        steering_output_message_.inside_deadband = steering_controller.output.inside_deadband;
        steering_output_message_.saturated = steering_controller.output.saturated;
        steering_output_message_.target_clamped = steering_controller.output.target_clamped;
        steering_output_message_.position_limit_active = steering_controller.output.position_limit_active;
        steering_output_message_.valid = steering_controller.output.valid;

        communication_.publish(steering_output_publisher_, &steering_output_message_);
    }
}

bool VidaRosNode::setTimestamp(builtin_interfaces__msg__Time &timestamp, int64_t local_time_us)
{
    int64_t epoch_ns = 0;
    if (!communication_.localTimeToEpochNanos(local_time_us, epoch_ns))
    {
        return false;
    }

    timestamp.sec = static_cast<int32_t>(epoch_ns / 1000000000LL);
    timestamp.nanosec = static_cast<uint32_t>(epoch_ns % 1000000000LL);
    return true;
}

bool VidaRosNode::fillEncoderMeasurement(
    vida_interfaces__msg__EncoderMeasurement &message,
    const EncoderReader::Snapshot &snapshot)
{
    if (!setTimestamp(message.timestamp, snapshot.timestamp_us))
    {
        return false;
    }

    message.count = snapshot.count;
    message.raw_velocity_m_s = snapshot.raw_velocity_m_s;
    message.filtered_velocity_m_s = snapshot.filtered_velocity_m_s;
    message.raw_acceleration_m_s2 = snapshot.raw_acceleration_m_s2;
    message.filtered_acceleration_m_s2 = snapshot.filtered_acceleration_m_s2;
    return true;
}
