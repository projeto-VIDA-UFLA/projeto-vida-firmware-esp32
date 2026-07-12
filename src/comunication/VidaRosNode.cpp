#include "comunication/VidaRosNode.h"
#include "utils/config.h"

namespace
{
    constexpr char NODE_NAME[] = "esp32_controller";
    constexpr char NODE_NAMESPACE[] = "/vida";
    constexpr char ENCODERS_TOPIC[] = "/vida/sensors/encoders";
    constexpr char STEERING_TOPIC[] = "/vida/sensors/steering";
    constexpr char CMD_STEERING_TOPIC[] = "/vida/actuators/steering";
    constexpr char CMD_DRIVE_TOPIC[] = "/vida/actuators/drive";
    constexpr int ROS_DOMAIN_ID = 0;
} // namespace

VidaRosNode *VidaRosNode::instance_ = nullptr;

VidaRosNode::VidaRosNode(SteeringMotor &steering_motor,
                         DriveMotor &drive_motor,
                         SteeringSensor &steering_sensor,
                         EncoderReader &rear_encoder,
                         EncoderReader &right_encoder,
                         EncoderReader &left_encoder)
    : communication_(NODE_NAME, NODE_NAMESPACE, ROS_DOMAIN_ID),
      encoders_publisher_(rcl_get_zero_initialized_publisher()),
      steering_publisher_(rcl_get_zero_initialized_publisher()),
      cmd_steering_publisher_(rcl_get_zero_initialized_publisher()),
      sensors_publish_timer_(rcl_get_zero_initialized_timer()),
      encoders_message_{},
      steering_message_{},
      cmd_steering_message_{},
      steering_motor_(steering_motor),
      drive_motor_(drive_motor),
      steering_sensor_(steering_sensor),
      rear_encoder_(rear_encoder),
      right_encoder_(right_encoder),
      left_encoder_(left_encoder)
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
            cmd_steering_publisher_,
            ROSIDL_GET_MSG_TYPE_SUPPORT(vida_interfaces, msg, Comands),
            CMD_DRIVE_TOPIC))
    {
        return false;
    }

    return communication_.addTimer(sensors_publish_timer_, TASK_COMUNICATION_PERIOD_MS, publishSensorsCallback);
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

    const SteeringMotor::Snapshot steering_pwm = steering_motor_.getSnapshot();
    const DriveMotor::Snapshot drive_pwm = drive_motor_.getSnapshot();

    const EncoderReader::Snapshot left = left_encoder_.getMeasurementSnapshot();
    const EncoderReader::Snapshot right = right_encoder_.getMeasurementSnapshot();
    const EncoderReader::Snapshot rear = rear_encoder_.getMeasurementSnapshot();
    const SteeringSensor::Snapshot steering = steering_sensor_.getSnapshot();

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
