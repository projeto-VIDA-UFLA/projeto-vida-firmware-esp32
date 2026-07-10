#include "comunication/VidaRosNode.h"

#include "utils/config.h"

namespace
{
    constexpr char NODE_NAME[] = "esp32_controller";
    constexpr char NODE_NAMESPACE[] = "/vida";
    constexpr char SENSORS_TOPIC[] = "/vida/sensors";
    constexpr int ROS_DOMAIN_ID = 0;

    struct SensorSnapshot
    {
        uint32_t timestamp_ms;
        int32_t steering_raw_adc;
        int64_t rear_count;
        int64_t right_count;
        int64_t left_count;
        float rear_velocity;
        float right_velocity;
        float left_velocity;
        float rear_acceleration;
        float right_acceleration;
        float left_acceleration;
    };
} // namespace

VidaRosNode *VidaRosNode::instance_ = nullptr;

VidaRosNode::VidaRosNode(SteeringSensor &steering_sensor, EncoderReader &rear_encoder,
                         EncoderReader &right_encoder, EncoderReader &left_encoder)
    : communication_(NODE_NAME, NODE_NAMESPACE, ROS_DOMAIN_ID),
      sensors_publisher_(rcl_get_zero_initialized_publisher()),
      sensors_publish_timer_(rcl_get_zero_initialized_timer()),
      sensors_message_{},
      sensors_message_buffer_{},
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
    configureSensorsMessage();

    if (!communication_.addPublisher(
            sensors_publisher_,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
            SENSORS_TOPIC))
    {
        return false;
    }

    return communication_.addTimer(
        sensors_publish_timer_,
        TASK_COMUNICATION_PERIOD_MS,
        publishSensorsCallback);
}

void VidaRosNode::configureSensorsMessage()
{
    sensors_message_.data.data = sensors_message_buffer_;
    sensors_message_.data.size = SENSOR_MESSAGE_SIZE;
    sensors_message_.data.capacity = SENSOR_MESSAGE_SIZE;

    sensors_message_.layout.dim.data = nullptr;
    sensors_message_.layout.dim.size = 0;
    sensors_message_.layout.dim.capacity = 0;
    sensors_message_.layout.data_offset = 0;
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

    const SensorSnapshot snapshot = {
        millis(),
        steering_sensor_.getRawAdc(),
        rear_encoder_.getCount(),
        right_encoder_.getCount(),
        left_encoder_.getCount(),
        rear_encoder_.getVelocity(),
        right_encoder_.getVelocity(),
        left_encoder_.getVelocity(),
        rear_encoder_.getAcceleration(),
        right_encoder_.getAcceleration(),
        left_encoder_.getAcceleration()};

    sensors_message_buffer_[0] = static_cast<double>(snapshot.timestamp_ms);
    sensors_message_buffer_[1] = static_cast<double>(snapshot.steering_raw_adc);
    sensors_message_buffer_[2] = static_cast<double>(snapshot.rear_count);
    sensors_message_buffer_[3] = static_cast<double>(snapshot.right_count);
    sensors_message_buffer_[4] = static_cast<double>(snapshot.left_count);
    sensors_message_buffer_[5] = static_cast<double>(snapshot.rear_velocity);
    sensors_message_buffer_[6] = static_cast<double>(snapshot.right_velocity);
    sensors_message_buffer_[7] = static_cast<double>(snapshot.left_velocity);
    sensors_message_buffer_[8] = static_cast<double>(snapshot.rear_acceleration);
    sensors_message_buffer_[9] = static_cast<double>(snapshot.right_acceleration);
    sensors_message_buffer_[10] = static_cast<double>(snapshot.left_acceleration);

    communication_.publish(sensors_publisher_, &sensors_message_);
}
