#pragma once

#include <Arduino.h>

#include "comunication/RosCommunication.h"
#include "sensors/EncoderReader.h"
#include "sensors/SteeringSensor.h"

#if defined(__INTELLISENSE__) || defined(__clangd__)
#include "microros_intellisense_stub.h"
#else
#include <std_msgs/msg/float64_multi_array.h>
#endif

class VidaRosNode
{
public:
    VidaRosNode(SteeringSensor &steering_sensor,
                EncoderReader &rear_encoder,
                EncoderReader &right_encoder,
                EncoderReader &left_encoder);

    bool begin(Stream &transport);

private:
    static constexpr size_t SENSOR_MESSAGE_SIZE = 11;

    static void publishSensorsCallback(rcl_timer_t *timer, int64_t last_call_time);

    bool configure();
    void configureSensorsMessage();
    void publishSensors(rcl_timer_t *timer);

    static VidaRosNode *instance_;

    RosCommunication communication_;

    rcl_publisher_t sensors_publisher_;

    rcl_timer_t sensors_publish_timer_;

    std_msgs__msg__Float64MultiArray sensors_message_;

    double sensors_message_buffer_[SENSOR_MESSAGE_SIZE];

    SteeringSensor &steering_sensor_;
    EncoderReader &rear_encoder_;
    EncoderReader &right_encoder_;
    EncoderReader &left_encoder_;
};
