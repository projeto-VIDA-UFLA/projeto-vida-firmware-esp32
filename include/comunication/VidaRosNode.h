#pragma once

#include <Arduino.h>

#include "comunication/RosCommunication.h"
#include "sensors/EncoderReader.h"
#include "sensors/SteeringSensor.h"

#if defined(__INTELLISENSE__) || defined(__clangd__)
#include "microros_intellisense_stub.h"
#else
#include <vida_interfaces/msg/encoders.h>
#include <vida_interfaces/msg/steering.h>
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
    static void publishSensorsCallback(rcl_timer_t *timer, int64_t last_call_time);

    bool configure();
    void publishSensors(rcl_timer_t *timer);
    bool setTimestamp(builtin_interfaces__msg__Time &timestamp, int64_t local_time_us);
    bool fillEncoderMeasurement(vida_interfaces__msg__EncoderMeasurement &message,
                                const EncoderReader::Snapshot &snapshot);

    static VidaRosNode *instance_;

    RosCommunication communication_;

    rcl_publisher_t encoders_publisher_;
    rcl_publisher_t steering_publisher_;

    rcl_timer_t sensors_publish_timer_;

    vida_interfaces__msg__Encoders encoders_message_;
    vida_interfaces__msg__Steering steering_message_;

    SteeringSensor &steering_sensor_;
    EncoderReader &rear_encoder_;
    EncoderReader &right_encoder_;
    EncoderReader &left_encoder_;
};
