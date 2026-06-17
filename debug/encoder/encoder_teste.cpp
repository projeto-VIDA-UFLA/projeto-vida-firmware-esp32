#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#if defined(__INTELLISENSE__) || defined(__clangd__)
#include "microros_intellisense_stub.h"
#else
#include <std_msgs/msg/int32_multi_array.h>
#endif

#include "config.h"
#include "pinout.h"
#include "EncoderReader.h"
#include "RosCommunication.h"

namespace
{
    EncoderReader encoderLeft(ENC_LEFT_A, ENC_LEFT_B, "left");
    EncoderReader encoderRight(ENC_RIGHT_A, ENC_RIGHT_B, "right");
    EncoderReader encoderRear(ENC_REAR_A, ENC_REAR_B, "rear");

    constexpr uint32_t ROS_PUBLISH_PERIOD_MS = 50;
    constexpr int ROS_DOMAIN_ID = 0;

    RosCommunication ros("vida_encoder_node", "", ROS_DOMAIN_ID);

    rcl_publisher_t encoder_pub;
    rcl_timer_t encoder_timer;
    std_msgs__msg__Int32MultiArray encoder_msg;
    int32_t encoder_counts_data[3] = {0, 0, 0};

    void IRAM_ATTR isrRearEncoder()
    {
        encoderRear.handleInterrupt();
    }

    void IRAM_ATTR isrRightEncoder()
    {
        encoderRight.handleInterrupt();
    }

    void IRAM_ATTR isrLeftEncoder()
    {
        encoderLeft.handleInterrupt();
    }

    void syncEncoderMessage()
    {
        encoder_counts_data[0] = static_cast<int32_t>(encoderRear.getCount());
        encoder_counts_data[1] = static_cast<int32_t>(encoderLeft.getCount());
        encoder_counts_data[2] = static_cast<int32_t>(encoderRight.getCount());
    }

    void publishEncoderCounts(rcl_timer_t *timer, int64_t last_call_time)
    {
        (void)timer;
        (void)last_call_time;

        syncEncoderMessage();
        (void)ros.publish(encoder_pub, &encoder_msg);
    }

    void initEncoderMessage()
    {
        encoder_msg.layout.dim.capacity = 0;
        encoder_msg.layout.dim.size = 0;
        encoder_msg.layout.dim.data = nullptr;
        encoder_msg.layout.data_offset = 0;

        encoder_msg.data.capacity = 3;
        encoder_msg.data.size = 3;
        encoder_msg.data.data = encoder_counts_data;
        syncEncoderMessage();
    }
}

void setup()
{
    Serial.begin(SERIAL_BAUDRATE);
    delay(2000);

    encoderRear.begin();
    encoderRight.begin();
    encoderLeft.begin();

    attachInterrupt(digitalPinToInterrupt(ENC_REAR_A), isrRearEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_REAR_B), isrRearEncoder, CHANGE);

    attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_A), isrRightEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_B), isrRightEncoder, CHANGE);

    attachInterrupt(digitalPinToInterrupt(ENC_LEFT_A), isrLeftEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_LEFT_B), isrLeftEncoder, CHANGE);

    initEncoderMessage();

    ros.begin(Serial);
    ros.addPublisher(
        encoder_pub,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
        "/vida/encoder_counts");
    ros.addTimer(encoder_timer, ROS_PUBLISH_PERIOD_MS, publishEncoderCounts);
    ros.startTask("TaskMicroRos", 8192, 2, 1);
}

void loop()
{
    vTaskDelay(pdMS_TO_TICKS(1000));
}
