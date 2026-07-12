#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#if defined(__INTELLISENSE__) || defined(__clangd__)
#include "microros_intellisense_stub.h"
#else
#include <vida_interfaces/msg/encoders.h>
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
    vida_interfaces__msg__Encoders encoder_msg;

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
        encoderLeft.getSnapshot(
            encoder_msg.encoder_left_counts,
            encoder_msg.encoder_left_ts);
        encoderRight.getSnapshot(
            encoder_msg.encoder_right_counts,
            encoder_msg.encoder_right_ts);
        encoderRear.getSnapshot(
            encoder_msg.encoder_rear_counts,
            encoder_msg.encoder_rear_ts);
    }

    void publishEncoderCounts(rcl_timer_t *timer, int64_t last_call_time)
    {
        (void)timer;
        (void)last_call_time;

        syncEncoderMessage();
        (void)ros.publish(encoder_pub, &encoder_msg);
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

    syncEncoderMessage();

    ros.begin(Serial);
    ros.addPublisher(
        encoder_pub,
        ROSIDL_GET_MSG_TYPE_SUPPORT(vida_interfaces, msg, Encoders),
        "/vida/encoders");
    ros.addTimer(encoder_timer, ROS_PUBLISH_PERIOD_MS, publishEncoderCounts);
    ros.startTask("TaskMicroRos", 8192, 2, 1);
}

void loop()
{
    vTaskDelay(pdMS_TO_TICKS(1000));
}
