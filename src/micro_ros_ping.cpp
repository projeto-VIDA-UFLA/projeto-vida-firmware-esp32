#include <Arduino.h>

#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/int32.h>

#include "config.h"

namespace
{
rcl_publisher_t publisher;
std_msgs__msg__Int32 ping_msg;

void taskMicroRosPing(void* parameter)
{
    (void)parameter;

    set_microros_serial_transports(Serial);
    delay(2000);

    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    rcl_node_t node;

    while (rclc_support_init(&support, 0, nullptr, &allocator) != RCL_RET_OK)
    {
        delay(1000);
    }

    while (rclc_node_init_default(&node, "vida_ping_node", "", &support) != RCL_RET_OK)
    {
        delay(1000);
    }

    while (rclc_publisher_init_default(
               &publisher,
               &node,
               ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
               "/vida/ping") != RCL_RET_OK)
    {
        delay(1000);
    }

    ping_msg.data = 0;

    while (true)
    {
        rcl_publish(&publisher, &ping_msg, nullptr);
        ping_msg.data++;
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
}

void setup()
{
    Serial.begin(SERIAL_BAUDRATE);
    delay(1000);

    xTaskCreatePinnedToCore(
        taskMicroRosPing,
        "TaskMicroRosPing",
        8192,
        nullptr,
        PRIORITY_MICRO_ROS,
        nullptr,
        0
    );
}

void loop()
{
    vTaskDelay(pdMS_TO_TICKS(1000));
}
