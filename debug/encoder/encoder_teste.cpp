#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <stdio.h>

#if defined(__INTELLISENSE__) || defined(__clangd__)
#include "microros_intellisense_stub.h"
#else
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/int32_multi_array.h>
#endif

#include "config.h"
#include "pinout.h"
#include "EncoderReader.h"

namespace
{
    EncoderReader encoderLeft(ENC_LEFT_A, ENC_LEFT_B, "left");
    EncoderReader encoderRight(ENC_RIGHT_A, ENC_RIGHT_B, "right");
    EncoderReader encoderRear(ENC_REAR_A, ENC_REAR_B, "rear");

    constexpr uint32_t ROS_SPIN_PERIOD_MS = 10;
    constexpr uint32_t ROS_PUBLISH_PERIOD_MS = 50;
    constexpr uint32_t AGENT_WAIT_PERIOD_MS = 50;
    constexpr int ROS_DOMAIN_ID = 0;

#define EXECUTE_EVERY_N_MS(MS, X)  do { \
        static volatile int64_t init = -1; \
        if (init == -1) { init = uxr_millis(); } \
        if (uxr_millis() - init > MS) { X; init = uxr_millis(); } \
    } while (0)

    enum states
    {
        WAITING_AGENT,
        AGENT_AVAILABLE,
        AGENT_CONNECTED,
        AGENT_DISCONNECTED
    } state;

    rcl_allocator_t allocator;
    rcl_init_options_t init_options;
    rclc_support_t support;
    rcl_node_t node;
    rcl_publisher_t encoder_pub;
    rcl_timer_t timer;
    rclc_executor_t executor;

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

    void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
    {
        (void)last_call_time;

        if (timer == nullptr)
        {
            return;
        }

        syncEncoderMessage();
        (void)rcl_publish(&encoder_pub, &encoder_msg, nullptr);
    }

    bool create_entities()
    {
        const char *node_name = "vida_encoder_node";
        const char *ns = "";

        allocator = rcl_get_default_allocator();
        init_options = rcl_get_zero_initialized_init_options();

        if (rcl_init_options_init(&init_options, allocator) != RCL_RET_OK)
        {
            return false;
        }

        if (rcl_init_options_set_domain_id(&init_options, ROS_DOMAIN_ID) != RCL_RET_OK)
        {
            return false;
        }

        if (rclc_support_init_with_options(&support, 0, nullptr, &init_options, &allocator) != RCL_RET_OK)
        {
            return false;
        }

        if (rclc_node_init_default(&node, node_name, ns, &support) != RCL_RET_OK)
        {
            return false;
        }

        if (rclc_publisher_init(
                &encoder_pub,
                &node,
                ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
                "/vida/encoder_counts",
                &rmw_qos_profile_default) != RCL_RET_OK)
        {
            return false;
        }

        if (rclc_timer_init_default(
                &timer,
                &support,
                RCL_MS_TO_NS(ROS_PUBLISH_PERIOD_MS),
                timer_callback) != RCL_RET_OK)
        {
            return false;
        }

        unsigned int num_handles = 1;
        executor = rclc_executor_get_zero_initialized_executor();
        if (rclc_executor_init(&executor, &support.context, num_handles, &allocator) != RCL_RET_OK)
        {
            return false;
        }

        if (rclc_executor_add_timer(&executor, &timer) != RCL_RET_OK)
        {
            return false;
        }

        return true;
    }

    void destroy_entities()
    {
        rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
        (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

        (void)rclc_executor_fini(&executor);
        (void)rcl_timer_fini(&timer);
        (void)rcl_publisher_fini(&encoder_pub, &node);
        (void)rcl_node_fini(&node);
        (void)rcl_init_options_fini(&init_options);
        (void)rclc_support_fini(&support);
    }

    void task_micro_ros(void *parameter)
    {
        (void)parameter;

        for (;;)
        {
            switch (state)
            {
            case WAITING_AGENT:
                EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
                vTaskDelay(pdMS_TO_TICKS(AGENT_WAIT_PERIOD_MS));
                break;

            case AGENT_AVAILABLE:
                state = create_entities() ? AGENT_CONNECTED : WAITING_AGENT;
                if (state == WAITING_AGENT)
                {
                    destroy_entities();
                }
                break;

            case AGENT_CONNECTED:
                EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
                if (state == AGENT_CONNECTED)
                {
                    (void)rclc_executor_spin_some(&executor, RCL_MS_TO_NS(ROS_SPIN_PERIOD_MS));
                }
                vTaskDelay(pdMS_TO_TICKS(ROS_SPIN_PERIOD_MS));
                break;

            case AGENT_DISCONNECTED:
                destroy_entities();
                state = WAITING_AGENT;
                break;
            }
        }
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
    set_microros_serial_transports(Serial);
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
    state = WAITING_AGENT;

    xTaskCreatePinnedToCore(task_micro_ros, "TaskMicroRos", 8192, nullptr, 2, nullptr, 1);
}

void loop()
{
    vTaskDelay(pdMS_TO_TICKS(1000));
}
