#pragma once

#include "config.h"

#if ENABLE_MICRO_ROS_COMMAND_NODE

#include <Arduino.h>

#include <geometry_msgs/msg/twist.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>

#include "VehicleController.h"
#include "Watchdog.h"

class MicroRosCommand
{
public:
    MicroRosCommand(VehicleController& controller, Watchdog& command_watchdog);

    bool begin();
    void update();

    bool isReady() const;
    bool getSoftwareEnable() const;

private:
    VehicleController& controller_;
    Watchdog& command_watchdog_;

    rcl_allocator_t allocator_;
    rclc_support_t support_;
    rcl_node_t node_;
    rcl_subscription_t cmd_subscription_;
    rclc_executor_t executor_;
    geometry_msgs__msg__Twist cmd_msg_;

    bool initialized_;
    bool software_enable_;
    uint32_t last_init_attempt_ms_;

    static MicroRosCommand* active_instance_;
    static void commandCallback(const void* msg_in);
    bool initializeEntities();
    void handleCommand(const geometry_msgs__msg__Twist& msg);
};

#endif
