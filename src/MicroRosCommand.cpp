#include "MicroRosCommand.h"

#if ENABLE_MICRO_ROS_COMMAND_NODE

#include <micro_ros_platformio.h>

#include "config.h"
#include "utils.h"

MicroRosCommand* MicroRosCommand::active_instance_ = nullptr;

MicroRosCommand::MicroRosCommand(VehicleController& controller, Watchdog& command_watchdog)
    : controller_(controller),
      command_watchdog_(command_watchdog),
      allocator_(rcl_get_default_allocator()),
      support_(rclc_support_t()),
      node_(rcl_get_zero_initialized_node()),
      cmd_subscription_(rcl_get_zero_initialized_subscription()),
      executor_(rclc_executor_get_zero_initialized_executor()),
      cmd_msg_(),
      initialized_(false),
      software_enable_(false),
      last_init_attempt_ms_(0)
{
}

bool MicroRosCommand::begin()
{
    active_instance_ = this;
    set_microros_serial_transports(Serial);
    return initializeEntities();
}

bool MicroRosCommand::initializeEntities()
{
    if (rclc_support_init(&support_, 0, nullptr, &allocator_) != RCL_RET_OK)
    {
        return false;
    }

    if (rclc_node_init_default(&node_, MICRO_ROS_NODE_NAME, "", &support_) != RCL_RET_OK)
    {
        return false;
    }

    if (rclc_subscription_init_default(
            &cmd_subscription_,
            &node_,
            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
            MICRO_ROS_CMD_TOPIC) != RCL_RET_OK)
    {
        return false;
    }

    if (rclc_executor_init(&executor_, &support_.context, 1, &allocator_) != RCL_RET_OK)
    {
        return false;
    }

    if (rclc_executor_add_subscription(
            &executor_,
            &cmd_subscription_,
            &cmd_msg_,
            &MicroRosCommand::commandCallback,
            ON_NEW_DATA) != RCL_RET_OK)
    {
        return false;
    }

    initialized_ = true;
    software_enable_ = true;
    command_watchdog_.feed();
    return true;
}

void MicroRosCommand::update()
{
    if (!initialized_)
    {
        uint32_t now_ms = millis();
        if (now_ms - last_init_attempt_ms_ >= 1000)
        {
            last_init_attempt_ms_ = now_ms;
            initializeEntities();
        }
        return;
    }

    rclc_executor_spin_some(&executor_, RCL_MS_TO_NS(MICRO_ROS_SPIN_PERIOD_MS));
}

bool MicroRosCommand::isReady() const
{
#if REQUIRE_ODROID_HANDSHAKE
    return initialized_;
#else
    return true;
#endif
}

bool MicroRosCommand::getSoftwareEnable() const
{
    return software_enable_;
}

void MicroRosCommand::commandCallback(const void* msg_in)
{
    if (active_instance_ == nullptr || msg_in == nullptr)
    {
        return;
    }

    const geometry_msgs__msg__Twist* msg = static_cast<const geometry_msgs__msg__Twist*>(msg_in);
    active_instance_->handleCommand(*msg);
}

void MicroRosCommand::handleCommand(const geometry_msgs__msg__Twist& msg)
{
    float speed_normalized = clampFloat(
        static_cast<float>(msg.linear.x),
        0.0f,
        1.0f
    );
    float drive_command = 0.0f;
    if (speed_normalized > 0.0f)
    {
        drive_command = mapFloat(
            speed_normalized,
            0.0f,
            1.0f,
            DRIVE_OPEN_LOOP_MIN_COMMAND,
            DRIVE_OPEN_LOOP_MAX_COMMAND
        );
    }

    float steering_command = clampFloat(
        static_cast<float>(msg.angular.z),
        -MAX_ABS_STEER_COMMAND,
        MAX_ABS_STEER_COMMAND
    );

    controller_.setOpenLoopCommand(drive_command, steering_command);
    software_enable_ = true;
    command_watchdog_.feed();
}

#endif
