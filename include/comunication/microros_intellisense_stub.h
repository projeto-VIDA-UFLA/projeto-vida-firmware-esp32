#pragma once

#include <stddef.h>
#include <stdint.h>

// Header usado apenas pelo IntelliSense no Windows. A build real inclui os
// headers do micro-ROS gerados pelo micro_ros_platformio.

class Stream;

typedef int rcl_ret_t;
typedef int rmw_ret_t;

constexpr rcl_ret_t RCL_RET_OK = 0;
constexpr rmw_ret_t RMW_RET_OK = 0;
constexpr int ON_NEW_DATA = 0;

struct rcl_allocator_t
{
};
struct rcl_init_options_t
{
};
struct rcl_node_t
{
};
struct rcl_publisher_t
{
};
struct rcl_subscription_t
{
};
struct rcl_timer_t
{
};
struct rmw_context_t
{
};
struct rmw_qos_profile_t
{
};

struct rcl_context_t
{
};

struct rclc_support_t
{
    rcl_context_t context;
};

struct rclc_executor_t
{
};

struct rosidl_message_type_support_t
{
};

struct rosidl_runtime_c__int32__Sequence
{
    int32_t *data;
    size_t size;
    size_t capacity;
};

struct rosidl_runtime_c__String
{
    char *data;
    size_t size;
    size_t capacity;
};

struct std_msgs__msg__MultiArrayDimension
{
    rosidl_runtime_c__String label;
    uint32_t size;
    uint32_t stride;
};

struct std_msgs__msg__MultiArrayDimension__Sequence
{
    std_msgs__msg__MultiArrayDimension *data;
    size_t size;
    size_t capacity;
};

struct std_msgs__msg__MultiArrayLayout
{
    std_msgs__msg__MultiArrayDimension__Sequence dim;
    uint32_t data_offset;
};

struct std_msgs__msg__Int32MultiArray
{
    std_msgs__msg__MultiArrayLayout layout;
    rosidl_runtime_c__int32__Sequence data;
};

struct std_msgs__msg__Float64MultiArray
{
    int64_t std_msgs__msg__Float64MultiArray;
};

struct builtin_interfaces__msg__Time
{
    int32_t sec;
    uint32_t nanosec;
};

struct vida_interfaces__msg__EncoderMeasurement
{
    builtin_interfaces__msg__Time timestamp;
    int64_t count;
    float raw_velocity_m_s;
    float filtered_velocity_m_s;
    float raw_acceleration_m_s2;
    float filtered_acceleration_m_s2;
};

struct vida_interfaces__msg__Encoders
{
    vida_interfaces__msg__EncoderMeasurement left;
    vida_interfaces__msg__EncoderMeasurement right;
    vida_interfaces__msg__EncoderMeasurement rear;
};

struct vida_interfaces__msg__Steering
{
    builtin_interfaces__msg__Time timestamp;
    uint16_t raw_adc;
    float raw_angle_deg;
    float filtered_angle_deg;
    float filtered_angular_velocity_deg_s;
};

struct vida_interfaces__msg__Comands
{
    builtin_interfaces__msg__Time timestamp;   
    int16_t pwm;
};

struct vida_interfaces__msg__SteeringReference
{
    builtin_interfaces__msg__Time timestamp;
    float angle_deg;
    float angular_velocity_deg_s;
};

struct vida_interfaces__msg__DriveReference
{
    builtin_interfaces__msg__Time timestamp;
    float velocity_m_s;
    float acceleration_m_s2;
};

struct vida_interfaces__msg__LogicalEnable
{
    builtin_interfaces__msg__Time timestamp;
    bool enabled;
};

struct vida_interfaces__msg__SafetyStatus
{
    builtin_interfaces__msg__Time timestamp;
    uint8_t state;
    bool physical_enable;
    bool logical_enable;
    bool logical_enable_received;
    bool motion_permitted;
};

extern rmw_qos_profile_t rmw_qos_profile_default;

#define RCL_MS_TO_NS(ms) ((int64_t)(ms) * 1000000LL)
#define ROSIDL_GET_MSG_TYPE_SUPPORT(pkg, subfolder, name) \
    ((const rosidl_message_type_support_t *)nullptr)

inline int64_t uxr_millis()
{
    return 0;
}

inline rcl_allocator_t rcl_get_default_allocator()
{
    return rcl_allocator_t{};
}

inline rcl_init_options_t rcl_get_zero_initialized_init_options()
{
    return rcl_init_options_t{};
}

inline rcl_publisher_t rcl_get_zero_initialized_publisher()
{
    return rcl_publisher_t{};
}

inline rcl_subscription_t rcl_get_zero_initialized_subscription()
{
    return rcl_subscription_t{};
}

inline rcl_timer_t rcl_get_zero_initialized_timer()
{
    return rcl_timer_t{};
}

inline rcl_node_t rcl_get_zero_initialized_node()
{
    return rcl_node_t{};
}

inline rclc_executor_t rclc_executor_get_zero_initialized_executor()
{
    return rclc_executor_t{};
}

inline rcl_ret_t rcl_init_options_init(rcl_init_options_t *, rcl_allocator_t)
{
    return RCL_RET_OK;
}

inline rcl_ret_t rcl_init_options_set_domain_id(rcl_init_options_t *, int)
{
    return RCL_RET_OK;
}

inline rcl_ret_t rclc_support_init_with_options(rclc_support_t *, int, const char *const *, rcl_init_options_t *, rcl_allocator_t *)
{
    return RCL_RET_OK;
}

inline rcl_ret_t rclc_node_init_default(rcl_node_t *, const char *, const char *, rclc_support_t *)
{
    return RCL_RET_OK;
}

inline rcl_ret_t rclc_publisher_init(rcl_publisher_t *, rcl_node_t *, const rosidl_message_type_support_t *, const char *, const rmw_qos_profile_t *)
{
    return RCL_RET_OK;
}

inline rcl_ret_t rclc_subscription_init(rcl_subscription_t *, rcl_node_t *, const rosidl_message_type_support_t *, const char *, const rmw_qos_profile_t *)
{
    return RCL_RET_OK;
}

inline rcl_ret_t rclc_timer_init_default(rcl_timer_t *, rclc_support_t *, int64_t, void (*)(rcl_timer_t *, int64_t))
{
    return RCL_RET_OK;
}

inline rcl_ret_t rclc_executor_init(rclc_executor_t *, rcl_context_t *, unsigned int, rcl_allocator_t *)
{
    return RCL_RET_OK;
}

inline rcl_ret_t rclc_executor_add_timer(rclc_executor_t *, rcl_timer_t *)
{
    return RCL_RET_OK;
}

inline rcl_ret_t rclc_executor_add_subscription(rclc_executor_t *, rcl_subscription_t *, void *, void (*)(const void *), int)
{
    return RCL_RET_OK;
}

inline rmw_context_t *rcl_context_get_rmw_context(rcl_context_t *)
{
    return nullptr;
}

inline rmw_ret_t rmw_uros_set_context_entity_destroy_session_timeout(rmw_context_t *, int)
{
    return RMW_RET_OK;
}

inline rcl_ret_t rclc_executor_fini(rclc_executor_t *)
{
    return RCL_RET_OK;
}

inline rcl_ret_t rcl_timer_fini(rcl_timer_t *)
{
    return RCL_RET_OK;
}

inline rcl_ret_t rcl_publisher_fini(rcl_publisher_t *, rcl_node_t *)
{
    return RCL_RET_OK;
}

inline rcl_ret_t rcl_subscription_fini(rcl_subscription_t *, rcl_node_t *)
{
    return RCL_RET_OK;
}

inline rcl_ret_t rcl_node_fini(rcl_node_t *)
{
    return RCL_RET_OK;
}

inline rcl_ret_t rcl_init_options_fini(rcl_init_options_t *)
{
    return RCL_RET_OK;
}

inline rcl_ret_t rclc_support_fini(rclc_support_t *)
{
    return RCL_RET_OK;
}

inline rmw_ret_t rmw_uros_ping_agent(int, int)
{
    return RMW_RET_OK;
}

inline rmw_ret_t rmw_uros_sync_session(int)
{
    return RMW_RET_OK;
}

inline bool rmw_uros_epoch_synchronized()
{
    return true;
}

inline int64_t rmw_uros_epoch_nanos()
{
    return 0;
}

inline rcl_ret_t rclc_executor_spin_some(rclc_executor_t *, int64_t)
{
    return RCL_RET_OK;
}

inline rcl_ret_t rcl_publish(rcl_publisher_t *, const void *, void *)
{
    return RCL_RET_OK;
}

inline void set_microros_serial_transports(Stream &)
{
}
