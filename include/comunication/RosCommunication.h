#pragma once

#include <Arduino.h>

#if defined(__INTELLISENSE__) || defined(__clangd__)
#include "microros_intellisense_stub.h"
#else
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#endif

class RosCommunication
{
public:
    using TimerCallback = void (*)(rcl_timer_t *timer, int64_t last_call_time);
    using SubscriptionCallback = void (*)(const void *msgin);

    enum class State
    {
        WaitingAgent,
        AgentAvailable,
        AgentConnected,
        AgentDisconnected
    };

    RosCommunication(const char *node_name, const char *node_namespace, int domain_id);

    void begin(Stream &transport);
    bool startTask(const char *task_name, uint32_t stack_size, UBaseType_t priority, BaseType_t core_id);

    bool addPublisher(rcl_publisher_t &publisher, const rosidl_message_type_support_t *type_support, const char *topic_name, const rmw_qos_profile_t *qos = &rmw_qos_profile_default);
    bool addSubscription(rcl_subscription_t &subscription, const rosidl_message_type_support_t *type_support, const char *topic_name, void *message, SubscriptionCallback callback, const rmw_qos_profile_t *qos = &rmw_qos_profile_default);
    bool addTimer(rcl_timer_t &timer, uint32_t period_ms, TimerCallback callback);
   
    bool publish(rcl_publisher_t &publisher, const void *message);
    
    State getState() const;
    bool isConnected() const;

private:
    static constexpr size_t MAX_PUBLISHERS = 8;
    static constexpr size_t MAX_SUBSCRIPTIONS = 8;
    static constexpr size_t MAX_TIMERS = 8;

    struct PublisherConfig
    {
        rcl_publisher_t *publisher;
        const rosidl_message_type_support_t *type_support;
        const char *topic_name;
        const rmw_qos_profile_t *qos;
        bool initialized;
    };

    struct SubscriptionConfig
    {
        rcl_subscription_t *subscription;
        const rosidl_message_type_support_t *type_support;
        const char *topic_name;
        void *message;
        SubscriptionCallback callback;
        const rmw_qos_profile_t *qos;
        bool initialized;
    };

    struct TimerConfig
    {
        rcl_timer_t *timer;
        uint32_t period_ms;
        TimerCallback callback;
        bool initialized;
    };

    static void taskEntry(void *parameter);

    void taskLoop();
    bool createEntities();
    void destroyEntities();
    bool shouldRunEvery(uint32_t period_ms, int64_t &last_run_ms);

    const char *node_name_;
    const char *node_namespace_;
    int domain_id_;

    State state_;
    Stream *transport_;
    TaskHandle_t task_handle_;

    rcl_allocator_t allocator_;
    rcl_init_options_t init_options_;
    rclc_support_t support_;
    rcl_node_t node_;
    rclc_executor_t executor_;

    PublisherConfig publishers_[MAX_PUBLISHERS];
    SubscriptionConfig subscriptions_[MAX_SUBSCRIPTIONS];
    TimerConfig timers_[MAX_TIMERS];
    size_t publisher_count_;
    size_t subscription_count_;
    size_t timer_count_;
    bool init_options_created_;
    bool support_created_;
    bool node_created_;
    bool entities_created_;
    bool executor_created_;

    int64_t last_agent_wait_ping_ms_;
    int64_t last_agent_connected_ping_ms_;
};
