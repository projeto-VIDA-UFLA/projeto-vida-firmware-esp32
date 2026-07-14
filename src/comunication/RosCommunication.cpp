#include "comunication/RosCommunication.h"
#include "esp_timer.h"

RosCommunication::RosCommunication(const char *node_name,
                                   const char *node_namespace,
                                   int domain_id)
    : node_name_(node_name),
      node_namespace_(node_namespace),
      domain_id_(domain_id),
      state_(State::WaitingAgent),
      transport_(nullptr),
      task_handle_(nullptr),
      allocator_(rcl_get_default_allocator()),
      init_options_(rcl_get_zero_initialized_init_options()),
      support_(),
      node_(rcl_get_zero_initialized_node()),
      executor_(rclc_executor_get_zero_initialized_executor()),
      publishers_(),
      subscriptions_(),
      timers_(),
      publisher_count_(0),
      subscription_count_(0),
      timer_count_(0),
      init_options_created_(false),
      support_created_(false),
      node_created_(false),
      entities_created_(false),
      executor_created_(false),
      last_agent_wait_ping_ms_(-1),
      last_agent_connected_ping_ms_(-1),
      last_time_sync_ms_(-1)
{
}

void RosCommunication::begin(Stream &transport)
{
    transport_ = &transport;
    set_microros_serial_transports(transport);
    state_ = State::WaitingAgent;
}

bool RosCommunication::startTask(const char *task_name,
                                 uint32_t stack_size,
                                 UBaseType_t priority,
                                 BaseType_t core_id)
{
    return xTaskCreatePinnedToCore(taskEntry,
                                   task_name,
                                   stack_size,
                                   this,
                                   priority,
                                   &task_handle_,
                                   core_id) == pdPASS;
}

bool RosCommunication::addPublisher(rcl_publisher_t &publisher,
                                    const rosidl_message_type_support_t *type_support,
                                    const char *topic_name,
                                    const rmw_qos_profile_t *qos)
{
    if (entities_created_ || publisher_count_ >= MAX_PUBLISHERS)
    {
        return false;
    }

    publishers_[publisher_count_++] = {&publisher, type_support, topic_name, qos, false};
    return true;
}

bool RosCommunication::addSubscription(rcl_subscription_t &subscription,
                                       const rosidl_message_type_support_t *type_support,
                                       const char *topic_name,
                                       void *message,
                                       SubscriptionCallback callback,
                                       const rmw_qos_profile_t *qos)
{
    if (entities_created_ || subscription_count_ >= MAX_SUBSCRIPTIONS)
    {
        return false;
    }

    subscriptions_[subscription_count_++] = {
        &subscription,
        type_support,
        topic_name,
        message,
        callback,
        qos,
        false};
    return true;
}

bool RosCommunication::addTimer(rcl_timer_t &timer,
                                uint32_t period_ms,
                                TimerCallback callback)
{
    if (entities_created_ || timer_count_ >= MAX_TIMERS)
    {
        return false;
    }

    timers_[timer_count_++] = {&timer, period_ms, callback, false};
    return true;
}

bool RosCommunication::publish(rcl_publisher_t &publisher, const void *message)
{
    if (state_ != State::AgentConnected)
    {
        return false;
    }

    return rcl_publish(&publisher, message, nullptr) == RCL_RET_OK;
}

RosCommunication::State RosCommunication::getState() const
{
    return state_;
}

bool RosCommunication::isConnected() const
{
    return state_ == State::AgentConnected;
}

bool RosCommunication::isTimeSynchronized() const
{
    return state_ == State::AgentConnected && rmw_uros_epoch_synchronized();
}

bool RosCommunication::localTimeToEpochNanos(int64_t local_time_us, int64_t &epoch_ns) const
{
    if (!isTimeSynchronized())
    {
        return false;
    }

    const int64_t local_now_us = esp_timer_get_time();
    epoch_ns = rmw_uros_epoch_nanos() - (local_now_us - local_time_us) * 1000LL;
    return true;
}

void RosCommunication::taskEntry(void *parameter)
{
    static_cast<RosCommunication *>(parameter)->taskLoop();
}

void RosCommunication::taskLoop()
{
    constexpr uint32_t WAITING_AGENT_PING_PERIOD_MS = 500;
    constexpr uint32_t CONNECTED_AGENT_PING_PERIOD_MS = 200;
    constexpr uint32_t DISCONNECTED_LOOP_DELAY_MS = 10;
    constexpr uint32_t CONNECTED_SPIN_TIMEOUT_MS = 1;
    constexpr uint32_t CONNECTED_LOOP_DELAY_MS = 1;
    constexpr uint32_t TIME_SYNC_PERIOD_MS = 30000;

    for (;;)
    {
        switch (state_)
        {
        case State::WaitingAgent:
            if (shouldRunEvery(WAITING_AGENT_PING_PERIOD_MS, last_agent_wait_ping_ms_) &&
                rmw_uros_ping_agent(100, 1) == RMW_RET_OK)
            {
                state_ = State::AgentAvailable;
            }
            break;

        case State::AgentAvailable:
            state_ = createEntities() ? State::AgentConnected : State::WaitingAgent;
            if (state_ == State::WaitingAgent)
            {
                destroyEntities();
            }
            break;

        case State::AgentConnected:
            if (shouldRunEvery(TIME_SYNC_PERIOD_MS, last_time_sync_ms_))
            {
                (void)rmw_uros_sync_session(1000);
            }

            if (shouldRunEvery(CONNECTED_AGENT_PING_PERIOD_MS, last_agent_connected_ping_ms_) &&
                rmw_uros_ping_agent(100, 1) != RMW_RET_OK)
            {
                state_ = State::AgentDisconnected;
                break;
            }

            if (executor_created_)
            {
                (void)rclc_executor_spin_some(
                    &executor_,
                    RCL_MS_TO_NS(CONNECTED_SPIN_TIMEOUT_MS));
            }
            break;

        case State::AgentDisconnected:
            destroyEntities();
            state_ = State::WaitingAgent;
            last_agent_wait_ping_ms_ = -1;
            last_agent_connected_ping_ms_ = -1;
            last_time_sync_ms_ = -1;
            break;
        }

        const uint32_t loop_delay_ms = state_ == State::AgentConnected
                                           ? CONNECTED_LOOP_DELAY_MS
                                           : DISCONNECTED_LOOP_DELAY_MS;

        vTaskDelay(pdMS_TO_TICKS(loop_delay_ms));
    }
}

bool RosCommunication::createEntities()
{
    allocator_ = rcl_get_default_allocator();
    init_options_ = rcl_get_zero_initialized_init_options();

    if (rcl_init_options_init(&init_options_, allocator_) != RCL_RET_OK)
    {
        return false;
    }
    init_options_created_ = true;

    if (rcl_init_options_set_domain_id(&init_options_, domain_id_) != RCL_RET_OK)
    {
        return false;
    }

    if (rclc_support_init_with_options(&support_, 0, nullptr, &init_options_, &allocator_) != RCL_RET_OK)
    {
        return false;
    }
    support_created_ = true;

    if (rclc_node_init_default(&node_, node_name_, node_namespace_, &support_) != RCL_RET_OK)
    {
        return false;
    }
    node_created_ = true;

    for (size_t i = 0; i < publisher_count_; ++i)
    {
        PublisherConfig &publisher = publishers_[i];
        if (rclc_publisher_init(publisher.publisher,
                                &node_,
                                publisher.type_support,
                                publisher.topic_name,
                                publisher.qos) != RCL_RET_OK)
        {
            return false;
        }
        publisher.initialized = true;
    }

    for (size_t i = 0; i < subscription_count_; ++i)
    {
        SubscriptionConfig &subscription = subscriptions_[i];
        if (rclc_subscription_init(subscription.subscription,
                                   &node_,
                                   subscription.type_support,
                                   subscription.topic_name,
                                   subscription.qos) != RCL_RET_OK)
        {
            return false;
        }
        subscription.initialized = true;
    }

    for (size_t i = 0; i < timer_count_; ++i)
    {
        TimerConfig &timer = timers_[i];
        if (rclc_timer_init_default(timer.timer,
                                    &support_,
                                    RCL_MS_TO_NS(timer.period_ms),
                                    timer.callback) != RCL_RET_OK)
        {
            return false;
        }
        timer.initialized = true;
    }

    const unsigned int num_handles = subscription_count_ + timer_count_;
    if (num_handles > 0)
    {
        executor_ = rclc_executor_get_zero_initialized_executor();
        if (rclc_executor_init(&executor_, &support_.context, num_handles, &allocator_) != RCL_RET_OK)
        {
            return false;
        }
        executor_created_ = true;

        for (size_t i = 0; i < subscription_count_; ++i)
        {
            SubscriptionConfig &subscription = subscriptions_[i];
            if (rclc_executor_add_subscription(&executor_,
                                               subscription.subscription,
                                               subscription.message,
                                               subscription.callback,
                                               ON_NEW_DATA) != RCL_RET_OK)
            {
                return false;
            }
        }

        for (size_t i = 0; i < timer_count_; ++i)
        {
            if (rclc_executor_add_timer(&executor_, timers_[i].timer) != RCL_RET_OK)
            {
                return false;
            }
        }
    }

    entities_created_ = true;
    return true;
}

void RosCommunication::destroyEntities()
{
    if (!entities_created_ &&
        !executor_created_ &&
        !node_created_ &&
        !support_created_ &&
        !init_options_created_)
    {
        return;
    }

    if (support_created_)
    {
        rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support_.context);
        (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
    }

    if (executor_created_)
    {
        (void)rclc_executor_fini(&executor_);
        executor_created_ = false;
    }

    for (size_t i = 0; i < timer_count_; ++i)
    {
        if (timers_[i].initialized)
        {
            (void)rcl_timer_fini(timers_[i].timer);
            timers_[i].initialized = false;
        }
    }

    for (size_t i = 0; i < subscription_count_; ++i)
    {
        if (subscriptions_[i].initialized)
        {
            (void)rcl_subscription_fini(subscriptions_[i].subscription, &node_);
            subscriptions_[i].initialized = false;
        }
    }

    for (size_t i = 0; i < publisher_count_; ++i)
    {
        if (publishers_[i].initialized)
        {
            (void)rcl_publisher_fini(publishers_[i].publisher, &node_);
            publishers_[i].initialized = false;
        }
    }

    if (node_created_)
    {
        (void)rcl_node_fini(&node_);
        node_created_ = false;
    }

    if (init_options_created_)
    {
        (void)rcl_init_options_fini(&init_options_);
        init_options_created_ = false;
    }

    if (support_created_)
    {
        (void)rclc_support_fini(&support_);
        support_created_ = false;
    }

    entities_created_ = false;
}

bool RosCommunication::shouldRunEvery(uint32_t period_ms, int64_t &last_run_ms)
{
    const int64_t now_ms = uxr_millis();

    if (last_run_ms < 0 || now_ms - last_run_ms >= static_cast<int64_t>(period_ms))
    {
        last_run_ms = now_ms;
        return true;
    }

    return false;
}
