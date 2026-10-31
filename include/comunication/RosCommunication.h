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

/**
 * Infraestrutura micro-ROS generica: sessao com o agente, criacao das
 * entidades (no, publishers, subscriptions, timers) e reconexao automatica.
 *
 * As entidades sao registradas antes de startTask() e criadas a cada conexao
 * com o agente. A propria task (TaskRosCommunication) executa a maquina de
 * estados:
 *   WaitingAgent  -- ping ok -->  AgentAvailable  -- entidades criadas -->
 *   AgentConnected -- MAX_MISSED_AGENT_PINGS seguidos ou sem mensagens
 *   recebidas --> AgentDisconnected -- entidades destruidas --> WaitingAgent
 *
 * O ping nao confere a sessao: um agente que reinicia rapido responde aos
 * pings mas descarta a sessao antiga ("zumbi"). Por isso, se alguma
 * subscription ja recebeu mensagem nesta sessao e todas ficam
 * INBOUND_TIMEOUT_MS sem receber, a sessao e recriada. Sem nenhuma mensagem
 * desde a conexao (notebook desligado) nao ha reconexao em laco.
 *
 * Conectada, cada volta do laco faz no maximo uma chamada bloqueante
 * (sincronizacao de relogio ou ping, com timeouts curtos) e depois roda o
 * executor, que dispara os callbacks de subscriptions e timers.
 */
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

    // Define o transporte serial do micro-ROS.
    void begin(Stream &transport);
    // Cria a task que executa a maquina de estados da sessao.
    bool startTask(const char *task_name, uint32_t stack_size, UBaseType_t priority, BaseType_t core_id);

    // Registro de entidades: so antes da primeira conexao (ate 8 de cada).
    bool addPublisher(rcl_publisher_t &publisher, const rosidl_message_type_support_t *type_support, const char *topic_name, const rmw_qos_profile_t *qos = &rmw_qos_profile_default);
    bool addSubscription(rcl_subscription_t &subscription, const rosidl_message_type_support_t *type_support, const char *topic_name, void *message, SubscriptionCallback callback, const rmw_qos_profile_t *qos = &rmw_qos_profile_default);
    bool addTimer(rcl_timer_t &timer, uint32_t period_ms, TimerCallback callback);
   
    // Publica se conectado; false caso contrario.
    bool publish(rcl_publisher_t &publisher, const void *message);
    
    State getState() const;
    bool isConnected() const;
    // Relogio sincronizado com o host do agente (Odroid).
    bool isTimeSynchronized() const;
    // Converte um instante local (esp_timer) para o relogio do agente.
    // false enquanto o relogio nao estiver sincronizado.
    bool localTimeToEpochNanos(int64_t local_time_us, int64_t &epoch_ns) const;

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
        RosCommunication *owner;
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
    // Marca a chegada de uma mensagem e repassa ao callback registrado.
    static void subscriptionEntry(const void *message, void *context);

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
    int64_t last_time_sync_ms_;
    uint8_t missed_agent_pings_;
    // Ultima mensagem recebida (uxr_millis); so conta depois da 1a da sessao.
    int64_t last_inbound_ms_;
    bool inbound_received_;
};
