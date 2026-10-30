#include "safety/TaskSupervisor.h"



namespace
{
    uint32_t nowMs()
    {
        return static_cast<uint32_t>(esp_timer_get_time() / 1000LL);
    }

    static_assert(TASK_SUPERVISOR_TIMEOUT_MS > TASK_SUPERVISOR_PERIOD_MS,
                  "O timeout do supervisor deve ser maior que o periodo");
    static_assert(TASK_WDT_TIMEOUT_S >= 1,
                  "O watchdog do ESP-IDF trabalha em segundos inteiros");
} // namespace

TaskSupervisor::TaskSupervisor(ActuatorManager &actuator_manager)
    : actuator_manager_(actuator_manager),
      timer_(nullptr),
      started_at_us_(0),
      watchdog_subscribed_(false),
      last_alive_ms_{0, 0, 0},
      tripped_(false),
      tripped_task_(0)
{
}

bool TaskSupervisor::begin()
{
    // Reconfigura o watchdog do ESP-IDF (ja inicializado pelo framework).
    if (esp_task_wdt_init(TASK_WDT_TIMEOUT_S, true) != ESP_OK)
    {
        return false;
    }

    const uint32_t now_ms = nowMs();
    for (int i = 0; i < TASK_COUNT; ++i)
    {
        last_alive_ms_[i] = now_ms;
    }
    started_at_us_ = esp_timer_get_time();

    const esp_timer_create_args_t timer_args = {
        &TaskSupervisor::timerCallback,
        this,
        ESP_TIMER_TASK,
        "task_supervisor",
        false};
    if (esp_timer_create(&timer_args, &timer_) != ESP_OK)
    {
        return false;
    }

    return esp_timer_start_periodic(
               timer_,
               static_cast<uint64_t>(TASK_SUPERVISOR_PERIOD_MS) * 1000ULL) == ESP_OK;
}

void TaskSupervisor::reportAlive(Task task)
{
    last_alive_ms_[static_cast<uint8_t>(task)] = nowMs();
}

bool TaskSupervisor::isTripped() const
{
    return tripped_;
}

TaskSupervisor::Task TaskSupervisor::trippedTask() const
{
    return static_cast<Task>(tripped_task_);
}

void TaskSupervisor::timerCallback(void *argument)
{
    static_cast<TaskSupervisor *>(argument)->check();
}

void TaskSupervisor::check()
{
    // O watchdog do ESP-IDF vigia a task do esp_timer, onde este callback
    // roda: se o supervisor parar, o ESP32 reinicia.
    if (!watchdog_subscribed_)
    {
        watchdog_subscribed_ = esp_task_wdt_add(nullptr) == ESP_OK;
    }
    (void)esp_task_wdt_reset();

    // Retido: reescreve a saida segura a cada ciclo, caso a task travada
    // volte a escrever nos atuadores.
    if (tripped_)
    {
        actuator_manager_.forceSafeOutputs();
        return;
    }

    constexpr int64_t startup_grace_us =
        static_cast<int64_t>(TASK_SUPERVISOR_STARTUP_GRACE_MS) * 1000LL;
    if ((esp_timer_get_time() - started_at_us_) < startup_grace_us)
    {
        return;
    }

    const uint32_t now_ms = nowMs();
    for (int i = 0; i < TASK_COUNT; ++i)
    {
        // Idade com sinal (correta na virada do contador). Uma task do nucleo 1
        // pode registrar a batida entre a leitura de now_ms e esta comparacao,
        // ja no milissegundo seguinte: idade -1, que sem sinal virava ~49 dias
        // e disparava o supervisor por engano (ensaio de 30/10, flag 8192 com
        // as tasks em dia).
        const int32_t age_ms = static_cast<int32_t>(now_ms - last_alive_ms_[i]);
        if (age_ms > static_cast<int32_t>(TASK_SUPERVISOR_TIMEOUT_MS))
        {
            tripped_task_ = static_cast<uint8_t>(i);
            tripped_ = true;
            actuator_manager_.forceSafeOutputs();
            return;
        }
    }
}
