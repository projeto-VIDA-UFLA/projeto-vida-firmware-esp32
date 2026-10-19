#pragma once

#include <Arduino.h>
#include "esp_timer.h"

#include "actuators/ActuatorManager.h"

/**
 * Supervisor das tasks de tempo real do nucleo 1 (TaskSensors, TaskControl e
 * TaskSafety).
 *
 * Cada task chama reportAlive() a cada ciclo. Um timer periodico do
 * esp_timer (TASK_SUPERVISOR_PERIOD_MS), que roda no nucleo 0 fora das tasks
 * vigiadas, confere os sinais de vida. Uma task sem sinal por
 * TASK_SUPERVISOR_TIMEOUT_MS dispara ActuatorManager::forceSafeOutputs():
 * PWMs zerados e freio total escritos direto no hardware, sem depender do
 * mutex que a task travada pode estar segurando. A falha fica retida ate
 * reiniciar o ESP32; o supervisor nao reinicia sozinho, porque no reboot o
 * servo fica sem pulsos e o freio solta.
 *
 * O proprio supervisor e vigiado pelo watchdog de tasks do ESP-IDF
 * (TASK_WDT_TIMEOUT_S): se o timer parar, o ESP32 reinicia.
 *
 * Sem o supervisor, uma TaskSafety travada (ou um mutex preso) deixaria o
 * LEDC no ultimo PWM: os motores continuariam girando.
 */
class TaskSupervisor
{
public:
    enum class Task : uint8_t
    {
        SENSORS = 0,
        CONTROL = 1,
        SAFETY = 2
    };

    explicit TaskSupervisor(ActuatorManager &actuator_manager);

    // Configura o watchdog do ESP-IDF e inicia o timer. Chamar depois de
    // criar as tasks vigiadas.
    bool begin();

    // Sinal de vida da task, a cada ciclo.
    void reportAlive(Task task);

    bool isTripped() const;
    // Task que disparou o supervisor (valida se isTripped()).
    Task trippedTask() const;

private:
    static constexpr int TASK_COUNT = 3;

    static void timerCallback(void *argument);
    void check();

    ActuatorManager &actuator_manager_;
    esp_timer_handle_t timer_;
    int64_t started_at_us_;
    bool watchdog_subscribed_;

    // Escritos pelas tasks e lidos pelo timer: 32 bits (escrita atomica).
    volatile uint32_t last_alive_ms_[TASK_COUNT];
    volatile bool tripped_;
    volatile uint8_t tripped_task_;
};
