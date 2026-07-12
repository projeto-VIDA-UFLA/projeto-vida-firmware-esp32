#pragma once
#include <Arduino.h>

enum class SafetyState
{
    STATE_BOOT = 0,
    STATE_WAITING_ODROID,
    STATE_DISABLED,
    STATE_ENABLED,
    STATE_SAFE_STOP,
    STATE_FAULT
};

class SafetyManager
{
public:
    explicit SafetyManager(uint8_t physical_enable_pin);

    void begin();
    void update(bool odroid_ready,
                bool software_enable,
                bool watchdog_ok,
                bool critical_fault);

    bool canActuate() const;
    bool isPhysicalEnableOk() const;
    SafetyState getState() const;
    const char* getStateName() const;

private:
    uint8_t physical_enable_pin_;

    bool physical_enable_ok_;
    bool odroid_ready_;
    bool software_enable_;
    bool watchdog_ok_;
    bool critical_fault_;

    SafetyState state_;
};
