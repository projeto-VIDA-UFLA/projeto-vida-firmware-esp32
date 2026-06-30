#include "SafetyManager.h"

SafetyManager::SafetyManager(uint8_t physical_enable_pin)
    : physical_enable_pin_(physical_enable_pin),
      physical_enable_ok_(false),
      odroid_ready_(false),
      software_enable_(false),
      watchdog_ok_(false),
      critical_fault_(false),
      state_(SafetyState::STATE_BOOT)
{
}

void SafetyManager::begin()
{
    pinMode(physical_enable_pin_, INPUT);
    state_ = SafetyState::STATE_BOOT;
}

void SafetyManager::update(bool odroid_ready,
                           bool software_enable,
                           bool watchdog_ok,
                           bool critical_fault)
{
    physical_enable_ok_ = digitalRead(physical_enable_pin_) == HIGH;
    odroid_ready_ = odroid_ready;
    software_enable_ = software_enable;
    watchdog_ok_ = watchdog_ok;
    critical_fault_ = critical_fault;

    if (critical_fault_ || !physical_enable_ok_)
    {
        state_ = SafetyState::STATE_FAULT;
        return;
    }

    if (state_ == SafetyState::STATE_BOOT)
    {
        state_ = SafetyState::STATE_WAITING_ODROID;
        return;
    }

    if (!odroid_ready_)
    {
        state_ = SafetyState::STATE_WAITING_ODROID;
        return;
    }

    if (!watchdog_ok_)
    {
        state_ = SafetyState::STATE_SAFE_STOP;
        return;
    }

    if (!software_enable_)
    {
        state_ = SafetyState::STATE_DISABLED;
        return;
    }

    state_ = SafetyState::STATE_ENABLED;
}

bool SafetyManager::canActuate() const
{
    return state_ == SafetyState::STATE_ENABLED;
}

bool SafetyManager::isPhysicalEnableOk() const
{
    return physical_enable_ok_;
}

SafetyState SafetyManager::getState() const
{
    return state_;
}

const char* SafetyManager::getStateName() const
{
    switch (state_)
    {
        case SafetyState::STATE_BOOT: return "BOOT";
        case SafetyState::STATE_WAITING_ODROID: return "WAITING_ODROID";
        case SafetyState::STATE_DISABLED: return "DISABLED";
        case SafetyState::STATE_ENABLED: return "ENABLED";
        case SafetyState::STATE_SAFE_STOP: return "SAFE_STOP";
        case SafetyState::STATE_FAULT: return "FAULT";
        default: return "UNKNOWN";
    }
}
