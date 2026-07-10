#pragma once
#include <Arduino.h>

class Watchdog
{
public:
    explicit Watchdog(uint32_t timeout_ms);

    void begin();
    void feed();
    bool isOk() const;
    uint32_t timeSinceLastFeedMs() const;

private:
    uint32_t timeout_ms_;
    volatile uint32_t last_feed_ms_;
};
