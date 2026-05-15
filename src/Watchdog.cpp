#include "Watchdog.h"

Watchdog::Watchdog(uint32_t timeout_ms)
    : timeout_ms_(timeout_ms),
      last_feed_ms_(0)
{
}

void Watchdog::begin()
{
    feed();
}

void Watchdog::feed()
{
    last_feed_ms_ = millis();
}

bool Watchdog::isOk() const
{
    return (millis() - last_feed_ms_) <= timeout_ms_;
}

uint32_t Watchdog::timeSinceLastFeedMs() const
{
    return millis() - last_feed_ms_;
}
