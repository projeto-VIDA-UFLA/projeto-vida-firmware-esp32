#pragma once
#include <Arduino.h>

class EncoderReader
{
public:
    EncoderReader(uint8_t pin_a, uint8_t pin_b, const char* name);

    void begin();
    long getCount();
    void getSnapshot(int64_t &count, int64_t &timestamp_us);
    long getAndResetDelta();
    void reset();

    void handleInterrupt();

    const char* getName() const;

private:
    uint8_t pin_a_;
    uint8_t pin_b_;
    const char* name_;

    volatile int64_t count_;
    volatile long delta_;
    volatile int64_t last_timestamp_us_;
    volatile uint8_t last_state_;

    portMUX_TYPE mux_;

    uint8_t readState() const;
};
