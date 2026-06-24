#include "EncoderReader.h"
#include "esp_timer.h"

EncoderReader::EncoderReader(uint8_t pin_a, uint8_t pin_b, const char* name)
    : pin_a_(pin_a),
      pin_b_(pin_b),
      name_(name),
      count_(0),
      delta_(0),
      last_timestamp_us_(0),
      last_state_(0),
      mux_(portMUX_INITIALIZER_UNLOCKED)
{
}

void EncoderReader::begin()
{
    pinMode(pin_a_, INPUT);
    pinMode(pin_b_, INPUT);

    last_state_ = readState();
    last_timestamp_us_ = esp_timer_get_time();
}

uint8_t EncoderReader::readState() const
{
    uint8_t a = digitalRead(pin_a_);
    uint8_t b = digitalRead(pin_b_);
    return (a << 1) | b;
}

void EncoderReader::handleInterrupt()
{
    uint8_t current_state = readState();

    // Tabela de decodificação quadratura x4.
    // Índice: estado_anterior << 2 | estado_atual
    static const int8_t transition_table[16] = {
        0, -1,  1,  0,
        1,  0,  0, -1,
       -1,  0,  0,  1,
        0,  1, -1,  0
    };

    uint8_t index = (last_state_ << 2) | current_state;
    int8_t step = transition_table[index];

    if (step != 0)
    {
        portENTER_CRITICAL_ISR(&mux_);
        count_ += step;
        delta_ += step;
        last_timestamp_us_ = esp_timer_get_time();
        portEXIT_CRITICAL_ISR(&mux_);
    }

    last_state_ = current_state;
}

long EncoderReader::getCount()
{
    portENTER_CRITICAL(&mux_);
    long value = static_cast<long>(count_);
    portEXIT_CRITICAL(&mux_);
    return value;
}

void EncoderReader::getSnapshot(int64_t &count, int64_t &timestamp_us)
{
    portENTER_CRITICAL(&mux_);
    count = count_;
    timestamp_us = last_timestamp_us_;
    portEXIT_CRITICAL(&mux_);
}

long EncoderReader::getAndResetDelta()
{
    portENTER_CRITICAL(&mux_);
    long value = delta_;
    delta_ = 0;
    portEXIT_CRITICAL(&mux_);
    return value;
}

void EncoderReader::reset()
{
    portENTER_CRITICAL(&mux_);
    count_ = 0;
    delta_ = 0;
    last_timestamp_us_ = esp_timer_get_time();
    portEXIT_CRITICAL(&mux_);
}

const char* EncoderReader::getName() const
{
    return name_;
}
