#pragma once
#include <Arduino.h>

class SteeringSensor
{
public:
    explicit SteeringSensor(uint8_t adc_pin);

    void begin();
    void update();

    int getRawAdc() const;
    float getVoltage() const;
    float getAngleDeg() const;

private:
    uint8_t adc_pin_;

    int raw_adc_;
    float voltage_;
    float angle_deg_;

    float filtered_angle_deg_;
    bool filter_initialized_;
};
