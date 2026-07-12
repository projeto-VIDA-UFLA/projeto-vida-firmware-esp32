#pragma once
#include <Arduino.h>

class SteeringSensor
{
public:
    struct Snapshot
    {
        int64_t timestamp_us;
        uint16_t raw_adc;
        float raw_angle_deg;
        float filtered_angle_deg;
        float filtered_angular_velocity_deg_s;
    };

    explicit SteeringSensor(uint8_t adc_pin);

    void begin();
    void update();

    int getRawAdc() const;
    float getRawAngleDeg() const;
    float getFilteredAngleDeg() const;    
    float getAngularVelocity() const;
    Snapshot getSnapshot();

private:
    uint8_t adc_pin_;

    int raw_adc_;
    float raw_angle_deg_;
    float angle_deg_;     
    float angular_velocity_;

    float filtered_angle_deg_;
    bool filter_initialized_;
    int64_t previous_update_us_;
    int64_t measurement_timestamp_us_;
    portMUX_TYPE mux_;
};
