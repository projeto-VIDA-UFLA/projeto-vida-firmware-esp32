#include "sensors/SteeringSensor.h"
#include "utils/config.h"
#include "utils/utils.h"
#include "esp_timer.h"

SteeringSensor::SteeringSensor(uint8_t adc_pin)
    : adc_pin_(adc_pin),
      raw_adc_(0),
      raw_angle_deg_(0.0f),
      angle_deg_(0.0f),
      angular_velocity_(0.0f),
      filtered_angle_deg_(0.0f),      
      filter_initialized_(false),
      previous_update_us_(0),
      measurement_timestamp_us_(0),
      mux_(portMUX_INITIALIZER_UNLOCKED)
{
}

void SteeringSensor::begin()
{
    analogReadResolution(ADC_RESOLUTION_BITS);
    pinMode(adc_pin_, INPUT);
    previous_update_us_ = esp_timer_get_time();
    measurement_timestamp_us_ = previous_update_us_;
}

void SteeringSensor::update()
{
    const int64_t now_us = esp_timer_get_time();
    const int raw_adc = analogRead(adc_pin_);
    const float raw_angle_deg = mapFloat(raw_adc, 0.0f, 4095.0f, STEERING_MIN_DEG, STEERING_MAX_DEG);

    // Filtro passa-baixas simples.
    const float alpha = 0.15f;

    const bool had_previous_sample = filter_initialized_;
    if (!had_previous_sample)
    {
        filtered_angle_deg_ = raw_angle_deg;
        filter_initialized_ = true;
    }
    else
    {
        filtered_angle_deg_ = alpha * raw_angle_deg + (1.0f - alpha) * filtered_angle_deg_;
    }

    const float dt_s = (now_us - previous_update_us_) / 1000000.0f;
    const float raw_angular_velocity = had_previous_sample && dt_s > 0.0f
                                           ? (filtered_angle_deg_ - angle_deg_) / dt_s
                                           : 0.0f;
    const float angular_velocity = had_previous_sample
                                       ? alpha * raw_angular_velocity + (1.0f - alpha) * angular_velocity_
                                       : 0.0f;

    portENTER_CRITICAL(&mux_);
    raw_adc_ = raw_adc;
    raw_angle_deg_ = raw_angle_deg;
    angle_deg_ = filtered_angle_deg_;
    angular_velocity_ = angular_velocity;
    measurement_timestamp_us_ = now_us;
    portEXIT_CRITICAL(&mux_);
    previous_update_us_ = now_us;
}

int SteeringSensor::getRawAdc() const
{
    return raw_adc_;
}

float SteeringSensor::getRawAngleDeg() const
{
    return raw_angle_deg_;
}

float SteeringSensor::getFilteredAngleDeg() const
{
    return angle_deg_;
}

float SteeringSensor::getAngularVelocity() const{
    return angular_velocity_;
}

SteeringSensor::Snapshot SteeringSensor::getSnapshot()
{
    Snapshot snapshot{};
    portENTER_CRITICAL(&mux_);
    snapshot.timestamp_us = measurement_timestamp_us_;
    snapshot.raw_adc = static_cast<uint16_t>(raw_adc_);
    snapshot.raw_angle_deg = raw_angle_deg_;
    snapshot.filtered_angle_deg = angle_deg_;
    snapshot.filtered_angular_velocity_deg_s = angular_velocity_;
    portEXIT_CRITICAL(&mux_);
    return snapshot;
}
