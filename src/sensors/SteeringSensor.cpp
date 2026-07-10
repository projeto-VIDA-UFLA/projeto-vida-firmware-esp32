#include "sensors/SteeringSensor.h"
#include "utils/config.h"
#include "utils/utils.h"

SteeringSensor::SteeringSensor(uint8_t adc_pin)
    : adc_pin_(adc_pin),
      raw_adc_(0),
      angle_deg_(0.0f),
      angular_velocity_(0.0f),
      filtered_angle_deg_(0.0f),      
      filter_initialized_(false)
{
}

void SteeringSensor::begin()
{
    analogReadResolution(ADC_RESOLUTION_BITS);
    pinMode(adc_pin_, INPUT);
}

void SteeringSensor::update()
{
    raw_adc_ = analogRead(adc_pin_);
    raw_angle_deg_ = mapFloat(raw_adc_, 0.0f, 4095.0f, STEERING_MIN_DEG, STEERING_MAX_DEG);

    // Filtro passa-baixas simples.
    const float alpha = 0.15f;

    if (!filter_initialized_)
    {
        filtered_angle_deg_ = raw_angle_deg_;
        filter_initialized_ = true;
    }
    else
    {
        filtered_angle_deg_ = alpha * raw_angle_deg_ + (1.0f - alpha) * filtered_angle_deg_;
    }

    angle_deg_ = filtered_angle_deg_;

    // TODO: implementar calculo de velocidade angular
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
