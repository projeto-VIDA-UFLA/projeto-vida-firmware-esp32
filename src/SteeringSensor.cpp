#include "SteeringSensor.h"
#include "config.h"
#include "utils.h"

SteeringSensor::SteeringSensor(uint8_t adc_pin)
    : adc_pin_(adc_pin),
      raw_adc_(0),
      voltage_(0.0f),
      angle_deg_(0.0f),
      filtered_angle_deg_(0.0f),
      filter_initialized_(false)
{
}

void SteeringSensor::begin()
{
    analogReadResolution(ADC_RESOLUTION_BITS);
    pinMode(adc_pin_, INPUT);
    update();
}

void SteeringSensor::update()
{
    raw_adc_ = analogRead(adc_pin_);
    voltage_ = (static_cast<float>(raw_adc_) / ADC_MAX_VALUE) * ADC_REF_VOLTAGE;

    float measured_angle = mapFloat(
        voltage_,
        0.0f,
        ADC_REF_VOLTAGE,
        STEERING_MIN_DEG,
        STEERING_MAX_DEG
    );

    // Filtro passa-baixas simples.
    const float alpha = 0.15f;

    if (!filter_initialized_)
    {
        filtered_angle_deg_ = measured_angle;
        filter_initialized_ = true;
    }
    else
    {
        filtered_angle_deg_ = alpha * measured_angle + (1.0f - alpha) * filtered_angle_deg_;
    }

    angle_deg_ = filtered_angle_deg_;
}

int SteeringSensor::getRawAdc() const
{
    return raw_adc_;
}

float SteeringSensor::getVoltage() const
{
    return voltage_;
}

float SteeringSensor::getAngleDeg() const
{
    return angle_deg_;
}
