#include "sensors/SteeringSensor.h"
#include "utils/config.h"
#include "utils/utils.h"
#include "esp_timer.h"
#include <algorithm>
#include <cmath>
#include <cstdlib>

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
      measurement_valid_(false),
      rail_fault_(false),
      stuck_fault_(false),
      adc_history_{0, 0, 0},
      adc_history_count_(0),
      adc_history_index_(0),
      rail_since_us_(0),
      stuck_reference_adc_(-1),
      stuck_since_us_(0),
      mux_(portMUX_INITIALIZER_UNLOCKED)
{
}

float SteeringSensor::adcToAngleDeg(int raw_adc)
{
    // Sinal aplicado aos coeficientes em tempo de compilacao.
    constexpr float a = STEERING_ANGLE_SIGN * STEERING_POLY_A;
    constexpr float b = STEERING_ANGLE_SIGN * STEERING_POLY_B;
    constexpr float c = STEERING_ANGLE_SIGN * STEERING_POLY_C;
    constexpr float offset = STEERING_ANGLE_SIGN * STEERING_ZERO_OFFSET_DEG;

    // Subtracao inteira: no centro d e exatamente 0.
    const float d = static_cast<float>(raw_adc - STEERING_ADC_CENTER);

    // Horner: a*d + b*d^2 + c*d^3 = d*(a + d*(b + d*c)).
    // O "+ 0.0f" normaliza um eventual -0.0 para +0.0.
    return d * (a + d * (b + d * c)) - offset + 0.0f;
}

void SteeringSensor::begin()
{
    analogReadResolution(ADC_RESOLUTION_BITS);
    pinMode(adc_pin_, INPUT);
    previous_update_us_ = esp_timer_get_time();
    measurement_timestamp_us_ = 0;
    measurement_valid_ = false;
}

void SteeringSensor::update()
{
    const int64_t now_us = esp_timer_get_time();
    const int raw_adc = analogRead(adc_pin_);
    const int adc = medianAdc(raw_adc);
    const float raw_angle_deg = adcToAngleDeg(adc);
    const bool rail_fault = updateRailFault(adc, now_us);
    const bool stuck_fault = updateStuckFault(raw_adc, now_us);

    const bool had_previous_sample = filter_initialized_;
    if (!had_previous_sample)
    {
        filtered_angle_deg_ = raw_angle_deg;
        filter_initialized_ = true;
    }
    else
    {
        filtered_angle_deg_ = STEERING_FILTER_ALPHA * raw_angle_deg + (1.0f - STEERING_FILTER_ALPHA) * filtered_angle_deg_;
    }

    const float dt_s = (now_us - previous_update_us_) / 1000000.0f;
    
    const float raw_angular_velocity = had_previous_sample && dt_s > 0.0f
                                           ? (filtered_angle_deg_ - angle_deg_) / dt_s
                                           : 0.0f;

    const float angular_velocity = had_previous_sample
                                       ? STEERING_VELOCITY_FILTER_ALPHA * raw_angular_velocity + (1.0f - STEERING_VELOCITY_FILTER_ALPHA) * angular_velocity_
                                       : 0.0f;
    const bool measurement_valid =
        !rail_fault &&
        !stuck_fault &&
        std::isfinite(raw_angle_deg) &&
        std::isfinite(filtered_angle_deg_) &&
        std::isfinite(angular_velocity);

    portENTER_CRITICAL(&mux_);
    raw_adc_ = raw_adc;
    raw_angle_deg_ = raw_angle_deg;
    angle_deg_ = filtered_angle_deg_;
    angular_velocity_ = angular_velocity;
    measurement_timestamp_us_ = measurement_valid ? now_us : 0;
    measurement_valid_ = measurement_valid;
    rail_fault_ = rail_fault;
    stuck_fault_ = stuck_fault;
    portEXIT_CRITICAL(&mux_);
    previous_update_us_ = now_us;
}

int SteeringSensor::medianAdc(int raw_adc)
{
    adc_history_[adc_history_index_] = raw_adc;
    adc_history_index_ = static_cast<uint8_t>((adc_history_index_ + 1U) % 3U);
    if (adc_history_count_ < 3U)
    {
        ++adc_history_count_;
        return raw_adc;
    }

    const int a = adc_history_[0];
    const int b = adc_history_[1];
    const int c = adc_history_[2];
    return std::max(std::min(a, b), std::min(std::max(a, b), c));
}

bool SteeringSensor::updateRailFault(int adc, int64_t now_us)
{
    constexpr int64_t rail_time_us =
        static_cast<int64_t>(STEERING_ADC_RAIL_TIME_MS) * 1000LL;

    if (adc >= STEERING_ADC_VALID_MIN && adc <= STEERING_ADC_VALID_MAX)
    {
        rail_since_us_ = 0;
        return false;
    }

    if (rail_since_us_ == 0)
    {
        rail_since_us_ = now_us;
    }
    return (now_us - rail_since_us_) >= rail_time_us;
}

bool SteeringSensor::updateStuckFault(int raw_adc, int64_t now_us)
{
    constexpr int64_t stuck_time_us =
        static_cast<int64_t>(STEERING_ADC_STUCK_TIME_MS) * 1000LL;

    // Usa a leitura bruta: a mediana esconderia parte da oscilacao natural.
    if (stuck_reference_adc_ < 0 ||
        std::abs(raw_adc - stuck_reference_adc_) > STEERING_ADC_STUCK_TOLERANCE)
    {
        stuck_reference_adc_ = raw_adc;
        stuck_since_us_ = now_us;
        return false;
    }
    return (now_us - stuck_since_us_) >= stuck_time_us;
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

float SteeringSensor::getAngularVelocity() const
{
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
    snapshot.adc_rail_fault = rail_fault_;
    snapshot.adc_stuck_fault = stuck_fault_;
    snapshot.valid = measurement_valid_;
    portEXIT_CRITICAL(&mux_);
    return snapshot;
}
