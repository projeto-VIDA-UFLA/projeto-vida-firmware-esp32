#include "sensors/BatteryMonitor.h"

#include <cmath>

static_assert(BATTERY_SOC_FULL_V > BATTERY_SOC_EMPTY_V,
              "BATTERY_SOC_FULL_V deve ser maior que BATTERY_SOC_EMPTY_V");
static_assert(BATTERY_SHUNT_RESISTANCE_OHM >= 0.0f,
              "BATTERY_SHUNT_RESISTANCE_OHM: 0 = sem shunt configurado");

BatteryMonitor::BatteryMonitor(uint8_t i2c_address, int sda, int scl, TwoWire &wire)
    : i2c_address_(i2c_address),
      sda_(sda),
      scl_(scl),
      wire_(wire),
      initialized_(false),
      filter_initialized_(false),
      filtered_voltage_v_(0.0f),
      snapshot_{0, NAN, NAN, NAN, State::Invalid, false},
      mux_(portMUX_INITIALIZER_UNLOCKED)
{
}

void BatteryMonitor::begin()
{
    wire_.begin(sda_, scl_, BATTERY_I2C_FREQUENCY_HZ);
    wire_.setTimeOut(BATTERY_I2C_TIMEOUT_MS);
    initialized_ = probe();
}

void BatteryMonitor::update()
{
    const int64_t now_us = esp_timer_get_time();

    if (!initialized_)
    {
        initialized_ = probe();
        if (!initialized_)
        {
            storeInvalid(now_us);
            return;
        }
    }

    uint32_t raw_vbus = 0;
    uint32_t raw_vshunt = 0;
    if (!readRegister24(REG_VBUS, raw_vbus) || !readRegister24(REG_VSHUNT, raw_vshunt))
    {
        // Sensor sumiu (cabo, alimentacao): procura e reconfigura de novo.
        initialized_ = false;
        storeInvalid(now_us);
        return;
    }

    const float voltage_v = vbusRawToVolts(raw_vbus);
    if (!std::isfinite(voltage_v) || voltage_v > BATTERY_VOLTAGE_VALID_MAX_V)
    {
        storeInvalid(now_us);
        return;
    }

    const float current_a =
        BATTERY_SHUNT_RESISTANCE_OHM > 0.0f
            ? BATTERY_CURRENT_SIGN * vshuntRawToVolts(raw_vshunt) / BATTERY_SHUNT_RESISTANCE_OHM
            : NAN;

    // O SoC usa a tensao filtrada; a publicada e a sem filtro.
    filtered_voltage_v_ =
        filter_initialized_
            ? BATTERY_VOLTAGE_FILTER_ALPHA * voltage_v +
                  (1.0f - BATTERY_VOLTAGE_FILTER_ALPHA) * filtered_voltage_v_
            : voltage_v;
    filter_initialized_ = true;

    const float soc_percent = socFromVoltage(filtered_voltage_v_);
    const State state =
        soc_percent <= BATTERY_SOC_EMERGENCY_PERCENT ? State::Emergency : State::Normal;

    portENTER_CRITICAL(&mux_);
    snapshot_ = Snapshot{now_us, voltage_v, current_a, soc_percent, state, true};
    portEXIT_CRITICAL(&mux_);
}

BatteryMonitor::Snapshot BatteryMonitor::getSnapshot()
{
    portENTER_CRITICAL(&mux_);
    const Snapshot snapshot = snapshot_;
    portEXIT_CRITICAL(&mux_);
    return snapshot;
}

float BatteryMonitor::vbusRawToVolts(uint32_t raw)
{
    return static_cast<float>((raw >> 4U) & 0xFFFFFU) * VBUS_LSB_V;
}

float BatteryMonitor::vshuntRawToVolts(uint32_t raw)
{
    int32_t value = static_cast<int32_t>((raw >> 4U) & 0xFFFFFU);
    if ((value & 0x80000) != 0)
    {
        value -= 0x100000;
    }
    return static_cast<float>(value) * VSHUNT_LSB_V;
}

float BatteryMonitor::socFromVoltage(float voltage_v)
{
    const float soc = (voltage_v - BATTERY_SOC_EMPTY_V) /
                      (BATTERY_SOC_FULL_V - BATTERY_SOC_EMPTY_V) * 100.0f;
    return soc < 0.0f ? 0.0f : (soc > 100.0f ? 100.0f : soc);
}

bool BatteryMonitor::probe()
{
    uint16_t manufacturer_id = 0;
    uint16_t device_id = 0;
    if (!readRegister16(REG_MANUFACTURER_ID, manufacturer_id) ||
        !readRegister16(REG_DEVICE_ID, device_id) ||
        manufacturer_id != EXPECTED_MANUFACTURER_ID ||
        (device_id & EXPECTED_DEVICE_ID_MASK) != EXPECTED_DEVICE_ID)
    {
        return false;
    }

    // Filtro da tensao recomeca: o banco pode ter sido trocado.
    filter_initialized_ = false;
    return writeRegister16(REG_ADC_CONFIG, ADC_CONFIG_VALUE);
}

void BatteryMonitor::storeInvalid(int64_t now_us)
{
    portENTER_CRITICAL(&mux_);
    snapshot_ = Snapshot{now_us, NAN, NAN, NAN, State::Invalid, false};
    portEXIT_CRITICAL(&mux_);
}

bool BatteryMonitor::readRegister16(uint8_t reg, uint16_t &value)
{
    wire_.beginTransmission(i2c_address_);
    wire_.write(reg);
    if (wire_.endTransmission(false) != 0 ||
        wire_.requestFrom(i2c_address_, static_cast<uint8_t>(2)) != 2)
    {
        return false;
    }

    value = static_cast<uint16_t>(wire_.read()) << 8U;
    value |= static_cast<uint16_t>(wire_.read());
    return true;
}

bool BatteryMonitor::readRegister24(uint8_t reg, uint32_t &value)
{
    wire_.beginTransmission(i2c_address_);
    wire_.write(reg);
    if (wire_.endTransmission(false) != 0 ||
        wire_.requestFrom(i2c_address_, static_cast<uint8_t>(3)) != 3)
    {
        return false;
    }

    value = static_cast<uint32_t>(wire_.read()) << 16U;
    value |= static_cast<uint32_t>(wire_.read()) << 8U;
    value |= static_cast<uint32_t>(wire_.read());
    return true;
}

bool BatteryMonitor::writeRegister16(uint8_t reg, uint16_t value)
{
    wire_.beginTransmission(i2c_address_);
    wire_.write(reg);
    wire_.write(static_cast<uint8_t>(value >> 8U));
    wire_.write(static_cast<uint8_t>(value & 0xFFU));
    return wire_.endTransmission() == 0;
}
