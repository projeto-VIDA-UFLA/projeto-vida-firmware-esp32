#pragma once

#include <Arduino.h>
#include <Wire.h>

#include "utils/config.h"
#include "esp_timer.h"

/**
 * Monitor do banco de baterias de tracao (INA228 no I2C), lido pela
 * TaskBattery a cada BATTERY_MEASUREMENT_PERIOD_MS (nucleo 0, prioridade
 * baixa: o I2C pode bloquear ate BATTERY_I2C_TIMEOUT_MS e nao pode atrasar as
 * tasks do nucleo 1).
 *
 * Cadeia de medicao:
 *   VBUS  -> tensao do banco (195,3125 uV/LSB)
 *   VSHUNT / BATTERY_SHUNT_RESISTANCE_OHM -> corrente (312,5 nV/LSB, +-163,84 mV)
 *   tensao -> filtro passa-baixa (BATTERY_VOLTAGE_FILTER_ALPHA) -> SoC linear
 *   entre BATTERY_SOC_EMPTY_V e BATTERY_SOC_FULL_V -> estado (emergencia com
 *   SoC <= BATTERY_SOC_EMERGENCY_PERCENT).
 *
 * Sem resposta do INA228 a leitura fica invalida e o sensor e procurado de
 * novo no ciclo seguinte (pode ser ligado com o firmware rodando). O estado e
 * so informativo: o SafetyManager nao o usa.
 */
class BatteryMonitor
{
public:
    enum class State : uint8_t
    {
        Invalid = 0,
        Normal = 1,
        Emergency = 2
    };

    struct Snapshot
    {
        int64_t sample_timestamp_us; // instante da leitura, valida ou nao; 0 = nenhuma
        float voltage_v;             // sem filtro
        float current_a;             // NaN sem shunt configurado
        float soc_percent;
        State state;
        bool valid;
    };

    BatteryMonitor(uint8_t i2c_address, int sda, int scl, TwoWire &wire = Wire);

    // Inicia o barramento I2C e procura o INA228 (falha nao impede o boot).
    void begin();
    // Uma leitura completa; chamada pela TaskBattery.
    void update();
    Snapshot getSnapshot();

    // Conversoes dos registradores (24 bits, dado nos bits 23:4).
    static float vbusRawToVolts(uint32_t raw);
    static float vshuntRawToVolts(uint32_t raw); // complemento de 2, 20 bits
    static float socFromVoltage(float voltage_v);

private:
    static constexpr uint8_t REG_ADC_CONFIG = 0x01;
    static constexpr uint8_t REG_VSHUNT = 0x04;
    static constexpr uint8_t REG_VBUS = 0x05;
    static constexpr uint8_t REG_MANUFACTURER_ID = 0x3E;
    static constexpr uint8_t REG_DEVICE_ID = 0x3F;

    static constexpr uint16_t EXPECTED_MANUFACTURER_ID = 0x5449; // "TI"
    static constexpr uint16_t EXPECTED_DEVICE_ID_MASK = 0xFFF0;
    static constexpr uint16_t EXPECTED_DEVICE_ID = 0x2280;
    // Continuo so VSHUNT e VBUS, 1052 us por conversao, media de 16:
    // ~34 ms por leitura, filtra o chaveamento da controladora.
    static constexpr uint16_t ADC_CONFIG_VALUE = 0xBB6A;

    static constexpr float VBUS_LSB_V = 195.3125e-6f;
    static constexpr float VSHUNT_LSB_V = 312.5e-9f;

    bool probe();
    void storeInvalid(int64_t now_us);

    bool readRegister16(uint8_t reg, uint16_t &value);
    bool readRegister24(uint8_t reg, uint32_t &value);
    bool writeRegister16(uint8_t reg, uint16_t value);

    uint8_t i2c_address_;
    int sda_;
    int scl_;
    TwoWire &wire_;

    bool initialized_; // INA228 encontrado e configurado
    bool filter_initialized_;
    float filtered_voltage_v_;

    Snapshot snapshot_;
    portMUX_TYPE mux_;
};
