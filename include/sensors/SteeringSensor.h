#pragma once
#include <Arduino.h>

/**
 * Sensor de esterçamento (LVDT no ADC), lido pela TaskSensors a cada
 * TASK_SENSOR_PERIOD_MS.
 *
 * Cadeia de medicao:
 *   ADC -> mediana das 3 ultimas leituras -> polinomio cubico (calibracao)
 *   - STEERING_ZERO_OFFSET_DEG -> sinal (STEERING_INVERT_SIGN) -> filtro
 *   passa-baixa (STEERING_FILTER_ALPHA) -> velocidade angular filtrada.
 *
 * A leitura e invalidada (valid = false, timestamp_us = 0) com o ADC fora de
 * [STEERING_ADC_VALID_MIN, STEERING_ADC_VALID_MAX] por STEERING_ADC_RAIL_TIME_MS
 * (desconexao, curto ou alem do curso) ou travado em
 * +-STEERING_ADC_STUCK_TOLERANCE por STEERING_ADC_STUCK_TIME_MS (sinal perdido).
 * Angulo do modelo de bicicleta, em graus, REP-103 (esquerda positivo).
 */
class SteeringSensor
{
public:
    struct Snapshot
    {
        int64_t timestamp_us;  // instante da leitura valida; 0 se invalida
        uint16_t raw_adc;      // leitura bruta, antes da mediana
        float raw_angle_deg;   // angulo da mediana, sem filtro
        float filtered_angle_deg;
        float filtered_angular_velocity_deg_s;
        bool adc_rail_fault;  // ADC no fundo/fim de escala
        bool adc_stuck_fault; // ADC sem variacao (sinal perdido)
        bool valid;
    };

    explicit SteeringSensor(uint8_t adc_pin);

    void begin();
    // Le o ADC e atualiza angulo, velocidade e validade.
    void update();

    int getRawAdc() const;
    float getRawAngleDeg() const;
    float getFilteredAngleDeg() const;
    float getAngularVelocity() const;
    Snapshot getSnapshot();

    // Converte a leitura bruta do ADC (0-4095) no angulo da roda virtual do
    // modelo de bicicleta, em graus, na convencao de STEERING_INVERT_SIGN.
    static float adcToAngleDeg(int raw_adc);

private:
    // Mediana das 3 ultimas leituras: remove glitches isolados do ADC.
    int medianAdc(int raw_adc);
    bool updateRailFault(int adc, int64_t now_us);
    bool updateStuckFault(int raw_adc, int64_t now_us);

    uint8_t adc_pin_;

    int raw_adc_;
    float raw_angle_deg_;
    float angle_deg_;
    float angular_velocity_;

    float filtered_angle_deg_;
    bool filter_initialized_;
    int64_t previous_update_us_;
    int64_t measurement_timestamp_us_;
    bool measurement_valid_;
    bool rail_fault_;
    bool stuck_fault_;

    int adc_history_[3];
    uint8_t adc_history_count_;
    uint8_t adc_history_index_;
    int64_t rail_since_us_;       // 0 = dentro da faixa
    int stuck_reference_adc_;     // < 0 = sem referencia
    int64_t stuck_since_us_;

    portMUX_TYPE mux_;
};
