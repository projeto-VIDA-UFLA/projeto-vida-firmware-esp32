#pragma once

#include <Arduino.h>

/**
 * Leitor de encoder incremental em quadratura (decodificacao x4).
 *
 * A ISR (handleInterrupt, nos dois canais) atualiza a contagem e o instante
 * da ultima borda. update(), chamado pela TaskSensors a cada
 * TASK_SENSOR_PERIOD_MS, estima a velocidade linear da roda
 * (DRIVE_WHEEL_DIAMETER_M) pelo metodo M/T: ticks de uma janela de pelo menos
 * ENCODER_VELOCITY_WINDOW_MS divididos pelo tempo exato entre bordas. Depois
 * aplica filtros passa-baixa de primeira ordem a velocidade e a aceleracao.
 *
 * Velocidade positiva = sentido de contagem positivo do encoder.
 */
class EncoderReader
{
public:
    // Medicao coerente de um mesmo update().
    struct Snapshot
    {
        int64_t timestamp_us; // instante do update() (esp_timer)
        int64_t count;        // ticks acumulados
        float raw_velocity_m_s;          // M/T, atualizada a cada janela
        float filtered_velocity_m_s;     // DRIVE_VELOCITY_FILTER_ALPHA
        float raw_acceleration_m_s2;     // derivada da velocidade filtrada
        float filtered_acceleration_m_s2; // DRIVE_ACCEL_FILTER_ALPHA
    };

    /**
     * @brief Construtor da classe EncoderReader.
     *
     * @param pin_a Pino conectado ao canal A do encoder.
     * @param pin_b Pino conectado ao canal B do encoder.
     * @param tick_per_rev Ticks por revolucao da roda (contagem x4).
     * @param name Nome identificador do encoder.
     */
    EncoderReader(uint8_t pin_a, uint8_t pin_b, float tick_per_rev, const char *name);

    // Inicializa pinos e estados internos. Com isr, liga a interrupcao dos
    // dois canais (CHANGE) a ela; a isr deve chamar handleInterrupt().
    void begin(void (*isr)() = nullptr);
    void update(); // Atualiza velocidade e aceleração

    Snapshot getMeasurementSnapshot(); // Dados do mesmo ciclo de calculo.
    long getAndResetDelta();                                 // Retorna o delta de ticks e zera o acumulador
    long getCount();                                         // Retorna a contagem total de ticks
    void reset();                                            // Zera contadores e estados internos

    float getRawVelocity() const;     // Retorna velocidade bruta em m/s
    float getVelocity() const;        // Retorna velocidade filtrada em m/s
    float getRawAcceleration() const; // Retorna aceleração bruta em m/s²
    float getAcceleration() const;    // Retorna aceleração filtrada em m/s²

    void handleInterrupt(); // Trata a interrupção do encoder

    const char *getName() const; // Retorna o nome do encoder

private:
    uint8_t pin_a_;      // Pino do canal A
    uint8_t pin_b_;      // Pino do canal B
    float tick_per_rev_; // Ticks por revolução
    const char *name_;   // Nome do encoder

    // Dados atualizados pela interrupção
    volatile int64_t count_;
    volatile long delta_;
    volatile int64_t last_timestamp_; // instante da ultima borda valida
    volatile uint8_t last_state_;

    // Dados para cálculo de velocidade/aceleração
    int64_t previous_count_;
    int64_t previous_update_us_;
    float velocity_;
    float acceleration_;
    float raw_velocity_;
    float raw_acceleration_;
    float previous_velocity_;
    bool has_previous_update_;
    int64_t measurement_timestamp_us_;

    // Janela do metodo M/T
    bool window_active_;      // false: parado, espera a proxima borda
    int64_t window_count_;    // contagem na borda que abriu a janela
    int64_t window_edge_us_;  // instante dessa borda
    int64_t measurement_count_;

    // Proteção para acesso entre ISR e task
    portMUX_TYPE mux_;

    uint8_t readState() const; // Lê estado dos canais A/B

    float ticksToDistance(int64_t ticks) const; // Converte ticks em distância

    float calculateVelocity(int64_t delta_ticks, float dt) const;                                 // Calcula velocidade
    float estimateVelocity(int64_t count, int64_t edge_us, int64_t now_us);                     // Velocidade M/T
    float calculateAcceleration(float current_velocity, float previous_velocity, float dt) const; // Calcula aceleração

    float applyLowPassFilter(float raw_value, float filtered_value, float alpha) const; // Aplica filtro passa-baixa
};
