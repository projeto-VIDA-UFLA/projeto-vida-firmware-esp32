#pragma once

#include <Arduino.h>

/**
 * Leitor de encoder incremental em quadratura.
 */
class EncoderReader
{
public:
    struct Snapshot
    {
        int64_t timestamp_us;
        int64_t count;
        float raw_velocity_m_s;
        float filtered_velocity_m_s;
        float raw_acceleration_m_s2;
        float filtered_acceleration_m_s2;
    };

    /**
     * @brief Construtor da classe EncoderReader.
     *
     * @param pin_a Pino conectado ao canal A do encoder.
     * @param pin_b Pino conectado ao canal B do encoder.
     * @param tick_per_rev Quantidade de ticks por revolução do encoder.
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
    float getAcceleration() const;    // Retorna aceleração filtrada em m/²

    void handleInterrupt(); // Trata a interrupção do encoder

    const char *getName() const; // Retorna o nome do encoder

private:
    uint8_t pin_a_;      // Pino do canal A
    uint8_t pin_b_;      // Pino do canal B
    float tick_per_rev_; // Qtd de pulso por resolução
    const char *name_;   // Nome do encoder

    // Dados atualizados pela interrupção
    volatile int64_t count_;
    volatile long delta_;
    volatile int64_t last_timestamp_;
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
    int64_t measurement_count_;

    // Proteção para acesso entre ISR e task
    portMUX_TYPE mux_;

    uint8_t readState() const; // Lê estado dos canais A/B

    float ticksToDistance(int64_t ticks) const; // Converte ticks em distância

    float calculateVelocity(int64_t delta_ticks, float dt) const;                                 // Calcula velocidade
    float calculateAcceleration(float current_velocity, float previous_velocity, float dt) const; // Calcula aceleração

    float applyLowPassFilter(float raw_value, float filtered_value, float alpha) const; // Aplica filtro passa-baixa
};
