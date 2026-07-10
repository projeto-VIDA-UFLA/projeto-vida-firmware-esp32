#include "sensors/EncoderReader.h"
#include "utils/config.h"

#include "esp_timer.h"

// ============================================================
// Construtor
// ============================================================

/**
 * @brief Cria uma instância do leitor de encoder.
 *
 * Cada objeto EncoderReader representa um encoder físico do veículo.
 * O construtor apenas armazena as configurações recebidas e inicializa
 * as variáveis internas com valores seguros.
 *
 * A configuração dos pinos e a leitura inicial do estado do encoder são
 * feitas posteriormente no método begin().
 *
 * @param pin_a Pino do canal A do encoder.
 * @param pin_b Pino do canal B do encoder.
 * @param tick_per_rev Quantidade de ticks por revolução.
 * @param name Nome identificador do encoder.
 */
EncoderReader::EncoderReader(uint8_t pin_a, uint8_t pin_b, float tick_per_rev, const char *name)
    : pin_a_(pin_a),
      pin_b_(pin_b),
      tick_per_rev_(tick_per_rev),
      name_(name),

      // Variáveis atualizadas pela interrupção
      count_(0),
      delta_(0),
      last_timestamp_(0),
      last_state_(0),

      // Variáveis usadas nos cálculos periódicos
      previous_count_(0),
      previous_update_us_(0),
      velocity_(0.0f),
      acceleration_(0.0f),
      raw_velocity_(0.0f),
      raw_acceleration_(0.0f),
      previous_velocity_(0.0f),
      has_previous_update_(false),

      // Proteção para acesso concorrente entre ISR e task
      mux_(portMUX_INITIALIZER_UNLOCKED)
{
}

// ============================================================
// Inicialização
// ============================================================

/**
 * @brief Inicializa os pinos e estados internos do encoder.
 *
 * Essa função deve ser chamada antes do uso do encoder, normalmente
 * durante a inicialização do firmware.
 *
 * Ela configura os canais A e B como entrada, lê o estado inicial do encoder
 * e inicializa os valores usados nos cálculos de velocidade e aceleração.
 */
void EncoderReader::begin()
{
    pinMode(pin_a_, INPUT);
    pinMode(pin_b_, INPUT);

    last_state_ = readState();
    last_timestamp_ = esp_timer_get_time();

    previous_update_us_ = last_timestamp_;
    previous_count_ = 0;

    velocity_ = 0.0f;
    acceleration_ = 0.0f;
    raw_velocity_ = 0.0f;
    raw_acceleration_ = 0.0f;
    previous_velocity_ = 0.0f;

    has_previous_update_ = false;
}

// ============================================================
// Leitura dos canais do encoder
// ============================================================

/**
 * @brief Lê o estado atual dos canais A e B.
 *
 * O estado é codificado em dois bits:
 *
 * - bit 1: canal A;
 * - bit 0: canal B.
 *
 * Essa codificação facilita a comparação entre o estado anterior e o estado
 * atual na tabela de transição da quadratura.
 *
 * @return Estado atual codificado dos canais A e B.
 */
uint8_t EncoderReader::readState() const
{
    uint8_t a = digitalRead(pin_a_);
    uint8_t b = digitalRead(pin_b_);

    return (a << 1) | b;
}

// ============================================================
// Tratamento da interrupção
// ============================================================

/**
 * @brief Atualiza a contagem do encoder a partir da interrupção.
 *
 * Essa função deve ser chamada pela ISR associada aos canais do encoder.
 *
 * Ela lê o estado atual dos canais A e B, compara com o estado anterior e
 * usa uma tabela de transição para identificar se houve avanço, retorno ou
 * transição inválida.
 *
 * Como essa função roda em contexto de interrupção, ela deve permanecer curta.
 * Por isso, ela apenas:
 *
 * - lê o estado atual;
 * - calcula a transição;
 * - atualiza count_ e delta_;
 * - salva o timestamp da última transição válida.
 */
void EncoderReader::handleInterrupt()
{
    uint8_t current_state = readState();

    /**
     * Tabela de decodificação em quadratura x4.
     *
     * O índice é formado por:
     *
     *     estado_anterior << 2 | estado_atual
     *
     * Resultado:
     *
     * -  1: avanço de um tick;
     * - -1: retorno de um tick;
     * -  0: transição inválida ou sem movimento.
     */
    static const int8_t transition_table[16] = {
        0, -1, 1, 0,
        1, 0, 0, -1,
        -1, 0, 0, 1,
        0, 1, -1, 0};

    uint8_t index = (last_state_ << 2) | current_state;
    int8_t step = transition_table[index];

    if (step != 0)
    {
        portENTER_CRITICAL_ISR(&mux_);

        count_ += step;
        delta_ += step;
        last_timestamp_ = esp_timer_get_time();

        portEXIT_CRITICAL_ISR(&mux_);
    }

    last_state_ = current_state;
}

// ============================================================
// Atualização de velocidade e aceleração
// ============================================================

/**
 * @brief Atualiza os valores de velocidade e aceleração.
 *
 * Essa função deve ser chamada periodicamente por uma task.
 *
 * Ela lê a contagem atual do encoder, calcula quantos ticks ocorreram desde
 * a última atualização e converte essa variação em velocidade linear.
 *
 * Depois, calcula a aceleração com base na diferença entre a velocidade atual
 * e a velocidade anterior.
 *
 * Os valores brutos são armazenados separadamente e, em seguida, passam por
 * um filtro passa-baixa para gerar valores mais estáveis para controle.
 */
void EncoderReader::update()
{
    int64_t count = 0;
    int64_t now_us = esp_timer_get_time();

    // Lê a contagem de forma protegida, pois count_ é alterado pela ISR
    portENTER_CRITICAL(&mux_);
    count = count_;
    portEXIT_CRITICAL(&mux_);

    // Na primeira execução, apenas cria a referência inicial
    if (!has_previous_update_)
    {
        previous_count_ = count;
        previous_update_us_ = now_us;
        has_previous_update_ = true;
        return;
    }

    float dt_s = (now_us - previous_update_us_) / 1000000.0f;

    if (dt_s <= 0.0f)
    {
        return;
    }

    int64_t delta_ticks = count - previous_count_;

    // Velocidade bruta calculada diretamente pelos ticks
    raw_velocity_ = calculateVelocity(delta_ticks, dt_s);

    // Velocidade filtrada para uso em controle
    velocity_ = applyLowPassFilter(raw_velocity_, velocity_, DRIVE_VELOCITY_FILTER_ALPHA);

    // Aceleração bruta calculada a partir da velocidade filtrada
    raw_acceleration_ = calculateAcceleration(velocity_, previous_velocity_, dt_s);

    // Aceleração filtrada
    acceleration_ = applyLowPassFilter(raw_acceleration_, acceleration_, DRIVE_ACCEL_FILTER_ALPHA);

    previous_velocity_ = velocity_;
    previous_count_ = count;
    previous_update_us_ = now_us;
}

// ============================================================
// Leitura segura dos dados do encoder
// ============================================================

/**
 * @brief Retorna a contagem total acumulada do encoder.
 *
 * A leitura é protegida porque count_ pode ser alterado pela interrupção.
 *
 * @return Contagem total de ticks.
 */
long EncoderReader::getCount()
{
    portENTER_CRITICAL(&mux_);
    long value = static_cast<long>(count_);
    portEXIT_CRITICAL(&mux_);

    return value;
}

/**
 * @brief Retorna uma cópia consistente da contagem e do timestamp.
 *
 * Essa função permite ler count_ e last_timestamp_ juntos, evitando que uma
 * interrupção altere um deles durante a leitura.
 *
 * @param count Variável de saída para a contagem atual.
 * @param timestamp_us Variável de saída para o timestamp da última transição.
 */
void EncoderReader::getSnapshot(int64_t &count, int64_t &timestamp_us)
{
    portENTER_CRITICAL(&mux_);

    count = count_;
    timestamp_us = last_timestamp_;

    portEXIT_CRITICAL(&mux_);
}

/**
 * @brief Retorna o delta de ticks e zera o acumulador.
 *
 * Essa função é útil para trabalhar com deslocamento incremental.
 *
 * Enquanto getCount() retorna a contagem total, getAndResetDelta() retorna
 * apenas a variação acumulada desde a última chamada.
 *
 * @return Delta de ticks desde a última leitura.
 */
long EncoderReader::getAndResetDelta()
{
    portENTER_CRITICAL(&mux_);

    long value = delta_;
    delta_ = 0;

    portEXIT_CRITICAL(&mux_);

    return value;
}

// ============================================================
// Reset
// ============================================================

/**
 * @brief Zera os contadores e reinicia os estados internos.
 *
 * Essa função pode ser usada em rotinas de calibração, reinicialização do
 * firmware ou mudança de estado operacional do sistema.
 *
 * Ela zera a contagem total, o delta, velocidades, aceleração e força a próxima
 * chamada de update() a recriar a referência inicial.
 */
void EncoderReader::reset()
{
    int64_t now_us = esp_timer_get_time();

    portENTER_CRITICAL(&mux_);
    count_ = 0;
    delta_ = 0;
    last_timestamp_ = now_us;
    portEXIT_CRITICAL(&mux_);

    previous_count_ = 0;
    previous_update_us_ = now_us;

    velocity_ = 0.0f;
    acceleration_ = 0.0f;
    raw_velocity_ = 0.0f;
    raw_acceleration_ = 0.0f;
    previous_velocity_ = 0.0f;

    has_previous_update_ = false;
}

// ============================================================
// Getters
// ============================================================

/**
 * @brief Retorna a velocidade filtrada.
 *
 * @return Velocidade em m/s.
 */
float EncoderReader::getVelocity() const
{
    return velocity_;
}

/**
 * @brief Retorna a aceleração filtrada.
 *
 * @return Aceleração em m/s².
 */
float EncoderReader::getAcceleration() const
{
    return acceleration_;
}

/**
 * @brief Retorna a velocidade bruta.
 *
 * @return Velocidade bruta em m/s.
 */
float EncoderReader::getRawVelocity() const
{
    return raw_velocity_;
}

/**
 * @brief Retorna a aceleração bruta.
 *
 * @return Aceleração bruta em m/s².
 */
float EncoderReader::getRawAcceleration() const
{
    return raw_acceleration_;
}

/**
 * @brief Retorna o nome identificador do encoder.
 *
 * @return Nome do encoder.
 */
const char *EncoderReader::getName() const
{
    return name_;
}

// ============================================================
// Conversões e cálculos auxiliares
// ============================================================

/**
 * @brief Converte ticks do encoder em distância linear.
 *
 * A distância é calculada considerando o perímetro da roda:
 *
 *     perímetro = PI * diâmetro
 *
 * Depois, a fração de revolução representada pelos ticks é multiplicada pelo
 * perímetro da roda.
 *
 * @param ticks Quantidade de ticks.
 *
 * @return Distância em metros.
 */
float EncoderReader::ticksToDistance(int64_t ticks) const
{
    float wheel_circumference_m = PI * DRIVE_WHEEL_DIAMETER_M;

    return (static_cast<float>(ticks) / tick_per_rev_) * wheel_circumference_m;
}

/**
 * @brief Calcula a velocidade linear.
 *
 * A velocidade é calculada pela distância percorrida dividida pelo intervalo
 * de tempo.
 *
 * @param delta_ticks Variação de ticks no intervalo.
 * @param dt_s Intervalo de tempo em segundos.
 *
 * @return Velocidade em m/s.
 */
float EncoderReader::calculateVelocity(int64_t delta_ticks, float dt_s) const
{
    if (dt_s <= 0.0f)
    {
        return 0.0f;
    }

    return ticksToDistance(delta_ticks) / dt_s;
}

/**
 * @brief Calcula a aceleração linear.
 *
 * A aceleração é calculada pela diferença entre duas velocidades consecutivas
 * dividida pelo intervalo de tempo.
 *
 * @param current_velocity_m_s Velocidade atual em m/s.
 * @param previous_velocity_m_s Velocidade anterior em m/s.
 * @param dt_s Intervalo de tempo em segundos.
 *
 * @return Aceleração em m/s².
 */
float EncoderReader::calculateAcceleration(float current_velocity_m_s, float previous_velocity_m_s, float dt_s) const
{
    if (dt_s <= 0.0f)
    {
        return 0.0f;
    }

    return (current_velocity_m_s - previous_velocity_m_s) / dt_s;
}

/**
 * @brief Aplica um filtro passa-baixa de primeira ordem.
 *
 * Fórmula usada:
 *
 *     valor_filtrado = alpha * valor_bruto + (1 - alpha) * valor_filtrado_anterior
 *
 * Quanto maior o alpha, mais rápido o valor filtrado acompanha o valor bruto.
 * Quanto menor o alpha, mais suave fica o sinal, porém com maior atraso.
 *
 * @param raw_value Valor bruto atual.
 * @param filtered_value Valor filtrado anterior.
 * @param alpha Fator de suavização.
 *
 * @return Novo valor filtrado.
 */
float EncoderReader::applyLowPassFilter(
    float raw_value,
    float filtered_value,
    float alpha) const
{
    return alpha * raw_value + (1.0f - alpha) * filtered_value;
}