#pragma once

#include <Arduino.h>
#include "actuators/MotorHardware.h"

/**
 * Motor de tracao 36 V acionado por driver de potencia: 1 PWM de velocidade e
 * 1 pino de direcao (DIR, HIGH = re).
 *
 * A troca de sentido so acontece com o veiculo parado:
 *   DRIVING -> WAITING_FOR_STOP (PWM 0, DIR mantido)
 *           -> velocidade <= DRIVE_STOP_SPEED_THRESHOLD_MPS por
 *              DRIVE_STOP_CONFIRMATION_TIME_MS, com medicao de ate
 *              DRIVE_VELOCITY_MEASUREMENT_TIMEOUT_MS
 *           -> DIR trocado com PWM 0 -> DIRECTION_DEADTIME
 *              (DRIVE_DIRECTION_DEADTIME_MS sem torque) -> DRIVING.
 *
 * A re ainda nao funciona no hardware (falta o circuito de reles na saida do
 * driver); a logica de inversao ja esta pronta.
 */
class DriveMotor
{
public:
    enum class State : uint8_t
    {
        STOPPED,           // PWM 0
        DRIVING,           // PWM aplicado no sentido de DIR
        WAITING_FOR_STOP,  // pedido de sentido oposto: PWM 0 ate parar
        DIRECTION_DEADTIME // DIR acabou de mudar: PWM 0 por um tempo
    };

    // Velocidade usada para autorizar a troca de sentido.
    struct VelocityMeasurement
    {
        float velocity_m_s;
        int64_t timestamp_us; // instante da amostra (esp_timer)
        bool valid;
    };

    struct Snapshot
    {
        int pwm;                       // duty aplicado (0-255)
        bool reverse;                  // nivel atual do pino DIR
        int64_t timestamp_us;          // instante da ultima escrita
        State state;
        bool direction_change_pending; // state == WAITING_FOR_STOP
    };

    DriveMotor(uint8_t pwm_pin, uint8_t direction_pin, uint8_t pwm_channel,
               MotorHardware *hardware = nullptr);

    // Configura DIR (frente) antes do PWM e parte parado.
    void begin();

    // Comando normalizado [-1, 1]: > 0 frente, < 0 re, 0 para.
    // Comando nao finito para o motor.
    void setCommand(float normalized_command,
                    const VelocityMeasurement &velocity_measurement);

    // Zera o PWM. Nunca altera DIR: o veiculo pode estar em movimento.
    void stop();

    float getLastCommand() const;
    int getLastPwm() const;
    bool isReverse() const;
    State getState() const;
    Snapshot getSnapshot() const;

private:
    bool isVelocityFreshAndStopped(const VelocityMeasurement &measurement,
                                   int64_t now_us) const;
    void writePwmZero();
    void storeSnapshot(float command, int pwm, State state,
                       int64_t timestamp_us);

    uint8_t pwm_pin_;
    uint8_t direction_pin_;
    uint8_t pwm_channel_;

    float last_command_;
    int last_pwm_;
    bool reverse_;
    int64_t pwm_timestamp_us_;
    State state_;
    bool pending_reverse_;
    int64_t stopped_since_us_;
    int64_t direction_changed_at_us_;
    MotorHardware &hardware_;

    // Protege a leitura/escrita coerente do comando, PWM, direcao e timestamp.
    mutable portMUX_TYPE snapshot_mux_ = portMUX_INITIALIZER_UNLOCKED;
};
