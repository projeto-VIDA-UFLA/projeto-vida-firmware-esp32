#pragma once
#include <Arduino.h>
#include "actuators/MotorHardware.h"

/**
 * Servo de hobby (50 Hz) que puxa o cabo de aco do freio a disco traseiro.
 *
 * Posicao normalizada: 0 = solto, 1 = freio total, convertida linearmente em
 * pulso entre BRAKE_RELEASED_US e BRAKE_APPLIED_US.
 *
 * Modos:
 * - normal: a posicao segue setCommand() sem rampa;
 * - rampa de falha: sobe ate freio total a BRAKE_FAULT_RATE_PER_S;
 * - alivio parado: com o veiculo parado por BRAKE_HOLD_CONFIRMATION_MS o pulso
 *   e limitado a BRAKE_HOLD_US para nao forcar o servo. A posicao comandada
 *   nao muda; qualquer movimento volta ao pulso total.
 *
 * Sem pulsos o servo fica solto e o freio e liberado: esta classe nao cobre
 * reset do ESP32 nem queda da alimentacao do servo. Com BRAKE_CALIBRATED = 0
 * nenhum pulso e enviado.
 */
class BrakeServo
{
public:
    struct Snapshot
    {
        float position;
        int pulse_us; // 0 quando nenhum pulso e enviado (nao calibrado)
        bool fault_ramp_active;
        bool hold_active; // veiculo parado: pulso limitado a BRAKE_HOLD_US
        int64_t timestamp_us;
    };

    BrakeServo(uint8_t pin, uint8_t channel, MotorHardware *hardware = nullptr);

    // Freio total imediato: na partida o veiculo esta parado.
    void begin();

    // Operacao normal: a posicao segue o comando sem rampa.
    // Comando invalido ou fora de [0, 1] inicia a rampa de falha.
    void setCommand(float normalized_command);

    // Inicia a rampa ate freio total. Chamadas repetidas mantem a rampa.
    void applyFaultRamp();

    // Emergencia do supervisor de tasks: pulso de freio total direto no
    // hardware, sem tocar no estado (pode ser chamado sem o mutex).
    void forceFullBrake();

    // Avanca a rampa de falha e atualiza o alivio de veiculo parado.
    // Deve ser chamado periodicamente. vehicle_stopped deve ser false
    // quando a medicao de velocidade for invalida ou velha.
    void update(bool vehicle_stopped);

    // Posicao comandada [0, 1] (nao considera o limite do alivio parado).
    float getPosition() const;
    Snapshot getSnapshot() const;

private:
    void updateHold(bool vehicle_stopped, int64_t now_us);
    void writePosition(float position, int64_t now_us);

    uint8_t pin_;
    uint8_t channel_;

    float position_;
    int pulse_us_;
    bool fault_ramp_active_;
    int64_t last_ramp_update_us_;
    bool hold_active_;
    int64_t stopped_since_us_; // 0 = veiculo em movimento
    int64_t timestamp_us_;
    MotorHardware &hardware_;

    // Protege a leitura/escrita coerente da posicao, pulso e timestamp.
    mutable portMUX_TYPE snapshot_mux_ = portMUX_INITIALIZER_UNLOCKED;
};
