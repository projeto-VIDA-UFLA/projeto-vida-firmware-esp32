#pragma once
#include <Arduino.h>
#include "actuators/MotorHardware.h"

// Servo de hobby (50 Hz) que puxa o cabo de aco do freio.
// Posicao normalizada: 0 = solto, 1 = freio total.
// Sem pulsos o servo fica solto e o freio e liberado: esta classe nao cobre
// reset do ESP32 nem queda da alimentacao do servo.
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

    // Avanca a rampa de falha e atualiza o alivio de veiculo parado.
    // Deve ser chamado periodicamente. vehicle_stopped deve ser false
    // quando a medicao de velocidade for invalida ou velha.
    void update(bool vehicle_stopped);

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
