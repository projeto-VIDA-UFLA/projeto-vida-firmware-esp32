#pragma once

#include <Arduino.h>

/**
 * Caixa de correio entre a comunicacao ROS e as tasks de controle/seguranca.
 *
 * Os callbacks do VidaRosNode (TaskRosCommunication, nucleo 0) escrevem a
 * ultima mensagem recebida de cada tipo junto com o instante local de
 * chegada; TaskControl e TaskSafety (nucleo 1) leem copias coerentes. A idade
 * (agora - received_at_us) e o que os watchdogs avaliam: o timestamp da
 * mensagem (source_timestamp_ns) e guardado apenas para diagnostico.
 */
class CommandMailbox
{
public:
    // Ultima /vida/control/steering/reference recebida.
    struct SteeringReferenceSnapshot
    {
        float angle_deg;              // REP-103: esquerda positivo
        float angular_velocity_deg_s; // recebido, ainda nao usado
        int64_t source_timestamp_ns;  // timestamp da mensagem (relogio ROS)
        int64_t received_at_us;       // chegada no ESP32 (esp_timer)
        bool received;                // false ate a primeira mensagem
    };

    // Ultima /vida/control/drive/reference recebida.
    struct DriveReferenceSnapshot
    {
        float velocity_m_s;      // malha aberta: PWM assinado [-255, 255]
        float acceleration_m_s2; // recebido, ainda nao usado
        float brake_command;     // 0 = solto, 1 = freio total
        int64_t source_timestamp_ns;
        int64_t received_at_us;
        bool received;
    };

    // Ultimo /vida/safety/logical_enable recebido.
    struct LogicalEnableSnapshot
    {
        bool enabled;
        int64_t source_timestamp_ns;
        int64_t received_at_us;
        bool received;
    };

    // Ultimo /vida/safety/bench_mode_request recebido.
    struct BenchModeRequestSnapshot
    {
        bool requested;
        int64_t received_at_us;
        bool received;
    };

    CommandMailbox();

    // Escrita pelos callbacks ROS: registra o valor e o instante de chegada.
    void updateSteeringReference(float angle_deg,
                                 float angular_velocity_deg_s,
                                 int64_t source_timestamp_ns);
    void updateDriveReference(float velocity_m_s,
                              float acceleration_m_s2,
                              float brake_command,
                              int64_t source_timestamp_ns);
    void updateLogicalEnable(bool enabled, int64_t source_timestamp_ns);
    void updateBenchModeRequest(bool requested);

    // Leitura pelas tasks: copia coerente protegida por portMUX.
    SteeringReferenceSnapshot getSteeringReferenceSnapshot() const;
    DriveReferenceSnapshot getDriveReferenceSnapshot() const;
    LogicalEnableSnapshot getLogicalEnableSnapshot() const;
    BenchModeRequestSnapshot getBenchModeRequestSnapshot() const;

private:
    SteeringReferenceSnapshot steering_reference_;
    DriveReferenceSnapshot drive_reference_;
    LogicalEnableSnapshot logical_enable_;
    BenchModeRequestSnapshot bench_mode_request_;

    mutable portMUX_TYPE steering_mux_;
    mutable portMUX_TYPE drive_mux_;
    mutable portMUX_TYPE logical_enable_mux_;
    mutable portMUX_TYPE bench_mode_mux_;
};
