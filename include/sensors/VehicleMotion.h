#pragma once

#include <Arduino.h>

#include "actuators/DriveMotor.h"
#include "sensors/EncoderReader.h"

/**
 * Estado de movimento do veiculo a partir dos tres encoders.
 *
 * O freio atua so na roda traseira: as rodas dianteiras confirmam que o
 * veiculo parou mesmo com a roda traseira travada. Leitura invalida ou velha
 * de qualquer encoder conta como movimento.
 */
class VehicleMotion
{
public:
    VehicleMotion(EncoderReader &rear_encoder,
                  EncoderReader &right_encoder,
                  EncoderReader &left_encoder);

    // Veiculo parado: os tres encoders com leitura recente e abaixo do limiar.
    bool isStopped();

    // Velocidade longitudinal do veiculo para o controle de tracao: media das
    // rodas dianteiras (nao sao tracionadas nem freadas, entao nao patinam nem
    // travam). Sem compensacao de curva: em curva fica ~1/cos(delta) acima da
    // velocidade do eixo traseiro. timestamp_us = amostra mais antiga.
    // Em modo bancada (veiculo suspenso, dianteiras paradas) usa o encoder
    // traseiro.
    DriveMotor::VelocityMeasurement getVehicleSpeed(bool bench_mode = false);

    // Medicao usada pelo DriveMotor para autorizar a troca do pino DIR.
    // Combina os tres encoders pelo pior caso: maior velocidade em modulo e
    // amostra mais antiga.
    DriveMotor::VelocityMeasurement getStopDetectionVelocity();

private:
    static bool encoderReportsStopped(const EncoderReader::Snapshot &snapshot,
                                      int64_t now_us);

    EncoderReader &rear_encoder_;
    EncoderReader &right_encoder_;
    EncoderReader &left_encoder_;
};
