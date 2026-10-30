#pragma once

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "actuators/BrakeServo.h"
#include "actuators/DriveMotor.h"
#include "actuators/SteeringMotor.h"

/**
 * Ponto unico de acesso aos tres atuadores (tracao, estercamento e freio).
 *
 * Aplica os comandos somente quando o movimento esta permitido pelo
 * SafetyManager e o comando e valido e recente. Em qualquer outro caso para
 * os motores e leva o freio a posicao total em rampa. Tambem garante o
 * intertravamento: com o freio acima de BRAKE_DRIVE_CUTOFF a tracao e zerada.
 *
 * Usado por TaskControl (applyCommands), TaskSafety (setMotionPermit,
 * updateBrake) e pela telemetria (getSnapshot). Um mutex FreeRTOS serializa
 * todas as operacoes.
 */
class ActuatorManager
{
public:
    // Comando de um ciclo de controle para os tres atuadores.
    struct TimedCommand
    {
        float drive_command;    // tracao normalizada [-1, 1]
        float steering_command; // estercamento na convencao do motor [-1, 1]
        float brake_command;    // freio [0 = solto, 1 = total]
        int64_t produced_at_us; // instante de criacao (esp_timer)
        bool valid;             // saidas dos controladores validas
    };

    // Estado coerente dos atuadores para telemetria.
    struct Snapshot
    {
        DriveMotor::Snapshot drive;
        SteeringMotor::Snapshot steering;
        BrakeServo::Snapshot brake;
        bool motion_permitted;
        bool emergency_latched;
        bool supervisor_tripped;
    };

    ActuatorManager(DriveMotor &drive_motor, SteeringMotor &steering_motor,
                    BrakeServo &brake_servo);

    // Cria o mutex e parte com movimento bloqueado e atuadores parados.
    // Os begin() dos atuadores devem ser chamados antes.
    bool begin();

    // Aplica o comando se ele for valido, finito, dentro da faixa, com idade
    // ate CONTROL_OUTPUT_WATCHDOG_TIMEOUT_MS e o movimento estiver permitido.
    // Caso contrario para tudo. drive_velocity autoriza a troca de sentido da
    // tracao. Retorna true se o comando foi aplicado.
    bool applyCommands(
        const TimedCommand &command,
        const DriveMotor::VelocityMeasurement &drive_velocity);

    // Chamado pelo SafetyManager nas transicoes de estado. Sem permissao os
    // atuadores sao parados imediatamente. Nao libera com emergencia retida.
    void setMotionPermit(bool permitted);

    // Para os motores e aplica o freio em rampa, sem mudar a permissao.
    void stop();

    // Como stop(), mas retem a emergencia: o movimento nao volta a ser
    // permitido ate um novo begin(). Hoje nao ha chamador no firmware.
    void emergencyStop();

    // Chamado pelo supervisor de tasks (outra task, sem o mutex, que pode
    // estar preso pela task travada): escreve PWMs 0 e freio total direto no
    // hardware e retem a falha. Daqui em diante applyCommands() e
    // updateBrake() mantem os atuadores parados com freio total, e o
    // movimento nao volta a ser permitido ate reiniciar.
    void forceSafeOutputs();
    bool isSupervisorTripped() const;

    // Modo manobra (decidido pelo SafetyManager): freio solto e tracao parada,
    // com ou sem permissao de movimento. O estercamento so atua com permissao.
    // Ignorado com o supervisor disparado ou emergencia retida.
    void setManeuverBrakeRelease(bool release);
    bool isManeuverBrakeRelease() const;

    // Avanca a rampa de falha do freio e o alivio de veiculo parado.
    // Chamado periodicamente pela task de seguranca para nao depender da
    // task de controle.
    void updateBrake(bool vehicle_stopped);

    bool isMotionPermitted() const;
    bool isEmergencyLatched() const;
    Snapshot getSnapshot() const;

private:
    // Espera indefinidamente pelo mutex; false so se ele nao foi criado.
    bool takeMutex() const;
    void giveMutex() const;
    // Para tracao e estercamento e inicia a rampa do freio (mutex ja tomado).
    void stopUnlocked();
    // Motores parados e freio total enquanto o supervisor estiver disparado.
    void holdSupervisorStop();

    DriveMotor &drive_motor_;
    SteeringMotor &steering_motor_;
    BrakeServo &brake_servo_;

    mutable SemaphoreHandle_t mutex_;
    bool motion_permitted_;
    bool emergency_latched_;
    bool maneuver_brake_release_;
    // Escrito pelo supervisor sem o mutex: so passa de false para true.
    volatile bool supervisor_tripped_;
};
