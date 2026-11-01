//======================================================================
// FIRMWARE ESP32 - PROJETO VIDA (LMT/UFLA)
//
// Controle de baixo nivel do veiculo e ponte com o ROS 2 via micro-ROS.
// As tasks apenas chamam os objetos e repassam snapshots por parametro; a
// logica fica nas classes (ver doc/README.md).
//
//   Task                  Nucleo  Prioridade  Periodo
//   TaskSafety            1       4           TASK_SAFETY_PERIOD_MS  (5 ms)
//   TaskControl           1       3           TASK_CONTROL_PERIOD_MS (5 ms)
//   TaskSensors           1       2           TASK_SENSOR_PERIOD_MS  (2 ms)
//   TaskRosCommunication  0       2           laco de 1 ms, publica a cada 20 ms
//   TaskBattery           0       1           BATTERY_MEASUREMENT_PERIOD_MS (100 ms)
//======================================================================

//======================================================================
// BIBLIOTECAS
//======================================================================

// Inclui bibliotecas ESP32
#include <Arduino.h>
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Inclui classes utilitárias
#include "utils/pinout.h"
#include "utils/config.h"

// Inclui classes de sensores
#include "sensors/EncoderReader.h"
#include "sensors/SteeringSensor.h"
#include "sensors/VehicleMotion.h"
#include "sensors/BatteryMonitor.h"

// Inclui classes de atuadores
#include "actuators/SteeringMotor.h"
#include "actuators/DriveMotor.h"
#include "actuators/BrakeServo.h"
#include "actuators/ActuatorManager.h"

// Inclui classes de segurança
#include "safety/SafetyManager.h"
#include "safety/TaskSupervisor.h"

// Inclui classes de controle
#include "control/DriveController.h"
#include "control/SteeringController.h"

// Inclui classes de comunicação
#include "comunication/CommandMailbox.h"
#include "comunication/VidaRosNode.h"

//======================================================================
// INSTANCIA DE OBJETOS
//======================================================================

// Instanciando objetos de encoders
EncoderReader right_encoder(ENC_RIGHT_A, ENC_RIGHT_B, ENC_FRONT_TICKS_PER_REV, "front_right");
EncoderReader left_encoder(ENC_LEFT_A, ENC_LEFT_B, ENC_FRONT_TICKS_PER_REV, "front_left");
EncoderReader rear_encoder(ENC_REAR_A, ENC_REAR_B, ENC_REAR_TICKS_PER_REV, "rear");

// Estado de movimento do veiculo a partir dos tres encoders
VehicleMotion vehicle_motion(rear_encoder, right_encoder, left_encoder);

// Instanciando objeto do sensor de esterçamento
SteeringSensor steering_sensor(PIN_LVDT);

// Monitor do banco de baterias (INA228 no I2C)
BatteryMonitor battery_monitor(INA228_I2C_ADDRESS, PIN_I2C_SDA, PIN_I2C_SCL);

// Instanciando objetos que representam os atuadores
SteeringMotor steering_motor(PIN_STEER_RIGHT_PWM, PIN_STEER_LEFT_PWM, PWM_CH_STEER_RIGHT, PWM_CH_STEER_LEFT);
DriveMotor drive_motor(PIN_DRIVE_PWM, PIN_DRIVE_REVERSE, PWM_CH_DRIVE);
BrakeServo brake_servo(PIN_BRAKE_SERVO, PWM_CH_BRAKE);
ActuatorManager actuator_manager(drive_motor, steering_motor, brake_servo);

// Instanciando Controladores
SteeringController steering_controller;
DriveController drive_controller;

// Referencias e enable logico recebidos do ROS
CommandMailbox command_mailbox;

// Instanciando Objeto de gerenciamento de segurança
SafetyManager safety_manager(PIN_ENABLE_SAFETY, command_mailbox, actuator_manager, steering_controller);

// Supervisor das tasks do nucleo 1 (corta os atuadores se uma travar)
TaskSupervisor task_supervisor(actuator_manager);

// Instanciando nó do ROS
VidaRosNode ros_node(rear_encoder, right_encoder, left_encoder, steering_sensor, battery_monitor, safety_manager, actuator_manager, command_mailbox);

//======================================================================
// DEFINIÇÃO DE ISRs DOS ENCODERS
//======================================================================

void IRAM_ATTR isrRearEncoder()
{
  rear_encoder.handleInterrupt();
}

void IRAM_ATTR isrRightEncoder()
{
  right_encoder.handleInterrupt();
}

void IRAM_ATTR isrLeftEncoder()
{
  left_encoder.handleInterrupt();
}

//======================================================================
// TASKs
//======================================================================

// Le LVDT e encoders e informa o heartbeat ao SafetyManager.
void taskSensors(void *parameter)
{
  (void)parameter;
  TickType_t last_wake_time = xTaskGetTickCount();

  while (true)
  {
    steering_sensor.update();
    rear_encoder.update();
    right_encoder.update();
    left_encoder.update();

    task_supervisor.reportAlive(TaskSupervisor::Task::SENSORS);

    // Heartbeat registrado somente depois do ciclo completo.
    safety_manager.reportSensorTaskCycle(steering_sensor.getSnapshot(),
                                         rear_encoder.getMeasurementSnapshot(),
                                         right_encoder.getMeasurementSnapshot(),
                                         left_encoder.getMeasurementSnapshot());

    vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(TASK_SENSOR_PERIOD_MS));
  }
}

// Avalia a permissao de movimento e atualiza o freio (rampa / alivio parado).
void taskSafety(void *parameter)
{
  (void)parameter;
  TickType_t last_wake_time = xTaskGetTickCount();

  while (true)
  {
    task_supervisor.reportAlive(TaskSupervisor::Task::SAFETY);

    const bool vehicle_stopped = vehicle_motion.isStopped();
    safety_manager.update(vehicle_stopped);
    actuator_manager.updateBrake(vehicle_stopped);

    vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(TASK_SAFETY_PERIOD_MS));
  }
}

// Calcula esterçamento e tracao e aplica nos atuadores.
void taskControl(void *parameter)
{
  (void)parameter;
  TickType_t last_wake_time = xTaskGetTickCount();

  while (true)
  {
    const SteeringController::Output steering =
        steering_controller.update(command_mailbox.getSteeringReferenceSnapshot(),
                                   steering_sensor.getSnapshot(),
                                   actuator_manager.isMotionPermitted());

    task_supervisor.reportAlive(TaskSupervisor::Task::CONTROL);

    const bool bench_mode = safety_manager.isBenchModeActive();
    const DriveController::Output drive =
        drive_controller.update(command_mailbox.getDriveReferenceSnapshot(),
                                vehicle_motion.getVehicleSpeed(bench_mode),
                                actuator_manager.isMotionPermitted(),
                                bench_mode);

    const ActuatorManager::TimedCommand command{
        drive.normalized_command,
        steering.motor_command,
        drive.brake_command,
        esp_timer_get_time(),
        steering.valid && drive.valid};

    (void)actuator_manager.applyCommands(command, vehicle_motion.getStopDetectionVelocity());

    vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(TASK_CONTROL_PERIOD_MS));
  }
}

// Le tensao e corrente do banco. No nucleo 0 e abaixo do ROS: o I2C bloqueia
// a task por ate BATTERY_I2C_TIMEOUT_MS e nao entra no TaskSupervisor.
void taskBattery(void *parameter)
{
  (void)parameter;
  TickType_t last_wake_time = xTaskGetTickCount();

  while (true)
  {
    battery_monitor.update();
    vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(BATTERY_MEASUREMENT_PERIOD_MS));
  }
}

//======================================================================
// VOID SETUP
//======================================================================

void setup()
{
  // Inicializa servo motor do freio
  brake_servo.begin();

  // Inicia comunicação serial (o buffer de recepção precisa vir antes do begin)
  Serial.setRxBufferSize(SERIAL_RX_BUFFER_SIZE);
  Serial.begin(SERIAL_BAUDRATE);
  delay(1000);

  // Inicializa sensores e as interrupções dos encoders
  rear_encoder.begin(isrRearEncoder);
  right_encoder.begin(isrRightEncoder);
  left_encoder.begin(isrLeftEncoder);
  steering_sensor.begin();
  // Sem mensagens na Serial: ela e o transporte do micro-ROS. A falha aparece
  // em /vida/sensors/battery (valid = false).
  battery_monitor.begin();

  // Inicializa demais atuadores
  drive_motor.begin();
  steering_motor.begin();
  actuator_manager.begin();

  // Inicializa gerenciador de segurança
  safety_manager.begin();

  // Inicializa TASKs
  xTaskCreatePinnedToCore(taskSensors, "TaskSensors", 4096, nullptr, PRIORITY_SENSORS, nullptr, 1);
  xTaskCreatePinnedToCore(taskControl, "TaskControl", 4096, nullptr, PRIORITY_CONTROL, nullptr, 1);
  xTaskCreatePinnedToCore(taskSafety, "TaskSafety", 4096, nullptr, PRIORITY_SAFETY, nullptr, 1);
  xTaskCreatePinnedToCore(taskBattery, "TaskBattery", 4096, nullptr, PRIORITY_BATTERY, nullptr, 0);

  // Supervisor das tasks (timer no nucleo 0) e watchdog do ESP-IDF
  task_supervisor.begin();

  ros_node.begin(Serial);
}

void loop()
{
}
