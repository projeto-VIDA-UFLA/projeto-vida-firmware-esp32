#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "utils/pinout.h"
#include "utils/config.h"

#include "sensors/EncoderReader.h"
#include "sensors/SteeringSensor.h"

#include <std_msgs/msg/float64_multi_array.h>
#include "comunication/RosCommunication.h"

// ============================================================
// Comunicação micro-ROS
// ============================================================

// Nome do nó: /vida/esp32_controller
// ROS Domain ID: 0
RosCommunication ros_communication(
    "esp32_controller",
    "/vida",
    0);

// Publisher que enviará todos os valores dos sensores.
rcl_publisher_t sensors_publisher = rcl_get_zero_initialized_publisher();

// Timer responsável pela publicação periódica.
rcl_timer_t sensors_publish_timer = rcl_get_zero_initialized_timer();

// Mensagem publicada.
std_msgs__msg__Float64MultiArray sensors_message;

// Buffer estático da mensagem.
//
// Índices:
//
// 0 = tempo do ESP32 em milissegundos
// 1 = leitura ADC bruta do LVDT
// 2 = contagem do encoder traseiro
// 3 = contagem do encoder dianteiro direito
// 4 = contagem do encoder dianteiro esquerdo
// 5 = velocidade do encoder traseiro
// 6 = velocidade do encoder dianteiro direito
// 7 = velocidade do encoder dianteiro esquerdo
// 8 = aceleração do encoder traseiro
// 9 = aceleração do encoder dianteiro direito
// 10 = aceleração do encoder dianteiro esquerdo

static constexpr size_t SENSOR_MESSAGE_SIZE = 11;

double sensors_message_buffer[SENSOR_MESSAGE_SIZE] = {0.0};

// ============================================================
// Snapshot compartilhado entre as Tasks
// ============================================================

struct SensorSnapshot
{
  uint32_t timestamp_ms;

  int32_t steering_raw_adc;

  int64_t rear_count;
  int64_t right_count;
  int64_t left_count;

  float rear_velocity;
  float right_velocity;
  float left_velocity;

  float rear_acceleration;
  float right_acceleration;
  float left_acceleration;
};

SensorSnapshot sensor_snapshot = {};

// Protege o snapshot contra acesso simultâneo entre:
//
// - TaskSensors;
// - Task interna do micro-ROS.
portMUX_TYPE sensor_snapshot_mux = portMUX_INITIALIZER_UNLOCKED;

// ============================================================
// ISRs dos encoders
// ============================================================

EncoderReader rear_encoder(ENC_REAR_A, ENC_REAR_B, ENC_REAR_TICKS_PER_REV, "rear");
EncoderReader right_encoder(ENC_RIGHT_A, ENC_RIGHT_B, ENC_FRONT_TICKS_PER_REV, "front_right");
EncoderReader left_encoder(ENC_LEFT_A, ENC_LEFT_B, ENC_FRONT_TICKS_PER_REV, "front_left");

SteeringSensor steering_sensor(PIN_LVDT);

// ============================================================
// ISRs dos encoders
// ============================================================

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

// ============================================================
// Callback do timer micro-ROS
// ============================================================

void publishSensorsCallback(
    rcl_timer_t *timer,
    int64_t last_call_time)
{
  // Evita warnings de parâmetros não utilizados.
  (void)last_call_time;

  if (timer == nullptr)
  {
    return;
  }

  SensorSnapshot local_snapshot;

  /*
   * Copia rapidamente o snapshot protegido para uma variável local.
   *
   * A publicação ROS não deve acontecer dentro da região crítica,
   * pois ela pode demorar e bloquear a Task de sensores.
   */
  portENTER_CRITICAL(&sensor_snapshot_mux);
  local_snapshot = sensor_snapshot;
  portEXIT_CRITICAL(&sensor_snapshot_mux);

  sensors_message_buffer[0] = static_cast<double>(local_snapshot.timestamp_ms);

  sensors_message_buffer[1] = static_cast<double>(local_snapshot.steering_raw_adc);

  sensors_message_buffer[2] = static_cast<double>(local_snapshot.rear_count);

  sensors_message_buffer[3] = static_cast<double>(local_snapshot.right_count);

  sensors_message_buffer[4] = static_cast<double>(local_snapshot.left_count);

  sensors_message_buffer[5] = static_cast<double>(local_snapshot.rear_velocity);

  sensors_message_buffer[6] = static_cast<double>(local_snapshot.right_velocity);

  sensors_message_buffer[7] = static_cast<double>(local_snapshot.left_velocity);

  sensors_message_buffer[8] = static_cast<double>(local_snapshot.rear_acceleration);

  sensors_message_buffer[9] = static_cast<double>(local_snapshot.right_acceleration);

  sensors_message_buffer[10] = static_cast<double>(local_snapshot.left_acceleration);

  ros_communication.publish(sensors_publisher, &sensors_message);
}

// ============================================================
// TASKS
// ============================================================

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

    portENTER_CRITICAL(&sensor_snapshot_mux);
    sensor_snapshot.timestamp_ms = millis();
    sensor_snapshot.steering_raw_adc = steering_sensor.getRawAdc();
    sensor_snapshot.rear_count = rear_encoder.getCount();
    sensor_snapshot.right_count = right_encoder.getCount();
    sensor_snapshot.left_count = left_encoder.getCount();
    sensor_snapshot.rear_velocity = rear_encoder.getVelocity();
    sensor_snapshot.right_velocity = right_encoder.getVelocity();
    sensor_snapshot.left_velocity = left_encoder.getVelocity();
    sensor_snapshot.rear_acceleration = rear_encoder.getAcceleration();
    sensor_snapshot.right_acceleration = right_encoder.getAcceleration();
    sensor_snapshot.left_acceleration = left_encoder.getAcceleration();
    portEXIT_CRITICAL(&sensor_snapshot_mux);

    vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(TASK_SENSOR_PERIOD_MS));
  }
}

// ============================================================
// Configuração da mensagem ROS
// ============================================================

void configureSensorsMessage()
{
  /*
   * O Float64MultiArray normalmente utiliza memória dinâmica.
   *
   * Aqui apontamos a sequência diretamente para um buffer estático,
   * evitando malloc durante a execução do firmware.
   */
  sensors_message.data.data = sensors_message_buffer;
  sensors_message.data.size = SENSOR_MESSAGE_SIZE;
  sensors_message.data.capacity = SENSOR_MESSAGE_SIZE;

  /*
   * Não usaremos dimensões descritivas no MultiArray.
   *
   * O significado de cada posição é definido pela ordem documentada
   * acima.
   */
  sensors_message.layout.dim.data = nullptr;
  sensors_message.layout.dim.size = 0;
  sensors_message.layout.dim.capacity = 0;

  sensors_message.layout.data_offset = 0;
}

// ============================================================
// Configuração do micro-ROS
// ============================================================

bool configureMicroRos()
{
  configureSensorsMessage();

  // Registra o publisher.
  const bool publisher_added = ros_communication.addPublisher(sensors_publisher, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray), "/vida/sensors");

  if (!publisher_added)
  {
    return false;
  }

  /*
   * Registra o timer que substitui a antiga
   * taskCommunication().
   *
   * O período de publicação continua sendo definido por:
   *
   * TASK_COMUNICATION_PERIOD_MS
   */
  const bool timer_added = ros_communication.addTimer(sensors_publish_timer, TASK_COMUNICATION_PERIOD_MS, publishSensorsCallback);

  if (!timer_added)
  {
    return false;
  }

  return true;
}

void setup()
{
  Serial.begin(SERIAL_BAUDRATE);
  delay(1000);

  rear_encoder.begin();
  right_encoder.begin();
  left_encoder.begin();
  steering_sensor.begin();

  attachInterrupt(digitalPinToInterrupt(ENC_REAR_A), isrRearEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_REAR_B), isrRearEncoder, CHANGE);

  attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_A), isrRightEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_B), isrRightEncoder, CHANGE);

  attachInterrupt(digitalPinToInterrupt(ENC_LEFT_A), isrLeftEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_LEFT_B), isrLeftEncoder, CHANGE);

  xTaskCreatePinnedToCore(taskSensors, "TaskSensors", 4096, nullptr, PRIORITY_SENSORS, nullptr, 1);

  // --------------------------------------------------------
  // Registra publishers e timers
  // --------------------------------------------------------

  if (!configureMicroRos())
  {
    /*
     * Não usamos Serial.print porque a Serial será utilizada
     * pelo protocolo micro-ROS.
     */
    while (true)
    {
      delay(1000);
    }
  }

  // --------------------------------------------------------
  // Configura o transporte micro-ROS
  // --------------------------------------------------------

  ros_communication.begin(Serial);
}

ros_communication.startTask("TaskRosCommunication", 8192, 2, 0);

void loop()
{
}
