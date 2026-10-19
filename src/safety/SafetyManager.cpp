#include "safety/SafetyManager.h"
#include "safety/SafetyWatchdogs.h"

#include "utils/config.h"
#include "esp_timer.h"
#include <cmath>

SafetyManager::SafetyManager(uint8_t physical_enable_pin,
                             CommandMailbox &command_mailbox,
                             ActuatorManager &actuator_manager,
                             SteeringController &steering_controller)
    : physical_enable_pin_(physical_enable_pin),
      command_mailbox_(command_mailbox),
      actuator_manager_(actuator_manager),
      steering_controller_(steering_controller),
      sensor_health_{0, 0, false, 0, false, 0.0f, 0.0f},
      encoder_plausibility_(),
      encoders_disagree_latched_(false),
      bench_mode_active_(false),
      snapshot_()
{
    snapshot_.state = SafetyState::STATE_DISABLED;
    snapshot_.physical_enable = false;
    snapshot_.logical_enable = false;
    snapshot_.logical_enable_received = false;
    snapshot_.logical_enable_fresh = false;
    snapshot_.drive_reference_fresh = false;
    snapshot_.steering_reference_fresh = false;
    snapshot_.sensor_task_healthy = false;
    snapshot_.steering_sensor_fresh = false;
    snapshot_.control_output_fresh = false;
    snapshot_.encoders_disagree = false;
    snapshot_.bench_mode_active = false;
    snapshot_.supervisor_tripped = false;
    snapshot_.motion_permitted = false;
    snapshot_.inhibit_flags =
        INHIBIT_PHYSICAL_ENABLE_OFF |
        INHIBIT_LOGICAL_ENABLE_NOT_RECEIVED |
        INHIBIT_DRIVE_REFERENCE_NOT_FRESH |
        INHIBIT_STEERING_REFERENCE_NOT_FRESH;
    snapshot_.timestamp_us = 0;
}

void SafetyManager::begin()
{
    pinMode(physical_enable_pin_, INPUT);

    // A inicializacao e sempre bloqueada, independentemente do nivel fisico.
    actuator_manager_.setMotionPermit(false);

    const int64_t now_us = esp_timer_get_time();
    const bool physical_enable = digitalRead(physical_enable_pin_) == HIGH;

    portENTER_CRITICAL(&health_mux_);
    sensor_health_ = SensorHealthSnapshot{now_us, 0, false, 0, false, 0.0f, 0.0f};
    portEXIT_CRITICAL(&health_mux_);

    encoder_plausibility_.reset();
    encoders_disagree_latched_ = false;
    bench_mode_active_ = false;

    Snapshot initial_snapshot{};
    initial_snapshot.state = SafetyState::STATE_DISABLED;
    initial_snapshot.physical_enable = physical_enable;
    initial_snapshot.inhibit_flags =
        (physical_enable ? INHIBIT_NONE : INHIBIT_PHYSICAL_ENABLE_OFF) |
        INHIBIT_LOGICAL_ENABLE_NOT_RECEIVED |
        INHIBIT_DRIVE_REFERENCE_NOT_FRESH |
        INHIBIT_STEERING_REFERENCE_NOT_FRESH;
    initial_snapshot.timestamp_us = now_us;

    portENTER_CRITICAL(&snapshot_mux_);
    snapshot_ = initial_snapshot;
    portEXIT_CRITICAL(&snapshot_mux_);
}

void SafetyManager::reportSensorTaskCycle(
    const SteeringSensor::Snapshot &steering_snapshot,
    const EncoderReader::Snapshot &rear_snapshot,
    const EncoderReader::Snapshot &right_snapshot,
    const EncoderReader::Snapshot &left_snapshot)
{
    // O heartbeat indica a task viva com encoders validos. A validade do LVDT
    // e acompanhada a parte (INHIBIT_STEERING_SENSOR_NOT_FRESH), para que um
    // sensor invalido nao apareca como task de sensores parada.
    const bool encoders_valid =
        encoderSnapshotValid(rear_snapshot) &&
        encoderSnapshotValid(right_snapshot) &&
        encoderSnapshotValid(left_snapshot);

    if (!encoders_valid)
    {
        return;
    }

    const int64_t now_us = esp_timer_get_time();
    portENTER_CRITICAL(&health_mux_);
    // Heartbeat e metadados da amostra pertencem ao mesmo ciclo publicado.
    sensor_health_.heartbeat_us = now_us;
    sensor_health_.steering_snapshot_valid = steering_snapshot.valid;
    sensor_health_.steering_snapshot_us = steering_snapshot.timestamp_us;
    sensor_health_.encoder_speeds_valid = true;
    sensor_health_.rear_velocity_m_s = rear_snapshot.filtered_velocity_m_s;
    sensor_health_.front_velocity_m_s =
        0.5f * (right_snapshot.filtered_velocity_m_s + left_snapshot.filtered_velocity_m_s);
    portEXIT_CRITICAL(&health_mux_);
}

void SafetyManager::update(bool vehicle_stopped)
{
    constexpr int64_t logical_enable_timeout_us =
        static_cast<int64_t>(LOGICAL_ENABLE_WATCHDOG_TIMEOUT_MS) * 1000LL;
    constexpr int64_t drive_reference_timeout_us =
        static_cast<int64_t>(DRIVE_REFERENCE_WATCHDOG_TIMEOUT_MS) * 1000LL;
    constexpr int64_t steering_reference_timeout_us =
        static_cast<int64_t>(STEERING_REFERENCE_WATCHDOG_TIMEOUT_MS) * 1000LL;

    const int64_t now_us = esp_timer_get_time();
    const bool physical_enable = digitalRead(physical_enable_pin_) == HIGH;
    const CommandMailbox::LogicalEnableSnapshot logical_enable =
        command_mailbox_.getLogicalEnableSnapshot();
    const CommandMailbox::DriveReferenceSnapshot drive_reference =
        command_mailbox_.getDriveReferenceSnapshot();
    const CommandMailbox::SteeringReferenceSnapshot steering_reference =
        command_mailbox_.getSteeringReferenceSnapshot();
    const SteeringController::Snapshot control_output =
        steering_controller_.getLastOutput();

    SensorHealthSnapshot sensor_health{};
    portENTER_CRITICAL(&health_mux_);
    sensor_health = sensor_health_;
    portEXIT_CRITICAL(&health_mux_);

    const bool logical_enable_fresh = isFresh(logical_enable.received,
                                              logical_enable.received_at_us,
                                              now_us,
                                              logical_enable_timeout_us);

    const bool drive_reference_fresh = isFresh(drive_reference.received,
                                               drive_reference.received_at_us,
                                               now_us,
                                               drive_reference_timeout_us);

    const bool steering_reference_fresh = isFresh(steering_reference.received,
                                                  steering_reference.received_at_us,
                                                  now_us,
                                                  steering_reference_timeout_us);
    const bool control_required =
        physical_enable &&
        logical_enable.received &&
        logical_enable_fresh &&
        logical_enable.enabled &&
        drive_reference_fresh &&
        steering_reference_fresh;
    const SafetyWatchdogs::Result watchdogs = SafetyWatchdogs::evaluate(
        SafetyWatchdogs::Input{
            now_us,
            sensor_health.initialized_at_us,
            sensor_health.heartbeat_us,
            sensor_health.steering_snapshot_valid,
            sensor_health.steering_snapshot_us,
            control_required,
            control_output.output.valid,
            control_output.output.normalized_command,
            control_output.timestamp_us});
    const bool sensor_task_timed_out = watchdogs.sensor_task_timed_out;
    const bool sensor_task_healthy = watchdogs.sensor_task_healthy;
    const bool steering_sensor_fresh = watchdogs.steering_snapshot_fresh;
    const bool control_output_fresh = watchdogs.control_output_fresh;

    uint32_t inhibit_flags = INHIBIT_NONE;
    if (!physical_enable)
    {
        inhibit_flags |= INHIBIT_PHYSICAL_ENABLE_OFF;
    }

    if (!logical_enable.received)
    {
        inhibit_flags |= INHIBIT_LOGICAL_ENABLE_NOT_RECEIVED;
    }
    else
    {
        if (!logical_enable.enabled)
        {
            inhibit_flags |= INHIBIT_LOGICAL_ENABLE_FALSE;
        }

        if (!logical_enable_fresh)
        {
            inhibit_flags |= INHIBIT_LOGICAL_ENABLE_TIMEOUT;
        }
    }

    if (!drive_reference_fresh)
    {
        inhibit_flags |= INHIBIT_DRIVE_REFERENCE_NOT_FRESH;
    }

    if (!steering_reference_fresh)
    {
        inhibit_flags |= INHIBIT_STEERING_REFERENCE_NOT_FRESH;
    }
    if (sensor_task_timed_out)
    {
        inhibit_flags |= INHIBIT_SENSOR_TASK_TIMEOUT;
    }
    if (!steering_sensor_fresh)
    {
        inhibit_flags |= INHIBIT_STEERING_SENSOR_NOT_FRESH;
    }
    if (watchdogs.control_output_timed_out)
    {
        inhibit_flags |= INHIBIT_CONTROL_OUTPUT_TIMEOUT;
    }

    // Modo bancada: entra so parado; sai com pedido false ou sem pedido recente.
    constexpr int64_t bench_request_timeout_us =
        static_cast<int64_t>(BENCH_MODE_REQUEST_TIMEOUT_MS) * 1000LL;
    const CommandMailbox::BenchModeRequestSnapshot bench_request =
        command_mailbox_.getBenchModeRequestSnapshot();
    const bool bench_requested =
        bench_request.requested &&
        isFresh(bench_request.received, bench_request.received_at_us,
                now_us, bench_request_timeout_us);
    if (!bench_requested)
    {
        bench_mode_active_ = false;
    }
    else if (!bench_mode_active_ && vehicle_stopped)
    {
        bench_mode_active_ = true;
    }

    // Encoders discordantes: falha retida ate acao explicita do operador
    // (enable fisico desligado ou enable logico false). Desligada na bancada,
    // onde so a roda traseira gira.
    const bool rearm_requested =
        !physical_enable ||
        (logical_enable.received && logical_enable_fresh && !logical_enable.enabled);
    if (rearm_requested)
    {
        encoders_disagree_latched_ = false;
        encoder_plausibility_.reset();
    }
    else if (bench_mode_active_)
    {
        encoder_plausibility_.reset();
    }
    else
    {
        const ActuatorManager::Snapshot actuators = actuator_manager_.getSnapshot();
        const bool disagree = encoder_plausibility_.update(
            EncoderPlausibility::Input{
                now_us,
                actuators.drive.pwm > 0,
                sensor_task_healthy && sensor_health.encoder_speeds_valid,
                sensor_health.rear_velocity_m_s,
                sensor_health.front_velocity_m_s});
        encoders_disagree_latched_ = encoders_disagree_latched_ || disagree;
    }
    if (encoders_disagree_latched_)
    {
        inhibit_flags |= INHIBIT_ENCODERS_DISAGREE;
    }

    // Task travada: os atuadores ja foram forcados a parar pelo supervisor.
    const bool supervisor_tripped = actuator_manager_.isSupervisorTripped();
    if (supervisor_tripped)
    {
        inhibit_flags |= INHIBIT_TASK_SUPERVISOR_TRIP;
    }

    const bool motion_permitted =
        physical_enable &&
        logical_enable.received &&
        logical_enable_fresh &&
        logical_enable.enabled &&
        drive_reference_fresh &&
        steering_reference_fresh &&
        sensor_task_healthy &&
        steering_sensor_fresh &&
        control_output_fresh &&
        !encoders_disagree_latched_ &&
        !supervisor_tripped;
    const SafetyState new_state = motion_permitted
                                      ? SafetyState::STATE_READY
                                      : SafetyState::STATE_DISABLED;

    SafetyState previous_state;
    portENTER_CRITICAL(&snapshot_mux_);
    previous_state = snapshot_.state;
    portEXIT_CRITICAL(&snapshot_mux_);

    if (new_state != previous_state)
    {
        applyStateTransition(previous_state, new_state);
    }

    Snapshot new_snapshot{};
    new_snapshot.state = new_state;
    new_snapshot.physical_enable = physical_enable;
    new_snapshot.logical_enable = logical_enable.enabled;
    new_snapshot.logical_enable_received = logical_enable.received;
    new_snapshot.logical_enable_fresh = logical_enable_fresh;
    new_snapshot.drive_reference_fresh = drive_reference_fresh;
    new_snapshot.steering_reference_fresh = steering_reference_fresh;
    new_snapshot.sensor_task_healthy = sensor_task_healthy;
    new_snapshot.steering_sensor_fresh = steering_sensor_fresh;
    new_snapshot.control_output_fresh = control_output_fresh;
    new_snapshot.encoders_disagree = encoders_disagree_latched_;
    new_snapshot.bench_mode_active = bench_mode_active_;
    new_snapshot.supervisor_tripped = supervisor_tripped;
    new_snapshot.motion_permitted = motion_permitted;
    new_snapshot.inhibit_flags = inhibit_flags;
    new_snapshot.timestamp_us = now_us;

    portENTER_CRITICAL(&snapshot_mux_);
    snapshot_ = new_snapshot;
    portEXIT_CRITICAL(&snapshot_mux_);
}

bool SafetyManager::isBenchModeActive() const
{
    portENTER_CRITICAL(&snapshot_mux_);
    const bool active = snapshot_.bench_mode_active;
    portEXIT_CRITICAL(&snapshot_mux_);
    return active;
}

SafetyManager::Snapshot SafetyManager::getSnapshot() const
{
    portENTER_CRITICAL(&snapshot_mux_);
    const Snapshot snapshot = snapshot_;
    portEXIT_CRITICAL(&snapshot_mux_);
    return snapshot;
}

bool SafetyManager::encoderSnapshotValid(const EncoderReader::Snapshot &snapshot)
{
    return std::isfinite(snapshot.filtered_velocity_m_s) &&
           snapshot.timestamp_us > 0;
}

bool SafetyManager::isFresh(bool received,
                            int64_t received_at_us,
                            int64_t now_us,
                            int64_t timeout_us)
{
    return received &&
           received_at_us > 0 &&
           now_us >= received_at_us &&
           (now_us - received_at_us) <= timeout_us;
}

void SafetyManager::applyStateTransition(SafetyState previous_state,
                                         SafetyState new_state)
{
    (void)previous_state;
    actuator_manager_.setMotionPermit(new_state == SafetyState::STATE_READY);
}
