#include "actuators/BrakeServo.h"
#include "utils/config.h"
#include <cmath>

namespace
{
    constexpr int64_t SERVO_PERIOD_US = 1000000LL / BRAKE_SERVO_FREQUENCY_HZ;
    constexpr int64_t SERVO_MAX_DUTY = (1LL << BRAKE_SERVO_RESOLUTION_BITS) - 1;

    static_assert(BRAKE_FAULT_RATE_PER_S > 0.0f,
                  "BRAKE_FAULT_RATE_PER_S deve ser positivo");
    static_assert(BRAKE_DRIVE_CUTOFF >= 0.0f && BRAKE_DRIVE_CUTOFF < 1.0f,
                  "BRAKE_DRIVE_CUTOFF deve estar em [0, 1)");
#if BRAKE_CALIBRATED
    static_assert(BRAKE_APPLIED_US != BRAKE_RELEASED_US,
                  "Freio calibrado exige BRAKE_APPLIED_US medido");
    static_assert(BRAKE_RELEASED_US >= 500 && BRAKE_RELEASED_US <= 2500 &&
                      BRAKE_APPLIED_US >= 500 && BRAKE_APPLIED_US <= 2500,
                  "Pulsos do servo fora da faixa 500-2500 us");
    static_assert(BRAKE_HOLD_US != BRAKE_RELEASED_US &&
                      (BRAKE_HOLD_US - BRAKE_RELEASED_US) *
                              (BRAKE_HOLD_US - BRAKE_APPLIED_US) <= 0,
                  "BRAKE_HOLD_US deve ficar entre solto e freio total");
#endif

    constexpr int64_t HOLD_CONFIRMATION_US =
        static_cast<int64_t>(BRAKE_HOLD_CONFIRMATION_MS) * 1000LL;

    // Posicao normalizada equivalente a BRAKE_HOLD_US.
    constexpr float HOLD_POSITION =
        BRAKE_APPLIED_US == BRAKE_RELEASED_US
            ? 1.0f
            : static_cast<float>(BRAKE_HOLD_US - BRAKE_RELEASED_US) /
                  static_cast<float>(BRAKE_APPLIED_US - BRAKE_RELEASED_US);

    int positionToPulseUs(float position)
    {
        return static_cast<int>(std::lround(
            BRAKE_RELEASED_US +
            position * static_cast<float>(BRAKE_APPLIED_US - BRAKE_RELEASED_US)));
    }

    int pulseUsToDuty(int pulse_us)
    {
        return static_cast<int>((pulse_us * SERVO_MAX_DUTY + SERVO_PERIOD_US / 2) /
                                SERVO_PERIOD_US);
    }
} // namespace

BrakeServo::BrakeServo(uint8_t pin, uint8_t channel, MotorHardware *hardware)
    : pin_(pin),
      channel_(channel),
      position_(1.0f),
      pulse_us_(0),
      fault_ramp_active_(false),
      last_ramp_update_us_(0),
      hold_active_(false),
      stopped_since_us_(0),
      timestamp_us_(0),
      hardware_(hardware != nullptr ? *hardware : defaultMotorHardware())
{
}

void BrakeServo::begin()
{
    if (BRAKE_CALIBRATED)
    {
        hardware_.configureServo(pin_, channel_);
    }

    fault_ramp_active_ = false;
    hold_active_ = false;
    stopped_since_us_ = 0;
    writePosition(1.0f, hardware_.nowUs());
}

void BrakeServo::setCommand(float normalized_command)
{
    if (!std::isfinite(normalized_command) ||
        normalized_command < 0.0f || normalized_command > 1.0f)
    {
        applyFaultRamp();
        return;
    }

    fault_ramp_active_ = false;
    writePosition(normalized_command, hardware_.nowUs());
}

void BrakeServo::applyFaultRamp()
{
    if (fault_ramp_active_)
    {
        return;
    }

    fault_ramp_active_ = true;
    last_ramp_update_us_ = hardware_.nowUs();
}

void BrakeServo::update(bool vehicle_stopped)
{
    const int64_t now_us = hardware_.nowUs();
    const bool hold_was_active = hold_active_;
    updateHold(vehicle_stopped, now_us);

    if (fault_ramp_active_ && position_ < 1.0f)
    {
        const float dt_s = static_cast<float>(now_us - last_ramp_update_us_) / 1000000.0f;
        last_ramp_update_us_ = now_us;

        if (dt_s > 0.0f)
        {
            writePosition(std::fmin(1.0f, position_ + BRAKE_FAULT_RATE_PER_S * dt_s), now_us);
            return;
        }
    }

    // A posicao nao mudou, mas o limite de pulso sim.
    if (hold_active_ != hold_was_active)
    {
        writePosition(position_, now_us);
    }
}

float BrakeServo::getPosition() const
{
    portENTER_CRITICAL(&snapshot_mux_);
    const float position = position_;
    portEXIT_CRITICAL(&snapshot_mux_);
    return position;
}

BrakeServo::Snapshot BrakeServo::getSnapshot() const
{
    portENTER_CRITICAL(&snapshot_mux_);
    const Snapshot snapshot{position_, pulse_us_, fault_ramp_active_,
                            hold_active_, timestamp_us_};
    portEXIT_CRITICAL(&snapshot_mux_);
    return snapshot;
}

void BrakeServo::updateHold(bool vehicle_stopped, int64_t now_us)
{
    // Qualquer movimento volta ao freio total imediatamente.
    if (!vehicle_stopped)
    {
        stopped_since_us_ = 0;
        hold_active_ = false;
        return;
    }

    if (stopped_since_us_ == 0)
    {
        stopped_since_us_ = now_us;
    }

    hold_active_ = (now_us - stopped_since_us_) >= HOLD_CONFIRMATION_US;
}

void BrakeServo::writePosition(float position, int64_t now_us)
{
    // Parado, o servo nao puxa alem de BRAKE_HOLD_US. A posicao comandada e
    // mantida: o limite so afeta o pulso enviado.
    const float pulse_position = hold_active_ ? std::fmin(position, HOLD_POSITION) : position;

    // Sem calibracao a posicao e acompanhada (intertravamento e telemetria),
    // mas o pino nao e acionado.
    const int pulse_us = BRAKE_CALIBRATED ? positionToPulseUs(pulse_position) : 0;
    if (BRAKE_CALIBRATED)
    {
        hardware_.writePwm(channel_, pulseUsToDuty(pulse_us));
    }

    portENTER_CRITICAL(&snapshot_mux_);
    position_ = position;
    pulse_us_ = pulse_us;
    timestamp_us_ = now_us;
    portEXIT_CRITICAL(&snapshot_mux_);
}
