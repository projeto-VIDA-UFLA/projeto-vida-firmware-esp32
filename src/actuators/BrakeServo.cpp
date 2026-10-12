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
#endif

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

void BrakeServo::update()
{
    if (!fault_ramp_active_ || position_ >= 1.0f)
    {
        return;
    }

    const int64_t now_us = hardware_.nowUs();
    const float dt_s = static_cast<float>(now_us - last_ramp_update_us_) / 1000000.0f;
    last_ramp_update_us_ = now_us;

    if (!(dt_s > 0.0f))
    {
        return;
    }

    writePosition(std::fmin(1.0f, position_ + BRAKE_FAULT_RATE_PER_S * dt_s), now_us);
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
    const Snapshot snapshot{position_, pulse_us_, fault_ramp_active_, timestamp_us_};
    portEXIT_CRITICAL(&snapshot_mux_);
    return snapshot;
}

void BrakeServo::writePosition(float position, int64_t now_us)
{
    // Sem calibracao a posicao e acompanhada (intertravamento e telemetria),
    // mas o pino nao e acionado.
    const int pulse_us = BRAKE_CALIBRATED ? positionToPulseUs(position) : 0;
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
