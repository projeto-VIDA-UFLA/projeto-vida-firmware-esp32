#include "sensors/VehicleMotion.h"

#include <cmath>
#include "esp_timer.h"
#include "utils/config.h"

VehicleMotion::VehicleMotion(EncoderReader &rear_encoder,
                             EncoderReader &right_encoder,
                             EncoderReader &left_encoder)
    : rear_encoder_(rear_encoder),
      right_encoder_(right_encoder),
      left_encoder_(left_encoder)
{
}

bool VehicleMotion::isStopped()
{
    const int64_t now_us = esp_timer_get_time();
    return encoderReportsStopped(rear_encoder_.getMeasurementSnapshot(), now_us) &&
           encoderReportsStopped(right_encoder_.getMeasurementSnapshot(), now_us) &&
           encoderReportsStopped(left_encoder_.getMeasurementSnapshot(), now_us);
}

DriveMotor::VelocityMeasurement VehicleMotion::getStopDetectionVelocity()
{
    const EncoderReader::Snapshot snapshots[] = {
        rear_encoder_.getMeasurementSnapshot(),
        right_encoder_.getMeasurementSnapshot(),
        left_encoder_.getMeasurementSnapshot()};

    DriveMotor::VelocityMeasurement measurement{0.0f, snapshots[0].timestamp_us, true};
    for (const EncoderReader::Snapshot &snapshot : snapshots)
    {
        const float velocity_m_s = snapshot.filtered_velocity_m_s;
        if (!std::isfinite(velocity_m_s) || snapshot.timestamp_us <= 0)
        {
            measurement.valid = false;
            continue;
        }

        if (std::fabs(velocity_m_s) > std::fabs(measurement.velocity_m_s))
        {
            measurement.velocity_m_s = velocity_m_s;
        }
        if (snapshot.timestamp_us < measurement.timestamp_us)
        {
            measurement.timestamp_us = snapshot.timestamp_us;
        }
    }

    return measurement;
}

bool VehicleMotion::encoderReportsStopped(const EncoderReader::Snapshot &snapshot,
                                          int64_t now_us)
{
    constexpr int64_t timeout_us =
        static_cast<int64_t>(DRIVE_VELOCITY_MEASUREMENT_TIMEOUT_MS) * 1000LL;

    // Mesmo limiar (<=) usado pelo DriveMotor para autorizar a troca de DIR.
    return snapshot.timestamp_us > 0 &&
           now_us >= snapshot.timestamp_us &&
           (now_us - snapshot.timestamp_us) <= timeout_us &&
           std::isfinite(snapshot.filtered_velocity_m_s) &&
           std::fabs(snapshot.filtered_velocity_m_s) <= DRIVE_STOP_SPEED_THRESHOLD_MPS;
}
