#include "comunication/CommandMailbox.h"

#include "esp_timer.h"

CommandMailbox::CommandMailbox()
    : steering_reference_{},
      drive_reference_{},
      logical_enable_{false, 0, 0, false},
      steering_mux_(portMUX_INITIALIZER_UNLOCKED),
      drive_mux_(portMUX_INITIALIZER_UNLOCKED),
      logical_enable_mux_(portMUX_INITIALIZER_UNLOCKED)
{
}

void CommandMailbox::updateSteeringReference(float angle_deg,
                                             float angular_velocity_deg_s,
                                             int64_t source_timestamp_ns)
{
    const int64_t received_at_us = esp_timer_get_time();

    portENTER_CRITICAL(&steering_mux_);
    steering_reference_.angle_deg = angle_deg;
    steering_reference_.angular_velocity_deg_s = angular_velocity_deg_s;
    steering_reference_.source_timestamp_ns = source_timestamp_ns;
    steering_reference_.received_at_us = received_at_us;
    steering_reference_.received = true;
    portEXIT_CRITICAL(&steering_mux_);
}

void CommandMailbox::updateDriveReference(float velocity_m_s,
                                          float acceleration_m_s2,
                                          float brake_command,
                                          int64_t source_timestamp_ns)
{
    const int64_t received_at_us = esp_timer_get_time();

    portENTER_CRITICAL(&drive_mux_);
    drive_reference_.velocity_m_s = velocity_m_s;
    drive_reference_.acceleration_m_s2 = acceleration_m_s2;
    drive_reference_.brake_command = brake_command;
    drive_reference_.source_timestamp_ns = source_timestamp_ns;
    drive_reference_.received_at_us = received_at_us;
    drive_reference_.received = true;
    portEXIT_CRITICAL(&drive_mux_);
}

void CommandMailbox::updateLogicalEnable(bool enabled, int64_t source_timestamp_ns)
{
    const int64_t received_at_us = esp_timer_get_time();

    portENTER_CRITICAL(&logical_enable_mux_);
    logical_enable_.enabled = enabled;
    logical_enable_.source_timestamp_ns = source_timestamp_ns;
    logical_enable_.received_at_us = received_at_us;
    logical_enable_.received = true;
    portEXIT_CRITICAL(&logical_enable_mux_);
}

CommandMailbox::SteeringReferenceSnapshot CommandMailbox::getSteeringReferenceSnapshot() const
{
    portENTER_CRITICAL(&steering_mux_);
    const SteeringReferenceSnapshot snapshot = steering_reference_;
    portEXIT_CRITICAL(&steering_mux_);
    return snapshot;
}

CommandMailbox::DriveReferenceSnapshot CommandMailbox::getDriveReferenceSnapshot() const
{
    portENTER_CRITICAL(&drive_mux_);
    const DriveReferenceSnapshot snapshot = drive_reference_;
    portEXIT_CRITICAL(&drive_mux_);
    return snapshot;
}

CommandMailbox::LogicalEnableSnapshot CommandMailbox::getLogicalEnableSnapshot() const
{
    portENTER_CRITICAL(&logical_enable_mux_);
    const LogicalEnableSnapshot snapshot = logical_enable_;
    portEXIT_CRITICAL(&logical_enable_mux_);
    return snapshot;
}
