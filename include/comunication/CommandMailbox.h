#pragma once

#include <Arduino.h>

class CommandMailbox
{
public:
    struct SteeringReferenceSnapshot
    {
        float angle_deg;
        float angular_velocity_deg_s;
        int64_t source_timestamp_ns;
        int64_t received_at_us;
        bool received;
    };

    struct DriveReferenceSnapshot
    {
        float velocity_m_s;
        float acceleration_m_s2;
        int64_t source_timestamp_ns;
        int64_t received_at_us;
        bool received;
    };

    struct LogicalEnableSnapshot
    {
        bool enabled;
        int64_t source_timestamp_ns;
        int64_t received_at_us;
        bool received;
    };

    CommandMailbox();

    void updateSteeringReference(float angle_deg,
                                 float angular_velocity_deg_s,
                                 int64_t source_timestamp_ns);
    void updateDriveReference(float velocity_m_s,
                              float acceleration_m_s2,
                              int64_t source_timestamp_ns);
    void updateLogicalEnable(bool enabled, int64_t source_timestamp_ns);

    SteeringReferenceSnapshot getSteeringReferenceSnapshot() const;
    DriveReferenceSnapshot getDriveReferenceSnapshot() const;
    LogicalEnableSnapshot getLogicalEnableSnapshot() const;

private:
    SteeringReferenceSnapshot steering_reference_;
    DriveReferenceSnapshot drive_reference_;
    LogicalEnableSnapshot logical_enable_;

    mutable portMUX_TYPE steering_mux_;
    mutable portMUX_TYPE drive_mux_;
    mutable portMUX_TYPE logical_enable_mux_;
};
