#include <Arduino.h>
#include <unity.h>

#include <cmath>
#include <vector>

#include "actuators/ActuatorManager.h"
#include "actuators/DriveMotor.h"
#include "actuators/MotorHardware.h"
#include "actuators/SteeringMotor.h"
#include "esp_timer.h"
#include "safety/SafetyWatchdogs.h"
#include "utils/config.h"

namespace
{
    class FakeHardware final : public MotorHardware
    {
    public:
        void configureOutput(uint8_t) override {}
        void configurePwm(uint8_t, uint8_t) override {}
        void writeDirection(uint8_t, bool high) override { direction = high; }
        void writePwm(uint8_t channel, int duty) override
        {
            writes.push_back({channel, duty});
            if (channel == 0) drive = duty;
            if (channel == 1) right = duty;
            if (channel == 2) left = duty;
            simultaneous |= right > 0 && left > 0;
        }
        int64_t nowUs() const override { return now_us; }
        void advanceMs(int64_t ms) { now_us += ms * 1000LL; }

        struct Write { uint8_t channel; int duty; };
        int64_t now_us = 1000000;
        int drive = 0;
        int right = 0;
        int left = 0;
        bool direction = false;
        bool simultaneous = false;
        std::vector<Write> writes;
    };

    DriveMotor::VelocityMeasurement speed(FakeHardware &hw, float value,
                                           bool valid = true)
    {
        return {value, hw.nowUs(), valid};
    }

    SafetyWatchdogs::Input healthyWatchdogs()
    {
        constexpr int64_t now = 1000000;
        return {now, 500000, now - 1000, true, now - 1000,
                true, true, 0.5f, now - 1000};
    }

    void testSteeringStartsStoppedAndRunsPositive()
    {
        FakeHardware hw;
        SteeringMotor motor(10, 11, 1, 2, &hw);
        motor.begin();
        TEST_ASSERT_EQUAL(0, motor.getRightPwm());
        TEST_ASSERT_EQUAL(0, motor.getLeftPwm());
        motor.setCommand(0.5f);
        TEST_ASSERT_GREATER_THAN(0, motor.getRightPwm());
        TEST_ASSERT_EQUAL(0, motor.getLeftPwm());
    }

    void testSteeringReversalHasBothOffStateAndNoOverlap()
    {
        FakeHardware hw;
        SteeringMotor motor(10, 11, 1, 2, &hw);
        motor.begin();
        motor.setCommand(0.5f);
        motor.setCommand(-0.5f);
        TEST_ASSERT_EQUAL(0, motor.getRightPwm());
        TEST_ASSERT_EQUAL(0, motor.getLeftPwm());
        TEST_ASSERT_GREATER_OR_EQUAL(2, hw.writes.size());
        TEST_ASSERT_EQUAL(1, hw.writes[hw.writes.size() - 2].channel);
        TEST_ASSERT_EQUAL(0, hw.writes[hw.writes.size() - 2].duty);
        TEST_ASSERT_EQUAL(2, hw.writes[hw.writes.size() - 1].channel);
        TEST_ASSERT_EQUAL(0, hw.writes[hw.writes.size() - 1].duty);
        TEST_ASSERT_EQUAL_INT(
            static_cast<int>(SteeringMotor::State::REVERSAL_DEADTIME),
            static_cast<int>(motor.getState()));
        hw.now_us += STEERING_DIRECTION_DEADTIME_US;
        motor.setCommand(-0.5f);
        TEST_ASSERT_EQUAL(0, motor.getRightPwm());
        TEST_ASSERT_GREATER_THAN(0, motor.getLeftPwm());
        TEST_ASSERT_FALSE(hw.simultaneous);
    }

    void testEmergencyAndInvalidCommandsStopAllPwm()
    {
        FakeHardware hw;
        DriveMotor drive(12, 13, 0, &hw);
        SteeringMotor steering(10, 11, 1, 2, &hw);
        ActuatorManager manager(drive, steering);
        drive.begin();
        steering.begin();
        TEST_ASSERT_TRUE(manager.begin());
        manager.setMotionPermit(true);
        const int64_t now = esp_timer_get_time();
        TEST_ASSERT_TRUE(manager.applyCommands(
            {0.0f, 0.5f, now, true}, {0.0f, now, true}));
        manager.emergencyStop();
        TEST_ASSERT_EQUAL(0, drive.getLastPwm());
        TEST_ASSERT_EQUAL(0, steering.getRightPwm());
        TEST_ASSERT_EQUAL(0, steering.getLeftPwm());
        steering.setCommand(NAN);
        TEST_ASSERT_EQUAL(0, steering.getRightPwm());
        steering.setCommand(INFINITY);
        TEST_ASSERT_EQUAL(0, steering.getLeftPwm());
    }

    void testDriveReversalRequiresConfirmedMeasuredStop()
    {
        FakeHardware hw;
        DriveMotor motor(12, 13, 0, &hw);
        motor.begin();
        motor.setCommand(0.6f, speed(hw, 0.2f));
        TEST_ASSERT_GREATER_THAN(0, motor.getLastPwm());
        motor.setCommand(-0.6f, speed(hw, 0.2f));
        TEST_ASSERT_EQUAL(0, motor.getLastPwm());
        TEST_ASSERT_FALSE(hw.direction);
        motor.setCommand(-0.6f, speed(hw, 0.0f));
        hw.advanceMs(DRIVE_STOP_CONFIRMATION_TIME_MS);
        motor.setCommand(-0.6f, speed(hw, 0.0f));
        TEST_ASSERT_TRUE(hw.direction);
        TEST_ASSERT_EQUAL(0, motor.getLastPwm());
        hw.advanceMs(DRIVE_DIRECTION_DEADTIME_MS);
        motor.setCommand(-0.6f, speed(hw, 0.0f));
        TEST_ASSERT_GREATER_THAN(0, motor.getLastPwm());
    }

    void testExpiredSpeedBlocksDirectionChange()
    {
        FakeHardware hw;
        DriveMotor motor(12, 13, 0, &hw);
        motor.begin();
        motor.setCommand(0.5f, speed(hw, 0.0f));
        DriveMotor::VelocityMeasurement expired{
            0.0f,
            hw.nowUs() -
                static_cast<int64_t>(DRIVE_VELOCITY_MEASUREMENT_TIMEOUT_MS + 1) *
                    1000LL,
            true};
        motor.setCommand(-0.5f, expired);
        hw.advanceMs(DRIVE_STOP_CONFIRMATION_TIME_MS + 1);
        motor.setCommand(-0.5f, expired);
        TEST_ASSERT_FALSE(hw.direction);
        TEST_ASSERT_EQUAL(0, motor.getLastPwm());
    }

    void testSensorHeartbeatAndSteeringSnapshotWatchdogs()
    {
        auto input = healthyWatchdogs();
        auto result = SafetyWatchdogs::evaluate(input);
        TEST_ASSERT_TRUE(result.sensor_task_healthy);
        TEST_ASSERT_FALSE(result.sensor_task_timed_out);
        TEST_ASSERT_TRUE(result.steering_snapshot_fresh);

        input.sensor_task_heartbeat_us =
            input.now_us -
            static_cast<int64_t>(SENSOR_TASK_WATCHDOG_TIMEOUT_MS + 1) * 1000LL;
        result = SafetyWatchdogs::evaluate(input);
        TEST_ASSERT_TRUE(result.sensor_task_timed_out);

        input = healthyWatchdogs();
        input.steering_snapshot_us =
            input.now_us -
            static_cast<int64_t>(STEERING_SENSOR_SNAPSHOT_TIMEOUT_MS + 1) *
                1000LL;
        TEST_ASSERT_FALSE(
            SafetyWatchdogs::evaluate(input).steering_snapshot_fresh);
    }

    void testControlOutputWatchdogAndDisabledPolicy()
    {
        auto input = healthyWatchdogs();
        TEST_ASSERT_TRUE(SafetyWatchdogs::evaluate(input).control_output_fresh);
        input.control_output_us =
            input.now_us -
            static_cast<int64_t>(CONTROL_OUTPUT_WATCHDOG_TIMEOUT_MS + 1) *
                1000LL;
        TEST_ASSERT_TRUE(
            SafetyWatchdogs::evaluate(input).control_output_timed_out);
        input.control_command = NAN;
        TEST_ASSERT_TRUE(
            SafetyWatchdogs::evaluate(input).control_output_timed_out);
        input.control_command = INFINITY;
        TEST_ASSERT_TRUE(
            SafetyWatchdogs::evaluate(input).control_output_timed_out);
        input.control_required = false;
        TEST_ASSERT_FALSE(
            SafetyWatchdogs::evaluate(input).control_output_timed_out);
    }

    void testActuatorPathAcceptsRecentAndZerosExpiredOutput()
    {
        FakeHardware hw;
        DriveMotor drive(12, 13, 0, &hw);
        SteeringMotor steering(10, 11, 1, 2, &hw);
        ActuatorManager manager(drive, steering);
        drive.begin();
        steering.begin();
        TEST_ASSERT_TRUE(manager.begin());
        const int64_t now = esp_timer_get_time();
        const DriveMotor::VelocityMeasurement stopped{0.0f, now, true};
        ActuatorManager::TimedCommand recent{0.0f, 0.5f, now, true};

        TEST_ASSERT_FALSE(manager.applyCommands(recent, stopped));
        TEST_ASSERT_EQUAL(0, steering.getRightPwm());
        manager.setMotionPermit(true);
        recent.produced_at_us = esp_timer_get_time();
        TEST_ASSERT_TRUE(manager.applyCommands(recent, stopped));
        TEST_ASSERT_GREATER_THAN(0, steering.getRightPwm());

        const ActuatorManager::TimedCommand expired{
            0.0f, 0.5f,
            now - static_cast<int64_t>(
                      CONTROL_OUTPUT_WATCHDOG_TIMEOUT_MS + 1) *
                      1000LL,
            true};
        TEST_ASSERT_FALSE(manager.applyCommands(expired, stopped));
        TEST_ASSERT_EQUAL(0, steering.getRightPwm());
        TEST_ASSERT_EQUAL(0, steering.getLeftPwm());
    }
}

void setup()
{
    delay(2000);
    UNITY_BEGIN();
    RUN_TEST(testSteeringStartsStoppedAndRunsPositive);
    RUN_TEST(testSteeringReversalHasBothOffStateAndNoOverlap);
    RUN_TEST(testEmergencyAndInvalidCommandsStopAllPwm);
    RUN_TEST(testDriveReversalRequiresConfirmedMeasuredStop);
    RUN_TEST(testExpiredSpeedBlocksDirectionChange);
    RUN_TEST(testSensorHeartbeatAndSteeringSnapshotWatchdogs);
    RUN_TEST(testControlOutputWatchdogAndDisabledPolicy);
    RUN_TEST(testActuatorPathAcceptsRecentAndZerosExpiredOutput);
    UNITY_END();
}

void loop() {}
