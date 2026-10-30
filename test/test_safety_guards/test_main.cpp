#include <Arduino.h>
#include <unity.h>

#include <cmath>
#include <vector>

#include "actuators/ActuatorManager.h"
#include "actuators/BrakeServo.h"
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
        void configureServo(uint8_t, uint8_t) override {}
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
        BrakeServo brake(4, 4, &hw);
        ActuatorManager manager(drive, steering, brake);
        drive.begin();
        steering.begin();
        TEST_ASSERT_TRUE(manager.begin());
        manager.setMotionPermit(true);
        const int64_t now = esp_timer_get_time();
        TEST_ASSERT_TRUE(manager.applyCommands(
            {0.0f, 0.5f, 0.0f, now, true}, {0.0f, now, true}));
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

        // Carimbo gravado por outro nucleo depois da leitura de now_us: fresco.
        input = healthyWatchdogs();
        input.sensor_task_heartbeat_us = input.now_us + 100;
        input.steering_snapshot_us = input.now_us + 100;
        result = SafetyWatchdogs::evaluate(input);
        TEST_ASSERT_FALSE(result.sensor_task_timed_out);
        TEST_ASSERT_TRUE(result.steering_snapshot_fresh);
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
        BrakeServo brake(4, 4, &hw);
        ActuatorManager manager(drive, steering, brake);
        drive.begin();
        steering.begin();
        TEST_ASSERT_TRUE(manager.begin());
        const int64_t now = esp_timer_get_time();
        const DriveMotor::VelocityMeasurement stopped{0.0f, now, true};
        ActuatorManager::TimedCommand recent{0.0f, 0.5f, 0.0f, now, true};

        TEST_ASSERT_FALSE(manager.applyCommands(recent, stopped));
        TEST_ASSERT_EQUAL(0, steering.getRightPwm());
        manager.setMotionPermit(true);
        recent.produced_at_us = esp_timer_get_time();
        TEST_ASSERT_TRUE(manager.applyCommands(recent, stopped));
        TEST_ASSERT_GREATER_THAN(0, steering.getRightPwm());

        const ActuatorManager::TimedCommand expired{
            0.0f, 0.5f, 0.0f,
            now - static_cast<int64_t>(
                      CONTROL_OUTPUT_WATCHDOG_TIMEOUT_MS + 1) *
                      1000LL,
            true};
        TEST_ASSERT_FALSE(manager.applyCommands(expired, stopped));
        TEST_ASSERT_EQUAL(0, steering.getRightPwm());
        TEST_ASSERT_EQUAL(0, steering.getLeftPwm());
    }

    void testBrakeStartsAppliedAndFollowsCommand()
    {
        FakeHardware hw;
        BrakeServo brake(4, 4, &hw);
        brake.begin();
        TEST_ASSERT_EQUAL_FLOAT(1.0f, brake.getPosition());
        brake.setCommand(0.0f);
        TEST_ASSERT_EQUAL_FLOAT(0.0f, brake.getPosition());
        brake.setCommand(0.4f);
        TEST_ASSERT_EQUAL_FLOAT(0.4f, brake.getPosition());
        TEST_ASSERT_FALSE(brake.getSnapshot().fault_ramp_active);
    }

    void testBrakeFaultRampsToFullAtConfiguredRate()
    {
        FakeHardware hw;
        BrakeServo brake(4, 4, &hw);
        brake.begin();
        brake.setCommand(0.0f);
        brake.applyFaultRamp();
        hw.advanceMs(100);
        brake.update(false);
        TEST_ASSERT_FLOAT_WITHIN(1e-4f, BRAKE_FAULT_RATE_PER_S * 0.1f,
                                 brake.getPosition());
        // Repetir a falha nao reinicia nem acelera a rampa.
        brake.applyFaultRamp();
        hw.advanceMs(100);
        brake.update(false);
        TEST_ASSERT_FLOAT_WITHIN(1e-4f, BRAKE_FAULT_RATE_PER_S * 0.2f,
                                 brake.getPosition());
        hw.advanceMs(static_cast<int64_t>(2000.0f / BRAKE_FAULT_RATE_PER_S));
        brake.update(false);
        TEST_ASSERT_EQUAL_FLOAT(1.0f, brake.getPosition());
    }

    void testBrakeHoldNeedsConfirmedStopAndDropsOnMotion()
    {
        FakeHardware hw;
        BrakeServo brake(4, 4, &hw);
        brake.begin();
        brake.update(true);
        TEST_ASSERT_FALSE(brake.getSnapshot().hold_active);
        hw.advanceMs(BRAKE_HOLD_CONFIRMATION_MS - 1);
        brake.update(true);
        TEST_ASSERT_FALSE(brake.getSnapshot().hold_active);
        hw.advanceMs(1);
        brake.update(true);
        TEST_ASSERT_TRUE(brake.getSnapshot().hold_active);
        // O alivio limita so o pulso: a posicao comandada continua total.
        TEST_ASSERT_EQUAL_FLOAT(1.0f, brake.getPosition());

        // Movimento volta ao freio total imediatamente e reinicia a contagem.
        brake.update(false);
        TEST_ASSERT_FALSE(brake.getSnapshot().hold_active);
        brake.update(true);
        hw.advanceMs(BRAKE_HOLD_CONFIRMATION_MS - 1);
        brake.update(true);
        TEST_ASSERT_FALSE(brake.getSnapshot().hold_active);
    }

    void testBrakeInvalidCommandStartsFaultRamp()
    {
        const float invalid[] = {NAN, INFINITY, -0.1f, 1.1f};
        for (float command : invalid)
        {
            FakeHardware hw;
            BrakeServo brake(4, 4, &hw);
            brake.begin();
            brake.setCommand(0.0f);
            brake.setCommand(command);
            TEST_ASSERT_EQUAL_FLOAT(0.0f, brake.getPosition());
            TEST_ASSERT_TRUE(brake.getSnapshot().fault_ramp_active);
        }
    }

    void testBrakeCutsDriveAndRampsWhenMotionBlocked()
    {
        FakeHardware hw;
        DriveMotor drive(12, 13, 0, &hw);
        SteeringMotor steering(10, 11, 1, 2, &hw);
        BrakeServo brake(4, 4, &hw);
        ActuatorManager manager(drive, steering, brake);
        drive.begin();
        steering.begin();
        brake.begin();
        TEST_ASSERT_TRUE(manager.begin());
        manager.setMotionPermit(true);
        const DriveMotor::VelocityMeasurement stopped{0.0f, hw.nowUs(), true};

        TEST_ASSERT_TRUE(manager.applyCommands(
            {0.5f, 0.0f, 0.0f, esp_timer_get_time(), true}, stopped));
        TEST_ASSERT_GREATER_THAN(0, drive.getLastPwm());

        // Freio acionado: tracao zerada, esterçamento continua.
        TEST_ASSERT_TRUE(manager.applyCommands(
            {0.5f, 0.5f, 0.5f, esp_timer_get_time(), true}, stopped));
        TEST_ASSERT_EQUAL(0, drive.getLastPwm());
        TEST_ASSERT_GREATER_THAN(0, steering.getRightPwm());

        // Freio fora da faixa invalida o comando inteiro.
        TEST_ASSERT_FALSE(manager.applyCommands(
            {0.5f, 0.0f, 1.5f, esp_timer_get_time(), true}, stopped));
        TEST_ASSERT_EQUAL(0, drive.getLastPwm());

        // Bloqueio de seguranca: freio em rampa ate total.
        TEST_ASSERT_TRUE(manager.applyCommands(
            {0.0f, 0.0f, 0.0f, esp_timer_get_time(), true}, stopped));
        manager.setMotionPermit(false);
        TEST_ASSERT_TRUE(brake.getSnapshot().fault_ramp_active);
        hw.advanceMs(static_cast<int64_t>(2000.0f / BRAKE_FAULT_RATE_PER_S));
        manager.updateBrake(false);
        TEST_ASSERT_EQUAL_FLOAT(1.0f, brake.getPosition());
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
    RUN_TEST(testBrakeStartsAppliedAndFollowsCommand);
    RUN_TEST(testBrakeFaultRampsToFullAtConfiguredRate);
    RUN_TEST(testBrakeHoldNeedsConfirmedStopAndDropsOnMotion);
    RUN_TEST(testBrakeInvalidCommandStartsFaultRamp);
    RUN_TEST(testBrakeCutsDriveAndRampsWhenMotionBlocked);
    UNITY_END();
}

void loop() {}
