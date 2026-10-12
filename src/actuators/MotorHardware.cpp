#include "actuators/MotorHardware.h"

#include "esp_timer.h"
#include "utils/config.h"

namespace
{
    class ArduinoMotorHardware final : public MotorHardware
    {
    public:
        void configureOutput(uint8_t pin) override
        {
            pinMode(pin, OUTPUT);
        }

        void configurePwm(uint8_t pin, uint8_t channel) override
        {
            ledcSetup(channel, PWM_FREQUENCY_HZ, PWM_RESOLUTION_BITS);
            ledcAttachPin(pin, channel);
        }

        void configureServo(uint8_t pin, uint8_t channel) override
        {
            ledcSetup(channel, BRAKE_SERVO_FREQUENCY_HZ, BRAKE_SERVO_RESOLUTION_BITS);
            ledcAttachPin(pin, channel);
        }

        void writeDirection(uint8_t pin, bool high) override
        {
            digitalWrite(pin, high ? HIGH : LOW);
        }

        void writePwm(uint8_t channel, int duty) override
        {
            ledcWrite(channel, duty);
        }

        int64_t nowUs() const override
        {
            return esp_timer_get_time();
        }
    };
}

MotorHardware &defaultMotorHardware()
{
    static ArduinoMotorHardware hardware;
    return hardware;
}
