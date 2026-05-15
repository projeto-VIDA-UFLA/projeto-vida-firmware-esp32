#pragma once
#include <Arduino.h>
#include "VehicleController.h"
#include "Watchdog.h"
#include "EncoderReader.h"

class SerialCommand
{
public:
    SerialCommand(VehicleController& controller,
                  Watchdog& command_watchdog,
                  EncoderReader& rear_encoder,
                  EncoderReader& left_encoder,
                  EncoderReader& right_encoder);

    void begin();
    void update();

    bool getSoftwareEnable() const;
    bool isOdroidReady() const;

private:
    VehicleController& controller_;
    Watchdog& command_watchdog_;
    EncoderReader& rear_encoder_;
    EncoderReader& left_encoder_;
    EncoderReader& right_encoder_;

    bool software_enable_;
    bool odroid_ready_;
    String buffer_;

    void processCommand(const String& line);
    bool requiresOdroidReady(const String& key) const;
    bool parseTwoFloats(const String& value, float& first, float& second) const;
};
