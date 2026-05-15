#include "SerialCommand.h"
#include "config.h"

SerialCommand::SerialCommand(VehicleController& controller,
                             Watchdog& command_watchdog,
                             EncoderReader& rear_encoder,
                             EncoderReader& left_encoder,
                             EncoderReader& right_encoder)
    : controller_(controller),
      command_watchdog_(command_watchdog),
      rear_encoder_(rear_encoder),
      left_encoder_(left_encoder),
      right_encoder_(right_encoder),
      software_enable_(false),
      odroid_ready_(false),
      buffer_()
{
}

void SerialCommand::begin()
{
    software_enable_ = false;
    odroid_ready_ = false;
    buffer_.reserve(80);
}

void SerialCommand::update()
{
    while (Serial.available() > 0)
    {
        char c = static_cast<char>(Serial.read());

        if (c == '\n' || c == '\r')
        {
            if (buffer_.length() > 0)
            {
                processCommand(buffer_);
                buffer_ = "";
            }
        }
        else
        {
            buffer_ += c;
        }
    }
}

bool SerialCommand::getSoftwareEnable() const
{
    return software_enable_;
}

bool SerialCommand::isOdroidReady() const
{
#if REQUIRE_ODROID_HANDSHAKE
    return odroid_ready_;
#else
    return true;
#endif
}

void SerialCommand::processCommand(const String& line)
{
    String cmd = line;
    cmd.trim();

    int space_index = cmd.indexOf(' ');
    String key = space_index >= 0 ? cmd.substring(0, space_index) : cmd;
    String value = space_index >= 0 ? cmd.substring(space_index + 1) : "";

    key.trim();
    value.trim();

    if (key == "odroid_ready")
    {
        odroid_ready_ = value.length() == 0 || value.toInt() != 0;
        if (!odroid_ready_)
        {
            software_enable_ = false;
            controller_.setSpeedTarget(0.0f, DRIVE_DEFAULT_MAX_ACCEL_M_S2);
            controller_.setSteeringTarget(0.0f, STEERING_DEFAULT_MAX_VEL_DEG_S);
        }

        Serial.printf("[HANDSHAKE] odroid_ready=%d\n", odroid_ready_);
        if (odroid_ready_)
        {
            command_watchdog_.feed();
            Serial.println("esp_ready");
        }
    }
    else if (requiresOdroidReady(key))
    {
        Serial.print("[HANDSHAKE] ignored before odroid_ready: ");
        Serial.println(cmd);
    }
    else if (key == "enable")
    {
        software_enable_ = value.toInt() != 0;
        command_watchdog_.feed();
        Serial.printf("[CMD] software_enable=%d\n", software_enable_);
    }
    else if (key == "speed")
    {
        float velocity_m_s = 0.0f;
        float max_accel_m_s2 = DRIVE_DEFAULT_MAX_ACCEL_M_S2;
        if (!parseTwoFloats(value, velocity_m_s, max_accel_m_s2))
        {
            Serial.println("[CMD] usage: speed <m_s> <max_accel_m_s2>");
            return;
        }

        controller_.setSpeedTarget(velocity_m_s, max_accel_m_s2);
        command_watchdog_.feed();
        Serial.printf(
            "[CMD] speed target=%.3f m/s accel_max=%.3f m/s2\n",
            controller_.getTargetVelocityMps(),
            controller_.getMaxAccelMps2()
        );
    }
    else if (key == "steer_target")
    {
        float target_deg = 0.0f;
        float max_velocity_deg_s = STEERING_DEFAULT_MAX_VEL_DEG_S;
        if (!parseTwoFloats(value, target_deg, max_velocity_deg_s))
        {
            Serial.println("[CMD] usage: steer_target <deg> <max_vel_deg_s>");
            return;
        }

        controller_.setSteeringTarget(target_deg, max_velocity_deg_s);
        command_watchdog_.feed();
        Serial.printf(
            "[CMD] steer target=%.3f deg vel_max=%.3f deg/s\n",
            controller_.getSteeringTargetDeg(),
            controller_.getSteeringVelocityMaxDegS()
        );
    }
    else if (key == "reset_enc")
    {
        rear_encoder_.reset();
        left_encoder_.reset();
        right_encoder_.reset();
        Serial.println("[CMD] encoders reset");
    }
    else if (key == "status")
    {
        Serial.println("[CMD] status requested");
    }
    else
    {
        Serial.print("[CMD] unknown: ");
        Serial.println(cmd);
    }
}

bool SerialCommand::requiresOdroidReady(const String& key) const
{
#if REQUIRE_ODROID_HANDSHAKE
    if (odroid_ready_)
    {
        return false;
    }

    return key == "enable" ||
           key == "speed" ||
           key == "steer_target";
#else
    (void)key;
    return false;
#endif
}

bool SerialCommand::parseTwoFloats(const String& value, float& first, float& second) const
{
    String args = value;
    args.trim();

    int space_index = args.indexOf(' ');
    if (space_index < 0)
    {
        return false;
    }

    String first_text = args.substring(0, space_index);
    String second_text = args.substring(space_index + 1);
    first_text.trim();
    second_text.trim();

    if (first_text.length() == 0 || second_text.length() == 0)
    {
        return false;
    }

    first = first_text.toFloat();
    second = second_text.toFloat();
    return true;
}
