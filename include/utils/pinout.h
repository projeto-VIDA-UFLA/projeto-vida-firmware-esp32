#pragma once

// ============================================================
// Pinagem ESP32 DevKit 38 pinos - Projeto VIDA
// ============================================================

// Encoders em quadratura
#define ENC_REAR_A   19
#define ENC_REAR_B   18

#define ENC_RIGHT_A 35
#define ENC_RIGHT_B 34

#define ENC_LEFT_A 33
#define ENC_LEFT_B 32

// Sensor analógico de esterçamento / LVDT / potenciômetro
#define PIN_LVDT     36  // ADC1_CH0

// Motor de esterçamento - 2 PWMs independentes
#define PIN_STEER_RIGHT_PWM 25
#define PIN_STEER_LEFT_PWM  26

// Motor longitudinal - PWM + direção
#define PIN_DRIVE_PWM       27

// Sugestão inicial. Altere se houver conflito no hardware.
#define PIN_DRIVE_REVERSE   14

// Servo do freio (cabo de aco)
#define PIN_BRAKE_SERVO     4

// Barramento I2C do monitor de bateria (INA228)
#define PIN_I2C_SDA 21
#define PIN_I2C_SCL 22
// Endereco I2C de 7 bits do INA228 (A0 e A1 em GND)
#define INA228_I2C_ADDRESS 0x40

// Enable físico de segurança
// HIGH = painel/contator fisicamente habilitado
// LOW  = painel fisicamente desabilitado (causa não identificável pelo ESP32)
#define PIN_ENABLE_SAFETY   13
