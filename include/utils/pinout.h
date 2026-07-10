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

// Enable físico de segurança
// HIGH = sistema habilitado
// LOW  = emergência / sistema desabilitado
#define PIN_ENABLE_SAFETY   23
