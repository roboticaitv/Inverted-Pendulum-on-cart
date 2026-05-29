#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H
// Configuration
#define PWM_FREQ 20000
#define PWM_RES 8
#define IPROPI_R_OHMS 1500.0f
#define IPROPI_RATIO 1000.0f

class DRV8251 {
private:
  int _in1_pin;
  int _in2_pin;
  int _sense_pin;
  uint8_t _pwm_ch;
  int _last_dir = 0;  // 0=Stop, 1=Fwd, -1=Rev

  // ---> THE DEADBAND VARIABLE <---
  // Adjust this number (e.g., 15-35) until the motors perfectly
  // overcome their static friction when given a PWM command of 1.
  int _deadband = 0;

  // Current Sensing Variables
  uint32_t _ipropi_acc = 0;
  uint16_t _ipropi_samples = 0;
  float _motor_current_A = 0.0f;

  // constants defined inside class to avoid conflicts
  static constexpr float R_OHMS = 1500.0f;
  static constexpr float RATIO = 1000.0f;

public:
  DRV8251(int in1, int in2, int sensePin, uint8_t pwmCh) {
    _in1_pin = in1;
    _in2_pin = in2;
    _sense_pin = sensePin;
    _pwm_ch = pwmCh;
  }

  void begin() {
    pinMode(_in1_pin, OUTPUT);
    pinMode(_in2_pin, OUTPUT);

    // Only set up sense pin if it's a valid pin (not -1)
    if (_sense_pin >= 0) {
      pinMode(_sense_pin, INPUT);
    }

    ledcSetup(_pwm_ch, 20000, 8);  // 20kHz, 8-bit
    stop();
  }

  void stop() {
    // THE FIX: If we are already stopped, do nothing and return immediately!
    if (_last_dir == 0) return;

    // Detach PWM from both pins to ensure they are standard GPIOs
    ledcDetachPin(_in1_pin);
    ledcDetachPin(_in2_pin);

    // Force both LOW (Coast Mode)
    digitalWrite(_in1_pin, LOW);
    digitalWrite(_in2_pin, LOW);

    _last_dir = 0;
  }

  void updateCurrentSense() {
    if (_sense_pin < 0) return;  // Skip if disabled

    _ipropi_acc += analogRead(_sense_pin);
    _ipropi_samples++;

    if (_ipropi_samples >= 64) {
      float adc_avg = (float)_ipropi_acc / _ipropi_samples;
      float v_ipropi = (adc_avg / 4095.0f) * 3.3f;
      _motor_current_A = (v_ipropi / R_OHMS) * RATIO;

      _ipropi_acc = 0;
      _ipropi_samples = 0;
    }
  }

  void setSpeed(int pwm) {
    pwm = constrain(pwm, -255, 255);

    // DO NOT STOP MOTOR COMPLETELY IN CONTROL LOOP
    if (pwm == 0) {
      pwm = 0;  // keep direction active, still write PWM
    }

    if (pwm > 0) {
      if (_last_dir != 1) {
        ledcDetachPin(_in1_pin);
        pinMode(_in1_pin, OUTPUT);
        digitalWrite(_in1_pin, HIGH);

        ledcAttachPin(_in2_pin, _pwm_ch);
        _last_dir = 1;
      }
      ledcWrite(_pwm_ch, 255 - pwm);
    } else {
      if (_last_dir != -1) {
        ledcDetachPin(_in2_pin);
        pinMode(_in2_pin, OUTPUT);
        digitalWrite(_in2_pin, HIGH);

        ledcAttachPin(_in1_pin, _pwm_ch);
        _last_dir = -1;
      }
      ledcWrite(_pwm_ch, 255 - abs(pwm));
    }
  }

  float getCurrent() {
    return _motor_current_A;
  }
};

// --- Global Settings ---
// Ensure we use the correct resolution/attenuation globally
void setupADC() {
  analogReadResolution(12);        // 0–4095
  analogSetAttenuation(ADC_11db);  // 0–3.3 V
}

class SimpleServo {
private:
  uint8_t _pin;
  uint8_t _channel;

  // Servo Settings (Standard 360 Servo)
  // 50Hz = 20ms period
  // 14-bit resolution = 16383 counts
  const int STOP_US = 1500;
  const int MAX_FWD_US = 2000;
  const int MAX_REV_US = 1000;

public:
  // Constructor
  SimpleServo(uint8_t pin, uint8_t channel) {
    _pin = pin;
    _channel = channel;
  }

  void begin() {
    pinMode(_pin, OUTPUT);

    // Setup Timer: 50 Hz, 14-bit Resolution
    // Note: We use 14-bit because 8-bit is too coarse for servos
    ledcSetup(_channel, 50, 14);
    ledcAttachPin(_pin, _channel);

    stop();
  }

  // Input: -100 (Max Reverse) to 100 (Max Forward)
  void write(int speed) {
    speed = constrain(speed, -100, 100);

    int pulse_us = 0;

    if (speed == 0) {
      pulse_us = STOP_US;
    } else if (speed > 0) {
      pulse_us = map(speed, 0, 100, STOP_US, MAX_FWD_US);
    } else {
      pulse_us = map(speed, -100, 0, MAX_REV_US, STOP_US);
    }

    // Convert Microseconds to Duty Cycle (14-bit)
    // Duty = (pulse_us / 20000us) * 16383
    uint32_t duty = (pulse_us * 16383) / 20000;

    ledcWrite(_channel, duty);
  }

  void stop() {
    write(0);
  }
};

#pragma once
#include <Arduino.h>

// ================================================================
// BTS7960 Motor Driver Class for ESP32
// ================================================================
//
// Wiring:
//   RPWM → forward PWM pin   (ESP32 GPIO, LEDC capable)
//   LPWM → reverse PWM pin   (ESP32 GPIO, LEDC capable)
//   R_EN → enable pin        (tie HIGH permanently, or pass -1 to skip)
//   L_EN → enable pin        (tie HIGH permanently, or pass -1 to skip)
//   R_IS → current sense     (analog pin, or pass -1 to skip)
//   L_IS → current sense     (analog pin, or pass -1 to skip)
//
// Usage (minimal — enable pins wired HIGH on board):
//   BTS7960 motor(RPWM_PIN, LPWM_PIN, RPWM_CH, LPWM_CH);
//
// Usage (full — with enable and current sense):
//   BTS7960 motor(RPWM_PIN, LPWM_PIN, RPWM_CH, LPWM_CH,
//                 R_EN_PIN, L_EN_PIN, R_IS_PIN, L_IS_PIN);
//
// Differences from DRV8251:
//   - Two PWM channels (one per direction) instead of one shared channel
//   - PWM is non-inverted: ledcWrite(ch, 255) = full speed
//   - No ledcDetachPin/ledcAttachPin switching — both channels always attached
//   - Brake mode available (both sides HIGH = active braking)
//   - Current sense ratio matches BTS7960 datasheet (~8500:1)
//
// setSpeed() interface is identical to DRV8251 — drop-in replacement.
// ================================================================

class BTS7960 {
private:
  // Pin assignments
  int _rpwm_pin;  // Forward PWM
  int _lpwm_pin;  // Reverse PWM
  int _r_en_pin;  // Right enable  (-1 = not connected, assumed wired HIGH)
  int _l_en_pin;  // Left enable   (-1 = not connected, assumed wired HIGH)
  int _r_is_pin;  // Right current sense (-1 = disabled)
  int _l_is_pin;  // Left current sense  (-1 = disabled)

  // LEDC channels (must be different, and unused by anything else)
  uint8_t _rpwm_ch;
  uint8_t _lpwm_ch;

  int _last_dir = 0;  // 0=coast, 1=forward, -1=reverse

  // Current sensing accumulators
  uint32_t _is_acc = 0;
  uint16_t _is_samples = 0;
  float _motor_current_A = 0.0f;

  // BTS7960 current sense: IS pin sources I_motor / 8500
  // With a 1 kΩ resistor to GND: V_IS = I_motor × (1000 / 8500)
  // Rearranged: I_motor = V_IS × 8500 / 1000
  // Change R_SENSE_OHMS if you use a different resistor.
  static constexpr float IS_RATIO = 8500.0f;
  static constexpr float R_SENSE_OHMS = 1000.0f;
  static constexpr float ADC_REF_V = 3.3f;
  static constexpr float ADC_RESOLUTION = 4095.0f;

public:
  // ── Constructor ──────────────────────────────────────────────
  // Minimal (enable pins tied HIGH on module):
  //   BTS7960 m(RPWM, LPWM, RPWM_CH, LPWM_CH);
  //
  // Full:
  //   BTS7960 m(RPWM, LPWM, RPWM_CH, LPWM_CH, R_EN, L_EN, R_IS, L_IS);
  BTS7960(int rpwm_pin, int lpwm_pin,
          uint8_t rpwm_ch, uint8_t lpwm_ch,
          int r_en_pin = -1, int l_en_pin = -1,
          int r_is_pin = -1, int l_is_pin = -1)
    : _rpwm_pin(rpwm_pin), _lpwm_pin(lpwm_pin),
      _rpwm_ch(rpwm_ch), _lpwm_ch(lpwm_ch),
      _r_en_pin(r_en_pin), _l_en_pin(l_en_pin),
      _r_is_pin(r_is_pin), _l_is_pin(l_is_pin) {}

  // ── begin() — call once in setup() ───────────────────────────
  void begin() {
    if (_r_en_pin >= 0) {
      pinMode(_r_en_pin, OUTPUT);
      digitalWrite(_r_en_pin, HIGH);
    }
    if (_l_en_pin >= 0) {
      pinMode(_l_en_pin, OUTPUT);
      digitalWrite(_l_en_pin, HIGH);
    }
    if (_r_is_pin >= 0) pinMode(_r_is_pin, INPUT);
    if (_l_is_pin >= 0) pinMode(_l_is_pin, INPUT);

    ledcSetup(_rpwm_ch, 20000, 8);
    ledcSetup(_lpwm_ch, 20000, 8);
    ledcAttachPin(_rpwm_pin, _rpwm_ch);
    ledcAttachPin(_lpwm_pin, _lpwm_ch);

    // ↓ REPLACE coast() with this:
    ledcWrite(_rpwm_ch, 0);
    ledcWrite(_lpwm_ch, 0);
    _last_dir = 0;
  }

  // ── setSpeed() — drop-in replacement for DRV8251::setSpeed() ─
  //
  // pwm > 0 : forward  (RPWM = pwm,  LPWM = 0)
  // pwm < 0 : reverse  (RPWM = 0,    LPWM = |pwm|)
  // pwm = 0 : coast    (RPWM = 0,    LPWM = 0)
  //
  // NOTE: Unlike DRV8251, PWM is NOT inverted here.
  //       ledcWrite(ch, 255) = maximum speed.
  void setSpeed(int pwm) {
    pwm = constrain(pwm, -255, 255);

    if (pwm > 0) {
      ledcWrite(_lpwm_ch, 0);  // kill reverse first to avoid shoot-through
      ledcWrite(_rpwm_ch, pwm);
      _last_dir = 1;
    } else if (pwm < 0) {
      ledcWrite(_rpwm_ch, 0);     // kill forward first
      ledcWrite(_lpwm_ch, -pwm);  // abs value
      _last_dir = -1;
    } else {
      // pwm == 0: coast (free-wheel)
      ledcWrite(_rpwm_ch, 0);
      ledcWrite(_lpwm_ch, 0);
      _last_dir = 0;
    }
  }

  // ── stop() — coast (same as setSpeed(0)) ─────────────────────
  // Matches DRV8251 interface. Skips if already stopped.
  void stop() {
    if (_last_dir == 0) return;
    ledcWrite(_rpwm_ch, 0);
    ledcWrite(_lpwm_ch, 0);
    _last_dir = 0;
  }

  // ── brake() — active braking (both sides HIGH) ───────────────
  // BTS7960 supports this natively. Stronger stop than coast.
  // Use for E-STOP or hard stop. Not available on DRV8251.
  void brake() {
    ledcWrite(_rpwm_ch, 255);
    ledcWrite(_lpwm_ch, 255);
    _last_dir = 0;
  }

  // ── updateCurrentSense() — call every control tick ───────────
  // Accumulates 64 samples then computes average current in Amps.
  // Reads the active direction's IS pin automatically.
  void updateCurrentSense() {
    // Pick the IS pin for the active direction
    int is_pin = -1;
    if (_last_dir == 1 && _r_is_pin >= 0) is_pin = _r_is_pin;
    if (_last_dir == -1 && _l_is_pin >= 0) is_pin = _l_is_pin;
    if (is_pin < 0) return;

    _is_acc += analogRead(is_pin);
    _is_samples++;

    if (_is_samples >= 64) {
      float adc_avg = (float)_is_acc / _is_samples;
      float v_is = (adc_avg / ADC_RESOLUTION) * ADC_REF_V;
      _motor_current_A = (v_is / R_SENSE_OHMS) * IS_RATIO;
      _is_acc = 0;
      _is_samples = 0;
    }
  }

  // ── Getters ──────────────────────────────────────────────────
  float getCurrent() const {
    return _motor_current_A;
  }
  int getLastDir() const {
    return _last_dir;
  }
};

#endif