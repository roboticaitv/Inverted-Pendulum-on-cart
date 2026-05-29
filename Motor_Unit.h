#ifndef MOTOR_UNIT_H
#define MOTOR_UNIT_H

#include "PID.h"

// By using templates, we completely avoid the "does not name a type" compiler errors
// because the compiler resolves the types in the main .ino file.
template <typename MotorType, typename EncoderType>
struct MotorUnit {
  
  MotorType* motor;
  EncoderType* encoder;
  
  PID pos_pid; // Outer loop: Target Position -> Target Velocity
  PID vel_pid; // Inner loop: Target Velocity -> PWM

  int32_t accumulated_ticks = 0;
  bool control_did_read     = false;

  float filtered_velocity   = 0.0f;
  float alpha               = 0.3f; // LPF constant
  float linear_position_m   = 0.0f; // State variable x
  float linear_velocity_ms  = 0.0f; // State variable x_dot

  MotorUnit(MotorType* m, EncoderType* e, PID p_pid, PID v_pid)
    : motor(m), encoder(e), pos_pid(p_pid), vel_pid(v_pid) {}

  void begin() {
    motor->begin();
    encoder->begin();
  }

  void supervisorTick() {
    int16_t delta = (int16_t)encoder->getCount();
    encoder->clear();
    accumulated_ticks += delta;
  }

  void setSpeed(int pwm) {
    motor->setSpeed(pwm);
  }
};

#endif