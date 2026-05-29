#ifndef PID_H
#define PID_H
struct PID {
  float kp, ki, kd;
  float integral;
  float prev_error;
  float integral_limit;  // anti-windup clamp
  float output_limit;    // output clamp

  PID(float kp, float ki, float kd, float int_limit, float out_limit)
    : kp(kp), ki(ki), kd(kd),
      integral(0), prev_error(0),
      integral_limit(int_limit), output_limit(out_limit) {}

  float compute(float setpoint, float measured, float dt) {
    float error      = setpoint - measured;
    integral        += error * dt;
    integral         = constrain(integral, -integral_limit, integral_limit);
    float derivative = (error - prev_error) / dt;
    prev_error       = error;
    float output     = kp * error + ki * integral + kd * derivative;
    return constrain(output, -output_limit, output_limit);
  }

  void reset() {
    integral   = 0;
    prev_error = 0;
  }
};

#endif