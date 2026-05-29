#ifndef DOUBLE_PENDULUM_H
#define DOUBLE_PENDULUM_H

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "Config.h"
#include "MT6816.h"
#include "Motor_Unit.h"
#include "Motor_Control.h"
#include "ICM45686.h"
#include "MT6701_I2C.h"
#include "MT6701.h"
#include "PID.h"
#include "esp_task_wdt.h"

// ---------------------------------------------------------
// SYSTEM STATE
// ---------------------------------------------------------
enum SystemMode {
  MODE_IDLE,
  MODE_ACTIVE,
  MODE_HOMING,
  MODE_RETURN
};

enum PendulumState {
  STATE_SWINGUP,
  STATE_BALANCE
};

struct SystemState {
  volatile bool running = false;
  volatile SystemMode current_mode = MODE_IDLE;
  volatile PendulumState pen_state = STATE_SWINGUP;
  
  // Homing variables
  volatile int homing_stage = 0;
  volatile float target_cart_position_m = 0.0f;
  volatile bool flagA = false;

  // Zeroing/Offsets
  volatile bool request_zero = false;
  volatile float arm1_offset_deg = 0.0f;
  volatile float arm2_offset_deg = 0.0f;

  // Test Variables
  struct {
    volatile bool active = false;
    volatile float target_vel = 0.0f;
    volatile uint64_t end_time_us = 0;
    volatile uint32_t duration_ms = 0;
    volatile bool trigger = false;
    volatile bool notify_complete = false;
  } test;

  // Control Logic Toggles
  volatile bool auto_swingup_enabled = false;
};

extern SystemState sysState;

// ---------------------------------------------------------
// SENSOR & MOTOR OBJECTS
// ---------------------------------------------------------
extern MT6816 angleSensor;
extern BTS7960 motor;
extern EncoderPCNT encM;
extern MotorUnit<BTS7960, EncoderPCNT> unitM;

// --- Task Handles ---
extern TaskHandle_t PhysicsTaskHandle;

// Command Parsing Buffer (used in SerialTask)
extern char cmd[16];
extern float arg1, arg2, arg3, arg4;

// ---------------------------------------------------------
// TELEMETRY PACKET
// ---------------------------------------------------------
struct TelemetryPacket {
  float cart_pos;
  float cart_vel;
  float arm1_angle;
  float pwm_out;
  SystemMode mode;
  int32_t enc_ticks;
  uint32_t timestamp_us;

  // Debugging variables
  float pos_error;
  float target_vel; 

  // Individual LQR Terms
  float term_theta;
  float term_w;
  float term_x;
  float term_v;
  float term_integral;
};

extern QueueHandle_t telemetryQueue;

#endif