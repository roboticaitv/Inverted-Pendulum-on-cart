#include "DOUBLE_PENDULUM.h"

// ================================================================
// GLOBAL INSTANTIATIONS
// ================================================================
SystemState sysState;

MT6816 angleSensor(Config::Pins::MT6816_CS, SPI);
BTS7960 motor(Config::Pins::MOTOR_RPWM, Config::Pins::MOTOR_LPWM, Config::Pins::MOTOR_RPWM_CH, Config::Pins::MOTOR_LPWM_CH);
EncoderPCNT encM(PCNT_UNIT_0, (gpio_num_t)Config::Pins::ENCODER_A, (gpio_num_t)Config::Pins::ENCODER_B);

// The pos_pid and vel_pid arguments are currently kept for the homing routine.
MotorUnit<BTS7960, EncoderPCNT> unitM(
    &motor, 
    &encM,
    PID(2.0f, 0.0f, 0.0f, 1.0f, 2.0f),         // pos_pid
    PID(200.0f, 0.0f, 0.0f, 255.0f, 255.0f)    // vel_pid
);

TaskHandle_t PhysicsTaskHandle = NULL;
QueueHandle_t telemetryQueue;

char cmd[16];
float arg1 = 0, arg2 = 0, arg3 = 0, arg4 = 0;

void IRAM_ATTR isrA() {
  sysState.flagA = true;
}

void setup() {
  Serial.begin(1000000);
  delay(1000);
  Serial.println("\n=== Double Pendulum Sensor Merge ===");

  SPI.begin(Config::Pins::SPI_SCK, Config::Pins::SPI_MISO, Config::Pins::SPI_MOSI, 9);
  Serial.println("✓ Shared SPI Bus Initialized");

  angleSensor.begin();
  Serial.println("✓ MT6816 SPI Initialized");

  motor.begin();
  encM.begin();

  pinMode(Config::Pins::HOME_SWITCH, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(Config::Pins::HOME_SWITCH), isrA, FALLING);
  
  telemetryQueue = xQueueCreate(1, sizeof(TelemetryPacket));
  
  xTaskCreatePinnedToCore(SerialTask, "SerialTask", 4096, NULL, 10, NULL, 0);
  xTaskCreatePinnedToCore(PhysicsTask, "PhysicsTask", 8192, NULL, 24, &PhysicsTaskHandle, 1);
}

void loop() {
  vTaskDelay(portMAX_DELAY);  // suspend forever instead of delete
}

// ================================================================
// CORE 1: HARD REAL-TIME PHYSICS TASK (4000 Hz)
// ================================================================
void PhysicsTask(void *pvParameters) {
  const int64_t interval_us = 250;  // 4000 Hz outer loop
  int64_t next_tick = esp_timer_get_time() + interval_us;

  uint8_t oversample_tick = 0;
  float sum_arm1 = 0;

  // --- ANGULAR VELOCITY FILTER ---
  static float filtered_arm1_vel = 0.0f;
  static float arm1_angular_velocity = 0.0f;  // declared here so catch can zero it

  // --- STATE TRACKING ---
  static int32_t prev_ticks = 0;
  static float prev_arm1_angle = 0.0f;

  for (;;) {
    while (esp_timer_get_time() < next_tick) {
    }
    next_tick += interval_us;

    // 1. FAST READ (4000 Hz)
    sum_arm1 += angleSensor.readAngleDegrees();

    // 2. CONTROL TICK (1000 Hz)
    if (oversample_tick == 0) {
      // --- D. STATE MACHINE & DIRECT PWM OUTPUT ---
      float pwm_output = 0.0f;
      float debug_control = 0.0f;
      
      // NEW: Variables to catch the individual terms
      float debug_term_theta = 0.0f;
      float debug_term_w = 0.0f;
      float debug_term_x = 0.0f;
      float debug_term_v = 0.0f;
      float debug_term_integral = 0.0f;
      
      // --- A. LIMIT SWITCH E-STOP ---
      if (sysState.flagA) {
        unitM.accumulated_ticks = 0;
        prev_ticks = 0;
        unitM.linear_position_m = 0.0f;
        unitM.linear_velocity_ms = 0.0f;
        unitM.filtered_velocity = 0.0f;
        sysState.target_cart_position_m = 0.0f;
        sysState.flagA = false;

        if (sysState.current_mode == MODE_ACTIVE) {
          sysState.current_mode = MODE_IDLE;
          unitM.setSpeed(0);
          Serial.println(">> E-STOP: LIMIT SWITCH HIT!");
        }
      }

      // --- B. SENSOR PROCESSING ---
      float raw_arm1 = sum_arm1 / 4.0f;

      if (sysState.request_zero) {
        sysState.arm1_offset_deg = raw_arm1;
        filtered_arm1_vel = 0.0f;
        arm1_angular_velocity = 0.0f;
        prev_arm1_angle = 0.0f;

        // Reset everything on the thread that actually uses it
        unitM.accumulated_ticks = 0;
        prev_ticks = 0;  // IMPORTANT: Reset prev_ticks too!
        unitM.linear_position_m = 0.0f;
        unitM.linear_velocity_ms = 0.0f;
        unitM.filtered_velocity = 0.0f;

        sysState.request_zero = false;
      }

      float clean_arm1 = raw_arm1 - sysState.arm1_offset_deg;

      while (clean_arm1 > 180.0f) clean_arm1 -= 360.0f;
      while (clean_arm1 <= -180.0f) clean_arm1 += 360.0f;

      // Filtered angular velocity — raw finite-diff at 1 kHz is unusable
      float delta_angle = clean_arm1 - prev_arm1_angle;

      // Wrap-safe derivative
      while (delta_angle > 180.0f) delta_angle -= 360.0f;
      while (delta_angle <= -180.0f) delta_angle += 360.0f;

      float raw_arm1_vel = delta_angle / Config::Hardware::DT;
      prev_arm1_angle = clean_arm1;
      filtered_arm1_vel = Config::StateMachine::AlphaVel * raw_arm1_vel + (1.0f - Config::StateMachine::AlphaVel) * filtered_arm1_vel;
      arm1_angular_velocity = filtered_arm1_vel;

      // --- C. CART KINEMATICS ---
      unitM.supervisorTick();
      int32_t current_ticks = unitM.accumulated_ticks;
      int32_t delta_ticks = current_ticks - prev_ticks;
      prev_ticks = current_ticks;

      unitM.linear_position_m = (current_ticks / Config::Hardware::TICKS_PER_PULLEY_REV) * Config::Hardware::METERS_PER_REV;
      float raw_vel_ms = (delta_ticks / Config::Hardware::TICKS_PER_PULLEY_REV) * Config::Hardware::METERS_PER_REV / Config::Hardware::DT;
      unitM.filtered_velocity = (unitM.alpha * raw_vel_ms) + ((1.0f - unitM.alpha) * unitM.filtered_velocity);
      unitM.linear_velocity_ms = unitM.filtered_velocity;

      // --- D. STATE MACHINE & DIRECT PWM OUTPUT ---
      float debug_pos_error = clean_arm1;

      // INTEGRATOR VARIABLES (To kill the steady-state drift)
      static float x_integral = 0.0f;

      if (sysState.current_mode == MODE_IDLE) {
        pwm_output = 0.0f;  // Let the slew limiter handle deceleration safely
        filtered_arm1_vel = 0.0f;
        arm1_angular_velocity = 0.0f;
        sysState.pen_state = STATE_SWINGUP;
        x_integral = 0.0f;

      } else if (sysState.current_mode == MODE_HOMING) {
        // --- [Your existing homing logic here] ---

      } else if (sysState.current_mode == MODE_RETURN) {
        float x = unitM.linear_position_m;
        float v = unitM.linear_velocity_ms;

        // Stop condition (within 10mm and nearly stopped)
        if (fabsf(x) < 0.010f && fabsf(v) < 0.02f) {
          sysState.current_mode = MODE_ACTIVE;
          pwm_output = 0.0f;
          Serial.println(">> Arrived at Zero. Auto-switching to MODE: ACTIVE");
        } else {
          // Centering PID to push through stiction
          static float return_i_term = 0.0f;
          return_i_term += x * Config::Hardware::DT * 500.0f;
          return_i_term = constrain(return_i_term, -80.0f, 80.0f); // Anti-windup
          
          float p_term = 1200.0f * x;
          float d_term = 150.0f * v;
          debug_control = p_term + d_term + return_i_term;
          debug_control = constrain(debug_control, -200.0f, 200.0f);

          // Stiction for Return Mode (Must push in the direction of intended/actual motion)
          float stiction_sign = 0.0f;
          if (fabsf(v) < 0.02f) {
            if (debug_control > 5.0f) stiction_sign = 1.0f;
            else if (debug_control < -5.0f) stiction_sign = -1.0f;
          } else {
            stiction_sign = (v > 0.0f) ? -1.0f : 1.0f;
          }

          pwm_output = debug_control + (Config::MotorModel::A_ff * v) + (Config::MotorModel::C_ff * stiction_sign);
        }

      } else if (sysState.current_mode == MODE_ACTIVE) {
        if (sysState.test.trigger) {
          sysState.test.end_time_us = esp_timer_get_time() + (int64_t)sysState.test.duration_ms * 1000LL;
          sysState.test.active = true;
          sysState.test.trigger = false;
        }

        if (sysState.test.active) {
          if (esp_timer_get_time() < sysState.test.end_time_us) {
            pwm_output = sysState.test.target_vel;  // raw PWM bypass
          } else {
            sysState.test.active = false;
            sysState.current_mode = MODE_IDLE;
            pwm_output = 0.0f;
            sysState.test.notify_complete = true;
          }
        } else {

          // 0. ANGLE ERROR
          const float TARGET_ANGLE_DEG = 180.0f;
          float angle_error = clean_arm1 - TARGET_ANGLE_DEG;
          while (angle_error > 180.0f) angle_error -= 360.0f;
          while (angle_error <= -180.0f) angle_error += 360.0f;

          float theta = angle_error * DEG_TO_RAD;
          float w = arm1_angular_velocity * DEG_TO_RAD;
          float x = unitM.linear_position_m;
          float v = unitM.linear_velocity_ms;

          // 1. STATE TRANSITIONS
          if (sysState.pen_state == STATE_SWINGUP) {
            if (fabsf(angle_error) < Config::StateMachine::CatchAngleDeg && fabsf(arm1_angular_velocity) < Config::StateMachine::MaxCatchVelDegs) {
              sysState.pen_state = STATE_BALANCE;
              arm1_angular_velocity = 0.0f;
              filtered_arm1_vel = 0.0f;
              x_integral = 0.0f;  // Reset integrator on catch
            }
          } else if (sysState.pen_state == STATE_BALANCE) {
            if (fabsf(angle_error) > Config::StateMachine::DropAngleDeg) {
              sysState.pen_state = STATE_SWINGUP;
              x_integral = 0.0f;  // Reset integrator on drop
            }
          }

          // 2. STATE EXECUTION
          if (sysState.pen_state == STATE_SWINGUP) {
            static enum { SWING_LOW, SWING_HIGH, SWING_UNCERTAIN } swingState = SWING_UNCERTAIN;
            static float currentSwingPeak = 0.0f;
            static float lastPeakAngle = 0.0f;
            
            if (sysState.auto_swingup_enabled) {
              // 1. Zero Crossing Detection
              if (fabsf(clean_arm1) > currentSwingPeak) currentSwingPeak = fabsf(clean_arm1);

              bool crossed = false;
              if (swingState != SWING_HIGH && clean_arm1 > 5.0f) {
                  swingState = SWING_HIGH;
                  crossed = true;
              } else if (swingState != SWING_LOW && clean_arm1 < -5.0f) {
                  swingState = SWING_LOW;
                  crossed = true;
              }

              if (crossed) {
                  lastPeakAngle = currentSwingPeak;
                  currentSwingPeak = 0.0f;
              }

              // 2. Phase-Lead Pump Logic
              float lookAhead = clean_arm1 + (filtered_arm1_vel * 0.15f);
              float pump_scale = 1.0f;
              
              // Smoothly ramp down the pump strength as we approach the top
              if (lastPeakAngle > 120.0f) {
                  pump_scale = (175.0f - lastPeakAngle) / 55.0f;
                  pump_scale = constrain(pump_scale, 0.0f, 1.0f);
              }

              float pump = 0.0f;
              if (lookAhead > 0) {
                  pump = Config::LQR::PumpK * pump_scale; // Push Left -> Stick whips Right
              } else {
                  pump = -Config::LQR::PumpK * pump_scale; // Push Right -> Stick whips Left
              }

              // 3. E-Stop & Rail Safety (RailLimitM is 0.40)
              if (x > 0.30f && pump < 0.0f) pump = 0.0f; 
              if (x < -0.30f && pump > 0.0f) pump = 0.0f;

              // 4. Soft centering
              debug_control = pump + (20.0f * x); 
            } else {
              debug_control = 20.0f * x;
            }

            // Normal Stiction for Swingup
            float stiction_sign = 0.0f;
            if (fabsf(v) < 0.002f) {
              if (debug_control > 5.0f) stiction_sign = 1.0f;
              else if (debug_control < -5.0f) stiction_sign = -1.0f;
            } else {
              stiction_sign = (v > 0.0f) ? 1.0f : -1.0f;
            }

            pwm_output = debug_control + (Config::MotorModel::A_ff * v) + (Config::MotorModel::C_ff * stiction_sign);
            debug_pos_error = angle_error;
          }

          else if (sysState.pen_state == STATE_BALANCE) {
            // THE INTEGRATOR
            x_integral += (x * Config::Hardware::DT);
            x_integral = constrain(x_integral, -Config::LQR::IntLimit, Config::LQR::IntLimit);

            // 1. Calculate individual terms
            debug_term_theta = Config::LQR::Kth * theta;
            debug_term_w = Config::LQR::Kw * w;
            
            // To move the cart towards the center, we must first lean the stick towards the center.
            // To lean the stick towards the center, we must accelerate the cart AWAY from the center.
            // Therefore, Kx, Kv, and IntK must have the OPPOSITE sign of Kth and Kw!
            debug_term_x = -Config::LQR::Kx * x;
            debug_term_v = -Config::LQR::Kv * v;
            debug_term_integral = -Config::LQR::IntK * x_integral;

            // 2. Sum for total control output
            debug_control = debug_term_theta 
                            + debug_term_w 
                            + debug_term_x 
                            + debug_term_v
                            + debug_term_integral;

            // --- INVERTED SOFT RAIL LIMITS ---
            // Since Positive debug_control drives LEFT, and Negative drives RIGHT:
            if (x > Config::StateMachine::RailLimitM && debug_control < 0.0f) {
              debug_control = 0.0f;
            }
            else if (x < -Config::StateMachine::RailLimitM && debug_control > 0.0f) {
              debug_control = 0.0f;
            }

            // LQR-DRIVEN STICTION (Must push in the direction of intended/actual motion)
            float stiction_sign = 0.0f;
            if (fabsf(v) < 0.02f) {
              // If stopped, stiction opposes intended acceleration (debug_control)
              if (debug_control > 5.0f) stiction_sign = 1.0f;
              else if (debug_control < -5.0f) stiction_sign = -1.0f;
            } else {
              // If moving, stiction opposes velocity. 
              // Positive PWM = LEFT. Negative PWM = RIGHT.
              // If v > 0 (RIGHT), we need to push RIGHT (Negative PWM) to overcome friction.
              stiction_sign = (v > 0.0f) ? -1.0f : 1.0f;
            }

            pwm_output = debug_control + Config::MotorModel::A_ff * v + Config::MotorModel::C_ff * stiction_sign;
            debug_pos_error = angle_error;
          }
        }
      }

      // ===============================================
      // GLOBAL HARDWARE APPLICATION (Runs Every Tick)
      // ===============================================

      // 1. Align math with physical hardware wiring
      pwm_output = Config::MotorModel::SIGN * pwm_output;

      // 2. The "Punch" Slew Rate Limiter
      static float prev_pwm_output = 0.0f;
      float pwm_delta = pwm_output - prev_pwm_output;

      if (fabsf(pwm_delta) > 115.0f) {
        if (pwm_delta > 0.0f) {
          pwm_output = prev_pwm_output + 25.0f;
        } else {
          pwm_output = prev_pwm_output - 25.0f;
        }
      }
      prev_pwm_output = pwm_output;

      // 3. Command the Motor Driver
      pwm_output = constrain(pwm_output, -255.0f, 255.0f);
      unitM.setSpeed((int)pwm_output);

      // --- E. TELEMETRY ---
      TelemetryPacket pkt;
      pkt.cart_pos = unitM.linear_position_m;
      pkt.cart_vel = unitM.linear_velocity_ms;
      pkt.arm1_angle = clean_arm1;
      pkt.pwm_out = pwm_output;
      pkt.mode = (sysState.current_mode == MODE_ACTIVE && sysState.pen_state == STATE_BALANCE) ? (SystemMode)99 : sysState.current_mode;
      pkt.enc_ticks = unitM.accumulated_ticks;
      pkt.timestamp_us = (uint32_t)esp_timer_get_time();
      pkt.pos_error = debug_pos_error;
      pkt.target_vel = debug_control;

      // Attach the new individual terms
      pkt.term_theta = debug_term_theta;
      pkt.term_w = debug_term_w;
      pkt.term_x = debug_term_x;
      pkt.term_v = debug_term_v;
      pkt.term_integral = debug_term_integral;

      xQueueOverwrite(telemetryQueue, &pkt);

      sum_arm1 = 0;
    }
    oversample_tick = (oversample_tick + 1) % 4;
  }
}

// ================================================================
// CORE 0: SERIAL COMMAND & TELEMETRY TASK
// ================================================================
void SerialTask(void *pvParameters) {
  TelemetryPacket rxPkt;
  unsigned long last_print_time = 0;
  const unsigned long TELEMETRY_INTERVAL_MS = 50;

  for (;;) {
    // ── 1. PARSE SERIAL COMMANDS ───────────────────────────────
    if (Serial.available() > 0) {
      String input = Serial.readStringUntil('\n');
      input.trim();

      if (input.length() > 0) {
        char cmd[16];
        float arg1 = 0, arg2 = 0, arg3 = 0, arg4 = 0;
        int parsed = sscanf(input.c_str(), "%15s %f %f %f %f", cmd, &arg1, &arg2, &arg3, &arg4);

        String command = String(cmd);
        command.toUpperCase();

        // --- SYSTEM MODES ---
        if (command == "STOP") {
          sysState.current_mode = MODE_IDLE;
          unitM.setSpeed(0);
          Serial.println(">> IDLE");
        } else if (command == "START") {
          sysState.current_mode = MODE_ACTIVE;
          Serial.println(">> ACTIVE");
        } else if (command == "HOME") {
          sysState.current_mode = MODE_HOMING;
          sysState.homing_stage = 0;
          Serial.println(">> HOMING");
        } else if (command == "CENTER") {
          sysState.current_mode = MODE_RETURN;
          Serial.println(">> Stopping and Centering Cart... (Will AUTO-START when done)");
        } else if (command == "ZERO") {
          if (sysState.current_mode == MODE_IDLE) {
            sysState.request_zero = true;
            Serial.println(">> Zero Requested.");
          } else {
            Serial.println(">> ERROR: Must be IDLE to ZERO.");
          }
        }

        // --- CALIBRATION & TUNING ---
        else if (command == "MOTOR" && parsed >= 3) {
          Config::MotorModel::A_ff = arg1;
          Config::MotorModel::C_ff = arg2;
          Serial.printf(">> Motor model: A_ff=%.1f  C_ff=%.1f\n", Config::MotorModel::A_ff, Config::MotorModel::C_ff);
        } else if (command == "FLIP") {
          Config::MotorModel::SIGN = -Config::MotorModel::SIGN;
          Serial.printf(">> Motor sign flipped to %d\n", Config::MotorModel::SIGN);
        } else if (command == "GAINS" && parsed >= 5) {
          Config::LQR::Kth = arg1;
          Config::LQR::Kw = arg2;
          Config::LQR::Kx = arg3;
          Config::LQR::Kv = arg4;
          Serial.printf(">> GAINS: Kth=%.1f  Kw=%.1f  Kx=%.1f  Kv=%.1f\n", Config::LQR::Kth, Config::LQR::Kw, Config::LQR::Kx, Config::LQR::Kv);
        } else if (command == "PUMP" && parsed >= 2) {
          Config::LQR::PumpK = arg1;
          Serial.printf(">> Pump gain: %.1f\n", Config::LQR::PumpK);
        } else if (command == "RAIL" && parsed >= 2) {
          Config::StateMachine::RailLimitM = arg1;
          Serial.printf(">> Rail limit: %.3f m\n", Config::StateMachine::RailLimitM);
        } else if (command == "CATCH" && parsed >= 2) {
          Config::StateMachine::MaxCatchVelDegs = arg1;
          Serial.printf(">> Catch velocity gate: %.1f deg/s\n", Config::StateMachine::MaxCatchVelDegs);
        } else if (command == "SWINGUP" && parsed >= 2) {
          sysState.auto_swingup_enabled = (arg1 > 0.5f);
          Serial.printf(">> Auto Swing-up: %s\n", sysState.auto_swingup_enabled ? "ON" : "OFF");
        }

        // --- HARDWARE TESTING ---
        else if (command == "RAW" && parsed >= 2) {
          if (sysState.current_mode == MODE_IDLE) {
            unitM.setSpeed((int)arg1);
            Serial.printf(">> Raw PWM applied: %d  (send STOP to halt)\n", (int)arg1);
          } else {
            Serial.println(">> ERROR: Send STOP first, then RAW.");
          }
        } else if (command == "VELTEST" && parsed >= 3) {
          if (sysState.current_mode == MODE_IDLE) {
            sysState.test.target_vel = arg1;
            sysState.test.duration_ms = (uint32_t)arg2;
            sysState.test.trigger = true;
            sysState.current_mode = MODE_ACTIVE;
            Serial.printf(">> VELTEST: PWM=%.0f for %.0f ms\n", arg1, arg2);
          } else {
            Serial.println(">> ERROR: Send STOP first.");
          }
        } else if (command == "STATUS") {
          Serial.printf(">> Motor:  A_ff=%.1f  C_ff=%.1f  sign=%d\n", Config::MotorModel::A_ff, Config::MotorModel::C_ff, Config::MotorModel::SIGN);
          Serial.printf(">> Gains:  Kth=%.1f  Kw=%.1f  Kx=%.1f  Kv=%.1f\n", Config::LQR::Kth, Config::LQR::Kw, Config::LQR::Kx, Config::LQR::Kv);
          Serial.printf(">> Pump=%.1f  Rail=%.3fm  Catch=%.1fdeg/s\n", Config::LQR::PumpK, Config::StateMachine::RailLimitM, Config::StateMachine::MaxCatchVelDegs);
        }

        // --- CATCH ALL ---
        else {
          Serial.println(">> Commands: STOP | START | HOME | CENTER | ZERO | STATUS | FLIP");
          Serial.println(">> MOTOR Aff Cff | GAINS Kth Kw Kx Kv | PUMP K | RAIL m | CATCH deg/s");
        }
      }
    }

    // ── 2. TELEMETRY — peek, don't consume ─────────────────────────
    if (sysState.test.notify_complete) {
      sysState.test.notify_complete = false;
      Serial.println(">> TEST COMPLETE.");
    }
    
    if (xQueuePeek(telemetryQueue, &rxPkt, 0) == pdPASS) {
      if (millis() - last_print_time >= TELEMETRY_INTERVAL_MS) {
        last_print_time = millis();
        Serial.printf("%lu, %d, %ld, %.4f, %.4f, %.4f, %.2f, %.0f, %.0f, %.1f, %.1f, %.1f, %.1f, %.1f\n",
                      rxPkt.timestamp_us, rxPkt.mode, rxPkt.enc_ticks,
                      rxPkt.cart_pos, rxPkt.cart_vel, rxPkt.arm1_angle,
                      rxPkt.pos_error, rxPkt.target_vel, rxPkt.pwm_out,
                      rxPkt.term_theta, rxPkt.term_w, rxPkt.term_x, rxPkt.term_v, rxPkt.term_integral);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}