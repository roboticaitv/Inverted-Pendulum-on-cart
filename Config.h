#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

namespace Config {

    struct Pins {
        // I2C
        static constexpr int I2C_SDA = 13;
        static constexpr int I2C_SCL = 12;

        // SPI
        static constexpr int SPI_MOSI = 10;
        static constexpr int SPI_MISO = 8;
        static constexpr int SPI_SCK  = 11;
        static constexpr int MT6816_CS = 9;
        static constexpr int ICM45686_CS = -1;

        // Motor (BTS7960)
        static constexpr int MOTOR_RPWM = 2;
        static constexpr int MOTOR_LPWM = 3;
        static constexpr int MOTOR_RPWM_CH = 0;
        static constexpr int MOTOR_LPWM_CH = 1;

        // Encoder
        static constexpr int ENCODER_A = 5;
        static constexpr int ENCODER_B = 6;

        // Switches & Probes
        static constexpr int HOME_SWITCH = 15;
        static constexpr int PROBE_ALL_TICKS = -1;
        static constexpr int PROBE_HEAVY_TICK = -1;
    };

    struct Hardware {
        static constexpr int SAMPLE_RATE_HZ = 1000;
        static constexpr int SPI_CLOCK_HZ = 10000000;
        static constexpr int ACCEL_RANGE_G = 16;
        static constexpr int GYRO_RANGE_DPS = 2000;

        // Kinematics
        static constexpr float DT = 0.001f;
        static constexpr float METERS_PER_REV = 0.120f;
        static constexpr float MOTOR_ENCODER_CPR = 4057.0f;
        static constexpr float GEAR_RATIO = 18.8f;
        static constexpr float TICKS_PER_PULLEY_REV = MOTOR_ENCODER_CPR * GEAR_RATIO;
    };

    // Extern variables allow tuning via Serial CLI without rebooting
    struct MotorModel {
        static float A_ff;       // Back-EMF compensation (PWM per m/s)
        static float C_ff;       // Stiction compensation (PWM)
        static int SIGN;         // Direction multiplier
    };

    struct LQR {
        static float Kth;        // Pendulum angle
        static float Kw;         // Pendulum angular vel
        static float Kx;         // Cart position
        static float Kv;         // Cart velocity
        static float PumpK;      // Swing-up energy pump
        static float IntLimit;   // Integrator bounds
        static float IntK;       // Integrator gain
    };

    struct StateMachine {
        static float RailLimitM;      // Soft rail limit (m from center)
        static float MaxCatchVelDegs; // Max angular vel to attempt catch
        static float CatchAngleDeg;   // Angle to trigger catch
        static float DropAngleDeg;    // Angle to drop catch
        static float AlphaVel;        // Angular velocity LPF constant
        
        static constexpr float CENTER_OFFSET_M = 0.25f;
        static constexpr float HOMING_SPEED_MS = 0.1f;
    };

}

#endif
