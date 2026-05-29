#include "Config.h"

namespace Config {  
    // Default Motor Model Constants
    float MotorModel::A_ff = -425.0f;
    float MotorModel::C_ff = 40.0f;
    int MotorModel::SIGN = 1;

    // Default LQR Gains
    float LQR::Kth = 1500.0f;
    float LQR::Kw = 120.0f;
    float LQR::Kx = 150.0f;
    float LQR::Kv = 200.0f;
    float LQR::PumpK = 30.0f;
    float LQR::IntLimit = 2.0f;
    float LQR::IntK = 20.0f;

    // Default State Machine Parameters
    float StateMachine::RailLimitM = 0.4f;
    float StateMachine::MaxCatchVelDegs = 30.0f;
    float StateMachine::CatchAngleDeg = 15.0f;
    float StateMachine::DropAngleDeg = 25.0f;
    float StateMachine::AlphaVel = 0.20f;
}
