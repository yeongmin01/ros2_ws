#pragma once

#include "motor_driver/md_protocol.hpp"

class MDMath
{
    public:
        static int16_t convertVelocityToRPM(float velocity);
        static float setVelocityReq(float req_vel, float cur_vel, int16_t torque_req);
    
        static constexpr float max_velocity = 8.3; // m/s
        static constexpr uint8_t max_torque = 50;
        static constexpr float wheel_radius = 0.7; // m
        static constexpr float gear_ratio = 22.55;
        static constexpr float max_acceleration = 10; // m/s2
};