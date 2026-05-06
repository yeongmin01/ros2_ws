#include "motor_driver/md_math.hpp"
#include <cmath>

int16_t MDMath::convertVelocityToRPM(float velocity)
{
    int16_t rpm = 0;
    
    rpm = (velocity * 60 * gear_ratio) / (2 * wheel_radius * 3.14);
    
    return rpm;
}

float MDMath::setVelocityReq(MDProtocol::MDCmd md_cmd, float cur_vel)
{
    float velocity_req = md_cmd.velocity_req;
    velocity_req = cur_vel;
    // 100ms
    if(md_cmd.gear_d && !md_cmd.gear_n)
    {
        velocity_req += (max_acceleration * static_cast<float>(md_cmd.torque_req)/static_cast<float>(max_torque))/10;
    }
    else if (md_cmd.gear_r && !md_cmd.gear_n)
    {
        velocity_req += (max_acceleration * static_cast<float>(-md_cmd.torque_req)/static_cast<float>(max_torque))/10;
    }
    else
    {
        velocity_req = 0;
    }

    if(velocity_req >= max_velocity)
    {
        velocity_req = max_velocity;
    }

    return velocity_req;
}