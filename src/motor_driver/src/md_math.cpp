#include "motor_driver/md_math.hpp"

int16_t MDMath::convertVelocityToRPM(float velocity)
{
    int16_t rpm = 0;
    
    rpm = (velocity * 60 * gear_ratio) / (2 * wheel_radius * 3.14);
    
    return rpm;
}

float MDMath::setVelocityReq(float cur_vel, int16_t torque_req)
{
    float velocity_req = cur_vel;

    // 100ms
    velocity_req += (max_acceleration/10) * torque_req/max_torque;

    if(velocity_req >= max_velocity)
    {
        velocity_req = max_velocity;
    }

    return velocity_req;
}