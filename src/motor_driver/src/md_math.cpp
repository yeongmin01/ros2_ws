#include "motor_driver/md_math.hpp"

int16_t MDMath::convertVelocityToRPM(int16_t velocity)
{
    int16_t rpm = 0;
    
    rpm = (velocity * 60 * gear_ratio) / (2 * wheel_radius * 3.14);
    
    return rpm;
}

int16_t MDMath::setVelocityReq(int16_t cur_vel, int16_t torque_req)
{
    int16_t velocity_req = cur_vel;
    
    velocity_req += max_acceleration * torque_req/max_torque;

    return velocity_req;
}