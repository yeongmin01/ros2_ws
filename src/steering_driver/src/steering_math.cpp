#include "steering_driver/steering_math.hpp"
#include <array>
#include <cstdint>
#include <unistd.h>

//std::array<uint16_t, 8> convertAxisSteeringToPosition(std::array<float, 4> axis_steering)
//{
//    std::array<uint16_t, 8> position{};
//
//    for(int i = 0; i< axis_steering.size(); i++)
//    {
//        if(axis_steering[i] < 0)
//        {
//            position[i] = SteeringMath::max_absENC_position - (axis_steering[i] / SteeringMath::degree_per_position);
//            position[i + 1] = SteeringMath::max_absENC_position - (axis_steering[i] / SteeringMath::degree_per_position);
//        }
//        else
//        {
//            position[i] = axis_steering[i] / SteeringMath::degree_per_position;
//            position[i + 1] = axis_steering[i] / SteeringMath::degree_per_position;
//        }
//    }
//
//    return position;
//}

std::array<uint32_t, 8> SteeringMath::convertWheelSteeringToPosition(std::array<float, 8> wheel_steering)
{
    std::array<uint32_t, 8> position{};

    for(size_t i = 0; i< wheel_steering.size(); i++)
    {
        if(wheel_steering[i] < 0)
        {
            position[i] = SteeringMath::max_absENC_position - (wheel_steering[i] / SteeringMath::degree_per_position);
        }
        else
        {
            position[i] = wheel_steering[i] / SteeringMath::degree_per_position;
        }
    }

    return position;
}
// 8개 바퀴 SteerAngle을 4축 axis SteerAngle로 변환
std::array<float, 4> SteeringMath::convertWheelSteerToAxisSteer(std::array<float, 8> wheel_steering_cur)
{
    std::array<float, 4> axis_steering{};

    for(size_t i = 0; i < axis_steering.size(); i++)
    {
        axis_steering[i] = (wheel_steering_cur[i] + wheel_steering_cur[i + 1]) / 2;
    }
    return axis_steering;
}

std::array<float, 4> SteeringMath::setSteeringReq(std::array<float, 8> wheel_steering_cur, std::array<uint16_t, 4> ampere_req)
{
    std::array<float, 4> steering_req = SteeringMath::convertWheelSteerToAxisSteer(wheel_steering_cur);

    for(size_t i = 0; i < steering_req.size(); i++)
    {
        steering_req[i] += ((static_cast<float>(SteeringMath::zero_position_ampere - ampere_req[i]))/static_cast<float>(SteeringMath::max_ampere - SteeringMath::zero_position_ampere) * SteeringMath::max_angular_velocity)/10;
        if(steering_req[i] < SteeringMath::min_axis_steer_angle[i])
        {
            steering_req[i] = SteeringMath::min_axis_steer_angle[i];
        }
        if(steering_req[i] > SteeringMath::max_axis_steer_angle[i])
        {
            steering_req[i] = SteeringMath::max_axis_steer_angle[i];
        }
    }

    return steering_req;
}