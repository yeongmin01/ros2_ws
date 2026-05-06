#include <array>
#include <cstdint>

class SteeringMath
{
    public:
        //static std::array<uint16_t, 8> convertAxisSteeringToPosition(std::array<float, 4> axis_steering);
        static std::array<uint32_t, 8> convertWheelSteeringToPosition(std::array<float, 8> wheel_steering);
        static std::array<float, 4> convertWheelSteerToAxisSteer(std::array<float, 8> wheel_steering_cur);
        static std::array<float, 4> setSteeringReq(std::array<float, 8> steering_cur, std::array<uint16_t, 4> current_req);

        static constexpr float max_angular_velocity = 13.9; //13.9f;
        static constexpr uint16_t max_ampere = 32767;
        static constexpr uint16_t min_ampere = 6558;
        static constexpr uint16_t zero_position_ampere = 19660;
        static constexpr uint16_t max_absENC_position = 16384;
        static constexpr float degree_per_position = 0.022; // Abs Encoder 1당 degree 값

        static constexpr std::array<float, 4> min_axis_steer_angle = {-27.0, -23.0, -23.0, -27.0};
        static constexpr std::array<float, 4> max_axis_steer_angle = {27.0, 23.0, 23.0, 27.0};
};