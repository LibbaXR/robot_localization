#include <chrono>

namespace robot_localization
{
    using MyTime = std::chrono::steady_clock::time_point;
    using Duration = std::chrono::duration<double>;
}