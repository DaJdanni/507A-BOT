#include "coordinates.hpp"

template <class F>
vex::task launch_task(F&& function) {
  return vex::task([](void* parameters) {
    std::unique_ptr<std::function<void()>> ptr{static_cast<std::function<void()>*>(parameters)};
    (*ptr)();
    return 0;
  }, new std::function<void()>(std::forward<F>(function)));
}

int sgn(float num) {
    if (num >= 0) {
        return 1;
    } else {
        return -1;
    }
}

float to_wheel_travel(float input, float diameter, float gearRatio) {
    return (input * (gearRatio / 360.0 * M_PI * diameter));
}

float slew(float output, float prevOutput, float maxChange) {
    float change = output - prevOutput;
    if (maxChange == 0) return output;
    if (change > maxChange) change = maxChange;
    else if (change < -maxChange) change = -maxChange;
    return output + change;
}

float to_rad(float angle_deg) {
    return (angle_deg * (M_PI / 180));
}

float to_deg(float angle_rad) {
    return (angle_rad * (180 / M_PI));
}
