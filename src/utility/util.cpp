#include "vex.h"

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

float findCurvature(Bucees::Coordinates currentPosition, Bucees::Coordinates point, float theta) {
    // which side the point is on the tracking circle
    float side = sgn(std::sin(theta) * (point.x - currentPosition.x) - std::cos(theta) * (point.y - currentPosition.y));

    // calculate center point and radius
    float a = -std::tan(theta);
    float c = std::tan(theta) * currentPosition.x - currentPosition.y;
    float x = std::fabs(a * point.x + point.y + c) / sqrtf((a * a) + 1);
    float d = std::hypotf(point.x - currentPosition.x, point.y - currentPosition.y);

    return side * ((2 * x) / (d * d));
};

double calculateLinearVelocity(int motorRPM, double gearRatio, double wheelDiameter) {
    double wheelCircumference = M_PI * wheelDiameter;
    return motorRPM * gearRatio * wheelCircumference / 60;
};