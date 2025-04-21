#pragma once
#include "vex.h"

using namespace vex;

namespace cholib {
    void setSensors(cholib::trackingWheel* horizontalSensor, cholib::trackingWheel* perpendicularSensor);
    void setSensors(cholib::trackingWheel* horizontalSensor, cholib::trackingWheel* perpendicularSensor, int32_t distancePort);
    void wallReset(int wall);
    void setCoordinates(cholib::Coordinates coordinates, bool radians = true);
    cholib::Coordinates getCoordinates(bool radians);
    void updateCoordinates();
    void initOdom();
}