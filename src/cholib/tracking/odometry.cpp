#include "vex.h"
#include "cholib/util.hpp"
#include "cholib/chassis/trackingWheel.hpp"
#include "cholib/tracking/odometry.hpp"

cholib::trackingWheel* horizontalSensor = nullptr;
cholib::trackingWheel* perpendicularSensor = nullptr;
vex::distance wall = NULL;

cholib::Coordinates coordinates(0, 0, 0);
cholib::Coordinates previousCoordinates(0, 0, 0);

float previousRightPosition;
float previousBackPosition;
float previousIMU;


void cholib::setSensors(cholib::trackingWheel* horizontalSensor, cholib::trackingWheel* perpendicularSensor) {

}

void cholib::setSensors(cholib::trackingWheel* horizontalSensor, cholib::trackingWheel* perpendicularSensor, int32_t distancePort) {

}

void cholib::wallReset(int wall) {

}

void cholib::setCoordinates(cholib::Coordinates coordinates, bool radians) {

}

void cholib::updateCoordinates() {
    // // Get the changes:
    // float rightDelta = currentRightPosition - this->previousRightPosition;
    // float backDelta = currentBackPosition - this->previousBackPosition;
    // float deltaIMU = currentIMU - this->previousIMU;

    // // Calculate the average orientation:
    // float averageTheta = this->previousIMU + deltaIMU / 2;

    // // Calculate the local position of the robot:
    // float localXPosition;
    // float localYPosition;

    // if (deltaIMU == 0) { // prevent divide by 0
    // localXPosition = backDelta;
    // localYPosition = rightDelta;
    // } else {
    // localXPosition = 2 * sinf(deltaIMU / 2) * ((backDelta / deltaIMU) + this->backTrackerDistance);
    // localYPosition = 2 * sinf(deltaIMU / 2) * ((rightDelta / deltaIMU) + this->rightTrackerDistance);
    // }

    // Bucees::Coordinates currentCoordinates = previousCoordinates;

    // // Calculate the global position of the robot:
    // // currentCoordinates.x += localYPosition * sinf(averageTheta) - (localXPosition * cos(averageTheta));
    // // currentCoordinates.y += localYPosition * cosf(averageTheta) + (localXPosition * sinf(averageTheta));
    // currentCoordinates.x += localYPosition * sinf(averageTheta);
    // currentCoordinates.y += localYPosition * cosf(averageTheta);
    // currentCoordinates.x += localXPosition * -cosf(averageTheta);
    // currentCoordinates.y += localXPosition * sinf(averageTheta);
    // currentCoordinates.theta = currentIMU;

    // // Set the previous values now that we calculated everything:
    // this->previousRightPosition = currentRightPosition;
    // this->previousBackPosition = currentBackPosition;
    // this->previousIMU = currentIMU;

    // return currentCoordinates;
}

void cholib::initOdom() {
    launch_task([&] {
        while (1) {
            
            wait(20, msec);
        }
    });
}