#include "vex.h"

Bucees::Coordinates Odometry::updatePosition(Bucees::Coordinates previousCoordinates, float currentRightPosition, float currentBackPosition, float currentIMU, bool reversed) {
  // Get the changes:
  float rightDelta = currentRightPosition - this->previousRightPosition;
  float backDelta = currentBackPosition - this->previousBackPosition;
  float deltaIMU = currentIMU - this->previousIMU;
  
  // Calculate the average orientation:
  float averageTheta = this->previousIMU - deltaIMU / 2;

  // Calculate the local position of the robot:
  float localXPosition;
  float localYPosition;

  if (deltaIMU == 0) { // prevent divide by 0
    localXPosition = backDelta;
    localYPosition = rightDelta;
  } else {
    localXPosition = 2 * sinf(deltaIMU / 2) * (backDelta / deltaIMU + this->backTrackerDistance);
    localYPosition = 2 * sinf(deltaIMU / 2) * (rightDelta / deltaIMU + this->rightTrackerDistance);
  }

  Bucees::Coordinates currentCoordinates = previousCoordinates;
  
  // Calculate the global position of the robot:
  currentCoordinates.x += localYPosition * sinf(averageTheta);
  currentCoordinates.y += localYPosition * cosf(averageTheta);
  currentCoordinates.x += localXPosition * -cosf(averageTheta);
  currentCoordinates.y += localXPosition * sinf(averageTheta);
  currentCoordinates.theta = currentIMU;

  // Set the previous values now that we calculated everything:
  this->previousRightPosition = currentRightPosition;
  this->previousBackPosition = currentBackPosition;
  this->previousIMU = currentIMU;

  return currentCoordinates;
}

void Odometry::resetOdometry() {
  this->previousRightPosition = 0;
  this->previousBackPosition = 0;
  this->previousIMU = 0;
}

void Odometry::setOdometry(float rightTrackerDistance, float backTrackerDistance) {
    this->rightTrackerDistance = rightTrackerDistance;
    this->backTrackerDistance = backTrackerDistance;
}