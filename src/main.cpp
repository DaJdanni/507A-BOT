/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       jkeep                                                     */
/*    Created:      1/1/2025, 7:18:58 PM                                      */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include "include\cholib\api.hpp"

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here
vex::brain Brain = vex::brain();
vex::controller Controller = vex::controller();

motor FrontLeft = motor(PORT17, ratio6_1, false); //false 17
motor BottomLeft = motor(PORT18, ratio6_1, true); //true 18
motor TopLeft = motor(PORT20, ratio6_1, true); //true 20

motor FrontRight = motor(PORT15, ratio6_1, true); //true 15
motor BottomRight = motor(PORT16, ratio6_1, false);//false 16
motor TopRight = motor(PORT19, ratio6_1, false);//false 19

motor Intake = motor(PORT10, ratio6_1, false);

motor ladyBrown1 = motor(PORT11, false);
motor ladyBrown2 = motor(PORT12, true);

motor_group LeftSide = motor_group(FrontLeft, BottomLeft, TopLeft);
motor_group RightSide = motor_group(FrontRight, BottomRight, TopRight);
motor_group ladyBrown = motor_group(ladyBrown1, ladyBrown2);

rotation frontTracker(PORT3, true);
rotation backTracker(PORT8); 

optical RingFilter(PORT4);
optical RingFilterBottom(PORT7);

distance GoalDetector(PORT6);

//vex::aivision visionSensor(PORT)

inertial InertialSensor(PORT5);


void pre_auton(void) {
  FrontLeft.setBrake(coast);
  TopLeft.setBrake(coast);
  BottomLeft.setBrake(coast);
  FrontRight.setBrake(coast);
  TopLeft.setBrake(coast);
  BottomRight.setBrake(coast);

  Intake.setBrake(coast);
  ladyBrown1.setBrake(hold);
  ladyBrown2.setBrake(hold);

  LeftSide.resetPosition();
  RightSide.resetPosition();

  ladyBrown1.resetPosition();
  ladyBrown2.resetPosition();
  ladyBrown.resetPosition();

  InertialSensor.calibrate();
  
  launch_task([&] {wait(3, sec); printf("wat\n");});
  printf("hey\n");
  waitUntil(InertialSensor.isCalibrating() == false);
}

void autonomous(void) {
  // std::map<int, std::map<int, std::function<void(bool)>>> autons = {
  //   {0, { // Red Side Autons
  //     {1, goalRushRed},
  //     {2, negSideRed},
  //     {3, doNothing}
  //   }},
  //   {1, { // Blue Side Autons
  //     {1, goalRushBlue},
  //     {2, negSideBlue},
  //     {3, soloBlueAWP}
  //   }},
  //   {2, { // Skills Autons
  //     {1, skills},
  //     {2, testSkills},
  //     {3, doNothing}
  //   }}
  // };

  // std::cout << activeTab << currentAuton << elims << std::endl;
  // autons[activeTab][currentAuton](elims);

  //autons[0][1](false);
  return;
}

double driveCurve(double x, double scale) {
  if (scale != 0) {
    return (scale * pow(1.01, x) - scale);
    //return (pow(2.718, (scale * (std::fabs(x) - 12))) / 1000) * x;
  }
  return x;
}

void usercontrol(void) {
  while (1) {

    Brain.Screen.clearScreen();

    double LeftJoystickPosition = Controller.Axis3.position();
    double RightJoystickPosition = Controller.Axis1.position();

    if (fabs(LeftJoystickPosition) < 5) {
      LeftJoystickPosition = 0;
    }

    if (fabs(RightJoystickPosition) < 5) {
      RightJoystickPosition = 0;
    }

    LeftJoystickPosition *= 0.12;
    RightJoystickPosition = driveCurve(RightJoystickPosition, 18.1212);

    LeftJoystickPosition *= 0.12;

    LeftSide.spin(fwd, LeftJoystickPosition + RightJoystickPosition, volt);
    RightSide.spin(fwd, LeftJoystickPosition - RightJoystickPosition, volt);

    Brain.Screen.printAt(50, 25, "BackLeft Temp: %f", BottomLeft.temperature(temperatureUnits::fahrenheit));
    Brain.Screen.printAt(50, 50, "BackRight Temp: %f", BottomRight.temperature(temperatureUnits::fahrenheit));
    Brain.Screen.printAt(50, 75, "TopLeft Temp: %f", TopLeft.temperature(temperatureUnits::fahrenheit));
    Brain.Screen.printAt(50, 100, "TopRight Temp: %f", TopRight.temperature(temperatureUnits::fahrenheit));
    Brain.Screen.printAt(50, 125, "FrontLeft Temp: %f", FrontLeft.temperature(temperatureUnits::fahrenheit));
    Brain.Screen.printAt(50, 150, "FrontRight Temp: %f", FrontRight.temperature(temperatureUnits::fahrenheit));
    Brain.Screen.printAt(50, 175, "Intake Temp: %f", Intake.temperature(temperatureUnits::fahrenheit));

    //Brain.Screen.drawImageFromFile("Brain_Screen_Logo.png", 0, 0);

    Brain.Screen.render();

    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
