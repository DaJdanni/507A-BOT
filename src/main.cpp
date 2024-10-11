/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       JDANNI                                                    */
/*    Created:      12/14/2023, 9:35:31 PM                                    */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here
vex::brain Brain = vex::brain();
vex::controller Controller = vex::controller();

motor FrontLeft(PORT1, gearSetting::ratio6_1, true);
motor TopLeft(PORT2, gearSetting::ratio6_1, false);
motor BackLeft(PORT3, gearSetting::ratio6_1, true);

motor FrontRight(PORT6, gearSetting::ratio6_1, false);
motor TopRight(PORT7, gearSetting::ratio6_1, true);
motor BackRight(PORT8, gearSetting::ratio6_1, false);

motor Intake(PORT9);

motor Lift(PORT18, gearSetting::ratio36_1, true);

inertial InertialSensor(PORT12);

motor_group LeftSide(FrontLeft, TopLeft, BackLeft);
motor_group RightSide(FrontRight, TopRight, BackRight);
drivetrain Drivetrain(LeftSide, RightSide);

rotation backTrackerSensor(PORT19);
rotation rightTrackerSensor(PORT20, false);

//optical RingFilter(PORT);

Bucees::PIDSettings LINEAR_SETTINGS {
  // FEEDFORWARD GAIN
  0, 
  // ACCELERATION GAIN 
  0,
  // PROPORTIONAL GAIN
  1, 
  // INTEGRAL GAIN
  0.0, 
  // DERIVATIVE GAIN
  1.85, 
  // EXIT ERROR
  2.5, 
  // INTEGRAL THRESHOLD
  5, 
  // TIMEOUT TIME
  0, 
  // MAXIMUM VOLTAGES
  12
};

Bucees::PIDSettings ANGULAR_90_SETTINGS {
  // FEEDFORWARD GAIN
  0, 
  // ACCELERATION GAIN
  0.0,
  // PROPORTIONAL GAIN
  0.4175,
  // INTEGRAL GAIN
  0.11,
  // DERIVATIVE GAIN
  1.58,
  // EXIT ERROR
  1, 
  // INTEGRAL THRESHOLD
  15, 
  // TIMEOUT TIME
  0, 
  // MAXIMUM VOLTAGES
  12
};

Bucees::PIDSettings ANTIDRIFT_1TILE {
  // FEEDFORWARD GAIN
  0, 
  // ACCELERATION GAIN
  0.0,
  // PROPORTIONAL GAIN
  0.1,
  // INTEGRAL GAIN
  0.0,
  // DERIVATIVE GAIN
  0.0,
  // EXIT ERROR
  1, 
  // INTEGRAL THRESHOLD
  15, 
  // TIMEOUT TIME
  0, 
  // MAXIMUM VOLTAGES
  12
};

Bucees::PIDSettings LIFT_SETTINGS {
  // FEEDFORWARD GAIN
  0, 
  // ACCELERATION GAIN
  0.0,
  // PROPORTIONAL GAIN
  4,
  // INTEGRAL GAIN
  0.0,
  // DERIVATIVE GAIN
  1.2,
  // EXIT ERROR
  1, 
  // INTEGRAL THRESHOLD
  15, 
  // TIMEOUT TIME
  0, 
  // MAXIMUM VOLTAGES
  12
};

Bucees::FAPIDController Linear(LINEAR_SETTINGS);

Bucees::FAPIDController Angular(ANGULAR_90_SETTINGS);

Bucees::FAPIDController AntiDrift(ANTIDRIFT_1TILE);

Bucees::FAPIDController LiftController(LIFT_SETTINGS);

// TRACKING WHEEL EXAMPLE:
/*Bucees::TrackingWheel RightTracker(
  // INSERT RIGHT TRACKER MOTOR GROUP IF NO TRACKING WHEEL, INSERT ROTATION SENSOR PORT IF TRACKING WHEEL:
  PORT16,

  // UNCOMMENT AND PUT REVERSED = TRUE/FALSE DEPENDING ON SETUP
  false,

  // INSERT WHEEL DIAMETER FOR THE TRACKING WHEEL [OR WHEEL DIAMETER FOR THE MOTOR GROUP]:
  2.9,

  // INSERT THE OFFSET FROM THE TRACKING CENTER:
  0,

  // INSERT THE GEAR RATIO OF THE TRACKING WHEEL [OR GEAR RATIO OF THE MOTOR GROUP]:
  1.f/1.f
);*/

// RIGHT TRACKER DRIVETRAIN MOTORS EXAMPLE:
Bucees::TrackingWheel RightTracker(
  &RightSide,

  3.25,

  5.75,

  36.f/48.f
);

// BACK TRACKER EXAMPLE:
Bucees::TrackingWheel BackTracker(
  PORT11,

  false,

  -2.75,
  
  -1.375,

  1.f/1.f
);


Bucees::Robot Robot(
  // INSERT THE WHEEL DIAMETER OF YOUR DRIVETRAIN WHEELS:
  Bucees::Omniwheel::NEW_325,

  // INSERT THE GEAR RATIO OF YOUR DRIVETRAIN USING: X.f/X.f (REPLACE THE X WITH YOUR DRIVETRAIN GEAR RATIO):
  36.f/48.f,
  
  // INSERT THE TRACK WIDTH OF YOUR DRIVETRAIN IN INCHES:
  11.412,

  // INSERT THE LEFT MOTOR GROUP OF YOUR DRIVETRAIN:
  &LeftSide,

  // INSERT THE RIGHT MOTOR GROUP OF YOUR DRIVETRAIN:
  &RightSide,

  // INSERT THE PORT OF YOUR INERTIAL SENSOR, IF YOU DO NOT USE AN INERTIAL SENSOR, REPLACE WITH "nullptr"
  PORT12,

  // INSERT THE RIGHT TRACKING WHEEL OBJECT:
  &RightTracker,

  // INSERT THE BACK TRACKING WHEEL OBJECT [REPLACE WITH NULLPTR IF NO BACK TRACKER]:
  &BackTracker,

  // INSERT THE LINEAR PID SETTINGS OBJECT:
  &Linear,

  // INSERT THE ANGULAR PID SETTINGS OBJECT:
  &Angular,

  // INSERT THE ANTI DRIFT PID SETTINGS OBJECT:
  &AntiDrift
);


// for macros:
template <class F>
vex::task launch_task(F&& function) {
  return vex::task([](void* parameters) {
    std::unique_ptr<std::function<void()>> ptr{static_cast<std::function<void()>*>(parameters)};
    (*ptr)();
    return 0;
  }, new std::function<void()>(std::forward<F>(function)));
}

bool mogoDB;
bool toggleMogo;
bool intakeLiftDB;
bool toggleIntakeLift;
bool liftClampDB;
bool toggleLiftClamp;
bool liftRatchetDB;
bool toggleLiftRatchet;
bool extenderDB;
bool toggleExtender;

bool inLiftMacro = false;
bool cancelMacro = false;

void toggleMogoF() {

  pneumatics Mogo(Brain.ThreeWirePort.G);

  if (mogoDB == true) {return;}
  mogoDB = true;

  toggleMogo = !toggleMogo;

  if (toggleMogo == true) {
    Mogo.open();
  } else {
    Mogo.close();
  }

  wait(50, msec);
  mogoDB = false;
}

void toggleIntakeLiftF() {

  pneumatics IntakeLift(Brain.ThreeWirePort.F);

  if (intakeLiftDB == true) return;
  intakeLiftDB = true;

  toggleIntakeLift = !toggleIntakeLift;

  if (toggleIntakeLift == true) {
    IntakeLift.open();
  } else {
    IntakeLift.close();
  }

  wait(50, msec);
  intakeLiftDB = false;
}

const float liftDesiredPosition = 272; // degrees

void raiseLift() {

  pneumatics LiftClamp(Brain.ThreeWirePort.C);

  if (inLiftMacro == true) return;
  cancelMacro = false;
  inLiftMacro = true;

  Lift.spin(fwd, 12, volt);
  waitUntil(liftDesiredPosition - Lift.position(degrees) < 250);
  LiftClamp.open();
  waitUntil(liftDesiredPosition - Lift.position(degrees) < 3 || cancelMacro == true);

  if (cancelMacro == true) {
    std::cout << "Cancelled macro!" << std::endl;
  }

  Lift.stop();

  inLiftMacro = false;
}

void toggleLiftClampF() {
  raiseLift();
}

void toggleLiftRatchetF() {
  pneumatics LiftRatchet(Brain.ThreeWirePort.D);

  if (liftRatchetDB == true) {return;}
  liftRatchetDB = true;

  toggleLiftRatchet = !toggleLiftRatchet;

  if (toggleLiftRatchet == true) {
    LiftRatchet.open();
  } else {
    LiftRatchet.close();
  }

  wait(50, msec);
  liftRatchetDB = false;
}

void toggleExtenderF() {
  pneumatics Extender(Brain.ThreeWirePort.E);

  if (extenderDB == true) {return;}
  extenderDB = true;

  toggleExtender = !toggleExtender;

  if (toggleExtender == true) {
    printf("EXTENDER OPEN! \n");
    Extender.open();
  } else {
    printf("EXTENDER CLOSE! \n");
    Extender.close();
  }

  wait(50, msec);
  extenderDB = false;
}


void pre_auton(void) {

  FrontLeft.setBrake(coast);
  TopLeft.setBrake(coast);
  BackLeft.setBrake(coast);
  FrontRight.setBrake(coast);
  TopLeft.setBrake(coast);
  BackRight.setBrake(coast);

  Intake.setBrake(coast);
  Lift.setBrake(hold);
  
  LeftSide.resetPosition();
  RightSide.resetPosition();
  Lift.resetPosition();

  Intake.setVelocity(100, pct);

  InertialSensor.calibrate();
  waitUntil(InertialSensor.isCalibrating() == false);

  Robot.initOdom();

  //Robot.setRobotCoordinates({0, 0, 180});
}

void printCoordinates() {
    Bucees::Coordinates currentCoordinates = Robot.getRobotCoordinates(false);
    printf("ATM right now: %f, %f, %f \n", currentCoordinates.x, currentCoordinates.y, currentCoordinates.theta);
}

void AutonBlueWP() {
  printf("blue side now");
   pneumatics Mogo(Brain.ThreeWirePort.G);
  pneumatics IntakeLift(Brain.ThreeWirePort.F);

  // pid is tuned badly so 9 is default
  Linear.setMaxVoltages(12);
  Angular.setMaxVoltages(9);

  // go for the mogo:
  Robot.DriveFor(-33, false, 850);
  Robot.TurnFor(-36, 650);

  // speed down  to grab the mogo
  Linear.setMaxVoltages(5.5);
  Robot.DriveFor(-11.5, false, 850);
  
  // Put ring in mogo:
  Mogo.open();
  wait(0.5, sec);
  Intake.spin(reverse, 12, volt);
  wait(1, sec); 
  
  // Get second ring:
  Linear.setMaxVoltages(8.5);
  Robot.TurnFor(10, 500);
  Robot.DriveFor(12.5, false, 800);

  wait(2, seconds);

  Intake.stop(coast);

  printf("second ring \n");

  // Drop first mogo after scoring second ring
  Mogo.close();

  // Face and get other mogo:
  Robot.DriveFor(9.5, false, 400);
  Robot.TurnFor(69.5, 800);

  Linear.setMaxVoltages(7.5);

  Robot.DriveFor(-30.5, false, 800);

  Mogo.open();

  // Start pathing to the next ring:

  wait(0.2, seconds);

  Robot.TurnFor(0, 750);

  printCoordinates();

  IntakeLift.open();

  Linear.setMaxVoltages(12);

  Robot.DriveFor(22.5, false, 650);

  Robot.TurnFor(-90, 600);

  Linear.setMaxVoltages(4.5);

  Robot.DriveFor(12.5, false, 500);

  Intake.spin(reverse, 12, volt);

  IntakeLift.close();

  wait(0.5, seconds);

  Linear.setMaxVoltages(9);
  Angular.setMaxVoltages(9);

  Robot.DriveFor(-7.5);

  Robot.TurnFor(-140, 500);

  // wait(3.5, sec);

  // Linear.setMaxVoltages(8.5);

  // Robot.DriveFor(60, false, 1500, false);

  // wait(0.2, seconds);

  // Mogo.close();

 // Robot.TurnFor(-32.5, 700, true);

  //wait(0.25, sec);

  //Mogo.close();
  
  printCoordinates();



  // 3 -34 6.4

  // 16 -13 72
}

void AutonRedWP() {
 pneumatics Mogo(Brain.ThreeWirePort.G);
  pneumatics IntakeLift(Brain.ThreeWirePort.F);

  // pid is tuned badly so 9 is default
  Linear.setMaxVoltages(12);
  Angular.setMaxVoltages(9);

  // go for the mogo:
  Robot.DriveFor(-33, false, 850);
  Robot.TurnFor(34.5, 650);

  // speed down  to grab the mogo
  Linear.setMaxVoltages(5.5);
  Robot.DriveFor(-10.5, false, 800);
  
  // Put ring in mogo:
  Mogo.open();
  wait(0.5, sec);
  Intake.spin(reverse, 12, volt);
  wait(1, sec);
  
  // Get second ring:
  Linear.setMaxVoltages(8.5);
  Robot.TurnFor(-10, 500);
  Robot.DriveFor(12.5, false, 800);

  wait(2, seconds);

  Intake.stop(coast);

  printf("second ring \n");

  // Drop first mogo after scoring second ring
  Mogo.close();

  // Face and get other mogo:
  Robot.DriveFor(9.5, false, 400);
  Robot.TurnFor(-69.5, 800);

  Linear.setMaxVoltages(7.5);

  Robot.DriveFor(-30.5, false, 800);

  Mogo.open();

  // Start pathing to the next ring:

  wait(0.2, seconds);

  Robot.TurnFor(0, 750);

  printCoordinates();

  IntakeLift.open();

  Linear.setMaxVoltages(12);

  Robot.DriveFor(22.5, false, 650);

  Robot.TurnFor(90, 600);

  Linear.setMaxVoltages(4.5);

  Robot.DriveFor(12.5, false, 500);

  IntakeLift.close();

  wait(0.5, seconds);

  Intake.spin(reverse, 12, volt);

  Linear.setMaxVoltages(9);
  Angular.setMaxVoltages(9);

  Robot.DriveFor(-7.5);

  Robot.TurnFor(140, 500);

  // wait(100, sec);

  // Linear.setMaxVoltages(6);

  // Robot.DriveFor(60, false, 1500, false);

 // Robot.TurnFor(-32.5, 700, true);

  //wait(0.25, sec);

  //Mogo.close();
  
  printCoordinates();



  // 3 -34 6.4

  // 16 -13 72

}

void HalfWP() {
  pneumatics Mogo(Brain.ThreeWirePort.G);
  pneumatics IntakeLift(Brain.ThreeWirePort.F);

  Robot.DriveFor(-16, false, 600);

  Robot.TurnFor(-40, 500, false);

  Linear.setMaxVoltages(6);

  Robot.DriveFor(-12, false, 500);

  Mogo.open(); // grab mogo

  wait(0.2, seconds);

  Intake.spin(reverse, 12, volt);

  wait(1.5, seconds);

  Robot.TurnFor(70, 500);

  Intake.stop();

  Robot.DriveFor(16, false, 500);

}

void macroDisrupter() {
  cancelMacro = true;
}

void liftClampChecker() {

  pneumatics LiftClamp(Brain.ThreeWirePort.C);

  while (1) {

    if (Lift.position(degrees) < 0) {
      LiftClamp.close();
    } else if (Lift.position(degrees) < 50) {
      
    }

    wait(10, msec);
  }
}

// scooper arm thing:
// ring clamp thing:
// lift ratchet thing

void openSeasme() {
  Lift.spin(fwd, 12, volt);
  Lift.stop();
}

void autonomous(void) {
  printf("blue side \n");

  pneumatics Mogo(Brain.ThreeWirePort.G);
  pneumatics IntakeLift(Brain.ThreeWirePort.F);

  Lift.spin(reverse, 3, volt); // makes sure lift doesnt pop out
  
  Robot.DriveFor(-33, true, 0);
  Robot.TurnFor(32, 500);

  Robot.DriveFor(-10.5, false, 600, true);

  wait(0.375, seconds);

  Mogo.open();

  Robot.waitChassis();

  wait(0.25, seconds);

  Intake.spin(reverse, 12, volt);

  Robot.DriveFor(5, true, 0);

  Robot.TurnFor(-15, 500);
  Robot.DriveFor(12.5, false, 800);

}

void usercontrol(void) {

  // pneumatics:
  Controller.ButtonB.pressed(toggleMogoF);
  Controller.ButtonRight.pressed(toggleExtenderF);
  Controller.ButtonDown.pressed(toggleLiftClampF);
  Controller.ButtonUp.pressed(toggleLiftRatchetF);
  Controller.ButtonA.pressed(toggleIntakeLiftF); // intake lift
  Controller.ButtonL1.pressed(macroDisrupter);

  launch_task([&] {
    liftClampChecker();
  });

  launch_task([&] {

  });
  
  while (1) {

    double LeftJoystickPosition = Controller.Axis3.position();
    double RightJoystickPosition = Controller.Axis1.position();

    if (fabs(LeftJoystickPosition) < 5) {
      LeftJoystickPosition = 0;
    }

    if (fabs(RightJoystickPosition) < 5) {
      RightJoystickPosition = 0;
    }

    LeftJoystickPosition *= 0.12;
    RightJoystickPosition *= 0.12;

    LeftSide.spin(fwd, LeftJoystickPosition + RightJoystickPosition, volt);
    RightSide.spin(fwd, LeftJoystickPosition - RightJoystickPosition, volt);

    if (Controller.ButtonL2.pressing()) {
      Intake.spin(forward, 12, volt);
    } else if (Controller.ButtonR2.pressing()) {
      Intake.spin(reverse, 12, volt);
    } else {
      Intake.stop();
    }

    if (Controller.ButtonR1.pressing()) {
      Lift.spin(forward, 12, volt);
    } else if (Controller.ButtonL1.pressing()) {
      Lift.spin(reverse, 12, volt);
    } else if (inLiftMacro == false) {
      Lift.stop();
    }

    Brain.Screen.drawImageFromFile("Brain_Screen_Logo.png", 0, 0);

    Bucees::Coordinates currentCoordinates = Robot.getRobotCoordinates(false);
    printf("mp: %f\n", Lift.position(degrees));
    //printf("current: %f, %f, %f \n", currentCoordinates.x, currentCoordinates.y, currentCoordinates.theta);

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
