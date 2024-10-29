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

optical RingFilter(PORT21);

Bucees::PIDSettings L_Settings {
  // FEEDFORWARD GAIN
  0, 
  // ACCELERATION GAIN 
  5,
  // PROPORTIONAL GAIN
  1.1275, 
  // INTEGRAL GAIN
  0.0, 
  // DERIVATIVE GAIN
  5.2,  //4.5, 4.7, 5.2, 
  // EXIT ERROR
  1, 
  // INTEGRAL THRESHOLD
  5, 
  // TIMEOUT TIME
  0, 
  // MAXIMUM VOLTAGES
  12  
};

Bucees::PIDSettings LMogo_Settings { // for 6 rings
  // FEEDFORWARD GAIN
  0, 
  // ACCELERATION GAIN 
  5,
  // PROPORTIONAL GAIN
  1.1275, 
  // INTEGRAL GAIN
  0.0, 
  // DERIVATIVE GAIN
  5.2,  //4.5, 4.7, 5.2, 
  // EXIT ERROR
  1, 
  // INTEGRAL THRESHOLD
  5, 
  // TIMEOUT TIME
  0, 
  // MAXIMUM VOLTAGES
  12  
};

Bucees::PIDSettings A0_Settings { // not done [0-60]
// right turn: good, left turn: bad 
  // FEEDFORWARD GAIN
  0, 
  // ACCELERATION GAIN
  0.0,
  // PROPORTIONAL GAIN
  0.32,
  // INTEGRAL GAIN
  0.0137,
  // DERIVATIVE GAIN
  2.1,
  // EXIT ERROR
  1, 
  // INTEGRAL THRESHOLD
  15, 
  // TIMEOUT TIME
  0, 
  // MAXIMUM VOLTAGES
  12
};

Bucees::PIDSettings A60_Settings { // DONE [60-120]
  // FEEDFORWARD GAIN
  0, 
  // ACCELERATION GAIN
  0.0,
  // PROPORTIONAL GAIN
  0.32,
  // INTEGRAL GAIN
  0.00825,
  // DERIVATIVE GAIN
  2.1,
  // EXIT ERROR
  1, 
  // INTEGRAL THRESHOLD
  15, 
  // TIMEOUT TIME
  0, 
  // MAXIMUM VOLTAGES
  12
};

Bucees::PIDSettings A120_Settings { // DONE [120-180]
  // FEEDFORWARD GAIN
  0, 
  // ACCELERATION GAIN 
  0,
  // PROPORTIONAL GAIN
  0.29, 
  // INTEGRAL GAIN
  0.0088125, // 0.00881 for 180,
  // DERIVATIVE GAIN
  2.1, 
  // EXIT ERROR
  1, 
  // INTEGRAL THRESHOLD
  17.5, 
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
  0.125,
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

Bucees::FAPIDController Linear(L_Settings);

Bucees::FAPIDController Angular(A60_Settings);

Bucees::FAPIDController AntiDrift(A0_Settings);

//Bucees::FAPIDController LiftController(LIFT_SETTINGS);

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

  7, // 5.75

  36.f/48.f
);

// BACK TRACKER EXAMPLE:
Bucees::TrackingWheel BackTracker(
  PORT11,

  false,

  -2.75,
  
  -2, // -1.375

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
  nullptr,

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
bool startMatchDB;
bool toggleStartMatch;

bool inLiftMacro = false;
bool cancelMacro = false;

void halfOpen(double desiredPosition = 125) {
  Lift.stop();
  Lift.spin(fwd, 12, volt);
  waitUntil(Lift.position(degrees) > desiredPosition);
  Lift.stop();
}

void resetLift() {
  Lift.stop();
  Lift.spin(reverse, 4.5, volt);
  waitUntil(Lift.position(degrees) < 0);
  Lift.stop();
}


void toggleStartMatchF() {

  pneumatics LiftClamp(Brain.ThreeWirePort.C);

  if (startMatchDB == true) {return;}
  startMatchDB = true;
  toggleStartMatch = true;

  printf("hi chode strart \n");

  halfOpen(60);
  LiftClamp.open();
  wait(250, msec);
  LiftClamp.close();
  resetLift();
  // Lift.spin(reverse, 6, volt);
  // wait(0.2, seconds);
  // Lift.stop();

  wait(50, msec);
  toggleStartMatch = false;
  startMatchDB = false;
}


void toggleMogoF() {

  pneumatics Mogo(Brain.ThreeWirePort.G);

  printf("PRessed mOgo\n");

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
    Extender.open();
  } else {
    Extender.close();
  }

  wait(50, msec);
  extenderDB = false;
}


// ALWAYS SET TO -1 DURING A COMPETITION MATCH
int autonSelected = -1;
std::string autonLabel = "Auton Selected: NONE";

class autonButton {
  private:
  int xPos, yPos, width, height, autonID;
  bool state;
  vex::color offColor;
  vex::color onColor;
  const char* label;

  public:

  autonButton(int x, int y, int width, int height, int autonId, bool state, vex::color offColor, vex::color onColor, const char* label) :
  xPos(x), yPos(y), width(width), height(height), autonID(autonId), state(state), offColor(offColor), onColor(onColor), label(label) {}

  void render() {
    vex::color renderColor = state ? onColor : offColor;
    Brain.Screen.drawRectangle(xPos, yPos, width, height, renderColor);
    Brain.Screen.printAt(xPos, yPos, label);
  }

  void detectClick() {
    if(Brain.Screen.pressing() && Brain.Screen.xPosition() >= xPos && Brain.Screen.xPosition() <= xPos + width &&
    Brain.Screen.yPosition() >= yPos && Brain.Screen.yPosition() <= yPos + width) { // clicked
      state = true;
      autonSelected = autonID;
      autonLabel = "Auton Selected: " + std::string(label);
    } else {
      state = false;
    }

  }
};

autonButton buttons[] = {
  autonButton(30, 30, 60, 60, 0, false, 0xE00000, 0x1f1c1c, "R_SAWP"),
  autonButton(150, 30, 60, 60, 1, false, 0x0000E0, 0x1f1c1c, "B_SAWP"),
  autonButton(270, 30, 60, 60, 2, false, 0x32a852, 0x1f1c1c, "R_OPP"),
  autonButton(390, 30, 60, 60, 3, false, 0xcf6c21, 0x1f1c1c, "B_OPP"),
  autonButton(30, 150, 60, 60, 4, false, 0xd1c821, 0x1f1c1c, "4-"),
  autonButton(150, 150, 60, 60, 5, false, 0x351ee3, 0x1f1c1c, "5-"),
  autonButton(270, 150, 60, 60, 6, false, 0xa11ee3, 0x1f1c1c, "testGS"),
  autonButton(390, 150, 60, 60, 7, false, 0xe3143e, 0x1f1c1c, "Skills")
};

void autonSelector() {
  while (1) {

    Brain.Screen.clearScreen();

    if (!Competition.isEnabled()) {
      for (int i = 0; i < 8; i++) {
        buttons[i].render();
        buttons[i].detectClick();
      }

      Brain.Screen.printAt(90, 125, autonLabel.c_str());
    }

    Brain.Screen.render();

    wait(20, msec);
  }
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

  launch_task([&] {
    autonSelector();
  });

  RingFilter.setLight(ledState::on);
  RingFilter.setLightPower(100);

  InertialSensor.calibrate();
  waitUntil(InertialSensor.isCalibrating() == false);

  Robot.initOdom();

  //Robot.setRobotCoordinates({0, 0, 180});
}

void activateMotionChaining(bool reversed = false, float minSpeed = 5) {
  L_Settings.exitError = 8;
  LMogo_Settings.exitError = 8;
  A0_Settings.exitError = 20;
  A60_Settings.exitError = 20;
  A120_Settings.exitError = 20
  ;
  Linear.setExitError(8);
  Angular.setExitError(20);
  Robot.defaultMinSpeed = minSpeed;
  Robot.reversedChaining = reversed;
}

void deactivateMotionChaining(bool reversed = false) {
  L_Settings.exitError = 1;
  LMogo_Settings.exitError = 1;
  A0_Settings.exitError = 1;
  A60_Settings.exitError = 1;
  A120_Settings.exitError = 1;
  Linear.setExitError(1);
  Angular.setExitError(1);
  Robot.defaultMinSpeed = 0;
  LeftSide.stop(hold);
  RightSide.stop(hold);
  Robot.reversedChaining = reversed;
}

void printCoordinates() {
    Bucees::Coordinates currentCoordinates = Robot.getRobotCoordinates(false);
    printf("ATM right now: %f, %f, %f \n", currentCoordinates.x, currentCoordinates.y, currentCoordinates.theta);
}

void IntakeToLift() {
  Intake.spin(reverse, 6.5, volt);

  waitUntil(RingFilter.isNearObject() == true);

  std::cout << "Found object" << std::endl;

  Intake.spin(fwd, 9, volt);

  wait(0.5, sec);
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

void openSeasme() {
  Lift.spin(fwd, 12, volt);
  Lift.stop();
}

void RedSoloAWP() {
  
  pneumatics Mogo(Brain.ThreeWirePort.G);
  pneumatics IntakeLift(Brain.ThreeWirePort.F);

  Lift.spin(reverse, 7, volt); // makes sure lift doesnt pop out

  //-- GOAL RUSH SECTION --//
  
  Robot.DriveFor(-37.5, L_Settings, true, 0);
  Robot.TurnFor(32, A0_Settings, 550);

  Robot.DriveFor(-11, L_Settings, false, 800, true);

  wait(0.5, seconds);

  Mogo.open();

  Robot.waitChassis();

  Linear.setMaxVoltages(12);

  //wait(0.25, seconds);

  Intake.spin(reverse, 12, volt);

  //-- SECOND RING SCORING SECTION --//

  Robot.DriveFor(7.5, L_Settings, true, 300);

  Robot.TurnFor(-25, A0_Settings, 500);
  Robot.DriveFor(11, L_Settings, false, 600);
  Linear.setMaxVoltages(6.5);
  Robot.DriveFor(-7.5, L_Settings, true, 500);

  wait(0.6, seconds);

  Intake.stop(coast);

  Mogo.close();

  //-- SECOND GOAL GRAB SECTION --//

  Linear.setMaxVoltages(12);
  Robot.DriveFor(7.5, L_Settings, false, 400);
  Robot.TurnFor(-90, A60_Settings, 800);

  Robot.DriveFor(-26.5, L_Settings, true, 1350, true);

  wait(0.85, seconds);

  Mogo.open();

  Robot.waitChassis();

  //-- THIRD RING SCORE SECTION --//

  Robot.TurnFor(0, A60_Settings, 950);

  Linear.setMaxVoltages(12);

  Robot.DriveFor(24.5, L_Settings, false, 1000);

  launch_task([&]{
    halfOpen();
  });

  Robot.TurnFor(90, A60_Settings, 800);

  wait(0.15, seconds);

  IntakeLift.open();

  Intake.spin(reverse, 12, volt);

  Linear.setMaxVoltages(9);

  Robot.DriveFor(10, L_Settings, false);

  IntakeLift.close();

  wait(0.45, seconds);

  Linear.setMaxVoltages(12);

  Robot.DriveFor(-20, L_Settings, true, 700);

  Intake.spin(reverse, 10, volt);

  launch_task([&] {
    resetLift();
  });

  //-- LADDER TOUCH SECTION --//

  Robot.TurnFor(-22.5, A60_Settings, 1000);

  Linear.setMaxVoltages(12);

  Robot.DriveFor(-38, L_Settings, true, 4000, true);

  wait(0.8, sec);

  Mogo.close();

  Intake.spin(reverse, 4.5, volt);

  Robot.waitChassis();

  return;
}

void BlueSoloAWP() {
  // TUNE LATER
  
  pneumatics Mogo(Brain.ThreeWirePort.G);
  pneumatics IntakeLift(Brain.ThreeWirePort.F);

  AntiDrift.setGains(A120_Settings);

  Lift.spin(reverse, 7, volt); // makes sure lift doesnt pop out

  //-- GOAL RUSH SECTION --//
  
  Robot.DriveFor(-35, L_Settings, true, 0);
  Robot.TurnFor(-37.5, A0_Settings, 550);

  Linear.setMaxVoltages(9);

  Robot.DriveFor(-15, L_Settings, false, 900, true);

  wait(0.65, seconds);

  Mogo.open();

  Robot.waitChassis();

  Linear.setMaxVoltages(12);

  //wait(0.25, seconds);

  Intake.spin(reverse, 12, volt);

  //-- SECOND RING SCORING SECTION --//

  Robot.DriveFor(7.5, L_Settings, true, 0);

  Robot.TurnFor(31.5, A0_Settings, 700);
  Robot.DriveFor(15, L_Settings, false, 600);
  wait(200, msec);
  Linear.setMaxVoltages(6.5);
  Robot.DriveFor(-7.5, L_Settings, true, 500);

  wait(0.6, seconds);

  Intake.stop(coast);

  Mogo.close();

  //-- SECOND GOAL GRAB SECTION --//

  Linear.setMaxVoltages(12);
  Robot.DriveFor(7.5, L_Settings, false, 400);
  Robot.TurnFor(90, A60_Settings, 800);

  Robot.DriveFor(-26.5, L_Settings, true, 1350, true);

  wait(0.9, seconds);

  Mogo.open();

  Robot.waitChassis();

  //-- THIRD RING SCORE SECTION --//

  Robot.TurnFor(0, A60_Settings, 800);

  Linear.setMaxVoltages(12);

  Robot.DriveFor(20.5, L_Settings, false, 800);

  launch_task([&]{
    halfOpen();
  });

  Robot.TurnFor(-90, A60_Settings, 650);

  wait(0.15, seconds);

  IntakeLift.open();

  Intake.spin(reverse, 12, volt);

  Linear.setMaxVoltages(10);

  Robot.DriveFor(9.5, L_Settings, false);

  IntakeLift.close();

  wait(0.6, seconds);

  Linear.setMaxVoltages(10.5);

  Robot.DriveFor(-20, L_Settings, true, 700);

  launch_task([&] {
    resetLift();
  });


  //-- LADDER TOUCH SECTION --//

  Robot.TurnFor(25, A60_Settings, 1000);

  Linear.setMaxVoltages(12);

  Robot.DriveFor(-37, L_Settings, true, 0, true);

  wait(0.8, sec);

  Mogo.close();

  Intake.spin(reverse, 4.5, volt);

  Robot.waitChassis();

  return;
}

void RedOppSide() {
  pneumatics Mogo(Brain.ThreeWirePort.G);
  pneumatics IntakeLift(Brain.ThreeWirePort.F);

  Lift.spin(reverse, 7, volt); // makes sure lift doesnt pop out

  Robot.DriveFor(-19, L_Settings, true);
  Robot.TurnFor(25.5, A0_Settings, 450);
  Robot.DriveFor(-10.5, L_Settings, true, 700);

  Mogo.open();

  Robot.DriveFor(-3.5, L_Settings, true, 300);

  // launch_task([&]{
  //   halfOpen(100);
  // });

  Robot.TurnFor(90, A60_Settings, 800);

  Intake.spin(reverse, 12, volt);

  Robot.DriveFor(19, L_Settings, false);

  Robot.TurnFor(180, A60_Settings, 800);

  Robot.DriveFor(12.5, L_Settings);

  wait(350, msec);

  Robot.TurnFor(180, A0_Settings, 400);

  Robot.DriveFor(-9, L_Settings, true, 400);

  Robot.TurnFor(160, A0_Settings, 400);

  Robot.DriveFor(11, L_Settings, 500);

  wait(350, msec);

  Robot.DriveFor(-20, L_Settings);

  Robot.TurnFor(60, A60_Settings, 1000);

  Intake.spin(reverse, 4.5, volt);

  // Linear.setMaxVoltages(7);

  // Robot.DriveFor(-48, L_Settings, true, 1750, true);
  
  // wait(800, msec);

  //Mogo.close();

  return;
  
  Robot.TurnFor(108, A120_Settings, 1250);

  Robot.DriveFor(24.5, L_Settings, false);

  Linear.setMaxVoltages(4.5);

  Robot.DriveFor(6, L_Settings, false);

  // positioning time

  Robot.DriveFor(-10, L_Settings, true);

  Robot.TurnFor(45, A0_Settings, 800);

  Robot.DriveFor(-11, L_Settings, true);

  launch_task([&] {
    resetLift();
  });

  Robot.TurnFor(135, A60_Settings, 800);

  //Mogo.close();

  Robot.DriveFor(21, L_Settings, false);

  Robot.TurnFor(96, A0_Settings, 400);

  // Linear.setMaxVoltages(5);

  // Robot.DriveFor(3.5, L_Settings);

  // Angular.setMaxVoltages(12);

  // wait(0.5, seconds);

  // Robot.DriveFor(9, L_Settings);

  // wait(250, msec);

  // IntakeToLift();

  // Intake.spin(reverse, 12, volt);

  // wait(250, msec);

  // IntakeToLift();
}

void BlueOppSide() {
    pneumatics Mogo(Brain.ThreeWirePort.G);
  pneumatics IntakeLift(Brain.ThreeWirePort.F);

  Lift.spin(reverse, 7, volt); // makes sure lift doesnt pop out

  Robot.DriveFor(-19, L_Settings, true);
  Robot.TurnFor(-31, A0_Settings, 450);
  Robot.DriveFor(-10.5, L_Settings, true, 700);

  Mogo.open();

  Robot.DriveFor(-3.5, L_Settings, true, 500);

  // launch_task([&]{
  //   halfOpen(100);
  // });

  Robot.TurnFor(-90, A60_Settings, 800);

  Intake.spin(reverse, 12, volt);

  Robot.DriveFor(15, L_Settings, false);

  Robot.TurnFor(-180, A60_Settings, 800);

  Robot.DriveFor(12.5, L_Settings);

  wait(350, msec);

  Robot.TurnFor(-180, A0_Settings, 400);

  Robot.DriveFor(-9, L_Settings, true, 400);

  Robot.TurnFor(-140, A0_Settings, 650);

  Robot.DriveFor(12.5, L_Settings, 600);

  wait(350, msec);

  Robot.DriveFor(-20, L_Settings);

  Robot.TurnFor(-60, A60_Settings, 1000);

  Intake.spin(reverse, 4.5, volt);

  Linear.setMaxVoltages(7);

  Robot.DriveFor(-27.5, L_Settings, true, 1750, true);
  
  wait(600, msec);

  Mogo.close();

  return;
  // pneumatics Mogo(Brain.ThreeWirePort.G);
  // pneumatics IntakeLift(Brain.ThreeWirePort.F);

  // Lift.spin(reverse, 7, volt); // makes sure lift doesnt pop out

  // Robot.DriveFor(-16, true);
  // Robot.TurnFor(-31.5, 600);
  // Robot.DriveFor(-5.5, true);

  // Mogo.open();

  // Angular.setMaxVoltages(10);

  // launch_task([&]{
  //   halfOpen(100);
  // });

  // Robot.TurnFor(50, 1250);

  // IntakeLift.open();

  // Linear.setMaxVoltages(8);

  // Intake.spin(reverse, 12, volt);

  // Robot.DriveFor(21.5, false);

  // IntakeLift.close();

  // wait(0.5, seconds);

  // Robot.DriveFor(-7.5, false);

  // Linear.setMaxVoltages(12);

  // wait(0.65, seconds);

  // Angular.setMaxVoltages(9);
  
  // Robot.TurnFor(-112, 1500);

  // Robot.DriveFor(28, false);

  // Linear.setMaxVoltages(4.5);

  // Robot.DriveFor(6, false);

  // // positioning time

  // Robot.DriveFor(-10, true);

  // Robot.TurnFor(-45, 800);

  // Robot.DriveFor(-11.5, true);

  // launch_task([&] {
  //   resetLift();
  // });

  // Robot.TurnFor(-135, 1200);

  // //Mogo.close();

  // Robot.DriveFor(13.5, false);

  // Robot.TurnFor(-95, 750);

  // Linear.setMaxVoltages(6.5);

  // Robot.DriveFor(3.5);

  // Angular.setMaxVoltages(12);

  // wait(0.5, seconds);

  // Robot.DriveFor(7);

  // wait(250, msec);

  // IntakeToLift();

  // Intake.spin(reverse, 12, volt);

  // wait(250, msec);

  // IntakeToLift();
}

void skills() {
  pneumatics Mogo(Brain.ThreeWirePort.G);
  pneumatics IntakeLift(Brain.ThreeWirePort.F);
  pneumatics Extender(Brain.ThreeWirePort.E);

  Lift.spin(reverse, 7, volt);

  // Score preload on alliance stake
  Intake.spin(reverse, 12, volt);
  wait(400, msec);

  Robot.DriveFor(13.5, L_Settings);

  Robot.TurnFor(90, A60_Settings, 800);

  Intake.spin(fwd, 12, volt);

  // Grab mogo
  Linear.setMaxVoltages(9);
  //Robot.DriveToPoint(13.5, 13, L_Settings, A0_Settings, 0, true);
  Robot.DriveFor(-22, L_Settings, true, 1000, true);

  wait(700, msec);

  Intake.spin(reverse, 12, volt);

  Mogo.open();

  Robot.waitChassis();

  Linear.setMaxVoltages(12);

  Robot.TurnFor(0, A60_Settings, 800);

  Robot.DriveFor(24, L_Settings);

  wait(600, msec);

  Robot.TurnFor(45, A0_Settings, 450);

  Intake.stop();

  Robot.DriveFor(32, L_Settings, false, 1500, true);

  wait(600, msec);

  Intake.spin(reverse, 12, volt);

  Robot.waitChassis();

  wait(100, msec);

  Intake.stop();

  Robot.DriveFor(-32, L_Settings, true);

  Intake.spin(reverse, 12, volt);

  Robot.TurnFor(-90, A60_Settings, 800);

  Robot.DriveFor(20.5, L_Settings, 1000); // maybe change this

  Robot.TurnFor(180, A120_Settings, 1000);

  Linear.setMaxVoltages(12);

  Robot.DriveFor(18, L_Settings, true, 900);

  wait(600, msec);

  Robot.DriveFor(14.5, L_Settings, true, 800);

  Robot.TurnFor(-55, A120_Settings, 1000);

  Robot.DriveFor(13.5, LMogo_Settings, 800);

  wait(350, msec);

  Robot.TurnFor(32.5, A60_Settings, 800);

  Robot.DriveFor(-13.5, LMogo_Settings, true, 800);

  Intake.spin(fwd, 12, volt);

  Mogo.close();

  wait(450, msec);

  Robot.DriveFor(10.5, L_Settings, false, 500);

  Robot.TurnFor(-90, A0_Settings, 900);

  wait(100, sec);

 // printCoordinates(); // 5,8,-92; 7,7,-92, 8, 7, -92;

  Linear.setMaxVoltages(12); 

  Robot.DriveToPoint(70, 5.5, L_Settings, A0_Settings, 0, true);

  Robot.TurnFor(-90, A0_Settings, 200);

  Linear.setMaxVoltages(7.5);

  Robot.DriveFor(-10, L_Settings, true, 800);

  //Robot.DriveToPoint(100, 5.5, L_Settings, A0_Settings, 800, true);

  Intake.stop();

  Mogo.open();

  Robot.waitChassis();

  Robot.TurnFor(0, A60_Settings, 600);

  printCoordinates(); // [60,-10]

  Intake.spin(reverse, 12, volt);

  Linear.setMaxVoltages(12);

  wait(100, sec);

  Robot.DriveToPoint(90, 35, L_Settings, A0_Settings);

  wait(500, msec);

  Robot.TurnFor(90, A60_Settings, 600);

  Robot.DriveToPoint(110, 32.5, L_Settings, A0_Settings);

  wait(500, msec);

  Robot.TurnFor(25, A60_Settings, 600);

  printCoordinates();

  Robot.DriveToPoint(115, 48, L_Settings, A0_Settings);

  wait(500, msec);


  // Robot.TurnFor(0, A0_Settings, 650);

  // Robot.DriveFor(-10.5, LMogo_Settings, true);

  // Robot.TurnFor(45, A0_Settings, 650);

  // Robot.DriveFor(-4, LMogo_Settings);

  // Mogo.close();

  // Robot.DriveFor(4, L_Settings);

  //Robot.TurnFor(-90, A60_Settings);

}

void testGainScheduling() {
  pneumatics Mogo(Brain.ThreeWirePort.G);

  Robot.DriveFor(72, L_Settings, true);
  Controller.rumble("-");
}

void tuneOffsets() {
  Robot.TurnFor(180, A120_Settings, 1000);
  
  Bucees::Coordinates currentCoordinates = Robot.getRobotCoordinates(false);
  printf("current: %f, %f, %f \n", currentCoordinates.x, currentCoordinates.y, currentCoordinates.theta);
}

// DataSink_Motor<vex::motor*, DATA_TYPES> kalman("KalmanDataMotor", 
// {
// Bucees::createDataCapture(&TopLeft, MOTOR_VELOCITY_RPM),
// Bucees::createDataCapture(&TopRight, MOTOR_VELOCITY_RPM),
// Bucees::createDataCapture(&FrontLeft, MOTOR_VELOCITY_RPM),
// Bucees::createDataCapture(&FrontRight, MOTOR_VELOCITY_RPM),
// Bucees::createDataCapture(&BackLeft, MOTOR_VELOCITY_RPM),
// Bucees::createDataCapture(&BackRight, MOTOR_VELOCITY_RPM),}, 
// 50);

// DataSink_Robot<Bucees::Robot*, DATA_TYPES> kalman2("KalmanDataOdom",
// {
//  Bucees::createDataCapture(&Robot, ODOMETRY_COORDINATE_X), 
//  Bucees::createDataCapture(&Robot, ODOMETRY_COORDINATE_Y), 
//  Bucees::createDataCapture(&Robot, ODOMETRY_COORDINATE_ROTATION_RAD), 
// }, 50);

// void testKalmanFilter() {

//   //Linear.setMaxVoltages(5);
// activateMotionChaining(false, 9);
//   // Robot.DriveToPoint(0, 24, L_Settings, A0_Settings, 0, true);
//  // Controller.rumble("-");
//   Robot.DriveToPoint(-24, 40, L_Settings, A60_Settings, 0, true);
//   deactivateMotionChaining();
// //  Controller.rumble("-");
//   Robot.DriveToPoint(-48, 40, L_Settings, A0_Settings, 0, true);
//  // Controller.rumble("-");
// }

// void testMotions() {
//   //L_Settings.maxVoltages = 9;
   
//   activateMotionChaining(false, 9);
//   Robot.DriveToPoint(0, 24, L_Settings, A0_Settings);
//  // Controller.rumble("-");
//   Robot.DriveToPoint(-24, 40, L_Settings, A60_Settings);
//   deactivateMotionChaining();
// //  Controller.rumble("-");
//   Robot.DriveToPoint(-48, 40, L_Settings, A0_Settings);
//  // Controller.rumble("-");
//   return;
// }

void autonomous(void) {

  switch (autonSelected) {

    case -1:
    return;

    case 0:
    RedSoloAWP();
    return;

    case 1:
    BlueSoloAWP();
    return;

    case 2:
    RedOppSide();
    return;

    case 3:
    BlueOppSide();
    return;

    case 4:
    return;

    case 5:
    return;

    case 6:
    //testKalmanFilter();
    return;

    case 7:
    skills();
    return;
  }
}

void usercontrol(void) {

  // pneumatics:
  Controller.ButtonB.pressed(toggleMogoF);
  Controller.ButtonY.pressed(toggleExtenderF);
  Controller.ButtonLeft.pressed(toggleStartMatchF);
  Controller.ButtonDown.pressed(toggleLiftClampF);
  Controller.ButtonUp.pressed(toggleLiftRatchetF);
  Controller.ButtonA.pressed(toggleIntakeLiftF); // intake lift
  Controller.ButtonL1.pressed(macroDisrupter);

  launch_task([&] {
    liftClampChecker();
  });

  launch_task([&] {

  });

  //Robot.setRobotCoordinates({8, 7, -92});
  
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
    } else if (inLiftMacro == false && startMatchDB == false) {
      Lift.stop();
      printf("stop it!!!\n");
    }

    Brain.Screen.printAt(50, 25, "BackLeft Temp: %f", BackLeft.temperature(temperatureUnits::fahrenheit));
    Brain.Screen.printAt(50, 50, "BackRight Temp: %f", BackRight.temperature(temperatureUnits::fahrenheit));
    Brain.Screen.printAt(50, 75, "TopLeft Temp: %f", TopLeft.temperature(temperatureUnits::fahrenheit));
    Brain.Screen.printAt(50, 100, "TopRight Temp: %f", TopRight.temperature(temperatureUnits::fahrenheit));
    Brain.Screen.printAt(50, 125, "FrontLeft Temp: %f", FrontLeft.temperature(temperatureUnits::fahrenheit));
    Brain.Screen.printAt(50, 150, "FrontRight Temp: %f", FrontRight.temperature(temperatureUnits::fahrenheit));

    Brain.Screen.drawImageFromFile("Brain_Screen_Logo.png", 0, 0);

    Bucees::Coordinates currentCoordinates = Robot.getRobotCoordinates(false);
   // std::cout << "Color: " << RingFilter.isNearObject() << std::endl;
    //printf("current: %f, %f, %f \n", currentCoordinates.x, currentCoordinates.y, currentCoordinates.theta);

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
