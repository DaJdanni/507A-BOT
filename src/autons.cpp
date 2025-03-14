#include "vex.h"
using namespace vex;

template <class F>
vex::task launch_task(F&& function) {
  return vex::task([](void* parameters) {
    std::unique_ptr<std::function<void()>> ptr{static_cast<std::function<void()>*>(parameters)};
    (*ptr)();
    return 0;
  }, new std::function<void()>(std::forward<F>(function)));
}

int waitTimeFilter = 170;
bool blueFilter;
bool redFilter;

void bottomFilter(int timeout, vex::brakeType braketype = brakeType::coast) {
  int currentTime = 0;
  while (1) {

    if (RingFilterBottom.isNearObject() == true) {
      std::cout << "DETECTED" << std::endl;
      Intake.stop(braketype);
      break;        
    }

    currentTime += 10;

    if (currentTime >= timeout) break;

    wait(10, msec);
  }
}

void topFilter(int timeout, vex::brakeType braketype = brakeType::coast) {
  int currentTime = 0;
  while (1) {

    if (RingFilter.isNearObject() == true) {
      std::cout << "DETECTED" << std::endl;
      Intake.stop(braketype);
      break;        
    }

    currentTime += 10;

    if (currentTime >= timeout) break;

    wait(10, msec);
  }
}

int delay1Filter = 105;
int delay2Filter = 105;


void filter(COLOR_SORTER sortColor) {
  if (RingFilter.isNearObject() != true) return;
 // std::cout << "h" << std::endl;
  if (RingFilter.hue() < 30 && sortColor != FILTER_RED) return;
  //std::cout << "h1" << std::endl;
  if (RingFilter.hue() > 190 && sortColor != FILTER_BLUE) return;
  //std::cout << "h2" << std::endl;
  //if (RingFilter.hue() > 20 && RingFilter.hue() < 200) return;
  //std::cout << "hey" << std::endl;
  //Controller.rumble(".");
  wait(delay1Filter, msec);
  Intake.spin(forward, 12, volt);
  wait(delay2Filter, msec);
  Intake.spin(reverse, 12, volt);
}


void filterBlueRings() {
  filter(FILTER_BLUE);
}

void filterRedRings() {
  filter(FILTER_RED);
}

void testSkills(bool elims) {

}

void doNothing(bool elims) {

}


//   printCoordinates();
void startDrivetrain(double speed) {
  LeftSide.spin(fwd, speed, volt);
  RightSide.spin(fwd, speed, volt);
}

void stopDrivetrain(brakeType brake) {
  LeftSide.stop(brake);
  RightSide.stop(brake);
}

void limitLadyBrown() {
  while (1) {
    float rotationPosition = (ladyBrown1.position(degrees) + ladyBrown2.position(degrees)) / 2;
    if (rotationPosition > 680) {
      ladyBrown.stop();
    }
    wait(10, msec);
  }
}

double stage1Macro = 74.5;

void skills(bool elims) {
  pneumatics Clamp(Brain.ThreeWirePort.A);
  pneumatics Doinker(Brain.ThreeWirePort.B);
  pneumatics goalRush(Brain.ThreeWirePort.C);

  Robot.setRobotCoordinates({-64, 0, 90});

  std::cout << "SKILLS" << std::endl;

  printCoordinates();

  Intake.setVelocity(100, pct);
  ladyBrown1.resetPosition();
  ladyBrown2.resetPosition();

  launch_task([&] {
    limitLadyBrown();
  });

  Intake.spin(reverse, 12, volt);

  wait(400, msec); // wait until finished scoring red alliance stake

  //-- WALL STAKE #1 SECTION --//

  Robot.DriveToPoint(-54, 0, L_Settings, A0_Settings, 400);

  Intake.stop(coast);

  Robot.TurnFor(-20, A120_Settings, 400);

  printCoordinates();

  Linear.setMaxVoltages(8.5);

  Robot.DriveToPoint(-46, -26, L_Settings, A0_Settings, 1250, true);

  Clamp.open(); // grab goal

  wait(200, msec);

  Linear.setMaxVoltages(12);

  Intake.spin(reverse, 12, volt);

  Robot.TurnFor(90, A60_Settings, 800); // face rings

  activateMotionChaining(false, 10);

  Robot.DriveToPoint(-35, -26, L_Settings, A0_Settings); // ring 1

  //Robot.DriveToPoint(27, 41, L_Settings, A0_Settings);

  deactivateMotionChaining(false);

  launch_task([&] {wait(1050, msec); lBPid(76.5, 1250);});
  Robot.DriveToPoint(25.5, -48, L_Settings, A0_Settings); // ring 2

  Linear.setMaxVoltages(12);
  
  Robot.DriveToPoint(1, -43, L_Settings, A0_Settings, 0, true, false, hold); // GET INTO POSITION

  launch_task([&] {  
    Intake.spin(forward, 6, volt);
    wait(60, msec);
    Intake.stop(coast);}
  );

  Robot.TurnFor(180, A60_Settings, 700); // face wall stake
  launch_task([&] {lBPid(195, 700);});

  Robot.waitChassis();

  printCoordinates();

  Linear.setMaxVoltages(6.5);
  Robot.DriveToPoint(0, -75, L_Settings, A0_Settings, 2000, false, true);

  Intake.spin(reverse, 12, volt);

  wait(900, msec);

  ladyBrown.spin(forward, 12, volt); // score wallstake

  wait(500, msec);

  ladyBrown.stop(coast);

  Robot.waitChassis();

  printCoordinates();

  launch_task([&] {lBPid(260);}); // reset lady brown

  //-- GOAL #1 SECTION --//

  Linear.setMaxVoltages(12);

  Robot.DriveToPoint(-2, -46, L_Settings, A0_Settings, 0, true); // back up

  Robot.TurnFor(-90, L_Settings, 800); // face the rings 
  
  Intake.spin(reverse, 12, volt);

  Linear.setMaxVoltages(9);

  Robot.DriveToPoint(-60, -48, L_Settings, A0_Settings, 2750); // drive straight into rings 3,4,5

  Robot.TurnFor(135, L_Settings, 800); // adjust position to face 6th ring

  Robot.DriveToPoint(-48, -60, L_Settings, A0_Settings, 1000); // grab it

  Robot.TurnFor(120, A120_Settings, 700);

  Linear.setMaxVoltages(12);

  printCoordinates();

  Robot.DriveToPoint(-62, -64, L_Settings, A60_Settings, 800, true); // go into the corner
  Intake.stop(coast);

  printCoordinates();

  Clamp.close();

  //Robot.wallResetOdom();

  printCoordinates();

  // GOAL #2 CORNER SECTION //

  activateMotionChaining(false, 10.5);
  Intake.spin(forward, 12, volt);

  Robot.DriveToPoint(32.5, -48, L_Settings, A0_Settings);
  launch_task([&] {lBPid(77.5);});

  deactivateMotionChaining();

  Intake.spin(reverse, 12, volt); // start intaking when we get close to the ring
  Linear.setMaxVoltages(9.5);

  Robot.DriveToPoint(48, -48, L_Settings, A0_Settings);  // grab ring

  wait(200, msec);

  Robot.TurnFor(210, A0_Settings, 500); // face the goal

  Linear.setMaxVoltages(9.5);

  Robot.DriveToPoint(58, -20.5, L_Settings, A0_Settings, 1500, true); // drive into the goal

  Clamp.open();

  wait(200, msec);

  Robot.TurnFor(172.5, A0_Settings, 500); // face corner to sweep

  Doinker.open();

  Linear.setMaxVoltages(12);

  Robot.DriveToPoint(72, -72, L_Settings, A0_Settings, 1600); // get into corner

  // start sweeping
  LeftSide.spin(fwd, 10.5, volt);
  RightSide.spin(reverse, 10.5, volt);

  waitUntil(Robot.getAbsoluteHeading() > 250);

  LeftSide.stop(coast);
  RightSide.stop(coast);

  Doinker.close();
  wait(350, msec);

  Robot.TurnFor(300, A0_Settings, 600); // adjust for better angle

  // Robot.TurnFor(100, A0_Settings, 1000);

  // Robot.TurnFor(250, A0_Settings, 1000);

  Intake.stop(coast);
  Doinker.close();
  Clamp.close();

  launch_task([&] {
    wait(200, msec);
    lBPid(140, 1250, 10);
  });


  Linear.setMaxVoltages(12);

  Robot.DriveToPoint(68, -67, L_Settings, A0_Settings, 400, true); // back into corner

  Robot.wallResetOdom();

  //-- ALLIANCE STAKE BLUE SECTION --//

  Robot.DriveToPoint(49, -26, L_Settings, A0_Settings); // drive towards goal

  Robot.TurnFor(180, L_Settings, 800); // turn around to grab it

  Intake.spin(forward, 12, volt);

  Linear.setMaxVoltages(8.5);

  Robot.DriveToPoint(49, 1.5, L_Settings, A0_Settings, 0, true, false, hold); // get ready to clamp it

  Clamp.open();

  wait(200, msec);

  Linear.setMaxVoltages(12);

  Robot.TurnFor(90, A60_Settings, 800); // face wallstake

  Robot.waitChassis();

  Linear.setMaxVoltages(8);

  startDrivetrain(5.5);

  wait(1200, msec);

  stopDrivetrain(coast);

  Linear.setMaxVoltages(12);

  Robot.DriveFor(-7, L_Settings, 500); // position yourself
  
  // score
  ladyBrown.spin(forward, 12, volt);
  wait(700, msec);
  ladyBrown.stop(coast);  

  Intake.spin(reverse, 12, volt);

  //-- GOAL #2 SECTION --//

  Robot.DriveToPoint(46, 0, L_Settings, A0_Settings, 800, true); // backup

  launch_task([&] {lBPid(0, 1250);});
  Robot.TurnFor(210, A60_Settings, 500); // face ring

  printCoordinates();

  //wait(100, sec);

  Robot.DriveToPoint(26, -26, L_Settings, A0_Settings);

  Angular.setMaxVoltages(5.5);

  Robot.TurnFor(315, A120_Settings, 1000);

  Intake.stop();

  Angular.setMaxVoltages(12);
  Linear.setMaxVoltages(10);

  Robot.DriveToPoint(0, 0, L_Settings, A0_Settings);

  launch_task([&] {wait(450, msec); Intake.spin(reverse, 12, volt);});
  Linear.setMaxVoltages(10.5);
  Robot.DriveToPoint(-27, 22.5, L_Settings, A0_Settings);

  Robot.DriveToPoint(-39.5, 40.5, L_Settings, A0_Settings);

  Robot.TurnFor(270, L_Settings, 800);

  Linear.setMaxVoltages(11);

  Robot.DriveToPoint(-64, 40.5, L_Settings, A0_Settings, 800);

  wait(500, msec);

  Robot.TurnFor(35, L_Settings, 800);

  Robot.DriveToPoint(-48, 52, L_Settings, A0_Settings, 1000);

  Robot.TurnFor(110, A120_Settings, 800);

  Clamp.close();

  Linear.setMaxVoltages(12);

  printCoordinates();

  Robot.DriveToPoint(-66, 66, L_Settings, A60_Settings, 800, true); // go into the corner
  Intake.stop(coast);

  //Robot.wallResetOdom();

  printCoordinates();

  Linear.setMaxVoltages(12);

  Robot.DriveToPoint(-21, 46, L_Settings, A0_Settings, 0, false, true);

  wait(200, msec);
  launch_task([&] {lBPid(77.5);});

  Intake.spin(reverse, 12, volt);

  Robot.waitChassis();

  Linear.setMaxVoltages(8.5);

  Robot.DriveToPoint(-48, 17.5, L_Settings, A0_Settings, 0, true);

  Clamp.open();
  wait(200, msec);

  printCoordinates();

  Intake.stop(coast);
  wait(200, msec);

  Linear.setMaxVoltages(12);

  Robot.DriveToPoint(0.5, 38, L_Settings, A0_Settings, 0, false, false, hold);
  launch_task([&] {lBPid(195, 700);});

  Robot.TurnFor(0, A120_Settings, 800);

  Linear.setMaxVoltages(7);
  Robot.DriveToPoint(0, 75, L_Settings, A0_Settings, 2000, false, true);

  Intake.spin(reverse, 12, volt);

  wait(1050, msec);

  ladyBrown.spin(forward, 12, volt); // score wallstake

  wait(500, msec);

  ladyBrown.stop(coast);

  Robot.waitChassis();

  printCoordinates();

  launch_task([&] {lBPid(260);}); // reset lady brown

  //-- GOAL #3 SECTION --//

  Linear.setMaxVoltages(12);

  Robot.DriveToPoint(0, 48, L_Settings, A0_Settings, 0, true); // back up

  Robot.TurnFor(90, A60_Settings, 400);

  Linear.setMaxVoltages(10.5);

  Intake.spin(reverse, 12, volt);

  Robot.DriveToPoint(24, 48, L_Settings, A0_Settings); // second ring

  Robot.TurnFor(180, A120_Settings, 400);

  Robot.DriveToPoint(27, 22, L_Settings, A0_Settings); // third ring

  Robot.TurnFor(45, A120_Settings, 400);

  Robot.DriveToPoint(47, 47.5, L_Settings, A0_Settings); // fourth

  Robot.TurnFor(85, A0_Settings, 400);

  Robot.DriveToPoint(62, 46.5, L_Settings, A0_Settings, 1000); // and fifth

  Linear.setMaxVoltages(12);

  printCoordinates();

  Robot.DriveToPoint(16, 65, L_Settings, A0_Settings, 1000, true);

  Doinker.open();

  launch_task([&] {
    waitUntil(RingFilter.isNearObject() == true);
    if (RingFilter.hue() > 190) Intake.stop(coast);
  });

  Robot.DriveToPoint(67, 72, L_Settings, A0_Settings, 1500);

  LeftSide.spin(fwd, 12, volt);
  RightSide.spin(reverse, 12, volt);

  waitUntil(Robot.getAbsoluteHeading() > 200);

  Intake.stop(coast);

  Doinker.close();

  Clamp.close();

  Robot.DriveToPoint(70, 70, L_Settings, A0_Settings, 800, true);

  Robot.DriveToPoint(24, 24, L_Settings, A0_Settings);

  launch_task([&] {lBPid(900);});

  Robot.TurnFor(45, A120_Settings, 800);

  //Intake.spin(forward, 12, volt);

  startDrivetrain(-4.5);

  wait(1000, msec);

  stopDrivetrain(coast);

  // turn for 255
  // drive to -12,19 reverse for goal
  // -43, 43 for next ring
  // -37, 67 for line up wallstsake
  // turn to 270
  // score
  // back up to -38, 66
  // drive to -38.5, 108 for 2 rings in a line
  // drive to -39, 99 back up
  //turn to 313
  // drive to -48, 108 for another ring
  // turn to 22 for last ring
  // drive to -39, 123 to intake it
  // turn to 115 for last corner
  // -48, 127 for dropoff



}

void soloBlueAWP(bool elims) {
  pneumatics Clamp(Brain.ThreeWirePort.A);
  pneumatics Doinker(Brain.ThreeWirePort.B);
  pneumatics goalRush(Brain.ThreeWirePort.C);
  pneumatics Pistake(Brain.ThreeWirePort.H);

  RingFilter.objectDetected(filterRedRings);
  Intake.setVelocity(100, pct);
  ladyBrown1.resetPosition();
  ladyBrown2.resetPosition();

  delay1Filter = 100;
  delay2Filter = 90;
  
  std::cout << "SOLO AWP BLUE" << std::endl;

  Robot.setRobotCoordinates({52.4, 23, 90});

  Linear.setMaxVoltages(9.5);

  Robot.DriveToPoint(22, 23, L_Settings, A0_Settings, 0, true);

  Clamp.open();
  wait(200, msec);

  Linear.setMaxVoltages(10.5);

  activateMotionChaining(false, 4);

  Robot.TurnFor(315, A60_Settings, 800);

  Intake.spin(reverse, 12, volt);

  Robot.DriveToPoint(9.5, 39, L_Settings, A0_Settings);

  deactivateMotionChaining();

  Robot.DriveToPoint(9.5, 54, L_Settings, A0_Settings);

  Angular.setMaxVoltages(7);

  Robot.TurnFor(135, A120_Settings, 650);

  Angular.setMaxVoltages(12);

  activateMotionChaining(false, 5.5);

  Robot.DriveToPoint(32, 44, L_Settings, A0_Settings);

  deactivateMotionChaining();

  Linear.setMaxVoltages(12);

  Robot.DriveToPoint(53, 18, L_Settings, A0_Settings);

  launch_task([&] {wait(300, msec); lBPid(77.5, 1000);});
  
  Clamp.close();
  Pistake.open();
  wait(200, msec);

  Robot.TurnFor(180, A120_Settings, 700);

  Robot.DriveToPoint(48, -1.5, L_Settings, A0_Settings, 0, false, false);

  Robot.TurnFor(84, A60_Settings, 800);
  wait(700, msec);

  Intake.stop(coast);
  lBPid(290, 500);

  //Intake.spin(reverse, 12, volt);

  startDrivetrain(5.5);

  wait(800, msec);

  stopDrivetrain(coast);

  Robot.DriveFor(-7, L_Settings, 500); // position yourself
  
  // score
  ladyBrown.spin(forward, 12, volt);
  wait(700, msec);
  ladyBrown.stop(coast);  

  //-- GOAL #2 SECTION --//

  //Robot.DriveToPoint(53.5, 0, L_Settings, A0_Settings, 800, true); // backup

  Intake.stop();

  activateMotionChaining(6.5);

  launch_task([&] {lBPid(0);});

  Robot.DriveToPoint(38, -16, L_Settings, A0_Settings, 0, true);

  Pistake.close();
  deactivateMotionChaining();

  Linear.setMaxVoltages(8.5);
  
  Robot.DriveToPoint(23.5, -27.5, L_Settings, A0_Settings, 0, true);

  Clamp.open();

  wait(50, msec);

  Robot.TurnFor(180, A120_Settings, 800);
  
  Linear.setMaxVoltages(12);

  Intake.spin(reverse, 12, volt);

  Robot.DriveToPoint(28, -51, L_Settings, A0_Settings);

  //launch_task([&] {wait(800, msec); Intake.spin(reverse, 12, volt);});

  Linear.setMaxVoltages(10.5);

  Robot.DriveToPoint(11.5, -16, L_Settings, A0_Settings, 0, true, false, hold);

 // Robot.TurnFor(-45, A120_Settings, 600);

  Clamp.close();

}

void goalRushRed(bool elims) { // RETUNE

  pneumatics Clamp(Brain.ThreeWirePort.A);
  pneumatics Doinker(Brain.ThreeWirePort.B);
  pneumatics goalRush(Brain.ThreeWirePort.C);
  
  std::cout << "GOAL RUSH RED" << std::endl;

  ladyBrown1.setPosition(60, deg);
  ladyBrown2.setPosition(60, deg);
  ladyBrown.setPosition(60, deg);

  launch_task([&] {
    launch_task([&] {wait(450, msec); topFilter(5000);});
    lBPid(258);
  });

  launch_task([&] {
  waitUntil(GoalDetector.objectRawSize() == -1 && GoalDetector.objectDistance(inches) < 1.5);
  goalRush.open();
  std::cout << "clamp" << std::endl;
  });

  Intake.spin(reverse, 12, volt);

  Doinker.open();

  Robot.DriveToPoint(-1, 36, L_Settings, A60_Settings);

  printCoordinates();

  Linear.setMaxVoltages(12);

  Robot.DriveToPoint(0, 13, L_Settings, A0_Settings, 1500, true, true);

  // wait(200, msec);
  // Intake.stop();
  wait(800, msec);;
  goalRush.close();
  wait(100, msec);
  Robot.DriveFor(-4, L_Settings, false, 300);
  Doinker.close();
  Robot.waitChassis();

  //wait(500, msec);

  Robot.TurnFor(170, A120_Settings, 800); // -10 offset

  Linear.setMaxVoltages(9);

  Robot.DriveToPoint(-8, 39.5, L_Settings, A0_Settings, 2500, true);
  Linear.setMaxVoltages(12);

  Clamp.open();

  //wait(200, sec);

  wait(200, msec);
  
  Intake.spin(reverse, 12, volt);

  printCoordinates();

  //Linear.setMaxVoltages(9.5);

  Robot.DriveToPoint(19.5, 4, L_Settings, A0_Settings, 0, false, true);

  wait(800, msec);
  
  Clamp.close();

  Robot.waitChassis();

  //launch_task([&] {lBPid(72.5);});//63.5 84

  printCoordinates();

  Robot.TurnFor(110, A0_Settings, 800);
  Linear.setMaxVoltages(12);

  //wait(150, msec);

  //set_intake(12);
  Intake.spin(reverse, 12, volt);

  startDrivetrain(4.25);

  launch_task([&] {topFilter(5000);});

  wait(1500, msec);

  stopDrivetrain(coast);

  Linear.setMaxVoltages(12);

  //Robot.DriveFor(-8, L_Settings, false, 650);

  //Robot.TurnFor(65, A60_Settings, 800);

  printCoordinates();

  //Robot.DriveFor(8, L_Settings, false, 200);
  
  Robot.DriveToPoint(-30 , -17.5, L_Settings, A0_Settings, 0, true);

  Robot.TurnFor(154, A0_Settings, 700);

  startDrivetrain(5);
  wait(800, msec);
  stopDrivetrain(coast);
  Robot.DriveFor(-7.5, L_Settings, false, 500);

  ladyBrown.spin(forward, 12, volt);
  wait(500, msec);
  ladyBrown.stop(coast);
 
  Linear.setMaxVoltages(10.5);
  Robot.DriveToPoint(-20, 28, L_Settings, A0_Settings, 0, true);

  // launch_task([&] {
  //   lBPid(0);
  //   ladyBrown1.setPosition(68, deg);
  //   ladyBrown2.setPosition(68, deg);
  //   ladyBrown.setPosition(68, deg);
  // });

  Clamp.open();

  wait(200, msec);

  Intake.spin(reverse, 12, volt);

  printCoordinates(false);

  lBPid(270, 800);

  Robot.TurnFor(-35, A60_Settings, 500);

  Clamp.close();

  Linear.setMaxVoltages(8);

  Robot.DriveFor(3.5, L_Settings, 800, 0, true);

  ladyBrown.spin(forward, 4.5, volt);
}

void goalRushBlue(bool elims) {

  pneumatics Clamp(Brain.ThreeWirePort.A);
  pneumatics Doinker(Brain.ThreeWirePort.B);
  pneumatics goalRush(Brain.ThreeWirePort.C);

  std::cout << "GOAL RUSH BLUE" << std::endl;

  ladyBrown1.setPosition(60, deg);
  ladyBrown2.setPosition(60, deg);
  ladyBrown.setPosition(60, deg);

  //RingFilter.objectDetected(filterRedRings);

  launch_task([&] {
    launch_task([&] {wait(200, msec); topFilter(5000);});
    lBPid(258);
  });

  launch_task([&] {
  waitUntil(GoalDetector.objectRawSize() == -1 && GoalDetector.objectDistance(inches) < 1.5);
  goalRush.open();
  std::cout << "clamp" << std::endl;
  });

  Intake.spin(reverse, 12, volt);

  Doinker.open();

  //Linear.setMaxVoltages(11);
  Robot.DriveToPoint(-1.5, 38.5, L_Settings, A60_Settings);

  printCoordinates();

  Linear.setMaxVoltages(12);

  Robot.DriveToPoint(0, 13, L_Settings, A0_Settings, 1500, true, true);

  // wait(200, msec);
  // Intake.stop();
  wait(800, msec);;
  goalRush.close();
  wait(100, msec);
  Robot.DriveFor(-4, L_Settings, false, 300);
  Doinker.close();
  Robot.waitChassis();

  wait(100, msec);

  Robot.TurnFor(170, A120_Settings, 800); // -10 offset

  Linear.setMaxVoltages(9);

  Robot.DriveToPoint(-7, 39, L_Settings, A0_Settings, 2000, true);
  Linear.setMaxVoltages(12);

  Clamp.open();

  wait(200, msec);
  
  Intake.spin(reverse, 12, volt);

  //Linear.setMaxVoltages(9.5);

  Robot.DriveToPoint(3, 0, L_Settings, A0_Settings, 0, false, true);

  wait(800, msec);

  Clamp.close();

  Robot.waitChassis();

  //launch_task([&] {lBPid(72.5);});//63.5 84

  printCoordinates();

  Robot.TurnFor(Robot.getAbsoluteHeading() + 25, A0_Settings, 800);
  //wait(1000, sec);
 // Robot.DriveFor(-8, L_Settings, false, 650);

  //wait(150, msec);

  //set_intake(12);

  //wait(250, msec);

  Intake.spin(reverse, 12, volt);

  startDrivetrain(4.25);

  launch_task([&] {topFilter(5000);});

  wait(1000, msec);

  stopDrivetrain(coast);

  Linear.setMaxVoltages(12);

  //Robot.DriveFor(-8, L_Settings, false, 650);

  //Robot.TurnFor(65, A60_Settings, 800);

  Linear.setMaxVoltages(12);

  printCoordinates();

  //Robot.DriveFor(8, L_Settings, false, 200);

  Robot.DriveToPoint(54, 23, L_Settings, A0_Settings, 0, true);

  Robot.TurnFor(160, A0_Settings, 800);

  startDrivetrain(5);
  wait(800, msec);
  stopDrivetrain(coast);
  Robot.DriveFor(-7.5, L_Settings, false, 500);

  ladyBrown.spin(forward, 12, volt);
  wait(500, msec);
  ladyBrown.stop(coast);
 
  Linear.setMaxVoltages(10.5);
  Robot.DriveToPoint(16.75, 41.5, L_Settings, A0_Settings, 1400, true);

  Clamp.open();

  wait(200, msec);

  Intake.spin(reverse, 12, volt);

  printCoordinates(false);

  lBPid(270, 800);

  Robot.TurnFor(35, A60_Settings, 500);

  Clamp.close();

  Linear.setMaxVoltages(8);

  Robot.DriveFor(3, L_Settings, 800, 0, true);

  ladyBrown.spin(forward, 7, volt);

}

void negSideBlue(bool elims) {
  // hey incase im not here, the setup is just parallel to the line tape 
  // also dont touch skills unless ur a sigma
  pneumatics Clamp(Brain.ThreeWirePort.A);
  pneumatics Doinker(Brain.ThreeWirePort.B);
  pneumatics goalRush(Brain.ThreeWirePort.C);
  pneumatics Pistake(Brain.ThreeWirePort.H);

  std::cout << "NEG SIDE BLUE" << std::endl;

  Robot.setRobotCoordinates({0, 0, 31});
  RingFilter.objectDetected(filterRedRings);

  //wait(100, sec);

  Robot.DriveFor(6, L_Settings, 600);

  ladyBrown.spin(forward, 12, volt);
  wait(400, msec);
  ladyBrown.stop(hold);

  Linear.setMaxVoltages(9);
  launch_task([&] {
    wait(500, msec); 
    lBPid(-50, 3000); 
    ladyBrown1.setPosition(0, deg);
    ladyBrown2.setPosition(0, deg);
    ladyBrown.setPosition(0, deg);
  });
  Robot.DriveToPoint(-15, -28.5, L_Settings, A0_Settings, 0, true);

  Clamp.open();

  wait(200, msec);

  Robot.TurnFor(-125, A120_Settings, 500);

  Intake.spin(reverse, 12, volt);

  Linear.setMaxVoltages(12);

  Robot.DriveToPoint(-35, -39.5, L_Settings, A0_Settings);

  wait(200, msec);

  printf("hello \n");
  printCoordinates();

  Robot.DriveToPoint(-11, -21, L_Settings, A0_Settings, 0, true);

  Linear.setMaxVoltages(10.5);

  printCoordinates();

  Robot.TurnFor(-90, A0_Settings, 500);

  Robot.DriveToPoint(-39, -21, L_Settings, A0_Settings);

  //wait(200, sec);

  Robot.TurnFor(180, A60_Settings, 500);

  Robot.DriveToPoint(-46, -39, L_Settings, A0_Settings);

  wait(200, msec);

  printCoordinates();

  Linear.setMaxVoltages(11);

  Robot.DriveToPoint(-43, 4, L_Settings, A0_Settings, 0, true);

  Linear.setMaxVoltages(12);

  Robot.TurnFor(-50, A120_Settings, 800);

  //wait(200, sec);

  printCoordinates();

  startDrivetrain(5);

  wait(800, msec);

  stopDrivetrain(coast);

  Robot.DriveToPoint(-10, -3.5, L_Settings, A0_Settings, 0, true);

  Robot.TurnFor(85, A120_Settings, 800);

  Pistake.open();

  Robot.DriveFor(15, L_Settings, true, 800);

  Pistake.close();

  wait(100, msec);

  Robot.DriveFor(-3.5, L_Settings, true, 500);
  wait(200, msec);
  Intake.stop(coast);

  if (elims == true) return;

  Robot.TurnFor(165, A60_Settings, 800);

  Linear.setMaxVoltages(12);

  Robot.DriveFor(15.5, L_Settings, false, 0, true);

  ladyBrown.spin(forward, 4, volt);
}

void negSideRed(bool elims) {
  pneumatics Clamp(Brain.ThreeWirePort.A);
  pneumatics Doinker(Brain.ThreeWirePort.B);
  pneumatics goalRush(Brain.ThreeWirePort.C);
  pneumatics Pistake(Brain.ThreeWirePort.H);

  std::cout << "NEG SIDE RED" << std::endl;

  Robot.setRobotCoordinates({0, 0, -31});
 // RingFilter.objectDetected(filterBlueRings);

  //wait(100, sec);

  Robot.DriveFor(6.5, L_Settings, 600);

  ladyBrown.spin(forward, 12, volt);
  wait(400, msec);
  ladyBrown.stop(hold);

  Linear.setMaxVoltages(9);
  launch_task([&] {
    wait(500, msec); 
    lBPid(-50, 3000); 
    ladyBrown1.setPosition(0, deg);
    ladyBrown2.setPosition(0, deg);
    ladyBrown.setPosition(0, deg);
  });
  Robot.DriveToPoint(16, -29, L_Settings, A0_Settings, 0, true);

  Clamp.open();

  wait(200, msec);

  Robot.TurnFor(125, A120_Settings, 500);

  Intake.spin(reverse, 12, volt);

  Linear.setMaxVoltages(12);

  Robot.DriveToPoint(31, -42.5, L_Settings, A0_Settings);

  printf("hello \n");
  printCoordinates();

  Robot.DriveToPoint(9.5, -27, L_Settings, A0_Settings, 0, true);

  Linear.setMaxVoltages(11);

  Robot.TurnFor(90, A0_Settings, 500);

  printCoordinates();

  Robot.DriveToPoint(36, -27, L_Settings, A0_Settings);

  //wait(200, sec);

  Robot.TurnFor(165, A120_Settings, 850);

  Robot.DriveToPoint(42, -43.5, L_Settings, A0_Settings);

  printCoordinates();

  Linear.setMaxVoltages(11);

  Robot.DriveToPoint(44, 4, L_Settings, A0_Settings, 0, true);

  Linear.setMaxVoltages(12);

  Robot.TurnFor(40, A120_Settings, 800);

  //wait(200, sec);

  printCoordinates();

  startDrivetrain(6.5);

  wait(800, msec);

  stopDrivetrain(coast);

  Robot.DriveToPoint(10, -6.5, L_Settings, A0_Settings, 0, true);

  printCoordinates();

  Robot.waitChassis();

  Robot.TurnFor(-85, A120_Settings, 800);

  Pistake.open();

  Robot.DriveFor(14.5, L_Settings, true, 800);

  Pistake.close();

  Robot.DriveFor(-5, L_Settings, true, 800);
  wait(200, msec);
  Intake.stop(coast);

  if (elims == true) return;

  Robot.TurnFor(-160, A60_Settings, 800);

  Linear.setMaxVoltages(12);

  Robot.DriveFor(15.5, L_Settings, false, 0, true);

  ladyBrown.spin(forward, 6, volt);
}
// void safeSoloRedAWP(bool elims) {
//   pneumatics Clamp(Brain.ThreeWirePort.G);
//   pneumatics IntakeLift(Brain.ThreeWirePort.F);
//   pneumatics Extender(Brain.ThreeWirePort.E);

//   FishMech.spin(forward, 3, volt);

//   Robot.DriveFor(-28.5, L_Settings, true);

//   Clamp.open();

//   wait(150, msec);

//   Intake.spin(reverse, 12, volt);

//   wait(500, msec);

//   Robot.TurnFor(-90, A60_Settings, 800);

//   Robot.DriveFor(21.5, L_Settings, 1000);

//   Linear.setMaxVoltages(7.5);

//   Robot.DriveFor(-16, L_Settings);

//   Robot.TurnFor(0, A60_Settings, 800);

//   Linear.setMaxVoltages(12);

//   Robot.DriveFor(19.5, L_Settings);

//   Robot.TurnFor(90, A60_Settings, 800);

//   Clamp.close();

//   launch_task([&] {
//     waitUntil(RingFilter.isNearObject() == true && RingFilter.hue() < 15);
//     Intake.stop();
//   });

//   Linear.setMaxVoltages(8.5);

//   Robot.DriveFor(27, L_Settings);

//   Linear.setMaxVoltages(12);

//   Intake.spin(reverse, 10.5, volt);

//   Robot.DriveFor(24, L_Settings);

//   Robot.TurnFor(0, A60_Settings, 800);

//   Robot.DriveFor(-21, L_Settings);

//   Clamp.open();

//  // wait(150, msec);

//   Intake.spin(reverse, 11, volt);

// //  wait(150, msec);

//   Robot.TurnFor(90, A60_Settings, 800);

//   Robot.DriveFor(21, L_Settings);

//   Linear.setMaxVoltages(7.5);

//   Robot.DriveFor(-27.5, L_Settings);

//   launch_task([&] {
//     fishMechLoop(200);
//   });

//   Robot.TurnFor(-135, A120_Settings, 800);

//   FishMech.spin(reverse, 3, volt);

//   Robot.DriveFor(10, L_Settings);  
// }

// void safeSoloBlueAWP(bool elims) {
//   pneumatics Clamp(Brain.ThreeWirePort.G);
//   pneumatics IntakeLift(Brain.ThreeWirePort.F);
//   pneumatics Extender(Brain.ThreeWirePort.E);

//   Robot.DriveFor(-28.5, L_Settings, true);

//   Clamp.open();

//   wait(150, msec);

//   Intake.spin(reverse, 12, volt);

//   wait(500, msec);

//   Robot.TurnFor(90, A60_Settings, 800);

//   Robot.DriveFor(21.5, L_Settings);

//   Linear.setMaxVoltages(6);

//   Robot.DriveFor(-16, L_Settings);

//   Robot.TurnFor(0, A60_Settings, 800);

//   Linear.setMaxVoltages(12);

//   Robot.DriveFor(19.5, L_Settings);

//   Robot.TurnFor(-90, A60_Settings, 800);

//   Clamp.close();

//   launch_task([&] {
//     waitUntil(RingFilter.isNearObject() == true && RingFilter.hue() > 100);
//     Intake.stop();
//   });

//   Linear.setMaxVoltages(8.5);

//   Robot.DriveFor(24, L_Settings);

//   Linear.setMaxVoltages(12);

//   Intake.spin(reverse, 11, volt);

//   Robot.DriveFor(24, L_Settings);

//   Robot.TurnFor(0, A60_Settings, 800);

//   Robot.DriveFor(-24, L_Settings);

//   Clamp.open();

//  // wait(150, msec);

//   Intake.spin(reverse, 10.5, volt);

// //  wait(150, msec);

//   Robot.TurnFor(-90, A60_Settings, 800);

//   Robot.DriveFor(20, L_Settings);

//   Linear.setMaxVoltages(6.5);

//   Robot.DriveFor(-25, L_Settings);

//   launch_task([&] {
//     fishMechLoop(200);
//   });

//   Robot.TurnFor(135, A120_Settings, 800);

//   FishMech.spin(reverse, 3, volt);

//   Robot.DriveFor(10, L_Settings);
// }