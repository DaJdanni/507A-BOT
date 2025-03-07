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


void filter(COLOR_SORTER sortColor) {
  // red ring hue = 15
  // blue ring hue = 210
  if (RingFilter.isNearObject() != true) return;
 // std::cout << "h" << std::endl;
  if (RingFilter.hue() < 15 && sortColor != FILTER_RED) return;
  //std::cout << "h1" << std::endl;
  if (RingFilter.hue() > 210 && sortColor != FILTER_BLUE) return;
  //std::cout << "h2" << std::endl;
  if (RingFilter.hue() > 20 && RingFilter.hue() < 200) return;
  std::cout << "hey" << std::endl;
  //rumble(".");
  wait(100, msec);
  Intake.stop(coast);
  wait(350, msec);
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

  std::cout << "SKILLS" << std::endl;

  Intake.setVelocity(100, pct);
  ladyBrown1.resetPosition();
  ladyBrown2.resetPosition();

  launch_task([&] {
    limitLadyBrown();
  });

  Intake.spin(reverse, 12, volt);

  wait(400, msec); // wait until finished scoring red alliance stake

  //-- WALL STAKE #1 SECTION --//

  Robot.DriveToPoint(0, 10, L_Settings, A0_Settings, 400);

  Intake.stop(coast);

  Robot.TurnFor(-105, A120_Settings, 400);

  Linear.setMaxVoltages(8.5);

  Robot.DriveToPoint(23.5, 15.5, L_Settings, A0_Settings, 1250, true);

  Clamp.open(); // grab goal

  wait(200, msec);

  Linear.setMaxVoltages(12);

  Intake.spin(reverse, 12, volt);

  Robot.TurnFor(0, A60_Settings, 700); // face rings

  activateMotionChaining(false, 5.5);

  Robot.DriveToPoint(23.5, 28, L_Settings, A0_Settings); // ring 1

  //Robot.DriveToPoint(27, 41, L_Settings, A0_Settings);

  deactivateMotionChaining(false);

  Robot.DriveToPoint(44, 68, L_Settings, A0_Settings); // slightly adjust

  Linear.setMaxVoltages(9);
  launch_task([&] {wait(0, msec); lBPid(76.5);});

  Robot.DriveToPoint(47, 86, L_Settings, A0_Settings); // ring 2

  Linear.setMaxVoltages(8.5);
  
  Robot.DriveToPoint(42, 52.5, L_Settings, A0_Settings, 0, true, false, hold); // GET INTO POSITION

  Intake.stop(coast);
  wait(200, msec);
  launch_task([&] {lBPid(190, 1000);});

  Robot.TurnFor(90, A60_Settings, 800); // face wall stake

  Robot.waitChassis();

  startDrivetrain(3.5);

  Intake.spin(reverse, 12, volt);

  wait(1400, msec);

  ladyBrown.spin(forward, 12, volt); // score wallstake

  wait(800, msec);

  ladyBrown.stop(coast);

  stopDrivetrain(coast);

  printCoordinates();

  launch_task([&] {lBPid(190);}); // reset lady brown

  //-- GOAL #1 SECTION --//

  Linear.setMaxVoltages(12);

  Robot.DriveToPoint(48, 53.5, L_Settings, A0_Settings, 0, true); // back up

  Robot.TurnFor(180, L_Settings, 800); // face the rings 
  
  Intake.spin(reverse, 12, volt);

  Linear.setMaxVoltages(7);

  Robot.DriveToPoint(48, -2.5, L_Settings, A0_Settings, 2750); // drive straight into rings 3,4,5

  Robot.TurnFor(42.5, L_Settings, 800); // adjust position to face 6th ring

  Robot.DriveToPoint(58, 13, L_Settings, A0_Settings, 1000); // grab it

  Robot.TurnFor(-15, A120_Settings, 800);

  Linear.setMaxVoltages(12);

  printCoordinates();

  Robot.DriveToPoint(70, -15, L_Settings, A60_Settings, 700, true); // go into the corner
  Intake.stop(coast);

  printCoordinates();

  Clamp.close();
  wait(200, msec);


  // GOAL #2 CORNER SECTION //

  activateMotionChaining(false, 6.5);
  Intake.spin(forward, 12, volt);

  Robot.DriveToPoint(53, 85, L_Settings, A0_Settings);
  launch_task([&] {lBPid(77.5);});

  deactivateMotionChaining();

  Intake.spin(reverse, 12, volt); // start intaking when we get close to the ring
  Linear.setMaxVoltages(5);

  Robot.DriveToPoint(54, 101.5, L_Settings, A0_Settings);  // grab ring

  wait(200, msec);

  Robot.TurnFor(110, A0_Settings, 500); // face the goal

  Linear.setMaxVoltages(9);

  Robot.DriveToPoint(21, 113, L_Settings, A0_Settings, 1750, true); // drive into the goal

  Clamp.open();

  wait(200, msec);

  Robot.TurnFor(85, A0_Settings, 800); // face corner to sweep

  Doinker.open();

  Linear.setMaxVoltages(12);

  Robot.DriveToPoint(60, 116.5, L_Settings, A0_Settings, 1600); // get into corner

  // start sweeping
  LeftSide.spin(fwd, 10, volt);
  RightSide.spin(reverse, 10, volt);

  waitUntil(Robot.getAbsoluteHeading() > 140);

  LeftSide.stop(coast);
  RightSide.stop(coast);

  Doinker.close();
  wait(350, msec);

  Robot.TurnFor(235, A0_Settings, 800); // adjust for better angle

  // Robot.TurnFor(100, A0_Settings, 1000);

  // Robot.TurnFor(250, A0_Settings, 1000);

  Intake.stop(coast);
  Doinker.close();
  Clamp.close();

  launch_task([&] {
    wait(200, msec);
    lBPid(140, 1250, 10);
  });
  Linear.setMaxVoltages(9);

  Robot.DriveToPoint(64.5, 120, L_Settings, A0_Settings, 650, true); // back into corner
  Intake.spin(forward, 12, volt);

  wait(200, msec);

  //-- ALLIANCE STAKE BLUE SECTION --//

  Robot.DriveToPoint(32.5, 101, L_Settings, A0_Settings); // drive towards goal

  Robot.TurnFor(90, L_Settings, 800); // turn around to grab it

  Intake.spin(forward, 12, volt);

  Linear.setMaxVoltages(8);

  Robot.DriveToPoint(-1.5, 101, L_Settings, A0_Settings, 0, true, false, hold); // get ready to clamp it

  Clamp.open();

  wait(200, msec);

  Linear.setMaxVoltages(12);

  Robot.TurnFor(0, A60_Settings, 800); // face wallstake

  Robot.waitChassis();

  startDrivetrain(4); // drive into wallstake

  wait(1250, msec);

  stopDrivetrain(coast);

  Robot.DriveFor(-7.5, L_Settings, 500); // position yourself
  
  // score
  ladyBrown.spin(forward, 12, volt);
  wait(700, msec);
  ladyBrown.stop(coast);  

  Intake.spin(reverse, 12, volt);

  //-- GOAL #2 SECTION --//

  Robot.DriveToPoint(0, 98.5, L_Settings, A0_Settings, 0, true); // backup

  launch_task([&] {lBPid(0, 1250);});
  Robot.TurnFor(120, A60_Settings, 800); // face ring

  Robot.DriveToPoint(30, 80, L_Settings, A0_Settings);

  Angular.setMaxVoltages(9);

  Robot.TurnFor(220, A120_Settings, 800);

  Intake.stop();

  Angular.setMaxVoltages(12);
  Linear.setMaxVoltages(10);

  Robot.DriveToPoint(0, 55, L_Settings, A0_Settings);

  launch_task([&] {wait(800, msec); Intake.spin(reverse, 12, volt);});
  Linear.setMaxVoltages(6.5);
  Robot.DriveToPoint(-23, 38, L_Settings, A0_Settings);

  Robot.DriveToPoint(-45, 20, L_Settings, A0_Settings);

  Robot.TurnFor(180, L_Settings, 800);

  Linear.setMaxVoltages(8);

  Robot.DriveToPoint(-45, -0.5, L_Settings, A0_Settings, 1000);

  wait(500, msec);

  Robot.TurnFor(-42.5, L_Settings, 800);

  Robot.DriveToPoint(-57, 15, L_Settings, A0_Settings, 1250);

  Robot.TurnFor(15, A120_Settings, 800);

  Clamp.close();

  Linear.setMaxVoltages(12);

  printCoordinates();

  Robot.DriveToPoint(-64, -5, L_Settings, A60_Settings, 700, true); // go into the corner
  Intake.stop(coast);

  printCoordinates();

  wait(200, msec);

  Linear.setMaxVoltages(10);

  Robot.DriveToPoint(-48, 35.5, L_Settings, A0_Settings, 0, false, true);

  wait(200, msec);
  launch_task([&] {lBPid(77.5);});

  Intake.spin(reverse, 12, volt);

  Robot.waitChassis();

  Robot.DriveToPoint(-43, 10, L_Settings, A0_Settings, 0, true);
  
  Robot.TurnFor(270, A120_Settings, 800);

  Linear.setMaxVoltages(8);

  Robot.DriveToPoint(-15.5, 14.5, L_Settings, A0_Settings, 0, true);

  Clamp.open();
  wait(200, msec);

  printCoordinates();

  Intake.stop(coast);
  wait(200, msec);

  Robot.DriveToPoint(-34.5, 60, L_Settings, A0_Settings, 0, false, false, hold);
  launch_task([&] {lBPid(185);});

  Robot.TurnFor(270, A120_Settings, 800);

  startDrivetrain(3.5);

  Intake.spin(reverse, 12, volt);

  wait(1400, msec);

  ladyBrown.spin(forward, 12, volt); // score wallstake

  wait(800, msec);

  ladyBrown.stop(coast);

  stopDrivetrain(coast);

  printCoordinates();

  launch_task([&] {lBPid(190);}); // reset lady brown

  //-- GOAL #3 SECTION --//

  Linear.setMaxVoltages(12);

  Robot.DriveToPoint(-41.5, 63, L_Settings, A0_Settings, 0, true); // back up

  Robot.TurnFor(0, A120_Settings, 800);

  Linear.setMaxVoltages(8);

  Intake.spin(reverse, 12, volt);

  Robot.DriveToPoint(-41.5, 119, L_Settings, A0_Settings, 1850); // get all rings

  Robot.TurnFor(125, A120_Settings, 700);
  Clamp.close();

  Linear.setMaxVoltages(12);

  Robot.DriveFor(-15, L_Settings, false, 800);

  Robot.DriveFor(15, L_Settings);

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

void disruptBlueRing(bool elims) {

  pneumatics Clamp(Brain.ThreeWirePort.A);
  pneumatics Doinker(Brain.ThreeWirePort.B);
  pneumatics goalRush(Brain.ThreeWirePort.C);

  redFilter = true;
  waitTimeFilter = 150;
 // RingFilter.objectDetected(filter);

  launch_task([&] {
    waitUntil(RingFilter.isNearObject() == true);
    Intake.stop();
    return;
  });

  //FishMech.spin(forward, 3, volt);

  activateMotionChaining(false, 7);

  Robot.DriveToPoint(0, 37.5, L_Settings, A0_Settings);

  Intake.spin(reverse, 10.5, volt);

  deactivateMotionChaining();

  Robot.DriveToPoint(4.5, 42.5, L_Settings, A0_Settings);

  printCoordinates();

  Robot.TurnFor(47.5, A0_Settings, 700);

  Linear.setMaxVoltages(10);

  Robot.DriveFor(-24, LOdom_Settings, true, 1250);

  Clamp.open();

 // Intake.spin(reverse, 12, volt);

  wait(200, msec);

  Intake.spin(reverse, 12, volt);

  Robot.TurnFor(90, A60_Settings, 500);

  printCoordinates();

  Robot.DriveToPoint(13.5, 29, L_Settings, A0_Settings);

  wait(400, msec);
  Robot.TurnFor(0, A0_Settings, 800);

  Intake.spin(reverse, 12, volt);

  Robot.DriveFor(18.5, L_Settings, false, 1000);

  wait(0.2, seconds);

  Robot.DriveFor(-17, L_Settings, true, 1000);

  Robot.TurnFor(-174, A120_Settings, 1000);

  Robot.DriveFor(25, L_Settings, false, 1500);

  printCoordinates();

  wait(0.75, seconds);

  if (elims != true) {
      Robot.TurnFor(-45, A120_Settings, 1000);

      Linear.setMaxVoltages(7);

      Robot.DriveFor(45, L_Settings, false);

      wait(100, seconds); // 0.35

  } else {

    Intake.stop();

    Robot.TurnFor(-87.5, A120_Settings, 800);

    Linear.setMaxVoltages(11);

    Robot.DriveFor(50, L_Settings, false); // get to other corner

    wait(500, sec);

    Robot.TurnFor(-90, A0_Settings, 200);

    wait(500, msec);

    Robot.DriveFor(40, L_Settings, false);

    wait(100, seconds); // 0.35
  }
}

void disruptRedRing(bool elims) {
  pneumatics Clamp(Brain.ThreeWirePort.G);
  pneumatics IntakeLift(Brain.ThreeWirePort.F);
  pneumatics Extender(Brain.ThreeWirePort.E);

  waitTimeFilter = 170;
  blueFilter = true;
  //RingFilter.objectDetected(filter);

       launch_task([&] {
        fishMechLoop(215);
      });

  launch_task([&] {
    waitUntil(RingFilter.isNearObject() == true);
    Intake.stop();
    return;
  });

  //Extender.open();

  Robot.DriveToPoint(0, 37.5, LOdom_Settings, A0_Settings);

  printCoordinates();

  //Linear.setMaxVoltages(10);

  Intake.spin(reverse, 10.5, volt);

  Robot.DriveToPoint(-4.15, 47, LOdom_Settings, A0_Settings, 1250);

  printCoordinates();

//   Extender.close();

  Robot.TurnFor(-48, A0_Settings, 700);

  Robot.DriveFor(-28.5, L_Settings, true, 1200);

  Clamp.open();

  wait(200, msec);

  Robot.TurnFor(-90, A60_Settings, 500);

  Intake.spin(reverse, 12, volt);

  printCoordinates();

  Linear.setMaxVoltages(12);

  Robot.DriveToPoint(-18, 29, LOdom_Settings, A0_Settings);

  wait(500, msec);

  Robot.TurnFor(-10.5, A0_Settings, 800);

  Intake.spin(reverse, 12, volt);

  Linear.setMaxVoltages(12);

  Robot.DriveFor(17.5, L_Settings, false, 1000);

  wait(0.2, seconds);

  Linear.setMaxVoltages(5);

  Robot.DriveFor(-16, L_Settings, true, 1000);

  Robot.TurnFor(178, A120_Settings, 1000);

  Linear.setMaxVoltages(12);

  Robot.DriveFor(25, L_Settings, false, 2000);

  printCoordinates();

  wait(0.75, seconds);

 // fishMechLoop(200);

  if (elims != true) {
    Robot.TurnFor(45, A120_Settings, 1000);

    //FishMech.stop();

    // launch_task([&] {
    //   fishMechLoop(215);
    // });

    Linear.setMaxVoltages(7);

    Robot.DriveFor(45, L_Settings, false);

    wait(100, seconds); // 0.35
  } else {

    Intake.stop();

    Robot.TurnFor(90, A120_Settings, 1000);

    Linear.setMaxVoltages(10);

    Robot.DriveFor(45, L_Settings, false); // get to other corner

    wait(500, sec);

    wait(1000, msec);

    Robot.DriveFor(30, L_Settings, false);

    wait(100, seconds); // 0.35
  }
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

  Robot.DriveToPoint(-1, 33.5, L_Settings, A60_Settings);

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

  Robot.TurnFor(125, A0_Settings, 800);
  Linear.setMaxVoltages(12);

  //wait(150, msec);

  //set_intake(12);
  Intake.spin(reverse, 12, volt);

  wait(150, msec);

  startDrivetrain(4.5);

  launch_task([&] {topFilter(5000);});

  wait(1500, msec);

  LeftSide.stop();
  RightSide.stop();

  Linear.setMaxVoltages(12);

  //Robot.DriveFor(-8, L_Settings, false, 650);

  //Robot.TurnFor(65, A60_Settings, 800);

  Linear.setMaxVoltages(12);

  printCoordinates();

  //Robot.DriveFor(8, L_Settings, false, 200);
  
  Robot.DriveToPoint(-34, -17.5, L_Settings, A0_Settings, 0, true);

  Robot.TurnFor(154, A0_Settings, 700);

  Robot.DriveFor(15, L_Settings, false, 900);
  wait(150, msec);
  Robot.DriveFor(-8, L_Settings, false, 700);

  ladyBrown.spin(forward, 12, volt);
  wait(400, msec);
  ladyBrown.stop(hold);
 
  Linear.setMaxVoltages(9.5);
  Robot.DriveToPoint(-22.5, 26.5, L_Settings, A0_Settings, 0, true);

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

}

void goalRushBlue(bool elims) {

  pneumatics Clamp(Brain.ThreeWirePort.A);
  pneumatics Doinker(Brain.ThreeWirePort.B);
  pneumatics goalRush(Brain.ThreeWirePort.C);

  std::cout << "GOAL RUSH BLUE" << std::endl;

  ladyBrown1.setPosition(0, deg);
  ladyBrown2.setPosition(0, deg);
  ladyBrown.setPosition(0, deg);

  // launch_task([&] {
  //   lBPid(67);
  //   wait(200, msec);
  //   lBPid(190);
  // });

  launch_task([&] {
  waitUntil(GoalDetector.objectRawSize() == -1 && GoalDetector.objectDistance(inches) < 1.5);
  goalRush.open();
  std::cout << "clamp" << std::endl;
  });

  detectJams = true;

  Intake.spin(reverse, 12, volt);

  launch_task([&] {topFilter(5000);});

  Doinker.open();

  //Linear.setMaxVoltages(11);
  Robot.DriveToPoint(-1.5, 3, L_Settings, A60_Settings);

  printCoordinates();

  Linear.setMaxVoltages(12);

  Robot.DriveToPoint(0, 13, L_Settings, A0_Settings, 1500, true, true);

  // wait(200, msec);
  // Intake.stop();
  wait(550, msec);;
  goalRush.close();
  wait(100, msec);
  Robot.DriveFor(-4, L_Settings, false, 300);
  Doinker.close();
  Robot.waitChassis();

  //wait(500, msec);

  Robot.TurnFor(170, A120_Settings, 800); // -10 offset

  Linear.setMaxVoltages(9);

  Robot.DriveToPoint(-7, 39, L_Settings, A0_Settings, 2000, true);
  Linear.setMaxVoltages(12);

  Clamp.open();

  wait(200, msec);
  
  Intake.spin(reverse, 12, volt);

  //Linear.setMaxVoltages(9.5);

  Robot.DriveToPoint(3, 3.5, L_Settings, A0_Settings);

  launch_task([&] {lBPid(72.5);});//63.5 84

  printCoordinates();

  Robot.TurnFor(Robot.getAbsoluteHeading() + 20, A0_Settings, 800);
  //wait(1000, sec);
 // Robot.DriveFor(-8, L_Settings, false, 650);

  //wait(150, msec);

  Clamp.close();

  //set_intake(12);
  Intake.spin(reverse, 12, volt);

  Robot.DriveFor(6.5, L_Settings, true, 750);

  wait(550, msec);
  Intake.stop(coast);
  wait(100, msec);
  lBPid(150, 400);
  //wait(250, msec);

  startDrivetrain(6);
  Intake.spin(reverse, 12, volt);

  launch_task([&] {topFilter(3000);});

  wait(800, msec);

  LeftSide.stop();
  RightSide.stop();

  Linear.setMaxVoltages(12);

  //Robot.DriveFor(-8, L_Settings, false, 650);

  //Robot.TurnFor(65, A60_Settings, 800);

  Linear.setMaxVoltages(12);

  printCoordinates();

  //Robot.DriveFor(8, L_Settings, false, 200);

  Robot.DriveToPoint(54.5, 23, L_Settings, A0_Settings, 0, true);

  Robot.TurnFor(160, A0_Settings, 800);

  Robot.DriveFor(15, L_Settings, false, 1000);
  wait(150, msec);
  Robot.DriveFor(-8, L_Settings, false, 500);

  ladyBrown.spin(forward, 12, volt);
  wait(400, msec);
  ladyBrown.stop(hold);
 
  Linear.setMaxVoltages(10.5);
  Robot.DriveToPoint(17.5, 44.5, L_Settings, A0_Settings, 1400, true);

  Clamp.open();

  wait(200, msec);

  Intake.spin(reverse, 12, volt);

  printCoordinates(false);

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

  Robot.DriveFor(5, L_Settings, 600);

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

  Robot.TurnFor(-139, A120_Settings, 500);

  Intake.spin(reverse, 12, volt);

  Linear.setMaxVoltages(12);

  Robot.DriveToPoint(-33.5, -40, L_Settings, A0_Settings);

  wait(200, msec);

  printf("hello \n");
  printCoordinates();

  Robot.DriveToPoint(-11, -21, L_Settings, A0_Settings, 0, true);

  Linear.setMaxVoltages(10.5);

  printCoordinates();

  Robot.TurnFor(-90, A0_Settings, 500);

  Robot.DriveToPoint(-39, -21, L_Settings, A0_Settings);

  //wait(200, sec);

  Robot.TurnFor(-200, A60_Settings, 500);

  Robot.DriveToPoint(-43, -37, L_Settings, A0_Settings);

  wait(200, msec);

  printCoordinates();

  Linear.setMaxVoltages(11);

  Robot.DriveToPoint(-43, 4, L_Settings, A0_Settings, 0, true);

  Linear.setMaxVoltages(12);

  Robot.TurnFor(277, A120_Settings, 800);

  //wait(200, sec);

  printCoordinates();

  startDrivetrain(5);

  wait(800, msec);

  stopDrivetrain(coast);

  Robot.DriveToPoint(-10, -3.5, L_Settings, A0_Settings, 0, true);

  Robot.TurnFor(60, A120_Settings, 800);

  Pistake.open();

  Robot.DriveFor(15, L_Settings, true, 800);

  Pistake.close();

  wait(100, msec);

  Robot.DriveFor(-3.5, L_Settings, true, 500);

  Robot.TurnFor(130, A60_Settings, 800);

  Linear.setMaxVoltages(8);

  Intake.stop(coast);

  Robot.DriveFor(30, L_Settings);
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

  Robot.DriveToPoint(28.5, -42.5, L_Settings, A0_Settings);

  printf("hello \n");
  printCoordinates();

  Robot.DriveToPoint(9.5, -27, L_Settings, A0_Settings, 0, true);

  Linear.setMaxVoltages(11);

  Robot.TurnFor(90, A0_Settings, 500);

  printCoordinates();

  Robot.DriveToPoint(36, -27, L_Settings, A0_Settings);

  //wait(200, sec);

  Robot.TurnFor(195, A120_Settings, 850);

  Robot.DriveToPoint(36, -41, L_Settings, A0_Settings);

  printCoordinates();

  Linear.setMaxVoltages(11);

  Robot.DriveToPoint(44, 4, L_Settings, A0_Settings, 0, true);

  Linear.setMaxVoltages(12);

  Robot.TurnFor(45, A120_Settings, 800);

  //wait(200, sec);

  printCoordinates();

  startDrivetrain(6.5);

  wait(800, msec);

  stopDrivetrain(coast);
  
  Robot.DriveToPoint(10, -6.5, L_Settings, A0_Settings, 0, true);

  printCoordinates();

  Robot.waitChassis();

  Robot.TurnFor(-57.5, A120_Settings, 800);

  Pistake.open();

  Robot.DriveFor(14.5, L_Settings, true, 800);

  Pistake.close();

  Robot.DriveFor(-5, L_Settings, true, 800);
  wait(200, msec);
  Intake.stop(coast);

  if (elims == true) return;

  Robot.TurnFor(-130, A60_Settings, 800);

  Linear.setMaxVoltages(8);

  Robot.DriveFor(30, L_Settings);
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