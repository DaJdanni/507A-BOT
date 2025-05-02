#pragma once
#include "vex.h"

// Robot & PID Configuarations
extern Bucees::Robot Robot;
extern vex::brain Brain;
extern vex::motor Intake;
extern vex::motor_group LeftSide;
extern vex::motor_group RightSide;
extern vex::motor ladyBrown;
extern vex::optical RingFilter;
extern vex::optical RingFilterBottom;
extern vex::distance GoalDetector;

extern Bucees::PIDSettings LOdom_Settings;
extern Bucees::PIDSettings L_Settings;
extern Bucees::PIDSettings LMogo_Settings;
extern Bucees::PIDSettings L_OdomSettings;
extern Bucees::PIDSettings A0_Settings;
extern Bucees::PIDSettings A60_Settings;
extern Bucees::PIDSettings A120_Settings;
extern Bucees::PIDSettings ANTIDRIFT_1TILE;
extern Bucees::PIDSettings FishSettings;

extern Bucees::FAPIDController Linear;
extern Bucees::FAPIDController Angular;

// global vars:
extern bool detectJams;


// Auton Helpers:
void fishMechLoop(double desiredPosition = 214, vex::directionType dir = vex::directionType::fwd);
void lBPid(double target, double defaultTimeout = 1250, double defaultSpeed = 12);
void printCoordinates(bool reversed = false);
void activateMotionChaining(bool reverse = false, float minSpeed = 5);
void deactivateMotionChaining(bool reversed = false);
void set_intake(int input);

// Autonomous Routines
void skills(bool elims);
void disruptBlueRing(bool elims);
void disruptRedRing(bool elims);
void goalRushRed(bool elims);
void goalRushBlue(bool elims);
void soloBlueAWP(bool elims);
void soloRedAWP(bool elims);
void safeSoloRedAWP(bool elims);
void safeSoloBlueAWP(bool elims);
void negSideBlue(bool elims);
void negSideRed(bool elims);
void testSkills(bool elims);
void doNothing(bool elims);