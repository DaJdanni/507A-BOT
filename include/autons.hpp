#pragma once
#include "vex.h"

// Robot & PID Configuarations
extern Bucees::Robot Robot;
extern vex::brain Brain;
extern vex::motor Intake;
extern vex::motor FishMech;
extern vex::optical RingFilter;

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


// Auton Helpers:
void fishMechLoop(double desiredPosition = 214, vex::directionType dir = vex::directionType::fwd);
void printCoordinates(bool reversed = false);
void activateMotionChaining(bool reverse = false, float minSpeed = 5);
void deactivateMotionChaining(bool reversed = false);

// Autonomous Routines
void skills();
void disruptBlueRing(bool elims);
void disruptRedRing(bool elims);
void goalRushRed(bool elims);
void goalRushBlue(bool elims);
void safeSoloRedAWP(bool elims);
void safeSoloBlueAWP(bool elims);