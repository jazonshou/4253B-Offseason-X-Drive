/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\24jasonz                                         */
/*    Created:      Mon Feb 08 2021                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// leftFront            motor         19              
// leftBack             motor         10              
// rightFront           motor         11              
// rightBack            motor         2               
// imu                  inertial      12              
// encoderRight         encoder       A, B            
// encoderLeft          encoder       C, D            
// encoderBack          encoder       E, F            
// ultrasonic           sonar         G, H            
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;

PID simpleMove(12, 7.5, 175, 100);

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  encoderBack.resetRotation();
  encoderLeft.resetRotation();
  encoderRight.resetRotation();
  imu.calibrate();
  wait(2000, msec);

  Chassis::setStop(hold);

  

  thread posTrack(Odom::Odometry);

  while(true){
    Chassis::opControl();
    if(Controller1.ButtonX.pressing()){
      simpleMove.moveTo(0, 0, 0);
    }
    wait(10, msec);
  }
}
