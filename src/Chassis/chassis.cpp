#include "vex.h"

using namespace vex;

//DRIVE HELPER FUNCTIONS
void Chassis::setVel(double LF, double LB, double RF, double RB){
  leftFront.spin(fwd, LF, rpm);
  leftBack.spin(fwd, LB, rpm);
  rightFront.spin(fwd, RF, rpm);
  rightBack.spin(fwd, RB, rpm);
}

void Chassis::setVolt(double LF, double LB, double RF, double RB){
  leftFront.spin(fwd, LF, volt);
  leftBack.spin(fwd, LB, volt);
  rightFront.spin(fwd, RF, volt);
  rightBack.spin(fwd, RB, volt);
}

void Chassis::setStop(brakeType type){
  leftFront.setStopping(type);
  leftBack.setStopping(type);
  rightFront.setStopping(type);
  rightBack.setStopping(type);
}

//DRIVER CONTROL FUNCTIONS
void Chassis::robotCentric(){
  int controllerX = Controller1.Axis4.position() * 2;
  int controllerY = Controller1.Axis3.position() * 2;
  int turning = Controller1.Axis1.position() * 2;

  //Deadzone
  if(abs(Controller1.Axis4.position()) < 5) {controllerX = 0;}
  if(abs(Controller1.Axis3.position()) < 5) {controllerY = 0;}
  if(abs(Controller1.Axis1.position()) < 5) {turning = 0;}

  int leftF = controllerY + controllerX + turning;
  int leftB = controllerY - controllerX + turning;
  int rightF = controllerY - controllerX - turning;
  int rightB = controllerY + controllerX - turning;

  setVel(leftF, leftB, rightF, rightB);
}

void Chassis::fieldCentric(){
  //Controller variables
  int controllerX = Controller1.Axis4.position() * 2;
  int controllerY = Controller1.Axis3.position() * 2;
  int turning = Controller1.Axis1.position() * 2;
  
  //Deadzone
  if(abs(Controller1.Axis4.position()) < 5) {controllerX = 0;}
  if(abs(Controller1.Axis3.position()) < 5) {controllerY = 0;}
  if(abs(Controller1.Axis1.position()) < 5) {turning = 0;}

  //Quicc maths
  double magnitude = sqrt(pow(controllerX, 2) + pow(controllerY, 2));
  double theta = atan2(controllerY, controllerX);
  double theta2 = theta + Odom::globalPoint.angle;
  double x2 = magnitude * cos(theta2);
  double y2 = magnitude * sin(theta2);

  //Imu reset
  if(Controller1.ButtonA.pressing()) {imu.resetRotation();}

  //Motor output
  double leftF = y2 + x2 + turning;
  double leftB = y2 - x2 + turning;
  double rightF = y2 - x2 - turning;
  double rightB = y2 + x2 - turning;
  setVel(leftF, leftB, rightF, rightB);
}

static int cnt = 0;
static int preVal = 0;
void Chassis::opControl(){
  int val = Controller1.ButtonB.pressing();
  if(val == 1 && preVal == 0){
    cnt++;
  }
  preVal = val;
  if(cnt % 2 == 1){
    fieldCentric();
    Controller1.Screen.setCursor(2, 1);
    Controller1.Screen.clearLine(2);
    Controller1.Screen.print("Field Centric");
  } else {
    robotCentric();
    Controller1.Screen.setCursor(2, 1);
    Controller1.Screen.clearLine(2);
    Controller1.Screen.print("Robot Centric");
  }
}

//RESET
void Chassis::resetEnc(){
  encoderLeft.resetRotation(); encoderRight.resetRotation(); encoderBack.resetRotation();
}