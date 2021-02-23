#include "vex.h"

using namespace vex;

PID::PID(double _kP, double _kD, double _turn_kP, double _turn_kD){
  //Movement PID constants
  kP = _kP; kD = _kD; 
  //Movement PID
  derivative_x = 0.0; derivative_y = 0.0; prevError_x = 0.0; prevError_y = 0.0;
  //Turn PID constants
  turn_kP = _turn_kP; turn_kD = _turn_kD; 
  //Turn PID
  turn_derivative = 0.0; turn_prevError = 0.0;
  //getError function - for debugging
  dist = 0.0; turn_e = 0.0;
}

void PID::moveTo(double x, double y, double finalAngle){
  double startAngle = Odom::globalPoint.angle;
  while(true){
    //Calculates distance and angle of the current point to the target point
    double distanceToTarget = hypot(x - Odom::globalPoint.x, y - Odom::globalPoint.y);
    dist = distanceToTarget; //for multitasking
    double angleToTarget = atan2(y - Odom::globalPoint.y, x - Odom::globalPoint.x);
    double relativeAngleToTarget = Math::angleWrap(angleToTarget + Odom::globalPoint.angle); // - ?

    //Calculates the x and y components of the vector
    double relativeXToTarget = (cos(relativeAngleToTarget) * distanceToTarget);
    double relativeYToTarget = (sin(relativeAngleToTarget) * distanceToTarget);

    //Movement PID
    derivative_x = relativeAngleToTarget - prevError_x;
    prevError_x = relativeAngleToTarget;
    derivative_y = relativeAngleToTarget - prevError_y;
    prevError_y = relativeAngleToTarget;

    double movementXPower = relativeXToTarget * kP + derivative_x * kD;
    double movementYPower = relativeYToTarget * kP + derivative_x * kD;

    //Calculates turn angle
    double heading2 = finalAngle < 0 ? finalAngle + M_PI*2 : finalAngle - M_PI*2;
    finalAngle = (fabs(Odom::globalPoint.angle) - finalAngle < 
                  fabs(Odom::globalPoint.angle) - heading2) ? finalAngle : heading2;
    Brain.Screen.printAt(1, 220, "turn amount: %f", heading2);

    //Turn PID
    double turnError = -(finalAngle - Math::compressAngle(startAngle, Odom::globalPoint.angle)); turn_e = turnError;
    turn_derivative = turnError - turn_prevError;
    turn_prevError = turnError;
    double turnPower = turnError * turn_kP + turn_derivative * turn_kD;

    Brain.Screen.printAt(1, 200, "Turn Error: %f", turnError);

    if(fabs(turnError) < Math::getRadians(2) && fabs(relativeXToTarget) < 0.5 && fabs(relativeYToTarget) < 0.5) break;

    Chassis::setVel(movementYPower + movementXPower - turnPower, movementYPower - movementXPower - turnPower,
                    movementYPower - movementXPower + turnPower, movementYPower + movementXPower + turnPower);
    
    wait(10, msec);
  }
  Chassis::setVel(0, 0, 0, 0);
}

void PID::reset(){
  //Movement PID
  derivative_x = 0.0; derivative_y = 0.0; prevError_x = 0.0; prevError_y = 0.0;
  //Turn PID
  turn_derivative = 0.0; turn_prevError = 0.0;
  //Reset function - for debugging
  dist = 0.0; turn_e = 0.0;
}

double PID::getError(PIDtype type){
  if(type == PIDtype::movement){
    return dist;
  } else {
    return turn_e;
  }
}