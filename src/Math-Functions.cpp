#include "vex.h"

using namespace vex;

//CONVERSIONS (DISTANCE)
double Math::degToInch(double deg){
  return (deg / 360) * (M_PI * 2.75);
}
double Math::inchToDeg(double inch){
  return (inch / (M_PI * 2.75)) * 360;
}

//CONVERSIONS (ANGLE)
double Math::getRadians(double deg){
  return (deg * M_PI) / 180;
}
double Math::getDeg(double rad){
  return rad * (180/M_PI);
}

//HELPER FUNCTIONS
double Math::getHeading(double angle){
  while(!(angle >= 0 && angle < M_PI * 2)){
    if(angle < 0) angle += M_PI*2;
    if(angle >= M_PI * 2) angle -= M_PI * 2;
  }
  return angle;
}

double Math::angleWrap(double angle){
  while(angle < -M_PI){
    angle += 2 * M_PI;
  }
  while(angle > M_PI){
    angle -= 2 * M_PI;
  }
  return angle;
}

double Math::compressAngle(double startAngle, double angle){
  while(angle <= startAngle - M_PI*2){
    angle += M_PI*2;
  }
  while(angle >= startAngle + M_PI*2){
    angle -= M_PI*2;
  }
  return angle;
}

double Math::clip(double number, double min, double max){
  while(!(number >= min && number <= max)){
    if(number < min){
      number = min;
    }
    if(number > max){
      number = max;
    }
  }
  return number;
}

int Math::optimalTurnSide(double currentA, double targetA){
  double diff = targetA - currentA;
  if(diff < 0){
    diff += M_PI*2;
  }
  if(diff > M_PI){
    return 1;
  } else {
    return -1;
  }
}

//GEOMETRY FUNCTIONS
double Math::dist(graphPoint point1, graphPoint point2){
  return sqrt(pow(point2.x-point1.x, 2) + pow(point2.y-point1.y, 2));
}

bool Math::linePoint(graphPoint linePoint1, graphPoint linePoint2, graphPoint point) {
  // get distance from the point to the two ends of the line
  double d1 = dist({point.x, point.y}, {linePoint1.x,linePoint1.y});
  double d2 = dist({point.x, point.y}, {linePoint2.x,linePoint2.y});
  // get the length of the line
  double lineLen = dist({linePoint1.x,linePoint1.y}, {linePoint2.x,linePoint2.y});
  // since doubles are so minutely accurate, add
  // a little buffer zone that will give collision
  double buffer = 0.1;    // higher # = less accurate
  // if the two distances are equal to the line's 
  // length, the point is on the line!
  // note we use the buffer here to give a range, 
  // rather than one #
  if (d1+d2 >= lineLen-buffer && d1+d2 <= lineLen+buffer) {
    return true;
  }
  return false;
}

bool Math::pointCircle(graphPoint point, graphPoint circleCenter, double cr){
  if(dist({point.x, point.y}, {circleCenter.x, circleCenter.y}) < cr){
    return true;
  } else {
    return false;
  }
}

bool Math::lineCircle(graphPoint linePoint1, graphPoint linePoint2, graphPoint circleCenter, double r) {

  // is either end INSIDE the circle?
  // if so, return true immediately
  bool inside1 = pointCircle({linePoint1.x, linePoint1.y}, {circleCenter.x, circleCenter.y}, r);
  bool inside2 = pointCircle({linePoint2.x, linePoint2.y}, {circleCenter.x, circleCenter.y},r);
  if (inside1 || inside2) return true;

  // get length of the line
  double distX = linePoint1.x - linePoint2.x;
  double distY = linePoint1.y - linePoint2.y;
  double len = sqrt( (distX*distX) + (distY*distY) );

  // get dot product of the line and circle
  double dot = ( ((circleCenter.x-linePoint1.x)*(linePoint2.x-linePoint1.x)) + 
                 ((circleCenter.y-linePoint1.y)*(linePoint2.y-linePoint1.y)) ) / pow(len,2);

  // find the closest point on the line
  double closestX = linePoint1.x + (dot * (linePoint2.x-linePoint1.x));
  double closestY = linePoint1.y + (dot * (linePoint2.y-linePoint1.y));

  // is this point actually on the line segment?
  // if so keep going, but if not, return false
  bool onSegment = linePoint({linePoint1.x, linePoint1.y}, {linePoint2.x, linePoint2.y}, {closestX, closestY});
  if (!onSegment) return false;

  // get distance to closest point
  distX = closestX - circleCenter.x;
  distY = closestY - circleCenter.y;
  double distance = sqrt( (distX*distX) + (distY*distY) );

  if (distance <= r) {
    return true;
  }
  return false;
}

//PURE PURSUIT
/*std::vector<graphPoint> Math::lineCircleIntersection(graphPoint circleCenter, double radius, graphPoint linePoint1, graphPoint linePoint2){
  //Convert line to slope intercept form
  double slope = (linePoint2.y - linePoint1.y) / (linePoint2.x - linePoint1.x);
  double yIntercept = linePoint1.y - (slope * linePoint1.x); //might not need
  //Calculate the line perpendicular to the line being intersected which also crosses circle center
  double newSlope = -1 / slope;
}*/