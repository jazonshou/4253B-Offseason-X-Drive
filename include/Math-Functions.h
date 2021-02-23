#include <vector>

using namespace vex;

struct graphPoint {
  double x, y;
};

class Math {
  public: 
    //CONVERSIONS (DISTANCE)
    static double degToInch(double deg);
    static double inchToDeg(double inch);

    //CONVERSIONS (ANGLE)
    static double getRadians(double deg);
    static double getDeg(double rad);

    //HELPER FUNCTIONS
    static double getHeading(double angle);
    static double angleWrap(double angle);
    static double compressAngle(double startAngle, double angle);
    static double clip(double number, double min, double max);
    static int optimalTurnSide(double currentA, double targetA);

    //GEOMETRY FUNCTIONS
    static double dist(graphPoint point1, graphPoint point2);
    static bool linePoint(graphPoint linePoint1, graphPoint linePoint2, graphPoint point);
    static bool pointCircle(graphPoint point, graphPoint circleCenter, double cr);
    static bool lineCircle(graphPoint linePoint1, graphPoint linePoint2, graphPoint circleCenter, double r);

    //PURE PURSUIT
    static std::vector<graphPoint> lineCircleIntersection(graphPoint circleCenter, double radius, 
                                                          graphPoint linePoint1, graphPoint linePoint2);
};