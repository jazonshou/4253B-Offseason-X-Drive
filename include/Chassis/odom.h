using namespace vex;

struct Point {
  double x, y, angle;
};

struct encoderType{
  double left, right, back;
};

class Odom {
  public: 
    //CONSTANTS
    static constexpr double track = 13.5; //inches
    static constexpr double backEncOffset = 5.75; //inches
    static constexpr double wheelCircumference = 2.75; //inches

    //GLOBAL COORDINATES
    static Point globalPoint; //x, y, angle
    static Point prevGlobalPoint; 
    static Point globalDeltaPoint; //change in x, change in y, change in angle

    //LOCAL COORDINATES
    static Point localDeltaPoint; //change in x, change in y

    //SENSOR VALUES
    //encoder
    static encoderType encoderVal; //leftEnc, rightEnc, backEnc
    static encoderType prevEncoderVal; //prev leftEnc, rightEnc, backEnc
    static encoderType deltaEncoderVal; //change in leftEnc, rightEnc, backEnc
    //angle
    static double currentAngle;
    static double prevAngle;
    static double deltaAngle;

    //ODOMETRY FUNCTIONS
    static void updateSensors();
    static void updatePosition();
    static void reset();
    static void setPosition(double newX, double newY, double newAngle);

    //ODOMETRY THREAD
    static int Odometry();
};