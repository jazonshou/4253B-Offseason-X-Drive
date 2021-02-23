using namespace vex;

class Chassis {
  public:
    //DRIVE HELPER FUNCTIONS
    static void setVel(double LF, double LB, double RF, double RB);
    static void setVolt(double LF, double LB, double RF, double RB);
    static void setStop(brakeType type);

    //DRIVER CONTROL FUNCTIONS
    static void robotCentric();
    static void fieldCentric();
    static void opControl();

    //RESET
    static void resetEnc();
};