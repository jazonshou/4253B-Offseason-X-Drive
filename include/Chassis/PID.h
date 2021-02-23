using namespace vex;

enum class PIDtype{
  turn, 
  movement
};

class PID {
  private:
    double kP, kD, derivative_x, derivative_y, prevError_x, prevError_y, kF;
    double turn_kP, turn_kD, turn_derivative, turn_prevError, turn_kF; 
    double dist, turn_e;
  
  public:
    //Constructs PID constants
    PID(double _kP, double _kD, double _turn_kP, double _turn_kD);
    void moveTo(double x, double y, double finalAngle);
    void reset();
    double getError(PIDtype type);
};