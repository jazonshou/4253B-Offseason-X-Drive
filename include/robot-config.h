using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern motor leftFront;
extern motor leftBack;
extern motor rightFront;
extern motor rightBack;
extern inertial imu;
extern encoder encoderRight;
extern encoder encoderLeft;
extern encoder encoderBack;
extern sonar ultrasonic;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );