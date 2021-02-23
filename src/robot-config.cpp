#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor leftFront = motor(PORT19, ratio18_1, false);
motor leftBack = motor(PORT10, ratio18_1, false);
motor rightFront = motor(PORT11, ratio18_1, true);
motor rightBack = motor(PORT2, ratio18_1, true);
inertial imu = inertial(PORT12);
encoder encoderRight = encoder(Brain.ThreeWirePort.A);
encoder encoderLeft = encoder(Brain.ThreeWirePort.C);
encoder encoderBack = encoder(Brain.ThreeWirePort.E);
sonar ultrasonic = sonar(Brain.ThreeWirePort.G);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}