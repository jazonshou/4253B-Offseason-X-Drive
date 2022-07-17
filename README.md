# 4253B Offseason X Drive Code

We experimented with advanced algorithms like holonomic inverse kinematics, odometry, and PID to achieve optimum performance with our X drive. 

## Code Highlights 

* [`/src/Chassis/odom.cpp`](src/Chassis/odom.cpp) - Global state positioning using nonlinear state estimator.

We used 3 tracking wheels (2 vertical, 1 horizontal), to measure distance, and 1 inertial measurement unit (IMU), to measure rotation. The sensor inputs give feedback that can be used to calculate the robot's position during autonomous. This tracking algorithm allows us to have incredibly accurate movements that can dynamically adapt to outside influence. 

See more regarding odometry [here](http://thepilons.ca/wp-content/uploads/2018/10/Tracking.pdf) (*Introduction to Position Tracking*, VRC Team 5225A)

* [`/src/Chassis/chassis.cpp`](src/Chassis/chassis.cpp) - Class which controls basic chassis movements. 

At the very core of the software, we use basic functions to individual control the motors on the X drive. 

```cpp
void Chassis::setVel(double LF, double LB, double RF, double RB){
  leftFront.spin(fwd, LF, rpm);
  leftBack.spin(fwd, LB, rpm);
  rightFront.spin(fwd, RF, rpm);
  rightBack.spin(fwd, RB, rpm);
}
```

For opcontrol, we have 2 different methods for different drive styles. The first style is the traditional X drive arcade, otherwise known as robot-centric drive. 

```cpp
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
```

Next, we have field-centric control. Instead of having the movements be relative to the robot, the movements are relative to the field. Thus, no matter the rotation of the chassis, when the joystick moves forward, the drive will always go forward. 

```cpp
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
```

* [`/src/Chassis/PID.cpp`](src/Chassis/PID.cpp) - PID class which controls autonomous movements. 

We wanted to be able to tell the robot to go to absolute coordinates around the field. With the help of our odometry algorithm, we used inverse kinematics to tell our PID controller how much power needs to be applied to each motor. As a result, programming autonomous movements are very simple

```cpp
// Moves the chassis to (12_in, 6_in, 45_deg)
pid_obj.moveTo(12, 6, 45); 
```

## Installation & Usage

This project uses [VEXCode Pro](https://www.vexrobotics.com/vexcode/pro-v5). If you do not have VEXCode, you must download the software first. 

1. Download the project
2. Follow the example in ``src/main.cpp``
3. Customize!

## HolonomicLib

We have decided to continue pursuing the X drive path. Check out a more advanced version of our code at [HolonomicLib](https://github.com/Yessir120/HolonomicLib)!

## License
[MIT](https://choosealicense.com/licenses/mit/)
