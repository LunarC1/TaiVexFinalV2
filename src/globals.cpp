#include "main.h"


vex::brain brain;
 
vex::controller controller = vex::controller(vex::primary);

// vex::motor catapult = vex::motor(vex::PORT20, false);
// vex::motor catapult2 = vex::motor(vex::PORT17, true);
vex::motor intake = vex::motor(vex::PORT5, vex::ratio6_1, true);
vex::motor intake2 = vex::motor(vex::PORT7, vex::ratio6_1, false);

vex::motor fl = vex::motor(vex::PORT8, vex::ratio6_1, true);
vex::motor ml = vex::motor(vex::PORT9, vex::ratio6_1, true);
vex::motor bl = vex::motor(vex::PORT10, vex::ratio6_1, false);

vex::motor fr = vex::motor(vex::PORT2, vex::ratio6_1, true);
vex::motor mr = vex::motor(vex::PORT3, vex::ratio6_1, false);
vex::motor br = vex::motor(vex::PORT4, vex::ratio6_1, false);

// vex::motor fl = vex::motor(vex::PORT8, vex::ratio6_1, false);
// vex::motor ml = vex::motor(vex::PORT9, vex::ratio6_1, false);
// vex::motor bl = vex::motor(vex::PORT10, vex::ratio6_1, false);

// vex::motor fr = vex::motor(vex::PORT2, vex::ratio6_1, true);
// vex::motor mr = vex::motor(vex::PORT3, vex::ratio6_1, true);
// vex::motor br = vex::motor(vex::PORT4, vex::ratio6_1, true);

vex::motor_group leftDrivetrain = vex::motor_group(fl, ml, bl);
vex::motor_group rightDrivetrain = vex::motor_group(fr, mr, br);
vex::motor_group drivetrain = vex::motor_group(fl, fr, ml, mr, bl, br);

// vex::digital_out backRightWing = vex::digital_out(brain.ThreeWirePort.B);
// vex::digital_out frontRightWing = vex::digital_out(brain.ThreeWirePort.A);
// vex::digital_out hang = vex::digital_out(brain.ThreeWirePort.E);
// vex::digital_out pto = vex::digital_out(brain.ThreeWirePort.F);
// vex::digital_out frontLeftWing = vex::digital_out(brain.ThreeWirePort.C);
// vex::digital_out backLeftWing = vex::digital_out(brain.ThreeWirePort.G);

vex::digital_out pisstake = vex::digital_out(brain.ThreeWirePort.D);
vex::digital_out lift = vex::digital_out(brain.ThreeWirePort.E);
vex::digital_out mogo = vex::digital_out(brain.ThreeWirePort.F);
vex::digital_out redirect = vex::digital_out(brain.ThreeWirePort.G);
vex::digital_out stick = vex::digital_out(brain.ThreeWirePort.H);

vex::inertial imu = vex::inertial(vex::PORT20);

vex::rotation parallel = vex::rotation(vex::PORT12, false);
vex::rotation sideways = vex::rotation(vex::PORT11, false);
// vex::rotation sideways = vex::rotation(vex::PORT20, false);
// vex::rotation cataRotation = vex::rotation(vex::PORT19, false);