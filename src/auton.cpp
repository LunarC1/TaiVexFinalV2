#include "auton.h"


Odometry odometry(0, 0, 1.1);

TrackingWheel parallelWheel(parallel, 2);
TrackingWheel perpendicularWheel(sideways, 2);
Inertial inertial(imu, 1.0);
OdometrySensors odomSensors(&parallelWheel, &perpendicularWheel, &inertial); 
OdometryOffsets odomOffsets(0, 0);
// TwoWheelInertialOdometry odome(odomSensors, odomOffsets, foot);
TwoWheelInertialOdometry odome(odomSensors, odomOffsets, foot);

AsymptoticGains lateralKp = AsymptoticGains(16500, 16500, 1, 1);
AsymptoticGains angularKp = AsymptoticGains(480, 220, 28, 1.7);
AsymptoticGains correctKp = AsymptoticGains(200, 200, 1, 1);

// PID lateralPID = PID(lateralKp, 25000, 150, 0.5, true);
// PID angularPID = PID(angularKp, 2000, 12, 5, true);
// PID correctPID = PID(correctKp, 0, 40, 0, false);

PID lateralPID = PID(lateralKp, 27000, 800, 0.5, true);
PID angularPID = PID(angularKp, 2000, 12, 5, true);
PID correctPID = PID(correctKp, 0, 40, 0, false);

Motion motion(
  lateralPID,
  angularPID,
  correctPID,
  odometry);
vex::task odom(nullptr);

// MotorGroup slapperMotors({
//   Motor("cata", &catapult),
//   Motor("cata2", &catapult2)
// });

MotorGroup intakeMotors({
  Motor("intake1", &intake),
  Motor("intake2", &intake2)
});

// PistonGroup fWings({
//   Piston("left", &frontLeftWing),
//   Piston("right", &frontRightWing)
// });


// AsymptoticGains cataKp = AsymptoticGains(100, 100, 1, 1);
// PID cataPID = PID(cataKp, 0, 0, 0, 0);
// Cata slapper(&slapperMotors, &cataRotation, &cataPID);


int updateOdom() {

  //int count = 0;
  
  while (1) {
    uint64_t timestamp = vex::timer::systemHighResolution();

    odometry.tick();
    
    //if (count % 10 == 0) printf("%f\n", odometry.getPosX());
    //count++;
  
    wait((timestamp + 11000.0 - vex::timer::systemHighResolution()) / 1000.0, vex::msec);
  }

  return 0;
}


// int turnToGoal() {

//   while (1) {
//     uint64_t timestamp = vex::timer::systemHighResolution();

//     motion.turnPointOneIter(6, 4.15, 2, 0, 1);
  
//     wait((timestamp + 11000.0 - vex::timer::systemHighResolution()) / 1000.0, vex::msec);
//   }
// }


// int cataTask() {

//   while (1) {
//     uint64_t timestamp = vex::timer::systemHighResolution();

//     slapper.reset();
  
//     wait((timestamp + 11000.0 - vex::timer::systemHighResolution()) / 1000.0, vex::msec);
//   }
// }


void taskHandler(bool driverMode) {
  if (!driverMode) odom = vex::task(updateOdom);
  else odom.stop();
}


// void outtakeFinish() { 
//   while (fabs(intake.velocity(vex::rpm)) < 600) {
//     intakeMotors.spin(-12);
//     wait(10, vex::msec);
//   }
// }


void initialize() {
  //resets the readings of all sensors and updates the drivetrain mode

  imu.resetRotation();
  // parallel.resetPosition();
  sideways.resetPosition();

  drivetrain.setStopping(vex::brake);

  motion.setUnit(foot);
  // motion.setUnit(inch);
  motion.setPursuitSettings(1, 1, 0.2, 0.4, 0, 5);
  odometry.setUnit(foot);
  // odometry.setUnit(inch);

  odometry.setPos(0, 0);
  taskHandler(false);

  // slapper.setResetSetpoint(25);
  // slapper.setPower(0.78);
  // slapper.setShotCounterBounds(15, 40);
}



// void leftAWP() {
//     imu.setRotation(-45, vex::deg);
//     intakeMotors.spin(-12);
//      motion.turnHeading(-45, 0, 0, 0, 0);
  // motion.moveDist(0.6, 1, 1500);
//     backLeftWing = true;
//     wait(500, vex::msec);
//   motion.turnHeading(-90, 2, 0, 1 , 1000);
//     backLeftWing = false;
//   motion.turnHeading(135, 2, 0, 1, 1500);
//   motion.moveDist(1.2, 1, 1000);
//   motion.turnHeading(90, 2, 0, 1, 1000);
//   motion.movePose(3, -0.25, 90, 0, 0, 0, 0.5, 5000);
//   motion.turnHeading(90, 2, 0, 1, 1000);
// }


// void leftElims() {
//     intakeMotors.spin(12);
//     //frontLeftWing = true;
  // motion.movePose(0.4, 3.3, 0, 1, 0, 0, 1, 50, -1, false, false);
//     //frontLeftWing = false;
   //motion.movePose(0.4, 3.3, 0, 1, 0, 0, 1, 1000, -1, true, true);
//   motion.movePose(0.4, 3.3, 90, 0, 0, -0.7, -1, 1500);
//   motion.movePose(0.6, 3.3, 90, 0, 0, 1, 1, 1000, -1, false);
//     intakeMotors.spin(-12);

//   motion.push(1.2, false, 1, 1500);

//   motion.moveDist(-1000, 1, 100);
//     intakeMotors.spin(12);
//   motion.turnHeading(60, 0, 1, 1, 600);
//   motion.movePose(1.85, 3.75, 40, 0, 0, 0, 1, 1000);
//   motion.movePose(0.6, 3.3, 90, 0, 0, -0.7, -1, 1500);
//     intakeMotors.spin(-12);
//   motion.movePose(0.8, 3.3, 90, 0, 0, 1, 1, 1000, -1, false);

//   motion.push(1.2, false, 1, 1500);
// }


// void topMidR6() {
//     intakeMotors.spin(12);
//     frontRightWing = true;
//   motion.movePose(-0.7, 3.85, -13, 0, 0, 0.1, 1, 50, -1, false, false);
//     frontRightWing = false;
//   motion.movePose(-0.7, 3.85, -13, 0, 0, 0.1, 1, 1000, -1, true, true);
//   motion.movePoint(0.12, 0.3, -0.35, -1, 2000, true, false);
//     intakeMotors.spin(5);
//   motion.turnHeading(20, 2, 1, 1, 300);
    // intakeMotors.spin(-8);
//   motion.turnHeading(38, 2, 0.1, 1, 500);
//     outtakeFinish();
//   motion.movePose(-1.9, -0.45, -90, 1.2, 0, 0, 1, 200);
//     intakeMotors.spin(12);
//   motion.movePose(-1.9, -0.45, -90, 0.8, 0, 0, 1, 600);
//   motion.movePose(-1.9, -0.45, -90, 0, 0, 0, 1, 1500, -1, true, true);
//   motion.movePose(-0.85, -0.45, -90, 0, 0, -1, -1, 1500, -1, false);
//     backLeftWing = true;
//     backRightWing = true;
//   motion.movePose(1.8, 0.6, -135, 0.6, 0, -0, -0.8, 750, -1, true, false);
//     backLeftWing = false;
//   motion.movePose(1.8, 0.6, -135, 0, 0, -0, -0.8, 350, -1, true, false);
//   motion.turnHeading(180, 0, 0, 0, 0);
//   motion.moveDist(-1000, 1, 200);

//   motion.push(-1, 1000); //1st push

//   motion.movePoint(2.1, 1.45, 0.2, 1, 1000, true, false);
//     backRightWing = false;
//   motion.turnHeading(110, 2, 1, 1, 700);
//   motion.turnHeading(0, 1, 1, 1, 600);
//     intakeMotors.spin(-12);
//   motion.moveDist(1000, 1, 200);
  
//   motion.push(1, 1000); //2nd push

//   motion.movePose(2.18, 1.8, 0, 0.5, 0, -0.6, -1, 1000, -1, true);
//     intakeMotors.spin(12);
//     frontLeftWing = false;
//   motion.movePose(-1.3, 2.4, -51, 0, 0, 0, 1, 2000, -1, true, true);
//   motion.movePose(0.1, 3.4, 90, 0.5, 0, 1, 1, 820, -1, false, false);
//     intakeMotors.spin(-6);
//   motion.movePose(0.1, 3.4, 90, 0.1, 0, 1, 1, 380, -1, false, false);

//   motion.push(0.4, false, 1, 1000); //3rd push

//     intakeMotors.stop();
//   motion.moveDist(-1000, 1, 200);
//     intakeMotors.spin(12);
//   motion.movePose(-1.7, 4.45, -77, 0, 0, 0.05, 1, 2000, -1, true, true);
//   motion.turnHeading(60, 2, 1, 1, 600);
//   motion.turnHeading(90, 0, 0, 0, 0);
//     fWings.on();
//   motion.moveDist(1000, 1, 300);
//     intakeMotors.spin(-12);

//   motion.push(0.4, false, 1, 1000); //4th push

//     fWings.off();
//     intakeMotors.stop();
//   motion.moveDist(-1000, 1, 150);
//   motion.movePose(-1.3, 4.1, -90, 0, 0, 0, 1, 2000);
//   motion.turnHeading(-90, 2, 0, 1, 1000);
// }


// void botMidR6() {
//     intakeMotors.spin(12);
//     frontRightWing = true;
  // motion.movePose(-1.85, 3.85, -40, 1.05, 0, 0.1, 1, 60, -1, false, false);
//     frontRightWing = false;
//   motion.movePose(-1.85, 3.85, -40, 0.85, 0, 0.1, 1, 1500, -1, true, true);
//   motion.movePose(0.12, 0.3, 0, 0.5, 0, -0.35, -1, 1200, -1, true, false);
//     intakeMotors.spin(5);
//   motion.turnHeading(20, 2, 1, 1, 300);
//     intakeMotors.spin(-8);
//   motion.turnHeading(38, 2, 0.1, 1, 500);
//     outtakeFinish();
//   motion.movePose(-1.9, -0.45, -90, 1.2, 0, 0, 1, 200);
//     intakeMotors.spin(12);
//   motion.movePose(-1.9, -0.45, -90, 0.8, 0, 0, 1, 600);
//   motion.movePose(-1.9, -0.45, -90, 0, 0, 0, 1, 1500, -1, true, true);
//   motion.movePose(-0.85, -0.45, -90, 0, 0, -1, -1, 1500, -1, false);
//     backLeftWing = true;
//     backRightWing = true;
//   motion.movePose(1.8, 0.6, -135, 0.6, 0, -0, -0.80, 850, -1, true, false);
//     backLeftWing = false;
//   motion.movePose(1.8, 0.6, -135, 0, 0, -0, -0.80, 350, -1, true, false);
//   motion.turnHeading(180, 0, 0, 0, 0);
//   motion.moveDist(-1000, 1, 200);

//   motion.push(-1, 700); //1st push

//   motion.movePoint(2.1, 1.3, 0.2, 1, 1000, true, false);
//     backRightWing = false;
//   motion.turnHeading(110, 2, 1, 1, 700);
//   motion.turnHeading(0, 1, 1, 1, 600);
//     intakeMotors.spin(-12);
//   motion.moveDist(1000, 1, 200);
  
//   motion.push(1, 1000); //2nd push

//   motion.movePose(2.18, 1.8, 0, 0.5, 0, -0.6, -1, 1000, -1, true);
//     wait(100, vex::msec);
//     intakeMotors.spin(12);
//     frontLeftWing = false;
//   motion.movePose(-1.4, 2.4, -61, 0, 0, 0, 1, 2000, -1, true, true);
//   motion.movePose(0.1, 3.4, 90, 0.5, 0, 1, 1, 1500, -1, false, false);
//     intakeMotors.spin(-12);

//   motion.push(0.4, false, 1, 1000); //3rd push

//     intakeMotors.spin(12);
//   motion.movePose(-0.1, 3.1, 0, 0, 0, -0.15, -1, 2000, -1, true, false);
//   motion.movePose(-0.45, 3.7, 0, 0, 0, 0, 1, 1500, -1, true, true);
//   motion.turnHeading(60, 2, 1, 1, 600);
//   motion.turnHeading(90, 0, 0, 0, 0);
//     fWings.on();
//   motion.moveDist(1000, 1, 300);
//     intakeMotors.spin(-12);

//   motion.push(0.4, false, 1, 1000); //4th push

//     fWings.off();
//     intakeMotors.stop();
//   motion.moveDist(-1000, 1, 150);
//   motion.movePose(-1.3, 4.1, -90, 0, 0, 0, 1, 2000);
//   motion.turnHeading(-90, 2, 0, 1, 1000);
// }


// void skills() {

//     vex::task slapperTask(cataTask);
    // odometry.setPos(-0.0629, 0.0866);
//     intakeMotors.spin(-12);
  // motion.movePose(-1.5, 1.9, 0, 0.6, 0, 1, 1, 1200, -1, false);

//   motion.push(2.1, true, 1, 500); //pushes alliance triballs into goal

//     backRightWing = true;
  // motion.moveDist(-1000, 1, 100);
  // motion.turnHeading(63, 1, 0, 1, 600);
//   motion.push(-0.3, 500);
//     slapperTask.suspend();
//     catapult.stop();
//     vex::task turnToGoalTask(turnToGoal);

//     slapper.shoot(44, 20000); //shoots 44 triballs
//     //wait(17000, vex::msec);

//     turnToGoalTask.stop();
//     slapperTask.resume();
//     backRightWing = false;
//     drivetrain.stop();
  // motion.movePose(2.2, 6.6, 0, 5.2, 0, 1, 1, 800, -1, false);
    // fWings.on();
  // motion.movePose(2.2, 6.6, 0, 3.95, 0, 1, 1, 4000, -1, false);

  // motion.push(1, 1000); //pushes triballs over short barrier into alley

//     intakeMotors.spin(12);
//   motion.movePose(1.5, 7.3, 0, 0, 0, -1, -1, 1000, -1, true);
//     fWings.off();
//   motion.movePose(-0.2, 9.7, 45, 1, 0, 1, 1, 3000, 16, true);
//     frontRightWing = true;
//   motion.movePose(2.5, 9.9, 90, 1, 0, 1, 1, 2000, -1, false);
//   motion.movePose(8.45, 8.3, 180, 1.5, 0.5, 0.8, 0.8, 5000, -1, false);
//     slapperTask.stop();
//     catapult.stop();
//     intakeMotors.stop();
  
  // motion.push(1, 2000); //1st left push

  // motion.movePose(7.7, 8.8, 135, 0, 0, -1, -1, 150);
//     intakeMotors.spin(-12);
  // motion.movePose(7.7, 8.8, 135, 0, 0, -1, -1, 850);
  // motion.movePose(8.45, 8.3, 180, 0, 0, 1, 1, 1000, -1, false);
//     intakeMotors.stop();

  // motion.push(1, 2000); //2nd left push
  
//   motion.movePose(7.8, 8.8, 135, 0, 0, -0.8, -1, 1000);
//     intakeMotors.spin(12);
//     frontLeftWing = true;
//   motion.movePose(5.45, 6.3, 135, 0.3, 0, 0.45, 0.6, 1000, 15, true);
//     frontRightWing = false;
//   motion.movePose(5.45, 6.3, 160, 0.1, 0, 0.45, 0.45, 1700, 15, true);
//     intakeMotors.stop();
  // motion.movePose(5.8, 5.4, 90, 0, 0, 0.8, 0.8, 1500, -1, true);

//   motion.push(1, 2000); //1st middle push

//   motion.movePose(6.15, 5.4, 90, 0, 0, -0.4, -1, 1000);
//   motion.moveDist(1000, 1, 250);

//   motion.push(1, 2000); //2nd middle push

//     frontLeftWing = false;
//     intakeMotors.stop();
//   motion.moveDist(-1000, 1, 100);
//     intakeMotors.spin(12);
//   motion.movePose(4.1, 5.8, 180, 4.15, 0.5, 0.5, 1, 2000, 20);
//   motion.turnHeading(150, 2, 1, 1, 500);
//     intakeMotors.stop();
//     frontLeftWing = true;
//   motion.movePose(5.8, 5.15, 90, 1.5, 0, 0.6, 0.6, 2500, -1, false);

//   motion.push(0.8, 2000); //3rd middle push

//     fWings.off();
//   motion.moveDist(-1000, 1, 100);
//     intakeMotors.spin(12);
//   motion.movePose(4.1, 5, 180, 4, 0, 0.05, 1, 2000);
//   motion.turnHeading(150, 2, 1, 1, 600);
//     frontLeftWing = true;
//     intakeMotors.stop();
//   motion.movePose(5.8, 4.3, 90, 0, 0, 0.5, 0.5, 1000, -1, false);

//   motion.push(1, 2000); //4th middle push

//     frontLeftWing = false;
//     intakeMotors.stop();
//   motion.moveDist(-1000, 1, 100);
//     intakeMotors.spin(12); 
  // motion.movePose(4.45, 2.9, 168, 2.9, 0, 0.05, 1, 3500);
//   motion.turnHeading(90, 2, 1, 1, 500);
//     fWings.on();
//     intakeMotors.stop();
//   motion.movePose(6, 4.5, 90, 0, 0, 1, 1, 1500, -1, false);

//   motion.push(1, 2000); //5th middle push

//   motion.movePose(6.15, 4.6, 90, 0, 0, -0.4, -1, 1000);
//   motion.moveDist(1000, 1, 250);

//   motion.push(1, 2000); //6th middle push

//     fWings.off();
//   motion.moveDist(-1000, 1, 200);
//   motion.movePose(5.35, 4.2, 90, 0, 0, -0.75, -1, 1500);
//   motion.turnHeading(110, 2, 1, 1, 500);
//     intakeMotors.spin(-6);
//     fWings.on();
//   motion.movePose(6.05, 1.62, 135, 2, 0, 0.75, 0.75, 3000);
//     fWings.off();
//     intakeMotors.spin(12);
//   motion.movePose(6.35, -0.1, 180, 0, 0, 0.3, 1, 1200);
//   motion.turnHeading(110, 2, 1, 1, 500);
//     frontLeftWing = true;
//     intakeMotors.stop();
//   motion.movePose(8.55, 1.4, 0, 1.2, 0.15, 0.8, 0.8, 2000, -1, false);

//   motion.push(1, 2000); //1st right push

//   motion.movePose(7, 0, 45, 0, 0, -0.6, -1, 150);
//     intakeMotors.spin(-12);
//   motion.movePose(7, 0, 45, 0, 0, -0.6, -1, 1200);
//     frontLeftWing = false;
//     hang = true;
//   motion.movePose(8.55, 1.4, 1, 0, 0, 1, 1, 1000, -1, false);
//     intakeMotors.stop();

//   motion.push(1, 2000); //2nd right push

//     intakeMotors.spin(-12);
//     frontLeftWing = false;
//   motion.movePose(7.8, 1.7, 45, 0, 0, -0.5, -1, 2500, 20);
//   motion.movePose(4.8, 2.27, 115, 0, 0, -0, -0.8, 3500, 12);
//   motion.turnHeading(43, 2, 0, 1, 650);
//   motion.moveDist(-2, 0.5, 400);
//   motion.push(-0.5, 1000);
//   motion.moveDist(0.1, 0.6, 400);
//     pto = true;
//   drivetrain.spin(vex::fwd, 12, vex::volt); //hang
//     wait(400, vex::msec);
//     hang = false;
// }


// void skillsMacro() {

//     vex::task slapperTask(cataTask);
//     odometry.setPos(-0.0629, 0.0866);
//     intakeMotors.spin(-12);
  // motion.movePose(-1.5, 1.9, 0, 0.6, 0, 1, 1, 1200, -1, false);

//   motion.push(2.1, true, 1, 500); //pushes alliance triballs into goal

//     backRightWing = true;
//   motion.moveDist(-1000, 1, 100);
//   motion.turnHeading(63, 1, 0, 1, 600);
//   motion.push(-0.3, 500);
//     slapperTask.stop();
//     catapult.stop();
//     vex::task turnToGoalTask(turnToGoal);

//     slapper.shoot(44, 20000); //shoots 44 triballs

//     turnToGoalTask.stop();
//     backRightWing = false;
//     drivetrain.stop();
// }


void test(){
  // odometry.setUnit(inch);
  // motion.setUnit(inch);
  odometry.setPos(0, 0);
  imu.setRotation(0, vex::deg);
  // motion.movePose(24, 24, 90, 0.6, 0, 60, 127, 2500, 1, true, false);
  // while(1){
  //   motion.turnHeading(90, 2, 0, 1, 1000);
  //   motion.turnHeading(180, 2, 0, 1, 1000);
  //   motion.turnHeading(0, 2, 0, 1, 1000);
  //   wait(1, vex::sec);
  // }
  // motion.moveDist(2, 1, 2000);

  // motion.moveDist(-2, 1, 2000);

  // motion.turnPoint(2, 2, 2, 0, 1, 10000);
  // motion.movePoint(2, 2, 0, 0.4, 10000);
  // motion.turnPoint(0, 0, 2, 0, 1, 10000);
  // motion.movePoint(0, 0, 0, 0.4, 10000);
  // motion.turnPoint(2, 2, 2, 0, 1, 10000);
  // motion.movePoint(2, 2, 0, 0.4, 10000);
  // motion.turnPoint(0, 0, 2, 0, 1, 10000);
  // motion.movePoint(0, 0, 0, 0.4, 10000);
  // wait(3, vex::sec);
  // motion.turnPoint(0, 0, 2, 0, 1, 5000);

  // motion.movePose(2, 2, 45, 0.5, 0, 0, -1, 10000, -1, true, false); // -1 for max speed == reverse
  // motion.movePose(0, 4, 180, 0.5, 0, 0, -1, 10000, -1, true, false); // -1 for max speed == reverse
  // motion.moveDist(2, 1, 2000);

  // Angular PID Tuing
  // motion.turnPoint(2, 0, 2, 0, 1, 1500);
  // motion.turnHeading(90, 2, 0, 1, 2500);
  // motion.turnHeading(180, 2, 0, 1, 2500);
  // motion.turnHeading(0, 2, 0, 1, 2500);
  // motion.turnPoint(0, -2, 2, 0, 1, 1500);
  // motion.turnPoint(0, 2, 2, 0, 1, 1500);

  // // // Lateral PID tuning
  // // controller.Screen.clearScreen();
  // // controller.Screen.setCursor(1, 1);
  // // controller.Screen.print("X: %f Y: %f",odometry.getPosX(),odometry.getPosY());
  // motion.moveDist(2, 1, 2000);
  // // controller.Screen.clearScreen();
  // // controller.Screen.setCursor(1, 1);
  // // controller.Screen.print("X: %f Y: %f",odometry.getPosX(),odometry.getPosY());
  // motion.moveDist(-2, 1, 2000);
  // motion.turnHeading(90, 2, 0, 1, 2500);
  // motion.turnHeading(180, 2, 0, 1, 2500);
  // motion.turnHeading(0, 2, 0, 1, 2500);
}

void redNeg(){
  odometry.setPos(-4.25, 0.833);
  imu.setRotation(147, vex::deg); // 0 facing mid
  lift.set(true);
  wait(0.1, vex::sec);
  // motion.moveDist(0.4, 0.5, 1000);
  // motion.movePose(-5, 0, 147, 0, 0, 0, 0.5, 1200, -1, true, true);
  leftDrivetrain.spin(vex::fwd,4000,vex::voltageUnits::mV);
  rightDrivetrain.spin(vex::fwd,4000,vex::voltageUnits::mV);
  wait(0.8, vex::sec);
  leftDrivetrain.spin(vex::fwd,0,vex::voltageUnits::mV);
  rightDrivetrain.spin(vex::fwd,0,vex::voltageUnits::mV);
  // motion.push(0.7, 800);
  // motion.moveDist(0.7, 0.7, 1000);
  lift.set(false);
  wait(0.4, vex::sec);
  leftDrivetrain.spin(vex::fwd,-7000,vex::voltageUnits::mV);
  rightDrivetrain.spin(vex::fwd,-7000,vex::voltageUnits::mV);
  wait(0.5, vex::sec);
  leftDrivetrain.spin(vex::fwd,0,vex::voltageUnits::mV);
  rightDrivetrain.spin(vex::fwd,0,vex::voltageUnits::mV);
  motion.movePose(-2,2,135,0.5,0,0,-1,3000,-1,true,false);
  mogo.set(true);
    controller.Screen.clearScreen();
    controller.Screen.setCursor(1, 1);
    controller.Screen.print("X: %f Y: %f",odometry.getPosX(),odometry.getPosY());
  // intakeMotors.spin(12);
  // motion.turnPoint(-0.3, 2.9, 2, 0.2, 1, 1500);
  // motion.movePose(-0.35, 3.615, 270, 0.5, 0, 0.3, 1, 2500, -1, false, true);
  // motion.moveDist(0.8, 1, 1500);
  // motion.movePose(-2, 2, 315, 0.5, 0, 0.3, 1, 2500, -1, false, false);
  // motion.turnPoint(-2, 4, 2, 0.2, 1, 1500);
  // motion.movePose(-2,4,270,0.3,0,0.2,1,1500,-1,true,true);
}

void auton() {
  //main body of all autonomous functions

  initialize();


  //Path test(testPath);
  // odometry.setPos(0, 0);
  // while(1){ 
  //   controller.Screen.clearScreen();
  //   controller.Screen.setCursor(1, 1);
  //   controller.Screen.print("X: %f Y: %f",odometry.getPosX(),odometry.getPosY());
  //   wait(200, vex::msec);
  // }
  // test();
  redNeg();
  // motion.moveDist(3, 1, 2000);
  // leftAWP();
  // leftElims();
  // topMidR6();
  //  botMidR6();
  // skills();
  // skillsMacro();

  //motion.followAPS(test, 30000, 0.8, 10);
}