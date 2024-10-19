#include "opcontrol.h"
#include "auton.h"


bool reverseDrive = false, hanging = false, hangDone = false;

MotorGroup leftMotors({
  Motor("fl", &fl),
  Motor("ml", &ml),
  Motor("bl", &bl)
});

MotorGroup rightMotors({
  Motor("fr", &fr),
  Motor("mr", &mr),
  Motor("br", &br)
});

MotorGroup intakeMtrs({
  Motor("intake1", &intake),
  Motor("intake2", &intake2)
});

ChassisGeometry chassisGeometry(&leftMotors, &rightMotors, 1);
ChassisDriverSettings chassisDriverSettings(&controller, 1, 0, 800, true);
Tank chassis(chassisGeometry, chassisDriverSettings);

// PistonGroup frontWings({
//   Piston("left", &frontLeftWing),
//   Piston("right", &frontRightWing)
// });

// PistonGroup backWings({
//   Piston("left", &backLeftWing),
//   Piston("right", &backRightWing)
// });


void temperature() {
    controller.Screen.clearScreen();
    controller.Screen.setCursor(1, 1);
  controller.Screen.print("fl: %d          ", (int) fl.temperature(vex::percent));
  controller.Screen.print("fr: %d", (int) ml.temperature(vex::percent));
    controller.Screen.setCursor(2, 1);
  controller.Screen.print("ml: %d          ", (int) bl.temperature(vex::percent));
  controller.Screen.print("mr: %d", (int) fr.temperature(vex::percent));
    controller.Screen.setCursor(3, 1);
  controller.Screen.print("bl: %d          ", (int) mr.temperature(vex::percent));
  controller.Screen.print("br: %d", (int) br.temperature(vex::percent));
}


// void hangPressed() {

//   if (!hang && !hangDone) hang = true;
//   else if (!hangDone) {
//     pto = true;
//     hanging = true;
//     chassis.spinTank(12, 12, true, false);
//     wait(400, vex::msec);
//     hang = false;
//     hangDone = true;
//   }
// }


// void reverseDrivePressed() {

//   reverseDrive = !reverseDrive;
//   controller.rumble(".");
// }

float deadband(float input, float width){
  if (fabs(input)<width) return(0);
  return(input);
}

float to_volt(float percent){
  return(percent*12.0/100.0);
}

int mogoUC(){
  bool mogoc = false;
  while (1) {
    if(controller.ButtonY.pressing()) {
      // while(controller.ButtonY.pressing() != 0);
      waitUntil(controller.ButtonY.pressing() == 0);
      
      mogoc = !mogoc;
    }
    mogo.set(mogoc);
    wait(10, vex::msec);
  }
}

// int pisstakeUC(){
//   bool pisstakec = false;
//   while (1) {
//     if(controller.ButtonDown.pressing()) {
//       while(controller.ButtonDown.pressing() != 0)
//         pisstakec = !pisstakec; 
//     }
//     pisstake.set(pisstakec);
//     wait(10, vex::msec);
//   }
// }

int redirectUC(){
  bool redirectc = false;
  while (1) {
    if(controller.ButtonRight.pressing()) {
      // while(controller.ButtonRight.pressing() != 0)
      waitUntil(controller.ButtonRight.pressing() == 0);
      redirectc = !redirectc;
    }
    redirect.set(redirectc);
    wait(10, vex::msec);
  }
}

int liftUC(){
  bool liftc = false;
  while (1) {
    if(controller.ButtonL2.pressing()) {
      // while(controller.ButtonL2.pressing() != 0)
      waitUntil(controller.ButtonL2.pressing() == 0);
        liftc = !liftc; 
    }
    lift.set(liftc);
    wait(10, vex::msec);
  }
}

void opcontrol() {

  //kills autonomous tasks

    taskHandler(true);
    vex::task mogoTask(mogoUC);
    // vex::task pisstakeTask(pisstakeUC);
    vex::task redirectTask(redirectUC);
    vex::task liftTask(liftUC);

  //catapult initialization

  //   MotorGroup cataMotors({
  //     Motor("cata", &catapult),
  //     Motor("cata2", &catapult2)
  //   });
  //   Cata cata(&cataMotors, &cataRotation);
  //   cata.setResetSetpoint(0);
  //   cata.setPower(0.78);
  //   cata.setShotCounterBounds(15, 40);

  // //hang initialization

  //   controller.ButtonY.pressed(hangPressed);

  // //temperature checker
    
  //   controller.ButtonUp.pressed(temperature);

  // //for macro

  //   //auton();

  //drivetrain initialization

    leftMotors.setStopping(true);
    rightMotors.setStopping(true);
    // int A1, A3;
    // bool liftc = false, pisstakec = false, mogoc = false, redirectc = false;
    while (1)
    {
    

      // track system time

      uint64_t timestamp = vex::timer::systemHighResolution();

    //drivetrain code

    //  if (!hanging) (!reverseDrive) ? chassis.controllerFeedbackSpin() : chassis.controllerFeedbackSpin(true);
    
    //  A1 = deadband((controller.Axis1.value() * (12000.0 - chassisDriverSettings.fStaticFF) / 127.0),5);
    //  A3 = deadband((controller.Axis3.value() * (12000.0 - chassisDriverSettings.fStaticFF) / 127.0),5);

    double throttle = deadband(controller.Axis3.value(), 5);
    double turn = deadband(controller.Axis1.value(), 5);
    //  chassis.spinTank(A3 + A1, A3 - A1);
    leftMotors.spin(to_volt(throttle+turn));
    rightMotors.spin(to_volt(throttle-turn));

    (controller.ButtonR2.pressing()) ? stick.set(true) : stick.set(false);

    //intake code
      if (controller.ButtonL1.pressing()) intakeMtrs.spin(12);
      else if (controller.ButtonR1.pressing()) intakeMtrs.spin(-12);
      else intakeMtrs.stop();

    wait((timestamp + 11000.0 - vex::timer::systemHighResolution()) / 1000.0, vex::msec);
    }
}