#include "main.h"
#include "config.h"


#define RIGHTANGLE 780
#define TURNENCODERPRECISION 6
#define MOVEENCODERPRECISION 7
#define NUMDRIVEMOTORS 4
#define VELOCITYLIMIT 3
#define GYRODRIFTRATE 0.0

#define KP 0.10   //0.12
#define KI 0.0025 //0.003
#define INTEGRALLIMIT 6

#define DRIVEP 0.075
#define DRIVEI 0.0075
#define DRIVED 0.075
#define CORRD 0.25
#define TDRIVEP 0.145 //.130
#define TDRIVEI 0.0089
#define TDRIVED 0.07 //0.04
#define DRIVEINTEGRALLIMIT 1000
#define DRIVEMAXVEL 1
#define DRIVEPOSTOL 50
#define TDRIVEINTEGRALLIMIT 1000
#define TDRIVEMAXVEL 5
#define TDRIVEPOSTOL 6

#define GYRO_START 0;
uint32_t programStartTime = 0;

adi_gyro_t gyro;
#define max(a, b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a > _b ? _a : _b; })

#define min(a, b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a < _b ? _a : _b; })

void clearDriveMotors()
{
   motor_tare_position(PORT_DRIVELEFTFRONT);
   motor_tare_position(PORT_DRIVELEFTBACK);
   motor_tare_position(PORT_DRIVERIGHTFRONT);
   motor_tare_position(PORT_DRIVERIGHTBACK);
}

void assignDriveMotorsAuton(int power){
    motor_move(PORT_DRIVELEFTFRONT, power);
    motor_move(PORT_DRIVERIGHTFRONT, power);
    motor_move(PORT_DRIVELEFTBACK, power);
    motor_move(PORT_DRIVERIGHTBACK, power);
}

void assignDriveMotorsPower(int rightSide, int leftSide)
{
   motor_move(PORT_DRIVELEFTFRONT, leftSide);
   motor_move(PORT_DRIVELEFTBACK, leftSide);
   motor_move(PORT_DRIVERIGHTFRONT, rightSide);
   motor_move(PORT_DRIVERIGHTBACK, rightSide);
}

double averageVelocity()
{
   return (abs(motor_get_actual_velocity(PORT_DRIVELEFTFRONT)) + abs(motor_get_actual_velocity(PORT_DRIVELEFTBACK)) +
           abs(motor_get_actual_velocity(PORT_DRIVERIGHTFRONT)) + abs(motor_get_actual_velocity(PORT_DRIVERIGHTBACK))) /
          NUMDRIVEMOTORS;
}

void resetGyro()
{
   adi_gyro_reset(gyro);
}

int getHeading()
{
   return adi_gyro_get(gyro) - GYRODRIFTRATE * (millis() - programStartTime) * 0.001 + GYRO_START;
}

void assignDriveMotorsDist(int leftSide, int rightSide, int power, bool clear, bool turn)
{
   if (clear)
   {
      clearDriveMotors();
   }

   int currentPrecision = MOVEENCODERPRECISION;
   if (turn)
   {
      currentPrecision = TURNENCODERPRECISION;
   }

   motor_move_absolute(PORT_DRIVELEFTFRONT, leftSide, power);
   motor_move_absolute(PORT_DRIVELEFTBACK, leftSide, power);
   motor_move_absolute(PORT_DRIVERIGHTFRONT, rightSide, -power);
   motor_move_absolute(PORT_DRIVERIGHTBACK, rightSide, -power);
   delay(100);

   while ((abs(motor_get_position(PORT_DRIVELEFTFRONT) - leftSide) + abs(motor_get_position(PORT_DRIVELEFTBACK) - leftSide) +
           (abs(motor_get_position(PORT_DRIVERIGHTFRONT) - rightSide) + abs(motor_get_position(PORT_DRIVERIGHTBACK) - rightSide))) >
          currentPrecision * NUMDRIVEMOTORS) // ||
                                             //(turn && (averageVelocity() >= VELOCITYLIMIT)))
   {
      delay(20);
   }

   delay(250);
   assignDriveMotorsPower(0, 0);
}

void straightPID(int leftSide, int rightSide, int power, int heading)
{
   leftSide *= 200.0 / 257.0;
   rightSide *= 200.0 / 257.0;
   power *= 200.0 / 257.0;

   if (redAlliance)
   {
      heading = -heading;
   }

   clearDriveMotors();

   double curP = DRIVEP;
   double curI = DRIVEI;
   double curD = DRIVED;
   double CORR = CORRD;

   int leftErrorChange = 0;
   int rightErrorChange = 0;
   int leftError = 3 * leftSide;
   int rightError = 3 * rightSide;
   int prevLeftError = 3 * leftSide;
   int prevRightError = 3 * rightSide;
   int leftIntegral = 0;
   int rightIntegral = 0;

   bool endSequence = false;
   int startEnd = 0;
   int endTime = 500;

   while (abs(leftErrorChange) + abs(rightErrorChange) > DRIVEMAXVEL || (!endSequence || (millis() - startEnd < endTime)))
   {
      if (abs(leftError) + abs(rightError) <= DRIVEPOSTOL && !endSequence)
      {
         endSequence = true;
         startEnd = millis();
      }

      leftIntegral += leftError;
      rightIntegral += rightError;

      if (leftIntegral > DRIVEINTEGRALLIMIT)
      {
         leftIntegral = DRIVEINTEGRALLIMIT;
      }
      if (leftIntegral < -DRIVEINTEGRALLIMIT)
      {
         leftIntegral = -DRIVEINTEGRALLIMIT;
      }
      if (rightIntegral > DRIVEINTEGRALLIMIT)
      {
         rightIntegral = DRIVEINTEGRALLIMIT;
      }
      if (rightIntegral < -DRIVEINTEGRALLIMIT)
      {
         rightIntegral = -DRIVEINTEGRALLIMIT;
      }

      leftErrorChange = leftError - prevLeftError;
      rightErrorChange = rightError - prevRightError;

      prevLeftError = leftError;
      prevRightError = rightError;

      int rightCorrection = heading - getHeading();
      assignDriveMotorsPower(min(power, max(-power, leftError * DRIVEP + leftIntegral * DRIVEI + leftErrorChange * DRIVED + rightCorrection * CORR)), min(power, max(-power, rightError * DRIVEP + rightIntegral * DRIVEI + rightErrorChange * DRIVED - rightCorrection * CORR)));

      delay(5);

      leftError = 3 * leftSide - motor_get_position(PORT_DRIVELEFTFRONT) - motor_get_position(PORT_DRIVELEFTBACK);
      rightError = 3 * rightSide - motor_get_position(PORT_DRIVERIGHTFRONT) - motor_get_position(PORT_DRIVERIGHTBACK);
   }
   assignDriveMotorsPower(0, 0);
}

void turnGyro(int degrees, int power)
{
   uint32_t cur = millis();
   if (redAlliance)
   {
      degrees = -degrees;
   }

   double curP = TDRIVEP;
   double curI = TDRIVEI;
   double curD = TDRIVED;

   int errorChange = 0;
   int error = degrees - getHeading();
   int prevError = degrees - getHeading();
   int integral = 0;

   bool endSequence = false;
   int startEnd = 0;
   int endTime = 300; //500

   while ((abs(errorChange) > TDRIVEMAXVEL || (!endSequence || (millis() - startEnd < endTime))) && millis() - cur < 1000)
   {
      if (abs(error) <= TDRIVEPOSTOL && !endSequence)
      {
         endSequence = true;
         startEnd = millis();
      }

      integral += error;
      if (integral > TDRIVEINTEGRALLIMIT)
      {
         integral = TDRIVEINTEGRALLIMIT;
      }
      else if (integral < -TDRIVEINTEGRALLIMIT)
      {
         integral = -TDRIVEINTEGRALLIMIT;
      }

      errorChange = error - prevError;
      prevError = error;

      delay(5);

      error = degrees - getHeading();
      int powerToAssign = min(power, max(-power, error * TDRIVEP + integral * TDRIVEI + errorChange * TDRIVED));
      assignDriveMotorsPower(powerToAssign, -powerToAssign);
   }
   assignDriveMotorsPower(0, 0);
}

void coast(int ticks, int power, int heading, bool corr)
{
   ticks *= 200.0 / 257.0;
   power *= 200.0 / 257.0;

   if (redAlliance)
   {
      heading = -heading;
   }

   int error = 0;

   clearDriveMotors();
   assignDriveMotorsPower(power, power);
   while (abs(motor_get_position(PORT_DRIVELEFTFRONT)) + abs(motor_get_position(PORT_DRIVELEFTBACK)) +
              abs(motor_get_position(PORT_DRIVERIGHTFRONT)) + abs(motor_get_position(PORT_DRIVERIGHTBACK)) <
          ticks * NUMDRIVEMOTORS)
   {
      error = heading - getHeading();
      if (!corr)
      {
         error = 0;
      }
      assignDriveMotorsPower(power + error * CORRD, power - error * CORRD);
      delay(5);
   }
   assignDriveMotorsPower(0, 0);
}

void forward(int ticks, int power, int heading)
{
   straightPID(ticks, ticks, power, heading);
}

void backward(int ticks, int power, int heading)
{
   straightPID(-ticks, -ticks, power, heading);
}

void forwardCoast(int ticks, int power, int heading)
{
   coast(ticks, power, -heading, true);
}

void backwardCoast(int ticks, int power, int heading)
{
   coast(ticks, -power, heading, true);
}

void turnLeft(int degrees, int power)
{
   turnGyro(-degrees, power);
}

void turnRight(int degrees, int power)
{
   turnGyro(-degrees, power);
}
void turnLeftNOT(int power, int time){
    motor_move(PORT_DRIVELEFTFRONT, power);
    motor_move(PORT_DRIVERIGHTFRONT, -power);
    motor_move(PORT_DRIVELEFTBACK, power);
    motor_move(PORT_DRIVERIGHTBACK, -power);
    delay(time);
    assignDriveMotorsAuton(0);
}
void turnRightNOT(int power, int time){
    motor_move(PORT_DRIVELEFTFRONT, -power);
    motor_move(PORT_DRIVERIGHTFRONT, power);
    motor_move(PORT_DRIVELEFTBACK, -power);
    motor_move(PORT_DRIVERIGHTBACK, power);
    delay(time);
    assignDriveMotorsAuton(0);
}
void turnRD(int ticks, int power, bool clear){
     assignDriveMotorsDist(-ticks, ticks, power, clear, true);
}
void turnLD(int ticks, int power, bool clear){
     assignDriveMotorsDist(ticks, -ticks, power, clear, true);
}

void autonArm(int power){
   motor_move(PORT_ARMLEFT, power);
}
void autonRollers(int power){
    motor_move(PORT_ROLLERLEFT, power);
    motor_move(PORT_ROLLERRIGHT, -power);
}
void autonTilter(int power){
    motor_move(PORT_TILTER, power);
}
void flipout(){
   motor_move(PORT_ARMLEFT, 127);
   motor_move(PORT_ROLLERLEFT, -127);
   motor_move(PORT_ROLLERRIGHT, 127);
   delay(400); 
   motor_move(PORT_ARMLEFT, -127);
   delay(400);
   motor_move(PORT_ROLLERLEFT, 127);
   motor_move(PORT_ROLLERRIGHT, -127);
   delay(200);
   motor_move(PORT_ARMLEFT, -50);
   /*motor_move(PORT_ARMLEFT, 127);
   motor_move(PORT_ROLLERLEFT, -127);
   motor_move(PORT_ROLLERRIGHT, 127);
   assignDriveMotorsAuton(127);
   delay(300);
   assignDriveMotorsAuton(-127);
   delay(300);
   motor_move(PORT_ARMLEFT, -127);
   motor_move(PORT_ROLLERLEFT, 127);
   motor_move(PORT_ROLLERRIGHT, -127);*/
}
void oneCube(){
   
   assignDriveMotorsAuton(127);
   delay(800);
   assignDriveMotorsAuton(-127);
   delay(700);
   assignDriveMotorsAuton(0);
   flipout();

}
void driveForward(){
   assignDriveMotorsAuton(127);
   delay(10000);
   assignDriveMotorsAuton(0);
}
void pickupCubes5(){
   delay(200);
   autonRollers(127);
   assignDriveMotorsAuton(127);
   delay(300);
   assignDriveMotorsAuton(40);
   delay(800);
   assignDriveMotorsAuton(0);
   delay(200);
   assignDriveMotorsAuton(40);
   delay(1700);
   assignDriveMotorsAuton(0);
   delay(300);
   assignDriveMotorsAuton(-80);
   delay(150);
   assignDriveMotorsAuton(0);
}
void dropsmallTicks5(bool red){
   assignDriveMotorsAuton(-40);
   delay(300);
   assignDriveMotorsAuton(-60);
   delay(400);
   assignDriveMotorsAuton(-80);
   delay(600);
   assignDriveMotorsAuton(0);
   if(red == true){
      turnRD(830, 80, true);
   }else{
      turnLD(940, 80, true);
   }
   autonRollers(0);
   assignDriveMotorsAuton(60);
   delay(1500);
   assignDriveMotorsAuton(0);
   delay(200);
   assignDriveMotorsAuton(-60);
   delay(115);
   assignDriveMotorsAuton(0);
   autonRollers(-60);
   delay(265);
   autonRollers(0);
   autonTilter(47);
   //assignDriveMotorsAuton(20);
   delay(500);
   autonTilter(0);
   delay(200);
   autonTilter(70);
   delay(800);
   delay(900);
   //autonRollers(-45);
   delay(1000);
   assignDriveMotorsAuton(-30);
   delay(2000);
   autonTilter(0);
   assignDriveMotorsAuton(0);
}
void pickupCube6(bool red){
   delay(100);
   autonRollers(40);
   assignDriveMotorsAuton(40);
   delay(500);
   autonRollers(127);
   delay(800);
   assignDriveMotorsAuton(40);
   delay(800);
   assignDriveMotorsAuton(0);
   delay(150);
   assignDriveMotorsAuton(40);
   delay(800);
   assignDriveMotorsAuton(-40);
   delay(400);
   if(red == true){
      turnLD(230, 80, true);
   }else{
      turnRD(135, 80, true);
   }
   assignDriveMotorsAuton(60);
   delay(600);
   assignDriveMotorsAuton(0);
   delay(200);
   assignDriveMotorsAuton(-45);
   delay(500);
   assignDriveMotorsAuton(0);
   if(red == true){
      turnRD(220, 80, true);  
   }else{
      turnLD(130, 80, true);
   }

   assignDriveMotorsAuton(0);
   delay(100);
   assignDriveMotorsAuton(-80);
   delay(100);
   assignDriveMotorsAuton(0);
}
/*void pickupCube6Gyro(bool red){
   delay(100);
   autonRollers(40);
   assignDriveMotorsAuton(40);
   delay(500);
   autonRollers(127);
   delay(800);
   assignDriveMotorsAuton(40);
   delay(800);
   assignDriveMotorsAuton(0);
   delay(150);
   assignDriveMotorsAuton(40);
   delay(800);
   assignDriveMotorsAuton(-40);
   delay(400);
   if(red == true){
      turnLeft(310, 80);
   }else{
      turnRight(310, 80);
   }
   assignDriveMotorsAuton(60);
   delay(400);
   assignDriveMotorsAuton(0);
   delay(200);
   assignDriveMotorsAuton(-45);
   delay(500);
   assignDriveMotorsAuton(0);
   if(red == true){
      turnRight(0, 80);  
   }else{
      turnLeft(0, 80);
   }

   assignDriveMotorsAuton(0);
   delay(100);
   assignDriveMotorsAuton(-80);
   delay(100);
   assignDriveMotorsAuton(0);   
}*/
void pickupCube7(bool red){
   delay(100);
   autonRollers(40);
   assignDriveMotorsAuton(40);
   delay(500);
   autonRollers(127);
   delay(800);
   assignDriveMotorsAuton(40);
   delay(800);
   assignDriveMotorsAuton(0);
   delay(150);
   assignDriveMotorsAuton(40);
   delay(800);
   assignDriveMotorsAuton(-40);
   delay(400);
   if(red == true){
      turnLD(180, 80, true);
   }else{
      turnRD(180, 80, true);
   }
   assignDriveMotorsAuton(60);
   delay(350);
   if(red == false){
      delay(50);
   }
   assignDriveMotorsAuton(0);
   delay(700);
   autonArm(127);
   assignDriveMotorsAuton(-45);
   delay(500);
   autonArm(30);
   delay(200);
   if(red == true){
      delay(200);
   }
   autonArm(30);
   assignDriveMotorsAuton(0);
   if(red == true){
      turnRD(155, 80, true);  
   }else{
      turnLD(155, 80, true);
   }
   assignDriveMotorsAuton(0);
   delay(50);
   assignDriveMotorsAuton(45);
   delay(600);
   autonArm(-127);
   delay(300);
   autonArm(0);
   if(red == true){
      delay(200);
   }
   assignDriveMotorsAuton(0);
   delay(300);
  if(red == true){
     turnRD(50, 50, true);
  }
   assignDriveMotorsAuton(-60);
   delay(400);
   

}
void dropsmallTicks6(bool red){
   assignDriveMotorsAuton(-80);
   delay(700);
   
   if(red == true){
      turnRD(900, 60, true);
   }else{
      turnLD(920, 60, true);
   }
   autonRollers(0);
   assignDriveMotorsAuton(80);
   delay(900);
   assignDriveMotorsAuton(0);
   delay(75);
   assignDriveMotorsAuton(-60);
   delay(40);
   if(red == true){
      delay(100);
   }
   assignDriveMotorsAuton(0);
   autonRollers(-70);
   delay(350);
   autonRollers(0);
   autonTilter(47);
   //assignDriveMotorsAuton(20);
   delay(500);
   autonTilter(0);
   delay(75);
   autonTilter(70);
   delay(800);
   delay(900);
   //autonRollers(-45);
   delay(500);
   assignDriveMotorsAuton(-30);
   delay(1000);
   assignDriveMotorsAuton(0);
   autonTilter(-127);
   delay(800);
   autonTilter(0);
}
void dropsmallGyro6(bool red){
   assignDriveMotorsAuton(-80);
   delay(700);
   
   if(red == true){
      turnRD(900, 60, true);
   }else{
      turnLD(920, 60, true);
   }
   autonRollers(0);
   assignDriveMotorsAuton(80);
   delay(900);
   assignDriveMotorsAuton(0);
   delay(75);
   assignDriveMotorsAuton(-60);
   delay(80);
   if(red == true){
      delay(100);
   }
   assignDriveMotorsAuton(0);
   autonRollers(-70);
   delay(350);
   autonRollers(0);
   autonTilter(47);
   //assignDriveMotorsAuton(20);
   delay(500);
   autonTilter(0);
   delay(75);
   autonTilter(70);
   delay(800);
   delay(900);
   //autonRollers(-45);
   delay(500);
   assignDriveMotorsAuton(-30);
   delay(1000);
   assignDriveMotorsAuton(0);
   autonTilter(-127);
   delay(800);
   autonTilter(0);
}
void dropsmallTicks7(bool red){
   assignDriveMotorsAuton(-80);
   delay(500);
   
   if(red == true){
      assignDriveMotorsAuton(-80);
      delay(600);
      turnRD(900, 60, true);
   }else{
      turnLD(920, 60, true);
   }
   autonRollers(0);
   assignDriveMotorsAuton(80);
   delay(900);
   assignDriveMotorsAuton(0);
   delay(75);
   assignDriveMotorsAuton(-60);
   delay(80);
   assignDriveMotorsAuton(0);
   autonRollers(-60);
   delay(350);
   autonRollers(0);
   autonTilter(47);
   //assignDriveMotorsAuton(20);
   delay(500);
   autonTilter(0);
   delay(75);
   autonTilter(70);
   delay(800);
   delay(900);
   //autonRollers(-45);
   delay(500);
   assignDriveMotorsAuton(-30);
   delay(1000);
   assignDriveMotorsAuton(0);
   autonTilter(-127);
   delay(800);
   autonTilter(0);
}

void pickupFour() {
 autonRollers(127);
 delay(200);
 autonArm(-45);
 delay(950);
 //assignDriveMotorsAuton(-30);
 autonArm(-50);
 delay(500);
 //assignDriveMotorsAuton(0);
 delay(1100);
 autonArm(-90);
 delay(1000);
 autonArm(0);
}

void pickupTopDown(){


 autonRollers(127);
 assignDriveMotorsAuton(45);
 delay(650);

 assignDriveMotorsAuton(30);
 autonArm(127);
 delay(450);
 assignDriveMotorsAuton(0);
 delay(1400);
 assignDriveMotorsAuton(40);
 delay(800);
 autonArm(0);
 delay(65);
 assignDriveMotorsAuton(0);
 //autonArm(0);

 pickupFour();
}

void seventhCube(bool red){
 autonRollers(127);

 if(red == true){
    turnLeftNOT(80, 50);
 }else{
    turnRightNOT(80, 50);
 }

 assignDriveMotorsAuton(60);
 delay(550);
 assignDriveMotorsAuton(0);
 delay(500);


 assignDriveMotorsAuton(-80);
 delay(400);
 assignDriveMotorsAuton(0);


 if(red == true){
    turnRight(80, 50);
 }else{
    turnLeft(80, 50);
 }


}

void evanAuton6(bool red) {
 flipout();
 pickupTopDown();

 assignDriveMotorsAuton(-70);
 
 /*
 assignDriveMotorsAuton(-50);
 delay(300);
 assignDriveMotorsAuton(-30);
 delay(100);
 */
 if(red == true){
   delay(700);
   assignDriveMotorsAuton(0);
   turnLeftNOT(90, 560);
 }else{
   delay(850);
   assignDriveMotorsAuton(0);
   turnRightNOT(90, 470);
 }


 assignDriveMotorsAuton(70);
 delay(700);
 assignDriveMotorsAuton(50);
 delay(300);
 assignDriveMotorsAuton(0);


 assignDriveMotorsAuton(-60);
 delay(40);
 autonRollers(0);
 assignDriveMotorsAuton(0);


//dropping

 autonRollers(-60);
 delay(265);
 autonRollers(0);
 autonTilter(47);
 //assignDriveMotorsAuton(20);
 delay(500);
 autonTilter(0);
 delay(200); 
 autonTilter(70);
 delay(800);
 delay(900);
 //autonRollers(-45);
 delay(1000);
 assignDriveMotorsAuton(-50);
 delay(2000);
 autonTilter(0);
 assignDriveMotorsAuton(0);
}


void evanAuton7(bool red) {
 flipout();
 pickupTopDown();
 seventhCube(red);

 assignDriveMotorsAuton(-70);
 delay(990);
 /*
 assignDriveMotorsAuton(-50);
 delay(300);
 assignDriveMotorsAuton(-30);
 delay(100);
 */
 assignDriveMotorsAuton(0);

 if(red == true){
      turnLeftNOT(90, 470);
 }
 else{
      turnRightNOT(90, 470);
 }


 assignDriveMotorsAuton(70);
 delay(700);
 assignDriveMotorsAuton(50);
 delay(300);
 assignDriveMotorsAuton(0);


 assignDriveMotorsAuton(-60);
 delay(115);
 autonRollers(0);
 assignDriveMotorsAuton(0);


//dropping
  autonRollers(-60);
 delay(265);
 autonRollers(0);
 autonTilter(47);
 //assignDriveMotorsAuton(20);
 delay(500);
 autonTilter(0);
 delay(200);
 autonTilter(70);
 delay(800);
 delay(900);
 //autonRollers(-45);
 delay(1000);
 assignDriveMotorsAuton(-50);
 delay(2000);
 autonTilter(0);
 assignDriveMotorsAuton(0);
}
/*uwuwuwuwuwuwuwuwuwuwuwuwuwuwuwuwuwuwuwuwuwuwuwuwuwuwuwuwuwuwuwuwuwuwuwuwuwuwuwuwuwuwuwuwuwuwuwuwuwuwuwuwuwuwuwuwuwuwuwuwuwuwuwuwuwuwuwuwuwuwuwuwuwuwuwuwuwuwuwuwuwuwuwuwuwuwuwuwuwuwu*/
/* 8 CUBE AUTON */
void brake(int bpower, int btime){
   assignDriveMotorsAuton(bpower);
   delay(btime);
   assignDriveMotorsAuton(0);
}

/*void goodAuton8Test(bool red){
   //flipout
   flipout();
   autonRollers(127);
   delay(200);

   //pickup first line of cubes
   forwardCoast(400, 120, 0);
   forwardCoast(950, 80, 0);
   forwardCoast(600, 70, 0);
   forwardCoast(450, 60, 0);
   brake(-30, 100);
   delay(200);

   //come back
   backwardCoast(950, 120, 0);
   brake(30, 100);

   //turn 
   if(red == true){
      turnLeft(-540, 127);
   }else{
      turnRight(-540, 127);
   }

   //come back
   backwardCoast(1850, 120, 540);
   brake(30, 100);

   //align with next line
   if(red == true){
      turnRight(20, 127);
   }else{
      turnLeft(20, 127);
   }

   //pickup second line of cubes
   forwardCoast(500, 85, 10);
   forwardCoast(1010, 58, 10);
   forwardCoast(950, 48, 10);

   brake(-30, 100);
   delay(200);

   //come back
   backwardCoast(500, 110, 0);
   backwardCoast(500, 127, 0);
   backwardCoast(450, 127, 0);
   //if(red == true){
      backwardCoast(200, 127, 0);
   //}
   brake(40, 100);
   
   //turn to align with goal
   if(red == true){
      turnRight(1600, 65);
   }else{
      turnLeft(1600, 65);
   }

   //start tilting
   autonTilter(65);

   //go to goal
   forwardCoast(100, 100, 1590);
   autonRollers(-65);
   assignDriveMotorsAuton(80);
   delay(300);
   autonRollers(0);
   delay(500);

   assignDriveMotorsAuton(0);

   //drive back
   assignDriveMotorsAuton(-50);
   delay(150);
   autonRollers(40);
   assignDriveMotorsAuton(0);
   brake(20, 50);

   //tilt
   motor_move_velocity(PORT_TILTER, 127);
   delay(500);
   motor_move_velocity(PORT_TILTER, 100);
   delay(700);
   autonRollers(0);
   motor_move_velocity(PORT_TILTER, 70);
   delay(1200);
   assignDriveMotorsAuton(20);
   delay(150);
   assignDriveMotorsAuton(0);

   //back out
   assignDriveMotorsAuton(-70);
   delay(800);
   assignDriveMotorsAuton(0);
   autonTilter(-127);
   delay(800);
   autonTilter(0);
}*/
void pickupFirstRow(){
   //flipout
   flipout();

   //pickup first line of cubes
   forwardCoast(500, 100, 0);
   delay(200);
   //motor_move(PORT_ARMLEFT, -20);
   forwardCoast(850, 60, 0);
   brake(-100, 50);
   autonArm(127);
   delay(800);
   forwardCoast(150, 50, 0);
   brake(-100, 20);
   autonArm(-70);
   forwardCoast(300, 50, 0);
   brake(-100, 20);
   backwardCoast(150, 50, 0);
   delay(500);
}
void alignWithNextRow(bool red){
   //come back
   backwardCoast(600, 120, 0);
   brake(30, 100);

   //turn 
   if(red == true){
      turnLeft(-580, 127);
   }else{
      turnRight(-580, 127);
   }

   //come back
   backwardCoast(1800, 120, 540);
   brake(30, 100);

   //align with next line
   if(red == true){
      turnRight(-10, 127);
   }else{
      turnLeft(-10, 127);
   }
}
void pickupSecondRow7(){
   forwardCoast(700, 100, -10);
   forwardCoast(900, 80, -10);
   forwardCoast(800, 60, -10);
   brake(-30, 100);
   delay(200);
}
void pickupSecondRow8(){
   forwardCoast(800, 95, -10);
   forwardCoast(900, 75, -10);
   forwardCoast(800, 58, -10);
   brake(-30, 100);
   delay(200);
}
void comeBack(bool red){
   backwardCoast(500, 110, 0);
   backwardCoast(800, 127, 0);
   
   //turn to align with goal
   if(red == true){
      turnRight(1580, 65);
   }else{
      turnLeft(1580, 65);
   }
}
void drop(){
   //start tilting
   autonTilter(110);

   //go to goal
   forwardCoast(100, 100, 1580);
   autonRollers(-65);

   assignDriveMotorsAuton(60);
   delay(375);
   autonRollers(0);
   delay(250);
   autonRollers(40);
   delay(525);
   assignDriveMotorsAuton(0);
   

   //drive back
   assignDriveMotorsAuton(-50);
   delay(150);

   assignDriveMotorsAuton(0);
   brake(20, 50);

   //tilt
   motor_move_velocity(PORT_TILTER, 80);
   delay(600);
   autonRollers(0);
   motor_move_velocity(PORT_TILTER, 72);
   delay(450);
   assignDriveMotorsAuton(20);
   delay(200);
   assignDriveMotorsAuton(0);

   //back out
   assignDriveMotorsAuton(-70);
   delay(800);
   assignDriveMotorsAuton(0);
   autonTilter(-127);
   delay(800);
   autonTilter(0);
}
void nineCubeAuton(bool red){
   pickupFirstRow();
   alignWithNextRow(red);
   pickupSecondRow8();
   comeBack(red);
   drop();
}

/*-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
/* PROGRAMMING SKILLS */
void tower(bool tall){
   if(tall == true){
      autonArm(127);
      delay(2500);
      autonArm(20);
   }else{
      autonArm(127);
      delay(1500);
      autonArm(20);
   }
   forwardCoast(200, 70, 0);
   autonRollers(-127);
   delay(1000);
   autonRollers(0);
   backwardCoast(200, 70, 0);
   if(tall == true){
      autonArm(-127);
      delay(2600);
      autonArm(-10);
   }else{
      autonArm(-127);
      delay(1600);
      autonArm(-10);
   }
}
void backOut(){
   assignDriveMotorsAuton(-70);
   delay(1000);
   assignDriveMotorsAuton(0);
   autonTilter(-127);
   delay(800);
   autonTilter(0);
   brake(10, 100);
   turnLeft(0, 70);
   turnRight(40, 30);
}
void placeInSmallTower(){
   autonRollers(127);
   forwardCoast(1250, 90, 0);
   delay(600);
   backwardCoast(200, 70, 0);
   tower(false);
}
void alignWithNextTower(bool red){
   backwardCoast(1500, 100, 0);
   brake(10, 100);
   turnLeft(-900, 90);
   autonRollers(127);
   forwardCoast(500, 100, 900);
   brake(-10, 100);
   delay(600);
}
void placeInTallTower(){
   backwardCoast(200, 70, -900);
   tower(true);
}
void programmingSkills(bool red){
   /*goodAuton8Test(red);
   backOut();
   placeInSmallTower();*/
   alignWithNextTower(red);
   placeInSmallTower();
} 

/*-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/

void displayInfoAuton(void *param)
{
   lcd_initialize();
   while (true)
   {
      char tempString1[100];
      char tempString2[100];
      char tempString3[100];
      char tempString4[100];
      char tempString5[100];
      char tempString6[100];

      sprintf(tempString1, "Gyro Value: %d", (int)adi_gyro_get(gyro));
      sprintf(tempString2, "Battery Voltage: %d", battery_get_voltage());

      lcd_set_text(1, tempString1);
      lcd_set_text(2, tempString2);
      lcd_set_text(3, tempString3);
      lcd_set_text(4, tempString4);                 
      lcd_set_text(5, tempString5);
      lcd_set_text(6, tempString6);

      printf("here\n");

      //controller_print(CONTROLLER_MASTER, 0, 0, "RPM: %.2f", motor_get_actual_velocity(PORT_FLYWHEEL));

      delay(10);
   }
}

void autonomous()
{
   autonNumber = 3;
   redAlliance = true;
   task_t displayInfoTask = task_create(displayInfoAuton, "PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Display Info Task");
   switch (autonNumber)
   {
   case 1:
      oneCube();
      break;
   case 2:
      flipout();
      pickupCubes5(redAlliance);
      dropsmallTicks5(redAlliance);
      break;
   case 3:
      nineCubeAuton(redAlliance);
      break;
   case 4:
      evanAuton6(redAlliance);
      break;
   case 5:
      programmingSkills(true);
      break;
   default:
      break;
   }
}






























/*
void goodAuton8Red(bool red){
   flipout();
   autonRollers(127);
   delay(200);
   forwardCoast(400, 120, 0);
   forwardCoast(950, 80, 0);
   forwardCoast(600, 70, 0);
   forwardCoast(600, 60, 0);
   brake(-30, 100);
   delay(200);
   backwardCoast(1000, 120, 0);
   brake(30, 100);
   turnLeft(-510, 127);
   backwardCoast(2000, 120, 510);
   brake(30, 100);
   turnRight(0, 127);

   forwardCoast(500, 85, 0);
   forwardCoast(1010, 58, 0);
   forwardCoast(1100, 48, 0);
   brake(-30, 100);
   delay(200);

   backwardCoast(500, 110, 0);
   backwardCoast(600, 127, 0);
   backwardCoast(700, 127, 0);
   autonRollers(0);
   brake(30, 100);
   turnRight(1650, 65);
   autonTilter(65);
   forwardCoast(100, 100, 1550);

   assignDriveMotorsAuton(90);

   autonRollers(0);
   delay(700);
   assignDriveMotorsAuton(0);
   assignDriveMotorsAuton(-50);
   delay(150);
   assignDriveMotorsAuton(0);
   brake(20, 50);

   motor_move_velocity(PORT_TILTER, 127);
   autonRollers(-35);
   delay(300);
   autonRollers(0);
   delay(200);
   motor_move_velocity(PORT_TILTER, 100);
   delay(500);
   motor_move_velocity(PORT_TILTER, 70);
   delay(1150);
   assignDriveMotorsAuton(-30);
   delay(1000);
   assignDriveMotorsAuton(0);
   autonTilter(-127);
   delay(800);
   autonTilter(0);
}
void goodAuton8Blue(bool red){
   flipout();
   autonRollers(127);
   delay(200);
   forwardCoast(400, 120, 0);
   forwardCoast(950, 80, 0);
   forwardCoast(600, 70, 0);
   forwardCoast(600, 60, 0);
   brake(-30, 100);
   delay(200);
   backwardCoast(1000, 120, 0);
   brake(30, 100);
   turnRight(-510, 127);
   backwardCoast(2150, 120, 510);
   brake(30, 100);
   turnLeft(0, 127);

   forwardCoast(500, 85, 0);
   forwardCoast(1010, 58, 0);
   forwardCoast(1250, 48, 0);
   brake(-30, 100);
   delay(200);

   backwardCoast(500, 110, 0);
   backwardCoast(600, 127, 0);
   backwardCoast(300, 127, 0);
   brake(40, 100);
   
   turnLeft(1560, 65);
   autonRollers(0);
   autonTilter(65);
   forwardCoast(100, 100, 1590);

   assignDriveMotorsAuton(90);

   autonRollers(0);
   delay(800);
   assignDriveMotorsAuton(0);
   assignDriveMotorsAuton(-50);
   delay(150);
   assignDriveMotorsAuton(0);
   brake(20, 50);

   motor_move_velocity(PORT_TILTER, 127);
   autonRollers(-35);
   delay(300);
   autonRollers(0);
   delay(200);
   motor_move_velocity(PORT_TILTER, 100);
   delay(500);
   motor_move_velocity(PORT_TILTER, 70);
   delay(1150);
   assignDriveMotorsAuton(-30);
   delay(1000);
   assignDriveMotorsAuton(0);
   autonTilter(-127);
   delay(800);
   autonTilter(0);
}
void goodAuton8Both(bool red){
   //flipout
   flipout();
   autonRollers(127);
   delay(200);

   //pickup first line of cubes
   forwardCoast(400, 120, 0);
   forwardCoast(950, 80, 0);
   forwardCoast(600, 70, 0);
   forwardCoast(600, 60, 0);
   brake(-30, 100);
   delay(200);

   //come back
   backwardCoast(1000, 120, 0);
   brake(30, 100);

   //turn 
   if(red == true){
      turnLeft(-540, 127);
   }else{
      turnRight(-540, 127);
   }

   //come back
   backwardCoast(1900, 120, 540);
   brake(30, 100);

   //align with next line
   if(red == true){
      turnRight(20, 127);
   }else{
      turnLeft(20, 127);
   }

   //pickup second line of cubes
   if(red == true){
      forwardCoast(500, 85, 0);
      forwardCoast(1010, 58, 0);
      forwardCoast(1000, 48, 0);
   }else{
      forwardCoast(500, 85, 10);
      forwardCoast(1010, 58, 10);
      forwardCoast(1250, 48, 10);
   }

   brake(-30, 100);
   delay(200);

   //come back
   backwardCoast(500, 110, 0);
   backwardCoast(600, 127, 0);
   backwardCoast(300, 127, 0);
   if(red == true){
      backwardCoast(500, 127, 0);
   }
   brake(40, 100);
   
   //turn to align with goal
   if(red == true){
      turnRight(1600, 65);
   }else{
      turnLeft(1600, 65);
   }

   //start tilting
   autonTilter(65);

   //go to goal
   forwardCoast(100, 100, 1590);
   assignDriveMotorsAuton(80);
   autonRollers(0);
   delay(800);
   assignDriveMotorsAuton(0);

   //drive back
   assignDriveMotorsAuton(-50);
   delay(150);
   assignDriveMotorsAuton(0);
   brake(20, 50);

   //tilt
   motor_move_velocity(PORT_TILTER, 127);
   autonRollers(-55);
   delay(300);
   autonRollers(0);
   delay(200);
   motor_move_velocity(PORT_TILTER, 100);
   delay(500);
   motor_move_velocity(PORT_TILTER, 70);
   delay(1200);

   //back out
   assignDriveMotorsAuton(-50);
   delay(1000);
   assignDriveMotorsAuton(0);
   autonTilter(-127);
   delay(800);
   autonTilter(0);
}
void goodAuton8fasttilt2(bool red){
   flipout();
   autonRollers(127);
   delay(200);
   forwardCoast(400, 120, 0);
   forwardCoast(950, 80, 0);
   forwardCoast(600, 70, 0);
   forwardCoast(600, 60, 0);
   brake(-30, 100);
   delay(200);
   backwardCoast(1000, 120, 0);
   brake(30, 100);
   if(red == true){
      turnLeft(-510, 127);
      backwardCoast(2240, 120, 510);
      brake(30, 100);
      turnRight(0, 127);
   }else{
      turnRight(510, 127);
      backwardCoast(2100, 120, 510);
      brake(30, 100);
      turnLeft(0, 127);
   }
   
   forwardCoast(500, 85, 0);
   forwardCoast(1050, 55, 0);
   forwardCoast(1000, 45, 0);
   brake(-30, 100);
   delay(200);

   
   if(red == true){
      backwardCoast(500, 110, 0);
      backwardCoast(600, 127, 0);
      backwardCoast(1000, 127, 0);
      autonRollers(0);
      brake(30, 100);
      turnRight(1550, 65);
   }else{
      backwardCoast(500, 110, 0);
      backwardCoast(600, 127, 0);
      backwardCoast(700, 127, 0);
      autonRollers(0);
      brake(30, 100);
      turnLeft(1600, 65);
   }
   

   autonTilter(55);
   forwardCoast(100, 120, 1550);

   assignDriveMotorsAuton(100);

   autonRollers(0);
   delay(700);
   assignDriveMotorsAuton(0);
   assignDriveMotorsAuton(-50);
   delay(150);
   assignDriveMotorsAuton(0);
   brake(20, 50);

   motor_move_velocity(PORT_TILTER, 127);
   autonRollers(-35);
   delay(300);
   autonRollers(0);
   delay(200);
   motor_move_velocity(PORT_TILTER, 100);
   delay(400);
   motor_move_velocity(PORT_TILTER, 50);
   delay(800);
   assignDriveMotorsAuton(-30);
   delay(1000);
   assignDriveMotorsAuton(0);
   autonTilter(-127);
   delay(800);
   autonTilter(0);
}
void goodAuton8tilt(bool red){
   flipout();
   autonRollers(127);
   delay(200);
   forwardCoast(400, 120, 0);
   forwardCoast(950, 80, 0);
   forwardCoast(600, 70, 0);
   forwardCoast(600, 60, 0);
   brake(-30, 100);
   delay(200);
   backwardCoast(1000, 120, 0);
   brake(30, 100);
   turnLeft(-510, 127);
   backwardCoast(2240, 120, 510);
   brake(30, 100);
   turnRight(0, 127);

   forwardCoast(500, 85, 0);
   forwardCoast(1050, 55, 0);
   forwardCoast(1000, 45, 0);
   brake(-30, 100);
   delay(200);

   backwardCoast(500, 110, 0);
   backwardCoast(600, 127, 0);
   backwardCoast(1000, 127, 0);
   autonRollers(0);
   brake(30, 100);
   turnRight(1550, 65);
   autonTilter(55);
   forwardCoast(100, 120, 1550);

   assignDriveMotorsAuton(100);

   autonRollers(0);
   delay(700);
   assignDriveMotorsAuton(0);
   assignDriveMotorsAuton(-50);
   delay(150);
   assignDriveMotorsAuton(0);
   brake(20, 50);

   motor_move_velocity(PORT_TILTER, 127);
   autonRollers(-35);
   delay(300);
   autonRollers(0);
   delay(200);
   motor_move_velocity(PORT_TILTER, 80);
   delay(400);
   motor_move_velocity(PORT_TILTER, 50);
   delay(800);
   assignDriveMotorsAuton(-30);
   delay(1000);
   assignDriveMotorsAuton(0);
   autonTilter(-127);
   delay(800);
   autonTilter(0);
}
void goodAuton8(bool red){
   flipout();
   autonRollers(127);
   delay(200);
   forwardCoast(400, 120, 0);
   forwardCoast(950, 80, 0);
   forwardCoast(900, 70, 0);
   brake(-30, 100);
   delay(200);
   backwardCoast(1100, 120, 0);
   brake(30, 100);
   turnLeft(-510, 127);
   backwardCoast(2240, 120, 510);
   brake(30, 100);
   turnRight(0, 127);

   forwardCoast(500, 85, 0);
   forwardCoast(1050, 55, 0);
   forwardCoast(1200, 45, 0);
   brake(-30, 100);
   delay(200);

   backwardCoast(550, 70, 0);
   backwardCoast(1600, 127, 0);
   brake(30, 100);
   turnRight(1550, 65);

   forwardCoast(50, 120, 1550);

   assignDriveMotorsAuton(60);
   autonRollers(-10);
   delay(400);
   autonRollers(0);
   delay(200);
   assignDriveMotorsAuton(0);
   assignDriveMotorsAuton(-50);
   delay(150);
   assignDriveMotorsAuton(0);
   brake(20, 50);

   autonTilter(127);
   autonRollers(-35);
   //assignDriveMotorsAuton(20);
   delay(300);
   autonRollers(0);
   delay(200);
   autonTilter(80);
   delay(800);
   delay(500);
   //autonRollers(-45);
   delay(500);
   assignDriveMotorsAuton(-30);
   delay(1000);
   assignDriveMotorsAuton(0);
   autonTilter(-127);
   delay(800);
   autonTilter(0);
}*/