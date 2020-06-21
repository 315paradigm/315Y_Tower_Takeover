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

uint32_t programStartTime = 0;

extern adi_gyro_t gyro;

#define max(a, b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a > _b ? _a : _b; })

#define min(a, b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a < _b ? _a : _b; })


void assignDriveMotorsAuton(int power){
    motor_move(PORT_DRIVELEFTFRONT, power);
    motor_move(PORT_DRIVERIGHTFRONT, -power);
    motor_move(PORT_DRIVELEFTBACK, power);
    motor_move(PORT_DRIVERIGHTBACK, -power);
}

void clearDriveMotors()
{
   motor_tare_position(PORT_DRIVELEFTFRONT);
   motor_tare_position(PORT_DRIVELEFTBACK);
   motor_tare_position(PORT_DRIVERIGHTFRONT);
   motor_tare_position(PORT_DRIVERIGHTBACK);
}

void assignDriveMotorsPower(int leftSide, int rightSide)
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
   return adi_gyro_get(gyro) - GYRODRIFTRATE * (millis() - programStartTime) * 0.001;
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
   motor_move_absolute(PORT_DRIVERIGHTFRONT, rightSide, power);
   motor_move_absolute(PORT_DRIVERIGHTBACK, rightSide, power);
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
void turnRD(int ticks, int power, bool clear){
     assignDriveMotorsDist(-ticks, -ticks, power, clear, true);
}
void turnLD(int ticks, int power, bool clear){
     assignDriveMotorsDist(ticks, ticks, power, clear, true);
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
   int endTime = 500;

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
   coast(ticks, power, heading, true);
}

void backwardCoast(int ticks, int power, int heading)
{
   coast(ticks, -power, heading, true);
}

void turnLeft(int degrees, int power)
{
   turnGyro(degrees, power);
}

void turnRight(int degrees, int power)
{
   turnGyro(degrees, power);
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
   autonRollers(-127);
   delay(150);
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
      turnLD(180, 80, true);
   }else{
      turnRD(135, 80, true);
   }
   assignDriveMotorsAuton(60);
   delay(400);
   assignDriveMotorsAuton(0);
   delay(200);
   assignDriveMotorsAuton(-45);
   delay(500);
   assignDriveMotorsAuton(0);
   if(red == true){
      turnRD(160, 80, true);  
   }else{
      turnLD(130, 80, true);
   }

   assignDriveMotorsAuton(0);
   delay(100);
   assignDriveMotorsAuton(-80);
   delay(100);
   assignDriveMotorsAuton(0);
}
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
void pickupStack(){
  autonRollers(127);
  assignDriveMotorsAuton(80);
  delay(750);
  //delay(1200);
  assignDriveMotorsAuton(0);
}
void pickupFour() {
 autonRollers(127);
 delay(200);
 autonArm(-45);
 delay(950);
 //assignDriveMotorsAuton(-30);
 autonArm(-55);
 delay(300);
 assignDriveMotorsAuton(0);
 delay(1100);
 autonArm(-95);
 delay(950);
 autonArm(0);
}
void pickupTopDown(){
 delay(200);
 autonRollers(127);
 assignDriveMotorsAuton(45);
 delay(750);
 assignDriveMotorsAuton(30);
 autonArm(127);
 delay(450);
 assignDriveMotorsAuton(0);
 delay(1700);
 assignDriveMotorsAuton(40);
 delay(800);
 assignDriveMotorsAuton(0);
 autonArm(0);
 pickupFour();
}
void topDownBig(bool red) {
 flipout();
 delay(200);
 pickupTopDown();
 assignDriveMotorsAuton(-70);
 delay(1415);
 assignDriveMotorsAuton(0);
 if(red == true){
      turnLeft(90, 570);
 }
 else{
      turnRight(85, 600);
 }
 assignDriveMotorsAuton(75);
 delay(1250);
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
 assignDriveMotorsAuton(-30);
 delay(2000);
 autonTilter(0);
 assignDriveMotorsAuton(0);
}

void autonomous()
{
   autonNumber = 3;
   redAlliance = true;
   switch (autonNumber)
   {
   case 1:
      oneCube();
      break;
   case 2:
      flipout();
      pickupCubes5();
      dropsmallTicks5(redAlliance);
      break;
   case 3:
      flipout();
      pickupCube6(redAlliance);
      dropsmallTicks6(redAlliance);
      break;
   case 4:
      topDownBig(redAlliance);
      break;
   case 5:
      break;
   default:
      break;
   }
}









































































































/*
void pickupCube6_2(){
   delay(200);
   autonRollers(127);
   assignDriveMotorsAuton(80);
   delay(600);
   assignDriveMotorsAuton(40);
   delay(800);
   assignDriveMotorsAuton(0);
   delay(150);
   assignDriveMotorsAuton(60);
   delay(1500);
   assignDriveMotorsAuton(0);
   delay(500);
   assignDriveMotorsAuton(-80);
   delay(150);
   assignDriveMotorsAuton(0);
}
void pickupCubes5_8_1(){
   delay(200);
   autonRollers(127);
   assignDriveMotorsAuton(127);
   delay(300);
   assignDriveMotorsAuton(40);
   delay(800);
   assignDriveMotorsAuton(0);
   delay(200);
   assignDriveMotorsAuton(40);
   delay(500);
   assignDriveMotorsAuton(0);
   delay(100);
   assignDriveMotorsAuton(-80);
   delay(150);
   assignDriveMotorsAuton(0);
}
void pickupCubes5_8_2(){
   delay(200);
   autonRollers(127);
   assignDriveMotorsAuton(127);
   delay(300);
   assignDriveMotorsAuton(40);
   delay(800);
   assignDriveMotorsAuton(0);
   delay(200);
   assignDriveMotorsAuton(40);
   delay(1400);
   assignDriveMotorsAuton(0);
   delay(100);
   assignDriveMotorsAuton(-80);
   delay(150);
}
void dropsmall1(bool red){
   assignDriveMotorsAuton(-80);
   delay(1100);
   if(red == true){
      turnRight(80, 645);
   }else{
      turnLeft(80, 645);
   }
   autonRollers(0);
   assignDriveMotorsAuton(60);
   delay(1000);
   assignDriveMotorsAuton(0);
   delay(200);
   assignDriveMotorsAuton(-60);
   delay(60);
   assignDriveMotorsAuton(0);
   autonRollers(-60);
   delay(50);
   autonRollers(0);
   autonTilter(47);
   //assignDriveMotorsAuton(20);
   delay(500);
   autonTilter(0);
   delay(200);
   autonTilter(70);
   delay(800);
   delay(200);
   //autonRollers(-45);
   delay(400);
   delay(800);
   assignDriveMotorsAuton(-30);
   delay(2000);
   assignDriveMotorsAuton(0);
   
}
void pickupCube8(bool red){
   flipout();
   pickupCubes5_8_1();
   if(red == true){
      turnLD(100, 80, true);
   }else{
      turnRD(195, 80, true);
   }
   assignDriveMotorsAuton(-127);
   delay(900);
   assignDriveMotorsAuton(-50);
   delay(800);
   pickupCubes5_8_2();
   dropsmallTicks6(red);
}
void dropsmallTicks8(bool red){
   assignDriveMotorsAuton(-80);
   delay(700);
   if(red == true){
      turnRD(830, 55, true);
   }else{
      turnLD(860, 55, true);
   }
   autonRollers(0);
   assignDriveMotorsAuton(80);
   delay(1700);
   assignDriveMotorsAuton(0);
   delay(75);
   assignDriveMotorsAuton(-60);
   delay(80);
   assignDriveMotorsAuton(0);
   autonRollers(-60);
   delay(100);
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
   delay(1000);
   assignDriveMotorsAuton(-30);
   delay(1000);
   assignDriveMotorsAuton(0);
   autonTilter(-127);
   delay(800);
   autonTilter(0);
}
void knockOffOne(bool red){

   flipout();
   autonRollers(127);
   delay(200);
   assignDriveMotorsAuton(60);
   //delay(2000);
   //assignDriveMotorsAuton(0);
   delay(1500);
   assignDriveMotorsAuton(0);
   delay(300);
   assignDriveMotorsAuton(50);
   delay(400);
   assignDriveMotorsAuton(0);
   delay(800);
   assignDriveMotorsAuton(-100);
   delay(500);
   assignDriveMotorsAuton(0);
   
   delay(300);
   assignDriveMotorsAuton(0);
   delay(500);
   autonRollers(0);


   //turn to drop single cube
   if (red == true)
   {
     turnLD(400, 80, true);
   }
   else
   {
     turnRD(400, 80, true);
   }
   autonRollers(-127);
   delay(1000);
   autonRollers(0);
   if (red == true)
   {
     turnRD(400, 80, true);
   }
   else
   {
     turnLD(400, 80, true);
   }



   //forward to stack of 4
   //delay(150);


   pickupStack();
   pickStack();

   assignDriveMotorsAuton(-80);
   delay(200);
   assignDriveMotorsAuton(0);

   delay(150);

   //correctional turn
   if(red == true)
   {
     turnLD(100, 50, true);
   }
   else
   {
     turnRD(100, 50, true);
   }

   assignDriveMotorsAuton(-80);
   delay(400);
   assignDriveMotorsAuton(0);
   delay(200);
   if(red == true){
      turnLD(205, 80, true);
   }else{
      turnRD(180, 80, true);
   }
   assignDriveMotorsAuton(60);
   delay(500);
   assignDriveMotorsAuton(0);
   delay(200);
   assignDriveMotorsAuton(-45);
   delay(500);
   assignDriveMotorsAuton(0);
   if(red == true){
      turnRD(180, 80, true);
   }else{
      turnLD(200, 80, true);
   }

   assignDriveMotorsAuton(0);
   delay(100);
   assignDriveMotorsAuton(-80);
   delay(100);
   assignDriveMotorsAuton(0);
   assignDriveMotorsAuton(-60);

   //delay(150);
   //assignDriveMotorsAuton(-80);
   delay(1150);
   assignDriveMotorsAuton(0);


   delay(100);
   if (red == true)
   {
     turnLD(600, 90, true);
   }
   else
   {
     turnRD(450, 90, true);
   }
   assignDriveMotorsAuton(70);
   delay(1000);
   assignDriveMotorsAuton(0);
   delay(75);


   if (red == true)
   {
     turnLD(350, 60, true);
   }
   else
   {
     turnRD(350, 60, true);
   }
   assignDriveMotorsAuton(50);
   delay(1100);
   assignDriveMotorsAuton(0);
   delay(100);

   assignDriveMotorsAuton(-50);
   delay(130);
   assignDriveMotorsAuton(0);
   delay(100);
   //5 cube drop
   autonRollers(-60);
   delay(350);
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
void pickupCube(){
   delay(200);
   autonRollers(127);
   assignDriveMotorsAuton(127);
   delay(300);
   assignDriveMotorsAuton(40);
   delay(800);
   assignDriveMotorsAuton(0);
   delay(200);
   assignDriveMotorsAuton(40);
   delay(1000);
   assignDriveMotorsAuton(0);
   delay(300);
   assignDriveMotorsAuton(-80);
   delay(150);
   assignDriveMotorsAuton(0);
}
void dropsmall(bool red){
   assignDriveMotorsAuton(-80);
   delay(870);
   if(red == true){
      turnRight(80, 615);
   }else{
      turnLeft(80, 615);
   }
   autonRollers(0);
   assignDriveMotorsAuton(60);
   delay(1000);
   assignDriveMotorsAuton(0);
   delay(200);
   assignDriveMotorsAuton(-60);
   delay(80);
   assignDriveMotorsAuton(0);
   autonRollers(-60);
   delay(300);
   autonRollers(0);
   autonTilter(60);
   //assignDriveMotorsAuton(20);
   delay(500);
   autonTilter(0);
   delay(200);
   autonTilter(45);
   delay(800);
   delay(200);
   //autonRollers(-45);
   delay(400);
   delay(800);
   assignDriveMotorsAuton(-30);
   delay(2000);
   autonTilter(0);
   assignDriveMotorsAuton(0);
}

void pickupCube2 (bool red){
   flipout();
   autonRollers(127);
   assignDriveMotorsAuton(127);
   delay(500);
   assignDriveMotorsAuton(0);
   if(red == true){
      turnRight(70, 500);
   }else{
      turnLeft(70, 500);
   }
   autonRollers(-127);
   delay(500);
   if(red == true){
      turnLeft(70, 500);
   }else{
      turnRight(70, 500);
   }
   assignDriveMotorsAuton(-127);
   delay(600);
   assignDriveMotorsAuton(0);
   delay(200);
   autonTilter(127);
   assignDriveMotorsAuton(70);
   delay(500);
   autonTilter(0);
   assignDriveMotorsAuton(40);
   delay(500);
   autonTilter(-60);
   autonRollers(50);
   delay(500);
   autonTilter(0);
   assignDriveMotorsAuton(-127);
   delay(1000);
   assignDriveMotorsAuton(0);
   delay(300);
   assignDriveMotorsAuton(127);
   delay(300);
   assignDriveMotorsAuton(0);
   if(red == true){
      turnLeft(70, 500);
   }else{
      turnRight(70, 500);
   }
   delay(100);
   assignDriveMotorsAuton(127);
   delay(800);
   assignDriveMotorsAuton(0);
   autonRollers(-60);
   delay(300);
   autonRollers(0);
   autonTilter(60);
   //assignDriveMotorsAuton(20);
   delay(500);
   autonTilter(0);
   delay(200);
   autonTilter(45);
   delay(800);
   delay(200);
   //autonRollers(-45);
   delay(400);
   delay(800);
   assignDriveMotorsAuton(-30);
   delay(2000);
   autonTilter(0);
   assignDriveMotorsAuton(0);

}
void dropBig1(bool red){
   delay(800);
   assignDriveMotorsAuton(-100);
   delay(500);
   assignDriveMotorsAuton(0);
   if(red == true){
      turnRD(650, 127, true);
   }else{
      turnLD(650, 127, true);
   }
   assignDriveMotorsAuton(127);
   delay(600);
   assignDriveMotorsAuton(0);
   if(red == true){
      turnRD(350, 127, true);
   }else{
      turnLD(350, 127, true);
   }
   assignDriveMotorsAuton(127);
   delay(1500);
   assignDriveMotorsAuton(0);
   delay(200);
   autonTilter(47);
   delay(500);
   autonTilter(0);
   delay(200);
   autonTilter(70);
   delay(800);
   delay(200);
   autonTilter(0);
   assignDriveMotorsAuton(-30);
   delay(2000);
   assignDriveMotorsAuton(0);
}
void dropsmall2(bool red){
   assignDriveMotorsAuton(-80);
   delay(1600);
   if(red == true){
      turnRight(80, 675);
   }else{
      turnLeft(80, 675);
   }
   autonRollers(0);
   assignDriveMotorsAuton(60);
   delay(1000);
   assignDriveMotorsAuton(0);
   delay(200);
   assignDriveMotorsAuton(-60);
   delay(60);
   assignDriveMotorsAuton(0);
   autonRollers(-60);
   delay(50);
   autonRollers(0);
   autonTilter(47);
   //assignDriveMotorsAuton(20);
   delay(500);
   autonTilter(0);
   delay(200);
   autonTilter(70);
   delay(800);
   delay(200);
   //autonRollers(-45);
   delay(400);
   delay(800);
   autonArm(127);
   delay(100);
   autonArm(0);
   assignDriveMotorsAuton(-30);
   delay(2000);
   autonTilter(0);
   assignDriveMotorsAuton(0);
}
void pickupCubeBig(bool red){
   flipout();
   pickupCubes();
   assignDriveMotorsAuton(-127);
   delay(350);
   assignDriveMotorsAuton(0);
   delay(200);
   if(red == true){
      turnLeft(127, 202);
   }else{
      turnRight(127, 202);
   }
   assignDriveMotorsAuton(-70);
   delay(800);
   if(red == true){
      turnRight(127, 150);
   }else{
      turnLeft(127, 150);
   }
   assignDriveMotorsAuton(-70);
   delay(1000);
   assignDriveMotorsAuton(0);
   pickupCubes();
   dropsmall1(red);
}*/



