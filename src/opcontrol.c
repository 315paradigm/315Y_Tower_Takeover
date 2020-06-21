#include "main.h"
#include "config.h"
#include <stdio.h>
#include <inttypes.h>
extern void initializeDriveMotors();
 #define max(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a > _b ? _a : _b; })

 #define min(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a < _b ? _a : _b; })

extern adi_gyro_t gyro;

/*void assignDriveMotors(int leftSide, int rightSide){
    motor_move(PORT_DRIVELEFTFRONT, leftSide);
    motor_move(PORT_DRIVELEFTBACK, leftSide);
    motor_move(PORT_DRIVERIGHTFRONT, rightSide);     
    motor_move(PORT_DRIVERIGHTBACK, rightSide);
}*/

void drive(void* param){
    while (true) {
        int forward = -controller_get_analog(CONTROLLER_MASTER, ANALOG_RIGHT_X);
        int turn = controller_get_analog(CONTROLLER_MASTER, ANALOG_LEFT_Y);
        
        motor_move(PORT_DRIVELEFTFRONT, max(-127, min(127, forward + turn)));
        motor_move(PORT_DRIVERIGHTFRONT, -max(-127, min(127, forward - turn)));
        motor_move(PORT_DRIVELEFTBACK, max(-127, min(127, forward + turn)));
        motor_move(PORT_DRIVERIGHTBACK, -max(-127, min(127, forward - turn)));
        delay(20);
    }
    delay(10);
}
void slowStack(){
    motor_move_velocity(PORT_TILTER, 200);
    delay(700);
    motor_move_velocity(PORT_TILTER, 120);
    delay(600);
    motor_move_velocity(PORT_TILTER, 70);
    delay(2700);
    motor_move_velocity(PORT_TILTER, 0);
}
void fastStack(){
    motor_move(PORT_ROLLERLEFT, 30);
    motor_move(PORT_ROLLERRIGHT, -30);
    motor_move_velocity(PORT_TILTER, 200);
    delay(900);
    
    motor_move_velocity(PORT_TILTER, 150);
    delay(600);
    motor_move(PORT_ROLLERLEFT, 0);
    motor_move(PORT_ROLLERRIGHT, 0);
    motor_move_velocity(PORT_TILTER, 65);
    delay(4000);
    motor_move_velocity(PORT_TILTER, 0);
}
void tilter(void* param){
    while (true) {
        if (controller_get_digital(CONTROLLER_MASTER, DIGITAL_X) || controller_get_digital(CONTROLLER_PARTNER, DIGITAL_X)){
            motor_move(PORT_TILTER, 80);
            while (controller_get_digital(CONTROLLER_MASTER, DIGITAL_X) || controller_get_digital(CONTROLLER_PARTNER, DIGITAL_X)){

            }
            motor_move(PORT_TILTER, 0);
        }else if (controller_get_digital(CONTROLLER_MASTER, DIGITAL_B) || controller_get_digital(CONTROLLER_PARTNER, DIGITAL_B)){
            motor_move(PORT_TILTER, -127);
            delay(1000);
            motor_move(PORT_TILTER, 0);
        }
        if(controller_get_digital(CONTROLLER_PARTNER, DIGITAL_LEFT)){
            slowStack();
        }
        /*if(controller_get_digital(CONTROLLER_MASTER, DIGITAL_RIGHT) || controller_get_digital(CONTROLLER_PARTNER, DIGITAL_RIGHT)){
            motor_move(PORT_TILTER, 127);
            delay(2000);
            motor_move(PORT_TILTER, 100);
            delay(500);
            motor_move(PORT_TILTER, 50);
            delay(1000);
            motor_move(PORT_TILTER, 0);
        }*/
        
        if(controller_get_digital(CONTROLLER_MASTER, DIGITAL_DOWN) || controller_get_digital(CONTROLLER_PARTNER, DIGITAL_DOWN)){
            
            fastStack();
            
        }
        
        if(controller_get_digital(CONTROLLER_MASTER, DIGITAL_RIGHT) || controller_get_digital(CONTROLLER_PARTNER, DIGITAL_RIGHT)){
            motor_tare_position(PORT_TILTER);
            motor_move_velocity(PORT_TILTER, 200);
            delay(600);
            motor_move_absolute(PORT_TILTER, 3350, 100);
        }
        delay(20);

    }
    delay(10); 
}
void rollers(void* param){
    int intakeDirection = 0;
    while (true) {
        if (controller_get_digital(CONTROLLER_MASTER, DIGITAL_R1) || controller_get_digital(CONTROLLER_PARTNER, DIGITAL_R1)){
            if (intakeDirection != 1){
                motor_move(PORT_ROLLERLEFT, 127);
                motor_move(PORT_ROLLERRIGHT, -127);
                intakeDirection = 1;
            }else {
                motor_move(PORT_ROLLERLEFT, 0);
                motor_move(PORT_ROLLERRIGHT, 0);
                intakeDirection = 0;
            }
            delay(350);
        }

        if (controller_get_digital(CONTROLLER_MASTER, DIGITAL_R2) || controller_get_digital(CONTROLLER_PARTNER, DIGITAL_R2)){
            if (intakeDirection != -1){
                motor_move(PORT_ROLLERLEFT,-127);
                motor_move(PORT_ROLLERRIGHT, 127);
                intakeDirection = -1;
            }else {
                motor_move(PORT_ROLLERLEFT, 0);
                motor_move(PORT_ROLLERRIGHT, 0);
                intakeDirection = 0;
            }
            delay(350);
        }
        if (controller_get_digital(CONTROLLER_PARTNER, DIGITAL_A)){
            if(intakeDirection != 2){
                motor_move(PORT_ROLLERLEFT, 65);
                motor_move(PORT_ROLLERRIGHT, -65);
                intakeDirection = 2;
            }else{
                motor_move(PORT_ROLLERLEFT, 0);
                motor_move(PORT_ROLLERRIGHT, 0);
                intakeDirection = 0;
            } 
            delay(350);  
        }
        if (controller_get_digital(CONTROLLER_MASTER, DIGITAL_Y) || controller_get_digital(CONTROLLER_PARTNER, DIGITAL_Y)){
            motor_move(PORT_ROLLERLEFT, -65);
            motor_move(PORT_ROLLERRIGHT, 65);
            delay(250);
            motor_move(PORT_ROLLERLEFT, 30);
            motor_move(PORT_ROLLERRIGHT, -30);
            delay(300);
            motor_move(PORT_ROLLERLEFT, 0);
            motor_move(PORT_ROLLERRIGHT, 0);
            intakeDirection = 0;

        }
        if(controller_get_digital(CONTROLLER_MASTER, DIGITAL_LEFT)){
            motor_move(PORT_ROLLERLEFT, -127);
            motor_move(PORT_ROLLERRIGHT, 127);
            delay(700);
            motor_move(PORT_ROLLERLEFT, 127);
            motor_move(PORT_ROLLERRIGHT, -127);
            delay(600);
            motor_move(PORT_ROLLERLEFT, 0);
            motor_move(PORT_ROLLERRIGHT, 0);
        }
        
        delay(20);
    }
}
void arm(void* param){
    while (true) {
        if (controller_get_digital(CONTROLLER_MASTER, DIGITAL_L1) || controller_get_digital(CONTROLLER_PARTNER, DIGITAL_L1)){
            motor_move(PORT_ARMLEFT, 127);
            
            while (controller_get_digital(CONTROLLER_MASTER, DIGITAL_L1) || controller_get_digital(CONTROLLER_PARTNER, DIGITAL_L1)){
            }
            motor_move(PORT_ARMLEFT, 20);
            //motor_move(PORT_ARMRIGHT, 0);
        }else if (controller_get_digital(CONTROLLER_MASTER, DIGITAL_L2) || controller_get_digital(CONTROLLER_PARTNER, DIGITAL_L2)){
            motor_move(PORT_ARMLEFT, -127);
            //motor_move(PORT_ARMRIGHT, 127); 
            while (controller_get_digital(CONTROLLER_MASTER, DIGITAL_L2) || controller_get_digital(CONTROLLER_PARTNER, DIGITAL_L2)){

            }
            motor_move(PORT_ARMLEFT, -10);
            //motor_move(PORT_ARMRIGHT, 0);
        }
        if(controller_get_digital(CONTROLLER_MASTER, DIGITAL_UP) || controller_get_digital(CONTROLLER_PARTNER, DIGITAL_UP)){
            motor_move(PORT_ARMLEFT, 127);
            delay(1700);
            motor_move(PORT_ARMLEFT, 15);
        }
        if(controller_get_digital(CONTROLLER_MASTER, DIGITAL_A)){
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

        }
        delay(10);
    }
}
void test_motors(){
    while (true) {
        if(controller_get_digital(CONTROLLER_MASTER, DIGITAL_X)){
            motor_move(PORT_DRIVELEFTFRONT, -127);
            delay(2000);
            motor_move(PORT_DRIVELEFTFRONT, 10);
            delay(50);
            motor_move(PORT_DRIVELEFTFRONT, 0);
        } else if(controller_get_digital(CONTROLLER_MASTER, DIGITAL_A)){
            motor_move(PORT_DRIVERIGHTFRONT, -127);
            delay(2000);
            motor_move(PORT_DRIVERIGHTFRONT, 10);
            delay(50);
            motor_move(PORT_DRIVERIGHTFRONT, 0);
        } else if(controller_get_digital(CONTROLLER_MASTER, DIGITAL_B)){
            motor_move(PORT_DRIVERIGHTBACK, -127);
            delay(2000);
            motor_move(PORT_DRIVERIGHTBACK, 10);
            delay(50);
            motor_move(PORT_DRIVERIGHTBACK, 0);
        } else if(controller_get_digital(CONTROLLER_MASTER, DIGITAL_Y)){
            motor_move(PORT_DRIVELEFTBACK, -127);
            delay(2000);
            motor_move(PORT_DRIVELEFTBACK, 10);
            delay(50);
            motor_move(PORT_DRIVELEFTBACK, 0);
        }
    }
}
void displayInfo(void *param)
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
void opcontrol() {
  //  initializeDriveMotors();
    task_t driveTask = task_create(drive, "PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Drive Task");
    task_t armTask = task_create(arm, "PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Arm Task");
    task_t rolTask = task_create(rollers, "PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Rol Task"); 
    task_t tilterTask = task_create(tilter, "PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Tilter Task");
    task_t displayInfoTask = task_create(displayInfo, "PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Display Info Task");
}