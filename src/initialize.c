#include "main.h"
#include "config.h"

int autonNumber = 0;
bool redAlliance = false;
int32_t a;
adi_gyro_t gyro;

#define GYRO_MULTIPLIER 1.0
#define GYROPORT  1
#define NUMAUTONS 5

static const char *btnm_map[] = {"ONECUBE", "SMALL-5", "SMALL-8", "\n", 
                                 "EVAN", "NONE", ""};
static const char *auton_strings[] = {"ONECUBE", "SMALL-5", "SMALL-8", "EVAN", "NONE", ""};
static const char *alliance_map[] = {"Red", "Blue", ""};

static lv_res_t btnm_action(lv_obj_t *btnm, const char *txt)
{
   autonNumber = 0;
   for (int i = 0; i < sizeof(auton_strings) / sizeof(auton_strings[0]); i++)
   {
      printf("%s\n", auton_strings[i]);
      printf("%s\n", txt);
      printf("-----------\n");
      if (strcmp(auton_strings[i], txt) == 0)
      {
         autonNumber = i + 1;
         break;
      }
      lv_btnm_set_toggle(btnm, true, autonNumber);
   }

   return LV_RES_OK; /*Return OK because the button matrix is not deleted*/
}

static lv_res_t btnm_action_color(lv_obj_t *btnm, const char *txt)
{
   redAlliance = false;
   lv_btnm_set_toggle(btnm, true, 1);
   lv_btnm_set_toggle(btnm, true, 2);
   printf("FUNCTION CALLED");
   if (strcmp(txt, "Red") == 0)
   {
      redAlliance = true;
   }
   else if (strcmp(txt, "Blue") == 1)
   {
      redAlliance = false;
   }

   return LV_RES_OK; /*Return OK because the button matrix is not deleted*/
}
void initializeDriveMotors(){
    motor_set_gearing(PORT_DRIVELEFTFRONT, E_MOTOR_GEARSET_18);
    motor_set_gearing(PORT_DRIVERIGHTFRONT, E_MOTOR_GEARSET_18);
    motor_set_gearing(PORT_DRIVELEFTBACK, E_MOTOR_GEARSET_18);
    motor_set_gearing(PORT_DRIVERIGHTBACK, E_MOTOR_GEARSET_18);
    motor_set_reversed(PORT_DRIVELEFTFRONT, true);
    motor_set_reversed(PORT_DRIVERIGHTFRONT, true);
    motor_set_reversed(PORT_DRIVELEFTBACK, true);
    motor_set_reversed(PORT_DRIVERIGHTBACK, true);
}

void initialize(){
    motor_set_reversed(PORT_DRIVERIGHTFRONT, true);
    motor_set_reversed(PORT_DRIVERIGHTBACK, true);
   motor_set_gearing(PORT_DRIVERIGHTBACK, E_MOTOR_GEARSET_36);
    gyro = adi_gyro_init((uint8_t)GYROPORT, (double)GYRO_MULTIPLIER);
}

void disabled() {}

void competition_initialize()
{
   lv_theme_alien_init(40, NULL);

   lv_obj_t *title = lv_label_create(lv_scr_act(), NULL);
   lv_label_set_text(title, "Auton Selection");
   lv_obj_align(title, NULL, LV_ALIGN_IN_TOP_MID, 0, 10);

   lv_obj_t *btnm = lv_btnm_create(lv_scr_act(), NULL);
   lv_btnm_set_map(btnm, btnm_map);
   lv_btnm_set_action(btnm, btnm_action);
   lv_obj_set_size(btnm, LV_HOR_RES - 40, LV_VER_RES / 3);
   lv_obj_align(btnm, title, LV_ALIGN_OUT_BOTTOM_MID, 0, 20);

   lv_obj_t *allianceM = lv_btnm_create(lv_scr_act(), NULL);
   lv_btnm_set_map(allianceM, alliance_map);
   lv_btnm_set_action(allianceM, btnm_action_color);
   lv_obj_set_size(allianceM, LV_HOR_RES - 40, 50);
   lv_obj_align(allianceM, btnm, LV_ALIGN_OUT_BOTTOM_MID, 0, 20);

    gyro = adi_gyro_init((uint8_t)GYROPORT, (double)GYRO_MULTIPLIER);
}

