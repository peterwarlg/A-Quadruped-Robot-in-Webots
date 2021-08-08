/*
 * File:          wzt.c
 * Date:
 * Description:
 * Author:
 * Modifications:
 */
#include <webots/robot.h>
#include <webots/motor.h>
#include <stdio.h>
#include <math.h>
#include <webots/position_sensor.h>
/*
 * You may want to add macros here.
 */
#define TIME_STEP 64
static WbDeviceTag servo;
                 
int main(int argc, char **argv)
{
  wb_robot_init();
const char *SERVO_NAMES[1] = { "FL_motor1"};
servo=wb_robot_get_device(SERVO_NAMES[0]);

// //*************设置速度********/
  // for(int i=0;i<12;i++)
  // {
    wb_motor_set_velocity(servo, 1);
   // wb_position_sensor_enable(sensor[i],16);
  // } 
//*************设置速度********/ 
 

                      
while (wb_robot_step(TIME_STEP) != -1) {
  //double sensorr=wb_position_sensor_get_value(sensor[10]);
  //get_position();
  //default_position();
  //write_pos();
  //Sleep(2000);
  // for(int i=0;i<4;i++)
  // {
  // wb_motor_set_torque(servo[i][0],0);
  // wb_motor_set_torque(servo[i][1],0.2);
  // wb_motor_set_torque(servo[i][2],0.2);
  // }
  wb_motor_set_position(servo, 0.7);
  //printf("%f\n",set_position[3][0]);
}

 
  wb_robot_cleanup();

  return 0;
}
