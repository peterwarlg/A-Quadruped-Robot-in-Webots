/*
 * File:          wzt.c
 * Date:
 * Description:
 * Author:
 * Modifications:
 */
#include <windows.h>
#include <webots/robot.h>
#include <webots/motor.h>
#include <stdio.h>
#include <math.h>
#include "set.c"
#include "kinematic.c"
#include <webots/position_sensor.h>
/*
 * You may want to add macros here.
 */
#define TIME_STEP 4

void get_servosensor();

static WbDeviceTag servo[4][3];
static WbDeviceTag sensor[4][3];

double foot_to_kuan[4][3]={
                 {0,-0.20,0},//左前
                 {0,-0.20,0},//左后
                 {0,-0.20,0},//右前
                 {0,-0.20,0}};
                 
int main(int argc, char **argv)
{
  wb_robot_init();
  get_servosensor();

// //*************设置速度********/
  // for(int i=0;i<12;i++)
  // {
   // wb_motor_set_velocity(servo[i], 1);
   // wb_position_sensor_enable(sensor[i],16);
  // } 
//*************设置速度********/ 
              
while (wb_robot_step(TIME_STEP) != -1) {
  //double sensorr=wb_position_sensor_get_value(sensor[10]);
  //get_position();
trot(trot_para);
//inverse(foot_to_kuan);
write_pos(angel);
//write_pos(angel);
  // for(int i=0;i<4;i++)
  // {
  // wb_motor_set_torque(servo[i][2],1);
  // }
  // for(int i=0;i<4;i++)
  // {
  // wb_motor_set_torque(servo[1][0],1);
  // wb_motor_set_torque(servo[i][1],0.2);
  // wb_motor_set_torque(servo[i][2],0.2);
  // }
  //printf("%f\n",set_position[3][0]);
}

 
  wb_robot_cleanup();

  return 0;
}
