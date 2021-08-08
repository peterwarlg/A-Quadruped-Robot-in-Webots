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
//---------------------------------------------------
//足端相对髋关节位置//
double foot_to_kuan[4][3]={
                 {0,-0.20,0},//左前
                 {0,-0.20,0},//左后
                 {0,-0.20,0},//右前
                 {0,-0.20,0}};
//足端相对髋关节位置//  
//---------------------------------------------------               
int main(int argc, char **argv)
{
  wb_robot_init();
  get_servosensor();  //获取电机与传感器
              
while (wb_robot_step(TIME_STEP) != -1) {
  //double sensorr=wb_position_sensor_get_value(sensor[10]);
  //get_position();
trot(trot_para);           //计算trot步态足端位置
inverse(foot_to_kuan);     //由足端位置逆解关节角
write_torque(angel);       //力矩控制写入关节角
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
