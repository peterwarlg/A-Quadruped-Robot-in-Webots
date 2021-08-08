#ifndef set_c
#define set_c

#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/position_sensor.h>
#include <Windows.h>
#include <math.h>
#include <stdio.h>
#include "kinematic.c"

#define pi 3.1415926
#define rtod 57.29578
#define dtor 0.017453

 
float now_position[4][3];
float goal_position[4][3];
float last_position[4][3];



//*************初始位置********/
float set_position[4][3]={{ 0,-45,90},
                          {-0,-45,90},
                          { 0,-45,90},
                          {-0,-45,90}};
//*************初始位置********/

static WbDeviceTag servo[4][3];
static WbDeviceTag sensor[4][3];

const char *SERVO_NAMES[4][3] = { 
    {"FL_motor1_Alp", "FL_motor2_Alp", "FL_motor3_Alp"},
    {"FR_motor1_Alp", "FR_motor2_Alp", "FR_motor3_Alp"},
    {"BL_motor1_Alp", "BL_motor2_Alp", "BL_motor3_Alp"},
    {"BR_motor1_Alp", "BR_motor2_Alp", "BR_motor3_Alp"}};
    
const char *SENSOR_NAMES[4][3] = { 
    {"FL_sensor1_Alp", "FL_sensor2_Alp", "FL_sensor3_Alp"},
    {"FR_sensor1_Alp", "FR_sensor2_Alp", "FR_sensor3_Alp"},
    {"BL_sensor1_Alp", "BL_sensor2_Alp", "BL_sensor3_Alp"},
    {"BR_sensor1_Alp", "BR_sensor2_Alp", "BR_sensor3_Alp"}};
    
    
//*************定义舵机********/    
void get_servosensor()
{
  for(int i=0;i<4;i++)
    {
      for(int j=0;j<3;j++)
      {
        servo[i][j]=wb_robot_get_device(SERVO_NAMES[i][j]);
        sensor[i][j]=wb_robot_get_device(SENSOR_NAMES[i][j]);
      }
    }
  for(int i=0;i<4;i++)
    {
      for(int j=0;j<3;j++)
      {
        wb_motor_set_velocity(servo[i][j], 10);
        wb_position_sensor_enable(sensor[i][j],16);
      }
    }
}

void get_position()
{
  for(int i=0;i<4;i++)
    {
    for(int j=0;j<3;j++)
      {
        now_position[i][j]=wb_position_sensor_get_value(sensor[i][j]);
      }
    }
}
//*************定义舵机********/
 void default_position()
 {
   for(int i=0;i<4;i++)
    {
    for(int j=0;j<3;j++)
      {
       wb_motor_set_position(servo[i][j], set_position[i][j]*dtor);
      }
    }
}
 

#endif
