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
    {"FL_motor1", "FL_motor2", "FL_motor3"},
    {"FR_motor1", "FR_motor2", "FR_motor3"},
    {"BL_motor1", "BL_motor2", "BL_motor3"},
    {"BR_motor1", "BR_motor2", "BR_motor3"}};
    
const char *SENSOR_NAMES[4][3] = { 
    {"FL_sensor1", "FL_sensor2", "FL_sensor3"},
    {"FR_sensor1", "FR_sensor2", "FR_sensor3"},
    {"BL_sensor1", "BL_sensor2", "BL_sensor3"},
    {"BR_sensor1", "BR_sensor2", "BR_sensor3"}};
    
    
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
