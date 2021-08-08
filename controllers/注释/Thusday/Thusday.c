#include <windows.h>
#include <webots/robot.h>
#include <webots/motor.h>
#include <stdio.h>
#include <math.h>
#include <webots/position_sensor.h>
/*
 * You may want to add macros here.
 */
#define TIME_STEP 4
#define pi 3.1415926
static WbDeviceTag servo[2];
static WbDeviceTag sensor[2];
const char *SERVO_NAMES[2] ={"motor1", "motor2"};
const char *SENSOR_NAMES[2] ={"sensor1", "sensor2"};
double x,y;

            
int main(int argc, char **argv)
{
  wb_robot_init();
    for(int i=0;i<2;i++)
    {
     servo[i]=wb_robot_get_device(SERVO_NAMES[i]);
     sensor[i]=wb_robot_get_device(SENSOR_NAMES[i]);
     }
     wb_motor_set_velocity(servo[1], 1);
     wb_position_sensor_enable(sensor[0],16);
     double t_trot=0.0;
     float para[5]={0.1,0.05,0.05,0.05,0.05};         
while (wb_robot_step(TIME_STEP) != -1) {
  //double sensorr=wb_position_sensor_get_value(sensor[10]);
  // double pos=wb_position_sensor_get_value(sensor[0]);
  // printf("%f\n",pos);
  // if(pos<0.3)
  // { wb_motor_set_position(servo[0],0.3);}
  // else wb_motor_set_torque(servo[0],-1);
  if (t_trot>2) t_trot = 0;
    x = -para[2]*cos(pi*t_trot);
  if (t_trot>=0&&t_trot<=1){
     y = para[0]-para[1]*sin(pi*t_trot);
  }
  else{
     y = para[0]-0;
    }
wb_motor_set_position(servo[0],atan(x/y));
wb_motor_set_position(servo[1],sqrt(x*x+y*y));
  t_trot=t_trot+para[4];
}

 
  wb_robot_cleanup();

  return 0;
}
