/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/differential_wheels.h>, etc.
 */
#include <webots/robot.h>
#include <webots/keyboard.h>
#include <webots/motor.h>
#include <webots/gyro.h>
#include <webots/inertial_unit.h>
#include <webots/touch_sensor.h>
#include <webots/gps.h>
#include <assert.h>
#include <math.h>
#include <stdio.h>

double x,y,z;  //足端相对髋关节（侧摆关节位置） 
double body_RPY[3] = {0.0,0.0,0.0};		//机身姿态角 
double body_XYZ[3] = {0.0,0.0,0.0};		//机身中心位置 
int isTouch[4] = {0,0,0,0};				//足端传感器 
double CG[3];
double t_walk = 0;						//walk步态周期		
double sx = 0;
double sy = 0;
double sz = 0;
double IMU[3] = {0,0,0};			//IMU数据,r,p,y三个角度 


/*------这两个变量是四条腿的足端位置，逆解之后就可得到关节角（逆解函数是robot.c最后两个函数）-------*/ 
double FootendPosition_self[4][3];
double FootendPosition[4][3] = {
  {0,-350.0,0},
  {0,-350.0,0},
  {0,-350.0,0},
  {0,-350.0,0}
  };
/*-----要写新的步态或者运动的话就可以直接从这下手----*/


#include "myMatrix.c"
#include "zitai_control.c"
#include "robot.c"
/*
 * You may want to add macros here.
 */
#define TIME_STEP 16

void printf_xyzRPY(WbDeviceTag gps,WbDeviceTag imu,double t)
{
     printf(", %6.4f, %6.4f, %6.4f,",
        wb_gps_get_values(gps)[0],
        wb_gps_get_values(gps)[1],
        wb_gps_get_values(gps)[2]);
     printf(" %6.4f, %6.4f, %6.4f, %6.4f\n",
        wb_inertial_unit_get_roll_pitch_yaw(imu)[0],
        wb_inertial_unit_get_roll_pitch_yaw(imu)[1],
        wb_inertial_unit_get_roll_pitch_yaw(imu)[2],t);
}

int main(int argc, char **argv)
{
  /* necessary to initialize webots stuff */
//电机和足端的传感器定义，注意这些是name属性，不是节点的DEF//
  WbDeviceTag servos[4][3];
  const char *SERVO_NAMES[4][3] = { 
    {"FL_MOTOR1", "FL_MOTOR2", "FL_MOTOR3"},
    {"FR_MOTOR1", "FR_MOTOR2", "FR_MOTOR3"},
    {"BR_MOTOR1", "BR_MOTOR2", "BR_MOTOR3"},
    {"BL_MOTOR1", "BL_MOTOR2", "BL_MOTOR3"}
    };
    
  WbDeviceTag touchsensor[4];
  const char *TOUCH_NAMES[4] = { 
    "touch sensor_FL", "touch sensor_FR", "touch sensor_BR","touch sensor_BL"
    };
//电机和足端的传感器定义，注意这些是name属性，不是节点的DEF//    
 
      
  double ServoPos_FL[3];
  double ServoPos_FR[3];
  double ServoPos_HR[3];
  double ServoPos_HL[3];
  
  
  /*
  this is the real robot's rotate ServoDirection
  const double ServoDirect[12] = {
  
  */
  
  //this is the virtual robot's rotate ServoDirection in webots world
  const double ServoDirect[4][3] = {
  {1,-1,-1},
  {1,-1,-1},
  {1,-1,-1},
  {1,-1,-1} 
  };
  
  double t = 0.0;

  wb_robot_init();
  
  
  /*
   * You should declare here WbDeviceTag variables for storing
   * robot devices like this:
   *  WbDeviceTag my_sensor = wb_robot_get_device("my_sensor");
   *  WbDeviceTag my_actuator = wb_robot_get_device("my_actuator");
   */
  int i,j;
  for (i = 0; i<4; i++) 
  {
    for(j = 0; j<3; j++)
    {
       servos[i][j] = wb_robot_get_device(SERVO_NAMES[i][j]);//获取电机 
       wb_motor_set_velocity(servos[i][j],10);               //设置电机速度，当前为10 
    }
  }

	/************获取足端传感器，IMU，gps,键盘使能******/ 
  for(i=0;i<4;i++)
  {
    touchsensor[i] = wb_robot_get_device(TOUCH_NAMES[i]);
    assert(touchsensor[i]);
    wb_touch_sensor_enable(touchsensor[i],10);
  }
  
  WbDeviceTag imu = wb_robot_get_device("imu");
  assert(imu);  
  wb_inertial_unit_enable(imu,10);
  
  
  WbDeviceTag gps = wb_robot_get_device("gps");
  assert(gps);
  wb_gps_enable(gps,10);
  
  wb_keyboard_enable(TIME_STEP);
	/************获取足端传感器，IMU，gps,键盘使能******/  
	 
  /* main loop
   * Perform simulation steps of TIME_STEP milliseconds
   * and leave the loop when the simulation is over
   */
  printf(" Click 3D Windows,use keyboard to control robot\n");
 
  while (wb_robot_step(TIME_STEP) != -1) {

    new_key = wb_keyboard_get_key();
    if (new_key != prev_key) {
    	//这里的switch选项是不全的，全部的在robot.c里 
      switch (new_key) {
      case WB_KEYBOARD_UP:
        // increase amplitude
        body_XYZ[1] += 0.005;
        body_XYZ[1] = body_XYZ[1] > 0.1 ? 0.1 : (body_XYZ[1] < -0.2 ? -0.2 : body_XYZ[1]);
        printf("body_XYZ[1] = %f\n", body_XYZ[1]);
        break;
      case WB_KEYBOARD_DOWN:
        // decrease amplitude
        body_XYZ[1] -= 0.005;
        body_XYZ[1] = body_XYZ[1] > 0.1 ? 0.1 : (body_XYZ[1] < -0.2 ? -0.2 : body_XYZ[1]);
        printf("body_XYZ[1] = %f\n", body_XYZ[1]);
        break;
      case 'V':
        printf("Roll = %4.2f,Pitch = %4.2f,Yaw = %4.2f\n",
        wb_inertial_unit_get_roll_pitch_yaw(imu)[0],
        wb_inertial_unit_get_roll_pitch_yaw(imu)[1],
        wb_inertial_unit_get_roll_pitch_yaw(imu)[2]);

        //matrix_printf(ServoPos_FL,1,3);
        //matrix_printf(ServoPos_FR,1,3);
        //matrix_printf(ServoPos_HR,1,3);
        //matrix_printf(ServoPos_HL,1,3);
        break;
      case 'R':
        printf("RUN\n");
        RunMode = RUN;
        break;
      case 'C':
        printf("ZTCONTROL\n");
        RunMode = ZTCONTROL;
        break;
      case 'S':
        printf("STOP\n");
        RunMode = STOP;
        break;
      case 'Q':
        printf("TEST\n");
        RunMode = TEST;
        break;
      case 'Z':
        printf("WALK\n");
        RunMode = WALK;
        break;
      }
      prev_key = new_key;
    }
    

     if(t<4*T)
     {get_FootEndPosition(FootendPosition_self, t, SETUP);}
     else
     {get_FootEndPosition(FootendPosition, t, RunMode);}
     
	 
	 /*************四条腿逆解，由足端相对侧摆位置求解关节角*****/ 
     //matrix_printf(FootendPosition_self,4,3);
     //FL???
     get_foreleg_ServoPosition(ServoPos_FL,
     FootendPosition_self[0][0],FootendPosition_self[0][1],FootendPosition_self[0][2]);
     //FR???
     get_foreleg_ServoPosition(ServoPos_FR,
     FootendPosition_self[1][0],FootendPosition_self[1][1],FootendPosition_self[1][2]);
     //HR???
     get_hindleg_ServoPosition(ServoPos_HR,
     FootendPosition_self[2][0],FootendPosition_self[2][1],FootendPosition_self[2][2]);
     //HL???
     get_hindleg_ServoPosition(ServoPos_HL,
     FootendPosition_self[3][0],FootendPosition_self[3][1],FootendPosition_self[3][2]);
     /*************四条腿逆解，由足端相对侧摆位置求解关节角*****/ 
     
     
     //matrix_printf(body_XYZ,1,3);
     for(j = 0;j<3;j++)
     {
        //??????
        wb_motor_set_position(servos[0][j],ServoPos_FL[j] * ServoDirect[0][j]);
        wb_motor_set_position(servos[1][j],ServoPos_FR[j] * ServoDirect[1][j]);
        wb_motor_set_position(servos[2][j],ServoPos_HR[j] * ServoDirect[2][j]);
        wb_motor_set_position(servos[3][j],ServoPos_HL[j] * ServoDirect[3][j]);
     }
     //printf("ServoPos[0-2]: %4.2f,%4.2f,%4.2f\n",ServoPos_FL[0],ServoPos_FL[1],ServoPos_FL[2]);
     //printf("FootendPos[0-2]: %4.2f,%4.2f,%4.2f\n",FootendPosition[0][0],FootendPosition[0][1],FootendPosition[0][2]);
     //wb_robot_step(TIME_STEP);
     //printf("t=%4.2f\n",t);
     t += (double)TIME_STEP / 1000.0 ;
     if(RunMode == WALK)
     {t_walk += (double)TIME_STEP / 1000.0 ;}
     
     if(RunMode == TEST)
     {
       printf(",%7.5f,%7.5f,%7.5f,%7.5f,%7.5f,%7.5f,%7.5f\n",
      wb_inertial_unit_get_roll_pitch_yaw(imu)[0],
      wb_inertial_unit_get_roll_pitch_yaw(imu)[1],
      wb_inertial_unit_get_roll_pitch_yaw(imu)[2],
      wb_gps_get_values(gps)[0],
      wb_gps_get_values(gps)[1],
      wb_gps_get_values(gps)[2],
      t);
     }
     
     //printf("t_walk = %4.2f\n",t_walk);
     //matrix_printf(body_XYZ,1,3);
     //printf("FootendPosition=\n");
     //matrix_printf(FootendPosition,4,3);
     for(i=0;i<3;i++)
     {
       IMU[i] = wb_inertial_unit_get_roll_pitch_yaw(imu)[i];
     }
     for(i=0;i<4;i++)
     {
       isTouch[i] = wb_touch_sensor_get_value(touchsensor[i]);
     }

     //printf("%d,%d,%d,%d\n",isTouch[0],isTouch[1],isTouch[2],isTouch[3]);
     //matrix_printf(FootendPosition_self,4,3);
     //printf(" ,%6.4f,%6.4f,%6.4f,%6.4f\n",
     //ServoPos_FL[0],ServoPos_FR[0],ServoPos_HR[0],ServoPos_HL[0]);
     //matrix_printf(FootendPosition_self,4,3);
  };
  
  /* Enter your cleanup code here */
  
  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();
  
  return 0;
}

