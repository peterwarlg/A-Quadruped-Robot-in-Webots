#ifndef kinematic_c
#define kinematic_c

#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/position_sensor.h>
#include <math.h>
#include <stdio.h>
#include "set.c"

void get_position();
#define pi 3.1415926
#define rtod 57.29578
#define dtor 0.017453

#define L 0.56
#define W 0.3
#define L1 0.25
#define L2 0.25
double kp=15,kd=0.3;
float now_position[4][3];
float goal_position[4][3];
float last_position[4][3];
static WbDeviceTag servo[4][3];
static WbDeviceTag sensor[4][3];
float t_trot=0.0;
float trot_para[5]=
{0.25,0.05,0.05,0.05,0.06};//trot参数：机身高，步高，左步长，右步长,速度
double foot_to_kuan[4][3];
double kuan_to_world[4][3] = {
                 { L/2,0,-W/2},//左前
                 {-L/2,0,-W/2},//左后
                 { L/2,0, W/2},//右前
                 {-L/2,0, W/2}}; //右后


double angel[4][3];

void inverse(double foot_to_kuan[4][3])
{
  double L3[4];
  for(int i=0;i<4;i++)
    {
      L3[i]=sqrt(pow(foot_to_kuan[i][0],2)+pow(foot_to_kuan[i][1],2)+pow(foot_to_kuan[i][2],2));
    }
  
  for(int i=0;i<4;i++)
    {
      angel[i][0]=atan(foot_to_kuan[i][2]/foot_to_kuan[i][1]);
      if(i==0||i==1)
      {
      angel[i][1]=-(acos((L1*L1+L3[i]*L3[i]-L2*L2)/(2*L1*L3[i]))-atan(foot_to_kuan[i][0]/(foot_to_kuan[i][1]/cos(angel[i][0]))));
      angel[i][2]=pi-acos((L1*L1+L2*L2-L3[i]*L3[i])/(2*L1*L2));
      }
      else if(i==2||i==3)
      {
      angel[i][1]=(acos((L1*L1+L3[i]*L3[i]-L2*L2)/(2*L1*L3[i]))-atan(foot_to_kuan[i][0]/(foot_to_kuan[i][1]/cos(angel[i][0]))));
      angel[i][2]=-pi+acos((L1*L1+L2*L2-L3[i]*L3[i])/(2*L1*L2));
      }
    }
}

void write_pos(double pos[4][3])
{
  inverse(foot_to_kuan);
  for(int i=0;i<4;i++)
    {
      for(int j=0;j<3;j++)
        {
         wb_motor_set_position(servo[i][j], pos[i][j]);
        }
    }
}

void trot(float para[5])

{
  double foot_to_kuan_trot[4][3]={{0,0,0},  //fl
                                  {0,0,0},  //fr
                                  {0,0,0},  //bl
                                  {0,0,0}}; //br
  if (t_trot>2) t_trot = 0;
  
    foot_to_kuan_trot[0][0] = -para[2]*cos(pi*t_trot);
    foot_to_kuan_trot[1][0] = -para[3]*cos(pi*t_trot+pi);
    foot_to_kuan_trot[2][0] = -para[2]*cos(pi*t_trot);
    foot_to_kuan_trot[3][0] = -para[3]*cos(pi*t_trot+pi);
  
  if (t_trot>=0&&t_trot<=1){
    foot_to_kuan_trot[0][1] = para[0]-para[1]*sin(pi*t_trot);
    foot_to_kuan_trot[1][1] = para[0]-0;
    foot_to_kuan_trot[2][1] = para[0]-0;
    foot_to_kuan_trot[3][1] = para[0]-para[1]*sin(pi*t_trot);
  }
  else{
    foot_to_kuan_trot[0][1] = para[0]-0;
    foot_to_kuan_trot[1][1] = para[0]+para[1]*sin(pi*t_trot);
    foot_to_kuan_trot[2][1] = para[0]+para[1]*sin(pi*t_trot);
    foot_to_kuan_trot[3][1] = para[0]-0;
    }
  for(int i=0;i<4;i++)
  {
  for(int j=0;j<3;j++)
  {
  foot_to_kuan[i][j]=foot_to_kuan_trot[i][j];
  }
  }
  t_trot=t_trot+para[4];
}

void write_torque(double goal[4][3])
{
  double torque[4][3];
  double max_torque=20;
  get_position();
  for(int i=0;i<4;i++)
  {
  for(int j=0;j<3;j++)
  { 
  torque[i][j]=-kp*(now_position[i][j]-goal[i][j])
  -kd*((now_position[i][j]-last_position[i][j])/0.016);
  if(torque[i][j]>max_torque) torque[i][j]=max_torque;
  else if(torque[i][j]<-max_torque) torque[i][j]=-max_torque;
  if(j==1||j==0){
  wb_motor_set_torque(servo[i][j],torque[i][j]);}
  else if(j==2){
 wb_motor_set_position(servo[i][j],goal[i][j]);}
  last_position[i][j]=now_position[i][j];
  }}
  //printf("%f,%f\n",goal[0][0],goal[1][0]);
}
#endif
