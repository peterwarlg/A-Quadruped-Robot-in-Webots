#ifndef robot_c
#define robot_c

#define Kuan 70		// 
#define L1 200		// 三条腿的长度 
#define L2 210		// 
#define rad_2_deg(X) ( X / pi * 180.0 )
#define deg_2_rad(X) ( X / 180.0 * pi )
#define pi 3.1415926
#define T 2
#define BUCHANG 25		//步长 


#include <math.h>
#include <webots/inertial_unit.h>

/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */

enum RunMode{ RUN, BACK, STOP, ROTATE ,SETUP ,STAND, TEST,
 ROTATELEFT, ROTATERIGHT, MOVELEFT, MOVERIGHT, ZTCONTROL,WALK};
enum MoveMode{MoveLeg1,MoveLeg2,MoveLeg3,MoveLeg4,MoveBody,ReadytoMoveLeg,ReadytoMoveBody};
int new_key, prev_key = 0;
double High = 350;  //机身高度 
double ta;
double xa,ya,za;
double XYZ_s[3] = {0,0,0};
double weiyi = 0;
const double gait_phase_shift[7][4] = {
  { 0, 0.5, 0, 0.5}, //trot
  { 0, 0.5, 0.25, 0.75},//walk
  { 0, 0.1, 0.6, 0.5},//gallop  (transverse)
  { 0, 0.3, 0, 0.7}, //canter
  { 0, 0.5, 0.5, 0}, //pace
  { 0, 0, 0.5, 0.5}, //bound
  { 0, 0, 0, 0 } //pronk
};

const int Discontactleg[4] = {3,0,2,1};
//const int Discontactleg[4] = {1,0,3,2};
int disleg;
int leg = 0;
double V1 = 25, V2 = 0;         //这里的V1，V2是步高 ，具体看get_FootEndPosition这个函数 
double stride_length[4] = {BUCHANG,BUCHANG,BUCHANG,BUCHANG};

double x = 0, y = 0, z = 0;		//足端相对髋关节（侧摆关节）位置 
//这个x,y,z可以看作这个控制器的输出，所有步态的计算都是围绕这三个值，具体看get_FootEndPosition函数 
double freq = 1.0;
int RunMode = STAND;
int MoveMode = ReadytoMoveBody;


void Robot_Interpolation(double RPY_tar[3],double XYZ_tar[3],double RPY[3],double XYZ[3])
{
   int i;
   for(i=0;i<3;i++)
      {
         if(RPY_tar[i] > RPY[i])
         { body_RPY[i] += 0.01;}
         if(RPY_tar[i] < RPY[i])
         { body_RPY[i] -= 0.01;}
         if(XYZ_tar[i] > XYZ[i])
         { body_XYZ[i] += 0.0005;}
         if(XYZ_tar[i] < XYZ[i])
         { body_XYZ[i] -= 0.0005;}
      }
}

void zhitai_chabu(double t1,double t2,double t,double XYZ_0[3],double XYZ_tar[3])
{
  double ct = 0.5 * cos(pi / (t2 - t1) * (t - t1) + pi) + 1;
  int i;
  for(i = 0; i < 3; i++)
  {
    body_XYZ[i] = XYZ_0[i] + ct * (XYZ_tar[i] - XYZ_0[i]);
  }
 
}

void get_FootEndPosition (double result[][3],double t,int RunMode)
{
  switch(RunMode)
  {
    case RUN: 
    {
    //printf("%f\n",t);
      int i;
      double tp[4];
      for(i=0;i<4;i++)
      {
        tp[i] = (t + gait_phase_shift[0][i] * T);
        while(tp[i]>=T){tp[i] -= T;}
        if (tp[i] < 0.25 * T && tp[i] >= 0 * T)
           {
             x = stride_length[i] * (-4 * tp[i] / T);
             y = (-V2) * sqrt(1 - pow(x , 2) / pow(stride_length[i] , 2) ) - High;
           }
        if (tp[i] <= 0.75 * T && tp[i] >= 0.25 * T)
           {
             x = stride_length[i] * (4 * tp[i] / T - 2);
             y = V1 * sqrt(1 - pow(x , 2) / pow(stride_length[i] , 2) ) - High;
           }
        if (tp[i] < 1 * T && tp[i] > 0.75 * T)
           {
             x = stride_length[i] * (4 - 4 * tp[i] / T);
             y = (-V2) * sqrt(1 - pow(x , 2) / pow(stride_length[i] , 2) ) - High;
           }
        FootendPosition_self[i][0] = x;
        FootendPosition_self[i][1] = y;
        FootendPosition_self[i][2] = z;
      }
     
        
      break;
    }
    case BACK: 
    {
      int i;
      double tp[4];
      for(i=0;i<4;i++)
      {
        tp[i] = (t + gait_phase_shift[0][i] * T);
        while(tp[i]>=T){tp[i] -= T;}
        if (tp[i] < 0.25 * T && tp[i] >= 0 * T)
           {
             x = stride_length[i] * (-4 * tp[i] / T);
             y = (-V2) * sqrt(1 - pow(x , 2) / pow(stride_length[i] , 2) ) - High;
           }
        if (tp[i] <= 0.75 * T && tp[i] >= 0.25 * T)
           {
             x = stride_length[i] * (4 * tp[i] / T - 2);
             y = V1 * sqrt(1 - pow(x , 2) / pow(stride_length[i] , 2) ) - High;
           }
        if (tp[i] < 1 * T && tp[i] > 0.75 * T)
           {
             x = stride_length[i] * (4 - 4 * tp[i] / T);
             y = (-V2) * sqrt(1 - pow(x , 2) / pow(stride_length[i] , 2) ) - High;
           }
        result[i][0] = -x;
        result[i][1] = y;
        result[i][2] = z;
      }
      break;
    }
    case SETUP:
    {
      int i;
      x = 0;
      y = -470 + t / 4 / T * (470 - High);
      z = 0;
      for(i=0;i<4;i++)
      {
        result[i][0] = x;
        result[i][1] = y;
        result[i][2] = z;
      }
      break;
    }
    case STAND:
    {
      int i;
      x = 0;
      y = - High;
      z = 0;
      for(i=0;i<4;i++)
      {
        result[i][0] = x;
        result[i][1] = y;
        result[i][2] = z;
      }
      break;
    }
   
    case ROTATELEFT: 
    {
      int i;
      double tp[4];
      for(i=0;i<4;i++)
      {
        stride_length[i] = BUCHANG * 0.5;
        tp[i] = (t + gait_phase_shift[0][i] * T);
        while(tp[i]>=T){tp[i] -= T;}
        if (tp[i] < 0.25 * T && tp[i] >= 0 * T)
           {
             x = stride_length[i] * (-4 * tp[i] / T);
             y = (-V2) * sqrt(1 - pow(x , 2) / pow(stride_length[i] , 2) ) - High;
           }
        if (tp[i] <= 0.75 * T && tp[i] >= 0.25 * T)
           {
             x = stride_length[i] * (4 * tp[i] / T - 2);
             y = V1 * sqrt(1 - pow(x , 2) / pow(stride_length[i] , 2) ) - High;
           }
        if (tp[i] < 1 * T && tp[i] > 0.75 * T)
           {
             x = stride_length[i] * (4 - 4 * tp[i] / T);
             y = (-V2) * sqrt(1 - pow(x , 2) / pow(stride_length[i] , 2) ) - High;
           }
        switch(i)
        {
           case 0 :
             result[i][0] = -x * 0.5;
             result[i][1] = y;
             result[i][2] = -x * 0.5;
             break;
           case 1 :
             result[i][0] = x * 0.5;
             result[i][1] = y;
             result[i][2] = -x * 0.5;
             break;
           case 2 :
             result[i][0] = x * 0.5;
             result[i][1] = y;
             result[i][2] = x * 0.5;
             break;
           case 3 :
             result[i][0] = -x * 0.5;
             result[i][1] = y;
             result[i][2] = x * 0.5;
             break;
        }
      }
      break;
    }
    case ROTATERIGHT: 
    {
      int i;
      double tp[4];
      for(i=0;i<4;i++)
      {
        stride_length[i] = BUCHANG * 0.5;
        tp[i] = (t + gait_phase_shift[0][i] * T);
        while(tp[i]>=T){tp[i] -= T;}
        if (tp[i] < 0.25 * T && tp[i] >= 0 * T)
           {
             x = stride_length[i] * (-4 * tp[i] / T);
             y = (-V2) * sqrt(1 - pow(x , 2) / pow(stride_length[i] , 2) ) - High;
           }
        if (tp[i] <= 0.75 * T && tp[i] >= 0.25 * T)
           {
             x = stride_length[i] * (4 * tp[i] / T - 2);
             y = V1 * sqrt(1 - pow(x , 2) / pow(stride_length[i] , 2) ) - High;
           }
        if (tp[i] < 1 * T && tp[i] > 0.75 * T)
           {
             x = stride_length[i] * (4 - 4 * tp[i] / T);
             y = (-V2) * sqrt(1 - pow(x , 2) / pow(stride_length[i] , 2) ) - High;
           }
        switch(i)
        {
           case 0 :
             result[i][0] = x * 0.5;
             result[i][1] = y;
             result[i][2] = x * 0.5;
             break;
           case 1 :
             result[i][0] = -x * 0.5;
             result[i][1] = y;
             result[i][2] = x * 0.5;
             break;
           case 2 :
             result[i][0] = -x * 0.5;
             result[i][1] = y;
             result[i][2] = -x * 0.5;
             break;
           case 3 :
             result[i][0] = x * 0.5;
             result[i][1] = y;
             result[i][2] = -x * 0.5;
             break;
        }
      }
      break;
    }
    case MOVELEFT: 
    {
      int i;
      double tp[4];
      for(i=0;i<4;i++)
      {
        tp[i] = (t + gait_phase_shift[0][i] * T);
        while(tp[i]>=T){tp[i] -= T;}
        if (tp[i] < 0.25 * T && tp[i] >= 0 * T)
           {
             x = stride_length[i] * (-4 * tp[i] / T);
             y = (-V2) * sqrt(1 - pow(x , 2) / pow(stride_length[i] , 2) ) - High;
           }
        if (tp[i] <= 0.75 * T && tp[i] >= 0.25 * T)
           {
             x = stride_length[i] * (4 * tp[i] / T - 2);
             y = 0.2 * V1 * sqrt(1 - pow(x , 2) / pow(stride_length[i] , 2) ) - High;
           }
        if (tp[i] < 1 * T && tp[i] > 0.75 * T)
           {
             x = stride_length[i] * (4 - 4 * tp[i] / T);
             y = (-V2) * sqrt(1 - pow(x , 2) / pow(stride_length[i] , 2) ) - High;
           }
        result[i][0] = z;
        result[i][1] = y;
        result[i][2] = -x * 0.5;
      }
      break;
    }
    case MOVERIGHT: 
    {
      int i;
      double tp[4];
      for(i=0;i<4;i++)
      {
        tp[i] = (t + gait_phase_shift[0][i] * T);
        while(tp[i]>=T){tp[i] -= T;}
        if (tp[i] < 0.25 * T && tp[i] >= 0 * T)
           {
             x = stride_length[i] * (-4 * tp[i] / T);
             y = (-V2) * sqrt(1 - pow(x , 2) / pow(stride_length[i] , 2) ) - High;
           }
        if (tp[i] <= 0.75 * T && tp[i] >= 0.25 * T)
           {
             x = stride_length[i] * (4 * tp[i] / T - 2);
             y = 0.2 * V1 * sqrt(1 - pow(x , 2) / pow(stride_length[i] , 2) ) - High;
           }
        if (tp[i] < 1 * T && tp[i] > 0.75 * T)
           {
             x = stride_length[i] * (4 - 4 * tp[i] / T);
             y = (-V2) * sqrt(1 - pow(x , 2) / pow(stride_length[i] , 2) ) - High;
           }
        result[i][0] = z;
        result[i][1] = y;
        result[i][2] = x * 0.5;
      }
      break;
    }
    
    case ZTCONTROL: 
    {
      int i;
      for(i=0;i<4;i++)
      {
        FootendPosition[i][0] = 0.0;
        FootendPosition[i][1] = -350.0;
        FootendPosition[i][2] = 0.0;
      }
      //double body_RPY_target[3]= {0.00,0.00,0.00};
      //double body_XYZ_target[3]= {0.00,0.00,0.00};
      //body_RPY_target[0] = 0.25 * sin(3.14159 * t);
      //body_RPY_target[1] = 0.16 * cos(3.14159 * t);
      
      //body_RPY_target[2] = 0.10 * sin(3.14159 * t);
      
      //body_XYZ_target[0] = 2.30 * sin(3.14159 * t/2);
      //body_XYZ_target[2] = 0.05 * cos(3.14159 * t);
      //Robot_Interpolation(body_RPY_target,body_XYZ_target,body_RPY,body_XYZ);             
      
      body_RPY[0] = 0;
      body_RPY[1] = 0.16;
      
      //body_RPY[0] = 0.25 * sin(3.14159 * t);
      //body_RPY[1] = 0.16 * sin(3.14159 * t);
      
      zitai_nijie(FootendPosition,FootendPosition_self,body_RPY,body_XYZ);
     
    }
    break;
    case TEST:		//case TEST是保持机身姿态水平的，原本是把机器人放在可转动的平台上，我在webtos2019没找到这个模块 
    {
      int i;
      double dd;
      for(i=0;i<2;i++)
      {
        dd = -(IMU[i] * 0.2);
        dd = dd > 0.02 ? 0.02 : dd;
        dd = dd < -0.02 ? -0.02 : dd;
        body_RPY[i] += dd;
        body_RPY[i] = body_RPY[i] > 0.25 ? 025 : body_RPY[i];
        body_RPY[i] = body_RPY[i] < -0.25 ? -0.25 : body_RPY[i];
      }
      //body_RPY[0] += -(IMU[0] * 0.2);
      //body_RPY[1] += -(IMU[1] * 0.2);
      //body_RPY[2] += -(IMU[2] * 0.2);
      //body_RPY_target[2] = 0.10 * sin(3.14159 * t);
      
      //body_XYZ_target[0] = 2.30 * sin(3.14159 * t/2);
      //body_XYZ_target[2] = 0.05 * cos(3.14159 * t);
      //Robot_Interpolation(body_RPY_target,body_XYZ_target,body_RPY,body_XYZ);            
      
      
      zitai_nijie(FootendPosition,FootendPosition_self,
        body_RPY,body_XYZ);

     
    }
    break;
    case STOP:
    {
      double body_RPY_target[3]= {0.00,0.00,0.00};

      Robot_Interpolation(body_RPY_target,body_XYZ,body_RPY,body_XYZ);             
      
      zitai_nijie(FootendPosition,FootendPosition_self,body_RPY,body_XYZ);
    }
    break;
    case WALK:
    {
      switch(MoveMode)
      {
        case ReadytoMoveBody:
        {
          //printf("(%4.3f)ReadytoMoveBody(%d)\n",t_walk,disleg);
          disleg = Discontactleg[(leg)%4];
          Get_CG(CG,FootendPosition,disleg);
          MoveMode = MoveBody;
        }
        break;
        
        case MoveBody:
        {
          //printf("(%4.3f)MoveBody(%d)\n",t_walk,disleg);
          Robot_Interpolation(body_RPY,CG,body_RPY,body_XYZ);
          
          if((fabs(CG[0]-body_XYZ[0]) <= 0.002)&&
             (fabs(CG[1]-body_XYZ[1]) <= 0.002)&&
             (fabs(CG[2]-body_XYZ[2]) <= 0.002))
            {MoveMode = ReadytoMoveLeg;}
           
        }
        break;
        
        case ReadytoMoveLeg:
        {
          //printf("(%4.3f)ReadytoMoveLeg(%d)\n",t_walk,disleg);
          disleg = Discontactleg[(leg)%4];
          xa = FootendPosition[disleg][0];
          ya = FootendPosition[disleg][1];
          za = FootendPosition[disleg][2];
          ta = t_walk;
          MoveMode = MoveLeg1;
        }
        break;
        
        case MoveLeg1:
        {
          
          FootendPosition[disleg][1] += 100.0 / 32.0; 
          //printf("(%4.3f)MoveLeg1(%d),(%4.2f),(%4.2f),(%4.2f)\n",t_walk,disleg,FootendPosition[disleg][0],FootendPosition[disleg][1],FootendPosition[disleg][2]);
          if(FootendPosition[disleg][1] >= -250)
          {
            FootendPosition[disleg][1] = -250;
            MoveMode = MoveLeg2;
          }
          
        }
        break;
        
        case MoveLeg2:
        {
          
          FootendPosition[disleg][0] += BUCHANG / 64.0; 
          //printf("(%4.3f)MoveLeg2(%d),(%4.2f),(%4.2f),(%4.2f)\n",t_walk,disleg,FootendPosition[disleg][0],FootendPosition[disleg][1],FootendPosition[disleg][2]);
          if(FootendPosition[disleg][0] - xa > BUCHANG)
          {
            FootendPosition[disleg][0] = xa + BUCHANG;
            MoveMode = MoveLeg3;
          }

        }
        break;
        
        case MoveLeg3:
        {
         // printf("(%4.3f)MoveLeg3(%d),(%4.2f),(%4.2f),(%4.2f)\n",t_walk,disleg,FootendPosition[disleg][0],FootendPosition[disleg][1],FootendPosition[disleg][2]);
          FootendPosition[disleg][1] -= 100.0 / 32.0; 
          
          if(FootendPosition[disleg][1] - ya<= 0)
          {
            FootendPosition[disleg][1] = ya;
            MoveMode = ReadytoMoveBody;
            leg++;
          }
          //if(isTouch[disleg] == 1)
          //{
             
             //MoveMode = MoveLeg4;
          //}

        }
        break;
        
        case MoveLeg4:
        {
         // printf("(%4.3f)MoveLeg4(%d),(%4.2f),(%4.2f),(%4.2f)\n",t_walk,disleg,FootendPosition[disleg][0],FootendPosition[disleg][1],FootendPosition[disleg][2]);
          FootendPosition[disleg][1] += 10.0 / 32.0; 
          
          if(isTouch[disleg] == 0)
          {
             MoveMode = ReadytoMoveBody;
             leg++;
          }

        }
        break;
        
        default: break;
       
      }
      zitai_nijie(FootendPosition,FootendPosition_self,
        body_RPY,body_XYZ);

     
    
    }
    break;
    default: break;
  }
}


/*-------------这部分是已知足端位置（x,y,z）求关节角-------------*/
/*-------------两个函数分别为前后腿求关节角逆解-------------*/ 
void get_foreleg_ServoPosition (double result[],double x,double y,double z)
{
  result[0] = asin(z / sqrt(y*y+z*z));
  double xp = x;
  double yp = -sqrt(y*y+z*z) + Kuan;
  if(xp<0)
  {
    result[1] = atan(yp/xp) - pi - acos((L1*L1 
      + xp*xp + yp*yp - L2*L2)/(2*L1*sqrt(xp*xp + yp*yp))) + pi/2;
  }
  else if(xp>0)
  {
    result[1] = atan(yp/xp) - acos((L1*L1 
      + xp*xp + yp*yp - L2*L2)/(2*L1*sqrt(xp*xp + yp*yp))) + pi/2;
  }
  else
  {
    //xp==0
    result[1] = -acos((L1*L1 + xp*xp + yp*yp - L2*L2)
    /(2*L1*sqrt(xp*xp + yp*yp)));
  }
  result[2] = pi - acos((L1*L1 + L2*L2 - xp*xp - yp*yp) / (2 * L1 * L2));
}



void get_hindleg_ServoPosition (double result[],double x,double y,double z)
{
  result[0] = asin(z/sqrt(y*y+z*z));
  double xp = x;
  double yp = -sqrt(y*y+z*z) + Kuan;
  if(xp<0)
  {
    result[1] = atan(yp/xp) - pi + acos((L1*L1 
      + xp*xp + yp*yp - L2*L2)/(2*L1*sqrt(xp*xp + yp*yp))) + pi/2;  
  }
  else if(xp>0)
  {
    result[1] = atan(yp/xp) + acos((L1*L1 
      + xp*xp + yp*yp - L2*L2)/(2*L1*sqrt(xp*xp + yp*yp))) + pi/2; 
  }
  else
  {
     //xp==0
     result[1] = acos((L1*L1 + xp*xp + yp*yp - L2*L2)
     /(2*L1*sqrt(xp*xp + yp*yp))); 
  }
  result[2] = acos((L1*L1 + L2*L2 - xp*xp - yp*yp) / (2 * L1 * L2)) - pi;
}
/*-------------这部分是已知足端位置（x,y,z）求关节角--------------*/ 
#endif
