#ifndef zitai_control_c
#define zitai_control_c

const double leg_position[4][3] = { 
    {  0.25, 0, -0.16},
    {  0.25, 0,  0.16},
    { -0.25, 0,  0.16},
    { -0.25, 0, -0.16}
    };

double footend_position[4][3] = { 
    {  0, -0.35,  0},
    {  0, -0.35,  0},
    {  0, -0.35,  0},
    {  0, -0.35,  0}
    };
/*
void ZT_control(double FEP[4][3],double Roll,double Pitch,double Yaw,double High)
{
  int i;
  double Ti[4][4],RPY[4][4];
  double rxR[4][4],rzP[4][4],ryY[4][4];
  double tr1[4][4],tr2[4][4];
  matrix_rotx(rxR,Roll);
  matrix_rotz(rzP,Pitch);
  matrix_roty(ryY,Yaw);
  matrix_mul_444(RPY,rxR,rzP);
  matrix_mul_444(RPY,RPY,ryY);
  Matrix_quni4(RPY,RPY);
  for(i=0;i<4;i++)
  {
     matrix_trans(tr1,-leg_position[i][0],0,-leg_position[i][2]);
     matrix_trans(tr2,leg_position[i][0],-High,leg_position[i][2]);
     matrix_mul_444(Ti,tr1,RPY);
     matrix_mul_444(Ti,Ti,tr2);
     FEP[i][0] = Ti[0][3] * 1000;
     FEP[i][1] = Ti[1][3] * 1000;
     FEP[i][2] = Ti[2][3] * 1000;
  }
}
*/

//FEP[4][3]�����λ�ã����Կ����˺����������roll,p,yΪĿ����̬�ǣ�
//x,y,z������������λ�ýϳ�ʼ�ı仯��high�������ĸ߶� ,�����������û���� 
void ZT_control(double FEP[4][3],double Roll,double Pitch,
double Yaw,double High,double x,double y,double z)          
{
  int i;
  double Ti[4][4],RPY[4][4];
  double rxR[4][4],rzP[4][4],ryY[4][4];
  double op[4][1];
  double tr[4][4];
  matrix_rotx(rxR,-Roll);
  matrix_rotz(rzP,-Pitch);
  matrix_roty(ryY,-Yaw);
  matrix_trans(tr,x,y,z);
  matrix_mul_444(RPY,rxR,rzP);
  matrix_mul_444(RPY,RPY,ryY);
  matrix_mul_444(RPY,tr,RPY);
  Matrix_quni4(Ti,RPY);
  for(i=0;i<4;i++)
  {
     
     op[0][0] = leg_position[i][0];
     op[1][0] = -High;
     op[2][0] = leg_position[i][2];
     op[3][0] = 1;
     
     matrix_mul_414(op,Ti,op);
     FEP[i][0] = 1000 * (op[0][0] - leg_position[i][0]);
     FEP[i][1] = 1000 * (op[1][0] - leg_position[i][1]);
     FEP[i][2] = 1000 * (op[2][0] - leg_position[i][2]);
  }
}


//�ĸ�����Ϊ����������λ��;�������ţ���ڣ��ؽ�λ��;r,p,y�Ƕ�;��������λ��
//����Ϊ��֪����rpy�任����̬�仯���������������꣨�������ģ�λ�ã���֪�Źؽ���Ի�������λ�ã�����������
//����һ�������ļӼ��õ��������Źؽ�λ�ã�������robot.c�еĺ������ؽڽ� 
void zitai_nijie(double FEP[4][3],double FEP_self[4][3],
double b_RPY[3],double b_XYZ[3])
{
  double Roll = b_RPY[0];
  double Pitch = b_RPY[1];
  double Yaw = b_RPY[2];
  int i;
  double Ti[4][4],RPY[4][4];
  double rxR[4][4],rzP[4][4],ryY[4][4];
  double op[4][1];
  matrix_rotx(rxR,-Roll);
  matrix_rotz(rzP,-Pitch);
  matrix_roty(ryY,-Yaw);
  matrix_mul_444(RPY,rxR,rzP);
  matrix_mul_444(RPY,RPY,ryY);
  Matrix_quni4(Ti,RPY);
  for(i=0;i<4;i++)
  {
     op[0][0] = leg_position[i][0];
     op[1][0] = leg_position[i][1];
     op[2][0] = leg_position[i][2];
     op[3][0] = 1;
     
     matrix_mul_414(op,RPY,op);
     
     op[0][0] = FEP[i][0] /1000 + leg_position[i][0] - (op[0][0] + b_XYZ[0]);
     op[1][0] = FEP[i][1] /1000 + leg_position[i][1] - (op[1][0] + b_XYZ[1]);
     op[2][0] = FEP[i][2] /1000 + leg_position[i][2] - (op[2][0] + b_XYZ[2]);
     
     matrix_mul_414(op,Ti,op);
     
     FEP_self[i][0] = 1000 * op[0][0];
     FEP_self[i][1] = 1000 * op[1][0];
     FEP_self[i][2] = 1000 * op[2][0];
  }

}

void ZT_getselfFEP(double FEP[4][3],double FEP_self[4][3],
double b_RPY[3],double b_XYZ[3])
{
  double Roll = b_RPY[0];
  double Pitch = b_RPY[1];
  double Yaw = b_RPY[2];
  int i;
  double Ti[4][4],RPY[4][4];
  double rxR[4][4],rzP[4][4],ryY[4][4];
  double op[4][1];
  double tr[4][4];
  matrix_rotx(rxR,-Roll);
  matrix_rotz(rzP,-Pitch);
  matrix_roty(ryY,-Yaw);
  matrix_trans(tr,b_XYZ[0],b_XYZ[1],b_XYZ[2]);
  matrix_mul_444(RPY,rxR,rzP);
  matrix_mul_444(RPY,RPY,ryY);
  matrix_mul_444(RPY,tr,RPY);
  Matrix_quni4(Ti,RPY);
  for(i=0;i<4;i++)
  {
     op[0][0] = FEP[i][0] /1000 + leg_position[i][0];
     op[1][0] = FEP[i][1] /1000;
     op[2][0] = FEP[i][2] /1000 + leg_position[i][2];
     op[3][0] = 1;
     
     matrix_mul_414(op,Ti,op);
     
     FEP_self[i][0] = 1000 * (op[0][0] - leg_position[i][0]);
     FEP_self[i][1] = 1000 * (op[1][0] - leg_position[i][1]);
     FEP_self[i][2] = 1000 * (op[2][0] - leg_position[i][2]);
  }
}

void ZT_change(double FEP[4][3],double Roll,double Pitch,
double Yaw,double x,double y,double z)
{
  int i;
  double Ti[4][4],RPY[4][4];
  double rxR[4][4],rzP[4][4],ryY[4][4];
  double op[4][1];
  double tr[4][4];
  matrix_rotx(rxR,-Roll);
  matrix_rotz(rzP,-Pitch);
  matrix_roty(ryY,-Yaw);
  matrix_trans(tr,x,y,z);
  matrix_mul_444(RPY,rxR,rzP);
  matrix_mul_444(RPY,RPY,ryY);
  matrix_mul_444(RPY,tr,RPY);
  Matrix_quni4(Ti,RPY);
  for(i=0;i<4;i++)
  {
     
     op[0][0] = leg_position[i][0] + FEP[i][0] / 1000;
     op[1][0] = leg_position[i][1] + FEP[i][1] / 1000;
     op[2][0] = leg_position[i][2] + FEP[i][2] / 1000;
     op[3][0] = 1;
     
     matrix_mul_414(op,Ti,op);
     FEP[i][0] = 1000 * (op[0][0] - leg_position[i][0]);
     FEP[i][1] = 1000 * (op[1][0] - leg_position[i][1]);
     FEP[i][2] = 1000 * (op[2][0] - leg_position[i][2]);
  }
}

void Get_CG(double CG[3],double FEP[4][3],int discontactleg)
{
  CG[0] = (FEP[0][0]/1000 + FEP[1][0]/1000 + FEP[2][0]/1000 + FEP[3][0]/1000 - (leg_position[discontactleg][0] + FEP[discontactleg][0]/1000))/3.0;
  CG[1] = 0;
  CG[2] = (FEP[0][2]/1000 + FEP[1][2]/1000 + FEP[2][2]/1000 + FEP[3][2]/1000 - (leg_position[discontactleg][2] + FEP[discontactleg][2]/1000))/3.0;
 
}

#endif
