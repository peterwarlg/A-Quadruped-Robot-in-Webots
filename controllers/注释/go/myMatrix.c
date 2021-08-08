//全是矩阵运算，即矩阵的加减乘除，最后四个是机器人姿态变化矩阵 

#ifndef myMatrix_c
#define myMatrix_c

void Matrix_quni3(double result[][3],double a[][3])
{
	int i,j,m1,m2,n1,n2;
	double det3(double a[3][3]);
	double det2(double a1, double a2, double a3, double a4);
	double abs = det3(a);
	double rep[3][3];
	for(i = 0; i < 3; i++)
	{	for(j = 0; j < 3; j++)
		{
			rep[i][j] = a[i][j];
		}}
		for(i = 0; i < 3; i++)
		{	for(j = 0; j < 3; j++)
			{
				m1 = (i + 1) % 3;m2 = (i + 2) % 3;
				n1 = (j + 1) % 3;n2 = (j + 2) % 3;
				result[j][i] = det2(rep[m1][n1],rep[m1][n2],rep[m2][n1],rep[m2][n2]) / abs;
			}}

}

void Matrix_quni4(double result[][4],double a[][4])
{
	int i,j,m,n;
	double det3(double a[][3]);
	double det4(double a[][4]);
	double abs = det4(a);
	double rep[4][4];
	double b[3][3];

	for(i = 0; i < 4; i++)
	{
		for(j = 0; j < 4; j++)
		{
			rep[i][j] = a[i][j];
		}
	}
	for(i = 0; i < 4; i++)
	{	for(j = 0; j < 4; j++)
		{
			for(m=0;m<3;m++)
				{for(n=0;n<3;n++)
				{
					b[m][n] = rep[(m+i+1)%4][(n+j+1)%4];
				}}
				result[j][i] = det3(b) / abs * pow(-1.0,i+j);
		}
	}
}

double det2(double a1, double a2, double a3, double a4)
{
	return a1 * a4 - a2 * a3;
}

double det3(double a[][3])
{
	double sum;
	sum = a[0][0] * a[1][1] * a[2][2] + a[0][1] * a[1][2] * a[2][0]
	+ a[0][2] * a[1][0] * a[2][1] - a[0][2] * a[1][1] * a[2][0]
	- a[0][1] * a[1][0] * a[2][2] - a[0][0] * a[1][2] * a[2][1];
	return sum;
}

double det4(double x[][4])
{
	double sum = 0;
	double b[3][3];
	int i,j,k;
	for(k=0;k<4;k++)
	{
		for(i=0;i<3;i++)
		{
			for(j=0;j<3;j++)
			{
				b[i][j] = x[i+1][(j+k+1)%4];
			}
		}
		sum += x[0][k] * det3(b) * pow(-1.0,k);
	}
	return sum;
}

void Matrix_3to4(double result[][4],double a[][3])
{
	int i,j;
	for(i=0;i<4;i++)
		for(j=0;j<4;j++)
		{
			if(i<3&&j<3)
			{result[i][j] = a[i][j];}
			else
				if(i==3&&j==3)
				{result[i][j] = 1;}
				else
				{result[i][j] = 0;}
		}
}

void matrix_mul_444(double result[][4],double a[][4],double b[][4])
{
	int ii,jj,kk;
	int i_max = 4;
	int j_max = 4;
	int k_max = 4;
	double rep_a[4][4],rep_b[4][4];
	for(ii = 0; ii < 4; ii++)
	{
		for(jj = 0; jj < 4; jj++)
		{
			rep_a[ii][jj] = a[ii][jj];
			rep_b[ii][jj] = b[ii][jj];
		}
	}
	double sum;
	for(ii = 0; ii < i_max;  ii++)
	{
		for(jj = 0; jj < j_max; jj++)
		{
			sum = 0;
			for(kk = 0; kk < k_max; kk++)
			{
				sum = sum + rep_a[ii][kk] * rep_b[kk][jj];
			}
			result[ii][jj] = sum;
		}
	}
}

void matrix_mul_414(double result[][1],double a[][4],double b[][1])
{
	int ii,jj,kk;
	int i_max = 4;
	int j_max = 1;
	int k_max = 4;
	double rep_a[4][4],rep_b[4][1];
	for(ii = 0; ii < 4; ii++)
	{
		for(jj = 0; jj < 4; jj++)
		{
			rep_a[ii][jj] = a[ii][jj];
		}
		for(jj = 0; jj < 1; jj++)
		{
			rep_b[ii][jj] = b[ii][jj];
		}
	}
	double sum;
	for(ii = 0; ii < i_max;  ii++)
	{
		for(jj = 0; jj < j_max; jj++)
		{
			sum = 0;
			for(kk = 0; kk < k_max; kk++)
			{
				sum = sum + rep_a[ii][kk] * rep_b[kk][jj];
			}
			result[ii][jj] = sum;
		}
	}
}

void matrix_mul_333(double result[][3],double a[][3],double b[][3])
{
	int ii,jj,kk;
	int i_max = 3;
	int j_max = 3;
	int k_max = 3;
	double rep_a[3][3],rep_b[3][3];
	for(ii = 0; ii < 3; ii++)
	{
		for(jj = 0; jj < 3; jj++)
		{
			rep_a[ii][jj] = a[ii][jj];
			rep_b[ii][jj] = b[ii][jj];
		}
	}
	double sum;
	for(ii = 0; ii < i_max;  ii++)
	{
		for(jj = 0; jj < j_max; jj++)
		{
			sum = 0;
			for(kk = 0; kk < k_max; kk++)
			{
				sum = sum + rep_a[ii][kk] * rep_b[kk][jj];
			}
			result[ii][jj] = sum;
		}
	}
}

void matrix_mul_313(double result[][1],double a[][3],double b[][1])
{
	int ii,jj,kk;
	int i_max = 3;
	int j_max = 1;
	int k_max = 3;
	double rep_a[3][3],rep_b[3][1];
	for(ii = 0; ii < 3; ii++)
	{
		for(jj = 0; jj < 3; jj++)
		{
			rep_a[ii][jj] = a[ii][jj];
		}
		for(jj = 0; jj < 1; jj++)
		{
			rep_b[ii][jj] = b[ii][jj];
		}
	}
	double sum;
	for(ii = 0; ii < i_max;  ii++)
	{
		for(jj = 0; jj < j_max; jj++)
		{
			sum = 0;
			for(kk = 0; kk < k_max; kk++)
			{
				sum = sum + rep_a[ii][kk] * rep_b[kk][jj];
			}
			result[ii][jj] = sum;
		}
	}
}

void matrix_printf(double *result,int a,int b)
{
	int i,j;
	for(i=0;i<a;i++)
	{
		for(j=0;j<b;j++)
		{
			printf("%f,",result[i*b+j]);
		}
		printf("\n");
	}
}

void matrix_trans(double result[][4],double x,double y,double z)
{
	int i,j;
	for(i=0;i<4;i++)
	{
		for(j=0;j<4;j++)
		{
			if(i==j)
			{
				result[i][j] = 1;
			}
			else
			{
			    result[i][j] = 0;
			}
		}
	}
	result[0][3] = x;
	result[1][3] = y;
	result[2][3] = z;
}

void matrix_rotx(double result[][4],double x)
{
	int i,j;
	double a = x;
	for(i=0;i<4;i++)
	{
		for(j=0;j<4;j++)
		{
			if(i==j)
			{
				result[i][j] = 1;
			}
			else
			{
			    result[i][j] = 0;
			}
		}
	}
	result[1][1] = cos(a);
	result[1][2] = -sin(a);
	result[2][1] = sin(a);
	result[2][2] = cos(a);
}

void matrix_roty(double result[][4],double y)
{
	int i,j;
	double a = y;
	for(i=0;i<4;i++)
	{
		for(j=0;j<4;j++)
		{
			if(i==j)
			{
				result[i][j] = 1;
			}
			else
			{
			    result[i][j] = 0;
			}
		}
	}
	result[0][0] = cos(a);
	result[0][2] = sin(a);
	result[2][0] = -sin(a);
	result[2][2] = cos(a);
}

void matrix_rotz(double result[][4],double z)
{
	int i,j;
	double a = z;
	for(i=0;i<4;i++)
	{
		for(j=0;j<4;j++)
		{
			if(i==j)
			{
				result[i][j] = 1;
			}
			else
			{
			    result[i][j] = 0;
			}
		}
	}
	result[0][0] = cos(a);
	result[0][1] = -sin(a);
	result[1][0] = sin(a);
	result[1][1] = cos(a);
}

#endif
