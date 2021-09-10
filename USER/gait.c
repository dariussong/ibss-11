#include <math.h>
#include "pwm.h"	
#include "capture.h"
#include "adc.h"
#include "SV.h"
#include "gait.h"
#include <sys.h>
#include "uart.h"
#include "timer.h"
const short L1=38;
const short L2=47;//47
const short L3=32;//22
#define DE 1
KinematicsArm KMGecko;
int i;
float F1=38;
float F2=47;
float F3=32;


float x[4]={0};
float y[4]={0};
float z[4]={0};
float x_b=63;//62
float y_b=30.24;
float D;
float B;	

float D_onestep;
float L_onestep=15;
float W_onestep=0;
float delta=0;
float H_onestep =15;
float stime=0;
float Time=8;
float cx[4]={0};
float cy[4]={0};
float cz[4]={0};
float delta;
void movement_trot(void)
{
	float a[4][3] = {0};     //   a1 = a[0]   a2=a[LF]
	float theta[4][2] = {0};
	delta=delta*3.1415/180;
  float alpha;
	float belta;
	D = sqrt((x_b+F2)*(x_b+F2)+(y_b+F1)*(y_b+F1));
	B = sqrt(x_b*x_b+y_b*y_b);
	alpha = acos((x_b+F2)/D);
	belta = acos(x_b/B);

	if(s<3*T/2/4&&s>=0)
	{	
		
		x[0]=-B*cos(belta+delta/2*s/T*2)-L_onestep/2*2*s/T+D*cos(alpha);
		y[0]=-B*sin(belta+delta/2*s/T*2)-W_onestep/2*2*s/T+D*sin(alpha);
		z[0]=-L3;	
		
		x[1]=D*cos(alpha-delta*s/T*2*4/3)-B*cos(belta-delta/2*s/T*2)+L_onestep*s/T*2/2*8/3-L_onestep*2*s/T/2;
		y[1]=-D*sin(alpha-delta*s/T*2*4/3)+B*sin(belta-delta/2*s/T*2)+W_onestep*2*s/T/2*8/3-W_onestep*2*s/T/2;
		z[1]=-L3+H_onestep-H_onestep*(sqrt(L_onestep*L_onestep+W_onestep*W_onestep)*2*s/T/2*4/3- sqrt(L_onestep*L_onestep+W_onestep*W_onestep)/4)*
			(sqrt(L_onestep*L_onestep+W_onestep*W_onestep)*2*s/T/2*4/3- sqrt(L_onestep*L_onestep+W_onestep*W_onestep)/4)/((L_onestep*L_onestep+W_onestep*W_onestep)/16);
			
		x[2]= -D*cos(alpha-delta*s/T*2*4/3)+B*cos(belta-delta/2*s/T*2)+L_onestep*s/T*2/2*8/3-L_onestep*2*s/T/2;
		y[2]= D*sin(alpha-delta*s/T*2*4/3)-B*sin(belta-delta/2*s/T*2)+W_onestep*2*s/T/2*8/3-W_onestep*2*s/T/2;
		z[2]= -L3+H_onestep-H_onestep*(sqrt(L_onestep*L_onestep+W_onestep*W_onestep)*2*s/T/2*4/3- sqrt(L_onestep*L_onestep+W_onestep*W_onestep)/4)*
			(sqrt(L_onestep*L_onestep+W_onestep*W_onestep)*2*s/T/2*4/3- sqrt(L_onestep*L_onestep+W_onestep*W_onestep)/4)/((L_onestep*L_onestep+W_onestep*W_onestep)/16);		
		
		x[3]=B*cos(belta+delta/2*s/T*2)-L_onestep/2*2*s/T-D*cos(alpha);
		y[3]=B*sin(belta+delta/2*s/T*2)-W_onestep/2*2*s/T-D*sin(alpha);
		z[3]=-L3;
		}
	if(s>=3*T/2/4&&s<T/2)
	{
		x[0]=-B*cos(belta+delta/2*s/T*2)-L_onestep/2*2*s/T+D*cos(alpha);
		y[0]=-B*sin(belta+delta/2*s/T*2)-W_onestep/2*2*s/T+D*sin(alpha);
		z[0]=-L3;
		
		x[1]=D*cos(alpha-delta)-B*cos(belta-delta/2*s/T*2)+L_onestep-L_onestep*2*s/T/2;
		y[1]=-D*sin(alpha-delta)+B*sin(belta-delta/2*s/T*2)+W_onestep-W_onestep*2*s/T/2;
		z[1]=-L3;
		
		x[2]=-D*cos(alpha-delta)+B*cos(belta-delta/2*s/T*2)+L_onestep-L_onestep*2*s/T/2;
		y[2]=D*sin(alpha-delta)-B*sin(belta-delta/2*s/T*2)+W_onestep-W_onestep*2*s/T/2;
		z[2]=-L3;	
		
		x[3]=B*cos(belta+delta/2*s/T*2)-L_onestep/2*2*s/T-D*cos(alpha);
		y[3]=B*sin(belta+delta/2*s/T*2)-W_onestep/2*2*s/T-D*sin(alpha);
		z[3]=-L3;
	}
		if(s>=T/2&&s<7*T/8)
		{
		x[0]=D*cos(alpha+delta*(s-T/2)/T*2*4/3)-B*cos(belta+delta/2*s/T*2*4/3)+L_onestep*2*(s-T/2)/T*4/3-L_onestep*2*s/T/2;
		y[0]=D*sin(alpha+delta*(s-T/2)/T*2*4/3)-B*sin(belta+delta/2*s/T*2*4/3)+W_onestep*2*(s-T/2)/T*4/3-W_onestep*2*s/T/2;
		z[0]= -L3+H_onestep-H_onestep*(sqrt(L_onestep*L_onestep+W_onestep*W_onestep)*2*(s-T/2)/T/2*4/3- sqrt(L_onestep*L_onestep+W_onestep*W_onestep)/4)*
		(sqrt(L_onestep*L_onestep+W_onestep*W_onestep)*2*(s-T/2)/T/2*4/3- sqrt(L_onestep*L_onestep+W_onestep*W_onestep)/4)/((L_onestep*L_onestep+W_onestep*W_onestep)/16);

		x[1]=D*cos(alpha-delta)-B*cos(belta-delta/2*s/T*2)+L_onestep-L_onestep*2*s/T/2;
		y[1]=-D*sin(alpha-delta)+B*sin(belta-delta/2*s/T*2)+W_onestep-W_onestep*2*s/T/2;
		z[1]=-L3;
		
		x[2]=-D*cos(alpha-delta)+B*cos(belta-delta/2*s/T*2)+L_onestep-L_onestep*2*s/T/2;
		y[2]=D*sin(alpha-delta)-B*sin(belta-delta/2*s/T*2)+W_onestep-W_onestep*2*s/T/2;
		z[2]=-L3;
		
		x[3]=-D*cos(alpha+delta*(s-T/2)/T*2*4/3)+B*cos(belta+delta/2*s/T*2)+L_onestep*2*(s-T/2)/T*4/3-L_onestep*2*s/T/2;
		y[3]=-D*sin(alpha+delta*(s-T/2)/T*2*4/3)+B*sin(belta+delta/2*s/T*2)+W_onestep*2*(s-T/2)/T*4/3-W_onestep*2*s/T/2;
		z[3]= -L3+H_onestep-H_onestep*(sqrt(L_onestep*L_onestep+W_onestep*W_onestep)*2*(s-T/2)/T/2*4/3- sqrt(L_onestep*L_onestep+W_onestep*W_onestep)/4)*
		(sqrt(L_onestep*L_onestep+W_onestep*W_onestep)*2*(s-T/2)/T/2*4/3- sqrt(L_onestep*L_onestep+W_onestep*W_onestep)/4)/((L_onestep*L_onestep+W_onestep*W_onestep)/16);
		}
		if(s>=7*T/8&&s<T)
		{
			x[0]=D*cos(alpha-delta)-B*cos(belta-delta/2*s/T*2)+L_onestep-L_onestep*2*s/T/2;
			y[0]=D*sin(alpha-delta)-B*sin(belta-delta/2*s/T*2)+W_onestep-W_onestep*2*s/T/2;
			z[0]=-L3;
			
			x[1]=D*cos(alpha-delta)-B*cos(belta-delta/2*s/T*2)+L_onestep-L_onestep*2*s/T/2;
			y[1]=-D*sin(alpha-delta)+B*sin(belta-delta/2*s/T*2)+W_onestep-W_onestep*2*s/T/2;
			z[1]=-L3;
			
			x[2]=-D*cos(alpha-delta)+B*cos(belta-delta/2*s/T*2)+L_onestep-L_onestep*2*s/T/2;
			y[2]=D*sin(alpha-delta)-B*sin(belta-delta/2*s/T*2)+W_onestep-W_onestep*2*s/T/2;
			z[2]=-L3;
			
			x[3]=-D*cos(alpha-delta)+B*cos(belta-delta/2*s/T*2)+L_onestep-L_onestep*2*s/T/2;
			y[3]=-D*sin(alpha-delta)+B*sin(belta-delta/2*s/T*2)+W_onestep-W_onestep*2*s/T/2;
			z[3]=-L3;
		}
		
			a[0][0] = -asin(L3 / sqrt(z[0] * z[0] + y[0] * y[0]) ) - atan2(z[0],y[0]);
			a[0][1] = (asin((y[0] * y[0] + x[0] * x[0] + z[0] * z[0] + L1 * L1 - L2 * L2 - L3 * L3) / ( 2 * L1 * sqrt (y[0] * y[0] +  x[0] * x[0] + z[0] * z[0] - L3 * L3)) )
											- atan2(sqrt(y[0] * y[0] + z[0] * z[0] - L3 * L3) , x[0]));
			a[0][2] = asin((L1 * L1 + L2 * L2 + L3 * L3 - y[0] * y[0] - x[0] * x[0] - z[0] * z[0]) / (2 * L1 * L2));
	
			a[1][0] = asin(L3 / sqrt(y[1] * y[1] + z[1] * z[1]) ) + atan2(z[1],-y[1]);
			a[1][1] = -asin((y[1] * y[1] + x[1] * x[1] + z[1] * z[1] + L1 * L1 - L2 * L2 - L3 * L3) / ( 2 * L1 * sqrt (y[1] * y[1] +  x[1] * x[1] + z[1] * z[1] - L3 * L3)) )
											+ atan2(sqrt(y[1] * y[1] + z[1] * z[1] - L3 * L3) , x[1]);
			a[1][2] = -asin((L1 * L1 + L2 * L2 + L3 * L3 - y[1] * y[1] - x[1] * x[1] - z[1] * z[1]) / (2 * L1 * L2));
	
			a[2][0] = -asin(L3 / sqrt(y[2] * y[2] + z[2] * z[2]) ) - atan2(z[2],y[2]);
			a[2][1] = -(asin((y[2] * y[2] + x[2] * x[2] + z[2] * z[2] + L1 * L1 - L2 * L2 - L3 * L3) / ( 2 * L1 * sqrt (y[2] * y[2] +  x[2] * x[2] + z[2] * z[2] - L3 * L3)) 
											 - atan2(sqrt(y[2] * y[2] + z[2] * z[2] - L3 * L3) , -x[2])));
			a[2][2] = -asin((L1 * L1 + L2 * L2 + L3 * L3 - y[2] * y[2] - x[2] * x[2] - z[2] * z[2]) / (2 * L1 * L2));
	
			a[3][0] = asin(L3 / sqrt(y[3] * y[3] + z[3] * z[3]) ) + atan2(z[3],-y[3]);
			a[3][1] = asin((y[3] * y[3] + x[3] * x[3] + z[3] * z[3] + L1 * L1 - L2 * L2 - L3 * L3) / ( 2 * L1 * sqrt (y[3] * y[3] +  x[3] * x[3] + z[3] * z[3] - L3 * L3)) )
											- atan2(sqrt(y[3] * y[3] + z[3] * z[3] - L3 * L3) , -x[3]);
			a[3][2] = asin((L1 * L1 + L2 * L2 + L3 * L3 - y[3] * y[3] - x[3] * x[3] - z[3] * z[3]) / (2 * L1 * L2));
				theta[0][0]=(a[0][0]+a[0][1]);
				theta[0][1]=(a[0][1]-a[0][0]);
				
				theta[0][0]=(int) (theta[0][0]/3.1415926*180*100);//rf alpha 角，最内侧
				theta[0][1]=(int) (theta[0][1]/3.1415926*180*100);
				a[0][2]=(int) (a[0][2]/3.1415926*180*100);

				theta[0][0]= (float) (theta[0][0]/100);
				theta[0][1]= (float) (theta[0][1]/100);
				a[0][2]= (float) (a[0][2]/100);
				
				theta[1][0]=(a[1][0]+a[1][1]);
				theta[1][1]=(a[1][1]-a[1][0]);
				
				theta[1][0]=(int) (theta[1][0]/3.1415926*180*100);//rf alpha 角，最内侧
				theta[1][1]=(int) (theta[1][1]/3.1415926*180*100);
				a[1][2]=(int) (a[1][2]/3.1415926*180*100);
			
				theta[1][0]= (float) (theta[1][0]/100);
				theta[1][1]= (float) (theta[1][1]/100);
				a[1][2]= (float) (a[1][2]/100);

				theta[2][0]=(a[2][1]-a[2][0]);
				theta[2][1]=(a[2][1]+a[2][0]);
				
				theta[2][0]=(int) (theta[2][0]/3.1415926*180*100);//rf alpha 角，最内侧
				theta[2][1]=(int) (theta[2][1]/3.1415926*180*100);
				a[2][2]=(int) (a[2][2]/3.1415926*180*100);

				theta[2][0]= (float) (theta[2][0]/100);
				theta[2][1]= (float) (theta[2][1]/100);
				a[2][2]= (float) (a[2][2]/100);
				
				theta[3][0]=(a[3][1]-a[3][0]);
				theta[3][1]=(a[3][1]+a[3][0]);
				
				theta[3][0]=(int) (theta[3][0]/3.1415926*180*100);//rf alpha 角，最内侧
				theta[3][1]=(int) (theta[3][1]/3.1415926*180*100);
				a[3][2]=(int) (a[3][2]/3.1415926*180*100);

				theta[3][0]= (float) (theta[3][0]/100);
				theta[3][1]= (float) (theta[3][1]/100);
				a[3][2]= (float) (a[3][2]/100);
			
				Angle(theta[0][0]+KMGecko.StartAngle[LF_J3],LF_J3);
				Angle(theta[0][1]+KMGecko.StartAngle[LF_J2],LF_J2);
				Angle(a[0][2]+KMGecko.StartAngle[LF_J1],LF_J1);
			
				Angle(theta[1][0]+KMGecko.StartAngle[RF_J3],RF_J3);
				Angle(theta[1][1]+KMGecko.StartAngle[RF_J2],RF_J2);
				Angle(a[1][2]+KMGecko.StartAngle[RF_J1],RF_J1);
					
				Angle(theta[2][0]+KMGecko.StartAngle[LH_J3],LH_J3);
				Angle(theta[2][1]+KMGecko.StartAngle[LH_J2],LH_J2);
				Angle(a[2][2]+KMGecko.StartAngle[LH_J1],LH_J1);
	
				Angle(theta[3][0]+KMGecko.StartAngle[RH_J3],RH_J3);
				Angle(theta[3][1]+KMGecko.StartAngle[RH_J2],RH_J2);
				Angle(a[3][2]+KMGecko.StartAngle[RH_J1],RH_J1);		
		if(s>=T)
		{
			s=0;
		}
		
}
void movement_tripod(void)
{
	float a[4][3] = {0};     //   a1 = a[0]   a2=a[LF]
	float theta[4][2] = {0};
	delta=delta*3.1415/180;
  float alpha;
	float belta;
	D = sqrt((x_b+F2)*(x_b+F2)+(y_b+F1)*(y_b+F1));
	B = sqrt(x_b*x_b+y_b*y_b);
	alpha = acos((x_b+F2)/D);
	belta = acos(x_b/B);
	float extra = 30;
	float L_onestep1=1*L_onestep;
	if(s<T/4&&s>=0)
	{	
		x[0]=-B*cos(belta+delta/4*s/T*4)-L_onestep1/4*4*s/T+L2+x_b;
		y[0]=-B*sin(belta+delta/4*s/T*4)-W_onestep/4*4*s/T+L1+y_b;
		z[0]=-L3;
		
		x[1]=(D*cos(alpha-delta*s/T*4)-B*cos(belta-delta/4*s/T*4)+L_onestep1*s/T*4-L_onestep1*4*s/T/4);
		y[1]=-D*sin(alpha-delta*s/T*4)+B*sin(belta-delta/4*s/T*4)+W_onestep*4*s/T*3/4;
		z[1]=-L3+H_onestep-H_onestep*(sqrt(L_onestep1*L_onestep1+W_onestep*W_onestep)*4*s/T*3/4- sqrt(L_onestep1*L_onestep1+W_onestep*W_onestep)/8*3)*
		(sqrt(L_onestep1*L_onestep1+W_onestep*W_onestep)*4*s/T*3/4- sqrt(L_onestep1*L_onestep1+W_onestep*W_onestep)/8*3)/((L_onestep1*L_onestep1+W_onestep*W_onestep)/8/8*3*3);
		
		x[2]=B*cos(belta-delta/4*s/T*4)-L_onestep/4*4*s/T-L2-x_b;
		y[2]=-B*sin(belta-delta/4*s/T*4)-W_onestep/4*4*s/T+L1+y_b;
		z[2]=-L3;
		
		x[3]=B*cos(belta+delta/4*s/T*4)-L_onestep/4*4*s/T-L2-x_b;
		y[3]=B*sin(belta+delta/4*s/T*4)-W_onestep/4*4*s/T-L1-y_b;
		z[3]=-L3;
		}
		if(s>=T/4&&s<T/2)
		{
			x[0]=-B*cos(belta+delta/4*s/T*4)-L_onestep1/4*4*s/T+L2+x_b;
			y[0]=-B*sin(belta+delta/4*s/T*4)-W_onestep/4*4*s/T+L1+y_b;
			z[0]=-L3;
			
			x[1]=(D*cos(alpha-delta)-B*cos(belta-delta/4*s/T*4)+L_onestep1-L_onestep1*4*s/T/4);
			y[1]=-D*sin(alpha-delta)+B*sin(belta-delta/4*s/T*4)+W_onestep-W_onestep*4*s/T/4;
			z[1]=-L3;
			
			x[2]=B*cos(belta-delta/4*s/T*4)-L_onestep/4*4*s/T-L2-x_b;
			y[2]=-B*sin(belta-delta/4*s/T*4)-W_onestep/4*4*s/T+L1+y_b;
			z[2]=-L3;
			
			x[3]=-D*cos(alpha+delta*(s-T/4)/T*4)+B*cos(belta+delta/4*s/T*4)+L_onestep*4*(s-T/4)/T-L_onestep*4*s/T/4;
			y[3]=-D*sin(alpha+delta*(s-T/4)/T*4)+B*sin(belta+delta/4*s/T*4)+W_onestep*4*(s-T/4)/T-W_onestep*4*s/T/4;
			z[3]=-L3+H_onestep-H_onestep*(sqrt(L_onestep*L_onestep+W_onestep*W_onestep)*4*(s-T/4)/T*3/4-sqrt(L_onestep*L_onestep+W_onestep*W_onestep)/8*3)*
			(sqrt(L_onestep*L_onestep+W_onestep*W_onestep)*4*(s-T/4)/T*3/4- sqrt(L_onestep*L_onestep+W_onestep*W_onestep)/8*3)/((L_onestep*L_onestep+W_onestep*W_onestep)/8*3/8*3);
		}
		if(s>=T/2&&s<T*3/4)
		{
			x[0]=D*cos(alpha+delta*(s-T/2)/T*4)-B*cos(belta+delta/4*s/T*4)+L_onestep1*4*(s-T/2)/T-L_onestep1*4*s/T/4;
			y[0]=D*sin(alpha+delta*(s-T/2)/T*4)-B*sin(belta+delta/4*s/T*4)+W_onestep*4*(s-T/2)/T-W_onestep*4*s/T/4;
			z[0]=-L3+H_onestep-H_onestep*(sqrt(L_onestep1*L_onestep1+W_onestep*W_onestep)*4*(s-T/2)/T*3/4-sqrt(L_onestep1*L_onestep1+W_onestep*W_onestep)/8*3)*
			(sqrt(L_onestep1*L_onestep1+W_onestep*W_onestep)*4*(s-T/2)/T*3/4- sqrt(L_onestep1*L_onestep1+W_onestep*W_onestep)/8*3)/((L_onestep1*L_onestep1+W_onestep*W_onestep)/8*3/8*3);

			x[1]=(D*cos(alpha-delta)-B*cos(belta-delta/4*s/T*4)+L_onestep1-L_onestep1*4*s/T/4);
			y[1]=-D*sin(alpha-delta)+B*sin(belta-delta/4*s/T*4)+W_onestep-W_onestep*4*s/T/4;
			z[1]=-L3;
			
			x[2]=B*cos(belta-delta/4*s/T*4)-L_onestep/4*4*s/T-L2-x_b;
			y[2]=-B*sin(belta-delta/4*s/T*4)-W_onestep/4*4*s/T+L1+y_b;
			z[2]=-L3;
			
			x[3]=-D*cos(alpha+delta)+B*cos(belta+delta/4*s/T*4)+L_onestep-L_onestep*4*s/T/4;
			y[3]=-D*sin(alpha+delta)+B*sin(belta+delta/4*s/T*4)+W_onestep-W_onestep*4*s/T/4;
			z[3]=-L3;
		}
		if(s>=3*T/4&&s<T)
		{
			x[0]=D*cos(alpha+delta)-B*cos(belta+delta/4*s/T*4)+L_onestep1-L_onestep1*4*s/T/4;
			y[0]=D*sin(alpha+delta)-B*sin(belta+delta/4*s/T*4)+W_onestep-W_onestep*4*s/T/4;
			z[0]=-L3;
			
			x[1]=(D*cos(alpha-delta)-B*cos(belta-delta/4*s/T*4)+L_onestep1-L_onestep1*4*s/T/4);
			y[1]=-D*sin(alpha-delta)+B*sin(belta-delta/4*s/T*4)+W_onestep-W_onestep*4*s/T/4;
			z[1]=-L3;
			
			x[2]=-D*cos(alpha+delta*(s-3*T/4)/T*4)+B*cos(belta-delta/4*(s-3*T/4)/T*4-3*delta/4)+L_onestep*4*(s-3*T/4)/T-L_onestep*4*(s-3*T/4)/T/4-3*L_onestep/4;
			y[2]=D*sin(alpha+delta*(s-3*T/4)/T*4)-B*sin(belta-delta/4*(s-3*T/4)/T*4-3*delta/4)+W_onestep*4*(s-3*T/4)/T-W_onestep*4*(s-3*T/4)/T/4-3*W_onestep/4;
			z[2]=-L3+H_onestep-H_onestep*(sqrt(L_onestep*L_onestep+W_onestep*W_onestep)*4*(s-3*T/4)/T*3/4-sqrt(L_onestep*L_onestep+W_onestep*W_onestep)/8*3)*
			(sqrt(L_onestep*L_onestep+W_onestep*W_onestep)*4*(s-3*T/4)/T*3/4- sqrt(L_onestep*L_onestep+W_onestep*W_onestep)/8*3)/((L_onestep*L_onestep+W_onestep*W_onestep)/8*3/8*3);
			
			x[3]=-D*cos(alpha+delta)+B*cos(belta+delta/4*s/T*4)+L_onestep-L_onestep*4*s/T/4;
			y[3]=-D*sin(alpha+delta)+B*sin(belta+delta/4*s/T*4)+W_onestep-W_onestep*4*s/T/4;
			z[3]=-L3;
			


		}
			a[0][0] = -asin(L3 / sqrt(z[0] * z[0] + y[0] * y[0]) ) - atan2(z[0],y[0]);
			a[0][1] = (asin((y[0] * y[0] + x[0] * x[0] + z[0] * z[0] + L1 * L1 - L2 * L2 - L3 * L3) / ( 2 * L1 * sqrt (y[0] * y[0] +  x[0] * x[0] + z[0] * z[0] - L3 * L3)) )
											- atan2(sqrt(y[0] * y[0] + z[0] * z[0] - L3 * L3) , x[0]));
			a[0][2] = asin((L1 * L1 + L2 * L2 + L3 * L3 - y[0] * y[0] - x[0] * x[0] - z[0] * z[0]) / (2 * L1 * L2));
	
			a[1][0] = asin(L3 / sqrt(y[1] * y[1] + z[1] * z[1]) ) + atan2(z[1],-y[1]);
			a[1][1] = -asin((y[1] * y[1] + x[1] * x[1] + z[1] * z[1] + L1 * L1 - L2 * L2 - L3 * L3) / ( 2 * L1 * sqrt (y[1] * y[1] +  x[1] * x[1] + z[1] * z[1] - L3 * L3)) )
											+ atan2(sqrt(y[1] * y[1] + z[1] * z[1] - L3 * L3) , x[1]);
			a[1][2] = -asin((L1 * L1 + L2 * L2 + L3 * L3 - y[1] * y[1] - x[1] * x[1] - z[1] * z[1]) / (2 * L1 * L2));
	
			a[2][0] = -asin(L3 / sqrt(y[2] * y[2] + z[2] * z[2]) ) - atan2(z[2],y[2]);
			a[2][1] = -(asin((y[2] * y[2] + x[2] * x[2] + z[2] * z[2] + L1 * L1 - L2 * L2 - L3 * L3) / ( 2 * L1 * sqrt (y[2] * y[2] +  x[2] * x[2] + z[2] * z[2] - L3 * L3)) 
											 - atan2(sqrt(y[2] * y[2] + z[2] * z[2] - L3 * L3) , -x[2])));
			a[2][2] = -asin((L1 * L1 + L2 * L2 + L3 * L3 - y[2] * y[2] - x[2] * x[2] - z[2] * z[2]) / (2 * L1 * L2));
	
			a[3][0] = asin(L3 / sqrt(y[3] * y[3] + z[3] * z[3]) ) + atan2(z[3],-y[3]);
			a[3][1] = asin((y[3] * y[3] + x[3] * x[3] + z[3] * z[3] + L1 * L1 - L2 * L2 - L3 * L3) / ( 2 * L1 * sqrt (y[3] * y[3] +  x[3] * x[3] + z[3] * z[3] - L3 * L3)) )
											- atan2(sqrt(y[3] * y[3] + z[3] * z[3] - L3 * L3) , -x[3]);
			a[3][2] = asin((L1 * L1 + L2 * L2 + L3 * L3 - y[3] * y[3] - x[3] * x[3] - z[3] * z[3]) / (2 * L1 * L2));
				theta[0][0]=(a[0][0]+a[0][1]);
				theta[0][1]=(a[0][1]-a[0][0]);
				
				theta[0][0]=(int) (theta[0][0]/3.1415926*180*100);//rf alpha 角，最内侧
				theta[0][1]=(int) (theta[0][1]/3.1415926*180*100);
				a[0][2]=(int) (a[0][2]/3.1415926*180*100);

				theta[0][0]= (float) (theta[0][0]/100);
				theta[0][1]= (float) (theta[0][1]/100);
				a[0][2]= (float) (a[0][2]/100);
				
				theta[1][0]=(a[1][0]+a[1][1]);
				theta[1][1]=(a[1][1]-a[1][0]);
				
				theta[1][0]=(int) (theta[1][0]/3.1415926*180*100);//rf alpha 角，最内侧
				theta[1][1]=(int) (theta[1][1]/3.1415926*180*100);
				a[1][2]=(int) (a[1][2]/3.1415926*180*100);
			
				theta[1][0]= (float) (theta[1][0]/100);
				theta[1][1]= (float) (theta[1][1]/100);
				a[1][2]= (float) (a[1][2]/100);

				theta[2][0]=(a[2][1]-a[2][0]);
				theta[2][1]=(a[2][1]+a[2][0]);
				
				theta[2][0]=(int) (theta[2][0]/3.1415926*180*100);//rf alpha 角，最内侧
				theta[2][1]=(int) (theta[2][1]/3.1415926*180*100);
				a[2][2]=(int) (a[2][2]/3.1415926*180*100);

				theta[2][0]= (float) (theta[2][0]/100);
				theta[2][1]= (float) (theta[2][1]/100);
				a[2][2]= (float) (a[2][2]/100);
				
				theta[3][0]=(a[3][1]-a[3][0]);
				theta[3][1]=(a[3][1]+a[3][0]);
				
				theta[3][0]=(int) (theta[3][0]/3.1415926*180*100);//rf alpha 角，最内侧
				theta[3][1]=(int) (theta[3][1]/3.1415926*180*100);
				a[3][2]=(int) (a[3][2]/3.1415926*180*100);

				theta[3][0]= (float) (theta[3][0]/100);
				theta[3][1]= (float) (theta[3][1]/100);
				a[3][2]= (float) (a[3][2]/100);
			
				Angle(theta[0][0]+KMGecko.StartAngle[LF_J3],LF_J3);
				Angle(theta[0][1]+KMGecko.StartAngle[LF_J2],LF_J2);
				Angle(a[0][2]+KMGecko.StartAngle[LF_J1],LF_J1);
			
				Angle(theta[1][0]+KMGecko.StartAngle[RF_J3],RF_J3);
				Angle(theta[1][1]+KMGecko.StartAngle[RF_J2],RF_J2);
				Angle(a[1][2]+KMGecko.StartAngle[RF_J1],RF_J1);
					
				Angle(theta[2][0]+KMGecko.StartAngle[LH_J3],LH_J3);
				Angle(theta[2][1]+KMGecko.StartAngle[LH_J2],LH_J2);
				Angle(a[2][2]+KMGecko.StartAngle[LH_J1],LH_J1);
	
				Angle(theta[3][0]+KMGecko.StartAngle[RH_J3],RH_J3);
				Angle(theta[3][1]+KMGecko.StartAngle[RH_J2],RH_J2);
				Angle(a[3][2]+KMGecko.StartAngle[RH_J1],RH_J1);		
		if(s>=T)
		{
			s=0;
		}
		
}
void movement_tripod_br(void)
{
	float a[4][3] = {0};     //   a1 = a[0]   a2=a[LF]
	float theta[4][2] = {0};
	delta=delta*3.1415/180;
  float alpha;
	float belta;
	D = sqrt((x_b+F2)*(x_b+F2)+(y_b+F1)*(y_b+F1));
	B = sqrt(x_b*x_b+y_b*y_b);
	alpha = acos((x_b+F2)/D);
	belta = acos(x_b/B);
	float extra = 30;
	float L_onestep1=1*L_onestep;
	if(s<T/4*7/8&&s>=0)
	{	
		x[0]=-B*cos(belta+delta/4*s/T*4)-L_onestep/4*4*s/T+L2+x_b;
		y[0]=-B*sin(belta+delta/4*s/T*4)-W_onestep/4*4*s/T+L1+y_b;
		z[0]=-L3;
		
		x[1]=(D*cos(alpha-delta*s/T*4)-B*cos(belta-delta/4*s/T*4)+L_onestep*s/T*4-L_onestep*4*s/T/4);
		y[1]=-D*sin(alpha-delta*s/T*4)+B*sin(belta-delta/4*s/T*4)+W_onestep*4*s/T*3/4;
//		z[1]=-L3+H_onestep-H_onestep*(sqrt(L_onestep*L_onestep+W_onestep*W_onestep)*4*s/T*3/4*7/8- sqrt(L_onestep*L_onestep+W_onestep*W_onestep)/8*3*7/8)*
//		(sqrt(L_onestep*L_onestep+W_onestep*W_onestep)*4*s/T*3/4*7/8- sqrt(L_onestep*L_onestep+W_onestep*W_onestep)/8*3*7/8)/((L_onestep*L_onestep+W_onestep*W_onestep)/8/8*3*3);
		z[1]=-L3+H_onestep-H_onestep*(sqrt(L_onestep*L_onestep+W_onestep*W_onestep)*4*s/T*3/4- sqrt(L_onestep*L_onestep+W_onestep*W_onestep)/32*10)*
		(sqrt(L_onestep*L_onestep+W_onestep*W_onestep)*4*s/T*3/4- sqrt(L_onestep*L_onestep+W_onestep*W_onestep)/32*10)/((L_onestep*L_onestep+W_onestep*W_onestep)/32/32*10*10);
		
		
		x[2]=B*cos(belta-delta/4*s/T*4)-L_onestep/4*4*s/T-L2-x_b;
		y[2]=-B*sin(belta-delta/4*s/T*4)-W_onestep/4*4*s/T+L1+y_b;
		z[2]=-L3;
		
		x[3]=B*cos(belta+delta/4*s/T*4)-L_onestep/4*4*s/T-L2-x_b;
		y[3]=B*sin(belta+delta/4*s/T*4)-W_onestep/4*4*s/T-L1-y_b;
		z[3]=-L3;
		}
	if(s>=T/32*7&&s<T/4)
	{	
		x[0]=-B*cos(belta+delta/4*s/T*4)-L_onestep/4*4*s/T+L2+x_b;
		y[0]=-B*sin(belta+delta/4*s/T*4)-W_onestep/4*4*s/T+L1+y_b;
		z[0]=-L3;
		
		x[1]=(D*cos(alpha-delta*s/T*4)-B*cos(belta-delta/4*s/T*4)+L_onestep*s/T*4-L_onestep*4*s/T/4);
		y[1]=-D*sin(alpha-delta*s/T*4)+B*sin(belta-delta/4*s/T*4)+W_onestep*4*s/T*3/4;
		z[1]=-L3-0.21*H_onestep+16*(s-7*T/32)/T*0.21*H_onestep;
	
		x[2]=B*cos(belta-delta/4*s/T*4)-L_onestep/4*4*s/T-L2-x_b;
		y[2]=-B*sin(belta-delta/4*s/T*4)-W_onestep/4*4*s/T+L1+y_b;
		z[2]=-L3;
		
		x[3]=B*cos(belta+delta/4*s/T*4)-L_onestep/4*4*s/T-L2-x_b;
		y[3]=B*sin(belta+delta/4*s/T*4)-W_onestep/4*4*s/T-L1-y_b;
		z[3]=-L3;
		}
		if(s>=T/4&&s<7*T/16)
		{

			x[0]=D*cos(alpha+delta*(s-T/4)/T*4)-B*cos(belta+delta/4*s/T*4)+L_onestep*4*(s-T/4)/T-L_onestep*4*s/T/4;
			y[0]=D*sin(alpha+delta*(s-T/4)/T*4)-B*sin(belta+delta/4*s/T*4)+W_onestep*4*(s-T/4)/T-W_onestep*4*s/T/4;
			z[0]=-L3+H_onestep-H_onestep*(sqrt(L_onestep*L_onestep+W_onestep*W_onestep)*4*(s-T/4)/T*3/4-sqrt(L_onestep*L_onestep1+W_onestep*W_onestep)/32*8)*
			(sqrt(L_onestep*L_onestep+W_onestep*W_onestep)*4*(s-T/4)/T*3/4- sqrt(L_onestep*L_onestep+W_onestep*W_onestep)/32*8)/((L_onestep1*L_onestep+W_onestep*W_onestep)/32/32*8*8);
						
			x[1]=(D*cos(alpha-delta)-B*cos(belta-delta/4*s/T*4)+L_onestep-L_onestep*4*s/T/4);
			y[1]=-D*sin(alpha-delta)+B*sin(belta-delta/4*s/T*4)+W_onestep-W_onestep*4*s/T/4;
			z[1]=-L3;
			
			x[2]=B*cos(belta-delta/4*s/T*4)-L_onestep/4*4*s/T-L2-x_b;
			y[2]=-B*sin(belta-delta/4*s/T*4)-W_onestep/4*4*s/T+L1+y_b;
			z[2]=-L3;
			
			
			x[3]=B*cos(belta+delta/4*s/T*4)-L_onestep/4*4*s/T-L2-x_b;
			y[3]=B*sin(belta+delta/4*s/T*4)-W_onestep/4*4*s/T-L1-y_b;
			z[3]=-L3;
		}
		if(s>=7*T/16&&s<T/2)
		{
			x[0]=D*cos(alpha+delta*(s-T/4)/T*4)-B*cos(belta+delta/4*s/T*4)+L_onestep*4*(s-T/4)/T-L_onestep*4*s/T/4;
			y[0]=D*sin(alpha+delta*(s-T/4)/T*4)-B*sin(belta+delta/4*s/T*4)+W_onestep*4*(s-T/4)/T-W_onestep*4*s/T/4;
			z[0]=-L3-0.5625*H_onestep+16*(s-7*T/16)/T*0.5625*H_onestep;
			
			x[1]=(D*cos(alpha-delta)-B*cos(belta-delta/4*s/T*4)+L_onestep-L_onestep*4*s/T/4);
			y[1]=-D*sin(alpha-delta)+B*sin(belta-delta/4*s/T*4)+W_onestep-W_onestep*4*s/T/4;
			z[1]=-L3;
			
			x[2]=B*cos(belta-delta/4*s/T*4)-L_onestep/4*4*s/T-L2-x_b;
			y[2]=-B*sin(belta-delta/4*s/T*4)-W_onestep/4*4*s/T+L1+y_b;
			z[2]=-L3;
			
			
			x[3]=B*cos(belta+delta/4*s/T*4)-L_onestep/4*4*s/T-L2-x_b;
			y[3]=B*sin(belta+delta/4*s/T*4)-W_onestep/4*4*s/T-L1-y_b;
			z[3]=-L3;
		}
		if(s>=T/2&&s<T*11/16)
		{
			x[0]=D*cos(alpha+delta)-B*cos(belta+delta/4*s/T*4)+L_onestep-L_onestep*4*s/T/4;
			y[0]=D*sin(alpha+delta)-B*sin(belta+delta/4*s/T*4)+W_onestep-W_onestep*4*s/T/4;
			z[0]=-L3;

			x[1]=(D*cos(alpha-delta)-B*cos(belta-delta/4*s/T*4)+L_onestep1-L_onestep1*4*s/T/4);
			y[1]=-D*sin(alpha-delta)+B*sin(belta-delta/4*s/T*4)+W_onestep-W_onestep*4*s/T/4;
			z[1]=-L3;
			
			x[2]=B*cos(belta-delta/4*s/T*4)-L_onestep/4*4*s/T-L2-x_b;
			y[2]=-B*sin(belta-delta/4*s/T*4)-W_onestep/4*4*s/T+L1+y_b;
			z[2]=-L3;
			

			x[3]=-D*cos(alpha+delta*(s-T/2)/T*4)+B*cos(belta+delta/4*s/T*4)+L_onestep*4*(s-T/2)/T-L_onestep*4*s/T/4;
			y[3]=-D*sin(alpha+delta*(s-T/2)/T*4)+B*sin(belta+delta/4*s/T*4)+W_onestep*4*(s-T/2)/T-W_onestep*4*s/T/4;
			z[3]=-L3+H_onestep-H_onestep*(sqrt(L_onestep*L_onestep+W_onestep*W_onestep)*4*(s-T/2)/T*3/4-sqrt(L_onestep*L_onestep+W_onestep*W_onestep)/32*8)*
			(sqrt(L_onestep*L_onestep+W_onestep*W_onestep)*4*(s-T/2)/T*3/4- sqrt(L_onestep*L_onestep+W_onestep*W_onestep)/32*8)/((L_onestep*L_onestep+W_onestep*W_onestep)/32/32*8*8);
		}
			if(s>=T*11/16&&s<T*3/4)
		{
			x[0]=D*cos(alpha+delta)-B*cos(belta+delta/4*s/T*4)+L_onestep-L_onestep*4*s/T/4;
			y[0]=D*sin(alpha+delta)-B*sin(belta+delta/4*s/T*4)+W_onestep-W_onestep*4*s/T/4;
			z[0]=-L3;

			x[1]=(D*cos(alpha-delta)-B*cos(belta-delta/4*s/T*4)+L_onestep1-L_onestep1*4*s/T/4);
			y[1]=-D*sin(alpha-delta)+B*sin(belta-delta/4*s/T*4)+W_onestep-W_onestep*4*s/T/4;
			z[1]=-L3;
			
			x[2]=B*cos(belta-delta/4*s/T*4)-L_onestep/4*4*s/T-L2-x_b;
			y[2]=-B*sin(belta-delta/4*s/T*4)-W_onestep/4*4*s/T+L1+y_b;
			z[2]=-L3;
			
			x[3]=-D*cos(alpha+delta*(s-T/2)/T*4)+B*cos(belta+delta/4*s/T*4)+L_onestep*4*(s-T/2)/T-L_onestep*4*s/T/4;
			y[3]=-D*sin(alpha+delta*(s-T/2)/T*4)+B*sin(belta+delta/4*s/T*4)+W_onestep*4*(s-T/2)/T-W_onestep*4*s/T/4;
			z[3]=-L3-0.5625*H_onestep+16*(s-11*T/16)/T*0.5625*H_onestep;
		}
		if(s>=3*T/4&&s<15*T/16)
		{
			x[0]=D*cos(alpha+delta)-B*cos(belta+delta/4*s/T*4)+L_onestep-L_onestep*4*s/T/4;
			y[0]=D*sin(alpha+delta)-B*sin(belta+delta/4*s/T*4)+W_onestep-W_onestep*4*s/T/4;
			z[0]=-L3;
			
			x[1]=(D*cos(alpha-delta)-B*cos(belta-delta/4*s/T*4)+L_onestep-L_onestep*4*s/T/4);
			y[1]=-D*sin(alpha-delta)+B*sin(belta-delta/4*s/T*4)+W_onestep-W_onestep*4*s/T/4;
			z[1]=-L3;
			
			x[2]=-D*cos(alpha+delta*(s-3*T/4)/T*4)+B*cos(belta-delta/4*(s-3*T/4)/T*4-3*delta/4)+L_onestep*4*(s-3*T/4)/T-L_onestep*4*(s-3*T/4)/T/4-3*L_onestep/4;
			y[2]=D*sin(alpha+delta*(s-3*T/4)/T*4)-B*sin(belta-delta/4*(s-3*T/4)/T*4-3*delta/4)+W_onestep*4*(s-3*T/4)/T-W_onestep*4*(s-3*T/4)/T/4-3*W_onestep/4;
			z[2]=-L3+H_onestep-H_onestep*(sqrt(L_onestep*L_onestep+W_onestep*W_onestep)*4*(s-3*T/4)/T*3/4-sqrt(L_onestep*L_onestep+W_onestep*W_onestep)/32*8)*
			(sqrt(L_onestep*L_onestep+W_onestep*W_onestep)*4*(s-3*T/4)/T*3/4- sqrt(L_onestep*L_onestep+W_onestep*W_onestep)/32*8)/((L_onestep*L_onestep+W_onestep*W_onestep)/32/32*8*8);
			
			x[3]=-D*cos(alpha+delta)+B*cos(belta+delta/4*s/T*4)+L_onestep-L_onestep*4*s/T/4;
			y[3]=-D*sin(alpha+delta)+B*sin(belta+delta/4*s/T*4)+W_onestep-W_onestep*4*s/T/4;
			z[3]=-L3;
			
		}
			if(s>=15*T/16&&s<T)
		{
			x[0]=D*cos(alpha+delta)-B*cos(belta+delta/4*s/T*4)+L_onestep-L_onestep*4*s/T/4;
			y[0]=D*sin(alpha+delta)-B*sin(belta+delta/4*s/T*4)+W_onestep-W_onestep*4*s/T/4;
			z[0]=-L3;
			
			x[1]=(D*cos(alpha-delta)-B*cos(belta-delta/4*s/T*4)+L_onestep-L_onestep*4*s/T/4);
			y[1]=-D*sin(alpha-delta)+B*sin(belta-delta/4*s/T*4)+W_onestep-W_onestep*4*s/T/4;
			z[1]=-L3;
			
			x[2]=-D*cos(alpha+delta*(s-3*T/4)/T*4)+B*cos(belta-delta/4*(s-3*T/4)/T*4-3*delta/4)+L_onestep*4*(s-3*T/4)/T-L_onestep*4*(s-3*T/4)/T/4-3*L_onestep/4;
			y[2]=D*sin(alpha+delta*(s-3*T/4)/T*4)-B*sin(belta-delta/4*(s-3*T/4)/T*4-3*delta/4)+W_onestep*4*(s-3*T/4)/T-W_onestep*4*(s-3*T/4)/T/4-3*W_onestep/4;
			z[2]=-L3-0.5625*H_onestep+16*(s-15*T/16)/T*0.5625*H_onestep;
			
			x[3]=-D*cos(alpha+delta)+B*cos(belta+delta/4*s/T*4)+L_onestep-L_onestep*4*s/T/4;
			y[3]=-D*sin(alpha+delta)+B*sin(belta+delta/4*s/T*4)+W_onestep-W_onestep*4*s/T/4;
			z[3]=-L3;
		}
			a[0][0] = -asin(L3 / sqrt(z[0] * z[0] + y[0] * y[0]) ) - atan2(z[0],y[0]);
			a[0][1] = (asin((y[0] * y[0] + x[0] * x[0] + z[0] * z[0] + L1 * L1 - L2 * L2 - L3 * L3) / ( 2 * L1 * sqrt (y[0] * y[0] +  x[0] * x[0] + z[0] * z[0] - L3 * L3)) )
											- atan2(sqrt(y[0] * y[0] + z[0] * z[0] - L3 * L3) , x[0]));
			a[0][2] = asin((L1 * L1 + L2 * L2 + L3 * L3 - y[0] * y[0] - x[0] * x[0] - z[0] * z[0]) / (2 * L1 * L2));
	
			a[1][0] = asin(L3 / sqrt(y[1] * y[1] + z[1] * z[1]) ) + atan2(z[1],-y[1]);
			a[1][1] = -asin((y[1] * y[1] + x[1] * x[1] + z[1] * z[1] + L1 * L1 - L2 * L2 - L3 * L3) / ( 2 * L1 * sqrt (y[1] * y[1] +  x[1] * x[1] + z[1] * z[1] - L3 * L3)) )
											+ atan2(sqrt(y[1] * y[1] + z[1] * z[1] - L3 * L3) , x[1]);
			a[1][2] = -asin((L1 * L1 + L2 * L2 + L3 * L3 - y[1] * y[1] - x[1] * x[1] - z[1] * z[1]) / (2 * L1 * L2));
	
			a[2][0] = -asin(L3 / sqrt(y[2] * y[2] + z[2] * z[2]) ) - atan2(z[2],y[2]);
			a[2][1] = -(asin((y[2] * y[2] + x[2] * x[2] + z[2] * z[2] + L1 * L1 - L2 * L2 - L3 * L3) / ( 2 * L1 * sqrt (y[2] * y[2] +  x[2] * x[2] + z[2] * z[2] - L3 * L3)) 
											 - atan2(sqrt(y[2] * y[2] + z[2] * z[2] - L3 * L3) , -x[2])));
			a[2][2] = -asin((L1 * L1 + L2 * L2 + L3 * L3 - y[2] * y[2] - x[2] * x[2] - z[2] * z[2]) / (2 * L1 * L2));
	
			a[3][0] = asin(L3 / sqrt(y[3] * y[3] + z[3] * z[3]) ) + atan2(z[3],-y[3]);
			a[3][1] = asin((y[3] * y[3] + x[3] * x[3] + z[3] * z[3] + L1 * L1 - L2 * L2 - L3 * L3) / ( 2 * L1 * sqrt (y[3] * y[3] +  x[3] * x[3] + z[3] * z[3] - L3 * L3)) )
											- atan2(sqrt(y[3] * y[3] + z[3] * z[3] - L3 * L3) , -x[3]);
			a[3][2] = asin((L1 * L1 + L2 * L2 + L3 * L3 - y[3] * y[3] - x[3] * x[3] - z[3] * z[3]) / (2 * L1 * L2));
				theta[0][0]=(a[0][0]+a[0][1]);
				theta[0][1]=(a[0][1]-a[0][0]);
				
				theta[0][0]=(int) (theta[0][0]/3.1415926*180*100);//rf alpha 角，最内侧
				theta[0][1]=(int) (theta[0][1]/3.1415926*180*100);
				a[0][2]=(int) (a[0][2]/3.1415926*180*100);

				theta[0][0]= (float) (theta[0][0]/100);
				theta[0][1]= (float) (theta[0][1]/100);
				a[0][2]= (float) (a[0][2]/100);
				
				theta[1][0]=(a[1][0]+a[1][1]);
				theta[1][1]=(a[1][1]-a[1][0]);
				
				theta[1][0]=(int) (theta[1][0]/3.1415926*180*100);//rf alpha 角，最内侧
				theta[1][1]=(int) (theta[1][1]/3.1415926*180*100);
				a[1][2]=(int) (a[1][2]/3.1415926*180*100);
			
				theta[1][0]= (float) (theta[1][0]/100);
				theta[1][1]= (float) (theta[1][1]/100);
				a[1][2]= (float) (a[1][2]/100);

				theta[2][0]=(a[2][1]-a[2][0]);
				theta[2][1]=(a[2][1]+a[2][0]);
				
				theta[2][0]=(int) (theta[2][0]/3.1415926*180*100);//rf alpha 角，最内侧
				theta[2][1]=(int) (theta[2][1]/3.1415926*180*100);
				a[2][2]=(int) (a[2][2]/3.1415926*180*100);

				theta[2][0]= (float) (theta[2][0]/100);
				theta[2][1]= (float) (theta[2][1]/100);
				a[2][2]= (float) (a[2][2]/100);
				
				theta[3][0]=(a[3][1]-a[3][0]);
				theta[3][1]=(a[3][1]+a[3][0]);
				
				theta[3][0]=(int) (theta[3][0]/3.1415926*180*100);//rf alpha 角，最内侧
				theta[3][1]=(int) (theta[3][1]/3.1415926*180*100);
				a[3][2]=(int) (a[3][2]/3.1415926*180*100);

				theta[3][0]= (float) (theta[3][0]/100);
				theta[3][1]= (float) (theta[3][1]/100);
				a[3][2]= (float) (a[3][2]/100);
			
				Angle(theta[0][0]+KMGecko.StartAngle[LF_J3],LF_J3);
				Angle(theta[0][1]+KMGecko.StartAngle[LF_J2],LF_J2);
				Angle(a[0][2]+KMGecko.StartAngle[LF_J1],LF_J1);
			
				Angle(theta[1][0]+KMGecko.StartAngle[RF_J3],RF_J3);
				Angle(theta[1][1]+KMGecko.StartAngle[RF_J2],RF_J2);
				Angle(a[1][2]+KMGecko.StartAngle[RF_J1],RF_J1);
					
				Angle(theta[2][0]+KMGecko.StartAngle[LH_J3],LH_J3);
				Angle(theta[2][1]+KMGecko.StartAngle[LH_J2],LH_J2);
				Angle(a[2][2]+KMGecko.StartAngle[LH_J1],LH_J1);
	
				Angle(theta[3][0]+KMGecko.StartAngle[RH_J3],RH_J3);
				Angle(theta[3][1]+KMGecko.StartAngle[RH_J2],RH_J2);
				Angle(a[3][2]+KMGecko.StartAngle[RH_J1],RH_J1);		
		if(s>=T)
		{
			s=0;
		}
		


}
void movement_tilt(void)
{
float a[4][3] = {0};     //   a1 = a[0]   a2=a[LF]
	float theta[4][2] = {0};
	delta=delta*3.1415/180;
  float alpha;
	float belta;
	float gamma;
	D = sqrt((x_b+F2)*(x_b+F2)+(y_b+F1)*(y_b+F1));
	B = sqrt(x_b*x_b+y_b*y_b);
	alpha = acos((x_b+F2)/D);
	belta = acos(x_b/B);
	gamma = 10*3.1415/180;
		if(s<T/4&&s>=0)
	{	
		x[0]=-B*cos(belta+delta/4*s/T*4)-L_onestep/4*4*s/T+L2+x_b;
		y[0]=-B*sin(belta+delta/4*s/T*4)-W_onestep/4*4*s/T+L1+y_b;
		z[0]=-L3;
		
		x[1]=D*cos(alpha-delta*s/T*4)-B*cos(belta-delta/4*s/T*4)+L_onestep*s/T*4-L_onestep*4*s/T/4;
		y[1]=-D*sin(alpha-delta*s/T*4)+B*sin(belta-delta/4*s/T*4)+W_onestep*4*s/T*3/4+sin(gamma)*sin(H_onestep-H_onestep*(sqrt(L_onestep*L_onestep+W_onestep*W_onestep)*4*s/T*3/4- sqrt(L_onestep*L_onestep+W_onestep*W_onestep)/8*3)*
		(sqrt(L_onestep*L_onestep+W_onestep*W_onestep)*4*s/T*3/4- sqrt(L_onestep*L_onestep+W_onestep*W_onestep)/8*3)/((L_onestep*L_onestep+W_onestep*W_onestep)/8/8*3*3));
		z[1]=-L3+cos(gamma)*(H_onestep-H_onestep*(sqrt(L_onestep*L_onestep+W_onestep*W_onestep)*4*s/T*3/4- sqrt(L_onestep*L_onestep+W_onestep*W_onestep)/8*3)*
		(sqrt(L_onestep*L_onestep+W_onestep*W_onestep)*4*s/T*3/4- sqrt(L_onestep*L_onestep+W_onestep*W_onestep)/8*3)/((L_onestep*L_onestep+W_onestep*W_onestep)/8/8*3*3));
		
		x[2]=B*cos(belta-delta/4*s/T*4)-L_onestep/4*4*s/T-L2-x_b;
		y[2]=-B*sin(belta-delta/4*s/T*4)-W_onestep/4*4*s/T+L1+y_b;
		z[2]=-L3;
		
		x[3]=B*cos(belta+delta/4*s/T*4)-L_onestep/4*4*s/T-L2-x_b;
		y[3]=B*sin(belta+delta/4*s/T*4)-W_onestep/4*4*s/T-L1-y_b;
		z[3]=-L3;
		}
		if(s>=T/4&&s<T/2)
		{
			x[0]=-B*cos(belta+delta/4*s/T*4)-L_onestep/4*4*s/T+L2+x_b;
			y[0]=-B*sin(belta+delta/4*s/T*4)-W_onestep/4*4*s/T+L1+y_b;
			z[0]=-L3;
			
			x[1]=D*cos(alpha-delta)-B*cos(belta-delta/4*s/T*4)+L_onestep-L_onestep*4*s/T/4;
			y[1]=-D*sin(alpha-delta)+B*sin(belta-delta/4*s/T*4)+W_onestep-W_onestep*4*s/T/4;
			z[1]=-L3;
			
			x[2]=B*cos(belta-delta/4*s/T*4)-L_onestep/4*4*s/T-L2-x_b;
			y[2]=-B*sin(belta-delta/4*s/T*4)-W_onestep/4*4*s/T+L1+y_b;
			z[2]=-L3;
			
			x[3]=-D*cos(alpha+delta*(s-T/4)/T*4)+B*cos(belta+delta/4*s/T*4)+L_onestep*4*(s-T/4)/T-L_onestep*4*s/T/4;
			y[3]=-D*sin(alpha+delta*(s-T/4)/T*4)+B*sin(belta+delta/4*s/T*4)+W_onestep*4*(s-T/4)/T-W_onestep*4*s/T/4+sin(gamma)*(H_onestep-H_onestep*(sqrt(L_onestep*L_onestep+W_onestep*W_onestep)*4*(s-T/4)/T*3/4-sqrt(L_onestep*L_onestep+W_onestep*W_onestep)/8*3)*
			(sqrt(L_onestep*L_onestep+W_onestep*W_onestep)*4*(s-T/4)/T*3/4- sqrt(L_onestep*L_onestep+W_onestep*W_onestep)/8*3)/((L_onestep*L_onestep+W_onestep*W_onestep)/8*3/8*3));
			z[3]=-L3+cos(gamma)*(H_onestep-H_onestep*(sqrt(L_onestep*L_onestep+W_onestep*W_onestep)*4*(s-T/4)/T*3/4-sqrt(L_onestep*L_onestep+W_onestep*W_onestep)/8*3)*
			(sqrt(L_onestep*L_onestep+W_onestep*W_onestep)*4*(s-T/4)/T*3/4- sqrt(L_onestep*L_onestep+W_onestep*W_onestep)/8*3)/((L_onestep*L_onestep+W_onestep*W_onestep)/8*3/8*3));
		}
		if(s>=T/2&&s<T*3/4)
		{
			x[0]=D*cos(alpha+delta*(s-T/2)/T*4)-B*cos(belta+delta/4*s/T*4)+L_onestep*4*(s-T/2)/T-L_onestep*4*s/T/4;
			y[0]=D*sin(alpha+delta*(s-T/2)/T*4)-B*sin(belta+delta/4*s/T*4)+W_onestep*4*(s-T/2)/T-W_onestep*4*s/T/4-sin(gamma)*(H_onestep-H_onestep*(sqrt(L_onestep*L_onestep+W_onestep*W_onestep)*4*(s-T/2)/T*3/4-sqrt(L_onestep*L_onestep+W_onestep*W_onestep)/8*3)*
			(sqrt(L_onestep*L_onestep+W_onestep*W_onestep)*4*(s-T/2)/T*3/4- sqrt(L_onestep*L_onestep+W_onestep*W_onestep)/8*3)/((L_onestep*L_onestep+W_onestep*W_onestep)/8*3/8*3));
			z[0]=-L3+cos(gamma)*(H_onestep-H_onestep*(sqrt(L_onestep*L_onestep+W_onestep*W_onestep)*4*(s-T/2)/T*3/4-sqrt(L_onestep*L_onestep+W_onestep*W_onestep)/8*3)*
			(sqrt(L_onestep*L_onestep+W_onestep*W_onestep)*4*(s-T/2)/T*3/4- sqrt(L_onestep*L_onestep+W_onestep*W_onestep)/8*3)/((L_onestep*L_onestep+W_onestep*W_onestep)/8*3/8*3));

			x[1]=D*cos(alpha-delta)-B*cos(belta-delta/4*s/T*4)+L_onestep-L_onestep*4*s/T/4;
			y[1]=-D*sin(alpha-delta)+B*sin(belta-delta/4*s/T*4)+W_onestep-W_onestep*4*s/T/4;
			z[1]=-L3;
			
			x[2]=B*cos(belta-delta/4*s/T*4)-L_onestep/4*4*s/T-L2-x_b;
			y[2]=-B*sin(belta-delta/4*s/T*4)-W_onestep/4*4*s/T+L1+y_b;
			z[2]=-L3;
			
			x[3]=-D*cos(alpha+delta)+B*cos(belta+delta/4*s/T*4)+L_onestep-L_onestep*4*s/T/4;
			y[3]=-D*sin(alpha+delta)+B*sin(belta+delta/4*s/T*4)+W_onestep-W_onestep*4*s/T/4;
			z[3]=-L3;
		}
		if(s>=3*T/4&&s<T)
		{
			x[0]=D*cos(alpha+delta)-B*cos(belta+delta/4*s/T*4)+L_onestep-L_onestep*4*s/T/4;
			y[0]=D*sin(alpha+delta)-B*sin(belta+delta/4*s/T*4)+W_onestep-W_onestep*4*s/T/4;
			z[0]=-L3;
			
			x[1]=D*cos(alpha-delta)-B*cos(belta-delta/4*s/T*4)+L_onestep-L_onestep*4*s/T/4;
			y[1]=-D*sin(alpha-delta)+B*sin(belta-delta/4*s/T*4)+W_onestep-W_onestep*4*s/T/4;
			z[1]=-L3;
			
			x[2]=-D*cos(alpha+delta*(s-3*T/4)/T*4)+B*cos(belta-delta/4*(s-3*T/4)/T*4-3*delta/4)+L_onestep*4*(s-3*T/4)/T-L_onestep*4*(s-3*T/4)/T/4-3*L_onestep/4;
			y[2]=D*sin(alpha+delta*(s-3*T/4)/T*4)-B*sin(belta-delta/4*(s-3*T/4)/T*4-3*delta/4)+W_onestep*4*(s-3*T/4)/T-W_onestep*4*(s-3*T/4)/T/4-3*W_onestep/4-sin(gamma)*(H_onestep-H_onestep*(sqrt(L_onestep*L_onestep+W_onestep*W_onestep)*4*(s-3*T/4)/T*3/4-sqrt(L_onestep*L_onestep+W_onestep*W_onestep)/8*3)*
			(sqrt(L_onestep*L_onestep+W_onestep*W_onestep)*4*(s-3*T/4)/T*3/4- sqrt(L_onestep*L_onestep+W_onestep*W_onestep)/8*3)/((L_onestep*L_onestep+W_onestep*W_onestep)/8*3/8*3));
			z[2]=-L3+cos(gamma)*(H_onestep-H_onestep*(sqrt(L_onestep*L_onestep+W_onestep*W_onestep)*4*(s-3*T/4)/T*3/4-sqrt(L_onestep*L_onestep+W_onestep*W_onestep)/8*3)*
			(sqrt(L_onestep*L_onestep+W_onestep*W_onestep)*4*(s-3*T/4)/T*3/4- sqrt(L_onestep*L_onestep+W_onestep*W_onestep)/8*3)/((L_onestep*L_onestep+W_onestep*W_onestep)/8*3/8*3));
			
			x[3]=-D*cos(alpha+delta)+B*cos(belta+delta/4*s/T*4)+L_onestep-L_onestep*4*s/T/4;
			y[3]=-D*sin(alpha+delta)+B*sin(belta+delta/4*s/T*4)+W_onestep-W_onestep*4*s/T/4;
			z[3]=-L3;
			


		}
//	if(s<3*T/2/4&&s>=0)
//	{	
//		
//		x[0]=-B*cos(belta+delta/2*s/T*2)-L_onestep/2*2*s/T+D*cos(alpha);
//		y[0]=-B*sin(belta+delta/2*s/T*2)-W_onestep/2*2*s/T+D*sin(alpha);
//		z[0]=-L3;	
//		
//		x[1]=D*cos(alpha-delta*s/T*2*4/3)-B*cos(belta-delta/2*s/T*2)+L_onestep*s/T*2/2*8/3-L_onestep*2*s/T/2;
//		y[1]=-D*sin(alpha-delta*s/T*2*4/3)+B*sin(belta-delta/2*s/T*2)+W_onestep*2*s/T/2*8/3-W_onestep*2*s/T/2+sin(gamma)*(H_onestep-H_onestep*(sqrt(L_onestep*L_onestep+W_onestep*W_onestep)*2*s/T/2*4/3- sqrt(L_onestep*L_onestep+W_onestep*W_onestep)/4)*
//			(sqrt(L_onestep*L_onestep+W_onestep*W_onestep)*2*s/T/2*4/3- sqrt(L_onestep*L_onestep+W_onestep*W_onestep)/4)/((L_onestep*L_onestep+W_onestep*W_onestep)/16));
//		z[1]=-L3+cos(gamma)*(H_onestep-H_onestep*(sqrt(L_onestep*L_onestep+W_onestep*W_onestep)*2*s/T/2*4/3- sqrt(L_onestep*L_onestep+W_onestep*W_onestep)/4)*
//			(sqrt(L_onestep*L_onestep+W_onestep*W_onestep)*2*s/T/2*4/3- sqrt(L_onestep*L_onestep+W_onestep*W_onestep)/4)/((L_onestep*L_onestep+W_onestep*W_onestep)/16));
//			
//		x[2]= -D*cos(alpha-delta*s/T*2*4/3)+B*cos(belta-delta/2*s/T*2)+L_onestep*s/T*2/2*8/3-L_onestep*2*s/T/2;
//		y[2]= D*sin(alpha-delta*s/T*2*4/3)-B*sin(belta-delta/2*s/T*2)+W_onestep*2*s/T/2*8/3-W_onestep*2*s/T/2-sin(gamma)*(H_onestep-H_onestep*(sqrt(L_onestep*L_onestep+W_onestep*W_onestep)*2*s/T/2*4/3- sqrt(L_onestep*L_onestep+W_onestep*W_onestep)/4)*
//			(sqrt(L_onestep*L_onestep+W_onestep*W_onestep)*2*s/T/2*4/3- sqrt(L_onestep*L_onestep+W_onestep*W_onestep)/4)/((L_onestep*L_onestep+W_onestep*W_onestep)/16));
//		z[2]= -L3+cos(gamma)*(H_onestep-H_onestep*(sqrt(L_onestep*L_onestep+W_onestep*W_onestep)*2*s/T/2*4/3- sqrt(L_onestep*L_onestep+W_onestep*W_onestep)/4)*
//			(sqrt(L_onestep*L_onestep+W_onestep*W_onestep)*2*s/T/2*4/3- sqrt(L_onestep*L_onestep+W_onestep*W_onestep)/4)/((L_onestep*L_onestep+W_onestep*W_onestep)/16));		
//		
//		x[3]=B*cos(belta+delta/2*s/T*2)-L_onestep/2*2*s/T-D*cos(alpha);
//		y[3]=B*sin(belta+delta/2*s/T*2)-W_onestep/2*2*s/T-D*sin(alpha);
//		z[3]=-L3;
//		}
//	if(s>=3*T/2/4&&s<T/2)
//	{
//		x[0]=-B*cos(belta+delta/2*s/T*2)-L_onestep/2*2*s/T+D*cos(alpha);
//		y[0]=-B*sin(belta+delta/2*s/T*2)-W_onestep/2*2*s/T+D*sin(alpha);
//		z[0]=-L3;
//		
//		x[1]=D*cos(alpha-delta)-B*cos(belta-delta/2*s/T*2)+L_onestep-L_onestep*2*s/T/2;
//		y[1]=-D*sin(alpha-delta)+B*sin(belta-delta/2*s/T*2)+W_onestep-W_onestep*2*s/T/2;
//		z[1]=-L3;
//		
//		x[2]=-D*cos(alpha-delta)+B*cos(belta-delta/2*s/T*2)+L_onestep-L_onestep*2*s/T/2;
//		y[2]=D*sin(alpha-delta)-B*sin(belta-delta/2*s/T*2)+W_onestep-W_onestep*2*s/T/2;
//		z[2]=-L3;	
//		
//		x[3]=B*cos(belta+delta/2*s/T*2)-L_onestep/2*2*s/T-D*cos(alpha);
//		y[3]=B*sin(belta+delta/2*s/T*2)-W_onestep/2*2*s/T-D*sin(alpha);
//		z[3]=-L3;
//	}
//		if(s>=T/2&&s<7*T/8)
//		{
//		x[0]=D*cos(alpha+delta*(s-T/2)/T*2*4/3)-B*cos(belta+delta/2*s/T*2*4/3)+L_onestep*2*(s-T/2)/T*4/3-L_onestep*2*s/T/2;
//		y[0]=D*sin(alpha+delta*(s-T/2)/T*2*4/3)-B*sin(belta+delta/2*s/T*2*4/3)+W_onestep*2*(s-T/2)/T*4/3-W_onestep*2*s/T/2-sin(gamma)*(H_onestep-H_onestep*(sqrt(L_onestep*L_onestep+W_onestep*W_onestep)*2*(s-T/2)/T/2*4/3- sqrt(L_onestep*L_onestep+W_onestep*W_onestep)/4)*
//		(sqrt(L_onestep*L_onestep+W_onestep*W_onestep)*2*(s-T/2)/T/2*4/3- sqrt(L_onestep*L_onestep+W_onestep*W_onestep)/4)/((L_onestep*L_onestep+W_onestep*W_onestep)/16));
//		z[0]= -L3+cos(gamma)*(H_onestep-H_onestep*(sqrt(L_onestep*L_onestep+W_onestep*W_onestep)*2*(s-T/2)/T/2*4/3- sqrt(L_onestep*L_onestep+W_onestep*W_onestep)/4)*
//		(sqrt(L_onestep*L_onestep+W_onestep*W_onestep)*2*(s-T/2)/T/2*4/3- sqrt(L_onestep*L_onestep+W_onestep*W_onestep)/4)/((L_onestep*L_onestep+W_onestep*W_onestep)/16));

//		x[1]=D*cos(alpha-delta)-B*cos(belta-delta/2*s/T*2)+L_onestep-L_onestep*2*s/T/2;
//		y[1]=-D*sin(alpha-delta)+B*sin(belta-delta/2*s/T*2)+W_onestep-W_onestep*2*s/T/2;
//		z[1]=-L3;
//		
//		x[2]=-D*cos(alpha-delta)+B*cos(belta-delta/2*s/T*2)+L_onestep-L_onestep*2*s/T/2;
//		y[2]=D*sin(alpha-delta)-B*sin(belta-delta/2*s/T*2)+W_onestep-W_onestep*2*s/T/2;
//		z[2]=-L3;
//		
//		x[3]=-D*cos(alpha+delta*(s-T/2)/T*2*4/3)+B*cos(belta+delta/2*s/T*2)+L_onestep*2*(s-T/2)/T*4/3-L_onestep*2*s/T/2;
//		y[3]=-D*sin(alpha+delta*(s-T/2)/T*2*4/3)+B*sin(belta+delta/2*s/T*2)+W_onestep*2*(s-T/2)/T*4/3-W_onestep*2*s/T/2+sin(gamma)*(H_onestep-H_onestep*(sqrt(L_onestep*L_onestep+W_onestep*W_onestep)*2*(s-T/2)/T/2*4/3- sqrt(L_onestep*L_onestep+W_onestep*W_onestep)/4)*
//		(sqrt(L_onestep*L_onestep+W_onestep*W_onestep)*2*(s-T/2)/T/2*4/3- sqrt(L_onestep*L_onestep+W_onestep*W_onestep)/4)/((L_onestep*L_onestep+W_onestep*W_onestep)/16));
//		z[3]= -L3+cos(gamma)*(H_onestep-H_onestep*(sqrt(L_onestep*L_onestep+W_onestep*W_onestep)*2*(s-T/2)/T/2*4/3- sqrt(L_onestep*L_onestep+W_onestep*W_onestep)/4)*
//		(sqrt(L_onestep*L_onestep+W_onestep*W_onestep)*2*(s-T/2)/T/2*4/3- sqrt(L_onestep*L_onestep+W_onestep*W_onestep)/4)/((L_onestep*L_onestep+W_onestep*W_onestep)/16));
//		}
//		if(s>=7*T/8&&s<T)
//		{
//			x[0]=D*cos(alpha-delta)-B*cos(belta-delta/2*s/T*2)+L_onestep-L_onestep*2*s/T/2;
//			y[0]=D*sin(alpha-delta)-B*sin(belta-delta/2*s/T*2)+W_onestep-W_onestep*2*s/T/2;
//			z[0]=-L3;
//			
//			x[1]=D*cos(alpha-delta)-B*cos(belta-delta/2*s/T*2)+L_onestep-L_onestep*2*s/T/2;
//			y[1]=-D*sin(alpha-delta)+B*sin(belta-delta/2*s/T*2)+W_onestep-W_onestep*2*s/T/2;
//			z[1]=-L3;
//			
//			x[2]=-D*cos(alpha-delta)+B*cos(belta-delta/2*s/T*2)+L_onestep-L_onestep*2*s/T/2;
//			y[2]=D*sin(alpha-delta)-B*sin(belta-delta/2*s/T*2)+W_onestep-W_onestep*2*s/T/2;
//			z[2]=-L3;
//			
//			x[3]=-D*cos(alpha-delta)+B*cos(belta-delta/2*s/T*2)+L_onestep-L_onestep*2*s/T/2;
//			y[3]=-D*sin(alpha-delta)+B*sin(belta-delta/2*s/T*2)+W_onestep-W_onestep*2*s/T/2;
//			z[3]=-L3;
//		}
		
			a[0][0] = -asin(L3 / sqrt(z[0] * z[0] + y[0] * y[0]) ) - atan2(z[0],y[0]);
			a[0][1] = (asin((y[0] * y[0] + x[0] * x[0] + z[0] * z[0] + L1 * L1 - L2 * L2 - L3 * L3) / ( 2 * L1 * sqrt (y[0] * y[0] +  x[0] * x[0] + z[0] * z[0] - L3 * L3)) )
											- atan2(sqrt(y[0] * y[0] + z[0] * z[0] - L3 * L3) , x[0]));
			a[0][2] = asin((L1 * L1 + L2 * L2 + L3 * L3 - y[0] * y[0] - x[0] * x[0] - z[0] * z[0]) / (2 * L1 * L2));
	
			a[1][0] = asin(L3 / sqrt(y[1] * y[1] + z[1] * z[1]) ) + atan2(z[1],-y[1]);
			a[1][1] = -asin((y[1] * y[1] + x[1] * x[1] + z[1] * z[1] + L1 * L1 - L2 * L2 - L3 * L3) / ( 2 * L1 * sqrt (y[1] * y[1] +  x[1] * x[1] + z[1] * z[1] - L3 * L3)) )
											+ atan2(sqrt(y[1] * y[1] + z[1] * z[1] - L3 * L3) , x[1]);
			a[1][2] = -asin((L1 * L1 + L2 * L2 + L3 * L3 - y[1] * y[1] - x[1] * x[1] - z[1] * z[1]) / (2 * L1 * L2));
	
			a[2][0] = -asin(L3 / sqrt(y[2] * y[2] + z[2] * z[2]) ) - atan2(z[2],y[2]);
			a[2][1] = -(asin((y[2] * y[2] + x[2] * x[2] + z[2] * z[2] + L1 * L1 - L2 * L2 - L3 * L3) / ( 2 * L1 * sqrt (y[2] * y[2] +  x[2] * x[2] + z[2] * z[2] - L3 * L3)) 
											 - atan2(sqrt(y[2] * y[2] + z[2] * z[2] - L3 * L3) , -x[2])));
			a[2][2] = -asin((L1 * L1 + L2 * L2 + L3 * L3 - y[2] * y[2] - x[2] * x[2] - z[2] * z[2]) / (2 * L1 * L2));
	
			a[3][0] = asin(L3 / sqrt(y[3] * y[3] + z[3] * z[3]) ) + atan2(z[3],-y[3]);
			a[3][1] = asin((y[3] * y[3] + x[3] * x[3] + z[3] * z[3] + L1 * L1 - L2 * L2 - L3 * L3) / ( 2 * L1 * sqrt (y[3] * y[3] +  x[3] * x[3] + z[3] * z[3] - L3 * L3)) )
											- atan2(sqrt(y[3] * y[3] + z[3] * z[3] - L3 * L3) , -x[3]);
			a[3][2] = asin((L1 * L1 + L2 * L2 + L3 * L3 - y[3] * y[3] - x[3] * x[3] - z[3] * z[3]) / (2 * L1 * L2));
//				theta[0][0]=(a[0][0]+a[0][1]);
//				theta[0][1]=(a[0][1]-a[0][0]);
				
				a[0][0]=(int) (a[0][0]/3.1415926*180*100);//rf alpha 角，最内侧
				a[0][1]=(int) (a[0][1]/3.1415926*180*100);
				a[0][2]=(int) (a[0][2]/3.1415926*180*100);

				a[0][0]= (float) (-a[0][0]/100);
				a[0][1]= (float) (-a[0][1]/100);
				a[0][2]= (float) (a[0][2]/100);
//				
//				theta[1][0]=(a[1][0]+a[1][1]);
//				theta[1][1]=(a[1][1]-a[1][0]);
				
				a[1][0]=(int) (a[1][0]/3.1415926*180*100);//rf alpha 角，最内侧
				a[1][1]=(int) (a[1][1]/3.1415926*180*100);
				a[1][2]=(int) (a[1][2]/3.1415926*180*100);
			
				a[1][0]= (float) (-a[1][0]/100);
				a[1][1]= (float) (-a[1][1]/100);
				a[1][2]= (float) (a[1][2]/100);

//				theta[2][0]=(a[2][1]-a[2][0]);
//				theta[2][1]=(a[2][1]+a[2][0]);
				
				a[2][0]=(int) (a[2][0]/3.1415926*180*100);//rf alpha 角，最内侧
				a[2][1]=(int) (a[2][1]/3.1415926*180*100);
				a[2][2]=(int) (a[2][2]/3.1415926*180*100);

				a[2][0]= (float) (a[2][0]/100);
				a[2][1]= (float) (-a[2][1]/100);
				a[2][2]= (float) (a[2][2]/100);
				
//				theta[3][0]=(a[3][1]-a[3][0]);
//				theta[3][1]=(a[3][1]+a[3][0]);
				
				a[3][0]=(int) (a[3][0]/3.1415926*180*100);//rf alpha 角，最内侧
				a[3][1]=(int) (a[3][1]/3.1415926*180*100);
				a[3][2]=(int) (a[3][2]/3.1415926*180*100);

				a[3][0]= (float) (a[3][0]/100);
				a[3][1]= (float) (-a[3][1]/100);
				a[3][2]= (float) (a[3][2]/100);
			
				Angle(a[0][0]+KMGecko.StartAngle[LF_J3],LF_J3);
				Angle(a[0][1]+KMGecko.StartAngle[LF_J2],LF_J2);
				Angle(a[0][2]+KMGecko.StartAngle[LF_J1],LF_J1);
			
				Angle(a[1][0]+KMGecko.StartAngle[RF_J3],RF_J3);
				Angle(a[1][1]+KMGecko.StartAngle[RF_J2],RF_J2);
				Angle(a[1][2]+KMGecko.StartAngle[RF_J1],RF_J1);
					
				Angle(a[2][0]+KMGecko.StartAngle[LH_J3],LH_J3);
				Angle(a[2][1]+KMGecko.StartAngle[LH_J2],LH_J2);
				Angle(a[2][2]+KMGecko.StartAngle[LH_J1],LH_J1);
	
				Angle(a[3][0]+KMGecko.StartAngle[RH_J3],RH_J3);
				Angle(a[3][1]+KMGecko.StartAngle[RH_J2],RH_J2);
				Angle(a[3][2]+KMGecko.StartAngle[RH_J1],RH_J1);		
		if(s>=T)
		{
			s=0;
		}
		




}
void StartAngleInit1(void)//darius's robot
{
/**version1**/
	//关节1、2、3分别为踝关节、膝关节、髋关节
	//number 3
//	KMGecko.StartAngle[RF_J3] = -3; //52
//	KMGecko.StartAngle[RF_J2] = -6; //26//large figure turning to the inside
//	KMGecko.StartAngle[RF_J1] = 7; //// large figure turning to the inside
//   //number 2
//	KMGecko.StartAngle[RH_J3] = 5;//-10//-25;	
//	KMGecko.StartAngle[RH_J2] = -3;//50;	small figure clockwise
//	KMGecko.StartAngle[RH_J1] = -5;//30;small figure turning to the clw
//  //number 0
//	KMGecko.StartAngle[LF_J3] = 0;//-10
//	KMGecko.StartAngle[LF_J2] = 0;//-13 small figure turning to the inside
//	KMGecko.StartAngle[LF_J1] = -2;//-58 small figure turning to the inside
//	//number 1
//	KMGecko.StartAngle[LH_J3] = 2;//10///-30;//原值为-30
//	KMGecko.StartAngle[LH_J2] = 2;//0 large figure turning to the inside
//	KMGecko.StartAngle[LH_J1] = 0;//;small figure turning to the inside
	
	/*second*/
//	KMGecko.StartAngle[RF_J3] = 45-10; //52
//	KMGecko.StartAngle[RF_J2] = -45+10; //26//large figure turning to the inside
//	KMGecko.StartAngle[RF_J1] = 0; //// large figure turning to the inside
//   //number 2
//	KMGecko.StartAngle[RH_J3] = -45+12;	//-10//-25;	
//	KMGecko.StartAngle[RH_J2] = 45-12;	//50;	small figure clockwise
//	KMGecko.StartAngle[RH_J1] = 0 ;//30;small figure turning to the clw
//  //number 0
//	KMGecko.StartAngle[LF_J3] = -45+14;	//-10
//	KMGecko.StartAngle[LF_J2] = 45-14;	//-13 small figure turning to the inside
//	KMGecko.StartAngle[LF_J1] = 0;	//-58 small figure turning to the inside
//	//number 1
//	KMGecko.StartAngle[LH_J3] = 45-16;//10///-30;//原值为-30
//	KMGecko.StartAngle[LH_J2] = -45+16;//0 large figure turning to the inside
//	KMGecko.StartAngle[LH_J1] = 0;//;small figure turning to the inside		

		/*third*/
/*second hold*/
	KMGecko.StartAngle[RF_J3] = 45-100; //52
	KMGecko.StartAngle[RF_J2] = -45+88; //26//large figure turning to the inside
	KMGecko.StartAngle[RF_J1] = 20; //// large figure turning to the inside
   //number 2
	KMGecko.StartAngle[RH_J3] = -45+95;	//-10//-25;	
	KMGecko.StartAngle[RH_J2] = 45-95;	//50;	small figure clockwise
	KMGecko.StartAngle[RH_J1] = -30 ;//30;small figure turning to the clw
  //number 0
	KMGecko.StartAngle[LF_J3] = -45+90;	//-10
	KMGecko.StartAngle[LF_J2] = 45-90;	//-13 small figure turning to the inside
	KMGecko.StartAngle[LF_J1] = -35;	//-58 small figure turning to the inside
	//number 1
	KMGecko.StartAngle[LH_J3] = 45-98;//10///-30;//原值为-30
	KMGecko.StartAngle[LH_J2] = -45+107;//0 large figure turning to the inside
	KMGecko.StartAngle[LH_J1] = 30;//;small figure turning to the inside		
/*third hold*/
//	KMGecko.StartAngle[RF_J3] = 45-10-82; //52
//	KMGecko.StartAngle[RF_J2] = -45+10+82; //26//large figure turning to the inside
//	KMGecko.StartAngle[RF_J1] = 30; //// large figure turning to the inside
//   //number 2
//	KMGecko.StartAngle[RH_J3] = -45+16+85;	//-10//-25;	
//	KMGecko.StartAngle[RH_J2] = 45-16-80;	//50;	small figure clockwise
//	KMGecko.StartAngle[RH_J1] = -40 ;//30;small figure turning to the clw
//  //number 0
//	KMGecko.StartAngle[LF_J3] = -45+11+82;//-10
//	KMGecko.StartAngle[LF_J2] = 45-6-82;	//-13 small figure turning to the inside
//	KMGecko.StartAngle[LF_J1] = -30;	//-58 small figure turning to the inside
//	//number 1
//	KMGecko.StartAngle[LH_J3] = 45-10-85;//10///-30;//原值为-30
//	KMGecko.StartAngle[LH_J2] = -45+10+82;//0 large figure turning to the inside
//	KMGecko.StartAngle[LH_J1] = 35;//;small figure turning to the inside	

	
	/*航天三院*/
//	KMGecko.StartAngle[RF_J3] = 0; //52
//	KMGecko.StartAngle[RF_J2] = -30; //26//large figure turning to the inside
//	KMGecko.StartAngle[RF_J1] = -45; //// large figure turning to the inside
//   //number 2
//	KMGecko.StartAngle[RH_J3] = 0;//-10//-25;	
//	KMGecko.StartAngle[RH_J2] = 30;//50;	small figure clockwise
//	KMGecko.StartAngle[RH_J1] = 45;//30;small figure turning to the clw
//  //number 0
//	KMGecko.StartAngle[LF_J3] = 0;//-10
//	KMGecko.StartAngle[LF_J2] = 30;//-13 small figure turning to the inside
//	KMGecko.StartAngle[LF_J1] = 48;//-58 small figure turning to the inside
//	//number 1
//	KMGecko.StartAngle[LH_J3] = 0;//10///-30;//原值为-30
//	KMGecko.StartAngle[LH_J2] = -30;//0 large figure turning to the inside
//	KMGecko.StartAngle[LH_J1] = -45;//;small figure turning to the inside
}
void StartAngleInit2(void)//darius's robot
{
	
	float a[4][3] = {0};     //   a1 = a[0]   a2=a[LF]
	float theta[4][2] = {0};
	delta=delta*3.1415/180;
  float alpha;
	float belta;
	D = sqrt((x_b+F2)*(x_b+F2)+(y_b+F1)*(y_b+F1));
	B = sqrt(x_b*x_b+y_b*y_b);
	alpha = acos((x_b+F2)/D);
	belta = acos(x_b/B);
	float L_onestep1=1*L_onestep;

	for(stime=0 ; stime<Time ; stime=stime+0.02)
	{
	if(stime<Time/2&&stime>=0)
	{	
			x[0]=D*cos(alpha+delta*stime/Time*2)-B*cos(belta+delta*stime/Time*2)+L_onestep*2*stime/Time;
			y[0]=D*sin(alpha+delta*stime/Time*2)-B*sin(belta+delta*stime/Time*2)+W_onestep*2*stime/Time;
			z[0]=-L3+H_onestep-H_onestep*(sqrt(L_onestep*L_onestep+W_onestep*W_onestep)*4*stime/Time/2-sqrt(L_onestep*L_onestep+W_onestep*W_onestep)/2)*
			(sqrt(L_onestep*L_onestep+W_onestep*W_onestep)*4*stime/Time/2- sqrt(L_onestep*L_onestep+W_onestep*W_onestep)/2)/((L_onestep1*L_onestep+W_onestep*W_onestep)/4);
		
			x[1]=D*cos(alpha-delta*stime/Time*2)-B*cos(belta-delta*stime/Time*2)+L_onestep*stime/Time*2;
			y[1]=-D*sin(alpha-delta*stime/Time*2)+B*sin(belta-delta*stime/Time*2)+W_onestep*2*stime/Time;
//		z[1]=-L3+H_onestep-H_onestep*(sqrt(L_onestep*L_onestep+W_onestep*W_onestep)*4*s/T*3/4*7/8- sqrt(L_onestep*L_onestep+W_onestep*W_onestep)/8*3*7/8)*
//		(sqrt(L_onestep*L_onestep+W_onestep*W_onestep)*4*s/T*3/4*7/8- sqrt(L_onestep*L_onestep+W_onestep*W_onestep)/8*3*7/8)/((L_onestep*L_onestep+W_onestep*W_onestep)/8/8*3*3);
			z[1]=-L3+H_onestep-H_onestep*(sqrt(L_onestep*L_onestep+W_onestep*W_onestep)*4*stime/T/2- sqrt(L_onestep*L_onestep+W_onestep*W_onestep)/2)*
			(sqrt(L_onestep*L_onestep+W_onestep*W_onestep)*4*stime/T/2- sqrt(L_onestep*L_onestep+W_onestep*W_onestep)/2)/((L_onestep*L_onestep+W_onestep*W_onestep)/4);
		
		
			x[2]=-D*cos(alpha+delta*stime/Time*2)+B*cos(belta-delta*stime/Time*2)+L_onestep*4*stime/Time/2;
			y[2]=D*sin(alpha+delta*s/Time*2)-B*sin(belta-delta*stime/Time*2)+W_onestep*2*stime/Time;
			z[2]=-L3+H_onestep-H_onestep*(sqrt(L_onestep*L_onestep+W_onestep*W_onestep)*4*stime/Time/2-sqrt(L_onestep*L_onestep+W_onestep*W_onestep)/2)*
			(sqrt(L_onestep*L_onestep+W_onestep*W_onestep)*4*stime/Time/2- sqrt(L_onestep*L_onestep+W_onestep*W_onestep)/2)/((L_onestep*L_onestep+W_onestep*W_onestep)/4);
		
			x[3]=-D*cos(alpha+delta*stime/Time*2)+B*cos(belta+delta*stime/Time*2)+L_onestep*stime/Time*2;
			y[3]=-D*sin(alpha+delta*stime/Time*2)+B*sin(belta+delta*stime/Time*2)+W_onestep*stime/Time*2;
			z[3]=-L3+H_onestep-H_onestep*(sqrt(L_onestep*L_onestep+W_onestep*W_onestep)*stime/Time*2-sqrt(L_onestep*L_onestep+W_onestep*W_onestep)/2)*
			(sqrt(L_onestep*L_onestep+W_onestep*W_onestep)*stime/Time*2- sqrt(L_onestep*L_onestep+W_onestep*W_onestep)/2)/((L_onestep*L_onestep+W_onestep*W_onestep)/4);
		}
		if(stime>=T/2&&stime<T)
		{	
			x[0]=D*cos(alpha+delta)-B*cos(belta+delta*(stime-Time/2)/Time*2)+L_onestep-L_onestep*(stime-Time/2)/Time*2;
			y[0]=D*sin(alpha+delta)-B*sin(belta+delta*(stime-Time/2)/Time*2)+W_onestep-W_onestep*(stime-Time/2)/Time*2;
			z[0]=-L3;
		
			x[1]=D*cos(alpha-delta)-B*cos(belta-delta*(stime-Time/2)/Time*2)+L_onestep-L_onestep*(stime-Time/2)/Time*2;
			y[1]=-D*sin(alpha-delta)+B*sin(belta-delta*(stime-Time/2)/Time*2)+W_onestep-W_onestep*(stime-Time/2)/Time*2;
			z[1]=-L3;
	
			x[2]=-D*cos(alpha-delta)+B*cos(belta-delta*(stime-Time/2)/Time*2)+L_onestep-L_onestep*(stime-Time/2)/Time*2;
			y[2]=D*sin(alpha-delta)-B*sin(belta-delta*(stime-Time/2)/Time*2)+W_onestep-W_onestep*(stime-Time/2)/Time*2;
			z[2]=-L3;
			
			x[3]=-D*cos(alpha+delta)+B*cos(belta+delta*(stime-Time/2)/Time*2)+L_onestep-L_onestep*(stime-Time/2)/Time*2;
			y[3]=-D*sin(alpha+delta)+B*sin(belta+delta*(stime-Time/2)/Time*2)+W_onestep-W_onestep*(stime-Time/2)/Time*2;
			z[3]=-L3;			
		}
			a[0][0] = -asin(L3 / sqrt(z[0] * z[0] + y[0] * y[0]) ) - atan2(z[0],y[0]);
			a[0][1] = (asin((y[0] * y[0] + x[0] * x[0] + z[0] * z[0] + L1 * L1 - L2 * L2 - L3 * L3) / ( 2 * L1 * sqrt (y[0] * y[0] +  x[0] * x[0] + z[0] * z[0] - L3 * L3)) )
											- atan2(sqrt(y[0] * y[0] + z[0] * z[0] - L3 * L3) , x[0]));
			a[0][2] = asin((L1 * L1 + L2 * L2 + L3 * L3 - y[0] * y[0] - x[0] * x[0] - z[0] * z[0]) / (2 * L1 * L2));
	
			a[1][0] = asin(L3 / sqrt(y[1] * y[1] + z[1] * z[1]) ) + atan2(z[1],-y[1]);
			a[1][1] = -asin((y[1] * y[1] + x[1] * x[1] + z[1] * z[1] + L1 * L1 - L2 * L2 - L3 * L3) / ( 2 * L1 * sqrt (y[1] * y[1] +  x[1] * x[1] + z[1] * z[1] - L3 * L3)) )
											+ atan2(sqrt(y[1] * y[1] + z[1] * z[1] - L3 * L3) , x[1]);
			a[1][2] = -asin((L1 * L1 + L2 * L2 + L3 * L3 - y[1] * y[1] - x[1] * x[1] - z[1] * z[1]) / (2 * L1 * L2));
	
			a[2][0] = -asin(L3 / sqrt(y[2] * y[2] + z[2] * z[2]) ) - atan2(z[2],y[2]);
			a[2][1] = -(asin((y[2] * y[2] + x[2] * x[2] + z[2] * z[2] + L1 * L1 - L2 * L2 - L3 * L3) / ( 2 * L1 * sqrt (y[2] * y[2] +  x[2] * x[2] + z[2] * z[2] - L3 * L3)) 
											 - atan2(sqrt(y[2] * y[2] + z[2] * z[2] - L3 * L3) , -x[2])));
			a[2][2] = -asin((L1 * L1 + L2 * L2 + L3 * L3 - y[2] * y[2] - x[2] * x[2] - z[2] * z[2]) / (2 * L1 * L2));
	
			a[3][0] = asin(L3 / sqrt(y[3] * y[3] + z[3] * z[3]) ) + atan2(z[3],-y[3]);
			a[3][1] = asin((y[3] * y[3] + x[3] * x[3] + z[3] * z[3] + L1 * L1 - L2 * L2 - L3 * L3) / ( 2 * L1 * sqrt (y[3] * y[3] +  x[3] * x[3] + z[3] * z[3] - L3 * L3)) )
											- atan2(sqrt(y[3] * y[3] + z[3] * z[3] - L3 * L3) , -x[3]);
			a[3][2] = asin((L1 * L1 + L2 * L2 + L3 * L3 - y[3] * y[3] - x[3] * x[3] - z[3] * z[3]) / (2 * L1 * L2));
				theta[0][0]=(a[0][0]+a[0][1]);
				theta[0][1]=(a[0][1]-a[0][0]);
				
				theta[0][0]=(int) (theta[0][0]/3.1415926*180*100);//rf alpha 角，最内侧
				theta[0][1]=(int) (theta[0][1]/3.1415926*180*100);
				a[0][2]=(int) (a[0][2]/3.1415926*180*100);

				theta[0][0]= (float) (theta[0][0]/100);
				theta[0][1]= (float) (theta[0][1]/100);
				a[0][2]= (float) (a[0][2]/100);
				
				theta[1][0]=(a[1][0]+a[1][1]);
				theta[1][1]=(a[1][1]-a[1][0]);
				
				theta[1][0]=(int) (theta[1][0]/3.1415926*180*100);//rf alpha 角，最内侧
				theta[1][1]=(int) (theta[1][1]/3.1415926*180*100);
				a[1][2]=(int) (a[1][2]/3.1415926*180*100);
			
				theta[1][0]= (float) (theta[1][0]/100);
				theta[1][1]= (float) (theta[1][1]/100);
				a[1][2]= (float) (a[1][2]/100);

				theta[2][0]=(a[2][1]-a[2][0]);
				theta[2][1]=(a[2][1]+a[2][0]);
				
				theta[2][0]=(int) (theta[2][0]/3.1415926*180*100);//rf alpha 角，最内侧
				theta[2][1]=(int) (theta[2][1]/3.1415926*180*100);
				a[2][2]=(int) (a[2][2]/3.1415926*180*100);

				theta[2][0]= (float) (theta[2][0]/100);
				theta[2][1]= (float) (theta[2][1]/100);
				a[2][2]= (float) (a[2][2]/100);
				
				theta[3][0]=(a[3][1]-a[3][0]);
				theta[3][1]=(a[3][1]+a[3][0]);
				
				theta[3][0]=(int) (theta[3][0]/3.1415926*180*100);//rf alpha 角，最内侧
				theta[3][1]=(int) (theta[3][1]/3.1415926*180*100);
				a[3][2]=(int) (a[3][2]/3.1415926*180*100);

				theta[3][0]= (float) (theta[3][0]/100);
				theta[3][1]= (float) (theta[3][1]/100);
				a[3][2]= (float) (a[3][2]/100);
			
				Angle(theta[0][0]+KMGecko.StartAngle[LF_J3],LF_J3);
				Angle(theta[0][1]+KMGecko.StartAngle[LF_J2],LF_J2);
				Angle(a[0][2]+KMGecko.StartAngle[LF_J1],LF_J1);
			
				Angle(theta[1][0]+KMGecko.StartAngle[RF_J3],RF_J3);
				Angle(theta[1][1]+KMGecko.StartAngle[RF_J2],RF_J2);
				Angle(a[1][2]+KMGecko.StartAngle[RF_J1],RF_J1);
					
				Angle(theta[2][0]+KMGecko.StartAngle[LH_J3],LH_J3);
				Angle(theta[2][1]+KMGecko.StartAngle[LH_J2],LH_J2);
				Angle(a[2][2]+KMGecko.StartAngle[LH_J1],LH_J1);
	
				Angle(theta[3][0]+KMGecko.StartAngle[RH_J3],RH_J3);
				Angle(theta[3][1]+KMGecko.StartAngle[RH_J2],RH_J2);
				Angle(a[3][2]+KMGecko.StartAngle[RH_J1],RH_J1);		

		HAL_Delay(20);
	}	
}
void StartAngleInit3(void)//darius's robot
{
	KMGecko.StartAngle[RF_J3] = 45-10-82+4; //52
	KMGecko.StartAngle[RF_J2] = -45+10+82+4; //26//large figure turning to the inside
	KMGecko.StartAngle[RF_J1] = 30-4; //// large figure turning to the inside
   //number 2
	KMGecko.StartAngle[RH_J3] = -45+16+85+4;	//-10//-25;	
	KMGecko.StartAngle[RH_J2] = 45-16-80+4;	//50;	small figure clockwise
	KMGecko.StartAngle[RH_J1] = -40 ;//30;small figure turning to the clw
  //number 0
	KMGecko.StartAngle[LF_J3] = -45+11+82+4;//-10
	KMGecko.StartAngle[LF_J2] = 45-6-82+4;	//-13 small figure turning to the inside
	KMGecko.StartAngle[LF_J1] = -30;	//-58 small figure turning to the inside
	//number 1
	KMGecko.StartAngle[LH_J3] = 45-10-85+4;//10///-30;//原值为-30
	KMGecko.StartAngle[LH_J2] = -45+10+82+4;//0 large figure turning to the inside
	KMGecko.StartAngle[LH_J1] = 35;//;small figure turning to the inside	
}
void StartAngleInit4(void)//darius's robot
{
	KMGecko.StartAngle[RF_J3] = 45-10-82+4+5; //52
	KMGecko.StartAngle[RF_J2] = -45+10+82+4-5; //26//large figure turning to the inside
	KMGecko.StartAngle[RF_J1] = 30-4; //// large figure turning to the inside
   //number 2
	KMGecko.StartAngle[RH_J3] = -45+16+85+4;	//-10//-25;	
	KMGecko.StartAngle[RH_J2] = 45-16-80+4;	//50;	small figure clockwise
	KMGecko.StartAngle[RH_J1] = -40 ;//30;small figure turning to the clw
  //number 0
	KMGecko.StartAngle[LF_J3] = -45+11+82+4-5;//-10
	KMGecko.StartAngle[LF_J2] = 45-6-82+4+5;	//-13 small figure turning to the inside
	KMGecko.StartAngle[LF_J1] = -30;	//-58 small figure turning to the inside
	//number 1
	KMGecko.StartAngle[LH_J3] = 45-10-85+4;//10///-30;//原值为-30
	KMGecko.StartAngle[LH_J2] = -45+10+82+4;//0 large figure turning to the inside
	KMGecko.StartAngle[LH_J1] = 35;//;small figure turning to the inside	
}
void StartAngleInit5(void)//darius's robot
{
	KMGecko.StartAngle[RF_J3] = 45-10-82+4-5; //52
	KMGecko.StartAngle[RF_J2] = -45+10+82+4+5; //26//large figure turning to the inside
	KMGecko.StartAngle[RF_J1] = 30-4; //// large figure turning to the inside
   //number 2
	KMGecko.StartAngle[RH_J3] = -45+16+85+4;	//-10//-25;	
	KMGecko.StartAngle[RH_J2] = 45-16-80+4;	//50;	small figure clockwise
	KMGecko.StartAngle[RH_J1] = -30 ;//30;small figure turning to the clw
  //number 0
	KMGecko.StartAngle[LF_J3] = -45+11+82+4+5;//-10
	KMGecko.StartAngle[LF_J2] = 45-6-82+4-5;	//-13 small figure turning to the inside
	KMGecko.StartAngle[LF_J1] = -30;	//-58 small figure turning to the inside
	//number 1
	KMGecko.StartAngle[LH_J3] = 45-10-85+4;//10///-30;//原值为-30
	KMGecko.StartAngle[LH_J2] = -45+10+82+4;//0 large figure turning to the inside
	KMGecko.StartAngle[LH_J1] = 30;//;small figure turning to the inside	
}
void InitRobotPosion(void)
{
//	if(((Time8Channel3HighTime<=1600)&&(Time8Channel3HighTime>=1400))||(Time8Channel3HighTime<=500))
//	{	
	Angle(KMGecko.StartAngle[LF_J3],LF_J3);
	Angle(KMGecko.StartAngle[LF_J1],LF_J1);
	Angle(KMGecko.StartAngle[LF_J2],LF_J2);
	
	Angle(KMGecko.StartAngle[RF_J1],RF_J1);
	Angle(KMGecko.StartAngle[RF_J2],RF_J2);		
	Angle(KMGecko.StartAngle[RF_J3],RF_J3);
	
	Angle(KMGecko.StartAngle[RH_J1],RH_J1);
	Angle(KMGecko.StartAngle[RH_J2],RH_J2);
	Angle(KMGecko.StartAngle[RH_J3],RH_J3);

	Angle(KMGecko.StartAngle[LH_J1],LH_J1);
	Angle(KMGecko.StartAngle[LH_J2],LH_J2);
	Angle(KMGecko.StartAngle[LH_J3],LH_J3);

//	setcurrentposition(RF,L1,L2,-L3);
//	setcurrentposition(RH,L1,L2,-L3);
//	setcurrentposition(LF,L1,L2,-L3);
//	setcurrentposition(LH,L1,L2,-L3);
//	HAL_Delay(30);	
}
void Angle(float angle,int8_t footnumber)
{
	int16_t ArrValue = 0;
	//ArrValue = 250 + (int32_t) (angle *0.18);
	//ArrValue = angle * 5 + 750;
	ArrValue = (int32_t) (angle * 5.56 + 750)	;
	if(footnumber == LF_J1)
	{
	//	User_PWM_SetValue(&htim4, TIM_CHANNEL_2,ArrValue);
		TIM_SetCompare1(&TIM4_Handler,ArrValue);
	}	
	else if(footnumber == LF_J2)
	{
	//	User_PWM_SetValue(&htim4, TIM_CHANNEL_1,ArrValue);
		TIM_SetCompare2(&TIM4_Handler,ArrValue);
	}	
	else if(footnumber == LF_J3)
	{
	//	User_PWM_SetValue(&htim4, TIM_CHANNEL_1,ArrValue);
		TIM_SetCompare3(&TIM4_Handler,ArrValue);
	}	
	else if(footnumber == LH_J1)
	{
	//	User_PWM_SetValue(&htim4, TIM_CHANNEL_1,ArrValue);
		TIM_SetCompare1(&TIM12_Handler,ArrValue);
	}	
	else if(footnumber == LH_J2)
	{
	//	User_PWM_SetValue(&htim4, TIM_CHANNEL_1,ArrValue);
		TIM_SetCompare2(&TIM12_Handler,ArrValue);
	}	
	else if(footnumber == LH_J3)
	{
	//	User_PWM_SetValue(&htim4, TIM_CHANNEL_1,ArrValue);
		TIM_SetCompare1(&TIM15_Handler,ArrValue);
	}	
	else if(footnumber == RF_J1)
	{
	//	User_PWM_SetValue(&htim3, TIM_CHANNEL_3,ArrValue);
		TIM_SetCompare1(&TIM1_Handler,ArrValue);
	}
	else if(footnumber == RF_J2)
	{
	//	User_PWM_SetValue(&htim3, TIM_CHANNEL_2,ArrValue);
		TIM_SetCompare2(&TIM1_Handler,ArrValue);
	}
	else if(footnumber == RF_J3)
	{
	//	User_PWM_SetValue(&htim1, TIM_CHANNEL_2,ArrValue);
		TIM_SetCompare3(&TIM1_Handler,ArrValue);
	}
	else if(footnumber == RH_J1)
	{
//		User_PWM_SetValue(&htim1, TIM_CHANNEL_3,ArrValue);
		TIM_SetCompare1(&TIM3_Handler,ArrValue);
	}
	else if(footnumber == RH_J2)
	{
//		User_PWM_SetValue(&htim1, TIM_CHANNEL_3,ArrValue);
		TIM_SetCompare2(&TIM3_Handler,ArrValue);
	}
	else if(footnumber == RH_J3)
	{
//		User_PWM_SetValue(&htim1, TIM_CHANNEL_3,ArrValue);
		TIM_SetCompare3(&TIM3_Handler,ArrValue);
	}
}