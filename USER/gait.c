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
const short L2=47;
const short L3=22;
#define DE 1
KinematicsArm KMGecko;
int i;
float F1=38;
float F2=47;
float F3=22;


float x[4]={0};
float y[4]={0};
float z[4]={0};
float x_b=62;
float y_b=30.24;
float D;
float B;	

float D_onestep;
float L_onestep=25;
float W_onestep=0;
float delta=0;
float H_onestep =15;       
	
float cx[4]={0};
float cy[4]={0};
float cz[4]={0};
float s_time;
float delta;
void movement(void)
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
	
	
//	if(s<=0.02)
//	{
//		L_onestep=onestep;
//	}
	if(s<3*T/2/4&&s>=0)
	{	
		
//		x[0]=47-L_onestep/4*4*s/T;
//		y[0]=38;
//		z[0]=-22;
//		x[1]=47+L_onestep*4*s/T*3/4;
//		y[1]=-38;
//		z[1]=-22+H_onestep-H_onestep*(L_onestep*4*s/T*3/4- L_onestep/8*3)* (L_onestep*4*s/T*3/4- L_onestep/8*3)/(L_onestep/8*3*L_onestep/8*3);
//		x[2]=-47+L_onestep/4*4*s/T;
//		y[2]=38;
//		z[2]=-22;
//		x[3]=-47+L_onestep/4*4*s/T;
//		y[3]=-38;
//		z[3]=-22;
		
		x[0]=-B*cos(belta+delta/2*s/T*2)-L_onestep/2*2*s/T+D*cos(alpha);
		y[0]=-B*sin(belta+delta/2*s/T*2)-W_onestep/2*2*s/T+D*sin(alpha);
		z[0]=-22;		
		x[1]=D*cos(alpha-delta*s/T*2*4/3)-B*cos(belta-delta/2*s/T*2)+L_onestep*s/T*2/2*8/3-L_onestep*2*s/T/2;
		y[1]=-D*sin(alpha-delta*s/T*2*4/3)+B*sin(belta-delta/2*s/T*2)+W_onestep*2*s/T/2*8/3-W_onestep*2*s/T/2;
		z[1]=-22+H_onestep-H_onestep*(sqrt(L_onestep*L_onestep+W_onestep*W_onestep)*2*s/T/2*4/3- sqrt(L_onestep*L_onestep+W_onestep*W_onestep)/4)*
			(sqrt(L_onestep*L_onestep+W_onestep*W_onestep)*2*s/T/2*4/3- sqrt(L_onestep*L_onestep+W_onestep*W_onestep)/4)/((L_onestep*L_onestep+W_onestep*W_onestep)/16);
			
		x[2]= -D*cos(alpha-delta*s/T*2*4/3)+B*cos(belta-delta/2*s/T*2)+L_onestep*s/T*2/2*8/3-L_onestep*2*s/T/2;
		y[2]= D*sin(alpha-delta*s/T*2*4/3)-B*sin(belta-delta/2*s/T*2)+W_onestep*2*s/T/2*8/3-W_onestep*2*s/T/2;
		z[2]= -22+H_onestep-H_onestep*(sqrt(L_onestep*L_onestep+W_onestep*W_onestep)*2*s/T/2*4/3- sqrt(L_onestep*L_onestep+W_onestep*W_onestep)/4)*
			(sqrt(L_onestep*L_onestep+W_onestep*W_onestep)*2*s/T/2*4/3- sqrt(L_onestep*L_onestep+W_onestep*W_onestep)/4)/((L_onestep*L_onestep+W_onestep*W_onestep)/16);
		
		
		x[3]=B*cos(belta+delta/2*s/T*2)-L_onestep/2*2*s/T-D*cos(alpha);
		y[3]=B*sin(belta+delta/2*s/T*2)-W_onestep/2*2*s/T-D*sin(alpha);
		z[3]=-22;
		}
	if(s>=3*T/2/4&&s<T/2)
	{
		x[0]=-B*cos(belta+delta/2*s/T*2)-L_onestep/2*2*s/T+D*cos(alpha);
		y[0]=-B*sin(belta+delta/2*s/T*2)-W_onestep/2*2*s/T+D*sin(alpha);
		z[0]=-22;
		x[1]=D*cos(alpha-delta)-B*cos(belta-delta/2*s/T*2)+L_onestep-L_onestep*2*s/T/2;
		y[1]=-D*sin(alpha-delta)+B*sin(belta-delta/2*s/T*2)+W_onestep-W_onestep*2*s/T/2;
		z[1]=-22;
		
		x[2]=-D*cos(alpha-delta)+B*cos(belta-delta/2*s/T*2)+L_onestep-L_onestep*2*s/T/2;
		y[2]=D*sin(alpha-delta)-B*sin(belta-delta/2*s/T*2)+W_onestep-W_onestep*2*s/T/2;
		z[2]=-22;		
		x[3]=B*cos(belta+delta/2*s/T*2)-L_onestep/2*2*s/T-D*cos(alpha);
		y[3]=B*sin(belta+delta/2*s/T*2)-W_onestep/2*2*s/T-D*sin(alpha);
		z[3]=-22;
	}
		if(s>=T/2&&s<7*T/8)
		{
		x[0]=D*cos(alpha+delta*(s-T/2)/T*2*4/3)-B*cos(belta+delta/2*s/T*2*4/3)+L_onestep*2*(s-T/2)/T*4/3-L_onestep*2*s/T/2;
		y[0]=D*sin(alpha+delta*(s-T/2)/T*2*4/3)-B*sin(belta+delta/2*s/T*2*4/3)+W_onestep*2*(s-T/2)/T*4/3-W_onestep*2*s/T/2;
		z[0]= -22+H_onestep-H_onestep*(sqrt(L_onestep*L_onestep+W_onestep*W_onestep)*2*(s-T/2)/T/2*4/3- sqrt(L_onestep*L_onestep+W_onestep*W_onestep)/4)*
		(sqrt(L_onestep*L_onestep+W_onestep*W_onestep)*2*(s-T/2)/T/2*4/3- sqrt(L_onestep*L_onestep+W_onestep*W_onestep)/4)/((L_onestep*L_onestep+W_onestep*W_onestep)/16);

			x[1]=D*cos(alpha-delta)-B*cos(belta-delta/2*s/T*2)+L_onestep-L_onestep*2*s/T/2;
			y[1]=-D*sin(alpha-delta)+B*sin(belta-delta/2*s/T*2)+W_onestep-W_onestep*2*s/T/2;
			z[1]=-22;
			
			x[2]=-D*cos(alpha-delta)+B*cos(belta-delta/2*s/T*2)+L_onestep-L_onestep*2*s/T/2;
			y[2]=D*sin(alpha-delta)-B*sin(belta-delta/2*s/T*2)+W_onestep-W_onestep*2*s/T/2;
			z[2]=-22;
			
			x[3]=-D*cos(alpha+delta*(s-T/2)/T*2*4/3)+B*cos(belta+delta/2*s/T*2)+L_onestep*2*(s-T/2)/T*4/3-L_onestep*2*s/T/2;
			y[3]=-D*sin(alpha+delta*(s-T/2)/T*2*4/3)+B*sin(belta+delta/2*s/T*2)+W_onestep*2*(s-T/2)/T*4/3-W_onestep*2*s/T/2;
			z[3]= -22+H_onestep-H_onestep*(sqrt(L_onestep*L_onestep+W_onestep*W_onestep)*2*(s-T/2)/T/2*4/3- sqrt(L_onestep*L_onestep+W_onestep*W_onestep)/4)*
		(sqrt(L_onestep*L_onestep+W_onestep*W_onestep)*2*(s-T/2)/T/2*4/3- sqrt(L_onestep*L_onestep+W_onestep*W_onestep)/4)/((L_onestep*L_onestep+W_onestep*W_onestep)/16);
		}
		if(s>=7*T/8&&s<T)
		{
			x[0]=D*cos(alpha-delta)-B*cos(belta-delta/2*s/T*2)+L_onestep-L_onestep*2*s/T/2;
			y[0]=D*sin(alpha-delta)-B*sin(belta-delta/2*s/T*2)+W_onestep-W_onestep*2*s/T/2;
			z[0]=-22;
			x[1]=D*cos(alpha-delta)-B*cos(belta-delta/2*s/T*2)+L_onestep-L_onestep*2*s/T/2;
			y[1]=-D*sin(alpha-delta)+B*sin(belta-delta/2*s/T*2)+W_onestep-W_onestep*2*s/T/2;
			z[1]=-22;
			
			x[2]=-D*cos(alpha-delta)+B*cos(belta-delta/2*s/T*2)+L_onestep-L_onestep*2*s/T/2;
			y[2]=D*sin(alpha-delta)-B*sin(belta-delta/2*s/T*2)+W_onestep-W_onestep*2*s/T/2;
			z[2]=-22;
			x[3]=-D*cos(alpha-delta)+B*cos(belta-delta/2*s/T*2)+L_onestep-L_onestep*2*s/T/2;
			y[3]=-D*sin(alpha-delta)+B*sin(belta-delta/2*s/T*2)+W_onestep-W_onestep*2*s/T/2;
			z[3]=-22;
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


void StartAngleInit(void)
{
			/**直角初始位置**/
	//关节1、2、3分别为踝关节、膝关节、髋关节
	//number 3
	KMGecko.StartAngle[RF_J3] = -3; //52
	KMGecko.StartAngle[RF_J2] = 3; //26//large figure turning to the inside
	KMGecko.StartAngle[RF_J1] = 2; //// large figure turning to the inside
   //number 2
	KMGecko.StartAngle[RH_J3] = 5;//-10//-25;	
	KMGecko.StartAngle[RH_J2] = -3;//50;	small figure clockwise
	KMGecko.StartAngle[RH_J1] = -5 ;//30;small figure turning to the clw
  //number 0
	KMGecko.StartAngle[LF_J3] = -2;//-10
	KMGecko.StartAngle[LF_J2] = -8;//-13 small figure turning to the inside
	KMGecko.StartAngle[LF_J1] = -2;//-58 small figure turning to the inside
	//number 1
	KMGecko.StartAngle[LH_J3] = 5;//10///-30;//原值为-30
	KMGecko.StartAngle[LH_J2] = 7;//0 large figure turning to the inside
	KMGecko.StartAngle[LH_J1] = 0;//;small figure turning to the inside
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