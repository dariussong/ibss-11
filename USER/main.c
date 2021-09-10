#include "sys.h"
#include <stdlib.h>
#include "string.h"
#include "delay.h"
#include "uart.h" 
#include "pwm.h"
#include "timer.h"
#include "adc.h"
#include "can.h"
#include "gait.h"
#include "SV.h"

int main(void)
{	
	u8 len=0;		
	Cache_Enable();                		//打开L1-Cache	
	HAL_Init();				        	//初始化HAL库
	Stm32_Clock_Init(160,5,2,4);  	    //设置时钟,400Mhz 
	delay_init(400);		//延时初始化
	FDCAN1_Mode_Init(5,8,31,8,FDCAN_MODE_NORMAL);  //普通测试
	HAL_Delay(100);
	
	MY_UART7_Init();
	SV_Init();//电磁阀与泵初始化
	pid_value_init();//泵的PID
	TIM1_PWM_Init(10000-1,200-1);//4分频 100M/200=500k的计数频率，ARR自动重装载为10000，PSC那么PWM频率为500k/10k=50HZ
	TIM3_PWM_Init(10000-1,200-1); 
	TIM4_PWM_Init(10000-1,200-1);
	TIM12_PWM_Init(10000-1,200-1); 
	TIM15_PWM_Init(10000-1,200-1); 	
	TIM16_PWM_Init(200-1,10000-1);//pump
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_SET);
	MX_ADC1_Init();
	TIM2_Init(10000-1,2000-1);
	
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_12,GPIO_PIN_RESET);
	StartAngleInit1();
	InitRobotPosion();
	HAL_Delay(500);
	StartAngleInit2();
	SV_test1();
	HAL_Delay(200);
	SV_test2();
//	TIM5_Init(10000-1,200-1);
//HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_RESET);
//HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,GPIO_PIN_RESET);
	while(1)
	{	

////		HAL_UART_Transmit(&huart7,&c,sizeof(c),1000);
//		TIM_SetCompare1(&TIM16_Handler,150);//1/8-1/125|5/200-25/200(1/40-1/8)
////		HAL_Delay(1000);
//		TIM_SetCompare1(&TIM1_Handler,450);//1/8-1/125|5/200-25/200(1/40-1/8)
//		HAL_Delay(300);
//		TIM_SetCompare1(&TIM1_Handler,750);//1/8-1/125|5/200-25/200(1/40-1/8)
//		HAL_Delay(150);
//		 float t;
//		 t=0;
//		 while(t<100)
//		 {
//			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_RESET);	
//			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_RESET);
//			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_SET);	
//			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,GPIO_PIN_SET);			 
//		 Read_All_Ad(); 
//		 sv_flag[3]=1;
//		 SV_ESTIMATE();
//		 HAL_UART_Transmit(&huart7, &adc_1_H,sizeof(adc_1_H),0xFFFF);		
//		 HAL_UART_Transmit(&huart7, &adc_1_L,sizeof(adc_1_H),0xFFFF);
//		 HAL_UART_Transmit(&huart7, &len,1,0xFFFF);
//		HAL_Delay(20);
//		 t+=1;
//		 }
//			 HAL_Delay(1000);
//		 while(t>=100&&t<200)
//		 {
//			 HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_SET);	
//			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_SET);
//			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_RESET);	
//			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,GPIO_PIN_RESET);	
//			 TIM_SetCompare1(&TIM16_Handler,199);
//			HAL_Delay(20);
//			t+=1;
//		 }
//		 	TIM_SetCompare2(&TIM1_Handler,620);
//		 TIM_SetCompare1(&TIM1_Handler,750);
//		 HAL_Delay(200);
//				TIM_SetCompare1(&TIM1_Handler,1150);
//				HAL_Delay(200); 		 
//		TIM_SetCompare1(&TIM16_Handler,190);
//		HAL_Delay(2000); 
//		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_RESET);	//rf0和别的是反的
//		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_RESET);	//rf1 
//	  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_RESET);	//rh0
//		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,GPIO_PIN_RESET);	//rh1 
//	  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_RESET);	//lf0
//		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,GPIO_PIN_RESET);	//lf1 
//	  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_RESET);	//lh0
//		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_RESET);	//lh1 
//		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_RESET);	//all0
//		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_RESET);	//all1	
//		HAL_Delay(2000);
//		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_SET);	//PB0置0
//		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_SET);	//PB1置1 
//	  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_SET);	//PB0置0
//		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,GPIO_PIN_SET);	//PB1置1 
//	  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_SET);	//PB0置0
//		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,GPIO_PIN_SET);	//PB1置1 
//	  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_SET);	//PB0置0
//		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_SET);	//PB1置1 
//		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_SET);	//PB0置0
//		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_SET);	//PB1置1 		 
//		HAL_Delay(2000);
//		TIM_SetCompare1(&TIM1_Handler,15);
	}
}
