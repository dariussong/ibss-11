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
	
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_12,GPIO_PIN_RESET);
	StartAngleInit();
	InitRobotPosion();
	HAL_Delay(500);
	SV_test1();
	SV_test2();
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_RESET);	//rf0和别的是反的		
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_RESET);	//rh0		
//	reverse2(0,-10,0,20);
	TIM5_Init(10000-1,200-1);
//	TIM2_Init(10000-1,200-1);
	while(1)
	 {
//		Showadc1();		
//		HAL_UART_Transmit(&huart7,&c,sizeof(c),1000);
//		TIM_SetCompare1(&TIM1_Handler,750);//1/8-1/125|5/200-25/200(1/40-1/8)
//		HAL_Delay(1000);
//		TIM_SetCompare1(&TIM1_Handler,1150);//1/8-1/125|5/200-25/200(1/40-1/8)
//		HAL_Delay(1000);
//		 Read_All_Ad(); 
//		 sv_flag[3]=1;
//		 SV_ESTIMATE();
//		 HAL_UART_Transmit(&huart7, &adc_1_H,sizeof(adc_1_H),0xFFFF);		
//		 HAL_UART_Transmit(&huart7, &adc_1_L,sizeof(adc_1_H),0xFFFF);
//		 HAL_UART_Transmit(&huart7, &len,1,0xFFFF);
////		TIM_SetCompare1(&TIM16_Handler,10);
//		HAL_Delay(20);
//		TIM_SetCompare1(&TIM16_Handler,190);
//		HAL_Delay(1000); 
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
//		Read_All_Ad();
//		HAL_Delay(50);
//    Robot_test3();
//		HAL_Delay(10);
//		TIM_SetCompare1(&TIM1_Handler,15);
//		robotTripodGait(); // simple tripod gait, the trajectory is stright line
	  }
	}


