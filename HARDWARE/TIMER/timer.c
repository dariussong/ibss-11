#include "timer.h"
#include "uart.h"
#include "adc.h"
#include "SV.h"
#include "gait.h"

//////////////////////////////////////////////////////////////////////////////////	 
//±¾³ÌĞòÖ»¹©Ñ§Ï°Ê¹ÓÃ£¬Î´¾­×÷ÕßĞí¿É£¬²»µÃÓÃÓÚÆäËüÈÎºÎÓÃÍ¾
//ALIENTEK STM32H7¿ª·¢°å
//¶¨Ê±Æ÷ÖĞ¶ÏÇı¶¯´úÂë	   
//ÕıµãÔ­×Ó@ALIENTEK
//¼¼ÊõÂÛÌ³:www.openedv.com
//´´½¨ÈÕÆÚ:2017/8/12
//°æ±¾£ºV1.0
//°æÈ¨ËùÓĞ£¬µÁ°æ±Ø¾¿¡£
//Copyright(C) ¹ãÖİÊĞĞÇÒíµç×Ó¿Æ¼¼ÓĞÏŞ¹«Ë¾ 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	

TIM_HandleTypeDef TIM5_Handler;      //¶¨Ê±Æ÷¾ä±ú 
TIM_HandleTypeDef TIM2_Handler;      //¶¨Ê±Æ÷¾ä±ú 
TIM_HandleTypeDef TIM7_Handler;      //¶¨Ê±Æ÷¾ä±ú 
TIM_OC_InitTypeDef TIM2_CH2Handler;
u8 mesg[4]={0X03,0X02,0XD0,0X12};	
u8 canbuf1[8]={0};
u8 canbuf2[8]={0};
u8 canbuf3[4]={0};


void TIM7_Init(u16 arr,u16 psc)
{  
    TIM7_Handler.Instance=TIM7;                          //Í¨ÓÃ¶¨Ê±Æ÷3
    TIM7_Handler.Init.Prescaler=psc;                     //·ÖÆµ
    TIM7_Handler.Init.CounterMode=TIM_COUNTERMODE_UP;    //ÏòÉÏ¼ÆÊıÆ÷
    TIM7_Handler.Init.Period=arr;                        //×Ô¶¯×°ÔØÖµ
    TIM7_Handler.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;//Ê±ÖÓ·ÖÆµÒò×Ó
   
		HAL_TIM_Base_Init(&TIM7_Handler);    
    HAL_TIM_Base_Start_IT(&TIM7_Handler); //Ê¹ÄÜ¶¨Ê±Æ÷3ºÍ¶¨Ê±Æ÷3¸üĞÂÖĞ¶Ï£ºTIM_IT_UPDATE    
}

void TIM5_Init(u16 arr,u16 psc)
{  
    TIM5_Handler.Instance=TIM5;                          //Í¨ÓÃ¶¨Ê±Æ÷3
    TIM5_Handler.Init.Prescaler=psc;                     //·ÖÆµ
    TIM5_Handler.Init.CounterMode=TIM_COUNTERMODE_UP;    //ÏòÉÏ¼ÆÊıÆ÷
    TIM5_Handler.Init.Period=arr;                        //×Ô¶¯×°ÔØÖµ
    TIM5_Handler.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;//Ê±ÖÓ·ÖÆµÒò×Ó
   
		HAL_TIM_Base_Init(&TIM5_Handler);    
    HAL_TIM_Base_Start_IT(&TIM5_Handler); //Ê¹ÄÜ¶¨Ê±Æ÷3ºÍ¶¨Ê±Æ÷3¸üĞÂÖĞ¶Ï£ºTIM_IT_UPDATE    
}

void TIM2_Init(u16 arr,u16 psc)
{  
    TIM2_Handler.Instance=TIM2;                          //Í¨ÓÃ¶¨Ê±Æ÷3
    TIM2_Handler.Init.Prescaler=psc;                     //·ÖÆµ
    TIM2_Handler.Init.CounterMode=TIM_COUNTERMODE_UP;    //ÏòÉÏ¼ÆÊıÆ÷
    TIM2_Handler.Init.Period=arr;                        //×Ô¶¯×°ÔØÖµ
    TIM2_Handler.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;//Ê±ÖÓ·ÖÆµÒò×Ó
   
		HAL_TIM_Base_Init(&TIM2_Handler);    
    HAL_TIM_Base_Start_IT(&TIM2_Handler); //Ê¹ÄÜ¶¨Ê±Æ÷6ºÍ¶¨Ê±Æ÷6¸üĞÂÖĞ¶Ï£ºTIM_IT_UPDATE  
  
		HAL_TIM_PWM_ConfigChannel(&TIM2_Handler,&TIM2_CH2Handler,TIM_CHANNEL_2);//ÅäÖÃTIM1Í¨µÀ2
    HAL_TIM_PWM_Start(&TIM2_Handler,TIM_CHANNEL_2);//¿ªÆôPWMÍ¨µÀ2
}


void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{
    if(htim->Instance==TIM5)
	{
		__HAL_RCC_TIM5_CLK_ENABLE();            //Ê¹ÄÜTIM5Ê±ÖÓ

		HAL_NVIC_SetPriority(TIM5_IRQn,1,3);    //ÉèÖÃÖĞ¶ÏÓÅÏÈ¼¶£¬ÇÀÕ¼ÓÅÏÈ¼¶1£¬×ÓÓÅÏÈ¼¶3
		HAL_NVIC_EnableIRQ(TIM5_IRQn);          //¿ªÆôITM3ÖĞ¶Ï   
	}  
	   if(htim->Instance==TIM2)
	{
		__HAL_RCC_TIM2_CLK_ENABLE();            //Ê¹ÄÜTIM5Ê±ÖÓ

		HAL_NVIC_SetPriority(TIM2_IRQn,1,3);    //ÉèÖÃÖĞ¶ÏÓÅÏÈ¼¶£¬ÇÀÕ¼ÓÅÏÈ¼¶1£¬×ÓÓÅÏÈ¼¶3
		HAL_NVIC_EnableIRQ(TIM2_IRQn);          //¿ªÆôITM3ÖĞ¶Ï   
	} 
		if(htim->Instance==TIM7)
	{
		__HAL_RCC_TIM7_CLK_ENABLE();            //Ê¹ÄÜTIM5Ê±ÖÓ

		HAL_NVIC_SetPriority(TIM7_IRQn,1,3);    //ÉèÖÃÖĞ¶ÏÓÅÏÈ¼¶£¬ÇÀÕ¼ÓÅÏÈ¼¶1£¬×ÓÓÅÏÈ¼¶3
		HAL_NVIC_EnableIRQ(TIM7_IRQn);          //¿  
	} 	
}

//¶¨Ê±Æ÷5ÖĞ¶Ï·şÎñº¯Êı
void TIM7_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&TIM7_Handler);
}

void TIM5_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&TIM5_Handler);
}

void TIM2_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&TIM2_Handler);
}

//¶¨Ê±Æ÷5ÖĞ¶Ï·şÎñº¯Êıµ÷ÓÃ
float s=-0.8;
float T=3;
float delta_t=0.02;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim==(&TIM5_Handler))
    {
			//Read_All_Ad();	
			//air_control_trot();
			//movement_trot();	
			movement_tripod_br();				
			s=s+delta_t;
		}
		if(htim==(&TIM2_Handler))
    {			
			Read_All_Ad();			
			battery_indicatior();
		}
		if(htim==(&TIM7_Handler))
    {	
			
		}

		
		

			
		
}