#include "uart.h"
#include "timer.h"
#include "gait.h"

#if SYSTEM_SUPPORT_OS
#include "includes.h"					//os ʹ��	  
#endif

u8 onestep=0;

#if EN_UART7_RX   //���ʹ���˽���
//����1�жϷ������
//ע��,��ȡUSARTx->SR�ܱ���Ī������Ĵ���   	
u8 UART_RX_BUF[UART_REC_LEN];     //���ջ���,���USART_REC_LEN���ֽ�.
//����״̬
//bit15��	������ɱ�־
//bit14��	���յ�0x0d
//bit13~0��	���յ�����Ч�ֽ���Ŀ
u16 UART_RX_STA=0;       //����״̬���	
u8 res;
u8 aRxBuffer[RXBUFFERSIZE];//HAL��ʹ�õĴ��ڽ��ջ���
UART_HandleTypeDef huart7; //UART���
u8 rdata[RDATASIZE];
u8 receive[receivedata];
void MY_UART7_Init(void)
{	
	//UART ��ʼ������
	huart7.Instance=UART7;					    //USART1
	huart7.Init.BaudRate=9600;				    //������
	huart7.Init.WordLength=UART_WORDLENGTH_8B;   //�ֳ�Ϊ8λ���ݸ�ʽ
	huart7.Init.StopBits=UART_STOPBITS_1;	    //һ��ֹͣλ
	huart7.Init.Parity=UART_PARITY_NONE;		    //����żУ��λ
	huart7.Init.HwFlowCtl=UART_HWCONTROL_NONE;   //��Ӳ������
	huart7.Init.Mode=UART_MODE_TX_RX;		    //�շ�ģʽ
	HAL_UART_Init(&huart7);					    //HAL_UART_Init()��ʹ��UART1
	
	HAL_UART_Receive_IT(&huart7,(u8 *)aRxBuffer, RXBUFFERSIZE);//�ú����Ὺ�������жϣ���־λUART_IT_RXNE���������ý��ջ����Լ����ջ���������������
  
}

//UART�ײ��ʼ����ʱ��ʹ�ܣ��������ã��ж�����
//�˺����ᱻHAL_UART_Init()����
//huart:���ھ��
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
    //GPIO�˿�����
	GPIO_InitTypeDef GPIO_Initure;
	
	if(huart->Instance==UART7)//����Ǵ���1�����д���1 MSP��ʼ��
	{
		__HAL_RCC_GPIOB_CLK_ENABLE();			//ʹ��GPIOAʱ��
		__HAL_RCC_UART7_CLK_ENABLE();			//ʹ��USART1ʱ��
	
		GPIO_Initure.Pin=GPIO_PIN_3|GPIO_PIN_4;			//PA9
		GPIO_Initure.Mode=GPIO_MODE_AF_PP;		//�����������
		GPIO_Initure.Pull=GPIO_PULLUP;			//����
		GPIO_Initure.Speed=GPIO_SPEED_FREQ_HIGH;//����
		GPIO_Initure.Alternate=GPIO_AF11_UART7;	//����ΪUSART1
		HAL_GPIO_Init(GPIOB,&GPIO_Initure);	   	//��ʼ��PA9
		
#if EN_UART7_RX
					//ʹ��USART1�ж�ͨ��
		HAL_NVIC_SetPriority(UART7_IRQn,3,3);			//��ռ���ȼ�3�������ȼ�3
		HAL_NVIC_EnableIRQ(UART7_IRQn);	
#endif
	}
}

u8 res = 0;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	u8 rec;
 
	if(huart->Instance==UART7)//����Ǵ���1
	{				
		rec=*(huart->pRxBuffPtr-1);
		
		if(rec!=0x0D && rec!=0x0A)
		{
			HAL_UART_Transmit(&huart7,&rec,1,0xFFFF);
			receive[res]=rec;
			res++;
		}
		else if(rec==0x0D)
		{
			for(u8 count=0; count<sizeof(receive)-res; count++)
			receive[res+count] = 'b';			
			res=0;				
		}
	}
		
	take_step();		
				
//			onestep=(receive[0]-48)*10*(taking-1)+receive[1]*10*(taking-1)-48;
				
}	

void take_step(void)
{	
	u8 taking=receivedata-1;
		while(receive[taking]=='b')
		{
		taking--;		
		}
		for(u8 minus=0; minus<taking;minus++)
			{	
			onestep=(receive[minus]-48)*10^(taking-minus-1)+onestep;
			}			
}	


void UART7_IRQHandler(void)                	
{ 
		HAL_UART_IRQHandler(&huart7);	//����HAL���жϴ����ú���	
  	HAL_UART_Receive_IT(&huart7,(u8 *)rdata, RDATASIZE);
}	
#endif
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//	if(huart->Instance==UART7)//����Ǵ���1
//	{
//		if((UART_RX_STA&0x8000)==0)//����δ���
//		{
//			if(UART_RX_STA&0x4000)//���յ���0x0d
//			{
//				if(aRxBuffer[0]!=0x0a)UART_RX_STA=0;//���մ���,���¿�ʼ
//				else UART_RX_STA|=0x8000;	//��������� 
//			}
//			else //��û�յ�0X0D
//			{	
//				if(aRxBuffer[0]==0x0d)UART_RX_STA|=0x4000;
//				else
//				{
//					UART_RX_BUF[UART_RX_STA&0X3FFF]=aRxBuffer[0] ;
//					UART_RX_STA++;
//					if(UART_RX_STA>(UART_REC_LEN-1))UART_RX_STA=0;//�������ݴ���,���¿�ʼ����	  
//				}		 
//			}
//		}


//		for(u8 count=0; count<huart->RxXferSize; count++)
//		{
//			rdata[count] = *(huart->pRxBuffPtr - huart->RxXferSize + count);
//			if(rdata[count] == 0x0A)
//			break;
//		}
//		rec=*(huart->pRxBuffPtr - huart->RxXferSize);
//		rec ++;
//		if(rec >= RDATASIZE || rdata[rec-1]== 0x0a)
//		{
//			rec=0;	
//		}
//		HAL_UART_Transmit(&huart7, (u8 *)rdata,RDATASIZE,0xFFFF);
// 		HAL_UART_Receive_IT(&huart7, (u8 *)rdata, RDATASIZE);

//	}
//} 