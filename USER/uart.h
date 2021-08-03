#ifndef __UART_H
#define __UART_H
#include "sys.h"
#include "stdio.h"	


#define UART_REC_LEN  			3  	//�����������ֽ��� 200
#define EN_UART7_RX 			1		//ʹ�ܣ�1��/��ֹ��0������1����
#define RXBUFFERSIZE   1 
#define RDATASIZE   1 
#define receivedata 20
extern u8 taking;
extern UART_HandleTypeDef huart7;
extern u8 receive[receivedata];

void MY_UART7_Init(void);
uint16_t UART_ReceiveData(USART_TypeDef* huart7);
extern u8  UART_RX_BUF[UART_REC_LEN]; //���ջ���,���USART_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з� 
extern u16 UART_RX_STA;         		//����״̬���	
//extern UART_HandleTypeDef UART7_Handler; //UART���
extern u8 aRxBuffer[RXBUFFERSIZE];
extern u8 onestep;
extern u8 rdata[RDATASIZE];
void take_step(void);
#endif 
