#ifndef __UART_H
#define __UART_H
#include "sys.h"
#include "stdio.h"	

#define UART_REC_LEN  			3  	//�����������ֽ��� 200
#define EN_UART7_RX 			1		//ʹ�ܣ�1��/��ֹ��0������1����
#define RXBUFFERSIZE   1 
#define RDATASIZE   1 
#define receivedata 30



void MY_UART7_Init(void);
extern UART_HandleTypeDef huart7;
extern char receive[receivedata];


extern int onestep;

//�յ����趨 ��ѹ�����ڣ���̬���г̣�̧�ȸ߶ȣ�ת��Ƕ�
extern int receiveSetP; 
extern int receiveSetT;
extern int receiveSetG;
extern int receiveSetL;
extern int receiveSetH;
extern int receiveSetW;
extern int recMotion_flag;

extern char *revbuf[10];

extern char *recstring0;

uint16_t UART_ReceiveData(USART_TypeDef* huart7);
extern u8  UART_RX_BUF[UART_REC_LEN]; //���ջ���,���USART_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з� 
extern u16 UART_RX_STA;         		//����״̬���	

extern u8 aRxBuffer[RXBUFFERSIZE];
extern u8 rdata[RDATASIZE];

void split(char *src,const char *separator,char **dest,int *num);

void sendP(void);

#endif 

//#define outputdata 4
//extern u8 out_put_1[outputdata];
//extern u8 out_put_2[outputdata];
//extern u8 out_put_3[outputdata];
//extern u8 callback1;
//extern u8 callback2;
//extern u8 callback3;