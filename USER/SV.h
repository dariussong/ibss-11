#ifndef _SV_H
#define _SV_H
#include "sys.h"

void SV_Init(void); //LED初始化函数
void SV_ESTIMATE(void);  // 判断函数
void SV_test1(void);
void SV_test2(void);
extern u8 sv_flag[4];
void pid_value_init(void);
void update_integral_list(float update_value);  // 积分项更新函数
void air_control(void);
#endif