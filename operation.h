#ifndef _OPERATION_H
#define _OPERATION_H
#include "defines.h"
void Initial(void);
void USART1_Send_Data(char * string);
void BLDC_Control(int state);
void Isensor_Zero_Calibration(int sample_times,int test_times,int error);
int Get_Speed_Given(void);
int Get_Current_UVW(int *u,int *v,int *w);
void Init_variables(void);
void SYS_Reinit(void);
void Motor_start_SQUARE(void);
void IPM_Charge(void);
void Update_Speed(void);
int Hall_filter(void);
void PID_calculator(struct typePID_Def * PID_object,int feedback);
int Get_Pre_Hall_State(int cur_hall_state);
void Change_PWM_state_PWM_ON(int hall_state);
int Gain_With_Time(int min,int max,int time_interval_ms,struct Time_Def * start_time);
void Code_Entrance(void);
void Motor_Running(void);
void filter(struct RC_FILTER *obj,int input_value);
void CSFB_calculate(void);
void SQUARE_ENTRANCE(void);
void Echo_speed(void);
void Echo_Make_Number(void);
void Com2TM1638(void);
#endif
