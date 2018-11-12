
#ifndef _MYGLOBAL_H
#define _MYGLOBAL_H
#include "defines.h"
extern int delay_ms(int ms);
extern void error(int error_code);
extern int Get_Time_Interval_0p1ms(struct Time_Def *t1,struct Time_Def *t2);
extern int Get_Time_Interval_ms(struct Time_Def *t1,struct Time_Def *t2);
extern int Get_Time_Interval_us(struct Time_Def *t1,struct Time_Def *t2);
extern void Plus_Time_ms(struct Time_Def *t,int ms);
extern void DAC_outputdata(int data);
extern void DAC_outputdata2(int data);
//extern  short int zspeed[5000];

extern int ADVal_current_UV[BUFFER_CURRENT];
extern int ADVal_GeiDing[BUFFER_DIANWEIQI];
extern int DISP_Buffer[10];
extern int disp_data;
extern char send_buffer[SEND_BUFFER_LENGTH];
extern char receive_buffer[RECEIVE_BUFFER_LENGTH];
extern int speed_kp_base_value;
extern int speed_ki_base_value;
extern int current_pid_out_up_limit;
extern int torque_pid_out_up_limit;

 
 

extern struct typePID_Def  PID_torque;
extern struct typePID_Def  PID_id;
extern struct typePID_Def PID_speed;
extern struct typePID_Def  PID_current;
extern struct Fault_Info_Def faultinfo;
extern struct PWM_Generator_Def pwm_generator;
extern struct Current_Sample_Info_Def currentinfo;
extern struct Time_Def globaltime;
extern struct Controller_Mode_Def controllermode;
extern struct RS485_Info_Def RS485info;
extern struct Hall2Speed_info_Def hall2speedinfo;
extern struct Motor_Starter motor_starter;
extern struct StateMachine Frame_sm;
extern struct Current_Stop_Feedback csfb;
extern struct FOC_SVPWM_Def focinfo_sv;
extern struct Rotor_Angle_Est_type rotor_angle_est;
extern struct RC_FILTER rc_current_d;
extern struct RC_FILTER rc_current_q;
extern struct RC_FILTER rc_current_uvw;
extern struct RC_FILTER rc_echo_speed;
extern struct RC_FILTER rc_hall_speed;
extern TIM_TimeBaseInitTypeDef tim_time_Base_init_square;
extern TIM_OCInitTypeDef tim_oc_init_square;
extern TIM_TimeBaseInitTypeDef tim_time_Base_init_foc;
extern TIM_OCInitTypeDef tim_oc_init_foc;
#endif

