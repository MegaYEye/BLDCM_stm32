#include "stm32f10x.h"


#ifndef _DEFINES_ME_H
#define _DEFINES_ME_H
typedef int YE_int32;
typedef short int YE_int16;
typedef unsigned int YE_Uint32;
typedef unsigned short int YE_Uint16;
#define FLASHMODE
//#define RAM_MODE
#define BUFFER_CURRENT 3
#define BUFFER_DIANWEIQI 10
#define SEND_BUFFER_LENGTH 10
#define RECEIVE_BUFFER_LENGTH 10
//************speed_filter**************//
#define RC_HALL_SPEED_INTERA_1000 900
//===========overspeed ===========/
#define OVER_SPEED_UP_GATE 4400
//1650
#define OVER_SPEED_DOWN_GATE 4100
#define OVER_SPEED_COUNT_GATE 20
//xian liu ruan qi dong
#define START_CURRENT 500
#define TIME_POWER_ON_DELAY 3
#define TIME_IPM_ENABLE_DELAY 4
#define MAGNET_POLE_PAIR 4
#define SQUARE1_FOC0 0
//========soft start============//
#define ACCELERATE_DELAY_MS 50
#define ACCELERATE_GATE 50
#define ACCELERATE_STEP_SPEED 3
#define ACCELERATE_TIME 10000
#define INNERLOOP_OPEN_TIME_MS 8000
//4
#define PWM_FREQ 15000


#define PWM_UPLIMIT  pwm_generator.pwm_period-10
#define PWM_DOWNLIMIT 10

#define RC_CURRENT_SAMPLE 85
#define RC_CURRENT_SAMPLE2 85

//430
#define OC_VALUE 3800 
//500 is 4A(sample : sum 3 times,u+v+w value),lilun jisuan:465
//should be changed
//3800 is over 30A
#define OC_LEVEL2_UPGATE 2500
#define OC_LEVEL2_DOWNGATE 2200 
/******overload 1126***********/
#define OVERLOAD_TIME_MS 10000
//5 is ok
#define OVERLOAD_UPGATE 1500
#define OVERLOAD_DOWNGATE 1400
/************duzhuan**************/
#define DUZHUAN_WATCHDOG_START_SPEED 100
#define HALL_MEM_SIZE 8
#define DUZHUAN_FUNCTION_TIME 3

#define SPEED_GIVEN_WIDTH 4200
//1400 or 1000
#define MIN_GIVEN_SPEED 120
//120
#define IPM_FO_RECHECK_TIME_DELAY_MS 200
//430
//about 14A
#define CURRENT_STOP_FEEDBACK_K 10
//50

#define LED1_ON GPIO_SetBits(GPIOE,GPIO_Pin_2)
#define LED2_ON GPIO_SetBits(GPIOE,GPIO_Pin_3)
#define LED1_OFF GPIO_ResetBits(GPIOE,GPIO_Pin_2)
#define LED2_OFF GPIO_ResetBits(GPIOE,GPIO_Pin_3)
#define RELAY_ON GPIO_SetBits(GPIOA,GPIO_Pin_7)
#define RELAY_OFF GPIO_ResetBits(GPIOA,GPIO_Pin_7)

#define T1_H GPIO_SetBits(GPIOB,GPIO_Pin_7)
#define T1_L GPIO_ResetBits(GPIOB,GPIO_Pin_7)
#define T2_H GPIO_SetBits(GPIOB,GPIO_Pin_6)
#define T2_L GPIO_ResetBits(GPIOB,GPIO_Pin_6)
#define T3_H GPIO_SetBits(GPIOB,GPIO_Pin_5)
#define T3_L GPIO_ResetBits(GPIOB,GPIO_Pin_5)
#define ECHO_DIO_H GPIO_SetBits(GPIOB,GPIO_Pin_8)
#define ECHO_DIO_L GPIO_ResetBits(GPIOB,GPIO_Pin_8)
#define ECHO_CLK_H GPIO_SetBits(GPIOB,GPIO_Pin_9)
#define ECHO_CLK_L GPIO_ResetBits(GPIOB,GPIO_Pin_9)
#define ECHO_STB_H GPIO_SetBits(GPIOE,GPIO_Pin_0)
#define ECHO_STB_L GPIO_ResetBits(GPIOE,GPIO_Pin_0)

#define IS_UV GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_4)>0
#define IS_OV GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_0)==0
#define IS_BKOV GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_5)==0
#define GET_JMP1 GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_10)
//zan ding:zhuan xiang.
#define GET_JMP2 GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_11)
//no use...
#define GET_HALL_UVW GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_6)*4+GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_7)*2+GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_8);
#define IS_PHASE_OC GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_1)>0
//fo=low means fault 
#define IS_IPM_FO GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_12)==0
#define IS_IPM_OT GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_14)==0
#define IPM_INPUT_ENABLE GPIO_SetBits(GPIOD,GPIO_Pin_13)
#define IPM_INPUT_DISABLE GPIO_ResetBits(GPIOD,GPIO_Pin_13)
#define IS_IPM_INPUT_ENABLE GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_13)!=0
#define PWMMODE_TIM1 TIM_OCMode_PWM2

struct typePID_Def{
	int Kp;
	int Ki;
	int Kd;
	int *given_value;

	signed int ek;
	signed int ek_1;
	signed int ek_2;
	signed int P;
	
	signed int IntContainer;
	int Out_Up_Limit;
	int Out_Down_Limit;
	int INT_Up_Limit;
	int INT_Down_Limit;
	int KI_prescaler;
	int KP_prescaler;
	int KD_prescaler;
	int PID_EN;

};
struct Time_Def{
	volatile long time_s;
	volatile long time_ms;
	volatile long time_us;
};
struct Fault_Info_Def{
	int uv;
	int ov;
	int ot;
	int bkov;
	int ipmfo;
	int oc;
	int hall;
	int start_fault;
	int oc_source;
	int overload;
	int i_sensor;
	int ting_ji;
	int over_speed;//shi liang kong zhi : fang zhi fei che
	
	int duzhuan_watchdog;
	int duzhuan_check_hall_counter;
	int hall_memory[6];

	struct Time_Def last_ipmfo_time;
	struct Time_Def last_oc_time;
	struct Time_Def next_fault_check_time;
	struct Time_Def overload_start_time;
	struct Time_Def oclevel2_start_time;

	int oc_counter;
	int ipmfo_counter;
	int fault_reset;
	int overload_flag;
	int oclevel2_flag;
	
};
struct PWM_Generator_Def{

	int pwm_value_square;
	int pwm_period;
};
struct Hall2Speed_info_Def{
	int cur_Hall_state;
	int pre_Hall_state;
	int last_Hall_state;
	volatile int speed;
	int speed_input;
	int duzhuan;
	struct Time_Def last_com_time;
	
	
};
 
struct Current_Sample_Info_Def{
	int AD_zero_current_u;
	int AD_zero_current_v;
	
	int zero_calibrator_alpha;
	int zero_calibrator_beta;
	int offset_zero_current_u;
	int offset_zero_current_v;
	
	int current_U;
	int current_V;
	int current_W;
	int current_uvw;
	int calibration_times;
	struct Time_Def calibration_start_time;
	struct Time_Def next_sample_time;
	struct Time_Def check_interval_time;

	int calibration_count;
	int calibration_step;
	int sample_data_u_calibration;
	int sample_data_v_calibration;
	int calibration_finish;
	int error_u;
	int error_v;

}
;

struct Controller_Mode_Def{
	int ipm_charging;
	int restarting;
	int zheng_fan_zhuan;
	int state_square1_foc0;


	int sys_Fault;
	int Udc;
	struct Time_Def next_echo_time;
}
;
struct RS485_Info_Def{
	int send_state;//0=idle,1=sending
	char *sendbuffer;
	char *receivebuffer;
	int pointer_receive_buffer;
};

struct StateMachine{
	int cur_state;
	int last_state;
	int pre_state;
};

struct Motor_Starter{
	int start_finish;
	int starting_step;
	struct Time_Def accelerate_start_time1;
	struct Time_Def accelerate_start_time2;
	struct Time_Def accelerate_start_time3;
	struct Time_Def accelerate_start_time4;
	int duty_limit_Base;
	int Max_PWMDUTY_auto_gain;
	int ipm_charging_step;
	int step3_finish_flag;
	int accelerate_speed_curve;
	struct Time_Def IPM_charing_start_time;

};

struct Current_Stop_Feedback{
	int feedback_K;
	int feedback_K2;
	int gate_value;
	int feedback_EN;
	int output_value;
};
struct Rotor_Angle_Est_type{
	int rotor_angle;
	int est_angle_60;
	int hall_COM_signal;
	int est_count;
	int last_count;
	int diff_count;
	int EST_EN;
	
};
struct FOC_SVPWM_Def{
	int I_abc_FB[3];
	int I_dq_FB[2];
	int I_alpha_beta_FB[2];

	int I_dq_given[2];
	int U_dq_given[2];
	int U_alpha_beta_given[2];
	int U_alpha_beta_given_Area;
	
	int U_maxCircle;
	int U_limitCircle;
	int Vector_Time[3];
	int Vector_Time_t1;
	int Vector_Time_t2;
	
	int SVPWM_Openloop_switch;
};
struct RC_FILTER
{
	int in_data;
	int out_data;
	int inertia_1000;
};

//==========macro::just for debug:MODE=============//
#define CLOSE_LOOP
#define ZHUANXIANG_ZHENG1FAN2 2
//==========macro::just for debug:DA=============//
//#define DAC1_ELEC_ANGLE
//#define DAC2_TIM1_CCR1
//#define DAC2_TIM1_CCR3//this 
//#define DAC1_TIM1_CCR1
//#define DAC1_HALL_V
//#define DAC1_HALL_W
//#define DAC1_HALL_6STEP
//#define DAC1_Voltage_UV
//#define DAC2_IAFB
//#define DAC1_IQFB
//#define DAC12_IABC_AB 
//#define DAC12_UAB_G
#define DAC12_PID_CURRENT
//#define DAC12_PIDSPEED
//#define DAC12_PID_TORQUE
//#define DAC1_START_STEP
//#define DAC12_PIDSPEED
//#define DAC1_SPEED
#endif
