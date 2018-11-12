#include "MyGlobal.h"
#include "defines.h"

int ADVal_current_UV[BUFFER_CURRENT];
int ADVal_GeiDing[BUFFER_DIANWEIQI];
int DISP_Buffer[10];
int disp_data;
char send_buffer[SEND_BUFFER_LENGTH];
char receive_buffer[RECEIVE_BUFFER_LENGTH];
int speed_kp_base_value;
int speed_ki_base_value;
int current_pid_out_up_limit;
int torque_pid_out_up_limit;
//short int zspeed[5000];



//na lu shang guan kai le,jiu jian ce na lu dian liu 
struct typePID_Def  PID_torque;
struct typePID_Def  PID_id;
struct typePID_Def PID_speed;
struct typePID_Def  PID_current;
struct Fault_Info_Def faultinfo;
struct PWM_Generator_Def pwm_generator;
struct Current_Sample_Info_Def currentinfo;
struct Time_Def globaltime;
struct Controller_Mode_Def controllermode;
struct RS485_Info_Def RS485info;
struct Hall2Speed_info_Def hall2speedinfo;
struct Motor_Starter motor_starter;
struct StateMachine Frame_sm;
struct Current_Stop_Feedback csfb;
struct FOC_SVPWM_Def focinfo_sv;
struct Rotor_Angle_Est_type rotor_angle_est;
struct RC_FILTER rc_current_d;
struct RC_FILTER rc_current_q;
struct RC_FILTER rc_current_uvw;
struct RC_FILTER rc_echo_speed;
struct RC_FILTER rc_hall_speed;
TIM_TimeBaseInitTypeDef tim_time_Base_init_square;
TIM_OCInitTypeDef tim_oc_init_square;
TIM_TimeBaseInitTypeDef tim_time_Base_init_foc;
TIM_OCInitTypeDef tim_oc_init_foc;
int delay_ms(int ms)
{
	struct Time_Def targettime;
	targettime=globaltime;
	Plus_Time_ms(&targettime,ms);
	
	
	if(Get_Time_Interval_ms(&globaltime,&targettime)<=0)
	{
		return 0;
	}
	else 
	{
		return 1;
	}
	
}
void error(int error_code)
{
		//push(error_code);
	int a=1;
	a++;
}
int Get_Time_Interval_ms(struct Time_Def *t1,struct Time_Def *t2)
{
		int cha_s=t1->time_s-t2->time_s;
		int cha_ms=t1->time_ms-t2->time_ms;
	//	int cha_us=t1->time_us-t2->time_us;
		int ans=cha_s*1000+cha_ms;
		return ans;
		
}
int Get_Time_Interval_0p1ms(struct Time_Def *t1,struct Time_Def *t2)
{
		int cha_s=t1->time_s-t2->time_s;
		int cha_ms=t1->time_ms-t2->time_ms;
	  int cha_0p1ms=(t1->time_us-t2->time_us)/100;
		int ans=cha_s*10000+cha_ms*10+cha_0p1ms;
		return ans;
}
int Get_Time_Interval_us(struct Time_Def *t1,struct Time_Def *t2)
{
		int cha_s=t1->time_s-t2->time_s;
		int cha_ms=t1->time_ms-t2->time_ms;
	  int cha_us=(t1->time_us-t2->time_us);
		int ans=cha_s*1000000+cha_ms*1000+cha_us;
		return ans;
}
void Plus_Time_ms(struct Time_Def *t,int ms)
{
	t->time_ms+=ms;
	if(t->time_ms>=1000)
	{
		t->time_s+=1;
		t->time_ms-=1000;
	}
}
void DAC_outputdata(int data)
{
	DAC_SetChannel1Data(DAC_Align_12b_R,data);
	DAC_SoftwareTriggerCmd(DAC_Channel_1,ENABLE);
}
void DAC_outputdata2(int data)
{
	DAC_SetChannel2Data(DAC_Align_12b_R,data);
	DAC_SoftwareTriggerCmd(DAC_Channel_2,ENABLE);
}
