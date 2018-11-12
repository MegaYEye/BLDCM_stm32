#include "Operation.h"
#include "DeviceDriver.h"
#include "stm32f10x.h"
#include "myglobal.h"
#include "FOC.h"
#include "fault.h"
void Init_variables(void)
{
	int i=0;
	for(i=0;i<HALL_MEM_SIZE;i++)
	{
		faultinfo.hall_memory[i]=0;
	}
	for(i=0;i<BUFFER_CURRENT;i++)
	{
			ADVal_current_UV[i]=0;
		
	}
	for(i=0;i<BUFFER_DIANWEIQI;i++)
	{
			ADVal_GeiDing[i]=0;
	}
	for(i=0;i<SEND_BUFFER_LENGTH;i++)
	{
			send_buffer[i]=0;
	}
	for(i=0;i<RECEIVE_BUFFER_LENGTH;i++)
	{
		receive_buffer[i]=0;
	}
	globaltime.time_s=0;
	globaltime.time_ms=0;
	globaltime.time_us=0;
	
	RS485info.pointer_receive_buffer=0;
	RS485info.receivebuffer= receive_buffer;
	RS485info.sendbuffer=send_buffer;
	RS485info.send_state=0;
 
	hall2speedinfo.cur_Hall_state=GET_HALL_UVW;
	hall2speedinfo.duzhuan=1;
	hall2speedinfo.last_com_time.time_ms=0;
	hall2speedinfo.last_com_time.time_s=0;
	hall2speedinfo.last_com_time.time_us=0;
	hall2speedinfo.pre_Hall_state=Get_Pre_Hall_State(hall2speedinfo.cur_Hall_state);
	hall2speedinfo.speed=0;
	rc_hall_speed.out_data=0;
	hall2speedinfo.speed_input=0;
	
 
	pwm_generator.pwm_period=SystemCoreClock/PWM_FREQ-1;


	currentinfo.AD_zero_current_u=0;
	currentinfo.AD_zero_current_v=0;
	currentinfo.current_U=0;
	currentinfo.current_V=0;
	currentinfo.current_W=0;
	currentinfo.current_uvw=0;
	currentinfo.zero_calibrator_alpha=0;
	currentinfo.zero_calibrator_beta=0;

	controllermode.ipm_charging=0;
	controllermode.sys_Fault=0;
	//controllermode.zheng_fan_zhuan=ZHUANXIANG_ZHENG1FAN2;
	//1125

	controllermode.Udc=10000;
	controllermode.state_square1_foc0=0;

	controllermode.next_echo_time.time_ms=0;
	controllermode.next_echo_time.time_s=0;
	controllermode.next_echo_time.time_us=0;
 
//this is pid.....
	speed_kp_base_value=17;
	speed_ki_base_value=1;
	
	PID_speed.Kd=0;
	PID_speed.Ki=speed_ki_base_value;
	;//too large
	PID_speed.Kp=17;//speed_kp_base_value;//too large 20140313:17
	PID_speed.KI_prescaler=60;//1126 100 yuanlai 600
	PID_speed.KP_prescaler=3;//gai zeng yi bu shou dian liu zao sheng ying xiang...
	PID_speed.KD_prescaler=100;
	
	PID_speed.Out_Down_Limit=-800;//how can i make it larger???
	PID_speed.Out_Up_Limit=1950;//0223
	PID_speed.INT_Up_Limit=PID_speed.Out_Up_Limit*1000;
	PID_speed.INT_Down_Limit=PID_speed.Out_Down_Limit*1000;
	PID_speed.ek=0;
	PID_speed.ek_1=0;
	PID_speed.ek_2=0;

	PID_speed.P=0;
	PID_speed.PID_EN=0;
	PID_speed.IntContainer=0;

	PID_speed.PID_EN=0;
	 
	 //this is pid.....
	 PID_torque.Kp=5;//0223
	 PID_torque.Ki=1;
	 PID_torque.Kd=0;
	 PID_torque.KP_prescaler=2;
	 PID_torque.KI_prescaler=60;//20140312
	 PID_torque.KD_prescaler=1000;
	 PID_torque.ek=0;
	 PID_torque.ek_1=0;
	 PID_torque.ek_2=0;
	 PID_torque.Out_Up_Limit=controllermode.Udc*70/100;//0.707fang zhi guo tiao zhi...
	 PID_torque.Out_Down_Limit=0;
	 PID_torque.INT_Up_Limit= PID_torque.Out_Up_Limit*1000;
	 PID_torque.INT_Down_Limit=PID_torque.Out_Down_Limit*1000;
	 torque_pid_out_up_limit=PID_torque.Out_Up_Limit;
	 PID_torque.P=0;
	 PID_torque.PID_EN=0;
	 //this is pid.....
	 PID_id.Kp=10;
	 PID_id.Ki=1;
	 PID_id.Kd=0;
	 PID_id.KP_prescaler=2;
	 PID_id.KI_prescaler=200;
	 PID_id.KD_prescaler=1000;
	 PID_id.ek=0;
	 PID_id.ek_1=0;
	 PID_id.ek_2=0;
	 PID_id.Out_Up_Limit=controllermode.Udc*57/100;//1128
	 PID_id.Out_Down_Limit=0-PID_id.Out_Up_Limit;
	 PID_id.INT_Up_Limit=PID_id.Out_Up_Limit*1000;
	 PID_id.INT_Down_Limit=PID_id.Out_Down_Limit*1000;

	 PID_id.P=0;
	 PID_id.PID_EN=0;
	 
	 	PID_current.Out_Down_Limit=PWM_DOWNLIMIT;
		PID_current.Out_Up_Limit=PWM_UPLIMIT;//test
		PID_current.INT_Up_Limit=PID_current.Out_Up_Limit*1000;
		PID_current.INT_Down_Limit=PID_current.Out_Down_Limit*1000;
		
		current_pid_out_up_limit=PID_current.Out_Up_Limit;
		
		//this is pid.....
		PID_current.Kp=10;
		PID_current.Ki=1;
		PID_current.Kd=0;
		PID_current.KP_prescaler=2;
		PID_current.KI_prescaler=100;//1204
		PID_current.KD_prescaler=100;

		PID_current.IntContainer=0;
		PID_current.ek=0;
		PID_current.ek_1=0;
		PID_current.ek_2=0;
		PID_current.P=0;

		PID_current.PID_EN=0;
		faultinfo.duzhuan_watchdog=0;
		faultinfo.duzhuan_check_hall_counter=0;
	 
	 rotor_angle_est.diff_count=0;
	 rotor_angle_est.est_count=0;
	 rotor_angle_est.last_count=0;
	 rotor_angle_est.rotor_angle=0;
	 rotor_angle_est.EST_EN=0;
	 rotor_angle_est.hall_COM_signal=0;
	
	faultinfo.over_speed=0;
	faultinfo.ting_ji=0;
	 faultinfo.bkov=0;
	 faultinfo.hall=0;
	 faultinfo.ipmfo=0;
	 faultinfo.oc=0;
	 faultinfo.i_sensor=0;
	 faultinfo.oc_source=0;
	 faultinfo.oc_counter=0;
	 faultinfo.ipmfo_counter=0;
	 faultinfo.fault_reset=0;
	 faultinfo.ot=0;
	 faultinfo.ov=0;
	 faultinfo.uv=0;
	 faultinfo.start_fault=0;

	 faultinfo.last_ipmfo_time.time_s=0;
	 faultinfo.last_ipmfo_time.time_ms=0;
	 faultinfo.last_ipmfo_time.time_us=0;
	 faultinfo.last_oc_time.time_s=0;
	 faultinfo.last_oc_time.time_ms=0;
	 faultinfo.last_oc_time.time_us=0;
	 faultinfo.overload_start_time.time_ms=0;
	 faultinfo.overload_start_time.time_s=0;
	 faultinfo.overload_start_time.time_us=0;
	 faultinfo.overload_flag=0;
	 faultinfo.overload=0;
	 faultinfo.oclevel2_flag=0;
	 faultinfo.oclevel2_start_time.time_us=0;
	 faultinfo.oclevel2_start_time.time_ms=0;
	 faultinfo.oclevel2_start_time.time_s=0;
 
	 Frame_sm.cur_state=0;
	 Frame_sm.last_state=0;
	 Frame_sm.pre_state=0;

	 
	 csfb.feedback_EN=1;
	 csfb.feedback_K=15;
	 csfb.feedback_K2=15;//you dai tiao zheng...
	 csfb.gate_value=2000;//1119:1300 //1126 1650 //20140228 1750
	 csfb.output_value=0;
	 
	 rc_current_d.inertia_1000=850;
	 rc_current_d.in_data=0;
	 rc_current_d.out_data=0;
	 rc_current_q.inertia_1000=850;
	 rc_current_q.in_data=0;
	 rc_current_q.out_data=0;
	
	focinfo_sv.U_maxCircle=controllermode.Udc;
	focinfo_sv.U_limitCircle=focinfo_sv.U_maxCircle*70/100;
	
	rc_echo_speed.inertia_1000=850;
	rc_echo_speed.in_data=0;
	rc_echo_speed.out_data=0;
	
	rc_hall_speed.in_data=0;
	rc_hall_speed.out_data=0;
	rc_hall_speed.inertia_1000=RC_HALL_SPEED_INTERA_1000;//temp
	
	hall2speedinfo.speed_input=0;
	

	
	
}
void ReInit_variables(void)
{
	int i=0;
	for(i=0;i<BUFFER_CURRENT;i++)
	{
			ADVal_current_UV[i]=0;
		
	}
	for(i=0;i<BUFFER_DIANWEIQI;i++)
	{
			ADVal_GeiDing[i]=0;
	}

	//globaltime.time_s=0;
	//globaltime.time_ms=0;
	//globaltime.time_us=0;
	for(i=0;i<HALL_MEM_SIZE;i++)
	{
		faultinfo.hall_memory[i]=0;
	}
	rc_hall_speed.in_data=0;
	rc_hall_speed.out_data=0;
	rc_hall_speed.inertia_1000=RC_HALL_SPEED_INTERA_1000;//temp
 
	hall2speedinfo.cur_Hall_state=GET_HALL_UVW;
	hall2speedinfo.duzhuan=1;
	hall2speedinfo.last_com_time.time_ms=0;
	hall2speedinfo.last_com_time.time_s=0;
	hall2speedinfo.last_com_time.time_us=0;
	hall2speedinfo.pre_Hall_state=GET_HALL_UVW;
	hall2speedinfo.last_Hall_state=0;
	hall2speedinfo.speed=0;
	rc_hall_speed.out_data=0;
	
 
//	pwm_generator.pwm_period=SystemCoreClock/PWM_FREQ-1;


	currentinfo.AD_zero_current_u=0;
	currentinfo.AD_zero_current_v=0;
	currentinfo.current_U=0;
	currentinfo.current_V=0;
	currentinfo.current_W=0;
	currentinfo.current_uvw=0;
	
	controllermode.ipm_charging=0;
	controllermode.sys_Fault=0;
 
	controllermode.next_echo_time.time_ms=0;
	controllermode.next_echo_time.time_s=0;
	controllermode.next_echo_time.time_us=0;
	//controllermode.zheng_fan_zhuan=1;
 
	rc_echo_speed.in_data=0;
	rc_echo_speed.out_data=0;
	//PID_speed.Out_Down_Limit=-2000;
	//PID_speed.Out_Up_Limit=2000;
	//PID_speed.INT_Up_Limit=1000000;
	//PID_speed.INT_Down_Limit=-1000000;
	PID_speed.ek=0;
	PID_speed.ek_1=0;
	PID_speed.ek_2=0;
	//PID_speed.Kd=0;
	//PID_speed.Ki=1;//too large
	//PID_speed.Kp=4;//too large
	PID_speed.P=0;
	//PID_speed.given_value=0;
	PID_speed.PID_EN=0;
	PID_speed.IntContainer=0;
	//PID_speed.INT_prescaler=600;
	//PID_speed.P_prescaler=2;
	
 
 faultinfo.over_speed=0;
	faultinfo.ting_ji=0;
	 faultinfo.bkov=0;
	 faultinfo.hall=0;
	 faultinfo.ipmfo=0;
	 faultinfo.oc=0;
	 faultinfo.i_sensor=0;
	 faultinfo.oc_source=0;
	 faultinfo.oc_counter=0;
	 faultinfo.ipmfo_counter=0;
	 faultinfo.ot=0;
	 faultinfo.ov=0;
	 faultinfo.uv=0;
	 faultinfo.overload_flag=0;
	 faultinfo.overload=0;
	 faultinfo.start_fault=0;
	 faultinfo.duzhuan_watchdog=0;
	 faultinfo.duzhuan_check_hall_counter=0;
	 faultinfo.last_ipmfo_time.time_s=0;
	 faultinfo.last_ipmfo_time.time_ms=0;
	 faultinfo.last_ipmfo_time.time_us=0;
	 faultinfo.last_oc_time.time_s=0;
	 faultinfo.last_oc_time.time_ms=0;
	 faultinfo.last_oc_time.time_us=0;
	 faultinfo.overload_start_time.time_ms=0;
	 faultinfo.overload_start_time.time_s=0;
	 faultinfo.overload_start_time.time_us=0;
	 
	 faultinfo.oclevel2_flag=0;
	 faultinfo.oclevel2_start_time.time_us=0;
	 faultinfo.oclevel2_start_time.time_ms=0;
	 faultinfo.oclevel2_start_time.time_s=0;
	 
 
	 faultinfo.fault_reset=0;
	 controllermode.state_square1_foc0=0;

	 csfb.output_value=0;

		PID_torque.Kp=10;
		PID_torque.Ki=10;
		hall2speedinfo.speed_input=0;


}
 
void Reset_PID(struct typePID_Def *PID_obj)
{
	PID_obj->ek=0;
	PID_obj->ek_1=0;
	PID_obj->ek_2=0;
	PID_obj->IntContainer=0;
	//PID_obj->given_value=NULL;
	PID_obj->P=0;
	PID_obj->PID_EN=0;
}

void USART1_Send_Data(char * string)
{
	int i;
	if(RS485info.send_state==0)
	{
		for(i=0;i<SEND_BUFFER_LENGTH;i++)
		{
				if(string[i]!='\0')
				{
						send_buffer[i]=string[i];
				}
				else
				{
						break;
				}
		}
		for(;i<SEND_BUFFER_LENGTH;i++)
		{
				send_buffer[i]='\0';
		}
		USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);	//is this the right way to send_buffer again?????
		RS485info.send_state=1;
	}
	
}

void IPM_Charge(void)
{
	
	if(motor_starter.ipm_charging_step==0)
	{
		controllermode.ipm_charging=1;
		motor_starter.ipm_charging_step=1;
		motor_starter.IPM_charing_start_time=globaltime;
		TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_ForcedAction_Active );
		TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Disable);
    TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Enable);
	
		TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_ForcedAction_Active );
		TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Disable);
    TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Enable);
	
		TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_ForcedAction_Active );
		TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Disable);
    TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Enable);
		TIM_GenerateEvent(TIM1,TIM_EventSource_COM);
		Plus_Time_ms(&(motor_starter.IPM_charing_start_time),400);
		motor_starter.ipm_charging_step=1;
	}
	else if(motor_starter.ipm_charging_step==1)
	{
			
		if(Get_Time_Interval_ms(&globaltime,&(motor_starter.IPM_charing_start_time))>0)
		{
			TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_ForcedAction_InActive );
			TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Disable);
			TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Disable);
		
			TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_ForcedAction_InActive );
			TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Disable);
			TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Disable);
		
			TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_ForcedAction_InActive );
			TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Disable);
			TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Disable);
			TIM_GenerateEvent(TIM1,TIM_EventSource_COM);
			
			Plus_Time_ms(&(motor_starter.IPM_charing_start_time),20);
			motor_starter.ipm_charging_step=2;
		}
	}
	else if(motor_starter.ipm_charging_step==2)
	{
		if(Get_Time_Interval_ms(&globaltime,&(motor_starter.IPM_charing_start_time))>0)
		{
			TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_ForcedAction_Active );
			TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable);
			TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Disable);
			TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_ForcedAction_Active );
			TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Enable);
			TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Disable);
		
			TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_ForcedAction_Active );
			TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Enable);
			TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Disable);
			TIM_GenerateEvent(TIM1,TIM_EventSource_COM);
			
			Plus_Time_ms(&(motor_starter.IPM_charing_start_time),20);
			motor_starter.ipm_charging_step=3;
		}
	}
	else if(motor_starter.ipm_charging_step==3)
	{
		if(Get_Time_Interval_ms(&globaltime,&(motor_starter.IPM_charing_start_time))>0)
		{
			TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_ForcedAction_InActive );
			TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Disable);
			TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Disable);
	
			TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_ForcedAction_InActive );
			TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Disable);
			TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Disable);
	
			TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_ForcedAction_InActive );
			TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Disable);
			TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Disable);
			TIM_GenerateEvent(TIM1,TIM_EventSource_COM);
			
			Plus_Time_ms(&(motor_starter.IPM_charing_start_time),20);
			motor_starter.ipm_charging_step=4;
		}
	}
	else if(motor_starter.ipm_charging_step==4)
	{
		if(Get_Time_Interval_ms(&globaltime,&(motor_starter.IPM_charing_start_time))>0)
		{
			motor_starter.ipm_charging_step=5;//finish
			controllermode.ipm_charging=0;
		}
	} 
}

void Init_Motor_start()
{
	int i=0;

	if(GET_JMP2>0)
	{
		controllermode.state_square1_foc0=1;
	}
	else
	{
		controllermode.state_square1_foc0=0;
	}
	//controllermode.state_square1_foc0=SQUARE1_FOC0;
	if(GET_JMP1>0)
	{
		controllermode.zheng_fan_zhuan=1;
	}
	else
	{
		controllermode.zheng_fan_zhuan=2;
	}
	
	LED1_ON;
	LED2_ON;
	T1_L;
	T2_L;
	T3_L;

	TIM_Cmd(TIM3, ENABLE);
	TIM_Cmd(TIM1,DISABLE);
	TIM_Cmd(TIM2,ENABLE);

	for(i=0;i<HALL_MEM_SIZE;i++)
	{
		faultinfo.hall_memory[i]=0;
	}
	
	motor_starter.start_finish=0;
	motor_starter.starting_step=0;
	currentinfo.calibration_step=0;
	currentinfo.zero_calibrator_alpha=0;
	currentinfo.zero_calibrator_beta=0;
	currentinfo.offset_zero_current_u=0;
	currentinfo.offset_zero_current_v=0;
	currentinfo.calibration_times=0;
	PID_id.PID_EN=0;
	PID_torque.PID_EN=0;
	PID_speed.PID_EN=0;
	PID_current.PID_EN=0;
	
	 rc_current_d.in_data=0;
	 rc_current_d.out_data=0;
	 rc_current_q.in_data=0;
	 rc_current_q.out_data=0;

	rc_current_uvw.inertia_1000=800;
	rc_current_uvw.in_data=0;
	rc_current_uvw.out_data=0;
	
	rc_hall_speed.in_data=0;
	rc_hall_speed.out_data=0;
	rc_hall_speed.inertia_1000=RC_HALL_SPEED_INTERA_1000;//temp
	 


	// controllermode.state_square1_foc0=0;
	faultinfo.duzhuan_watchdog=0;
	motor_starter.Max_PWMDUTY_auto_gain=0;
	 faultinfo.overload_flag=0;
	 rotor_angle_est.EST_EN=2;
	 for(i=0;i<BUFFER_CURRENT;i++)
	 {
		ADVal_current_UV[i]=0;
	 }
	 ADC_SoftwareStartConvCmd(ADC1,ENABLE);
	
}
void Set_PID_From_CurState(struct typePID_Def *PID_obj,int feedback,int P,int ek_1,int ek_2)
{
	int int_result=0;
	PID_obj->ek_1=ek_1;
	PID_obj->ek_2=ek_2;
	PID_obj->ek=*(PID_obj->given_value)-feedback;
	PID_obj->P=P;
	int_result=P-PID_obj->Kp*(PID_obj->ek)/PID_obj->KP_prescaler;
	
	if(int_result>(PID_obj->INT_Up_Limit))
	{
		int_result=(PID_obj->INT_Up_Limit);
	}
	if(int_result<(PID_obj->INT_Down_Limit))
	{
		int_result=(PID_obj->INT_Down_Limit);
	}
	PID_obj->IntContainer=int_result*1000;
	PID_obj->P=(PID_obj->Kp)*(PID_obj->ek)/(PID_obj->KP_prescaler);
	PID_obj->P+=PID_obj->IntContainer/1000;
	
}

void Motor_start_SQUARE(void)
{
			if(motor_starter.starting_step==0)
			{
				if(hall2speedinfo.speed_input>0)//2013.11.04
				{
					if((hall2speedinfo.speed==0)&&(hall2speedinfo.duzhuan>0))
					{
						Isensor_Zero_Calibration(20,10,50);
						if(currentinfo.calibration_finish>0)
						{
							if(GET_JMP1>0)
							{
								controllermode.zheng_fan_zhuan=1;
							}
							else
							{
								controllermode.zheng_fan_zhuan=2;
							}
							TIM_Cmd(TIM1,ENABLE);
							TIM_CtrlPWMOutputs(TIM1, ENABLE);//guan bu liao?????
							if(controllermode.sys_Fault==0&&Get_Speed_Given()>0)
							{
								IPM_INPUT_ENABLE;
								motor_starter.ipm_charging_step=0;
								motor_starter.starting_step=1;
								
							}
						}
				
					}
				}
				else //Speedinput<=0
				{
					if(IS_IPM_INPUT_ENABLE)
					{
						IPM_INPUT_DISABLE;
					}
				}
			}
			else if(motor_starter.starting_step==1)
			{
					IPM_Charge();
					if(motor_starter.ipm_charging_step>=5)
					{
	
							PID_current.Out_Up_Limit=0;
							PID_torque.Out_Up_Limit=0;
						
							Reset_PID(&PID_speed);
							Reset_PID(&PID_current);//0224

							motor_starter.starting_step=2;
						
							hall2speedinfo.cur_Hall_state=Hall_filter();

							Change_PWM_state_PWM_ON(hall2speedinfo.cur_Hall_state);
							TIM_GenerateEvent(TIM1,TIM_EventSource_COM);
							
							motor_starter.accelerate_start_time1=globaltime;
							currentinfo.calibration_step=0;
							IPM_INPUT_DISABLE;

							currentinfo.calibration_times=0;//jiao zhun duo shao ci,gu zhang pan duan....

					}
					

			} 
			else if(motor_starter.starting_step==2)
			{
				//pwm ying xiang jiao zhun jing du , suo yi zai kai zhe pwm de qing kuang xia, chong xin jiao zhun...
				Isensor_Zero_Calibration(50,20,8);
				if(currentinfo.calibration_finish>0)
				{
					if(controllermode.sys_Fault==0)
					{

						motor_starter.accelerate_start_time1=globaltime;

						hall2speedinfo.speed_input=0;
						motor_starter.accelerate_speed_curve=0;//you dai tiao zheng
						focinfo_sv.SVPWM_Openloop_switch=0;
						
						PID_current.Out_Up_Limit=1;
						//PID_torque.Out_Up_Limit=1;
						PID_current.INT_Up_Limit=1000;
						//PID_torque.INT_Up_Limit=1000;
	
						//???
						PID_speed.given_value=&(motor_starter.accelerate_speed_curve);
						PID_current.given_value=&(csfb.output_value);
	
						Reset_PID(&(PID_speed));
						Reset_PID(&(PID_current));
						PID_speed.PID_EN=1;
						PID_current.PID_EN=1;

						motor_starter.starting_step=3;
						IPM_INPUT_ENABLE;
					}						
				}
			}
			else if(motor_starter.starting_step==3)
			{
				

				int temp=0;
				PID_current.Out_Up_Limit=current_pid_out_up_limit*temp/1000;
				PID_current.INT_Up_Limit=PID_current.Out_Up_Limit*1000;
				
			
				hall2speedinfo.speed_input=Get_Speed_Given();
					motor_starter.accelerate_start_time1=globaltime;
					motor_starter.accelerate_start_time2=globaltime;
					motor_starter.accelerate_start_time3=globaltime;
				motor_starter.accelerate_start_time4=globaltime;
					motor_starter.accelerate_speed_curve=0;
					hall2speedinfo.speed_input=Get_Speed_Given();
					motor_starter.starting_step=4;
					motor_starter.Max_PWMDUTY_auto_gain=1;

					
					
				 

			}
			else if(motor_starter.starting_step==4)
			{	
		
				//int start=0;
				int end=1000;
				int temp=0;			
				if(Get_Time_Interval_ms(&globaltime,&(motor_starter.accelerate_start_time1))<20000)
				{
					temp=Gain_With_Time(1,end,20000,&(motor_starter.accelerate_start_time1));
				}
				if(temp>1000)
				{
					temp=1000;
				}

				if(temp>=950&&hall2speedinfo.speed<=10)
				{
					Set_Start_Fault();
				}
				
				if(motor_starter.accelerate_speed_curve>hall2speedinfo.speed_input)
				{
					PID_speed.given_value=&(hall2speedinfo.speed_input);
					motor_starter.starting_step=5;
			
				}
				else if(Get_Time_Interval_ms(&globaltime,&(motor_starter.accelerate_start_time2))>ACCELERATE_DELAY_MS)//0225
				{
						if(motor_starter.accelerate_speed_curve>hall2speedinfo.speed_input)
						{
							PID_speed.given_value=&(hall2speedinfo.speed_input);
							motor_starter.starting_step=5;
						//	PID_speed.KI_prescaler=60;
							//useful
							//set_pid bu yao yong tai duo...
							//Set_PID_From_CurState(&PID_speed,hall2speedinfo.speed,currentinfo.current_uvw,PID_speed.ek_1,PID_speed.ek_2);
							//Set_PID_From_CurState(&PID_current,currentinfo.current_uvw,PID_current.P,PID_speed.ek_1,PID_speed.ek_2);
						}
						else if(Get_Time_Interval_ms(&globaltime,&(motor_starter.accelerate_start_time2))>ACCELERATE_DELAY_MS)//0225
						{
							int m=Gain_With_Time(100,hall2speedinfo.speed_input,ACCELERATE_TIME,&(motor_starter.accelerate_start_time4));
							Plus_Time_ms(&(motor_starter.accelerate_start_time2),ACCELERATE_DELAY_MS);
							motor_starter.accelerate_speed_curve=m;
						}

				}
			}
			else if(motor_starter.starting_step==5)
			{
					motor_starter.start_finish=1;
			}		
			Echo_speed();
}
void Motor_start_FOC(void)
{
	if(motor_starter.starting_step==0)
			{
				if(hall2speedinfo.speed_input>0)//2013.11.04
				{
					if((hall2speedinfo.speed==0)&&(hall2speedinfo.duzhuan>0))
					{
						Isensor_Zero_Calibration(20,10,50);
						if(currentinfo.calibration_finish>0)
						{
							focinfo_sv.SVPWM_Openloop_switch=1;
							focinfo_sv.U_dq_given[1]=0;
							focinfo_sv.U_dq_given[0]=0;
							if(GET_JMP1>0)
							{
								controllermode.zheng_fan_zhuan=1;
							}
							else
							{
								controllermode.zheng_fan_zhuan=2;
							}
							TIM_Cmd(TIM1,ENABLE);
							TIM_CtrlPWMOutputs(TIM1, ENABLE);//guan bu liao?????
							if(controllermode.sys_Fault==0&&Get_Speed_Given()>0)
							{
								IPM_INPUT_ENABLE;
								motor_starter.ipm_charging_step=0;
								motor_starter.starting_step=1;
								
							}
						}
				
					}
				}
				else //Speedinput<=0
				{
					if(IS_IPM_INPUT_ENABLE)
					{
						IPM_INPUT_DISABLE;
					}
				}
			}
			else if(motor_starter.starting_step==1)
			{
					IPM_Charge();
					if(motor_starter.ipm_charging_step>=5)
					{
							focinfo_sv.U_dq_given[1]=0;
							focinfo_sv.U_dq_given[0]=0;
							
							PID_torque.Out_Up_Limit=0;
						
							Reset_PID(&PID_speed);
							Reset_PID(&PID_torque);
							Reset_PID(&PID_id);
		

							rotor_angle_est.EST_EN=2;
							rotor_angle_est.hall_COM_signal=0;
							rotor_angle_est.est_count=0;
							rotor_angle_est.last_count=0;
							motor_starter.starting_step=2;
							TIM_SelectOCxM(TIM1, TIM_Channel_1, PWMMODE_TIM1 );
							TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable);
							TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Enable);
							TIM_SelectOCxM(TIM1, TIM_Channel_2, PWMMODE_TIM1 );
							TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Enable);
							TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Enable);
							TIM_SelectOCxM(TIM1, TIM_Channel_3, PWMMODE_TIM1 );
							TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Enable);
							TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Enable);
							TIM_GenerateEvent(TIM1,TIM_EventSource_COM);
							hall2speedinfo.cur_Hall_state=Hall_filter();
							motor_starter.accelerate_start_time1=globaltime;
							currentinfo.calibration_step=0;
							IPM_INPUT_DISABLE;
							focinfo_sv.U_dq_given[0]=0;	
							focinfo_sv.U_dq_given[1]=0;
							currentinfo.calibration_times=0;//jiao zhun duo shao ci,gu zhang pan duan....
	
					}
					
			} 
			else if(motor_starter.starting_step==2)
			{
				//pwm ying xiang jiao zhun jing du , suo yi zai kai zhe pwm de qing kuang xia, chong xin jiao zhun...
				Isensor_Zero_Calibration(50,20,8);
				if(currentinfo.calibration_finish>0)
				{
					if(controllermode.sys_Fault==0)
					{

						motor_starter.accelerate_start_time1=globaltime;
						rotor_angle_est.EST_EN=2;
						focinfo_sv.U_dq_given[1]=0;
						focinfo_sv.U_dq_given[0]=0;	
						focinfo_sv.I_dq_given[1]=0;	
						focinfo_sv.I_dq_given[0]=0;	
						hall2speedinfo.speed_input=0;
						motor_starter.accelerate_speed_curve=0;//100;//you dai tiao zheng
						focinfo_sv.SVPWM_Openloop_switch=0;
						
						//PID_current.Out_Up_Limit=1;
						PID_torque.Out_Up_Limit=1;
						//PID_current.INT_Up_Limit=1000;
						PID_torque.INT_Up_Limit=1000;
	
						//???
						PID_speed.given_value=&(motor_starter.accelerate_speed_curve);
						PID_id.given_value=&(focinfo_sv.I_dq_given[0]);
						PID_torque.given_value=&(focinfo_sv.I_dq_given[1]);

						//PID_speed->given_value=0;
						//PID_id->given_value=0;
						//PID_torque->given_value=0;
						Reset_PID(&(PID_id));
						Reset_PID(&(PID_torque));
						Reset_PID(&(PID_speed));
						PID_id.PID_EN=1;
						PID_torque.PID_EN=1;
						PID_speed.PID_EN=1;
						motor_starter.accelerate_speed_curve=0;
						motor_starter.starting_step=3;
						IPM_INPUT_ENABLE;
					}						
				}
			}
			else if(motor_starter.starting_step==3)
			{
				
	
				int temp=0;

				PID_torque.Out_Up_Limit=torque_pid_out_up_limit*temp/1000;				
				PID_torque.INT_Up_Limit=PID_torque.Out_Up_Limit*1000;

				motor_starter.accelerate_speed_curve=0;//bi xu shi 0,fou ze ji fen bao he!!!
				
				hall2speedinfo.speed_input=Get_Speed_Given();
					motor_starter.accelerate_start_time1=globaltime;
					motor_starter.accelerate_start_time2=globaltime;
					motor_starter.accelerate_start_time3=globaltime;
				motor_starter.accelerate_start_time4=globaltime;
				motor_starter.starting_step=4;
				motor_starter.Max_PWMDUTY_auto_gain=1;

			}
			else if(motor_starter.starting_step==4)
			{	
				
				int start=0;
				int end=1000;
				int temp=0;			
				if(Get_Time_Interval_ms(&globaltime,&(motor_starter.accelerate_start_time1))<20000)
				{
					temp=Gain_With_Time(start,end,20000,&(motor_starter.accelerate_start_time1));
				}
				if(temp>1000)
				{
					temp=1000;
				}
				//motor_starter.duty_limit_Base=temp;

				if(temp>=950&&hall2speedinfo.speed<10)
				{
					Set_Start_Fault();
				}
		
				if(motor_starter.accelerate_speed_curve>hall2speedinfo.speed_input)
				{
					PID_speed.given_value=&(hall2speedinfo.speed_input);
					motor_starter.starting_step=5;
				//	PID_speed.KI_prescaler=60;
					//useful
					//set_pid bu yao yong tai duo...
					//Set_PID_From_CurState(&PID_speed,hall2speedinfo.speed,currentinfo.current_uvw,PID_speed.ek_1,PID_speed.ek_2);
					//Set_PID_From_CurState(&PID_current,currentinfo.current_uvw,PID_current.P,PID_speed.ek_1,PID_speed.ek_2);
				}
				else if(Get_Time_Interval_ms(&globaltime,&(motor_starter.accelerate_start_time2))>ACCELERATE_DELAY_MS)//0225
				{
					int m=Gain_With_Time(100,hall2speedinfo.speed_input,ACCELERATE_TIME,&(motor_starter.accelerate_start_time4));
					Plus_Time_ms(&(motor_starter.accelerate_start_time2),ACCELERATE_DELAY_MS);
					motor_starter.accelerate_speed_curve=m;
				}
			}
			else if(motor_starter.starting_step==5)
			{
					motor_starter.start_finish=1;
			}		
			Echo_speed();
}
void Motor_Start(void)
{

	if(Frame_sm.last_state!=1)
	{
		Init_Motor_start();
	}
	else
	{
			if(controllermode.state_square1_foc0==1)
			{
				Motor_start_SQUARE();
			}
			else
			{
				Motor_start_FOC();
			}
	}
#ifdef DAC1_START_STEP
	DAC_outputdata(motor_starter.starting_step*700);
#endif
}
void Isensor_Zero_Calibration(int sample_times,int test_times,int error)
{	
	if(currentinfo.calibration_step==0)
	{
		currentinfo.calibration_times++;
		if(currentinfo.calibration_times>100)
		{
			Set_Isensor_Fault();
		}
		TIM_ITConfig(TIM1,TIM_IT_CC4,DISABLE);
		currentinfo.calibration_start_time=globaltime;
		currentinfo.next_sample_time=globaltime;
		Plus_Time_ms(&(currentinfo.next_sample_time),1);
		currentinfo.calibration_count=0;
		currentinfo.calibration_step=1;
		currentinfo.sample_data_u_calibration=0;
		currentinfo.sample_data_v_calibration=0;
		currentinfo.calibration_finish=0;

	}
	else if(currentinfo.calibration_step==1)
	{
		if(Get_Time_Interval_ms(&(globaltime),&(currentinfo.next_sample_time))>=0)
		{
			int i=0;
			ADC_SoftwareStartConvCmd(ADC1,ENABLE);
			for(i=0;i<BUFFER_CURRENT;i++)
			{
				currentinfo.sample_data_u_calibration+=(ADVal_current_UV[i]>>16);
				currentinfo.sample_data_v_calibration+=(ADVal_current_UV[i]&0xffff);
			}
			currentinfo.calibration_count++;
			Plus_Time_ms(&(currentinfo.next_sample_time),1);
		}
		if(currentinfo.calibration_count>sample_times)
		{
			currentinfo.AD_zero_current_u=(currentinfo.sample_data_u_calibration/currentinfo.calibration_count);
			if(currentinfo.sample_data_u_calibration%currentinfo.calibration_count>=currentinfo.calibration_count/2)
			{
				currentinfo.AD_zero_current_u++;
			}
			currentinfo.AD_zero_current_v=(currentinfo.sample_data_v_calibration/currentinfo.calibration_count);
			if(currentinfo.sample_data_v_calibration%currentinfo.calibration_count>=currentinfo.calibration_count/2)
			{
				currentinfo.AD_zero_current_v++;
			}
			currentinfo.check_interval_time=globaltime;
			Plus_Time_ms(&(currentinfo.check_interval_time),50);
			currentinfo.calibration_count=0;
			currentinfo.error_u=0;
			currentinfo.error_v=0;
			currentinfo.calibration_step=2;
			
		}
		
	}
	else if(currentinfo.calibration_step==2)
	{
		if(Get_Time_Interval_ms(&(globaltime),&(currentinfo.check_interval_time))>0)
		{
			if(currentinfo.calibration_count++<=test_times)
			{
				int u,v,w;
				ADC_SoftwareStartConvCmd(ADC1,ENABLE);
				Get_Current_UVW(&u,&v,&w);
				currentinfo.error_u+=u;
				currentinfo.error_v+=v;
				Plus_Time_ms(&(currentinfo.check_interval_time),1);
			}
			else
			{

					if(currentinfo.error_u<error
						&&currentinfo.error_u>-error
						&&currentinfo.error_v<error
						&&currentinfo.error_v>-error
						)
					{
						if(currentinfo.AD_zero_current_u>5400&&currentinfo.AD_zero_current_u<6300
							&&currentinfo.AD_zero_current_v>5400&&currentinfo.AD_zero_current_v<6300)
						{
							TIM_ITConfig(TIM1,TIM_IT_CC4,ENABLE);
							currentinfo.calibration_step=3;
							currentinfo.calibration_finish=1;
						}
						
					}
					
					
					if(currentinfo.calibration_finish!=1)
					{
						currentinfo.calibration_step=0;
					}
			}		
		}
	}
}
int Get_Speed_Given(void)
{
	int sum=0,i;
	int speed_g;
	for(i=0;i<BUFFER_DIANWEIQI;i++)
	{
			sum+=ADVal_GeiDing[i];
	}
	sum=sum/BUFFER_DIANWEIQI;
	//600 0
	//3083 1500
	
	if(sum<600) 
	{
		sum=0;
		speed_g=sum;
	}
	else if(sum>=650&&sum<=750)
	{
		sum=MIN_GIVEN_SPEED;
		speed_g=sum;
	}
	else if(sum<650)
	{
		sum=hall2speedinfo.speed_input;
		speed_g=sum;
	}
	else if(sum>3000)
	{
		sum=3000;
		speed_g=(sum-750)*SPEED_GIVEN_WIDTH/2350+MIN_GIVEN_SPEED;
	}
	else
	{
			speed_g=(sum-750)*SPEED_GIVEN_WIDTH/2350+MIN_GIVEN_SPEED;
	}
	
	return speed_g;
}	

void PID_calculator(struct typePID_Def * PID_object,int feedback)
{	
	if(PID_object->PID_EN>0)
	{
		int temp;
		PID_object->ek_2=PID_object->ek_1;
		PID_object->ek_1=PID_object->ek;
		PID_object->ek=(*(PID_object->given_value)-feedback);
		
		temp=((PID_object->Ki*(PID_object->ek))*1000/(PID_object->KI_prescaler));
		PID_object->IntContainer+=temp;
		if(PID_object->IntContainer>PID_object->INT_Up_Limit )
		{
			PID_object->IntContainer-=temp;
			if(PID_object->IntContainer>PID_object->INT_Up_Limit )
			{
				PID_object->IntContainer=PID_object->INT_Up_Limit;//fang zhi error
			}
		}
		if(PID_object->IntContainer<PID_object->INT_Down_Limit )
		{
			PID_object->IntContainer-=temp;
			if(PID_object->IntContainer<PID_object->INT_Down_Limit )
			{
				PID_object->IntContainer=PID_object->INT_Down_Limit;//fang zhi error
			}
		}
		PID_object->P=(PID_object->Kp)*(PID_object->ek)/(PID_object->KP_prescaler);
		PID_object->P+=(PID_object->IntContainer)/1000;
		PID_object->P+=((PID_object->Kd)*((PID_object->ek)-(PID_object->ek_1))/(PID_object->KD_prescaler));
		if((PID_object->P)>(PID_object->Out_Up_Limit))
		{
				PID_object->P=PID_object->Out_Up_Limit;
		}
		if((PID_object->P)<(PID_object->Out_Down_Limit))
		{
				PID_object->P=PID_object->Out_Down_Limit;
		}
	}

}
 
int Get_Current_UVW(int *u,int *v,int *w)
{
	int i;
	int iu=0,iv=0,iw=0;
	int c_uvw=0;
	int databuffer[BUFFER_CURRENT];

	/***********OC JUDGEMENT**************/

	for(i=0;i<BUFFER_CURRENT;i++)
	{
			databuffer[i]=ADVal_current_UV[i];
	}
	for(i=0;i<BUFFER_CURRENT;i++)
	{
		iv+=databuffer[i]&0xffff;
		iu+=databuffer[i]>>16;
	}
	iu-=currentinfo.AD_zero_current_u;
	iv-=currentinfo.AD_zero_current_v;
	iu-=currentinfo.offset_zero_current_u;
	iv-=currentinfo.offset_zero_current_v;
	iw=0-(iu+iv);
	

	if(iu<0)
	{
		c_uvw+=(0-iu);
	}
	else
	{
		c_uvw+=iu;
	}
	if(iv<0)
	{
		c_uvw+=(0-iv);
	}
	else
	{
		c_uvw+=iv;
	}
	if(iw<0)
	{
		c_uvw+=(0-iw);
	}
	else
	{
		c_uvw+=iw;
	}

	
	/*************************************/
	*u=iu;
	*v=iv;
	*w=iw;
	return c_uvw;
}

int Get_Pre_Hall_State(int cur_hall_state)
{
		int ans=0;
		if(controllermode.zheng_fan_zhuan==1)
		{
			//assume:zheng:1 5 4 6 2 3
			
			switch(cur_hall_state)//ji suan xia yi ge zhuang tai, er bu shi dang qian de zhuang tai
			{
				
				case 1://HU=0 HV=0 HW=1
				 //Wp Vn 2
					ans= 5;
					break;
				case 2://HU=0 HV=1 HW=0
					 //Un Vp 4
					ans= 3;
					break;
				case 3://HU=0 HV=1 HW=1
					 //Un Wp 3
					ans= 1;	
					break;
				case 4://HU=1 HV=0 HW=0
					 //UP Wn 6
					ans= 6;
					break;
				case 5://HU=1 HV=0 HW=1
					 //UP Vn 1
					ans= 4;
					break;
				case 6://HU=1 HV=1 HW=0
					 //Vp Wn 5
					ans= 2;
					break;
				default:
					faultinfo.hall=1;
					error(5);
			}
				//error
		}
		else
		{
			//fan:4 5 1 3 2 6
			 
				switch(cur_hall_state)//ji suan xia yi ge zhuang tai, er bu shi dang qian de zhuang tai
				{
					
					case 1://HU=0 HV=0 HW=1
					 //Wp Vn 2
						ans= 3;
						break;
					case 2://HU=0 HV=1 HW=0
						 //Un Vp 4
						ans= 6;
						break;
					case 3://HU=0 HV=1 HW=1
						 //Un Wp 3
						ans= 2;
						break;
					case 4://HU=1 HV=0 HW=0
						 //UP Wn 6
						ans= 5;
						break;
					case 5://HU=1 HV=0 HW=1
						 //UP Vn 1
						ans= 1;
						break;
					case 6://HU=1 HV=1 HW=0
						 //Vp Wn 5
						ans= 4;
						break;
					default:
						faultinfo.hall=1;
						error(5);
				}
				
			}
			return ans;
}
void Update_Speed(void)
{
	int time_cha,time_quan,speed_t;
	int delta_time;
	time_cha=Get_Time_Interval_us(&globaltime,&(hall2speedinfo.last_com_time));
	
	delta_time=(SysTick->VAL*100)/(SysTick->LOAD);
	time_cha+=delta_time;
	time_quan=MAGNET_POLE_PAIR*6*time_cha;
	if(time_quan<=1)
	{
		speed_t=hall2speedinfo.speed;//bu bian...
	}
	else
	{
		speed_t=60000000/time_quan;
	}
	filter(&rc_hall_speed,speed_t);//20140313
	hall2speedinfo.speed=rc_hall_speed.out_data;
	hall2speedinfo.last_com_time=globaltime;
	hall2speedinfo.last_com_time.time_us+=delta_time;
	if(hall2speedinfo.last_com_time.time_us>=1000)
	{
		hall2speedinfo.last_com_time.time_us-=1000;
		hall2speedinfo.last_com_time.time_ms+=1;
	}
	if(hall2speedinfo.last_com_time.time_ms>=1000)
	{
		hall2speedinfo.last_com_time.time_s+=1;
		hall2speedinfo.last_com_time.time_ms-=1000;
	}
	filter(&rc_echo_speed,hall2speedinfo.speed);
#ifdef DAC1_SPEED
	DAC_outputdata(hall2speedinfo.speed*2);
#endif
#ifdef DAC2_SPEED
	DAC_outputdata2(hall2speedinfo.speed*2);
#endif

}
void Change_PWM_state_NOW_HPWM_LON(int curstate)
{
	if(controllermode.zheng_fan_zhuan==1)
	{
			switch(curstate)
			{
					case 5://Up-Vn
					{
						//channel 3
						 	TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_ForcedAction_InActive);
						TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Disable);
						TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Disable);
						//channel 1
						TIM_SelectOCxM(TIM1, TIM_Channel_1, PWMMODE_TIM1);
						TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable);
						TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Disable);
						//channel 2
						TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_ForcedAction_Active );//TIM_ForcedOC1Config
						TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Disable);
						TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Enable);
						break;		
					}
					case 1://Wp-Vn
					{
						//channel 2
						TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_ForcedAction_Active);
						TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Disable);
						TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Enable);
						
						TIM_SelectOCxM(TIM1, TIM_Channel_3,PWMMODE_TIM1 );//?????
						TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Enable);
						TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Disable);
						
						TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_ForcedAction_InActive);
						TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Disable);
						TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Disable);
						break;
					}
					case 3://Wp-Un
					{
			 
						TIM_SelectOCxM(TIM1, TIM_Channel_3, PWMMODE_TIM1);
						TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Enable);
						TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Disable);
				 
							TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_ForcedAction_InActive);
						TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Disable);
						TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Disable);
				 
						TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_ForcedAction_Active);
						TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Disable);
						TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Enable);
						break;
					}
					case 2://Vp-Un
					{

							TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_ForcedAction_InActive);
						TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Disable);
						TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Disable);

						TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_ForcedAction_Active);
						TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Disable);
						TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Enable);

						TIM_SelectOCxM(TIM1, TIM_Channel_2,PWMMODE_TIM1 );
						TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Enable);
						TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Disable);
						break;
					}
					case 6://Vp-Wn
					{
						TIM_SelectOCxM(TIM1, TIM_Channel_3,TIM_ForcedAction_Active );
						TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Disable);
						TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Enable);

							TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_ForcedAction_InActive);
						TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Disable);
						TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Disable);

						TIM_SelectOCxM(TIM1, TIM_Channel_2, PWMMODE_TIM1);
						TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Enable);
						TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Disable);
						break;
			 
					}
					case 4://Up-Wn
					{
						TIM_SelectOCxM(TIM1, TIM_Channel_1, PWMMODE_TIM1);
						TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable);
						TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Disable);

						TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_ForcedAction_Active);
						TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Disable);
						TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Enable);

							TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_ForcedAction_InActive);
						TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Disable);
						TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Disable);
						break;
					}
					default:
					{
						//error
							TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_ForcedAction_InActive);
						TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Disable);
						TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Disable);
						
							TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_ForcedAction_InActive);
						TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Disable);
						TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Disable);
						
							TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_ForcedAction_InActive);
						TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Disable);
						TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Disable);
						break;
					}
				}
	}
	else
	{
				switch(curstate)
				{
					case 5://Un-Vp
					{
						//channel 3
						TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_ForcedAction_InActive );
						TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Disable);
						TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Disable);
						//channel 1
						TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_ForcedAction_Active );
						TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Disable);
						TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Enable);
						//channel 2
						
						TIM_SelectOCxM(TIM1, TIM_Channel_2, PWMMODE_TIM1);
						TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Enable);
						TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Disable);
						break;		
					}
					case 1://Wn-Vp
					{
						//channel 2
						TIM_SelectOCxM(TIM1, TIM_Channel_2,PWMMODE_TIM1 );//?????
						TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Enable);
						TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Disable);

						
						TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_ForcedAction_Active);
						TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Disable);
						TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Enable);

						TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_ForcedAction_InActive );
						TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Disable);
						TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Disable);
						break;
					}
					case 3://Wn-Up
					{
			 
						TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_ForcedAction_Active);
						TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Disable);
						TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Enable);
				 
						TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_ForcedAction_InActive );
						TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Disable);
						TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Disable);
				 
						
						TIM_SelectOCxM(TIM1, TIM_Channel_1, PWMMODE_TIM1);
						TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable);
						TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Disable);
						break;
					}
					case 2://Vn-Up
					{

						TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_ForcedAction_InActive );
						TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Disable);
						TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Disable);

						TIM_SelectOCxM(TIM1, TIM_Channel_1,PWMMODE_TIM1 );
						TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable);
						TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Disable);

						TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_ForcedAction_Active);
						TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Disable);
						TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Enable);
						break;
					}
					case 6://Vn-Wp
					{
						TIM_SelectOCxM(TIM1, TIM_Channel_3, PWMMODE_TIM1);
						TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Enable);
						TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Disable);

						TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_ForcedAction_InActive );
						TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Disable);
						TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Disable);

						TIM_SelectOCxM(TIM1, TIM_Channel_2,TIM_ForcedAction_Active );
						TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Disable);
						TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Enable);
						break;
			 
					}
					case 4://Un-Wp
					{
						TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_ForcedAction_Active);
						TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Disable);
						TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Enable);

						TIM_SelectOCxM(TIM1, TIM_Channel_3, PWMMODE_TIM1);
						TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Enable);
						TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Disable);

						TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_ForcedAction_InActive );
						TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Disable);
						TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Disable);
						break;
					}
					default:
					{
						//error
						TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_ForcedAction_InActive );
						TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Disable);
						TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Disable);
						TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_ForcedAction_InActive );
						TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Disable);
						TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Disable);
						TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_ForcedAction_InActive );
						TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Disable);
						TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Disable);
						break;
					}
				}
		}
}
void Change_PWM_state_PWM_ON(int hall_state)
{
	Change_PWM_state_NOW_HPWM_LON(hall_state);
	     /*
	if(controllermode.zheng_fan_zhuan==1)
	{

				switch(hall_state)
				{
					case 5://Up-Vn
					{
						//channel 3
						TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_ForcedAction_InActive );
						TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Disable);
						TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Disable);
						//channel 1
						TIM_SelectOCxM(TIM1, TIM_Channel_1, PWMMODE_TIM1);
						TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable);
						TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Disable);
						//channel 2
						TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_ForcedAction_Active );
						TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Disable);
						TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Enable);
						break;		
					}
					case 1://Wp-Vn
					{
						//channel 2
						TIM_SelectOCxM(TIM1, TIM_Channel_2,PWMMODE_TIM1 );//?????
						TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Disable);
						TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Enable);

						TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_ForcedAction_Active);
						TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Enable);
						TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Disable);

						TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_ForcedAction_InActive );
						TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Disable);
						TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Disable);
						break;
					}
					case 3://Wp-Un-------------ERROR!!!!!
					{
			 
						TIM_SelectOCxM(TIM1, TIM_Channel_3, PWMMODE_TIM1);
						TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Enable);
						TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Disable);
				 
						TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_ForcedAction_InActive );
						TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Disable);
						TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Disable);
				 
						TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_ForcedAction_Active);
						TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Disable);
						TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Enable);
						break;
					}
					case 2://Vp-Un
					{

						TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_ForcedAction_InActive );
						TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Disable);
						TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Disable);

						TIM_SelectOCxM(TIM1, TIM_Channel_1,PWMMODE_TIM1 );
						TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Disable);
						TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Enable);

						TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_ForcedAction_Active);
						TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Enable);
						TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Disable);
						break;
					}
					case 6://Vp-Wn
					{
						TIM_SelectOCxM(TIM1, TIM_Channel_3,TIM_ForcedAction_Active );
						TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Disable);
						TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Enable);

						TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_ForcedAction_InActive );
						TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Disable);
						TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Disable);

						TIM_SelectOCxM(TIM1, TIM_Channel_2, PWMMODE_TIM1);
						TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Enable);
						TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Disable);
						break;
			 
					}
					case 4://Up-Wn
					{
						TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_ForcedAction_Active);
						TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable);
						TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Disable);

						TIM_SelectOCxM(TIM1, TIM_Channel_3, PWMMODE_TIM1);
						TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Disable);
						TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Enable);

						TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_ForcedAction_InActive );
						TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Disable);
						TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Disable);
						break;
					}
					default:
					{
						//error
						TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_ForcedAction_InActive );
						TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Disable);
						TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Disable);
						TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_ForcedAction_InActive );
						TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Disable);
						TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Disable);
						TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_ForcedAction_InActive );
						TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Disable);
						TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Disable);
						break;
					}
				}
	}
	else
	{
				switch(hall_state)
				{
					case 5://Un-Vp
					{
						//channel 3
						TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_ForcedAction_InActive );
						TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Disable);
						TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Disable);
						//channel 1
						TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_ForcedAction_Active );
						TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Disable);
						TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Enable);
						//channel 2
						
						TIM_SelectOCxM(TIM1, TIM_Channel_2, PWMMODE_TIM1);
						TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Enable);
						TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Disable);
						break;		
					}
					case 1://Wn-Vp
					{
						//channel 2
						TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_ForcedAction_Active);
						TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Enable);
						TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Disable);

						
						TIM_SelectOCxM(TIM1, TIM_Channel_3,PWMMODE_TIM1 );//?????
						TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Disable);
						TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Enable);

						TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_ForcedAction_InActive );
						TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Disable);
						TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Disable);
						break;
					}
					case 3://Wn-Up
					{
			 
						TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_ForcedAction_Active);
						TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Disable);
						TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Enable);
				 
						TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_ForcedAction_InActive );
						TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Disable);
						TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Disable);
				 
						
						TIM_SelectOCxM(TIM1, TIM_Channel_1, PWMMODE_TIM1);
						TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable);
						TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Disable);
						break;
					}
					case 2://Vn-Up
					{

						TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_ForcedAction_InActive );
						TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Disable);
						TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Disable);

						TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_ForcedAction_Active);
						TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable);
						TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Disable);

						TIM_SelectOCxM(TIM1, TIM_Channel_2,PWMMODE_TIM1 );
						TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Disable);
						TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Enable);
						break;
					}
					case 6://Vn-Wp
					{
						TIM_SelectOCxM(TIM1, TIM_Channel_3, PWMMODE_TIM1);
						TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Enable);
						TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Disable);

						TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_ForcedAction_InActive );
						TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Disable);
						TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Disable);

						TIM_SelectOCxM(TIM1, TIM_Channel_2,TIM_ForcedAction_Active );
						TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Disable);
						TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Enable);
						break;
			 
					}
					case 4://Un-Wp
					{
						TIM_SelectOCxM(TIM1, TIM_Channel_1, PWMMODE_TIM1);
						TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Disable);
						TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Enable);

						TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_ForcedAction_Active);
						TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Enable);
						TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Disable);

						TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_ForcedAction_InActive );
						TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Disable);
						TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Disable);
						break;
					}
					default:
					{
						//error
						TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_ForcedAction_InActive );
						TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Disable);
						TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Disable);
						TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_ForcedAction_InActive );
						TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Disable);
						TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Disable);
						
						TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_ForcedAction_InActive );
						TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Disable);
						TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Disable);
						break;
					}
				}
	}
	*/
	//20140121
}
void SYS_Reinit()
{
	IPM_INPUT_DISABLE;
	TIM_Cmd(TIM1,DISABLE);
	TIM_Cmd(TIM2,DISABLE);
	TIM_Cmd(TIM3,DISABLE);
	controllermode.sys_Fault=0;

	ReInit_variables();
	ADC_SoftwareStartConvCmd(ADC1,ENABLE);

	TIM_SetCompare1(TIM1,pwm_generator.pwm_period-1);
	TIM_SetCompare2(TIM1,pwm_generator.pwm_period-1);
	TIM_SetCompare3(TIM1,pwm_generator.pwm_period-1);
	TIM_Cmd(TIM3,ENABLE);	
	TIM_Cmd(TIM2,ENABLE);
	//TIM_Cmd(TIM1,ENABLE);
	
}
int Gain_With_Time(int min,int max,int time_interval_ms,struct Time_Def * start_time)
{
		int t_ms=Get_Time_Interval_ms(&globaltime,start_time);
		int cha=max-min;
		int ans;
	if(t_ms>100000) t_ms=100000;//fang zhi yi chu...
	ans=cha*t_ms/time_interval_ms+min;
	if(ans>max)
	{
			ans=max;
	}
	if(ans<min)
	{
			ans=min;
	}
	return ans;
}
void Update_state()
{
	int decision=0;
	hall2speedinfo.speed_input=Get_Speed_Given();
	Frame_sm.last_state=Frame_sm.cur_state;
	//20140311
	if((Frame_sm.cur_state==1)&&(hall2speedinfo.speed<=100))
	{
		csfb.gate_value=2000;
	}
	else
	{
		csfb.gate_value=2000;
	}
	


	
	if(controllermode.sys_Fault>0)
	{
			if(Frame_sm.cur_state==1)
			{
					decision=3;
			}
			else if(Frame_sm.cur_state==2)
			{
					decision=3;
			}
			else if(Frame_sm.cur_state==3)
			{
					decision=3;
			}
	}
	else
	{
			if(Frame_sm.cur_state==1)
			{
					if(motor_starter.start_finish>0)
					{
							decision=2;
					}
			}
			else if(Frame_sm.cur_state==2)
			{
					if((hall2speedinfo.duzhuan>0)&&(hall2speedinfo.speed<=0)&&(hall2speedinfo.speed_input<=0))
					{
							decision=1;
					}
			}
			else if(Frame_sm.cur_state==3)
			{
					if(faultinfo.fault_reset>0)
					{
						decision=1;//fault is cleared by user,restart the motor...
					}
			}
			
	}
	if(decision>0)
	{	
		Frame_sm.cur_state=decision;
	}
	else
	{
		Frame_sm.cur_state=Frame_sm.cur_state;
	}
}
int observer;
int observer2;
struct typePID_Def  PID_temp;
void Motor_Running(void)
{
		if(Frame_sm.last_state!=Frame_sm.cur_state)
		{
			faultinfo.next_fault_check_time=globaltime;
			Plus_Time_ms(&(faultinfo.next_fault_check_time),50);
			if(controllermode.sys_Fault==0)
			{
				//1124
				//IPM_INPUT_ENABLE;
				//TIM_Cmd(TIM1,ENABLE);
				//TIM_Cmd(TIM2,ENABLE);
				//TIM_Cmd(TIM3,ENABLE);
			}
		}
		//============fault check==============/
		Fault_Regular_Scan();
		Echo_speed();
		
//		PID_temp=PID_speed;
//		PID_temp.ek=0;
//		PID_temp.ek_1=0;
//		PID_temp.ek_2=0;
//		PID_temp.P=0;
//		PID_temp.IntContainer=0;
//		Set_PID_From_CurState(&PID_temp,hall2speedinfo.speed,PID_speed.P,0,0);
//		observer=PID_temp.IntContainer-PID_speed.IntContainer;
//		observer2=PID_temp.P-PID_speed.P;
}
void echo_fault(void)
{
		if(faultinfo.oc>0)
		{
			disp_data=10000+faultinfo.oc_source;
		}
		else if(faultinfo.ipmfo>0)
		{
			disp_data=20000;
		}
		else if(faultinfo.hall>0)
		{
			disp_data=30000;
		}
		else if(faultinfo.start_fault>0)
		{
			disp_data=40000;
		}
		else if(faultinfo.overload>0)
		{
			disp_data=50000;
		}
		else if(faultinfo.i_sensor>0)
		{
			disp_data=60000;
		}
		else if(faultinfo.ting_ji>0)
		{
			disp_data=70000;
		}
		else if(faultinfo.over_speed>900)
		{
			disp_data=80000;
		}
		else
		{
			disp_data=99999;
		}
		Echo_Make_Number();
		Com2TM1638();
}

void FaultProcess(void)
{
		if(Frame_sm.last_state!=Frame_sm.cur_state)
		{
			IPM_INPUT_DISABLE;
			TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_ForcedAction_InActive );
			TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Disable);
			TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Disable);
			
			TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_ForcedAction_InActive );
			TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Disable);
			TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Disable);
			
			TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_ForcedAction_InActive );
			TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Disable);
			TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Disable);
			TIM_GenerateEvent(TIM1,TIM_EventSource_COM);
			faultinfo.fault_reset=0;
			controllermode.restarting=0;
			echo_fault();
		
		}
		else
		{
			if(IS_IPM_INPUT_ENABLE)
			{
					IPM_INPUT_DISABLE;
			}
			if(Get_Speed_Given()<=0&&controllermode.restarting==0)
			{
					SYS_Reinit();
					controllermode.restarting=1;

			}
			if(controllermode.restarting==1)
			{
					//Isensor_Zero_Calibration();
				//	if(currentinfo.calibration_finish>0)
				//	{
						controllermode.sys_Fault=0;
						faultinfo.fault_reset=1;
				//	}
			}
			
			
		}
}

void filter(struct RC_FILTER *obj,int input_value)
{
	int temp;
	obj->in_data=input_value;
	temp=(obj->out_data)*(obj->inertia_1000)
	+(obj->in_data*(1000-obj->inertia_1000));
	temp=temp/1000;
	obj->out_data=temp;
}
void CSFB_calculate(void)
{
	currentinfo.current_uvw=Get_Current_UVW(&(currentinfo.current_U),&(currentinfo.current_V),&(currentinfo.current_W));
	filter(&rc_current_uvw,currentinfo.current_uvw);
	//--------------oc---------------------------------
	if(rc_current_uvw.out_data>OC_VALUE )
	{
		EXTI_GenerateSWInterrupt(EXTI_Line1);
	}
	//--------------oc level 2:20A 50ms------------------
	if(rc_current_uvw.out_data>OC_LEVEL2_UPGATE)
	{
		if(faultinfo.oclevel2_flag==0)
		{
			faultinfo.oclevel2_start_time=globaltime;
			faultinfo.oclevel2_flag=1;
		}
	}
	else if(rc_current_uvw.out_data<OC_LEVEL2_DOWNGATE)
	{
		faultinfo.oclevel2_flag=0;
	}
	else
	{
		;
	}
	
	if(faultinfo.oclevel2_flag>0)
	{
		if(Get_Time_Interval_ms(&globaltime,&(faultinfo.oclevel2_start_time))>50)
		{
			faultinfo.oclevel2_flag=2;
			EXTI_GenerateSWInterrupt(EXTI_Line1);
			faultinfo.oclevel2_flag=0;
		}
	}
	
	
	//-------------------overload------------------
	
	
	if(rc_current_uvw.out_data>OVERLOAD_UPGATE)
	{
		if(faultinfo.overload_flag==0)
		{
			faultinfo.overload_start_time=globaltime;
			faultinfo.overload_flag=1;
		}
		else
		{
			if(Get_Time_Interval_ms(&globaltime,&(faultinfo.overload_start_time))>OVERLOAD_TIME_MS)
			{
				Set_Overload_Fault();
			}
		}
	}
	else if(rc_current_uvw.out_data<OVERLOAD_DOWNGATE)
	{
		if(faultinfo.overload_flag>0)
		{
			faultinfo.overload_flag=0;
		}
	}
	else
	{
		if(faultinfo.overload_flag>0)
		{
			if(Get_Time_Interval_ms(&globaltime,&(faultinfo.overload_start_time))>OVERLOAD_TIME_MS)
			{
				Set_Overload_Fault();
			}
		}
	}

	if(csfb.feedback_EN==1)
	{
		if(controllermode.state_square1_foc0==1)
		{
			if(currentinfo.current_uvw>csfb.gate_value)
			{
				csfb.output_value=PID_speed.P-csfb.feedback_K*(currentinfo.current_uvw-csfb.gate_value);
				if(csfb.output_value<PID_speed.Out_Down_Limit)
				{
					csfb.output_value=PID_speed.Out_Down_Limit;
				}
			}
			else
			{
				csfb.output_value=PID_speed.P;
			}
		}
		else//sine wave
		{
			if(currentinfo.current_uvw>csfb.gate_value)
			{
				csfb.output_value=PID_speed.P-csfb.feedback_K2*(currentinfo.current_uvw-csfb.gate_value);
				if(csfb.output_value<PID_speed.Out_Down_Limit)
				{
					csfb.output_value=PID_speed.Out_Down_Limit;
				}
			}
			else
			{
				csfb.output_value=PID_speed.P;
			}
		}
	}
	else
	{
		csfb.output_value=PID_speed.P;
	}
	
	

}

void SQUARE_ENTRANCE(void)
{

	PID_calculator(&PID_current,currentinfo.current_uvw);
	pwm_generator.pwm_value_square=pwm_generator.pwm_period-PID_current.P;
	if(pwm_generator.pwm_value_square>PWM_UPLIMIT)
	{
		pwm_generator.pwm_value_square=PWM_UPLIMIT;
	}
	else if(pwm_generator.pwm_value_square<PWM_DOWNLIMIT)
	{
		pwm_generator.pwm_value_square=PWM_DOWNLIMIT;
	}
	TIM_SetCompare1(TIM1,pwm_generator.pwm_value_square);
	TIM_SetCompare2(TIM1,pwm_generator.pwm_value_square);
	TIM_SetCompare3(TIM1,pwm_generator.pwm_value_square);
		
}

void Echo_Make_Number(void)
{
	signed int i,j;
	int number_table[10]={0x3f,0x06,0x5b,0x4f,0x66,0x6d,0x7d,0x07,0x7f,0x6f};
	int data[9];
	for(i=0;i<8;i++)
	{
		DISP_Buffer[i]=0xFFFF;
	}
	data[0]=0;
	data[7]=0;
	data[8]=0;
	data[1]=(disp_data/100000)%10;
	data[2]=(disp_data/10000)%10;
	data[3]=(disp_data/1000)%10;
	data[4]=(disp_data/100)%10;
	data[5]=(disp_data/10)%10;
	data[6]=disp_data%10;
	
	for(i=0;i<8;i++)
	{
		data[i]=number_table[data[i]];
	}
	for(j=0;j<8;j++)
	{
		for(i=0;i<8;i++)
		{
			if((data[j+1]>>i)%2==0)
			{
				DISP_Buffer[i]&=(0xffff^(1<<j));
			}
			
		}
	}
	
}

void Echo_Make_HOLD()
{
	signed int i,j;
	


	int H=0x76;
	int O=0x5C;
	int L=0x06;
	int D=0x5E;
	int dot=0x80;
//	Uint16 O=0x
	//Common Anode Segment Table 0-9

/*	
	for(i=0;i<6;i++)
	{
		send2[i]=num%10;
		num=num/10;
		if(send2[i]!=0)
		{
			countnum_b=i;
		}
	
	}
*/
	
	//countnum_b=3;
	for(i=0;i<8;i++)
	{
		DISP_Buffer[i]=0xFFFF;
	}
 	for(j=0;j<8;j++)
	{
		for(i=0;i<8;i++)
		{
			if(j==3)
			{
				if((D>>i)%2==0)
					DISP_Buffer[i]&=(0xFFFF^(1<<j));
			}
			else if(j==2)
			{	
				if((L>>i)%2==0)
					DISP_Buffer[i]&=(0xFFFF^(1<<j));
			}
			else if(j==1)
			{	
				if((O>>i)%2==0)
					DISP_Buffer[i]&=(0xFFFF^(1<<j));
			}
			else if(j==0)
			{	
				if((H>>i)%2==0)
					DISP_Buffer[i]&=(0xFFFF^(1<<j));
			}
			else
			{
				if((dot>>i)%2==0)
					DISP_Buffer[i]&=(0xFFFF^(1<<j));
			}


		}
	}
 
 

}

void TM1638_Write_6(int DATA)		 
{
	int i;
	int j=0;
	for(i=0;i<8;i++)
	{
		ECHO_CLK_L;
		if((DATA&0x01)!=0)
			ECHO_DIO_H;
		else
			ECHO_DIO_L;
		DATA>>=1;
		for(j=0;j<10;j++);
		ECHO_CLK_H;
		for(j=0;j<10;j++);
	}
}


void Write_COM_6(int cmd)	 
{
	int j;
	ECHO_STB_L;
	for(j=0;j<10;j++);
	TM1638_Write_6(cmd);
	ECHO_STB_H;
	for(j=0;j<10;j++);
}
void init_TM1638_6(void)
{
	int i, j;
	
	Write_COM_6(0x40);       //地址增加模式，到显存
	ECHO_STB_L;
	for(j=0;j<10;j++);            
	TM1638_Write_6(0xc0);    
	for(j=0;j<10;j++);
	for(i=0;i<16;i++)	    
	{
		TM1638_Write_6(0x00);
		//DELAY_US(1);
	}
	      
	for(j=0;j<10;j++);
	ECHO_STB_H;
	for(j=0;j<10;j++);
	Write_COM_6(0x8a);
	for(j=0;j<10;j++);

}
void Com2TM1638()
{
	int i,j;
	Write_COM_6(0x40);       //地址增加模式，到显存
	ECHO_STB_L;	
	for(j=0;j<10;j++);        
	TM1638_Write_6(0xc0);    
	for(j=0;j<10;j++);
	for(i=0;i<8;i++)	    
	{
		TM1638_Write_6(DISP_Buffer[i]&0xff);
		for(j=0;j<10;j++); 
		TM1638_Write_6(DISP_Buffer[i]>>8);
		for(j=0;j<10;j++); 
	}
	     
	for(j=0;j<10;j++); 
	ECHO_STB_H;
	for(j=0;j<10;j++); 
//	Write_COM_6(0x8a);
//	DELAY_US(1);


}

void Echo_speed(void)
{
	if(Get_Time_Interval_ms(&globaltime,&(controllermode.next_echo_time))>0)
	{
		int temp;
		Plus_Time_ms(&(controllermode.next_echo_time),200);

		//filter(&rc_echo_speed,hall2speedinfo.speed);
		//disp_data=rc_echo_speed.out_data;
		
	//	just for debug
		
		if(PID_current.P>0)
		{
			temp=((PID_current.P)+5)/10;
		}
		else
		{
			temp=998;
		}
		/*temp*=1000;
		if(controllermode.state_square1_foc0==1)
		{
			temp+=(hall2speedinfo.speed_input)/10;
		}
		else
		{
			temp+=(hall2speedinfo.speed_input)/10;
		}
		*/
		disp_data=temp;
		
		Echo_Make_Number();
		Com2TM1638();
	}
}
int Hall_filter(void)
{
	int hall_filter[8]={0,0,0,0,0,0,0,0};
	int i;
	int max=0;
	int hall=0;
	int quit=0;
	//==========hall filter================/
		for(i=0;i<5;i++)
		{
			int a=GET_HALL_UVW;
			hall_filter[a]++;
		//	for(j=0;j<80;j++);
		}
		for(i=1;i<=6;i++)
		{
				if(max<hall_filter[i])
				{
					max=hall_filter[i];
					hall=i;
				}
		}
		if((hall_filter[0]>3)||(hall_filter[7]>3)||(max==0))
		{
			int count_hall=0,sample_hall=0;
			int j;
			for(j=0;j<50;j++)
			{
				sample_hall=GET_HALL_UVW;
				if((sample_hall==0)||(sample_hall==7))
				{
					count_hall++;
				}
				else if(max==0)
				{
					hall=sample_hall;
				}

			}
			if(count_hall>40)
			{
				Set_HALL_Fault();
				quit=1;//0122
			}
		}
		if(quit>0)
		{
			hall=0;
		}
	
		
		return hall;
		
	
}
void Code_Entrance(void)
{

		init_TM1638_6();

		Echo_Make_HOLD();
		Com2TM1638();

		while(globaltime.time_s<TIME_POWER_ON_DELAY);
		RELAY_ON;
		while(globaltime.time_s<TIME_IPM_ENABLE_DELAY);
		
		Frame_sm.cur_state=1;
		Frame_sm.pre_state=1;
		Frame_sm.last_state=0;
		while(1)
		{
			switch(Frame_sm.cur_state)
			{
				case 1://Motor_Start 
					Motor_Start();
					break;
				case 2:
					Motor_Running();
					break;
				case 3:
					FaultProcess();
					break;
			}
			Update_state();
		}
}





//end of file.....
