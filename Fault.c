#include "MyGlobal.h"
#include "defines.h"
#include "Fault.h"
void OC_Process(void)
{
	int i=0;
	int protect_method=0;
	int yuan_zhuang_tai=IS_IPM_INPUT_ENABLE;
	IPM_INPUT_DISABLE;
	LED1_OFF;


	if(controllermode.sys_Fault>0)
	{
		protect_method=2;
		//faultinfo.oc_source=999;//131124
	}
	else
	{
		if(rc_current_uvw.out_data>OC_VALUE)
		{
			protect_method=2;
			faultinfo.oc_source=1;
		}
		else if(faultinfo.oclevel2_flag==2)
		{
			protect_method=2;
			faultinfo.oc_source=2;
		}
		else if(Get_Time_Interval_ms(&globaltime,&faultinfo.last_oc_time)>30)
		{
			faultinfo.oc_counter=1;
			protect_method=1;
		}
		else
		{
			faultinfo.oc_counter++;
			if(faultinfo.oc_counter>=3)
			{
				protect_method=2;
				faultinfo.oc_source=3;
			}
			else
			{
				protect_method=1;
			}
		}
	}
	
	if(protect_method==1)
	{
		int count=0;
		while(IS_PHASE_OC)
		{
			count++;
			for(i=0;i<72;i++);//about 10us
			if(count>100)//about 1ms
			{
				protect_method=2;
				break;
			}
		}
		for(i=0;i<200;i++);
		if(protect_method==1)
		{
			LED1_ON;
			if(yuan_zhuang_tai>0)
			{
				IPM_INPUT_ENABLE;
			}
			else
			{
				IPM_INPUT_DISABLE;
			}
		}
		
	}
	if(protect_method==2)
	{
		LED1_OFF;
		IPM_INPUT_DISABLE;
		faultinfo.oc=1;
		controllermode.sys_Fault=1;
		TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_ForcedAction_InActive );
		TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Disable);
		TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Disable);
	
		TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_ForcedAction_InActive );
		TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Disable);
		TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Disable);
	
		TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_ForcedAction_InActive );
		TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Disable);
		TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Disable);
		TIM_GenerateEvent(TIM1,TIM_EventSource_COM);//CAN this work???
		
		while(IS_PHASE_OC);
		
	}

	
	faultinfo.last_oc_time=globaltime;

}
void IPMFT_Process(void)
{
	int i=0;
	int protect_method=0;
	int yuan_zhuang_tai=IS_IPM_INPUT_ENABLE;
	IPM_INPUT_DISABLE;
	LED2_OFF;
	
	if(controllermode.sys_Fault>0)
	{
		protect_method=2;
	}
	else
	{
		if(Get_Time_Interval_ms(&globaltime,&faultinfo.last_ipmfo_time)>IPM_FO_RECHECK_TIME_DELAY_MS)
		{
			faultinfo.ipmfo_counter=1;
			protect_method=1;

		}
		else
		{
			faultinfo.ipmfo_counter++;
			if(faultinfo.ipmfo_counter>=20)
			{
				protect_method=2;
			}
			else
			{
				protect_method=1;
			}
		}
	}
	
	
	if(protect_method==1)
	{
		while(IS_IPM_FO)
		{
			for(i=0;i<72;i++);
		}
		LED2_ON;
		if(yuan_zhuang_tai>0)
		{
			IPM_INPUT_ENABLE;
		}
		else
		{
			IPM_INPUT_DISABLE;
		}
	}
	else
	{
		LED2_OFF;
		IPM_INPUT_DISABLE;
		controllermode.sys_Fault=1;
		faultinfo.ipmfo=1;
		TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_ForcedAction_InActive );
		TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Disable);
		TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Disable);
	
		TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_ForcedAction_InActive );
		TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Disable);
		TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Disable);
	
		TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_ForcedAction_InActive );
		TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Disable);
		TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Disable);
		TIM_GenerateEvent(TIM1,TIM_EventSource_COM);//CAN this work???
		
		while(IS_IPM_FO);
	}
	
	
	faultinfo.last_ipmfo_time=globaltime;
}
void Set_Isensor_Fault(void)
{

	IPM_INPUT_DISABLE;
//	LED1_OFF;
//	LED2_OFF;
	T3_H;
	controllermode.sys_Fault=1;
	faultinfo.i_sensor=1;
	TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_ForcedAction_InActive );
	TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Disable);
	TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Disable);

	TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_ForcedAction_InActive );
	TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Disable);
	TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Disable);

	TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_ForcedAction_InActive );
	TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Disable);
	TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Disable);
	TIM_GenerateEvent(TIM1,TIM_EventSource_COM);//CAN this work???
}
void Set_HALL_Fault(void)
{
	IPM_INPUT_DISABLE;
//	LED1_OFF;
//	LED2_OFF;
	T1_H;
	controllermode.sys_Fault=1;
	faultinfo.hall=1;
	TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_ForcedAction_InActive );
	TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Disable);
	TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Disable);

	TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_ForcedAction_InActive );
	TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Disable);
	TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Disable);

	TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_ForcedAction_InActive );
	TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Disable);
	TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Disable);
	TIM_GenerateEvent(TIM1,TIM_EventSource_COM);//CAN this work???
}
void Set_Temp_Fault(void)
{
	IPM_INPUT_DISABLE;
	//LED1_OFF;
	//LED2_OFF;
	T2_H;
	controllermode.sys_Fault=1;
	faultinfo.ot=1;
	TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_ForcedAction_InActive );
	TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Disable);
	TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Disable);

	TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_ForcedAction_InActive );
	TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Disable);
	TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Disable);

	TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_ForcedAction_InActive );
	TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Disable);
	TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Disable);
	TIM_GenerateEvent(TIM1,TIM_EventSource_COM);//CAN this work???
}
void Set_Start_Fault(void)
{
	IPM_INPUT_DISABLE;
	//LED1_OFF;
	//LED2_OFF;
	T3_H;
	controllermode.sys_Fault=1;
	faultinfo.start_fault=1;
	TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_ForcedAction_InActive );
	TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Disable);
	TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Disable);

	TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_ForcedAction_InActive );
	TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Disable);
	TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Disable);

	TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_ForcedAction_InActive );
	TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Disable);
	TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Disable);
	TIM_GenerateEvent(TIM1,TIM_EventSource_COM);//CAN this work???
}
void Set_Overload_Fault(void)
{
	IPM_INPUT_DISABLE;
	//LED1_OFF;
	//LED2_OFF;
	T3_H;//i should find another pin...
	controllermode.sys_Fault=1;
	faultinfo.overload=1;
	TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_ForcedAction_InActive );
	TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Disable);
	TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Disable);

	TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_ForcedAction_InActive );
	TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Disable);
	TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Disable);

	TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_ForcedAction_InActive );
	TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Disable);
	TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Disable);
	TIM_GenerateEvent(TIM1,TIM_EventSource_COM);//CAN this work???
}
void Set_TingJi_Fault(void)
{
	IPM_INPUT_DISABLE;
	//LED1_OFF;
	//LED2_OFF;
	//T2_H;
	controllermode.sys_Fault=1;
	faultinfo.ting_ji=999;
	TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_ForcedAction_InActive );
	TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Disable);
	TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Disable);

	TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_ForcedAction_InActive );
	TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Disable);
	TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Disable);

	TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_ForcedAction_InActive );
	TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Disable);
	TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Disable);
	TIM_GenerateEvent(TIM1,TIM_EventSource_COM);//CAN this work???
}
void Set_Over_Speed_Fault(void)
{
	IPM_INPUT_DISABLE;
	//LED1_OFF;
	//LED2_OFF;
	//T2_H;
	controllermode.sys_Fault=1;
	faultinfo.over_speed=999;
	TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_ForcedAction_InActive );
	TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Disable);
	TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Disable);

	TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_ForcedAction_InActive );
	TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Disable);
	TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Disable);

	TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_ForcedAction_InActive );
	TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Disable);
	TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Disable);
	TIM_GenerateEvent(TIM1,TIM_EventSource_COM);//CAN this work???
}
void Fault_Regular_Scan(void)
{
		if(Get_Time_Interval_ms(&globaltime,&(faultinfo.next_fault_check_time))>0)
		{
			int sample_hall=GET_HALL_UVW;
			if(sample_hall==0||sample_hall==7)
			{
				int count_hall=0;
				int j;
				for(j=0;j<100;j++)
				{
					sample_hall=GET_HALL_UVW;
					if(sample_hall==0||sample_hall==7)
					{
						count_hall++;
					}
				}
				if(count_hall>80)
				{
					Set_HALL_Fault();
				}
			}
			if(IS_UV)
			{
				faultinfo.uv=1;
			}
			if(IS_OV)
			{
				faultinfo.ov=1;
			}
			if(IS_BKOV)
			{
				faultinfo.bkov=1;
			}
			if(IS_IPM_FO)
			{
				//faultinfo.ipmfo=1;
				//controllermode.sys_Fault=1;
				EXTI_GenerateSWInterrupt(EXTI_Line12);
			}
			if(IS_IPM_OT)
			{
				int count=0;
				int r=0;
				for(r=0;r<300;r++)
				{
					if(IS_IPM_OT)
					{
						count++;
					}
				}
				if(count>200)
				{
					Set_Temp_Fault();
				}
			}
			if(IS_PHASE_OC)
			{
				//faultinfo.oc_source=10000;
				//controllermode.sys_Fault=1;
				EXTI_GenerateSWInterrupt(EXTI_Line1);
			}
			
			if((faultinfo.duzhuan_watchdog>0)&&(hall2speedinfo.speed_input>0)&&(hall2speedinfo.duzhuan>0))//20140314
			{
				faultinfo.ting_ji=1;
				Set_TingJi_Fault();
			}
			else
			{
				faultinfo.ting_ji=0;
			}
			Plus_Time_ms(&(faultinfo.next_fault_check_time),50);
		}
}

 

