/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "myglobal.h"
#include "devicedriver.h"
#include "operation.h"
#include "Foc.h"
#include "Fault.h"
/** @addtogroup STM32F10x_StdPeriph_Template
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
	IPM_INPUT_DISABLE;
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
	IPM_INPUT_DISABLE;
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
	IPM_INPUT_DISABLE;
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
	IPM_INPUT_DISABLE;
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
	 ;
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */

void SysTick_Handler(void)
{
	globaltime.time_us+=100;
	if(globaltime.time_us>=1000)
	{
		globaltime.time_us=globaltime.time_us-1000;
		globaltime.time_ms+=1;
	}
	if(globaltime.time_ms>=1000)
	{
		globaltime.time_ms=globaltime.time_ms-1000;
		globaltime.time_s+=1;
	}
}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 
void EXTI1_IRQHandler(void)
{
	//oc
	EXTI_ClearITPendingBit(EXTI_Line1);
	OC_Process();
}

void EXTI15_10_IRQHandler(void)
{
	EXTI_ClearITPendingBit(EXTI_Line12);
	IPMFT_Process();
}
void USART1_IRQHandler(void)
{
	if(USART_GetFlagStatus(USART1,USART_FLAG_RXNE)==SET)
		receive_buffer[RS485info.pointer_receive_buffer++]=USART_ReceiveData(USART1);
	if(RS485info.pointer_receive_buffer>=RECEIVE_BUFFER_LENGTH)//this code needs to be rewrited soon...
	{
			RS485info.pointer_receive_buffer=0;
	}
//	m=m&0xFF;//maybe this is needed
}
void DMA1_Channel4_IRQHandler(void)// no use now
{
		if(DMA_GetFlagStatus(DMA1_FLAG_TC4)==SET)
		{
				DMA_ClearFlag(DMA1_FLAG_TC4); //must be cleaned by software,RM0008 P275
		}
		USART_DMACmd(USART1, USART_DMAReq_Tx, DISABLE);	//how to send_buffer again?????
		RS485info.send_state=0;
}

void TIM3_IRQHandler(void)
{

	if(TIM_GetITStatus(TIM3,TIM_IT_Trigger)!=RESET)
	{
		int quit=0;
		int hall=0;
		TIM_ClearITPendingBit(TIM3,TIM_IT_Trigger);

		hall=Hall_filter();

		if((hall==hall2speedinfo.cur_Hall_state)||
			(hall<1)||(hall>6))//fang zhi du zhuan hou dou dong...
		{
			int temp;
			
			temp=TIM3->CCR1+1;//1125 //bu huo zhi

			if(temp>9998) 
			{
				temp=9998;
			}
			TIM3->CNT=temp;// ji xu ji shu, fang zhi cnt bei ying jian qing ling
			quit=1;
			if(Get_Time_Interval_ms(&globaltime,&(hall2speedinfo.last_com_time))>500)
			{
				hall2speedinfo.duzhuan=1;
				hall2speedinfo.speed=0;
				rc_hall_speed.out_data=0;
				rc_echo_speed.in_data=0;
				rc_echo_speed.out_data=0;
			}
			
		}
		if(quit==0)
		{
			int k;
			int count[8]={0,0,0,0,0,0,0,0};
			
				hall2speedinfo.last_Hall_state=hall2speedinfo.cur_Hall_state;
				hall2speedinfo.cur_Hall_state=hall;			
				hall2speedinfo.pre_Hall_state=Get_Pre_Hall_State(hall2speedinfo.cur_Hall_state);
//==========du zhuan fang dou ========//
			faultinfo.hall_memory[faultinfo.duzhuan_check_hall_counter]=hall;
			faultinfo.duzhuan_check_hall_counter++;
			if(faultinfo.duzhuan_check_hall_counter>=HALL_MEM_SIZE)
			{
				faultinfo.duzhuan_check_hall_counter=0;
			}
			
			for(k=0;k<HALL_MEM_SIZE;k++)
			{
				count[faultinfo.hall_memory[k]]++;
				if((count[0]>0)||(count[7]>0))
				{
					break;
				}
				else if(count[faultinfo.hall_memory[k]]>DUZHUAN_FUNCTION_TIME)
				{
					hall2speedinfo.duzhuan=1;
					hall2speedinfo.speed=0;
					rc_hall_speed.out_data=0;
				}
			}
			
				if(controllermode.state_square1_foc0==1)
				{
					Change_PWM_state_PWM_ON(hall);
					TIM_GenerateEvent(TIM1,TIM_EventSource_COM);
					//controllermode.state_square1_foc0=1;
				}
				Update_Speed();
				hall2speedinfo.duzhuan=0;
				rotor_angle_est.hall_COM_signal=1;
		if((hall2speedinfo.speed>OVER_SPEED_UP_GATE)&&(hall2speedinfo.speed>hall2speedinfo.speed_input+100))
		{
			faultinfo.over_speed++;
		}
		else if((hall2speedinfo.speed<OVER_SPEED_DOWN_GATE)||(hall2speedinfo.speed<hall2speedinfo.speed_input+50))
		{
			faultinfo.over_speed=0;
		}
		
		if(faultinfo.over_speed>OVER_SPEED_COUNT_GATE)
		{
			faultinfo.over_speed=999;
			Set_Over_Speed_Fault();
			
		}
		
#ifdef  DAC1_HALL_U
		DAC_outputdata(500*(hall&0x4));
#endif
#ifdef  DAC2_HALL_U
		DAC_outputdata2(500*(hall&0x4));
#endif
		
#ifdef  DAC1_HALL_V
		DAC_outputdata(500*(hall&0x2));
#endif
#ifdef  DAC2_HALL_V
		DAC_outputdata2(500*(hall&0x2));
#endif
		
#ifdef  DAC1_HALL_W
		DAC_outputdata(500*(hall&0x1));
#endif
#ifdef  DAC2_HALL_W
		DAC_outputdata2(500*(hall&0x1));
#endif
		
		
#ifdef DAC1_HALL_6STEP
		DAC_outputdata(500*hall);
#endif
#ifdef DAC2_HALL_6STEP
		DAC_outputdata2(500*hall);
#endif
		}
	}
	
	if(TIM_GetITStatus(TIM3,TIM_IT_CC3)!=RESET)
	{
		TIM_ClearITPendingBit(TIM3,TIM_IT_CC3);
		hall2speedinfo.duzhuan=1;
		hall2speedinfo.speed=0;
		rc_hall_speed.out_data=0;
		rc_echo_speed.in_data=0;
		rc_echo_speed.out_data=0;
		TIM_SetCounter(TIM3,1);
	}
	if(TIM_GetITStatus(TIM3,TIM_IT_Update)!=RESET)
	{
			TIM_ClearITPendingBit(TIM3,TIM_IT_Update);
	}
	if(TIM_GetITStatus(TIM3,TIM_IT_CC1)!=RESET)
	{
		TIM_ClearITPendingBit(TIM3,TIM_IT_CC1);
	}
		if(TIM_GetITStatus(TIM3,TIM_IT_CC2)!=RESET)
	{
		TIM_ClearITPendingBit(TIM3,TIM_IT_CC2);
	}
		if(TIM_GetITStatus(TIM3,TIM_IT_CC4)!=RESET)
	{
		TIM_ClearITPendingBit(TIM3,TIM_IT_CC4);
	}


		//20140312
	if(hall2speedinfo.speed>DUZHUAN_WATCHDOG_START_SPEED)
	{
		faultinfo.duzhuan_watchdog=1;
	}
	
	if((faultinfo.duzhuan_watchdog>0)&&(hall2speedinfo.speed_input>0)&&(hall2speedinfo.duzhuan>0))//20140312
	{
		faultinfo.ting_ji=1;
		Set_TingJi_Fault();
	}
	else
	{
		faultinfo.ting_ji=0;
	}
	
}


void TIM1_UP_IRQHandler(void)
{
	TIM_ClearITPendingBit(TIM1,TIM_IT_Update); 
	CSFB_calculate();
	if(controllermode.state_square1_foc0==0)
	{
		focinfo_sv.I_dq_given[1]=csfb.output_value;
		//0227
		//if(focinfo_sv.I_dq_given[1]<0)
		//{
		//	focinfo_sv.I_dq_given[1]=0;
		//}
		FOC_ENTRANCE();
	}
	else
	{
		SQUARE_ENTRANCE();
		//Rotor_angle_Est();//0122
#ifdef DAC12_PID_CURRENT
			DAC_outputdata(PID_current.ek*PID_current.Kp/2+2048);
			DAC_outputdata2(PID_current.IntContainer/3000+2048);
#endif
	}
	
}

void TIM1_TRG_COM_IRQHandler(void)
{ 
	TIM_ClearITPendingBit(TIM1, TIM_IT_COM); 
}
//ce shi :com zhong duan di que zhi neng zai xia ci COM dao lai shi qi zuo yong....

void TIM2_IRQHandler(void)
{
	int speed_ek_abs=PID_speed.ek;
	TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	if(speed_ek_abs<0) 
	{
		speed_ek_abs=-speed_ek_abs;
	}

	//PID here
//	PID_speed.KI_prescaler=(speed_ek_abs+1)*15;
	//PID_speed.KI_prescaler=60;
	
	hall2speedinfo.speed_input=Get_Speed_Given();
	PID_calculator(&PID_speed,hall2speedinfo.speed);
	
	if(motor_starter.Max_PWMDUTY_auto_gain>0)
	{
		int temp;
		temp=Gain_With_Time(motor_starter.duty_limit_Base,1000,INNERLOOP_OPEN_TIME_MS,&(motor_starter.accelerate_start_time3));

		PID_current.Out_Up_Limit=current_pid_out_up_limit*temp/1000;
		PID_torque.Out_Up_Limit=torque_pid_out_up_limit*temp/1000;
		if(PID_current.Out_Up_Limit>current_pid_out_up_limit)
		{
			PID_current.Out_Up_Limit=current_pid_out_up_limit;
		}
		if(PID_torque.Out_Up_Limit>torque_pid_out_up_limit)
		{
			PID_torque.Out_Up_Limit=torque_pid_out_up_limit;
		}

		PID_current.INT_Up_Limit=PID_current.Out_Up_Limit*1000;
		PID_torque.INT_Up_Limit=PID_torque.Out_Up_Limit*1000;
		if(temp>=1000)
		{
			motor_starter.Max_PWMDUTY_auto_gain=0;
		}
	}

#ifdef DAC12_PIDSPEED
	DAC_outputdata(2048+PID_speed.ek*PID_speed.Kp/3);
	DAC_outputdata2(2048+PID_speed.IntContainer/2000);
#endif
	
}
void TIM1_CC_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM1,TIM_IT_CC4)!=RESET)
	{	
		TIM_ClearITPendingBit(TIM1,TIM_IT_CC4);
		ADC_SoftwareStartConvCmd(ADC1,ENABLE);
	}
		
}

//end of file.....
