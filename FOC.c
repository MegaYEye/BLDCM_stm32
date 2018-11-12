#include "MyGlobal.h"
#include "FOC.h"
#include "operation.h"
int sin_table[91]={0,17,35,52,70,87,105,122,139,156,174,191,208,225,242,259,276,292,309,326,342,358,375,391,407,423,438,454,469,485,500,515,530,545,559,574,588,602,616,629,643,656,669,682,695,707,719,731,743,755,766,777,788,799,809,819,829,839,848,857,866,875,883,891,899,906,914,921,927,934,940,946,951,956,961,966,970,974,978,982,985,988,990,993,995,996,998,999,999,1000,1000};
int SIN_INT(int angle_360)
{
	int value;
	int temp;
	int fuhao;
	while(angle_360<0){angle_360+=360;}
	temp=angle_360/180;
	if(temp%2==1) fuhao=-1;
	else fuhao=1;
	while(angle_360>180)
	{
		angle_360-=180;
	}
	while(angle_360<0)
	{
		angle_360+=180;
	}
	if(angle_360>90)
	{
		angle_360=180-angle_360;
	}
	value=sin_table[angle_360]*fuhao;
	return value;
	
}
int COS_INT(int angle_360)
{
	return SIN_INT(angle_360+90);
}

int sqrt_INT(int x) {
	int result = 1000;    //initial
	int precise = 2; //jing du
	int count=0;
	while(count++<50) {
		int d = (result + x / result) / 2;
		if ((result - d < precise)&&(result-d>-precise))
		{
			break;
		}
		result = d;
		
	}
	
	return result;
	
}
void Clarke(int *abc,int *alpha_beta)
{
	int a=abc[0];
	int b=abc[1];
	//int c=abc[2];
	//int alpha=816*a-b*408-c*408;
	//int beta=707*b-707*c;
	//p165	
	int alpha=1225*a;
	int beta=707*a+1414*b;
	alpha_beta[0]=alpha/1000;
	alpha_beta[1]=beta/1000;
}
void Reverse_Clarke(int *alpha_beta,int *abc)
{
	int alpha=alpha_beta[0];
	int beta=alpha_beta[1];
	abc[0]=(816*alpha)/1000;
	abc[1]=(-408*alpha+707*beta)/1000;
	abc[2]=(-408*alpha-707*beta)/1000;
}
void Park(int *alpha_beta,int *dq,int angle_360)
{
	int alpha=alpha_beta[0];
	int beta=alpha_beta[1];
	int c=COS_INT(angle_360);
	int s=SIN_INT(angle_360);
	dq[0]=c*alpha+s*beta;
	dq[1]=-s*alpha+c*beta;

	dq[0]/=1000;
	dq[1]/=1000;
}
void Reverse_Park(int *dq,int *alpha_beta,int angle_360)
{
	int d=dq[0];
	int q=dq[1];

	alpha_beta[0]=COS_INT(angle_360)*d-SIN_INT(angle_360)*q;
	alpha_beta[1]=SIN_INT(angle_360)*d+COS_INT(angle_360)*q;

	alpha_beta[0]=alpha_beta[0]/1000;
	alpha_beta[1]=alpha_beta[1]/1000;


}

void SVPWM_AreaDecision(void)
{	
	//checked,no problem found
	int a,b,c,n,Area;
	int x,y,z;
	x=focinfo_sv.U_alpha_beta_given[1]*1000;//cos:theta-90 
	y=(866*focinfo_sv.U_alpha_beta_given[0]-focinfo_sv.U_alpha_beta_given[1]*500);//cos:theta-330 
	z=-866*focinfo_sv.U_alpha_beta_given[0]-focinfo_sv.U_alpha_beta_given[1]*500;//cos:theta-210
	a=(x>0)?1:0;
	b=(y>0)?1:0;
	c=(z>0)?1:0;
	n=4*c+2*b+1*a;
	switch(n)
	{
	case 1:    
		Area=2;
		break;
	case 2:
		Area=6;
		break;
	case 3:
		Area=1;
		break;
	case 4:
		Area=4;
		break;
	case 5:
		Area=3;
		break;
	case 6:
		Area=5;
		break;
	default:
		Area=0;
		
		break;
	}
		
	
	if(Area>0)
	{	
		focinfo_sv.U_alpha_beta_given_Area=Area;
	}
	else//boundary of 2 areas
	{
		focinfo_sv.U_alpha_beta_given_Area=focinfo_sv.U_alpha_beta_given_Area;
	}

}
void SVPWM_TimeDecision2(void)
{
	int udc,t1,t2,t0,period,ua,ub,k,total;
	udc=controllermode.Udc;
	ua=focinfo_sv.U_alpha_beta_given[0];
	ub=focinfo_sv.U_alpha_beta_given[1];
	period=pwm_generator.pwm_period*2-1;
 
	switch(focinfo_sv.U_alpha_beta_given_Area)
	{
			case 1:
				t1=866*ua-500*ub;//u1
				t2=1000*ub;//u2
				break;
			case 2:
				t2=866*ua+500*ub;//u2
				t1=-866*ua+500*ub;//u3
				break;
			case 3:
				t1=1000*ub;//u3
				t2=-866*ua-500*ub;//u4
				break;
			case 4:
				t2=-866*ua+500*ub;//u4
				t1=-1000*ub;//u5
				break;
			case 5:
				t1=-866*ua-500*ub;//u5
				t2=866*ua-500*ub;//u6
				break;
			case 6:
				t2=-1000*ub;//u6
				t1=866*ua+500*ub;//u1
				break;
			default:
				t1=0;
				t2=0;
				;
	}
	t1=t1/1000;
	t2=t2/1000;
	
	k=1414*period/1000;
	t1=k*t1/udc;
	t2=k*t2/udc;
	total=t1+t2;
	if(total>period)
	{
		t1=t1*period/total;
		t2=t2*period/total;//bug t2
	}

	t0=period-t1-t2;
	focinfo_sv.Vector_Time_t1=t1;
	focinfo_sv.Vector_Time_t2=t2;
	if(t0<=0)
	{
		t0=0;
	}
	
	focinfo_sv.Vector_Time[0]=t0/4;//first apper vector
	if(focinfo_sv.Vector_Time[0]>=pwm_generator.pwm_period)
	{
			focinfo_sv.Vector_Time[0]=pwm_generator.pwm_period;
	}
	focinfo_sv.Vector_Time[1]=focinfo_sv.Vector_Time[0]+t1/2;//second appear vector
	if(focinfo_sv.Vector_Time[1]>=pwm_generator.pwm_period)
	{
			focinfo_sv.Vector_Time[1]=pwm_generator.pwm_period;
	}
	focinfo_sv.Vector_Time[2]=focinfo_sv.Vector_Time[1]+t2/2;//zero vector---111
	if(focinfo_sv.Vector_Time[2]>=pwm_generator.pwm_period)
	{
			focinfo_sv.Vector_Time[2]=pwm_generator.pwm_period;
	}
}

void U_alpha_beta_Limit(void)
{
	int v_usa=focinfo_sv.U_alpha_beta_given[0];
	int v_usb=focinfo_sv.U_alpha_beta_given[1];
	int length;

	//zhe li xie de bu hao
	length=v_usa*v_usa+v_usb*v_usb;
	length=sqrt_INT(length);
	if(length>focinfo_sv.U_limitCircle)
	{
		focinfo_sv.U_alpha_beta_given[0]=(v_usa*focinfo_sv.U_limitCircle)/length;
		focinfo_sv.U_alpha_beta_given[1]=(v_usb*focinfo_sv.U_limitCircle)/length;
	}
	else
	{
		focinfo_sv.U_alpha_beta_given[0]=v_usa;
		focinfo_sv.U_alpha_beta_given[1]=v_usb;
	}
	
	
}
void FOC_TIM_Action2(void)
{
	switch(focinfo_sv.U_alpha_beta_given_Area)
	{
		case 1://ABC 000 100 110 111 110 100 000//ok
			TIM1->CCR1=focinfo_sv.Vector_Time[0];
			TIM1->CCR2=focinfo_sv.Vector_Time[1];
			TIM1->CCR3=focinfo_sv.Vector_Time[2];
		break;
		case 2://ABC 000 010 110 111 110 010 000
			TIM1->CCR2=focinfo_sv.Vector_Time[0];
			TIM1->CCR1=focinfo_sv.Vector_Time[1];
			TIM1->CCR3=focinfo_sv.Vector_Time[2];
		break;
		case 3://ABC 000 010 011 111 011 010 000 ok
			TIM1->CCR2=focinfo_sv.Vector_Time[0];
			TIM1->CCR3=focinfo_sv.Vector_Time[1];
			TIM1->CCR1=focinfo_sv.Vector_Time[2];
		break;
		case 4://ABC 000 001 011 111 011 001 000 ok
			TIM1->CCR3=focinfo_sv.Vector_Time[0];
			TIM1->CCR2=focinfo_sv.Vector_Time[1];
			TIM1->CCR1=focinfo_sv.Vector_Time[2];
		break;
		case 5://ABC 000 001 101 111 101 001 000//ok
			TIM1->CCR3=focinfo_sv.Vector_Time[0];
			TIM1->CCR1=focinfo_sv.Vector_Time[1];
			TIM1->CCR2=focinfo_sv.Vector_Time[2];
		break;
		case 6://ABC 000 100 101 111 101 100 000//ok
			TIM1->CCR1=focinfo_sv.Vector_Time[0];
			TIM1->CCR3=focinfo_sv.Vector_Time[1];
			TIM1->CCR2=focinfo_sv.Vector_Time[2];
		break;
		default:;
	}
	
}
 
//zheng:-120 fan : 60
 

void FOC_debug(void)
{
	#ifdef DAC1_ELEC_ANGLE
		DAC_outputdata(rotor_angle_est.rotor_angle*2);
#endif
#ifdef DAC2_ELEC_ANGLE
		DAC_outputdata2(rotor_angle_est.rotor_angle*2);
#endif
#ifdef DAC1_TIM1_CCR1
		DAC_outputdata(4096-TIM1->CCR1/2);
#endif
#ifdef DAC2_TIM1_CCR1
		DAC_outputdata2(4096-TIM1->CCR1/2);
#endif
#ifdef DAC1_TIM1_CCR2
		DAC_outputdata(4096-TIM1->CCR2/2);
#endif
#ifdef DAC2_TIM1_CCR2
		DAC_outputdata2(4096-TIM1->CCR2/2);
#endif
#ifdef DAC1_TIM1_CCR3
		DAC_outputdata(4096-TIM1->CCR3/2);
#endif
#ifdef DAC2_TIM1_CCR3
		DAC_outputdata2(4096-TIM1->CCR3/2);
#endif
#ifdef DAC1_Voltage_UV
	DAC_outputdata((TIM1->CCR2-TIM1->CCR1+2048)/2);
#endif
#ifdef DAC2_Voltage_UV
	DAC_outputdata2((TIM1->CCR2-TIM1->CCR1+2048)/2);
#endif
#ifdef DAC1_Voltage_UW
	DAC_outputdata((TIM1->CCR3-TIM1->CCR1+2048)/2);
#endif
#ifdef DAC2_Voltage_UW
	DAC_outputdata2((TIM1->CCR3-TIM1->CCR1+2048)/2);
#endif
#ifdef DAC1_IQFB
	DAC_outputdata(10*(focinfo_sv.I_dq_FB[1])+2048);
#endif
#ifdef DAC2_IQFB
	DAC_outputdata2(10*(focinfo_sv.I_dq_FB[1])+2048);
#endif
#ifdef DAC1_IAFB
	DAC_outputdata(10*(focinfo_sv.I_alpha_beta_FB[0])+2048);
#endif
#ifdef DAC2_IAFB
	DAC_outputdata2(10*(focinfo_sv.I_alpha_beta_FB[0])+2048);
#endif
#ifdef DAC12_IABC_AB
	DAC_outputdata(10*(focinfo_sv.I_abc_FB[0])+2048);
#endif
#ifdef DAC12_IABC_AB
	DAC_outputdata2(10*(focinfo_sv.I_abc_FB[1])+2048);
#endif
#ifdef DAC12_UAB_G
 DAC_outputdata((focinfo_sv.U_alpha_beta_given[0])/5+2048);
#endif
#ifdef DAC12_UAB_G
 DAC_outputdata2((focinfo_sv.U_alpha_beta_given[1])/5+2048);
#endif

}
void Rotor_angle_Est()
{
	int angle_base;
	int est_delta_angle;
		if(rotor_angle_est.EST_EN<=0)
		{
			rotor_angle_est.diff_count=0;
			rotor_angle_est.est_count=0;
			rotor_angle_est.last_count=9999999;
			rotor_angle_est.hall_COM_signal=0;
			rotor_angle_est.rotor_angle=0;
			return;
		}
		switch(hall2speedinfo.cur_Hall_state)
		{	
			case 2:
				angle_base=30;
				break;
			case 3:
				angle_base=90;
				break;
			case 1:
				angle_base=150;
				break;
			case 5:
				angle_base=210;
				break;
			case 4:
				angle_base=270;
				break;
			case 6:
				angle_base=330;
				break;
		}
		if(controllermode.zheng_fan_zhuan==2)//1107
		{
			angle_base+=60;
		}
		rotor_angle_est.est_count++;
		if(rotor_angle_est.est_count>=300000)
		{
				rotor_angle_est.est_count=300000;
		}
		if(rotor_angle_est.last_count<=0)
		{
			rotor_angle_est.last_count=9999999;
		}
		est_delta_angle=60*rotor_angle_est.est_count/rotor_angle_est.last_count;
		if(est_delta_angle>90) 
		{
			est_delta_angle=90;
		}
		
		if(rotor_angle_est.hall_COM_signal>0)
		{
			rotor_angle_est.diff_count=rotor_angle_est.est_count-rotor_angle_est.last_count;
			rotor_angle_est.last_count=rotor_angle_est.est_count;
			rotor_angle_est.est_count=0;
			rotor_angle_est.hall_COM_signal=0;
			rotor_angle_est.est_angle_60=est_delta_angle;
			
			if(est_delta_angle>85||est_delta_angle<35)//20140311
			{
				rotor_angle_est.EST_EN=2;
			}
			
			else if((est_delta_angle>40)&&(est_delta_angle<80))//20140311
			{
				rotor_angle_est.EST_EN=1;
			}
			else
			{
				;//est_en bu bian...
			}
		}
		else if(hall2speedinfo.duzhuan>0)//20140311
		{
			rotor_angle_est.EST_EN=2;
		}
		
		if(rotor_angle_est.EST_EN==1)
		{
			if(controllermode.zheng_fan_zhuan==1)//1107
			{
				rotor_angle_est.rotor_angle=angle_base+est_delta_angle;
			}
			else
			{
				rotor_angle_est.rotor_angle=angle_base-est_delta_angle;
			}
		}
		else if(rotor_angle_est.EST_EN==2)
		{
			if(controllermode.zheng_fan_zhuan==1)//1107
			{
				rotor_angle_est.rotor_angle=angle_base+30;//liu Bu huan xiang ci lian wei zhi
			}
			else
			{
				rotor_angle_est.rotor_angle=angle_base-30;
			}
		}
	
		while(rotor_angle_est.rotor_angle>360)
		{
			rotor_angle_est.rotor_angle-=360;
		}
		while(rotor_angle_est.rotor_angle<=0)
		{
			rotor_angle_est.rotor_angle+=360;
		}
		FOC_debug();

}
//struct FOC_SVPWM_Def abaa;
void Isensor_zero_calibration_dynamic(void)
{
	int ab[2];
	int abc[3];
	if(hall2speedinfo.speed<=0)
	{
			return;
	}
	

	currentinfo.zero_calibrator_alpha+=focinfo_sv.I_alpha_beta_FB[0];
	currentinfo.zero_calibrator_beta+=focinfo_sv.I_alpha_beta_FB[1];
	if(currentinfo.zero_calibrator_alpha>=2000000)
	{
		currentinfo.zero_calibrator_alpha=2000000;
	}
	else if(currentinfo.zero_calibrator_alpha<=-2000000)
	{
		currentinfo.zero_calibrator_alpha=-2000000;
	}
	
	if(currentinfo.zero_calibrator_beta>=2000000)
	{
		currentinfo.zero_calibrator_beta=2000000;
	}
	else if(currentinfo.zero_calibrator_beta<=-2000000)
	{
		currentinfo.zero_calibrator_beta=-2000000;
	}
	
	ab[0]=currentinfo.zero_calibrator_alpha/30000;
	ab[1]=currentinfo.zero_calibrator_beta/30000;
	Reverse_Clarke(ab,abc);
	currentinfo.offset_zero_current_u=abc[0];
	currentinfo.offset_zero_current_v=abc[1];
	if(currentinfo.offset_zero_current_u>50)
	{
			currentinfo.offset_zero_current_u=50;
	}
	else if(currentinfo.offset_zero_current_u<-50)
	{
		currentinfo.offset_zero_current_u=-50;
	}
	if(currentinfo.offset_zero_current_v>50)
	{
			currentinfo.offset_zero_current_v=50;
	}
	else if(currentinfo.offset_zero_current_v<-50)
	{
		currentinfo.offset_zero_current_v=-50;
	}
}
void FOC_ENTRANCE(void)
{
	Rotor_angle_Est();
	Get_Current_UVW(&(focinfo_sv.I_abc_FB[0]),&(focinfo_sv.I_abc_FB[1]),&(focinfo_sv.I_abc_FB[2]));
	Clarke(focinfo_sv.I_abc_FB,focinfo_sv.I_alpha_beta_FB);
	Park((focinfo_sv.I_alpha_beta_FB),(focinfo_sv.I_dq_FB),(rotor_angle_est.rotor_angle));//error
	//filter(&rc_current_d,focinfo_sv.I_dq_FB[0]);
	//filter(&rc_current_q,focinfo_sv.I_dq_FB[1]);
	if(focinfo_sv.SVPWM_Openloop_switch==0)
	{
		PID_calculator(&PID_id,focinfo_sv.I_dq_FB[0]);
		if(controllermode.zheng_fan_zhuan==1)//1107
		{
			PID_calculator(&PID_torque,focinfo_sv.I_dq_FB[1]);
		}
		else
		{
			PID_calculator(&PID_torque,0-focinfo_sv.I_dq_FB[1]);
		}
		focinfo_sv.U_dq_given[0]=PID_id.P;
		if(controllermode.zheng_fan_zhuan==1)//1107
		{
			focinfo_sv.U_dq_given[1]=PID_torque.P;
		}
		else
		{
			focinfo_sv.U_dq_given[1]=0-PID_torque.P;
		}
	}
#ifdef DAC12_PID_TORQUE
	DAC_outputdata(PID_torque.ek*PID_torque.Kp/3+2048);
	DAC_outputdata2(PID_torque.IntContainer/2000+2048);
#endif
	Reverse_Park(focinfo_sv.U_dq_given,focinfo_sv.U_alpha_beta_given,rotor_angle_est.rotor_angle);
	SVPWM_AreaDecision();
	U_alpha_beta_Limit();
	SVPWM_TimeDecision2();
	FOC_TIM_Action2();
#ifdef ISENSOR_DYM_CALIBRATION
	Isensor_zero_calibration_dynamic();
#endif
	
	
}




