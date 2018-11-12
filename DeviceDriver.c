#include "DeviceDriver.h"
#include "stm32f10x.h"
#include "defines.h"
#include "MyGlobal.h"
//nvic
/*
0 0:oc
0 1:ipmfo
0 3:TIM3 trigger/CC
0 2:TIM1 COM
1 0 TIM1 Update
1 1 TIM1 CC
1 2 TIM2
2 1USART
2 2USART




*/

void RCC_config(void)
{
	//for gpio
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA
	|RCC_APB2Periph_GPIOB
	|RCC_APB2Periph_GPIOC
	|RCC_APB2Periph_GPIOD
	|RCC_APB2Periph_GPIOE
	,ENABLE);
	//for adc,regular simutaneous mode
	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2,ENABLE);
	RCC_ADCCLKConfig(RCC_PCLK2_Div8);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1|RCC_APB2Periph_ADC2|RCC_APB2Periph_ADC3 ,ENABLE);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC,ENABLE);
	
}
void Init_DAC()
{
	DAC_InitTypeDef DAC_InitStructure;
	DAC_InitStructure.DAC_Trigger = DAC_Trigger_Software;
			
	DAC_InitStructure.DAC_WaveGeneration=DAC_WaveGeneration_None;
	DAC_InitStructure.DAC_OutputBuffer=DAC_OutputBuffer_Disable;
		
	DAC_InitStructure.DAC_LFSRUnmask_TriangleAmplitude=DAC_LFSRUnmask_Bit0;
	DAC_Init(DAC_Channel_1, &DAC_InitStructure);
	DAC_Init(DAC_Channel_2,&DAC_InitStructure);
	DAC_Cmd(DAC_Channel_1, ENABLE);
	DAC_Cmd(DAC_Channel_2, ENABLE);

}

void Init_SysTick(void)
{
	//NVIC_SetPriority(SysTick_IRQn, 2);
		SysTick_Config(SystemCoreClock/10000);//100us between per interupt
}
//½è¼ø£º¹Ù·½3.5¿â£¬RegSimul_DualModeÀý³Ì
void ADC_config(void)
{
	ADC_InitTypeDef adc_init_struct;
	DMA_InitTypeDef dma_init_struct;
	
	DMA_DeInit(DMA1_Channel1);
	dma_init_struct.DMA_BufferSize=BUFFER_CURRENT;
	dma_init_struct.DMA_DIR=DMA_DIR_PeripheralSRC;
	dma_init_struct.DMA_M2M=DMA_M2M_Disable;
	dma_init_struct.DMA_MemoryBaseAddr=(uint32_t)(&ADVal_current_UV[0]);
	dma_init_struct.DMA_MemoryDataSize=DMA_MemoryDataSize_Word;
	dma_init_struct.DMA_MemoryInc=DMA_MemoryInc_Enable;
	dma_init_struct.DMA_Mode=DMA_Mode_Circular;	
	dma_init_struct.DMA_PeripheralBaseAddr=(uint32_t)(&(ADC1->DR));
	dma_init_struct.DMA_PeripheralDataSize=DMA_PeripheralDataSize_Word;
	dma_init_struct.DMA_PeripheralInc=DMA_PeripheralInc_Disable;
	dma_init_struct.DMA_Priority=DMA_Priority_High;
	DMA_Init(DMA1_Channel1,&dma_init_struct);
	DMA_Cmd(DMA1_Channel1,ENABLE);
	
	//adc1
	adc_init_struct.ADC_Mode=ADC_Mode_RegSimult;
	adc_init_struct.ADC_ScanConvMode = ENABLE;
	adc_init_struct.ADC_ContinuousConvMode = DISABLE;
	adc_init_struct.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	adc_init_struct.ADC_DataAlign = ADC_DataAlign_Right;
	adc_init_struct.ADC_NbrOfChannel = 3;//adc1:current u u
	ADC_Init(ADC1, &adc_init_struct);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 1, ADC_SampleTime_7Cycles5);   //how many us? ---about 8
	ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 2, ADC_SampleTime_7Cycles5); 
	ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 3, ADC_SampleTime_7Cycles5);
//	ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 4, ADC_SampleTime_1Cycles5);
//ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 5, ADC_SampleTime_1Cycles5); 
	ADC_DMACmd(ADC1, ENABLE);
	
	//adc2
	adc_init_struct.ADC_Mode = ADC_Mode_RegSimult;
	adc_init_struct.ADC_ScanConvMode = ENABLE;
	adc_init_struct.ADC_ContinuousConvMode = DISABLE;
	adc_init_struct.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;//set extsel = software start AD converter
	adc_init_struct.ADC_DataAlign = ADC_DataAlign_Right;
	adc_init_struct.ADC_NbrOfChannel = 3;
	ADC_Init(ADC2, &adc_init_struct);
	ADC_RegularChannelConfig(ADC2, ADC_Channel_13, 1, ADC_SampleTime_7Cycles5);
	ADC_RegularChannelConfig(ADC2, ADC_Channel_13, 2, ADC_SampleTime_7Cycles5);
	ADC_RegularChannelConfig(ADC2, ADC_Channel_13, 3, ADC_SampleTime_7Cycles5);
//		ADC_RegularChannelConfig(ADC2, ADC_Channel_13, 4, ADC_SampleTime_1Cycles5);
//	ADC_RegularChannelConfig(ADC2, ADC_Channel_13, 5, ADC_SampleTime_1Cycles5);
	ADC_ExternalTrigConvCmd(ADC2, ENABLE);//set exttrig
	
	ADC_Cmd(ADC1, ENABLE);

//	ADC_TempSensorVrefintCmd(ENABLE);//I don't want to use temp sensor,so,this code may be of no use.
	
	ADC_ResetCalibration(ADC1);
	while(ADC_GetResetCalibrationStatus(ADC1));
	ADC_StartCalibration(ADC1);
	while(ADC_GetCalibrationStatus(ADC1));

	ADC_Cmd(ADC2, ENABLE);
	ADC_ResetCalibration(ADC2);
	while(ADC_GetResetCalibrationStatus(ADC2));
	ADC_StartCalibration(ADC2);
	while(ADC_GetCalibrationStatus(ADC2));

	//adc3 using dma2 channel 5
	DMA_DeInit(DMA2_Channel5);
	dma_init_struct.DMA_BufferSize=BUFFER_DIANWEIQI;
	dma_init_struct.DMA_DIR=DMA_DIR_PeripheralSRC;
	dma_init_struct.DMA_M2M=DMA_M2M_Disable;
	dma_init_struct.DMA_MemoryBaseAddr=(u32)(ADVal_GeiDing);
	dma_init_struct.DMA_MemoryDataSize=DMA_MemoryDataSize_Word;
	dma_init_struct.DMA_MemoryInc=DMA_MemoryInc_Enable;
	dma_init_struct.DMA_Mode=DMA_Mode_Circular;
	dma_init_struct.DMA_PeripheralBaseAddr=((uint32_t)0x40013C4C);
	dma_init_struct.DMA_PeripheralInc=DMA_PeripheralInc_Disable;
	dma_init_struct.DMA_PeripheralDataSize=DMA_PeripheralDataSize_HalfWord;
	dma_init_struct.DMA_Priority=DMA_Priority_Low;
	
	DMA_Init(DMA2_Channel5, &dma_init_struct);
	DMA_Cmd(DMA2_Channel5, ENABLE);
	adc_init_struct.ADC_Mode=ADC_Mode_Independent;
	adc_init_struct.ADC_DataAlign=ADC_DataAlign_Right;
	adc_init_struct.ADC_ExternalTrigConv=ADC_ExternalTrigConv_None;
	adc_init_struct.ADC_NbrOfChannel=1;
	adc_init_struct.ADC_ScanConvMode=DISABLE;
	adc_init_struct.ADC_ContinuousConvMode=ENABLE;
	ADC_Init(ADC3, &adc_init_struct);
	ADC_RegularChannelConfig(ADC3, ADC_Channel_10, 1, ADC_SampleTime_239Cycles5);
	ADC_DMACmd(ADC3, ENABLE);


	ADC_Cmd(ADC3, ENABLE);
	ADC_ResetCalibration(ADC3);
	while(ADC_GetResetCalibrationStatus(ADC3));
	ADC_StartCalibration(ADC3);
	while(ADC_GetCalibrationStatus(ADC3));
	ADC_SoftwareStartConvCmd(ADC3, ENABLE);
	
}
void Initial_All_GPIO(void)
{
	
	GPIO_InitTypeDef gpio_init_struct;
	//remap tim1
	GPIO_PinRemapConfig(GPIO_FullRemap_TIM1,ENABLE);
	GPIO_PinRemapConfig(GPIO_FullRemap_TIM3,ENABLE);
	//------------GPIO_A------------------
	//pa8,485mode,gpio
	gpio_init_struct.GPIO_Mode=GPIO_Mode_Out_PP;
	gpio_init_struct.GPIO_Pin=GPIO_Pin_8;
	gpio_init_struct.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&gpio_init_struct);
	//pa9,USART1_TX,
	gpio_init_struct.GPIO_Mode=GPIO_Mode_AF_PP;
	gpio_init_struct.GPIO_Pin=GPIO_Pin_9;
	gpio_init_struct.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&gpio_init_struct);
	//pa10,USART1_RX
	gpio_init_struct.GPIO_Mode=GPIO_Mode_IPU;
	gpio_init_struct.GPIO_Pin=GPIO_Pin_10;
	GPIO_Init(GPIOA,&gpio_init_struct);
	//gpioa,others,np
	gpio_init_struct.GPIO_Mode=GPIO_Mode_Out_PP;
	gpio_init_struct.GPIO_Pin=GPIO_Pin_1
	|GPIO_Pin_2
	|GPIO_Pin_3
	|GPIO_Pin_4
	|GPIO_Pin_5
	|GPIO_Pin_6
	|GPIO_Pin_7
	|GPIO_Pin_11
	|GPIO_Pin_12;
	gpio_init_struct.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&gpio_init_struct);
	//------------GPIO_B------------------
	//pb0,ov
	gpio_init_struct.GPIO_Mode=GPIO_Mode_IPU;
	gpio_init_struct.GPIO_Pin=GPIO_Pin_0;
	gpio_init_struct.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&gpio_init_struct);
	//p,temp,spi_cs
	gpio_init_struct.GPIO_Mode=GPIO_Mode_Out_PP;
	gpio_init_struct.GPIO_Pin=GPIO_Pin_1|
	GPIO_Pin_5|
	GPIO_Pin_6|
	GPIO_Pin_7|
	GPIO_Pin_8|
	GPIO_Pin_9|
	GPIO_Pin_10|
	GPIO_Pin_12;
	gpio_init_struct.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&gpio_init_struct);
	//pb13:spi_clk,pb15:spi_mosi
	gpio_init_struct.GPIO_Mode=GPIO_Mode_AF_PP;
	gpio_init_struct.GPIO_Pin=GPIO_Pin_13|GPIO_Pin_15;
	gpio_init_struct.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&gpio_init_struct);
	//pb14,spi_miso
	gpio_init_struct.GPIO_Mode=GPIO_Mode_IPU;
	gpio_init_struct.GPIO_Pin=GPIO_Pin_14;
	GPIO_Init(GPIOB,&gpio_init_struct);
	//------------GPIO_C------------------
	//pc0:ad,pc2:ad_iv,pc3:ad_iu
	gpio_init_struct.GPIO_Mode=GPIO_Mode_AIN;
	gpio_init_struct.GPIO_Pin=GPIO_Pin_0|GPIO_Pin_2|GPIO_Pin_3;
	GPIO_Init(GPIOC,&gpio_init_struct);
	//pc4:uv,pc5:xiefang,pc6:HU,pc7:HV,pc8:HW,PC10:JMP1,PC11:JMP2;
	gpio_init_struct.GPIO_Mode=GPIO_Mode_IPU;
	gpio_init_struct.GPIO_Pin=GPIO_Pin_4|
	GPIO_Pin_5|
	GPIO_Pin_6|
	GPIO_Pin_7|
	GPIO_Pin_8|
	GPIO_Pin_10|
	GPIO_Pin_11;
	GPIO_Init(GPIOC,&gpio_init_struct);
	//pc1:,pc9:,pc12:,pc13:np
	gpio_init_struct.GPIO_Mode=GPIO_Mode_Out_PP;
	gpio_init_struct.GPIO_Pin=GPIO_Pin_1|GPIO_Pin_9|GPIO_Pin_12|GPIO_Pin_13;
	gpio_init_struct.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOC,&gpio_init_struct);
	//------------GPIO_D------------------
	//pd0-pd5:keys
	gpio_init_struct.GPIO_Mode=GPIO_Mode_IPU;
	gpio_init_struct.GPIO_Pin=GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5;
	GPIO_Init(GPIOD,&gpio_init_struct);
	//pd6-pd11:np(pd8:flash_reset),pD12:EN_IPM,PD15:DS18B20
	gpio_init_struct.GPIO_Mode=GPIO_Mode_Out_PP;
	gpio_init_struct.GPIO_Pin=
	GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11
	|GPIO_Pin_13|GPIO_Pin_15;
	gpio_init_struct.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOD,&gpio_init_struct);
	//pd12:IPMFO,pd14:IPMOT(EXTI)
	gpio_init_struct.GPIO_Mode=GPIO_Mode_IPU;
	gpio_init_struct.GPIO_Pin=GPIO_Pin_12|GPIO_Pin_14;
	GPIO_Init(GPIOD,&gpio_init_struct);
	//------------GPIO_E------------------
	//pe0:stb,pe2:led,pe3:led,pe4-pe7,pe15:np
	gpio_init_struct.GPIO_Pin=GPIO_Pin_0|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_15;
	gpio_init_struct.GPIO_Mode=GPIO_Mode_Out_PP;
	gpio_init_struct.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOE,&gpio_init_struct);
	//pe8-pe14:pwm
	gpio_init_struct.GPIO_Mode=GPIO_Mode_AF_PP;
	gpio_init_struct.GPIO_Pin=GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14;
	gpio_init_struct.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOE,&gpio_init_struct);
	//pe1:OC
	gpio_init_struct.GPIO_Mode=GPIO_Mode_IN_FLOATING;
	gpio_init_struct.GPIO_Pin=GPIO_Pin_1;
	gpio_init_struct.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOE,&gpio_init_struct);
	
	
}


void Init_EXTI_NVIC(void)
{
	EXTI_InitTypeDef EXTI_init_struct;
	NVIC_InitTypeDef NVIC_init_struct;
	
	
	EXTI_init_struct.EXTI_Line=EXTI_Line1;
	EXTI_init_struct.EXTI_Mode=EXTI_Mode_Interrupt;
	EXTI_init_struct.EXTI_Trigger=EXTI_Trigger_Rising;
	EXTI_init_struct.EXTI_LineCmd=ENABLE;
	
	//OC interupt

	NVIC_init_struct.NVIC_IRQChannel=EXTI1_IRQn;
	NVIC_init_struct.NVIC_IRQChannelCmd=ENABLE;
	NVIC_init_struct.NVIC_IRQChannelPreemptionPriority=0;
	NVIC_init_struct.NVIC_IRQChannelSubPriority=0;
	
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOE, GPIO_PinSource1); 
	EXTI_Init(&EXTI_init_struct);
	NVIC_Init(&NVIC_init_struct);
	
	//ipm fo
	EXTI_init_struct.EXTI_Line=EXTI_Line12;
	EXTI_init_struct.EXTI_Mode=EXTI_Mode_Interrupt;
	EXTI_init_struct.EXTI_Trigger=EXTI_Trigger_Falling;
	EXTI_init_struct.EXTI_LineCmd=ENABLE;
	
	NVIC_init_struct.NVIC_IRQChannel=EXTI15_10_IRQn;
	NVIC_init_struct.NVIC_IRQChannelCmd=ENABLE;
	NVIC_init_struct.NVIC_IRQChannelPreemptionPriority=0;
	NVIC_init_struct.NVIC_IRQChannelSubPriority=1;
	
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOD, GPIO_PinSource12); 
	EXTI_Init(&EXTI_init_struct);
	NVIC_Init(&NVIC_init_struct);
	
}

void Init_USART_NVIC_TX_DMA(void)
{
	int i=0;
	USART_InitTypeDef USART_init_struct;
	NVIC_InitTypeDef NVIC_init_struct;
	DMA_InitTypeDef DMA_init_struct;
		
	GPIO_ResetBits(GPIOA,GPIO_Pin_8);
 
	NVIC_init_struct.NVIC_IRQChannel=DMA1_Channel4_IRQn;
	NVIC_init_struct.NVIC_IRQChannelPreemptionPriority=2;
	NVIC_init_struct.NVIC_IRQChannelSubPriority=1;
	NVIC_init_struct.NVIC_IRQChannelCmd=ENABLE;
	
	NVIC_Init(&NVIC_init_struct);
	
	USART_init_struct.USART_BaudRate=115200;
	USART_init_struct.USART_Mode=USART_Mode_Rx|USART_Mode_Tx;
	USART_init_struct.USART_WordLength=USART_WordLength_9b;//8data+1stop
	USART_init_struct.USART_Parity=USART_Parity_No;
	USART_init_struct.USART_StopBits=USART_StopBits_1;
	USART_init_struct.USART_HardwareFlowControl=USART_HardwareFlowControl_None;// bu zi dong kong zhi RTS CTS
	
	USART_Init(USART1,&USART_init_struct);
	//USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);//liu dao yi hou zai yong
	//USART_ITConfig(USART1,USART_IT_TXE,ENABLE);
	USART_Cmd(USART1,ENABLE);
	i=0;
	for(i=0;i<10000;i++);//just for delay
	GPIO_SetBits(GPIOA,GPIO_Pin_8);
	
	USART_GetFlagStatus(USART1, USART_FLAG_TC);//clear the TC bit
	
	DMA_init_struct.DMA_BufferSize=SEND_BUFFER_LENGTH;
	DMA_init_struct.DMA_DIR=DMA_DIR_PeripheralDST;	
	DMA_init_struct.DMA_M2M=DMA_M2M_Disable;
	DMA_init_struct.DMA_MemoryBaseAddr=(u32)(send_buffer);
	DMA_init_struct.DMA_MemoryDataSize=DMA_MemoryDataSize_Word;
	DMA_init_struct.DMA_MemoryInc=DMA_MemoryInc_Enable;
	DMA_init_struct.DMA_Mode=DMA_Mode_Normal;//guan bi xun huan mo shi,yi ci xing chuan shu
	DMA_init_struct.DMA_PeripheralBaseAddr=(u32)(&USART1->DR);
	DMA_init_struct.DMA_PeripheralDataSize=DMA_PeripheralDataSize_Byte;
	DMA_init_struct.DMA_PeripheralInc=DMA_PeripheralInc_Disable; 
	DMA_init_struct.DMA_Priority=DMA_Priority_Low;
	DMA_Init(DMA1_Channel4,&DMA_init_struct);//channel 4 USART1 TX
	
	
	
	DMA_Cmd(DMA1_Channel4,ENABLE);
	DMA_ITConfig(DMA1_Channel4,DMA_IT_TC,ENABLE);
	
	
	
	USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);	//how to send_buffer again?????
	

	
}

void Init_USART_NVIC_RX_INT(void)
{
	int i=0;
	USART_InitTypeDef USART_init_struct;
	NVIC_InitTypeDef NVIC_init_struct;
	USART_init_struct.USART_BaudRate=115200;
	USART_init_struct.USART_Mode=USART_Mode_Rx|USART_Mode_Tx;
	USART_init_struct.USART_WordLength=USART_WordLength_9b;//8data+1stop
	USART_init_struct.USART_Parity=USART_Parity_No;
	USART_init_struct.USART_StopBits=USART_StopBits_1;
	USART_init_struct.USART_HardwareFlowControl=USART_HardwareFlowControl_None;// bu zi dong kong zhi RTS CTS

	NVIC_init_struct.NVIC_IRQChannel=USART1_IRQn;
	NVIC_init_struct.NVIC_IRQChannelPreemptionPriority=2;
	NVIC_init_struct.NVIC_IRQChannelSubPriority=2;
	NVIC_init_struct.NVIC_IRQChannelCmd=ENABLE;
	
	NVIC_Init(&NVIC_init_struct);
	GPIO_ResetBits(GPIOA,GPIO_Pin_8);
	for(i=0;i<10000;i++);//just for delay
	USART_Init(USART1,&USART_init_struct);
	USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);//liu dao yi hou zai yong
	//USART_ITConfig(USART1,USART_IT_TXE,ENABLE);
	USART_Cmd(USART1,ENABLE);
	
	//USART_GetFlagStatus(USART1, USART_FLAG_TC);
		
}
void Init_TIM1_SQUARE(void)
{
	int period_tim1=(SystemCoreClock / PWM_FREQ) - 1;//pwm freq=15K

	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	TIM_BDTRInitTypeDef TIM_BDTRInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	pwm_generator.pwm_period=period_tim1;
	//time base
	TIM_DeInit(TIM1);
	TIM_TimeBaseStructure.TIM_Prescaler=0;
	TIM_TimeBaseStructure.TIM_ClockDivision=0;//ckd:dead-time&digital filter clk
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Down;//dai ding
	TIM_TimeBaseStructure.TIM_Period=period_tim1;
	TIM_TimeBaseStructure.TIM_RepetitionCounter=0;
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
	tim_time_Base_init_square=TIM_TimeBaseStructure;

	//time oc
	//idle state mao si bu ying xiang shu chu pwm shi de dian ping
	TIM_OCInitStructure.TIM_OCIdleState=TIM_OCIdleState_Reset;//OIS1 OIS1N//mei yong ????shen me shi hou you yong???
	TIM_OCInitStructure.TIM_OCMode=PWMMODE_TIM1;//??????
	TIM_OCInitStructure.TIM_OCNIdleState=TIM_OCIdleState_Reset;//mei yong ????shen me shi hou you yong???
	TIM_OCInitStructure.TIM_OCNPolarity=TIM_OCNPolarity_High;
	TIM_OCInitStructure.TIM_OCPolarity=TIM_OCPolarity_High;//the polarity in CCER(OCx in active state)
	TIM_OCInitStructure.TIM_OutputNState=TIM_OutputNState_Enable;
	TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse=1;
	
	
	TIM_OC1Init(TIM1, &TIM_OCInitStructure);
	tim_oc_init_square=TIM_OCInitStructure;
	TIM_OCInitStructure.TIM_Pulse=1;//just for test
	TIM_OC2Init(TIM1, &TIM_OCInitStructure);
	
	TIM_OCInitStructure.TIM_Pulse=1;//just for test
	TIM_OC3Init(TIM1, &TIM_OCInitStructure);

	TIM_OCInitStructure.TIM_OutputNState=TIM_OutputNState_Disable;
	TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Disable;
	TIM_OCInitStructure.TIM_Pulse=90;//just for test
	TIM_OC4Init(TIM1, &TIM_OCInitStructure);//how to configure oc4?????
	
	TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;//OSSR bit
	TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;//OSSI bit
	TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_OFF;//lock
	TIM_BDTRInitStructure.TIM_DeadTime = 50;
	//TIM_BDTRInitStructure.TIM_Break = TIM_Break_Enable;
	TIM_BDTRInitStructure.TIM_Break = TIM_Break_Disable;
	TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_High;
	TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;//?????
	
	TIM_OC1PreloadConfig(TIM1,TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM1,TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM1,TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM1,TIM_OCPreload_Enable);
	
	TIM_BDTRConfig(TIM1, &TIM_BDTRInitStructure);
	// preload ARR register
  TIM_CCPreloadControl(TIM1, ENABLE);
	TIM_SelectCOM(TIM1, ENABLE);
	TIM_SelectInputTrigger(TIM1, TIM_TS_ITR2);//smcr register 
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_TRG_COM_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	
	NVIC_Init(&NVIC_InitStructure);
	TIM_ClearITPendingBit(TIM1,TIM_IT_COM);
	TIM_ITConfig(TIM1, TIM_IT_COM, ENABLE);
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM1_UP_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	
  NVIC_Init(&NVIC_InitStructure);
	//TIM_ClearITPendingBit(TIM1,TIM_IT_CC1|TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC4|TIM_IT_Update);
	//TIM_ITConfig(TIM1,TIM_IT_CC1|TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC4|TIM_IT_Update,ENABLE);
	TIM_ClearITPendingBit(TIM1,TIM_IT_Update);
	TIM_ITConfig(TIM1,TIM_IT_Update,ENABLE);
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM1_CC_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	TIM_ClearITPendingBit(TIM1,TIM_IT_CC1|TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC4);
	//TIM_ITConfig(TIM1,TIM_IT_CC1|TIM_IT_CC2|TIM_IT_CC3,ENABLE);
	TIM_ClearITPendingBit(TIM1,TIM_IT_CC4);
	TIM_ITConfig(TIM1,TIM_IT_CC4,ENABLE);
	
	TIM_Cmd(TIM1, DISABLE);
	
	TIM_CtrlPWMOutputs(TIM1, DISABLE);

}
void Init_TIM1_FOC(void)
{
		
	int period_tim1=(SystemCoreClock / PWM_FREQ) - 1;//pwm freq=15K
	
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	TIM_BDTRInitTypeDef TIM_BDTRInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	pwm_generator.pwm_period=period_tim1;
	//time base
	TIM_DeInit(TIM1);
	TIM_TimeBaseStructure.TIM_Prescaler=0;
	TIM_TimeBaseStructure.TIM_ClockDivision=0;//ckd:dead-time&digital filter clk
	//TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_CenterAligned2;//dai ding
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_CenterAligned3;//1119
	TIM_TimeBaseStructure.TIM_Period=period_tim1;
	TIM_TimeBaseStructure.TIM_RepetitionCounter=0;
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

	tim_time_Base_init_foc=TIM_TimeBaseStructure;
	//time oc
	//idle state mao si bu ying xiang shu chu pwm shi de dian ping
	TIM_OCInitStructure.TIM_OCIdleState=TIM_OCIdleState_Reset;//OIS1 OIS1N//mei yong ????shen me shi hou you yong???
	TIM_OCInitStructure.TIM_OCMode=PWMMODE_TIM1;//??????
	TIM_OCInitStructure.TIM_OCNIdleState=TIM_OCIdleState_Reset;//mei yong ????shen me shi hou you yong???
	TIM_OCInitStructure.TIM_OCNPolarity=TIM_OCNPolarity_High;
	TIM_OCInitStructure.TIM_OCPolarity=TIM_OCPolarity_High;//the polarity in CCER(OCx in active state)
	TIM_OCInitStructure.TIM_OutputNState=TIM_OutputNState_Enable;
	TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse=1;
	
	tim_oc_init_foc=TIM_OCInitStructure;
	
	TIM_OC1Init(TIM1, &TIM_OCInitStructure);
	
	TIM_OCInitStructure.TIM_Pulse=1;//just for test
	TIM_OC2Init(TIM1, &TIM_OCInitStructure);
	
	TIM_OCInitStructure.TIM_Pulse=1;//just for test
	TIM_OC3Init(TIM1, &TIM_OCInitStructure);

	TIM_OCInitStructure.TIM_OutputNState=TIM_OutputNState_Disable;
	TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Disable;
	//TIM_OCInitStructure.TIM_Pulse=(period_tim1-35);//2013.10.31
	TIM_OCInitStructure.TIM_Pulse=(period_tim1/2);//1119
	TIM_OC4Init(TIM1, &TIM_OCInitStructure);//how to configure oc4?????
	
	TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;//OSSR bit
	TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;//OSSI bit
	TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_OFF;//lock
	//TIM_BDTRInitStructure.TIM_DeadTime = 50;//1us
	TIM_BDTRInitStructure.TIM_DeadTime=150;
	//TIM_BDTRInitStructure.TIM_DeadTime = 250;//14us
	//TIM_BDTRInitStructure.TIM_Break = TIM_Break_Enable;
	TIM_BDTRInitStructure.TIM_Break = TIM_Break_Disable;
	TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_High;
	TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;//?????
	
	TIM_OC1PreloadConfig(TIM1,TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM1,TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM1,TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM1,TIM_OCPreload_Enable);
	
	TIM_BDTRConfig(TIM1, &TIM_BDTRInitStructure);
	// preload ARR register
  TIM_CCPreloadControl(TIM1, ENABLE);
	TIM_SelectCOM(TIM1, ENABLE);
	TIM_SelectInputTrigger(TIM1, TIM_TS_ITR2);//smcr register 
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_TRG_COM_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	
	NVIC_Init(&NVIC_InitStructure);

	
	NVIC_InitStructure.NVIC_IRQChannel=TIM1_UP_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	
  NVIC_Init(&NVIC_InitStructure);

	TIM_ClearITPendingBit(TIM1,TIM_IT_COM);
	TIM_ITConfig(TIM1, TIM_IT_COM, ENABLE);
	//TIM_ClearITPendingBit(TIM1,TIM_IT_CC1|TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC4|TIM_IT_Update);
	//TIM_ITConfig(TIM1,TIM_IT_CC1|TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC4|TIM_IT_Update,ENABLE);
	TIM_ClearITPendingBit(TIM1,TIM_IT_Update);
	TIM_ITConfig(TIM1,TIM_IT_Update,ENABLE);
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM1_CC_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	TIM_ClearITPendingBit(TIM1,TIM_IT_CC1|TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC4);
	//TIM_ITConfig(TIM1,TIM_IT_CC1|TIM_IT_CC2|TIM_IT_CC3,ENABLE);
	TIM_ClearITPendingBit(TIM1,TIM_IT_CC4);
	TIM_ITConfig(TIM1,TIM_IT_CC4,ENABLE);
	
	TIM_Cmd(TIM1, DISABLE);
	
	TIM_CtrlPWMOutputs(TIM1, DISABLE);
	


}
void Init_TIM1(void)
{
	int period_tim1=(SystemCoreClock / PWM_FREQ) - 1;//pwm freq=15K
	
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	TIM_BDTRInitTypeDef TIM_BDTRInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	pwm_generator.pwm_period=period_tim1;
	//time base
	TIM_TimeBaseStructure.TIM_Prescaler=0;
	TIM_TimeBaseStructure.TIM_ClockDivision=0;//ckd:dead-time&digital filter clk
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Down;//dai ding
	TIM_TimeBaseStructure.TIM_Period=period_tim1;
	TIM_TimeBaseStructure.TIM_RepetitionCounter=0;
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

	//time oc
	//idle state mao si bu ying xiang shu chu pwm shi de dian ping
	TIM_OCInitStructure.TIM_OCIdleState=TIM_OCIdleState_Reset;//OIS1 OIS1N//mei yong ????shen me shi hou you yong???
	TIM_OCInitStructure.TIM_OCMode=PWMMODE_TIM1;//??????
	TIM_OCInitStructure.TIM_OCNIdleState=TIM_OCIdleState_Reset;//mei yong ????shen me shi hou you yong???
	TIM_OCInitStructure.TIM_OCNPolarity=TIM_OCNPolarity_High;
	TIM_OCInitStructure.TIM_OCPolarity=TIM_OCPolarity_High;//the polarity in CCER(OCx in active state)
	TIM_OCInitStructure.TIM_OutputNState=TIM_OutputNState_Enable;
	TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse=1;
	
	
	TIM_OC1Init(TIM1, &TIM_OCInitStructure);
	
	TIM_OCInitStructure.TIM_Pulse=1;//just for test
	TIM_OC2Init(TIM1, &TIM_OCInitStructure);
	
	TIM_OCInitStructure.TIM_Pulse=1;//just for test
	TIM_OC3Init(TIM1, &TIM_OCInitStructure);

	TIM_OCInitStructure.TIM_OutputNState=TIM_OutputNState_Disable;
	TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Disable;
	TIM_OCInitStructure.TIM_Pulse=90;//just for test
	TIM_OC4Init(TIM1, &TIM_OCInitStructure);//how to configure oc4?????
	
	TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;//OSSR bit
	TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;//OSSI bit
	TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_OFF;//lock
	TIM_BDTRInitStructure.TIM_DeadTime = 50;
	//TIM_BDTRInitStructure.TIM_Break = TIM_Break_Enable;
	TIM_BDTRInitStructure.TIM_Break = TIM_Break_Disable;
	TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_High;
	TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;//?????
	
	TIM_OC1PreloadConfig(TIM1,TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM1,TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM1,TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM1,TIM_OCPreload_Enable);
	
	TIM_BDTRConfig(TIM1, &TIM_BDTRInitStructure);
	// preload ARR register
  TIM_CCPreloadControl(TIM1, ENABLE);
	TIM_SelectCOM(TIM1, ENABLE);
	TIM_SelectInputTrigger(TIM1, TIM_TS_ITR2);//smcr register 
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_TRG_COM_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	
	NVIC_Init(&NVIC_InitStructure);
	TIM_ClearITPendingBit(TIM1,TIM_IT_COM);
	TIM_ITConfig(TIM1, TIM_IT_COM, ENABLE);
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM1_UP_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	
  NVIC_Init(&NVIC_InitStructure);
	//TIM_ClearITPendingBit(TIM1,TIM_IT_CC1|TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC4|TIM_IT_Update);
	//TIM_ITConfig(TIM1,TIM_IT_CC1|TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC4|TIM_IT_Update,ENABLE);
	TIM_ClearITPendingBit(TIM1,TIM_IT_Update);
	TIM_ITConfig(TIM1,TIM_IT_Update,ENABLE);
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM1_CC_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	TIM_ClearITPendingBit(TIM1,TIM_IT_CC1|TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC4);
	//TIM_ITConfig(TIM1,TIM_IT_CC1|TIM_IT_CC2|TIM_IT_CC3,ENABLE);
	TIM_ClearITPendingBit(TIM1,TIM_IT_CC4);
	TIM_ITConfig(TIM1,TIM_IT_CC4,ENABLE);
	
	TIM_Cmd(TIM1, DISABLE);
	
	TIM_CtrlPWMOutputs(TIM1, DISABLE);

}
void Init_TIM2(void)
{
	int period_tim1=(SystemCoreClock / 3000) - 1;//zhuan su:500hz
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
 
	//time base
	TIM_TimeBaseStructure.TIM_Prescaler=3;//zan ding
	TIM_TimeBaseStructure.TIM_ClockDivision=0;//ckd:dead-time&digital filter clk
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;//dai ding
	TIM_TimeBaseStructure.TIM_Period=period_tim1;
	TIM_TimeBaseStructure.TIM_RepetitionCounter=0;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	TIM_ClearITPendingBit(TIM2,TIM_IT_Update);
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
   
	TIM_Cmd(TIM2, DISABLE);

} 
void Init_TIM3(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	TIM_TimeBaseStructure.TIM_ClockDivision=0;
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period=65535;
	TIM_TimeBaseStructure.TIM_Prescaler=1000;
	//about 1s
	TIM_TimeBaseStructure.TIM_RepetitionCounter=0;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	//TIM_PrescalerConfig(TIM3, 0, TIM_PSCReloadMode_Immediate);
	TIM_SelectHallSensor(TIM3, ENABLE); 
	TIM_SelectInputTrigger(TIM3, TIM_TS_TI1F_ED);//TI1 Edge Detector 
	
	TIM_SelectSlaveMode(TIM3, TIM_SlaveMode_Reset); //// On every TI1F_ED event the counter is resetted and update is tiggered 
	
	TIM_ICInitStructure.TIM_Channel=TIM_Channel_1;
	TIM_ICInitStructure.TIM_ICPolarity=TIM_ICPolarity_BothEdge;//TIM_ICPolarity_Rising;??????Falling??????
	TIM_ICInitStructure.TIM_ICFilter=0x2;
	TIM_ICInitStructure.TIM_ICPrescaler=TIM_ICPSC_DIV1;//Capture performed each time an edge is detected on the capture input. 
	TIM_ICInitStructure.TIM_ICSelection=TIM_ICSelection_TRC;//TIM Input 1, 2, 3 or 4 is selected to be connected to TRC. 
	TIM_ICInit(TIM3, &TIM_ICInitStructure); 
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; 
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //shou dong chan sheng com 
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; 
  TIM_OCInitStructure.TIM_Pulse = 0; // 1 is no delay; 2000 = 7ms
  TIM_OC2Init(TIM3, &TIM_OCInitStructure); 
	TIM_SelectOCxM(TIM3,TIM_Channel_2,TIM_OCMode_Inactive);
	
	// du zhuan jian ce
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; 
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable; 
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; 
  TIM_OCInitStructure.TIM_Pulse = 10000; 
  TIM_OC3Init(TIM3, &TIM_OCInitStructure); 
	
	// timer2 output compate signal is connected to TRIGO 
  NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn; 
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; 
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3; 
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
  NVIC_Init(&NVIC_InitStructure); 	
	
	TIM_SelectOutputTrigger(TIM3, TIM_TRGOSource_OC2Ref); 
	TIM_ClearITPendingBit(TIM3,TIM_IT_Trigger|TIM_IT_CC1|TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC4|TIM_IT_Update);
	TIM_ITConfig(TIM3, TIM_IT_Trigger|TIM_IT_CC3, ENABLE);
	TIM_SetCounter(TIM3,1);
	//TIM_UpdateRequestConfig(TIM3,TIM_UpdateSource_Regular );
	TIM_Cmd(TIM3, DISABLE);
}



void MCU_Device_Initial(void)
{
	RCC_config();
	Init_SysTick();
	Initial_All_GPIO();
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//zan ding
	Init_EXTI_NVIC();
	

	Init_TIM3();
	Init_TIM2();
	Init_TIM1_FOC();
	ADC_config();
	Init_DAC();
	
	Init_USART_NVIC_TX_DMA();
	
#ifdef RAM_MODE
	NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0); 
#endif
	DBGMCU_Config(DBGMCU_TIM1_STOP,ENABLE);
	DBGMCU_Config(DBGMCU_TIM2_STOP,ENABLE);
	DBGMCU_Config(DBGMCU_TIM3_STOP,ENABLE);
	
}

