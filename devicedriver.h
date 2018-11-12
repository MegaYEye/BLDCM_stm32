#ifndef _DEVICE_DRIVER_H
#define _DEVICE_DRIVER_H
void MCU_Device_Initial(void);

void Initial_All_GPIO(void);

void RCC_config(void);

void ADC_config(void);

void Init_TIM1(void);

void Init_TIM2(void);

void Init_TIM3(void);

void Init_USART_NVIC_TX_DMA(void);

void Init_USART_NVIC_RX_INT(void);

void Init_SysTick(void);

void Init_EXTI_NVIC(void);

void Init_DAC(void);


#endif
