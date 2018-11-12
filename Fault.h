#ifndef _FAULT_H
#define _FAULT_H
#include "defines.h"
void OC_Process(void);
void IPMFT_Process(void);
void Fault_Regular_Scan(void);
void Set_HALL_Fault(void);
void Set_Temp_Fault(void);
void Set_Start_Fault(void);
void Set_Overload_Fault(void);
void Set_Isensor_Fault(void);
void Set_Over_Speed_Fault(void);
void Set_TingJi_Fault(void);
#endif
