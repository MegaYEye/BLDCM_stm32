
#include "stm32f10x.h"
#include "DeviceDriver.h"
#include "Operation.h"
#include "defines.h"
#include "Myglobal.h"


void test(void);

int main()
{
 	MCU_Device_Initial();
	Init_variables();
	Code_Entrance();
	while(1)
	{
			;
	}
	/*you dai chang shi:
	speed:
	kp=17/1
	ki=5/1
	current:
	kp=10/2
	ki=1/30
	
	*/
}

















void test()
{
	;
}




