#include "rtos_init.h"

#include "utilities_debug.h"
#include "drivers_canmotor_low.h"
//#include "drivers_mpu6050_low.h"
#include "drivers_uartupper_low.h"
#include "drivers_uartrc_low.h"

#include "drivers_imu_low.h"
#include "utilities_tim.h"

#include "drivers_buzzer_low.h"
#include "application_remotecontrol.h"
#include "drivers_uartjudge_low.h"
uint8_t isInited = 0;
void rtos_init(){
  //wait for devices
//	for(int i=0; i < 3000; i++)
//	{
//		int a=42000; //at 168MHz 42000 is ok
//		while(a--);
//	}
//	playMusicWhenInit();
	
	fw_userTimeEnable();
	MPU6500_Init();
	IST8310_Init();
	rcInit();
	RemoteTaskInit();
	ctrlUartInit();
	InitJudgeUart();//初始化裁判系统读取串口
	motorInit();
//	mpu6050Init();
//	Init_Quaternion();
	fw_printfln("init success");
}
