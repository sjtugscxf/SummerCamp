#include "tasks_motor.h"
#include "drivers_canmotor_user.h"
#include "rtos_semaphore.h"

#include "utilities_debug.h"
#include "tasks_upper.h"
#include "drivers_led_user.h"
#include "utilities_minmax.h"
#include "drivers_uartrc_user.h"
#include "drivers_sonar_user.h"
#include "drivers_imu_user.h"
#include "application_pidfunc.h"
#include "application_setmotor.h"
#include "application_gimbalcontrol.h"
#include "application_chassiscontrol.h"
#include "application_auxmotorcontrol.h"
#include "application_remotecontrol.h"
#include "stdint.h"
#include "utilities_tim.h"
#include "drivers_uartjudge_low.h"

extern float ZGyroModuleAngle;
extern float angles[3];
// ��1��ʱ����̨�ſ�ʼ���ƣ���0������
uint8_t GM_RUN=0;
// Ӣ�۵�����
int8_t flUpDown = 0, frUpDown = 0, blUpDown = 0, brUpDown = 0, allUpDown = 0;
//��λ�Ƕ�
float yawAngleTarget = 0.0;
float pitchAngleTarget = 0.0;
float pitchZeroAngle = 0;
//�����ٶȽṹ�壬��ÿ��
ChassisSpeed_Ref_t ChassisSpeedRef;
FFollowLoc_t FFollowLoc;
//test
float target_dir=0;
void CMGMControlTask(void const * argument){
	while(1){
		osSemaphoreWait(CMGMCanRefreshSemaphoreHandle, osWaitForever);
//	 if((GetWorkState() == STOP_STATE)  || GetWorkState() == CALI_STATE || GetWorkState() == PREPARE_STATE || GetEmergencyFlag() == EMERGENCY)   //||Is_Serious_Error()|| dead_lock_flag == 1����ͣ����������У׼���޿�������ʱ����ʹ���̿���ֹͣ
//	 {
//		 //fw_printf("motor state error\r\n");
//		 yawAngleTarget=0;
//		 pitchAngleTarget=0;
//		 ChassisSpeedRef.forward_back_ref=0;
//		 ChassisSpeedRef.left_right_ref=0;
//		 ChassisSpeedRef.rotate_ref=0;
//		 target_dir=angles[0];
//	 }
//		static int32_t cnt=0;
//	 if(GM_RUN)
//	 {
//		 MINMAX(yawAngleTarget, YAWDOWNLIMIT, YAWUPLIMIT);
//		 MINMAX(pitchAngleTarget, PITCHDOWNLIMIT, PITCHUPLIMIT);
//		 setYawWithAngle(yawAngleTarget);
//		 setPitchWithAngle(pitchAngleTarget);
//	 }
		if(MODE_SWITCH == FOLLOW_MODE)
		{
			FFollowLoc.distance = (float)FollowLoc.distance;
			FFollowLoc.rotate = (float)FollowLoc.rotate;
			//FFollowLoc.traverse = (float)FollowLoc.traverse;
			
			ChassisSpeedRef.forward_back_ref = DISTANCE_KP * (FOLLOW_DISTANCE - FollowLoc.distance);
			if(ChassisSpeedRef.forward_back_ref>1500) ChassisSpeedRef.forward_back_ref = 1500;
			if(ChassisSpeedRef.forward_back_ref<-1500) ChassisSpeedRef.forward_back_ref = -1500;
			
			ChassisSpeedRef.rotate_ref = ROTATE_KP * (FOLLOW_ROTATE - FollowLoc.rotate);
			if(ChassisSpeedRef.rotate_ref>1500) ChassisSpeedRef.rotate_ref = 1500;
			if(ChassisSpeedRef.rotate_ref<-1500) ChassisSpeedRef.rotate_ref = -1500;
			
			if(FFollowLoc.traverse>0)
			{
				ChassisSpeedRef.left_right_ref = 1000;
				FFollowLoc.traverse -- ;
			}
			else if(FFollowLoc.traverse<0)
			{
				ChassisSpeedRef.left_right_ref = -1000;
				FFollowLoc.traverse ++ ;
			}
			else ChassisSpeedRef.left_right_ref = 0;
		}
		setChassisWithSpeed(ChassisSpeedRef.forward_back_ref, ChassisSpeedRef.left_right_ref, ChassisSpeedRef.rotate_ref);
	}
}

///////////////
//aux1: left belt
//aux2: right belt
//aux3: left lift
//aux4: right lift
//(aux1 and aux2): speed control
//(aux3 and aux4): speed and position control
double aux_motor34_position_target=0;//left lift

double aux_motor3_zero_angle=0;
double aux_motor4_zero_angle=0;

float aux1_targetSpeed=0;
float aux2_targetSpeed=0;

double plate_angle_target=0; //aux5
double plate_zero_angle=0;

double getBullet_angle_target=0;//aux6
double getBullet_zero_angle=0;

double aux34_limit = 38500;
double getBullet_limit=56000;
uint8_t aux_run=0;
void AMControlTask(void const * argument){
	while(1){
		osSemaphoreWait(AMCanRefreshSemaphoreHandle, osWaitForever);
		
		 if(GetWorkState() == PREPARE_STATE)
		 {
			 IOPool_getNextRead(AMUDBLRxIOPool, 0);
			 IOPool_getNextRead(AMUDBRRxIOPool, 0);
			 IOPool_getNextRead(AMPLATERxIOPool, 0);
			 IOPool_getNextRead(AMGETBULLETRxIOPool, 0);
			 aux_motor3_zero_angle = (IOPool_pGetReadData(AMUDBLRxIOPool, 0)->angle) * 360 / 8192.0;
			 aux_motor4_zero_angle = (IOPool_pGetReadData(AMUDBRRxIOPool, 0)->angle) * 360 / 8192.0;
			 plate_zero_angle=(IOPool_pGetReadData(AMPLATERxIOPool, 0)->angle) * 360 / 8192.0;
			 getBullet_zero_angle = (IOPool_pGetReadData(AMGETBULLETRxIOPool, 0)->angle) * 360 / 8192.0;
		 }
		 
		 if((GetWorkState() == STOP_STATE)  || GetWorkState() == CALI_STATE || GetWorkState() == PREPARE_STATE || GetEmergencyFlag() == EMERGENCY)   //||Is_Serious_Error()|| dead_lock_flag == 1����ͣ����������У׼���޿�������ʱ����ʹ���̿���ֹͣ
		 {
	//			 aux_motor1_position_target=0;
	//			 aux_motor2_position_target=0;
	//			 aux_motor3_position_target=0;
	//			 aux_motor4_position_target=0;
			 //aux1_targetSpeed=0;
			 //aux2_targetSpeed=0;
			 //continue;
		 }
		 
		 aux1_targetSpeed=(-ChassisSpeedRef.forward_back_ref-ChassisSpeedRef.rotate_ref)*2;
		 aux2_targetSpeed=(+ChassisSpeedRef.forward_back_ref-ChassisSpeedRef.rotate_ref)*2;
		 setAux1WithSpeed(aux1_targetSpeed);
		 setAux2WithSpeed(aux2_targetSpeed);
///
		 if(GetWorkState() == NORMAL_STATE)
		 {
//			 MINMAX(aux_motor1_position_target,aux1_limit,0);
//			 MINMAX(aux_motor2_position_target,aux2_limit,0);
			 MINMAX(aux_motor34_position_target,0,aux34_limit);
			MINMAX(getBullet_angle_target,0,getBullet_limit);
//			 setAux1WithAngle(aux_motor1_position_target+aux_motor1_zero_angle);
//			 setAux2WithAngle(aux_motor2_position_target+aux_motor2_zero_angle);
			 setAux3WithAngle(aux_motor34_position_target+aux_motor3_zero_angle);
			 setAux4WithAngle(-aux_motor34_position_target+aux_motor4_zero_angle);

			 setPlateWithAngle(plate_angle_target+plate_zero_angle);
			 setGetBulletWithAngle(getBullet_angle_target+getBullet_zero_angle);
		}
	}
}

void ShootOnce()
{
	static uint64_t last_shoot_time=0;
	uint64_t t=fw_getTimeMicros();
	if(t-last_shoot_time>200000)
	{
		if(redir_mode == 1) plate_angle_target+=90.0*95.8;
		else plate_angle_target-=90.0*95.8;
	}
	last_shoot_time=t;
	SetShootState(NOSHOOTING);
}
