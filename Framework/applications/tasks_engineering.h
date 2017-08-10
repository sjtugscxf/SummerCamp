#ifndef __TASKS_ENGINEERING_H
#define __TASKS_ENGINEERING_H

#include "stdint.h"
typedef enum
{
	ENGINEER_NO_MOVE = 1,
	ENGINEER_ADJUSTING = 2,
	ENGINEER_DISTANCE_OK = 3,
	ENGINEER_DISTANCE_AND_ROTATION_OK = 4,
	ENGINEER_PLACING = 5,
	ENGINEER_FETCHING = 6,
	ENGINEER_PLACING_COMPLETE =7,
	ENGINEER_FETCHING_COMPLETE =8,
	ENGINEER_DISCASDING =9,
	ENGINEER_REPLACING =10,
	ENGINEER_GRABING =11,//̧�ߣ������ץס��״̬��û�п����Ƿ�ץס��
	
	ENGINEER_MANUAL_PLACING=12,//�ֶ�����ץȡʱ���˶������е�״̬���˶���ɺ��״̬ΪENGINEER_GRABING
	ENGINEER_MANUAL_FETCHING=13,//�ֶ�����ץȡ���˶�������
	ENGINEER_MANUAL_STRETCHED=14,//�ֶ�ץȡ�˶�������״̬
	
	ENGINEER_BELT_MOVING=15,//Ƥ�����ֶ�����
	
	ENGINEER_RECOVERING=16,
}Engineer_State_t;

typedef enum
{
	//STOPֻ�����ⲿ��λ(rc)�����ڲ����������λ,
	ENGINEER_STOP = 1,
	ENGINEER_ADJUSTDISTANCE=2,
	ENGINEER_ADJUSTDISTANCE_AND_ROTATION=3,
	ENGINEER=4,
	ENGINEER_DISCARD_STUFF=5,
	ENGINEER_REPLACE_STUFF=6,
	//���������ڲ�������pause����λ
	ENGINEER_PAUSE=7,
	
	//�ֶ���������
	ENGINEER_MANUAL_PLACE=8,
	ENGINEER_MANUAL_FETCH=9,
	//������
	ENGINEER_STANDBY=10,

	//�ֶ�Ƥ�������
	ENGINEER_BELT_BACK=11,
	ENGINEER_BELT_FORWARD=12,
	
}Engineer_Order_t;

extern Engineer_State_t Engineering_State;
extern Engineer_Order_t Engineering_Order;
//extern uint8_t engineering_task_on;
//�ⲿ����
uint8_t engineer_grab_somthing();
void StartNewPlaceTask();
void StartNewFetchTask();
void StopEngineerTask();
void EngineeringTask(void const * argument);
void OrderReplace();
void EngineerPrintState();
void SetFetchHeight(uint8_t HeightIndex);
#endif

