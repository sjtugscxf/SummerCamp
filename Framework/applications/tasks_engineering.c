#include "tasks_engineering.h"
#include "stdint.h"
#include "tasks_motor.h"
#include "cmsis_os.h"
#include "application_pidfunc.h"
#include "stdlib.h"
#include "utilities_debug.h"
#include "drivers_uartupper_user.h"
#include "drivers_canmotor_user.h"
#include "application_auxmotorcontrol.h"
uint8_t engineer_task_on=0;

//Engineering_Step_t Engineering_Step = ENGINEERING_STOP;
//Engineering_Step_t Engineering_Last_Step = ENGINEERING_STOP;
//״̬������ö��
Engineer_Order_t Engineering_Order=ENGINEER_STANDBY;
Engineer_State_t Engineering_State=ENGINEER_NO_MOVE;

int32_t fetch_height=-11000;//-1000�ڶ���

extern uint32_t ADC_Value[60];
uint32_t targetValue2=1750;

uint32_t ad2_bias=0;
uint32_t ad3_bias=100;//��ad1Ϊ��׼

uint32_t ad6_bias=0;//��ad5Ϊ��׼
uint32_t ad7_bias=0;//��ad4Ϊ��׼
//�������pid�������ң������ͬһ����λ
PID_Regulator_t Engineer_ForwardBackPID = PID_INIT(0.5, 0.0, 0.0, 150, 150, 150, 50);
PID_Regulator_t Engineer_LeftRightPID = PID_INIT(0.5, 0.0, 0.0, 150, 150, 150, 50);
PID_Regulator_t Engineer_RotatePID = PID_INIT(0.5, 0.0, 0.0, 150, 150, 150, 50);
//�ڲ���������
void calibrationStep();
uint8_t Engineering_Grab_and_Place();
void Engineering_Adjustdistance();
void Engineering_AdjustRotationAndDistance();
void Engineering_Recover();
void Engineer_Discard_Stuff();
uint8_t Engineer_Replacing();
uint8_t Engineer_manual_fetch();
uint8_t Engineer_manual_place();
uint8_t engineer_grab_somthing();
uint8_t engineer_belt_back();
uint8_t engineer_belt_forward();
void EngineeringTask(void const * argument){
	
	while(1){
			switch(Engineering_Order)
			{
				case ENGINEER:
				{
					if(!Engineering_Grab_and_Place())
					{
						//��ͣ
						Engineering_Order=ENGINEER_PAUSE;
						fw_printfln("ENGINEER TASK END!");
					}
				}break;
				case ENGINEER_ADJUSTDISTANCE:
				{
					Engineering_Adjustdistance();
				}break;
				case ENGINEER_ADJUSTDISTANCE_AND_ROTATION:
				{
					Engineering_AdjustRotationAndDistance();
				}break;
				case ENGINEER_STOP:
				{
					fw_printfln("stop!");
					Engineering_Recover();
				}break;
				case ENGINEER_DISCARD_STUFF:
				{
					fw_printfln("Now Discard!");
					Engineer_Discard_Stuff();
				}break;
				case ENGINEER_REPLACE_STUFF:
				{
					fw_printfln("Now Replace!");
					Engineer_Replacing();
				}break;
				case ENGINEER_MANUAL_PLACE:
				{
					if(!Engineer_manual_place())
					{
						//��ͣ
						Engineering_Order=ENGINEER_PAUSE;
						fw_printfln("ENGINEER TASK END!");
					}
				}break;
				case ENGINEER_MANUAL_FETCH:
				{
					if(!Engineer_manual_fetch())
					{
						//��ͣ
						Engineering_Order=ENGINEER_PAUSE;
						fw_printfln("ENGINEER TASK END!");
					}
				}break;
				case ENGINEER_PAUSE:
				{
					fw_printfln("pause!");
					Engineering_Recover();
				}break;
				case ENGINEER_BELT_BACK:
				{
					fw_printfln("belt back");
					if(!engineer_belt_back())
						fw_printfln("belt not move");
				}break;
				case ENGINEER_BELT_FORWARD:
				{
					fw_printfln("back forward");
					if(!engineer_belt_forward())
						fw_printfln("belt not move");
				}break;
				case ENGINEER_STANDBY:
				{
					osDelay(10);
				}break;
				
			}
			
			osDelay(2);
	}
		
}
//���̳�������������
void Engineering_Adjustdistance()
{
		static int32_t closecnt=0;
		//fw_printfln("1");
		//fw_printf("engineering_adjusting!\r\n");
		////////////ע�⣡
		////ad1 �ұߣ�ad2���м䣬ad3�����
		int32_t ad1=0,ad2=0,ad3=0;
		for(uint16_t i=0;i<140;i++)
		{
			if(i%7==0)ad1+=ADC_Value[i];
			if(i%7==1)ad2+=ADC_Value[i];
			if(i%7==2)ad3+=ADC_Value[i];
//			if(i%6==4)ad5+=ADC_Value[i];
		}
		ad1/=20;
		ad2/=20;
		ad3/=20;
		ad2-=ad2_bias;
		ad3-=ad3_bias;
//		ad5/=20;
//		if(ad5>1000)
//		{
//			//�����з���
//			Engineering_Order=ENGINEER_ADJUSTDISTANCE_AND_ROTATION;
//			return;
//		}
		if(ad2<400&&ad1<400&&ad3<400)
		{
			osDelay(10);
			Engineering_Order=ENGINEER_STANDBY;
			return;
		}
		if(abs(ad2-targetValue2)<30 && abs(ad1-ad3)<30)
		{
			closecnt++;
		}
		else
		{
			closecnt--;
		}
		if(closecnt>60)
			closecnt=60;
		else if(closecnt<0)
			closecnt=0;
		if(closecnt>40)
		{
			fw_printf("close! cnt:%d\r\n",closecnt);
			fw_printf("%d  %d  %d",ad1,ad2,ad3);
			Engineering_State=ENGINEER_DISTANCE_OK;
			Engineering_Order=ENGINEER;
			closecnt=0;
			osDelay(20);
			return;
		}
		Engineering_State=ENGINEER_ADJUSTING;
		//PID
		Engineer_ForwardBackPID.target=targetValue2;
		Engineer_ForwardBackPID.feedback=ad2;
		Engineer_ForwardBackPID.Calc(&Engineer_ForwardBackPID);
		
		Engineer_LeftRightPID.target=0;
		Engineer_LeftRightPID.feedback= ad3-ad1;
		Engineer_LeftRightPID.Calc(&Engineer_LeftRightPID);
		
		ChassisSpeedRef.forward_back_ref=Engineer_ForwardBackPID.output/66.0*3000;
		ChassisSpeedRef.left_right_ref=Engineer_LeftRightPID.output/66.0*4000;
}
//���̳������Ƕ�
uint32_t targetValue5=1400;
void Engineering_AdjustRotationAndDistance()
{
		static int32_t rotate_closecnt=0;
		static int32_t left_right_closecnt=0;
		//fw_printfln("2");
//		fw_printfln("en");
		//ad4 �ұߣ�ad5��ad6�м䣬ad7�����
		int32_t ad4=0,ad5=0,ad6=0,ad7=0;
		for(uint16_t i=0;i<120;i++)
		{
			if(i%7==3)ad4+=ADC_Value[i];
			if(i%7==4)ad5+=ADC_Value[i];
			if(i%7==5)ad6+=ADC_Value[i];
			if(i%7==6)ad7+=ADC_Value[i];
		}
		ad4/=20;
		ad5/=20;
		ad6/=20;
		ad7/=20;
		ad6-=ad6_bias;
		ad7-=ad7_bias;
		if(ad5<400&&ad6<400)
		{
			osDelay(10);
			return;
		}
		if(abs(ad5-ad6)<80 && abs(ad5-targetValue5)<40 && abs(ad4-ad7)<150)
		{
			rotate_closecnt++;
		}
		else
		{
			rotate_closecnt--;
		}
		//fw_printfln("%d",rotate_closecnt);
		if(rotate_closecnt>60)
			rotate_closecnt=60;
		else if(rotate_closecnt<0)
			rotate_closecnt=0;
		if(rotate_closecnt>40)
		{
			fw_printf("close! cnt:%d\r\n",rotate_closecnt);
			fw_printf("%d  %d  %d  %d",ad4,ad5,ad6,ad7);
			Engineering_State=ENGINEER_DISTANCE_AND_ROTATION_OK;
			Engineering_Order=ENGINEER;
			rotate_closecnt=0;
			osDelay(20);
			return;
		}

		Engineering_State=ENGINEER_ADJUSTING;
		//PID
		Engineer_RotatePID.target=0;

		Engineer_RotatePID.feedback=ad6-ad5;
		Engineer_RotatePID.Calc(&Engineer_RotatePID);
		
		Engineer_ForwardBackPID.target=targetValue5;
		Engineer_ForwardBackPID.feedback=ad5;
		Engineer_ForwardBackPID.Calc(&Engineer_ForwardBackPID);
		
		Engineer_LeftRightPID.target=0;
		Engineer_LeftRightPID.feedback=ad7-ad4;
		Engineer_LeftRightPID.Calc(&Engineer_LeftRightPID);
		
		ChassisSpeedRef.forward_back_ref=Engineer_ForwardBackPID.output/66.0*2000;
		ChassisSpeedRef.rotate_ref=Engineer_RotatePID.output/66.0*1800;
		ChassisSpeedRef.left_right_ref=Engineer_LeftRightPID.output/66.0*1500;
		
}
//���̳�ץȡ��������
//���ú���
uint8_t Engineering_Lift(float value, uint32_t time_milis);
uint8_t Engineering_Stretch(float value, uint32_t time_milis);
uint8_t Engineering_Grab(float value, uint32_t time_milis);
uint8_t taskDelay(uint32_t time_milis);
uint8_t Engineering_Grab_and_Place()
{
		ChassisSpeedRef.forward_back_ref=0;
		ChassisSpeedRef.left_right_ref=0;
		ChassisSpeedRef.rotate_ref=0;
		if(Engineering_State==ENGINEER_PLACING_COMPLETE||Engineering_State==ENGINEER_GRABING||Engineering_State==ENGINEER_FETCHING_COMPLETE)
		{
			Engineering_Order=ENGINEER_STANDBY;
			osDelay(2);
			return 1;
		}
		if(Engineering_State==ENGINEER_DISTANCE_OK)
		{
			fw_printfln("Place");
			Engineering_State=ENGINEER_PLACING;
			//��̧��,צ���쿪
			if(!Engineering_Lift(-5000,100)) return 0;
			if(!Engineering_Grab(1350,100)) return 0;
			//�����ͣ����ϰ���
			aux4_targetSpeed=35000;
			if(!taskDelay(1500))
			{
				aux4_targetSpeed=0;
				return 0;
			}
			aux4_targetSpeed=0;
			//���½���ץȡ
			if(!Engineering_Lift(-600,100)) return 0;
			//if(!taskDelay(100)) return 0;
			if(!Engineering_Grab(0,200)) return 0;
			if(!taskDelay(100)) return 0;
			//̧��
			if(!Engineering_Lift(-32000,300)) return 0;
			//���
			if(!Engineering_Stretch(aux2_limit,1500)) return 0;
			//if(!taskDelay(500)) return 0;
			//����
			if(!Engineering_Lift(-28000,100)) return 0;
			//�ɿ�
			if(!Engineering_Grab(1350,100)) return 0;
//			//̧��
//			if(!Engineering_Lift(-20000,300)) return 0;
			//��λ
			if(!Engineering_Stretch(0,800)) return 0;
			//if(!Engineering_Lift(-800,250)) return 0;
			//״̬
			Engineering_State=ENGINEER_PLACING_COMPLETE;
		}
		else if(Engineering_State==ENGINEER_DISTANCE_AND_ROTATION_OK)
		{
			ChassisSpeedRef.forward_back_ref=0;
			ChassisSpeedRef.left_right_ref=0;
			ChassisSpeedRef.rotate_ref=0;
			fw_printfln("fetch");
			Engineering_State=ENGINEER_FETCHING;
			//��̧��
			if(!Engineering_Lift(-20000,500)) return 0;
			if(!Engineering_Grab(1800,50)) return 0;
			//���
			if(!Engineering_Stretch(aux2_limit,1000)) return 0;
			//ץ
			if(!Engineering_Lift(fetch_height,400)) return 0;
			//if(!taskDelay(200)) return 0;
			if(!Engineering_Grab(0,200)) return 0;
			if(!taskDelay(100)) return 0;
			//̧��
			if(!Engineering_Lift(-32000,400)) return 0;
			
			Engineering_State=ENGINEER_GRABING;
		}
		osDelay(2);
		Engineering_Order=ENGINEER_STANDBY;
		return 1;
}

uint8_t Engineer_Replacing()
{
	if(Engineering_State==ENGINEER_GRABING|| Engineering_State==ENGINEER_REPLACING)
	{
		Engineering_State=ENGINEER_REPLACING;
		//�����ͣ����ϰ���
		aux4_targetSpeed=-35000;
		//��λ
		if(!Engineering_Stretch(0,1500)) return 0;
		if(!taskDelay(700))
		{
			aux4_targetSpeed=0;
			return 0;
		}
//		aux4_targetSpeed=0;
		if(!Engineering_Lift(-5000,700)) return 0;//-24000����ǰ����߶�,-10000���и߶�
		if(!Engineering_Grab(1350,200)) return 0;
		Engineering_State=ENGINEER_FETCHING_COMPLETE;
		osDelay(600);
		aux4_targetSpeed=0;
	}
	Engineering_Order=ENGINEER_STANDBY;
	return 1;
}

uint8_t Engineering_Lift(float value, uint32_t time_milis)
{
	float original=aux_motor1_position_target;
	float tmp=(value-original)/time_milis;
	for(uint32_t i=0;i<time_milis+1;i++)
	{
		aux_motor1_position_target=original + i*tmp;
		if(Engineering_Order==ENGINEER_STOP)
		{	
			fw_printfln("stop called when lift!");
			return 0;
		}
		osDelay(1);
	}
	uint16_t cnt=0;
	while((GetAuxMotorRealAngle(1)-value) >2000 || (GetAuxMotorRealAngle(1)-value)<-2000)
	{
		if(cnt++>800)
		{
			Engineering_Order=ENGINEER_PAUSE;
			fw_printfln("lift not reach! target %f  real %f", value, GetAuxMotorRealAngle(1));
			return 0;
		}
		if(Engineering_Order==ENGINEER_STOP)
		{	
			fw_printfln("stop called when lift!");
			return 0;
		}
		osDelay(1);
	}
	return 1;
}
uint8_t Engineering_Stretch(float value, uint32_t time_milis)
{
	float original=aux_motor2_position_target;
	//ʹ��5�β�ֵ
	//float tmp=(value-original)/time_milis;
	for(uint32_t i=1;i<time_milis+1;i++)
	{
		float a_b=original-value;
		float tmp=((float)i/time_milis);
		float tmp3=tmp*tmp*tmp;
		float tmp4=tmp3*tmp;
		float tmp5=tmp4*tmp;
		aux_motor2_position_target=-6*a_b*tmp5+15*a_b*tmp4-10*a_b*tmp3+original;
		if(Engineering_Order==ENGINEER_STOP)
		{	
			fw_printfln("stop called when strech!");
			return 0;
		}
		//fw_printfln("%f",aux_motor2_position_target);
		osDelay(1);
	}
	uint16_t cnt=0;
	while((GetAuxMotorRealAngle(2)-value) >1000 || (GetAuxMotorRealAngle(2)-value)<-1000)
	{
		if(cnt++>500)
		{
			fw_printfln("stretch not reach!target %f  real %f", value, GetAuxMotorRealAngle(2));
			Engineering_Order=ENGINEER_PAUSE;
			return 0;
		}
		if(Engineering_Order==ENGINEER_STOP)
		{	
			fw_printfln("stop called when strech! ");
			return 0;
		}
		osDelay(1);
	}
	return 1;
}

uint8_t Engineering_Grab(float value, uint32_t time_milis)
{
	float original=aux_motor3_position_target;
	float tmp=(value-original)/time_milis;
	for(uint32_t i=0;i<time_milis+1;i++)
	{
		aux_motor3_position_target=original + i*tmp;
		if(Engineering_Order==ENGINEER_STOP)
		{	
			return 0;
		}
		osDelay(1);
	}
	return 1;
}

uint8_t taskDelay(uint32_t time_milis)
{
	for(int i=0;i<time_milis;i++)
	{
		if(Engineering_Order==ENGINEER_STOP)
		{	
			return 0;
		}
		osDelay(1);
	}
	return 1;
}

void Engineering_Recover()
{
	//û��ץȡ, ����no move������engineer complete
//	if(!engineer_grab_somthing() && Engineering_State!=ENGINEER_NO_MOVE && Engineering_State!=ENGINEER_COMPLETE)
	Engineering_Order=ENGINEER_STANDBY;
	if(!engineer_grab_somthing())
	{
		Engineering_State=ENGINEER_RECOVERING;
		fw_printfln("Not grabing, Now Recover!");
		//�ָ�����
		aux4_targetSpeed=0;
		aux_motor1_position_target=-20000;
		osDelay(1000);
		aux_motor2_position_target=0;
		aux_motor3_position_target=1350;
		osDelay(500);
		aux_motor1_position_target=0;
		osDelay(500);
		//״̬
		Engineering_State=ENGINEER_NO_MOVE;
	}
	else
	{
		fw_printfln("Grabing. Now recover");
		//
		aux4_targetSpeed=0;
		Engineering_Lift(-32000,500);
		Engineering_Stretch(aux2_limit,800);
		Engineering_Grab(0,100);
		Engineering_State=ENGINEER_GRABING;
	}
	osDelay(1);
}


//void StartNewEngineerTask()
//{
//	if(Engineering_Order==ENGINEER_STOP)
//	{
//		fw_printfln("Start new task!");
//		Engineering_Order=ENGINEER_PREPARE;
////		Engineering_State=ENGINEER_NO_MOVE;
//	}
//}



void StartNewFetchTask()
{
	if(Engineering_State==ENGINEER_GRABING)
	{
		fw_printfln("Fetch:Grabing Something");
		//צ����ץȡ,�Żأ�
		Engineering_Order=ENGINEER_REPLACE_STUFF;
	}
	else if(aux_motor1_position_target>-2000 && aux_motor2_position_target>-2000)
	{
		fw_printfln("Fetch: Go to Adjusting");
		//����Ϊ��λ�����Խ�����ҵ
		Engineering_Order=ENGINEER_ADJUSTDISTANCE_AND_ROTATION;
	}
	else
	{
		fw_printfln("Fetch:Go to PAUSE");
		//�����쳣���
		Engineering_Order=ENGINEER_PAUSE;
	}
}

void StopEngineerTask()
{
	if(Engineering_State!=ENGINEER_GRABING)
	{
		Engineering_Order=ENGINEER_STOP;
		Engineering_State=ENGINEER_NO_MOVE;
	}
//	Engineering_Order=ENGINEER_STOP;
//		Engineering_State=ENGINEER_NO_MOVE;
}

void StartNewPlaceTask()
{
//	if(aux_motor1_position_target<-10000 && aux_motor2_position_target<-20000 && aux_motor3_position_target<1200)
	if(Engineering_State==ENGINEER_GRABING)
	{
		fw_printfln("Place:Grabing Something");
		//צ����ץȡ
		Engineering_Order=ENGINEER_DISCARD_STUFF;
	}
	else if(aux_motor1_position_target>-2000 && aux_motor2_position_target>-2000)
	{
		fw_printfln("Place: Go to Adjusting");
		//����Ϊ��λ�����Խ�����ҵ
		Engineering_Order=ENGINEER_ADJUSTDISTANCE;
	}
	else
	{
		fw_printfln("Place:Go to PAUSE");
		//�����쳣���
		Engineering_Order=ENGINEER_PAUSE;
	}
}

void Engineer_Discard_Stuff()
{
	if(Engineering_State==ENGINEER_GRABING)
	{
		Engineering_State=ENGINEER_DISCASDING;
//		aux_motor1_position_target=-32000;
//		osDelay(500);
//		aux_motor2_position_target=-45000;
//		osDelay(500);
		aux_motor3_position_target=1350;
		osDelay(500);
		aux_motor2_position_target=0;
		aux_motor3_position_target=1350;
		osDelay(800);
		aux_motor1_position_target=-800;
		osDelay(500);
		Engineering_State=ENGINEER_NO_MOVE;
	}
	Engineering_Order=ENGINEER_STANDBY;
}

//void OrderReplace()
//{
//	if(Engineering_Order!=ENGINEER_REPLACE_STUFF)
//		Engineering_Order=ENGINEER_REPLACE_STUFF;
//}

void SetFetchHeight(uint8_t HeightIndex)
{
	if(HeightIndex==1)
	{
		if(fetch_height!=-10000)
			fetch_height=-10000;
	}
	else if(HeightIndex==2)
	{
		if(fetch_height!=-1200)
			fetch_height=-1200;
	}
}
///DEBUG////
void EngineerPrintState()
{
	switch(Engineering_State)
	{
		case ENGINEER_NO_MOVE:
		{
			fw_printfln("no move");
			break;
		}
		case ENGINEER_ADJUSTING:
		{
			fw_printfln("Adjusting");
			break;
		}
		case ENGINEER_DISTANCE_OK:
		{
			fw_printfln("distance ok");
			break;
		}
		case ENGINEER_DISTANCE_AND_ROTATION_OK:
		{
			fw_printfln("distance and rotation ok");
			break;
		}
		case ENGINEER_FETCHING:
		{
			fw_printfln("engineer fetching");
			break;
		}
		case ENGINEER_PLACING:
		{
			fw_printfln("engineer placing");
			break;
		}
		case ENGINEER_FETCHING_COMPLETE:
		{
			fw_printfln("engineer fetch complete");
			break;
		}
		case ENGINEER_PLACING_COMPLETE:
		{
			fw_printfln("engineer place complete");
			break;
		}
		case ENGINEER_DISCASDING:
		{
			fw_printfln("engineer discarding");
			break;
		}
		case ENGINEER_REPLACING:
		{
			fw_printfln("engineer replacing");
			break;
		}
		case ENGINEER_GRABING:
		{
			fw_printfln("grabing");
			break;
		}
		case ENGINEER_MANUAL_PLACING:
		{
			fw_printfln("Manual placing");
			break;
		}
		case ENGINEER_MANUAL_FETCHING:
		{
			fw_printfln("Manual fetching");
			break;
		}
		case ENGINEER_MANUAL_STRETCHED:
		{
			fw_printfln("Manual stretched");
			break;
		}
	}
}
////�ֶ���������
//�ֶ���
uint8_t Engineer_manual_place()
{
	//�ֶ�ģʽ���԰���engineer_complete
	if(Engineering_State==ENGINEER_NO_MOVE || Engineering_State==ENGINEER_FETCHING_COMPLETE|| Engineering_State==ENGINEER_PLACING_COMPLETE)
	{
		fw_printfln("Place");
		Engineering_State=ENGINEER_MANUAL_PLACING;
		//��̧��,צ���쿪
		if(!Engineering_Lift(-5000,100)) return 0;
		if(!Engineering_Grab(1350,100)) return 0;
		//�����ͣ����ϰ���
		aux4_targetSpeed=35000;
		if(!taskDelay(1500))
		{
			aux4_targetSpeed=0;
			return 0;
		}
		aux4_targetSpeed=0;
		//���½���ץȡ
		if(!Engineering_Lift(-600,100)) return 0;
		//if(!taskDelay(100)) return 0;
		if(!Engineering_Grab(0,200)) return 0;
		if(!taskDelay(100)) return 0;
		//̧��
		if(!Engineering_Lift(-32000,300)) return 0;
		//���
		if(!Engineering_Stretch(aux2_limit,1100)) return 0;
		//if(!taskDelay(500)) return 0;
		//����
		if(!Engineering_Lift(-30000,50)) return 0;
		//״̬
		Engineering_State=ENGINEER_GRABING;
		Engineering_Order=ENGINEER_STANDBY;
	}
	else if(Engineering_State==ENGINEER_GRABING)
	{
		fw_printfln("discard!");
		Engineering_Order=ENGINEER_DISCARD_STUFF;
	}
	else
	{
		Engineering_Order=ENGINEER_STANDBY;
	}
	return 1;
}
//�ֶ�ȡ
uint8_t Engineer_manual_fetch()
{
	if(Engineering_State==ENGINEER_NO_MOVE || Engineering_State==ENGINEER_FETCHING_COMPLETE|| Engineering_State==ENGINEER_PLACING_COMPLETE)
	{
		fw_printfln("fetch");
		Engineering_State=ENGINEER_MANUAL_FETCHING;
		//��̧��
		if(!Engineering_Lift(-25000,300)) return 0;
		if(!Engineering_Grab(1800,50)) return 0;
		//���
		if(!Engineering_Stretch(aux2_limit,400)) return 0;
		if(!Engineering_Lift(fetch_height-2000,300)) return 0;
		Engineering_State=ENGINEER_MANUAL_STRETCHED;
		Engineering_Order=ENGINEER_STANDBY;
	}
	else if(Engineering_State==ENGINEER_MANUAL_STRETCHED)
	{
		fw_printfln("fetched,now grab!");
		if(!Engineering_Lift(fetch_height,300)) return 0;
		if(!Engineering_Grab(0,100)) return 0;
		if(!taskDelay(100)) return 0;
		//̧��
		if(!Engineering_Lift(-32000,400)) return 0;
		Engineering_State=ENGINEER_GRABING;
		Engineering_Order=ENGINEER_STANDBY;
	}
	else
	{
		Engineering_Order=ENGINEER_STANDBY;
	}
	return 1;
}
//�������ڹ��̳���û��ץȡ��ʲô����
uint8_t engineer_grab_somthing()
{
	double tmp=GetAuxMotorRealAngle(3);
	if(aux_motor3_position_target<0.01&&aux_motor3_position_target>-0.01)
	{
//		fw_printfln("grab target:%f",aux_motor3_position_target);
		if(tmp>300)
			return 1;
	}
	return 0;
}

uint8_t engineer_belt_back()
{
	Engineering_State=ENGINEER_BELT_MOVING;
	aux4_targetSpeed=-35000;
	if(!(taskDelay(500)))
	{
		aux3_targetSpeed=0;
		Engineering_Order=ENGINEER_STANDBY;
		Engineering_State=ENGINEER_NO_MOVE;
		return 0;
	}
	aux4_targetSpeed=0;
	Engineering_Order=ENGINEER_STANDBY;
	Engineering_State=ENGINEER_NO_MOVE;
	return 1;
}

uint8_t engineer_belt_forward()
{
	Engineering_State=ENGINEER_BELT_MOVING;
	aux4_targetSpeed=35000;
	if(!(taskDelay(500)))
	{
		aux3_targetSpeed=0;
		Engineering_Order=ENGINEER_STANDBY;
		Engineering_State=ENGINEER_NO_MOVE;
		return 0;
	}
	aux4_targetSpeed=0;
	Engineering_Order=ENGINEER_STANDBY;
	Engineering_State=ENGINEER_NO_MOVE;
	return 1;
}
	
