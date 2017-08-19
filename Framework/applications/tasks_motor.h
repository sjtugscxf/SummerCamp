#ifndef TASKS_MOTOR_H
#define TASKS_MOTOR_H
#include "stdint.h"


extern uint8_t GM_RUN;

typedef __packed struct
{
    float forward_back_ref;
    float left_right_ref;
    float rotate_ref;
}ChassisSpeed_Ref_t;


extern ChassisSpeed_Ref_t ChassisSpeedRef;
extern float aux1_targetSpeed,aux2_targetSpeed;
extern double aux_motor34_position_target,getBullet_angle_target,plate_angle_target;
extern double aux34_limit,getBullet_limit;
void CMGMControlTask(void const * argument);
void AMControlTask(void const * argument);

typedef struct 
{
    float distance;
		float rotate;
		int16_t traverse;
}FFollowLoc_t;
extern FFollowLoc_t FFollowLoc;
void ShootOnce();

#define FOLLOW_DISTANCE  200.0
#define FOLLOW_ROTATE  200.0

#define DISTANCE_KP  40.0
#define ROTATE_KP  60.0
#endif
