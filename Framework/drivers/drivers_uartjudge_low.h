/**
  ******************************************************************************
  * File Name          : pdrivers_uartjudge_low.h
  * Description        : 裁判系统读取
  ******************************************************************************
  *
  * Copyright (c) 2017 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  * 底层函数
  ******************************************************************************
  */
#ifndef DRIVERS_UARTJUDGE_LOW_H
#define DRIVERS_UARTJUDGE_LOW_H

#include "utilities_iopool.h"
#include "cmsis_os.h"

typedef struct 
{
    uint32_t remainTime;
    uint16_t remainLifeValue;
    float realChassisOutV;
    float realChassisOutA;
    float remainPower;
}tGameInfo;

typedef enum
{
	ONLINE,
	OFFLINE
}JudgeState_e;

typedef struct 
{
    uint16_t distance;
		uint16_t rotate;
		uint16_t traverse;
}FollowLoc_t;
extern FollowLoc_t FollowLoc;

void judgeUartRxCpltCallback(void);
void InitJudgeUart(void);
void Judge_Refresh(void);

extern uint8_t JUDGE_Received;
extern uint8_t tmp_judge;
extern uint8_t buffer[10];
#endif
