#ifndef TASKS_UPPER_H
#define TASKS_UPPER_H

#include "utilities_iopool.h"
#include "utilities_minmax.h"
#include "application_gimbalcontrol.h"
IOPoolDeclare(upperIOPool, struct{float yawAdd; float pitchAdd;});

void getCtrlUartTask(void const * argument);

void CxfProcessData();

void zykProcessData();
void wave_task(void const * argument);
#endif
