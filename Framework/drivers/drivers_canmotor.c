#include "drivers_canmotor_low.h"
#include "drivers_canmotor_user.h"

#include "cmsis_os.h"

#include "peripheral_define.h"

#include "utilities_debug.h"
#include "utilities_iopool.h"
#include "utilities_tim.h"
#include "rtos_init.h"
#include "rtos_semaphore.h"

//RxIOPool
NaiveIOPoolDefine(CMFLRxIOPool, {0});
NaiveIOPoolDefine(CMFRRxIOPool, {0});
NaiveIOPoolDefine(CMBLRxIOPool, {0});
NaiveIOPoolDefine(CMBRRxIOPool, {0});

NaiveIOPoolDefine(GMPITCHRxIOPool, {0});
NaiveIOPoolDefine(GMYAWRxIOPool, {0});

NaiveIOPoolDefine(AMUDFLRxIOPool, {0});
NaiveIOPoolDefine(AMUDFRRxIOPool, {0});
NaiveIOPoolDefine(AMUDBLRxIOPool, {0});
NaiveIOPoolDefine(AMUDBRRxIOPool, {0});

NaiveIOPoolDefine(AMPLATERxIOPool, {0});
NaiveIOPoolDefine(AMGETBULLETRxIOPool, {0});
//TxIOPool
#define DataPoolInit \
	{ \
		{CM_TXID, 0, CAN_ID_STD, CAN_RTR_DATA, 8, {0}}, \
		{CM_TXID, 0, CAN_ID_STD, CAN_RTR_DATA, 8, {0}}, \
		{CM_TXID, 0, CAN_ID_STD, CAN_RTR_DATA, 8, {0}} \
	}
NaiveIOPoolDefine(CMTxIOPool, DataPoolInit);
#undef DataPoolInit 
	
#define DataPoolInit \
	{ \
		{GM_TXID, 0, CAN_ID_STD, CAN_RTR_DATA, 8, {0}}, \
		{GM_TXID, 0, CAN_ID_STD, CAN_RTR_DATA, 8, {0}}, \
		{GM_TXID, 0, CAN_ID_STD, CAN_RTR_DATA, 8, {0}} \
	}
NaiveIOPoolDefine(GMTxIOPool, DataPoolInit);
#undef DataPoolInit 
	
#define DataPoolInit \
	{ \
		{AM1_TXID, 0, CAN_ID_STD, CAN_RTR_DATA, 8, {0}}, \
		{AM1_TXID, 0, CAN_ID_STD, CAN_RTR_DATA, 8, {0}}, \
		{AM1_TXID, 0, CAN_ID_STD, CAN_RTR_DATA, 8, {0}} \
	}
NaiveIOPoolDefine(AM1TxIOPool, DataPoolInit);
#undef DataPoolInit 
	
#define DataPoolInit \
	{ \
		{AM2_TXID, 0, CAN_ID_STD, CAN_RTR_DATA, 8, {0}}, \
		{AM2_TXID, 0, CAN_ID_STD, CAN_RTR_DATA, 8, {0}}, \
		{AM2_TXID, 0, CAN_ID_STD, CAN_RTR_DATA, 8, {0}} \
	}
NaiveIOPoolDefine(AM2TxIOPool, DataPoolInit);
#undef DataPoolInit 

#define CanRxGetU16(canRxMsg, num) (((uint16_t)canRxMsg.Data[num * 2] << 8) + (uint16_t)canRxMsg.Data[num * 2 + 1])

uint8_t isRcanStarted_CMGM = 0, isRcanStarted_AM = 0;

CanRxMsgTypeDef CMGMCanRxMsg, AMCanRxMsg;
void motorInit(){
	CMGMMOTOR_CAN.pRxMsg = &CMGMCanRxMsg;
	/*##-- Configure the CAN2 Filter ###########################################*/
	CAN_FilterConfTypeDef  sFilterConfig;
	sFilterConfig.FilterNumber = 0;//14 - 27//14
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = 0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.BankNumber = 14;
  HAL_CAN_ConfigFilter(&CMGMMOTOR_CAN, &sFilterConfig);
	if(HAL_CAN_Receive_IT(&CMGMMOTOR_CAN, CAN_FIFO0) != HAL_OK){
		fw_Error_Handler(); 
	}
	isRcanStarted_CMGM = 1;
	
	AUXMOTOR_CAN.pRxMsg = &AMCanRxMsg;
	/*##-- Configure the CAN2 Filter ###########################################*/
	CAN_FilterConfTypeDef sFilterConfig2;
	sFilterConfig2.FilterNumber = 14;//14 - 27//14
	sFilterConfig2.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig2.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig2.FilterIdHigh = 0x0000;
  sFilterConfig2.FilterIdLow = 0x0000;
  sFilterConfig2.FilterMaskIdHigh = 0x0000;
  sFilterConfig2.FilterMaskIdLow = 0x0000;
  sFilterConfig2.FilterFIFOAssignment = 0;
  sFilterConfig2.FilterActivation = ENABLE;
  sFilterConfig2.BankNumber = 14;
  HAL_CAN_ConfigFilter(&AUXMOTOR_CAN, &sFilterConfig2);
	if(HAL_CAN_Receive_IT(&AUXMOTOR_CAN, CAN_FIFO0) != HAL_OK){
		fw_Error_Handler(); 
	}
	isRcanStarted_AM = 1;
	
}

void ZGyroReset(void){
	static uint8_t hasReset = 0;
	if(hasReset == 0 && fw_getTimeMicros() >= 1000 * 1000 * 2){
		fw_printfln("ZGyroReset");
		CanTxMsgTypeDef ZGYRO_TxMsg = {
			ZGYRO_TXID, 0, CAN_ID_STD, CAN_RTR_DATA, 8, 
			{0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07}
		};
		AUXMOTOR_CAN.pTxMsg = &ZGYRO_TxMsg;
		if(HAL_CAN_Transmit(&AUXMOTOR_CAN, 50) != HAL_OK){
			fw_Warning();
		}
		hasReset = 1;
	}
}

//uint16_t pitchAngle = 0, yawAngle = 0;
//uint32_t flAngle = 0, frAngle = 0, blAngle = 0, brAngle = 0;
//uint16_t flSpeed = 0, frSpeed = 0, blSpeed = 0, brSpeed = 0;
float ZGyroModuleAngle = 0.0;

void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* hcan){
	//CanRxMsgTypeDef *temp = IOPool_pGetWriteData(motorCanRxIOPool);
	if(hcan == &CMGMMOTOR_CAN){
		switch(CMGMCanRxMsg.StdId){
			case CMFL_RXID:
				IOPool_pGetWriteData(CMFLRxIOPool)->angle = CanRxGetU16(CMGMCanRxMsg, 0);
				IOPool_pGetWriteData(CMFLRxIOPool)->RotateSpeed = CanRxGetU16(CMGMCanRxMsg, 1);
				//EC06
				static float CMFLrealAngleLast = 0;
				float CMFLrealAngleCurr = CanRxGetU16(CMGMCanRxMsg, 0) * 360 / 8192.0;
				float CMFLrealAngleDiff;
				if(CMFLrealAngleCurr - CMFLrealAngleLast > 180){
					CMFLrealAngleDiff = CMFLrealAngleCurr - 360 - CMFLrealAngleLast;
				}else if(CMFLrealAngleCurr - CMFLrealAngleLast < -180){
					CMFLrealAngleDiff = CMFLrealAngleCurr + 360 - CMFLrealAngleLast;
				}else{
					CMFLrealAngleDiff = CMFLrealAngleCurr - CMFLrealAngleLast;
				}
				CMFLrealAngleLast = CMFLrealAngleCurr;
				IOPool_pGetWriteData(CMFLRxIOPool)->RotateSpeed = CMFLrealAngleDiff * 1000 / 6.0;
				//EC06
				IOPool_getNextWrite(CMFLRxIOPool);
				break;
			case CMFR_RXID:
				IOPool_pGetWriteData(CMFRRxIOPool)->angle = CanRxGetU16(CMGMCanRxMsg, 0);
				IOPool_pGetWriteData(CMFRRxIOPool)->RotateSpeed = CanRxGetU16(CMGMCanRxMsg, 1);
				//EC06
				static float CMFRrealAngleLast = 0;
				float CMFRrealAngleCurr = CanRxGetU16(CMGMCanRxMsg, 0) * 360 / 8192.0;
				float CMFRrealAngleDiff;
				if(CMFRrealAngleCurr - CMFRrealAngleLast > 180){
					CMFRrealAngleDiff = CMFRrealAngleCurr - 360 - CMFRrealAngleLast;
				}else if(CMFRrealAngleCurr - CMFRrealAngleLast < -180){
					CMFRrealAngleDiff = CMFRrealAngleCurr + 360 - CMFRrealAngleLast;
				}else{
					CMFRrealAngleDiff = CMFRrealAngleCurr - CMFRrealAngleLast;
				}
				CMFRrealAngleLast = CMFRrealAngleCurr;
				IOPool_pGetWriteData(CMFRRxIOPool)->RotateSpeed = CMFRrealAngleDiff * 1000 / 6.0;
				//EC06
				IOPool_getNextWrite(CMFRRxIOPool);
				break;
			case CMBL_RXID:
				IOPool_pGetWriteData(CMBLRxIOPool)->angle = CanRxGetU16(CMGMCanRxMsg, 0);
				IOPool_pGetWriteData(CMBLRxIOPool)->RotateSpeed = CanRxGetU16(CMGMCanRxMsg, 1);
				//EC06
				static float CMBLrealAngleLast = 0;
				float CMBLrealAngleCurr = CanRxGetU16(CMGMCanRxMsg, 0) * 360 / 8192.0;
				float CMBLrealAngleDiff;
				if(CMBLrealAngleCurr - CMBLrealAngleLast > 180){
					CMBLrealAngleDiff = CMBLrealAngleCurr - 360 - CMBLrealAngleLast;
				}else if(CMBLrealAngleCurr - CMBLrealAngleLast < -180){
					CMBLrealAngleDiff = CMBLrealAngleCurr + 360 - CMBLrealAngleLast;
				}else{
					CMBLrealAngleDiff = CMBLrealAngleCurr - CMBLrealAngleLast;
				}
				CMBLrealAngleLast = CMBLrealAngleCurr;
				IOPool_pGetWriteData(CMBLRxIOPool)->RotateSpeed = CMBLrealAngleDiff * 1000 / 6.0;
				//EC06
				IOPool_getNextWrite(CMBLRxIOPool);
				break;
			case CMBR_RXID:
				IOPool_pGetWriteData(CMBRRxIOPool)->angle = CanRxGetU16(CMGMCanRxMsg, 0);
				IOPool_pGetWriteData(CMBRRxIOPool)->RotateSpeed = CanRxGetU16(CMGMCanRxMsg, 1);
				//EC06
				static float CMBRrealAngleLast = 0;
				float CMBRrealAngleCurr = CanRxGetU16(CMGMCanRxMsg, 0) * 360 / 8192.0;
				float CMBRrealAngleDiff;
				if(CMBRrealAngleCurr - CMBRrealAngleLast > 180){
					CMBRrealAngleDiff = CMBRrealAngleCurr - 360 - CMBRrealAngleLast;
				}else if(CMBRrealAngleCurr - CMBRrealAngleLast < -180){
					CMBRrealAngleDiff = CMBRrealAngleCurr + 360 - CMBRrealAngleLast;
				}else{
					CMBRrealAngleDiff = CMBRrealAngleCurr - CMBRrealAngleLast;
				}
				CMBRrealAngleLast = CMBRrealAngleCurr;
				IOPool_pGetWriteData(CMBRRxIOPool)->RotateSpeed = CMBRrealAngleDiff * 1000 / 6.0;
				//EC06
				IOPool_getNextWrite(CMBRRxIOPool);
				break;
			case GMYAW_RXID:
				IOPool_pGetWriteData(GMYAWRxIOPool)->angle = CanRxGetU16(CMGMCanRxMsg, 0);
				IOPool_pGetWriteData(GMYAWRxIOPool)->realIntensity = CanRxGetU16(CMGMCanRxMsg, 1);
				IOPool_pGetWriteData(GMYAWRxIOPool)->giveIntensity = CanRxGetU16(CMGMCanRxMsg, 2);
				IOPool_getNextWrite(GMYAWRxIOPool);
				break;
			case GMPITCH_RXID:
				IOPool_pGetWriteData(GMPITCHRxIOPool)->angle = CanRxGetU16(CMGMCanRxMsg, 0);
				IOPool_pGetWriteData(GMPITCHRxIOPool)->realIntensity = CanRxGetU16(CMGMCanRxMsg, 1);
				IOPool_pGetWriteData(GMPITCHRxIOPool)->giveIntensity = CanRxGetU16(CMGMCanRxMsg, 2);
				IOPool_getNextWrite(GMPITCHRxIOPool);
				break;
			default:
			fw_Warning();
		}
		if(HAL_CAN_Receive_IT(&CMGMMOTOR_CAN, CAN_FIFO0) != HAL_OK){
			//fw_Warning();
			isRcanStarted_CMGM = 0;
		}else{
			isRcanStarted_CMGM = 1;
		}
		if(isInited == 1){
			osSemaphoreRelease(CMGMCanRefreshSemaphoreHandle);
		}
	}else if(hcan == &AUXMOTOR_CAN){
		switch(AMCanRxMsg.StdId){
			case AMUDFL_RXID:
				IOPool_pGetWriteData(AMUDFLRxIOPool)->angle = CanRxGetU16(AMCanRxMsg, 0);
				IOPool_pGetWriteData(AMUDFLRxIOPool)->RotateSpeed = CanRxGetU16(AMCanRxMsg, 1);
				IOPool_getNextWrite(AMUDFLRxIOPool);
				break;
			case AMUDFR_RXID:
				IOPool_pGetWriteData(AMUDFRRxIOPool)->angle = CanRxGetU16(AMCanRxMsg, 0);
				IOPool_pGetWriteData(AMUDFRRxIOPool)->RotateSpeed = CanRxGetU16(AMCanRxMsg, 1);
				IOPool_getNextWrite(AMUDFRRxIOPool);
				break;
			case AMUDBL_RXID:
				IOPool_pGetWriteData(AMUDBLRxIOPool)->angle = CanRxGetU16(AMCanRxMsg, 0);
				IOPool_pGetWriteData(AMUDBLRxIOPool)->RotateSpeed = CanRxGetU16(AMCanRxMsg, 1);
				IOPool_getNextWrite(AMUDBLRxIOPool);
				break;
			case AMUDBR_RXID:
				IOPool_pGetWriteData(AMUDBRRxIOPool)->angle = CanRxGetU16(AMCanRxMsg, 0);
				IOPool_pGetWriteData(AMUDBRRxIOPool)->RotateSpeed = CanRxGetU16(AMCanRxMsg, 1);
				IOPool_getNextWrite(AMUDBRRxIOPool);
				break;
			case AMPLATE_RXID:
				IOPool_pGetWriteData(AMPLATERxIOPool)->angle = CanRxGetU16(AMCanRxMsg, 0);
				IOPool_pGetWriteData(AMPLATERxIOPool)->RotateSpeed = CanRxGetU16(AMCanRxMsg, 1);
				IOPool_getNextWrite(AMPLATERxIOPool);
				break;
			case AMGETBULLET_RXID:
				IOPool_pGetWriteData(AMGETBULLETRxIOPool)->angle = CanRxGetU16(AMCanRxMsg, 0);
				IOPool_pGetWriteData(AMGETBULLETRxIOPool)->RotateSpeed = CanRxGetU16(AMCanRxMsg, 1);
				IOPool_getNextWrite(AMGETBULLETRxIOPool);
				break;
			case ZGYRO_RXID:
				ZGyroModuleAngle = -0.01f*((int32_t)(AMCanRxMsg.Data[0]<<24)|(int32_t)(AMCanRxMsg.Data[1]<<16) | (int32_t)(AMCanRxMsg.Data[2]<<8) | (int32_t)(AMCanRxMsg.Data[3])); 
				break;
			default:
			fw_Warning();
		}
		if(HAL_CAN_Receive_IT(&AUXMOTOR_CAN, CAN_FIFO0) != HAL_OK){
			//fw_Warning();
			isRcanStarted_AM = 0;
		}else{
			isRcanStarted_AM = 1;
		}
		if(isInited == 1){
			osSemaphoreRelease(AMCanRefreshSemaphoreHandle);
		}
	}
	
//	if(canRxMsg.StdId == CMFL_RXID){
//		flAngle = ((uint16_t)canRxMsg.Data[0] << 8) + (uint16_t)canRxMsg.Data[1];
//		flSpeed = ((uint16_t)canRxMsg.Data[2] << 8) + (uint16_t)canRxMsg.Data[3];
//	}else if(canRxMsg.StdId == CMFR_RXID){
//		frAngle = ((uint16_t)canRxMsg.Data[0] << 8) + (uint16_t)canRxMsg.Data[1];
//		frSpeed = ((uint16_t)canRxMsg.Data[2] << 8) + (uint16_t)canRxMsg.Data[3];
//	}else if(canRxMsg.StdId == CMBL_RXID){
//		blAngle = ((uint16_t)canRxMsg.Data[0] << 8) + (uint16_t)canRxMsg.Data[1];
//		blSpeed = ((uint16_t)canRxMsg.Data[2] << 8) + (uint16_t)canRxMsg.Data[3];
//	}else if(canRxMsg.StdId == CMBR_RXID){
//		brAngle = ((uint16_t)canRxMsg.Data[0] << 8) + (uint16_t)canRxMsg.Data[1];
//		brSpeed = ((uint16_t)canRxMsg.Data[2] << 8) + (uint16_t)canRxMsg.Data[3];
//	}else if(canRxMsg.StdId == GMPITCH_RXID){
//		pitchAngle = ((uint16_t)canRxMsg.Data[0] << 8) + (uint16_t)canRxMsg.Data[1];
//	}else if(canRxMsg.StdId == GMPITCH_RXID){
//		yawAngle = ((uint16_t)canRxMsg.Data[0] << 8) + (uint16_t)canRxMsg.Data[1];
//	}

}


void CMGMCanTransmitTask(void const * argument){
	while(1){
		osSemaphoreWait(CMGMCanHaveTransmitSemaphoreHandle, osWaitForever);//osWaitForever
		//fw_printfln("osWaitForeverCMGMCanHaveTransmitSemaphoreHandle");
		if(IOPool_hasNextRead(CMTxIOPool, 0)){
			osSemaphoreWait(CMGMCanTransmitSemaphoreHandle, osWaitForever);
			IOPool_getNextRead(CMTxIOPool, 0);
			CMGMMOTOR_CAN.pTxMsg = IOPool_pGetReadData(CMTxIOPool, 0);
			//taskENTER_CRITICAL();
			if(HAL_CAN_Transmit_IT(&CMGMMOTOR_CAN) != HAL_OK){
				fw_Warning();
				osSemaphoreRelease(CMGMCanTransmitSemaphoreHandle);
			}
			//taskEXIT_CRITICAL();
		}
		if(IOPool_hasNextRead(GMTxIOPool, 0)){
			osSemaphoreWait(CMGMCanTransmitSemaphoreHandle, osWaitForever);
			IOPool_getNextRead(GMTxIOPool, 0);
			CMGMMOTOR_CAN.pTxMsg = IOPool_pGetReadData(GMTxIOPool, 0);
			taskENTER_CRITICAL();
			if(HAL_CAN_Transmit_IT(&CMGMMOTOR_CAN) != HAL_OK){
				fw_Warning();
				osSemaphoreRelease(CMGMCanTransmitSemaphoreHandle);
			}
			taskEXIT_CRITICAL();
		}
		if(isRcanStarted_CMGM == 0){
				if(CMGMMOTOR_CAN.State == HAL_CAN_STATE_BUSY_RX){
					CMGMMOTOR_CAN.State = HAL_CAN_STATE_READY;
				}
				//taskENTER_CRITICAL();
				if(HAL_CAN_Receive_IT(&CMGMMOTOR_CAN, CAN_FIFO0) != HAL_OK){
					fw_Warning();
				}else{
					isRcanStarted_CMGM = 1;
				}
				//taskEXIT_CRITICAL();
			}
	}
}

void AMCanTransmitTask(void const * argument){
	while(1){
		osSemaphoreWait(AMCanHaveTransmitSemaphoreHandle, osWaitForever);//osWaitForever
		if(IOPool_hasNextRead(AM1TxIOPool, 0)){
			osSemaphoreWait(AMCanTransmitSemaphoreHandle, osWaitForever);
			IOPool_getNextRead(AM1TxIOPool, 0);
			AUXMOTOR_CAN.pTxMsg = IOPool_pGetReadData(AM1TxIOPool, 0);
			if(HAL_CAN_Transmit_IT(&AUXMOTOR_CAN) != HAL_OK){
				fw_Warning();
				osSemaphoreRelease(AMCanTransmitSemaphoreHandle);
			}
		}
		if(IOPool_hasNextRead(AM2TxIOPool, 0)){
			osSemaphoreWait(AMCanTransmitSemaphoreHandle, osWaitForever);
			IOPool_getNextRead(AM2TxIOPool, 0);
			AUXMOTOR_CAN.pTxMsg = IOPool_pGetReadData(AM2TxIOPool, 0);
			if(HAL_CAN_Transmit_IT(&AUXMOTOR_CAN) != HAL_OK){
				fw_Warning();
				osSemaphoreRelease(AMCanTransmitSemaphoreHandle);
			}
		}
		if(isRcanStarted_AM == 0){
				if(AUXMOTOR_CAN.State == HAL_CAN_STATE_BUSY_RX){
					AUXMOTOR_CAN.State = HAL_CAN_STATE_READY;
				}
				if(HAL_CAN_Receive_IT(&AUXMOTOR_CAN, CAN_FIFO0) != HAL_OK){
					fw_Warning();
				}else{
					isRcanStarted_AM = 1;
				}
			}
	}
}


void HAL_CAN_TxCpltCallback(CAN_HandleTypeDef* hcan){
	if(hcan == &CMGMMOTOR_CAN){
		osSemaphoreRelease(CMGMCanTransmitSemaphoreHandle);
	}else if(hcan == &AUXMOTOR_CAN){
		osSemaphoreRelease(AMCanTransmitSemaphoreHandle);
	}
}
