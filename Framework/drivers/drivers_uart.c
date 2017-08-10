#include "drivers_uart.h"
#include "drivers_uartrc_low.h"
#include "drivers_uartupper_low.h"

#include "peripheral_define.h"
#include "utilities_debug.h"
#include "usart.h"
#include "drivers_uartjudge_low.h"

int testuart = 0;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
	//fw_printfln("HAL_UART_RxCpltCallback");
	if(UartHandle == &RC_UART){
		//rcUartRxCpltCallback();
		fw_Warning();
	}else if(UartHandle == &CTRL_UART){
		//testuart = 1;
		ctrlUartRxCpltCallback();
	}else if(UartHandle == &JUDGE_UART){
		//testuart = 1;
		//HAL_UART_Receive_IT(&JUDGE_UART, &tmp_judge, 1);
		judgeUartRxCpltCallback();
	}
}   
