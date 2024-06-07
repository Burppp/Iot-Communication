#include "Lora.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "usart.h"
#include "stdint.h"

extern UART_HandleTypeDef hlpuart1;
extern uint8_t isPrintf; //定义于 main 函数: 是否打印日志
#define LPUART1_REC_LEN 1024
uint8_t bRxBufferUart1[1]; //接收数据
uint8_t LPUART1_RX_BUF[LPUART1_REC_LEN];//缓存数据
volatile uint16_t LPUART1_RX_LEN=0;
uint8_t LPUART1_RX_STA[];
uint8_t Tran_Data[];
#define CMD_LEN 100
char cmdSend[CMD_LEN]; //发送上报数据的 AT 指令
uint32_t DefaultTimeout=300;//超时
/*LoRa 模块 AT 指令发送
* 参数: uint_t *cmd，需要发送的命令
* uint8_t *result，期望获得的结果
* uint32_t timeOut，等待期望结果的时间
* uint8_t isPrintf，是否打印 Log
*/
void LoRa_SendCmd(uint8_t *cmd, uint8_t *result, uint32_t timeOut, uint8_t isPrintf) {
	char *pos;
	HAL_UART_Transmit( &hlpuart1, cmd,strlen((const char *)cmd), 0xff); //发送AT 指令
	HAL_UART_Receive_IT(&hlpuart1,bRxBufferUart1,1);//启动低功耗串口接收中断
	HAL_Delay(timeOut); //延时等待
	while(1)
	{
		pos= strstr((const char *) LPUART1_RX_BUF, (const char *) result); //期望结果，如"OK"
		printf("receive:%s\r\n", LPUART1_RX_BUF);//打印 AT 指令返回的数据
		if (pos) {
			printf("Success!\r\n");
			LPUART1_RX_LEN=0;
			memset(LPUART1_RX_BUF, 0, strlen((const char *)LPUART1_RX_BUF));//清除缓存
			break; //不清空数据，待处理
		}
		else{
			printf("Fail!\r\n");
			LPUART1_RX_LEN=0;
			memset(LPUART1_RX_BUF, 0, strlen((const char *)LPUART1_RX_BUF));//清除缓存
			HAL_UART_Transmit(&hlpuart1, cmd, strlen((const char *) cmd), 0xff); //error 或者无应答，再次发送
			HAL_UART_Receive_IT(&hlpuart1,bRxBufferUart1,1); // 启动低功耗串口接收中断
			HAL_Delay(timeOut);
		}
	}
}

/*LoRa 模块，透明传输点对点模式配置
* uint8_t isPrintf:是否打印 Log
* uint8_t isReboot:是否重启
* 模块地址：10086 通信信道：10 发射功率：11dbm
*/
void LoRa_T_P_Attach(uint8_t isPrintf,uint8_t isReboot){
	if(isReboot == 1)
	{
		//HAL_GPIO_WritePin(PB14_GPIO_Port, PB14_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
		HAL_Delay(1000);
		LoRa_SendCmd((uint8_t *)"AT+UART=7,0\r\n", (uint8_t *)"OK",DefaultTimeout, isPrintf);
		LoRa_SendCmd((uint8_t *)"AT+WLRATE=10,5\r\n", (uint8_t *)"OK",DefaultTimeout, isPrintf);
		LoRa_SendCmd((uint8_t *)"AT+TPOWER=0\r\n", (uint8_t *)"OK",DefaultTimeout, isPrintf);
		LoRa_SendCmd((uint8_t *)"AT+TMODE=0\r\n", (uint8_t *)"OK",DefaultTimeout, isPrintf);
		LoRa_SendCmd((uint8_t *)"AT+WLTIME=0\r\n", (uint8_t *)"OK",DefaultTimeout, isPrintf);
		LoRa_SendCmd((uint8_t *)"AT+CWMODE=0\r\n", (uint8_t *)"OK",DefaultTimeout, isPrintf);
		LoRa_SendCmd((uint8_t *)"AT+ADDR=27,66\r\n", (uint8_t *)"OK",DefaultTimeout, isPrintf);
		//HAL_GPIO_WritePin(PB14_GPIO_Port, PB14_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
		printf("Attach!\r\n");
	}
}

/** LoRa 模块，透明传输广播模式配置
* uint8_t isPrintf:是否打印 Log
* uint8_t isReboot:是否重启
* 模块地址：0xFFFF 通信信道：10 发射功率：11dbm
*/
void LoRa_T_V_Attach(uint8_t isPrintf,uint8_t isReboot)
{
	if(isReboot == 1)
	{
		//HAL_GPIO_WritePin(PB14_GPIO_Port, PB14_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
		HAL_Delay(1000);
		LoRa_SendCmd((uint8_t *)"AT+UART=7,0\r\n", (uint8_t *)"OK", DefaultTimeout, isPrintf);
		LoRa_SendCmd((uint8_t *)"AT+WLRATE=10,5\r\n", (uint8_t *)"OK", DefaultTimeout, isPrintf);
		LoRa_SendCmd((uint8_t *)"AT+TPOWER=0\r\n", (uint8_t *)"OK", DefaultTimeout, isPrintf);
		LoRa_SendCmd((uint8_t *)"AT+TMODE=0\r\n", (uint8_t *)"OK", DefaultTimeout, isPrintf);
		LoRa_SendCmd((uint8_t *)"AT+WLTIME=0\r\n", (uint8_t *)"OK", DefaultTimeout, isPrintf);
		LoRa_SendCmd((uint8_t *)"AT+CWMODE=0\r\n", (uint8_t *)"OK", DefaultTimeout, isPrintf);
		LoRa_SendCmd((uint8_t *)"AT+ADDR=FF,FF\r\n", (uint8_t *)"OK",DefaultTimeout, isPrintf);
		//HAL_GPIO_WritePin(PB14_GPIO_Port, PB14_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
		printf("Attach!\r\n");
	}
}

/*
* LoRa 模块，定向传输点对点模式配置
* uint8_t isPrintf:是否打印 Log
* uint8_t isReboot:是否重启
*/
void LoRa_D_P_Attach(uint8_t isPrintf,uint8_t isReboot)
{
	if(isReboot == 1)
	{
		//HAL_GPIO_WritePin(PB14_GPIO_Port, PB14_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
				HAL_Delay(1000);
				LoRa_SendCmd((uint8_t *)"AT+UART=7,0\r\n", (uint8_t *)"OK",DefaultTimeout, isPrintf);
				LoRa_SendCmd((uint8_t *)"AT+WLRATE=10,5\r\n", (uint8_t *)"OK",DefaultTimeout, isPrintf);
				LoRa_SendCmd((uint8_t *)"AT+TPOWER=0\r\n", (uint8_t *)"OK",DefaultTimeout, isPrintf);
				LoRa_SendCmd((uint8_t *)"AT+TMODE=1\r\n", (uint8_t *)"OK",DefaultTimeout, isPrintf);
				LoRa_SendCmd((uint8_t *)"AT+WLTIME=0\r\n", (uint8_t *)"OK",DefaultTimeout, isPrintf);
				LoRa_SendCmd((uint8_t *)"AT+CWMODE=0\r\n", (uint8_t *)"OK",DefaultTimeout, isPrintf);
				LoRa_SendCmd((uint8_t *)"AT+ADDR=27,66\r\n", (uint8_t *)"OK",DefaultTimeout, isPrintf);
				//HAL_GPIO_WritePin(PB14_GPIO_Port, PB14_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
				printf("Attach!\r\n");
		//请补充代码
	}
}

/*
* LoRa 模块，定向传输广播模式配置
* uint8_t isPrintf:是否打印 Log
* uint8_t isReboot:是否重启
*/
void LoRa_D_V_Attach(uint8_t isPrintf,uint8_t isReboot)
{
	if(isReboot == 1)
	{
		//HAL_GPIO_WritePin(PB14_GPIO_Port, PB14_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
				HAL_Delay(1000);
				LoRa_SendCmd((uint8_t *)"AT+UART=7,0\r\n", (uint8_t *)"OK", DefaultTimeout, isPrintf);
				LoRa_SendCmd((uint8_t *)"AT+WLRATE=10,5\r\n", (uint8_t *)"OK", DefaultTimeout, isPrintf);
				LoRa_SendCmd((uint8_t *)"AT+TPOWER=0\r\n", (uint8_t *)"OK", DefaultTimeout, isPrintf);
				LoRa_SendCmd((uint8_t *)"AT+TMODE=1\r\n", (uint8_t *)"OK", DefaultTimeout, isPrintf);
				LoRa_SendCmd((uint8_t *)"AT+WLTIME=0\r\n", (uint8_t *)"OK", DefaultTimeout, isPrintf);
				LoRa_SendCmd((uint8_t *)"AT+CWMODE=0\r\n", (uint8_t *)"OK", DefaultTimeout, isPrintf);
				LoRa_SendCmd((uint8_t *)"AT+ADDR=FF,FF\r\n", (uint8_t *)"OK",DefaultTimeout, isPrintf);
				//HAL_GPIO_WritePin(PB14_GPIO_Port, PB14_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
				printf("Attach!\r\n");
		//请补充代码
	}
}
