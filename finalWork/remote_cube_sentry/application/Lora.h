#include "gpio.h"

void LoRa_SendCmd(uint8_t *cmd, uint8_t *result, uint32_t timeOut, uint8_t isPrintf);
void LoRa_T_P_Attach(uint8_t isPrintf,uint8_t isReboot);
void LoRa_D_P_Attach(uint8_t isPrintf,uint8_t isReboot);
void LoRa_D_V_Attach(uint8_t isPrintf,uint8_t isReboot);
void LoRa_T_V_Attach(uint8_t isPrintf,uint8_t isReboot);