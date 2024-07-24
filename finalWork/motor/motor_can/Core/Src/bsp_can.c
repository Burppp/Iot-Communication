#include "bsp_can.h"
#include "string.h"


extern CAN_HandleTypeDef hcan;

static CAN_TxHeaderTypeDef tx_message;
static uint8_t can_send_data[8];
uint8_t can_receive_online_time=100;
void can_filter_init(void)
{

    CAN_FilterTypeDef can_filter_st;
		
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = CAN_ID_EXT|CAN_RTR_DATA;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterBank = 0;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan, &can_filter_st);
    HAL_CAN_Start(&hcan);
    HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);

}

/**
*   @brief  CAN接收回调函数 pwm uint16
*   @param  none
*   @return none
*   @author Shockely
*/

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
    switch (rx_header.StdId)
    {
        case MOTOR_CTRL:
				{
					memcpy(&(motor1.pwm1),rx_data,sizeof(uint16_t)*2);
					memcpy(&(motor2.pwm1),rx_data+4,sizeof(uint16_t)*2);
					can_receive_online_time=0;
				}
        default:
        {
            break;
        }
    }
}
/**
*   @brief  速度反馈函数
*   @param  motor_t
*   @return none
*   @author Shockely
*/

void can_vel_send(motor_t* motor1,motor_t* motor2)
{
	uint32_t send_mail_box;
  tx_message.IDE = CAN_ID_STD;
  tx_message.RTR = CAN_RTR_DATA;
	tx_message.DLC = 0x08;
	tx_message.StdId=MOTOR_REF;
	memcpy(can_send_data,&motor1->vel,sizeof(float));
	memcpy(can_send_data+4,&motor2->vel,sizeof(float));
	HAL_CAN_AddTxMessage(&hcan, &tx_message, can_send_data, &send_mail_box);
}
