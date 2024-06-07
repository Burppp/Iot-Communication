#ifndef BSP_USART_H
#define BSP_USART_H
#include "struct_typedef.h"

typedef enum
{
    CHASSIS_CTRL = 0,//底盘控制
    YAW_INFO,//陀螺仪数据
    REFEREE,//裁判系统数据
};

extern void usart1_init(uint8_t *rx_buf,uint16_t dma_buf_num);
extern void usart6_init(uint8_t *rx1_buf,uint16_t dma_buf_num);
extern void uart6_tx_one_byte(uint8_t* data);
extern void uart6_tx_mul_byte_dma(uint8_t* data,uint16_t len);
extern void usart6_tx_dma_enable(uint8_t* data,uint16_t len);
extern void usart1_tx_dma_init(void);
extern void usart1_tx_dma_enable(uint8_t *data, uint16_t len);
extern void usart3_init(uint8_t*rx_buf, uint16_t dma_buf_num);
extern void uart_send_data(uint8_t *buffer, uint16_t length, uint8_t id);

#endif
