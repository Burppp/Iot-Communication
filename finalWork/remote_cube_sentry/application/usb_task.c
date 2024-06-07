//
// Created by Shockley on 2022/11/9.
//

#include "usb_task.h"
#include "cmsis_os.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "CRC8_CRC16.h"
#include "packet.h"
#include "fifo.h"
#include "protocol_shaob.h"
#include "stdio.h"
#include <stdio.h>
#include <stdarg.h>
static uint8_t usb_buf[128];
extern QueueHandle_t CDC_send_queue;
extern void rm_dequeue_send_data(void* buf,uint16_t len);

_Noreturn void usb_task(void const * argument)
{
    MX_USB_DEVICE_Init();
    while(1)
    {
        if(xQueueReceive( CDC_send_queue, usb_buf, 10 ) == pdTRUE)
        {
            rm_dequeue_send_data(usb_buf,128);
        }
        osDelay(2);
    }

}