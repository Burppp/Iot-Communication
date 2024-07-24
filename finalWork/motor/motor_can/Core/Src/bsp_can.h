#ifndef BSP_CAN_H
#define BSP_CAN_H
#include "main.h"
typedef enum{
	MOTOR_CTRL=0x0001,
	MOTOR_REF=0x0002
}motor_id_e;

extern motor_t motor1;
extern motor_t motor2;
extern void can_filter_init(void);
extern void can_vel_send(motor_t* motor1,motor_t* motor2);
extern uint8_t can_receive_online_time;
#endif
