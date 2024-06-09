//
// Created by Lumos on 2024/6/8.
//

#ifndef REMOTE_CUBE_SENTRY_CAN_RECEIVE_H
#define REMOTE_CUBE_SENTRY_CAN_RECEIVE_H

/*轮子电机id：                                 解算坐标：      x(前)
            ****      前       ****                                 |
           * 2 *             * 1 *                                 |
            ****              ****                                 |
                                                                   |
           左                   右                    --------------z-----------y(右)
                                                                   |
            ****              ****                                 |
           * 3 *            * 4  *                                 |
            ****      后      ****                                 |

*/

#ifndef _CAN_RECEIVE_H_
#define _CAN_RECEIVE_H_
#include "struct_typedef.h"
#include "PID.h"
/******************** define *******************/

#define HALF_ECD_RANGE  4096
#define ECD_RANGE       8192
#define MOTOR_ECD_TO_RAD 0.000766990394f //      2*  PI  /8192
#define MOTOR_ECD_TO_ANGLE 0.0439453125f     //        360  /8192
#define MOTOR_RAD_TO_ANGLE 57.29577951308238f // 360*2PI
#define MOTOR_ANGLE_TO_RAD  0.0174532925199433f
/******************** struct *******************/

//CAN_ID 该枚举不区分CAN_1还是CAN_2
typedef enum
{
    //电机控制 发送ID
    CAN_MOTOR_0x200_ID = 0x200,
    CAN_MOTOR_0x1FF_ID = 0x1FF,
    CAN_MOTOR_0x2FF_ID = 0x2FF,

    //0X200对应的电机ID(CAN1)
    CAN_CHASSIS_3508_MOTOR_RF=0x202,
    CAN_CHASSIS_3508_MOTOR_LF=0x201,
    CAN_CHASSIS_3508_MOTOR_LB=0x204,
    CAN_CHASSIS_3508_MOTOR_RB=0x203,
} can_msg_id_e;

typedef enum {
    CAN_1,
    CAN_2,
}CAN_TYPE;

//电机的数据
typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;

    int32_t round_cnt;   //电机旋转的总圈数
    int32_t total_ecd;   //电机旋转的总编码器数值

    uint16_t offset_ecd;//电机的校准编码值

    int32_t total_dis;   //电机总共走的距离

    float torque_round_cnt;//转子转过的角度（-360-360）
    float real_round_cnt;//轮子转过的角度
    float real_angle_deg;
} motor_measure_t;

typedef struct
{
    const motor_measure_t *motor_measure;

    fp32 speed;

    fp32 rpm_set;

    pid_t speed_p;

    int16_t give_current;


}motor_3508_t;

/******************** extern *******************/

extern motor_measure_t motor_2006_measure[6];

void CAN_cmd_motor(CAN_TYPE can_type,can_msg_id_e CMD_ID,int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);

#endif


#endif //REMOTE_CUBE_SENTRY_CAN_RECEIVE_H
