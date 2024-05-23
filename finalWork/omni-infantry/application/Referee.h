//
// Created by xhuanc on 2021/10/23.
// Update by zxk on 2024/3/12
//

#ifndef _REFEREE_H_
#define _REFEREE_H_
#include "struct_typedef.h"
#include "stdbool.h"

#define REFREE_HEADER_SOF 0xA5
#define REF_PROTOCOL_FRAME_MAX_SIZE         128

#define REF_PROTOCOL_HEADER_SIZE            sizeof(frame_header_struct_t)
#define REF_PROTOCOL_CMD_SIZE               2
#define REF_PROTOCOL_CRC16_SIZE             2
#define REF_HEADER_CRC_LEN                  (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE)
#define REF_HEADER_CRC_CMDID_LEN            (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE + sizeof(uint16_t))
#define REF_HEADER_CMDID_LEN                (REF_PROTOCOL_HEADER_SIZE + sizeof(uint16_t))
#define REFEREE_BUFFER_SIZE     200

#define LEN_HEADER 5

//通信协议格式 位移量
typedef enum
{
    FRAME_HEADER= 0,
    CMD_ID               = 5,
    DATA                 = 7,
}RefereeFrameOffset;

// frame_header 格式
typedef enum
{
    SOF          = 0,//起始位
    DATA_LENGTH  = 1,//帧内数据长度,根据这个来获取数据长度
    SEQ          = 3,//包序号
    CRC8         = 4 //CRC8
}FrameHeaderOffset;

//裁判系统命令ID
/***************命令码ID********************

	ID: 0x0001  Byte:  3    比赛状态数据       			发送频率 1Hz
	ID: 0x0002  Byte:  1    比赛结果数据         		比赛结束后发送
	ID: 0x0003  Byte:  32   比赛机器人血量数据   		1Hz发送       **
	ID: 0x0004  Byte:  3   	飞镖发射状态  		?		飞镖发射时发送**
	ID: 0x0005  Byte:  3   	人工智能挑战赛加成与惩罚数据   **

	ID: 0x0101  Byte:  4    场地事件数据   				事件改变后发送
	ID: 0x0102  Byte:  3    场地补给站动作标识数据    	动作改变后发送
	ID: 0X0104  Byte:  2    裁判警告数据
	ID: 0x0105  Byte:  1    飞镖发射口倒计时

	ID: 0X0201  Byte: 15    机器人状态数据        		10Hz
	ID: 0X0202  Byte: 14    实时功率热量数据   			50Hz
	ID: 0x0203  Byte: 16    机器人位置数据           	10Hz
	ID: 0x0204  Byte:  1    机器人增益数据           	增益状态改变后发送
	ID: 0x0205  Byte:  3    空中机器人能量状态数据      10Hz
	ID: 0x0206  Byte:  1    伤害状态数据           		伤害发生后发送
	ID: 0x0207  Byte:  6    实时射击数据           		子弹发射后发送
	ID: 0x0208  Byte:  2    弹丸剩余数量  仅空中机器人 哨兵
	ID: 0x0209  Byte:  4    机器人RFID状态

	ID: 0x0301  Byte:  n    机器人间交互数据           	发送方触发发送,10Hz

*/
////命令码，修改前
//typedef enum
//{
//    Referee_ID_game_state                   = 0x0001,
//    Referee_ID_game_result                  = 0x0002,
//    Referee_ID_game_robot_survivors       	= 0x0003,//比赛机器人存活数据
//    Referee_ID_game_dart_state              = 0x0004, //飞镖发射状态
//    Referee_ID_game_buff                    = 0x0005,//buff
//    Referee_ID_event_data  					= 0x0101,//场地事件数据
//    Referee_ID_supply_projectile_action   	= 0x0102,//场地补给站动作标识数据
//    Referee_ID_supply_warm 	                = 0x0104,//裁判系统警告数据
//    Referee_ID_dart_shoot_time              = 0x0105, //飞镖发射口倒计时
//    Referee_ID_game_robot_state    			= 0x0201,//机器人状态数据
//    Referee_ID_power_heat_data    			= 0x0202,//实时功率热量数据
//    Referee_ID_game_robot_pos        		= 0x0203,//机器人位置数据
//    Referee_ID_buff_musk					= 0x0204,//机器人增益数据
//    Referee_ID_aerial_robot_energy			= 0x0205,//空中机器人能量状态数据
//    Referee_ID_robot_hurt					= 0x0206,//伤害状态数据
//    Referee_ID_shoot_data					= 0x0207,//实时射击数据
//    Referee_ID_bullet_remaining             = 0x0208,//剩余发射数
//    Referee_ID_rfid_status					= 0x0209,//机器人RFID状态，1Hz
//    Referee_ID_dart_client_directive        = 0x020A,//飞镖机器人客户端指令书, 10Hz
//    Referee_ID_robot_interactive_header_data	  = 0x0301,//机器人交互数据，——发送方触发——发送 10Hz
//    Referee_ID_controller_interactive_header_data = 0x0302,//自定义控制器交互数据接口，通过——客户端触发——发送 30Hz
//    Referee_ID_map_interactive_header_data        = 0x0303,//客户端小地图交互数据，——触发发送——
//    Referee_ID_keyboard_information               = 0x0304,//键盘、鼠标信息，通过——图传串口——发送
////    IDCustomData,
//}referee_cmd_id_t;


//修改后
typedef enum
{
    Referee_ID_game_state                   = 0x0001,//比赛状态数据
    Referee_ID_game_result                  = 0x0002,//比赛结果数据
//    Referee_ID_game_robot_survivors       	= 0x0003,//比赛机器人存活数据->机器人血量数据
    Referee_ID_game_robot_HP       	        = 0x0003,//比赛机器人存活数据->机器人血量数据
//    Referee_ID_game_dart_state              = 0x0004,//飞镖发射状态  删除
//    Referee_ID_game_buff                    = 0x0005,//buff        删除

    Referee_ID_event_data  					= 0x0101,//场地事件数据
    Referee_ID_supply_projectile_action   	= 0x0102,//场地补给站动作标识数据
    Referee_ID_supply_warm 	                = 0x0104,//裁判系统警告数据
//    Referee_ID_dart_shoot_time              = 0x0105, //飞镖发射口倒计时->飞镖发射相关数据
    Referee_ID_dart_info                    = 0x0105, //飞镖发射口倒计时->飞镖发射相关数据

    Referee_ID_game_robot_state    			= 0x0201,//机器人状态数据
    Referee_ID_power_heat_data    			= 0x0202,//实时功率热量数据
    Referee_ID_game_robot_pos        		= 0x0203,//本机器人位置数据
    Referee_ID_buff_musk					= 0x0204,//机器人增益数据
    Referee_ID_aerial_robot_energy			= 0x0205,//空中机器人能量状态数据
    Referee_ID_robot_hurt					= 0x0206,//伤害状态数据
    Referee_ID_shoot_data					= 0x0207,//实时射击数据
    Referee_ID_bullet_remaining             = 0x0208,//剩余发射数
    Referee_ID_rfid_status					= 0x0209,//机器人RFID状态，3Hz
    Referee_ID_dart_client_directive        = 0x020A,//飞镖机器人客户端指令书, 3Hz
    Referee_ID_dart_all_robot_position      = 0x020B,//己方所有机器人位置数据
    Referee_ID_radar_mark                   = 0x020C,//雷达标记进度数据
    Referee_ID_entry_info                   = 0x020D,//哨兵自主决策信息同步 1Hz
    Referee_ID_radar_info                   = 0x020E,//雷达自主决策信息同步 1Hz

    Referee_ID_robot_interactive_header_data	    = 0x0301,//机器人交互数据，——发送方触发——发送 10Hz
    Referee_ID_controller_interactive_header_data   = 0x0302,//自定义控制器与机器人交互数据，发送方触发发送，频率上限30Hz
    Referee_ID_map_command                          = 0x0303,//选手端小地图交互数据，选手端触发发送
    Referee_ID_keyboard_information                 = 0x0304,//键鼠遥控数据，固定30Hz,图传链路
    Referee_ID_robot_map_robot_data                 = 0x0305,//选手端小地图接收雷达数据,上限10Hz
    Referee_ID_robot_custom_client                  = 0x0306,//自定义控制器与选手端交互数据，发送方触发发送，频率上限30Hz
    Referee_ID_robot_entry_info_receive             = 0x0307,//选手端小地图接收哨兵数据,上限1Hz
    Referee_ID_robot_custom_info_receive            = 0x0308,//通过常规链路接收机器人的数据,在特定位置显示，上限3Hz
//    IDCustomData,?
}referee_cmd_id_t;



//裁判系统各命令的数据长度
typedef enum
{
    /* Std */
    Referee_LEN_FRAME_HEAD 	                    = 5,	// 帧头长度
    Referee_LEN_CMD_ID 		                    = 2,	// 命令码长度
    Referee_LEN_FRAME_TAIL 	                    = 2,	// 帧尾CRC16
    // 帧尾CRC16
    /* Ext */

    Referee_LEN_game_state       				=  11,	//0x0001
    Referee_LEN_game_result       				=  1,	//0x0002
    Referee_LEN_game_robot_HP     		        =  32,	//0x0003  比赛机器人血量数据
//    Referee_LEN_game_robot_survivors       		=  32,	//0x0003  比赛机器人血量数据
//    Referee_LED_game_missile_state              = 3  , //0X0004飞镖发射状态  删除
//    Referee_LED_game_buff                       =11 , //0X0005

    Referee_LEN_event_data  					=  4,	//0x0101  场地事件数据
    Referee_LEN_supply_projectile_action        =  4,	//0x0102场地补给站动作标识数据
    Referee_LEN_supply_warm                     = 3,    //0x0104 裁判系统警告
//    Referee_LEN_missile_shoot_time              = 3,    //0x0105 飞镖发射口倒计时
    Referee_LEN_dart_info                       = 3,    //0x0105 飞镖发射口倒计时

    Referee_LEN_game_robot_state    			= 13,	//0x0201 机器人状态数据
    Referee_LEN_power_heat_data   				= 16,	//0x0202 实时功率热量数据
    Referee_LEN_game_robot_pos        			= 16,	//0x0203 机器人位置数据
    Referee_LEN_buff_musk        				=  6,	//0x0204 机器人增益数据
    Referee_LEN_aerial_robot_energy        		=  2,	//0x0205 空中机器人能量状态数据
    Referee_LEN_robot_hurt        				=  1,	//0x0206 伤害状态数据
    Referee_LEN_shoot_data       				=  7,	//0x0207 实时射击数据
    Referee_LEN_bullet_remaining                = 6,    //0x0208剩余发射数
    Referee_LEN_rfid_status					    = 4,    //0x0209
    Referee_LEN_dart_client_directive           = 6,    //0x020A
    Referee_LEN_dart_all_robot_position         = 40,   //0x020B
    Referee_LEN_radar_mark                      = 6,    //0x020C
    Referee_LEN_entry_info                      =4,     //0x020D
    Referee_LEN_radar_info                      =1,     //0x020E

    Referee_LEN_robot_interactive_header_data   =128,   //0x0301
    Referee_LEN_controller_interactive_header_data=30,  //0x0302
    Referee_LEN_map_command                     =15,    //0x0303
    Referee_LEN_keyboard_information            =12,    //0x0304
    Referee_LEN_robot_map_robot_data            =10,    //0x0305
    Referee_LEN_robot_custom_client             =8,     //0x0306
    Referee_LEN_robot_entry_info_receive        =103,   //0x0307
    Referee_LEN_robot_custom_info_receive       =34,    //0x0308

}RefereeDataLength;


typedef enum{
    Referee_hero_red       = 1,
    Referee_engineer_red   = 2,
    Referee_infantry3_red  = 3,
    Referee_infantry4_red  = 4,
    Referee_infantry5_red  = 5,
    Referee_plane_red      = 6,

    Referee_hero_blue      = 101,
    Referee_engineer_blue  = 102,
    Referee_infantry3_blue = 103,
    Referee_infantry4_blue = 104,
    Referee_infantry5_blue = 105,
    Referee_plane_blue     = 106,
}Referee_robot_ID;


typedef struct {

    bool static_update;//静态元素是否要刷新
    uint8_t gimbal_mode;//云台模式 是人控 还是自控（自控是打符还是自瞄)
    uint8_t chassis_mode;//
    uint8_t block_warning;//堵弹警告
    uint8_t shoot_heat_limit;//当前热量限制
    fp32 super_cap_value;//超级电容值
    uint8_t fire_mode;
    float pitch_value;
    float relative_yaw_value;
}ui_robot_status_t;//机器人状态结构体 如果陀螺是否打开,弹舱是否打开，自瞄是否打开等信息

//裁判系统帧头结构体
typedef  struct
{
    uint8_t SOF;
    uint16_t data_length;
    uint8_t seq;
    uint8_t CRC8;
}__packed frame_header_struct_t;


/* ID: 0x0001  Byte:  11    比赛状态数据 */
typedef  struct
{
    uint8_t game_type : 4;
    uint8_t game_progress : 4;
    uint16_t stage_remain_time;
    uint64_t SyncTimeStamp;
} __packed ext_game_state_t;


/* ID: 0x0002  Byte:  1    比赛结果数据 */
typedef  struct
{
    uint8_t winner;
    bool game_over;
}__packed  ext_game_result_t;


/* ID: 0x0003  Byte:  32    比赛机器人血量数据 */
typedef  struct
{
    uint16_t red_1_robot_HP;
    uint16_t red_2_robot_HP;
    uint16_t red_3_robot_HP;
    uint16_t red_4_robot_HP;
    uint16_t red_5_robot_HP;
    uint16_t red_6_robot_HP;
    uint16_t red_7_robot_HP;
    uint16_t red_outpost_HP;
    uint16_t red_base_HP;

    uint16_t blue_1_robot_HP;
    uint16_t blue_2_robot_HP;
    uint16_t blue_3_robot_HP;
    uint16_t blue_4_robot_HP;
    uint16_t blue_5_robot_HP;
    uint16_t blue_6_robot_HP;
    uint16_t blue_7_robot_HP;

    uint16_t blue_outpost_HP;
    uint16_t blue_base_HP;
} __packed ext_game_robot_HP_t;

//V1.6.1删除
///* ID: 0x0004  Byte:  3    飞镖发射状态 */
//typedef  struct
//{
//    uint8_t dart_belong;
//    uint16_t stage_remaining_time;
//} __packed ext_dart_status_t;

//V1.6.1删除
///* ID: 0x0005  Byte:  11    buff */
//typedef  struct
//{
//    uint8_t F1_zone_status:1;
//    uint8_t F1_zone_buff_debuff_status:3;
//
//    uint8_t F2_zone_status:1;
//    uint8_t F2_zone_buff_debuff_status:3;
//
//    uint8_t F3_zone_status:1;
//    uint8_t F3_zone_buff_debuff_status:3;
//
//    uint8_t F4_zone_status:1;
//    uint8_t F4_zone_buff_debuff_status:3;
//
//    uint8_t F5_zone_status:1;
//    uint8_t F5_zone_buff_debuff_status:3;
//
//    uint8_t F6_zone_status:1;
//    uint8_t F6_zone_buff_debuff_status:3;
//
//    uint16_t red1_bullet_left;
//
//    uint16_t red2_bullet_left;
//
//    uint16_t blue1_bullet_left;
//
//    uint16_t blue2_bullet_left;
//
//}__packed  ext_ICRA_buff_debuff_zone_status_t;


/* ID: 0x0101  Byte:  4    场地事件数据 */
typedef  struct
{
    uint32_t event_type;
} __packed ext_event_data_t;


/* ID: 0x0102  Byte:  4    场地补给站动作标识数据 */
typedef  struct
{
//    uint8_t supply_projectile_id;  0
    uint8_t reserved;
    uint8_t supply_robot_id;
    uint8_t supply_projectile_step;
    uint8_t supply_projectile_num;
} __packed ext_supply_projectile_action_t;

//V1.6.1修改
/* ID: 0x0104  Byte: 2->3   裁判系统警告信息 */
typedef  struct
{
//    uint8_t level;
//    uint8_t foul_robot_id;
    uint8_t level;
    uint8_t offending_robot_id;
    uint8_t count;
} __packed  ext_referee_warning_t;

//V1.6.1修改
/* ID: 0x0105  Byte:1->3  飞镖发射口倒计时 */
typedef  struct
{
    uint8_t dart_remaining_time;
    uint16_t dart_info;  //新增
} __packed ext_dart_remaining_time_t;

//V1.6.1修改
///* ID: 0X0201  Byte: 27    机器人状态数据 */
//typedef  struct
//{
//    uint8_t robot_id;   //机器人ID，可用来校验发送
//    uint8_t robot_level;  //1一级，2二级，3三级
//    uint16_t remain_HP;  //机器人剩余血量
//    uint16_t max_HP; //机器人血量上限
//
//    uint16_t shooter1_17mm_cooling_rate;  //机器人 17mm 子弹热量冷却速度 单位 /s
//    uint16_t shooter1_17mm_cooling_limit;   // 机器人 17mm 子弹热量上限
//    uint16_t shooter1_17mm_speed_limit;
//
//
//    uint16_t shooter2_17mm_cooling_rate;
//    uint16_t shooter2_17mm_cooling_limit;
//    uint16_t shooter2_17mm_speed_limit;
//
//
//    uint16_t shooter_42mm_cooling_rate;
//    uint16_t shooter_42mm_cooling_limit;
//    uint16_t shooter_42mm_speed_limit;
//
//
//    uint16_t max_chassis_power;
//    uint8_t mains_power_gimbal_output : 1;
//    uint8_t mains_power_chassis_output : 1;
//    uint8_t mains_power_shooter_output : 1;
//} __packed ext_game_robot_state_t;

/* ID: 0X0201  Byte:     机器人状态数据 */

typedef  struct
{
    uint8_t robot_id;
    uint8_t robot_level;
    uint16_t current_HP;
    uint16_t maximum_HP;

    uint16_t shooter_barrel_cooling_value;
    uint16_t shooter_barrel_heat_limit;
    uint16_t  chassis_power_limit;

    uint8_t power_management_gimbal_output : 1;
    uint8_t power_management_chassis_output : 1;
    uint8_t power_management_shooter_output : 1;
} __packed ext_robot_status_t;

/* ID: 0X0202  Byte: 16    实时功率热量数据 */
typedef  struct
{
    uint16_t chassis_volt;
    uint16_t chassis_current;
    float chassis_power;   //瞬时功率
    uint16_t chassis_power_buffer;//60焦耳缓冲能量
    uint16_t shooter_heat0;//第1个 17mm 发射机构的枪口热量
    uint16_t shooter_heat1;// 第2个 17mm 发射机构的枪口热量
    uint16_t mobile_shooter_heat2;//42mm 发射机构的枪口热量
} __packed ext_power_heat_data_t;

//V1.6.1修改
///* ID: 0x0203  Byte: 16    机器人位置数据 */
//typedef  struct
//{
//    float x;
//    float y;
//    float z;
//    float yaw;
//} __packed ext_game_robot_pos_t;

/* ID: 0x0203  Byte: 16    机器人位置数据 */
typedef  struct
{
    float x;//单位m
    float y;
    float angle;//本机器人测速模块的朝向，单位：度。正北为 0 度
}__packed ext_robot_pos_t;

//V1.6.1修改
///* ID: 0x0204  Byte:  1    机器人增益数据 */
//typedef  struct
//{
//    uint8_t power_rune_buff;
//} __packed ext_buff_musk_t;

/* ID: 0x0204  Byte:  1    机器人增益数据 */
typedef struct
{
    uint8_t recovery_buff;          //回血,值表示百分比
    uint8_t cooling_buff;           //枪口冷却倍率,5表示5倍冷却
    uint8_t defence_buff;           //增防,下面三个都是百分比
    uint8_t vulnerability_buff;     //减防
    uint16_t attack_buff;           //攻击
}__packed  ext_buff_t;

/* ID: 0x0205  Byte:  1->2    空中机器人能量状态数据 */
typedef  struct
{
    uint8_t airforce_status; //空中机器人状态（0 正在冷却，1 冷却完毕，2 正在空中支援） V1.6.1新增
    uint8_t attack_time;
} __packed aerial_robot_energy_t;


/* ID: 0x0206  Byte:  1    伤害状态数据 */
typedef  struct
{
    uint8_t armor_id : 4;
    uint8_t hurt_type : 4;
    bool being_hurt;
} __packed ext_robot_hurt_t;


/* ID: 0x0207  Byte:  7    实时射击数据 */
typedef  struct
{
    uint8_t bullet_type;    // 1 17mm, 2 42mm
    uint8_t shooter_id;     // 1 17mm1, 2 17mm2, 3 42mm
    uint8_t bullet_freq;    //射速（单位：Hz）
    float bullet_speed;     //丸初速度（单位：m/s)
}__packed  ext_shoot_data_t;


/* ID: 0x0208  Byte:  6    子弹剩余数量 */
typedef  struct
{
    uint16_t bullet_remaining_num_17mm;
    uint16_t bullet_remaining_num_42mm;
    uint16_t coin_remaining_num;//金币剩余
} __packed ext_bullet_remaining_t;

/* ID: 0x0209  Byte:  4 	机器人RFID状态 */
typedef  struct
{
    uint32_t rfid_status;
} __packed ext_rfid_status_t;

////V1.6.1修改
///* ID:  0x020A  Byte:   	 */
//typedef  struct{
//    uint8_t dart_launch_opening_status;//当前飞镖发射口的状态
//    uint8_t dart_attack_target;        //飞镖的打击目标，默认为前哨站（1：前哨站，2：基地）
//    uint16_t target_change_time;       //切换打击目标时的比赛剩余时间
//    uint8_t first_dart_speed;          //检测到的第一枚飞镖速度，单位 0.1m/s/LSB
//    uint8_t second_dart_speed;         //检测到的第二枚飞镖速度，单位 0.1m/s/LSB
//    uint8_t third_dart_speed;          //检测到的第三枚飞镖速度，单位 0.1m/s/LSB
//    uint8_t fourth_dart_speed;         //检测到的第四枚飞镖速度，单位 0.1m/s/LSB
//    uint16_t last_dart_launch_time;    //最近一次的发射飞镖的比赛剩余时间，单位秒
//    uint16_t operate_launch_cmd_time;  //最近一次操作手确定发射指令时的比赛剩余时间，单位秒
//}__packed ext_dart_client_cmd_t; //LEN_DART_CLIENT_DIRECTIVE  表3-19

/* ID:  0x020A  Byte:6   	 */
typedef  struct
{
    uint8_t dart_launch_opening_status;     //飞镖发射状态 0 已经开启, 1 关闭, 2 正在开启或者关闭中,
    uint8_t reserved;                       //保留
    uint16_t target_change_time;            //切换击打目标时的比赛剩余时间，单位s,默认为0
    uint16_t latest_launch_cmd_time;        //最后一次操作手确定发射指令时的比赛剩余时间,单位s,初始值为0
}__packed ext_dart_client_cmd_t;

//V1.6.1新增 24赛季自动控制
/* ID:   0x020B  Byte:40   	 */
typedef  struct
{
    float hero_x;
    float hero_y;

    float engineer_x;
    float engineer_y;

    float standard_3_x;
    float standard_3_y;

    float standard_4_x;
    float standard_4_y;

    float standard_5_x;
    float standard_5_y;
}__packed ext_ground_robot_position_t;

//V1.6.1新增 24赛季雷达修改
/* ID:   0x020C  Byte:6  机器人被雷达标记进度 0-120	 */
typedef  struct
{
    uint8_t mark_hero_progress;
    uint8_t mark_engineer_progress;
    uint8_t mark_standard_3_progress;
    uint8_t mark_standard_4_progress;
    uint8_t mark_standard_5_progress;
    uint8_t mark_sentry_progress;
}__packed ext_radar_mark_data_t;

//V1.6.1新增 24赛季哨兵修改
/* ID:   0x020D  Byte:4  哨兵兑换发单量和血量信 */
typedef  struct
{
    uint32_t sentry_info;
} __packed ext_sentry_info_t;

//V1.6.1新增 24赛季雷达修改
/* ID:   0x020E  Byte:1  雷达触发双伤信息	 */
typedef struct
{
    uint8_t radar_info;
} __packed  ext_radar_info_t;

/*
	交互数据，包括一个统一的数据段头结构，
	包含了内容 ID，发送者以及接受者的 ID 和内容数据段，
	整个交互数据的包总共长最大为 128 个字节，
	减去 frame_header,cmd_id,frame_tail 以及数据段头结构的 6 个字节，
	故而发送的内容数据段最大为 113。
	整个交互数据 0x0301 的包上行频率为 10Hz。

	机器人 ID：
	1，英雄(红)；
	2，工程(红)；
	3/4/5，步兵(红)；
	6，空中(红)；
	7，哨兵(红)；
	11，英雄(蓝)；
	12，工程(蓝)；
	13/14/15，步兵(蓝)；
	16，空中(蓝)；
	17，哨兵(蓝)。
	客户端 ID：
	0x0101 为英雄操作手客户端( 红) ；
	0x0102 ，工程操作手客户端 ((红 )；
	0x0103/0x0104/0x0105，步兵操作手客户端(红)；
	0x0106，空中操作手客户端((红)；
	0x0111，英雄操作手客户端(蓝)；
	0x0112，工程操作手客户端(蓝)；
	0x0113/0x0114/0x0115，操作手客户端步兵(蓝)；
	0x0116，空中操作手客户端(蓝)。
*/


/* 交互数据接收信息：0x0301  */
typedef  struct
{
    uint16_t data_cmd_id;
    uint16_t send_ID;
    uint16_t receiver_ID;
}__packed ext_student_interactive_header_data_t;

typedef struct{
    uint16_t teammate_hero;
    uint16_t teammate_engineer;
    uint16_t teammate_infantry3;
    uint16_t teammate_infantry4;
    uint16_t teammate_infantry5;
    uint16_t teammate_plane;
    uint16_t teammate_sentry;

    uint16_t client_hero;
    uint16_t client_engineer;
    uint16_t client_infantry3;
    uint16_t client_infantry4;
    uint16_t client_infantry5;
    uint16_t client_plane;
}ext_interact_id_t;

/* 选手端小地图交互数据：0x0303  */
typedef  struct
{
    float target_position_x;
    float target_position_y;
    uint8_t cmd_keyboard;
    uint8_t target_robot_id;
    uint8_t cmd_source;
}__packed ext_map_command_t;

/* 键鼠遥控数据：0x0304  */
typedef struct
{
    int16_t mouse_x;
    int16_t mouse_y;
    int16_t mouse_z;
    int8_t left_button_down;
    int8_t right_button_down;
    uint16_t keyboard_value;
    uint16_t reserved;
}__packed ext_remote_control_t;

/* 选手端小地图接收雷达数据：0x0305  */
typedef struct
{
    uint16_t target_robot_id;
    float target_position_x;
    float target_position_y;
} ext_map_robot_data_t;

/* 自定义控制器与选手端交互数据：0x0306  */
typedef struct
{
    uint16_t key_value;
    uint16_t x_position:12;
    uint16_t mouse_left:4;
    uint16_t y_position:12;
    uint16_t mouse_right:4;
    uint16_t reserved;
}__packed ext_custom_client_data_t;

/* 选手端小地图接收哨兵数据：0x0307  */
typedef  struct
{
    uint8_t intention;
    uint16_t start_position_x;
    uint16_t start_position_y;
    int8_t delta_x[49];
    int8_t delta_y[49];
    uint16_t sender_id;
}__packed ext_map_data_t;

/* 选手端小地图接收机器人数据：0x0308  */
typedef  struct
{
    uint16_t sender_id;
    uint16_t receiver_id;
    uint8_t user_data[30];
} __packed ext_custom_info_t;

/*
	学生机器人间通信 cmd_id 0x0301，内容 ID:0x0200~0x02FF
	交互数据 机器人间通信：0x0301。
	发送频率：上限 10Hz

	字节偏移量 	大小 	说明 			备注
	0 			2 		数据的内容 ID 	0x0200~0x02FF
										可以在以上 ID 段选取，具体 ID 含义由参赛队自定义

	2 			2 		发送者的 ID 	需要校验发送者的 ID 正确性，

	4 			2 		接收者的 ID 	需要校验接收者的 ID 正确性，
										例如不能发送到敌对机器人的ID

	6 			n 		数据段 			n 需要小于 113

*/
typedef  struct
{
    uint8_t data[113]; //数据段,n需要小于113
} __packed robot_interactive_data_t;

typedef struct judge_info_struct {
    frame_header_struct_t 							FrameHeader;				// 帧头信息

    ext_game_state_t 							    GameState;				    // 0x0001           比赛状态数据
    ext_game_result_t 							    GameResult;				    // 0x0002         比赛结果数据
    ext_game_robot_HP_t 						    GameRobotHP;			    // 0x0003         机器人血量数据
//    ext_dart_status_t								GameDartStatus;				// 0x0004         飞镖发射状态
//    ext_ICRA_buff_debuff_zone_status_t	            GameICRABuff;           // 人工智能挑战赛加成与惩罚区状态  删除

    ext_event_data_t								EventData;					// 0x0101         场地事件数据
    ext_supply_projectile_action_t	                SupplyProjectileAction;		// 0x0102 补给站动作标识
    ext_referee_warning_t						    RefereeWarning;		        // 0x0104         裁判警告信息
    ext_dart_remaining_time_t				        DartRemainingTime;          // 0x0105         飞镖发射口倒计时


    ext_robot_status_t					            GameRobotStat;	            // 0x0201         比赛机器人状态
    ext_power_heat_data_t						    PowerHeatData;		        // 0x0202         实时功率热量数据
    ext_robot_pos_t						            GameRobotPos;			    // 0x0203         机器人位置
    ext_buff_t									    Buff;						// 0x0204     机器人增益
    aerial_robot_energy_t				            AerialRobotEnergy;// 0x0205             空中机器人能量状态
    ext_robot_hurt_t								RobotHurt;					//0x0206         伤害状态
    ext_shoot_data_t								ShootData;					//0x0207         实时射击信息(射频  射速  子弹信息)
    ext_bullet_remaining_t					        BulletRemaining;		    //0x0208	        子弹剩余发射数
    ext_rfid_status_t								RfidStatus;				    //0x0209	        RFID信息
    ext_dart_client_cmd_t                           DartClient;                 //0x020A         飞镖客户端
    ext_ground_robot_position_t                     RobotPosition;              //0x020B
    ext_radar_mark_data_t                           RadarMark;                  //0x020C
    ext_sentry_info_t                               SentryInfo;                 //0x020D
    ext_radar_info_t                                RadarInfo;                  //0x020E

    ext_student_interactive_header_data_t           StudentInteractive;//0x0301
    ext_map_command_t                               MapCommand;//0x0303
    ext_remote_control_t                            keyboard;      //0x0304 键鼠
    ext_map_robot_data_t                            EnemyPosition;      //0x0305 敌方机器人位置
    ext_custom_client_data_t                        Custom;             //0x0306 自定义控制器
    ext_map_data_t                                  SentryMapData;      //0x0307哨兵发送数据
    ext_custom_info_t                               SendData;           //0x0308 机器人自定义发送消息

    ext_interact_id_t								ids;								//与本机交互的机器人id
    uint16_t                                        SelfClient;       //本机客户端

} Referee_info_t;


/*
	学生机器人间通信 cmd_id 0x0301，内容 data_ID:0x0200~0x02FF
	交互数据 机器人间通信：0x0301。
	发送频率：数据上下行合计带宽不超过 5000 Byte。 上下行发送频率分别不超过30Hz。
 * +------+------+-------------+------------------------------------+
 * | byte | size |    breif    |            note                    |
 * |offset|      |             |                                    |
 * +------+------+-------------+------------------------------------+
 * |  0   |  2   | 	 data_ID   | 0x0200~0x02FF,可以在这些 ID 段选取 |
 * |      |      |             | 具体ID含义由参赛队自定义           |
 * +------|------|-------------|------------------------------------|
 * |  2   |  2   | 	sender_ID  | 需要校验发送者的 ID 正确性					|
 * +------|------|-------------|------------------------------------|
 * |  4   |  2   | receiver_ID | 需要校验接收者的 ID 正确性					|
 * |      |      |             | 例如不能发送到敌对机器人的ID				|
 * +------|------|-------------|------------------------------------|
 * |  6   |  n   | 		Data     | n 需要小于 113 										|
 * +------+------+-------------+------------------------------------+
*/


//绘制图形ID
typedef enum
{
    //0x200-0x02ff 	队伍自定义命令 格式  INTERACT_ID_XXXX
    UI_INTERACT_ID_delete_graphic 			= 0x0100,	/*客户端删除图形*/
    UI_INTERACT_ID_draw_one_graphic 		= 0x0101,	/*客户端绘制一个图形*/
    UI_INTERACT_ID_draw_two_graphic 		= 0x0102,	/*客户端绘制2个图形*/
    UI_INTERACT_ID_draw_five_graphic 	= 0x0103,	/*客户端绘制5个图形*/
    UI_INTERACT_ID_draw_seven_graphic 	= 0x0104,	/*客户端绘制7个图形*/
    UI_INTERACT_ID_draw_char_graphic 	= 0x0110,	/*客户端绘制字符图形*/
    UI_INTERACT_ID_bigbome_num					= 0x02ff
}Interact_ID;

typedef enum
{
    UI_LEN_INTERACT_delete_graphic     = 8,  //删除图层 2(数据内容ID)+2(发送者ID)+2（接收者ID）+2（数据内容）
    UI_LEN_INTERACT_draw_one_graphic   = 21, // 以上2+2+2+15
    UI_LEN_INTERACT_draw_two_graphic   = 36, //6+15*2
    UI_LEN_INTERACT_draw_five_graphic  = 81, //6+15*5
    UI_LEN_INTERACT_draw_seven_graphic = 111,//6+15*7
    UI_LEN_INTERACT_draw_char_graphic  = 51, //6+15+30（字符串内容）
}Interact_ID_len;

//图形数据
typedef  struct
{
    uint8_t graphic_name[3];
    uint32_t operate_tpye:3;
    uint32_t graphic_tpye:3;          //直线  矩形  正圆  椭圆  圆弧  浮点  整型  字符
    uint32_t layer:4;
    uint32_t color:4;
    uint32_t start_angle:9;           //空    空    空    空    角度  大小  大小  大小
    uint32_t end_angle:9;             //空    空    空    空          位数  空    长度
    uint32_t width:10;
    uint32_t start_x:11;              //起点  起点  圆心  圆心  圆心  起点  起点  起点
    uint32_t start_y:11;              //
    union {
        struct {
            uint32_t radius:10;      //空    空    半径  空    空    、    、    空
            uint32_t end_x:11;       //终点  对顶  空    半轴  半轴  、    、    空
            uint32_t end_y:11;       //                              数    数    空                  数    数    空
        };
        int32_t number;
    };

} __packed ui_graphic_data_struct_t;//ui开头 将整形和浮点型也视为图形 所以用graphic命名

typedef  struct
{
    frame_header_struct_t txFrameHeader;			//帧头
    uint16_t  CmdID;										//命令码
    ext_student_interactive_header_data_t   dataFrameHeader;//数据段头结构
    ui_graphic_data_struct_t clientData;		//数据段
    uint16_t	FrameTail;								//帧尾
}__packed ext_graphic_one_data_t;

typedef  struct
{
    frame_header_struct_t txFrameHeader;			//帧头
    uint16_t  CmdID;										//命令码
    ext_student_interactive_header_data_t   dataFrameHeader;//数据段头结构
    ui_graphic_data_struct_t clientData[2];		//数据段
    uint16_t	FrameTail;								//帧尾
}__packed ext_graphic_two_data_t;

typedef  struct
{
    frame_header_struct_t txFrameHeader;			//帧头
    uint16_t  CmdID;										//命令码
    ext_student_interactive_header_data_t   dataFrameHeader;//数据段头结构
    ui_graphic_data_struct_t clientData[5];		//数据段
    uint16_t	FrameTail;								//帧尾
}__packed ext_graphic_five_data_t;

typedef  struct
{
    frame_header_struct_t txFrameHeader;			//帧头
    uint16_t  CmdID;										//命令码
    ext_student_interactive_header_data_t   dataFrameHeader;//数据段头结构
    ui_graphic_data_struct_t clientData[7];		//数据段
    uint16_t	FrameTail;								//帧尾
}__packed ext_graphic_seven_data_t;

//绘字符串
//字符串除了ui_graphic_data_struct_t这个结构体变量外
// 还有30个字节的字符数据存储字符串
typedef  struct
{
    ui_graphic_data_struct_t graphic_data_struct;
    uint8_t data[30];
}__packed ui_string_t;

//固定数据段长度数据包
typedef  struct
{
    frame_header_struct_t txFrameHeader;			//帧头
    uint16_t  CmdID;										//命令码
    ext_student_interactive_header_data_t   dataFrameHeader;//数据段头结构
    ui_string_t clientData;//数据段
    uint16_t	FrameTail;								//帧尾
}__packed ext_string_data_t;

//****************************绘图的数据段内容****************************/


//typedef  struct//图形
//{
//    uint8_t graphic_name[3];
//    uint32_t operate_tpye:3;
//    uint32_t graphic_tpye:3; //直线  矩形  正圆  椭圆  圆弧  浮点  整型  字符
//    uint32_t layer:4;
//    uint32_t color:4;
//    uint32_t start_angle:9;  //空    空    空    空    角度  大小  大小  大小
//    uint32_t end_angle:9;    //空    空    空    空          位数  空    长度
//    uint32_t width:10;
//    uint32_t start_x:11;     //起点  起点  圆心  圆心  圆心  起点  起点  起点
//    uint32_t start_y:11;     //
//} __packed graphic_data_struct_t;
//
//typedef  struct//浮点数
//{
//    uint8_t graphic_name[3];
//    uint32_t operate_tpye:3;
//    uint32_t graphic_tpye:3;
//    uint32_t layer:4;
//    uint32_t color:4;
//    uint32_t start_angle:9;
//    uint32_t end_angle:9;
//    uint32_t width:10;
//    uint32_t start_x:11;
//    uint32_t start_y:11;
//    uint32_t number;
//}__packed  float_data_struct_t;
//
//typedef  struct//整型数
//{
//    uint8_t graphic_name[3];
//    uint32_t operate_tpye:3;
//    uint32_t graphic_tpye:3;
//    uint32_t layer:4;
//    uint32_t color:4;
//    uint32_t start_angle:9;
//    uint32_t end_angle:9;
//    uint32_t width:10;
//    uint32_t start_x:11;
//    uint32_t start_y:11;
//    uint32_t number;
//} __packed int_data_struct_t;

/* data_ID: 0X0100  Byte:  2	    客户端删除图形*/
typedef  struct
{
    uint8_t operate_type;
    uint8_t layer;//图层数：0~9
}__packed ext_client_custom_graphic_delete_t;

/* 图层删除操作的枚举 */
typedef enum
{
    UI_NONE_delete    = 0,
    UI_GRAPHIC_delete = 1,
    UI_ALL_delete     = 2
}Delete_Graphic_Operate;//ext_client_custom_graphic_delete_t：uint8_t operate_type

//bit 0-2
typedef enum
{
    UI_NONE   = 0,/*空操作*/
    UI_ADD    = 1,/*增加图层*/
    UI_MODIFY = 2,/*修改图层*/
    UI_DELETE = 3,/*删除图层*/
}Graphic_Operate;//graphic_data_struct_t：uint32_t operate_tpye
/*图层操作*/

//bit3-5
/*图层类型*/
typedef enum
{
    UI_LINE      = 0,//直线
    UI_RECTANGLE = 1,//矩形
    UI_CIRCLE    = 2,//正圆
    UI_OVAL      = 3,//椭圆
    UI_ARC       = 4,//圆弧
    UI_FLOAT     = 5,//浮点数
    UI_INT       = 6,//整型数
    UI_CHAR      = 7 //字符
}Graphic_Type;
/*图层类型*/

//bit 6-9图层数 最大为9，最小0

//bit 10-13颜色
typedef enum
{
    UI_RED_BLUE  = 0,//红蓝主色
    UI_YELLOW    = 1,
    UI_GREEN     = 2,
    UI_ORANGE    = 3,
    UI_FUCHSIA   = 4,	/*紫红色*/
    UI_PINK      = 5,
    UI_CYAN_BLUE = 6,	/*青色*/
    UI_BLACK     = 7,
    UI_WHITE     = 8
}Graphic_Color;

typedef enum {

    UI_ZERO_LAYER=0,
    UI_ONE_LAYER,
    UI_TWO_LAYER,
    UI_THREE_LAYER,
    UI_FOUR_LAYER,
    UI_FIVE_LAYER,
    UI_SIX_LAYER,
    UI_SEVEN_LAYER,
    UI_EIGHT_LAYER,

}Graphic_layer;
/*图层颜色类型*/
//bit 14-31 角度 [0,360]

/*
 * 数据结构体
 */

//删除图层
typedef  struct
{
    frame_header_struct_t txFrameHeader;
    uint16_t  CmdID;
    ext_student_interactive_header_data_t   dataFrameHeader;
    ext_client_custom_graphic_delete_t clientData;
    uint16_t	FrameTail;
} __packed deleteLayer_data_t;

/* ID:   0x0120  Byte:4  哨兵自主决策指令 */
typedef  struct
{
    uint32_t sentry_cmd;
} __packed ext_sentry_cmd_t;
//用来配置ui的颜色
typedef struct
{
    uint32_t cover_color;//弹舱开否
    uint32_t auto_aim_color;//自瞄开否
    uint32_t spin_color;//旋转开否
    uint32_t fire_color;//摩擦轮开启否
    uint32_t ui_color;//UI开启提示颜色
} __packed ext_ui_color;


/* ID:   0x0121  Byte:1  雷达自主决策指令 */
typedef  struct
{
    uint8_t radar_cmd;
} __packed ext_radar_cmd_t;

////绘象形图
//typedef  struct
//{
//    frame_header_struct_t txFrameHeader;			//帧头
//    uint16_t  CmdID;										//命令码
//    ext_student_interactive_header_data_t   dataFrameHeader;//数据段头结构
//    graphic_data_struct_t clientData;		//数据段
//    uint16_t	FrameTail;								//帧尾
//}__packed ext_graphic_one_data_t;
//
//typedef  struct
//{
//    frame_header_struct_t txFrameHeader;
//    uint16_t  CmdID;
//    ext_student_interactive_header_data_t   dataFrameHeader;
//    graphic_data_struct_t clientData[2];
//    uint16_t	FrameTail;
//}__packed ext_graphic_two_data_t;
//
//
//typedef  struct
//{
//    frame_header_struct_t txFrameHeader;
//    uint16_t  CmdID;
//    ext_student_interactive_header_data_t   dataFrameHeader;
//    graphic_data_struct_t clientData[5];
//    uint16_t	FrameTail;
//}__packed ext_graphic_five_data_t;
//
//
//typedef  struct
//{
//    frame_header_struct_t txFrameHeader;
//    uint16_t  CmdID;
//    ext_student_interactive_header_data_t   dataFrameHeader;
//    graphic_data_struct_t clientData[7];
//    uint16_t	FrameTail;
//}__packed ext_graphic_seven_data_t;
//
//
////绘制浮点型
//typedef  struct
//{
//    frame_header_struct_t txFrameHeader;
//    uint16_t  CmdID;
//    ext_student_interactive_header_data_t   dataFrameHeader;
//    float_data_struct_t clientData[2];
//    uint16_t	FrameTail;
//}__packed ext_float_two_data_t;
//
//typedef  struct
//{
//    frame_header_struct_t txFrameHeader;
//    uint16_t  CmdID;
//    ext_student_interactive_header_data_t   dataFrameHeader;
//    float_data_struct_t clientData;
//    uint16_t	FrameTail;
//}__packed ext_float_one_data_t;
//
//
//typedef  struct
//{
//    frame_header_struct_t txFrameHeader;
//    uint16_t  CmdID;
//    ext_student_interactive_header_data_t   dataFrameHeader;
//    float_data_struct_t clientData[7];
//    uint16_t	FrameTail;
//}__packed ext_float_seven_data_t;
//
////绘制整型
//typedef  struct
//{
//    frame_header_struct_t txFrameHeader;
//    uint16_t  CmdID;
//    ext_student_interactive_header_data_t   dataFrameHeader;
//    int_data_struct_t clientData[2];
//    uint16_t	FrameTail;
//}__packed ext_int_two_data_t;
//
//typedef  struct
//{
//    frame_header_struct_t txFrameHeader;
//    uint16_t  CmdID;
//    ext_student_interactive_header_data_t   dataFrameHeader;
//    int_data_struct_t clientData;
//    uint16_t	FrameTail;
//}__packed ext_int_one_data_t;
//
//typedef  struct
//{
//    frame_header_struct_t txFrameHeader;
//    uint16_t  CmdID;
//    ext_student_interactive_header_data_t   dataFrameHeader;
//    int_data_struct_t clientData[7];
//    uint16_t	FrameTail;
//}__packed ext_int_seven_data_t;

//enum {
//    not_start,
//    prepare,
//    self_detect,
//    five_countdown,
//
//};
extern ui_robot_status_t ui_robot_status;
extern Referee_info_t Referee;
extern void referee_task(void const*argument);

_Noreturn extern void UI_paint_task(void const*argument);
extern uint8_t usart6_buf[REFEREE_BUFFER_SIZE];
extern uint8_t usart1_buf[REFEREE_BUFFER_SIZE];


#endif //DEMO1_REFEREE_H
