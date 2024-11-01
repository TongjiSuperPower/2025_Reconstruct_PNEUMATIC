#ifndef _REFEREE_TASK_H_
#define _REFEREE_TASK_H_

#include "stdio.h"
#include "struct_typedef.h"
#include "can.h"
#include "main.h"
#include "bsp_usart.h"
#include "fifo.h"
#include "protocol.h"


#define USART_REC_LEN  			200
#define EN_USART1_RX 			1
#define EN_USART6_RX 			1
#define USART6_dma_rx_len 		80
#define USART6_dma_tx_len 		128

#define USART_RX_BUF_LENGHT     512
#define REFEREE_FIFO_BUF_LENGTH 1024

#define REFEREE_FRAME_HEADER_SOF                ((uint8_t)(0xA5))

#define REFEREE_STUDENT_ROBOT_MAX               ((uint16_t)(0x0200))
#define REFEREE_STUDENT_ROBOT_MIN               ((uint16_t)(0x02FF))

#define REFEREE_STUDENT_CLIENT_SOF              ((uint16_t)(0xD180))


//机器人ID
typedef enum
{
    RED_HERO        = 1,
    RED_ENGINEER    = 2,
    RED_STANDARD_1  = 3,
    RED_STANDARD_2  = 4,
    RED_STANDARD_3  = 5,
    RED_AERIAL      = 6,
    RED_SENTRY      = 7,
    BLUE_HERO       = 11,
    BLUE_ENGINEER   = 12,
    BLUE_STANDARD_1 = 13,
    BLUE_STANDARD_2 = 14,
    BLUE_STANDARD_3 = 15,
    BLUE_AERIAL     = 16,
    BLUE_SENTRY     = 17,
} robot_id_t;

//比赛状态数据
typedef __packed struct //cmd_id (0x0001)
{
	uint8_t game_type : 4;//0-3 bit
	uint8_t game_progress : 4;//4-7 bit
	uint16_t stage_remain_time;//offset=1
	uint64_t SyncTimeStamp;//offset=3
} ext_game_status_t;

//比赛结果数据
typedef __packed struct //cmd_id (0x0002)
{
	uint8_t winner;//0 ?? 1 ???? 2 ????
} ext_game_result_t;

//机器人存活数据
typedef __packed struct //cmd_id (0x0003)

{
	uint16_t red_1_robot_HP;//offset=0
	uint16_t red_2_robot_HP;//offset=2
	uint16_t red_3_robot_HP;//offset=4
	uint16_t red_4_robot_HP;//offset=6
	uint16_t red_5_robot_HP;//offset=8
	uint16_t red_7_robot_HP;//offset=10
	uint16_t red_outpost_HP;//offset=12
	uint16_t red_base_HP;//offset=14

	uint16_t blue_1_robot_HP;//offset=16
	uint16_t blue_2_robot_HP;//offset=18
	uint16_t blue_3_robot_HP;//offset=20
	uint16_t blue_4_robot_HP;//offset=22
	uint16_t blue_5_robot_HP;//offset=24
	uint16_t blue_7_robot_HP;//offset=26
	uint16_t blue_outpost_HP;//offset=28
	uint16_t blue_base_HP;//offset=30
} ext_game_robot_HP_t;


//场地时事件数据
typedef __packed struct //cmd_id (0x0101)
{
	uint32_t event_type;
} ext_event_data_t;

//补给站动作标识
typedef __packed struct //cmd_id (0x0102)

{
	uint8_t supply_projectile_id;//offset=0
	uint8_t supply_robot_id;//offset=1
	uint8_t supply_projectile_step;//offset=2
	uint8_t supply_projectile_num;
} ext_supply_projectile_action_t;

//裁判警告信息
typedef __packed struct  //cmd_id (0x0104)
{
	uint8_t level;
	uint8_t offending_robot_id;
	uint8_t count; 
} ext_referee_warning_t;

//飞镖发射口倒计时
typedef __packed struct  //cmd_id(0x0105)
{
	uint8_t dart_remaining_time;
	uint16_t dart_info;
} ext_dart_info_t;

//比赛机器人状态
typedef __packed struct  //cmd_id(0x0201)
{
	uint8_t robot_id;

	uint8_t robot_level;
	uint16_t current_HP;
	uint16_t max_HP;

	uint16_t shooter_barrel_cooling_value;
	uint16_t shooter_barrel_heat_limit;


	uint16_t chassis_power_limit;
	uint8_t power_management_gimbal_output : 1;
	uint8_t power_management_chassis_output : 1;
	uint8_t power_management_shooter_output : 1;
} ext_robot_status_t;

//实时功率热量数据
typedef __packed struct  // 0x0202
{
	uint16_t chassis_voltage;
	uint16_t chassis_current;
	float chassis_power;
	uint16_t buffer_energy;
	uint16_t shooter_17mm_1_barrel_heat;
	uint16_t shooter_17mm_2_barrel_heat;
	uint16_t shooter_42mm_barrel_heat; 
} ext_power_heat_data_t;

//机器人位置
typedef __packed struct  //0x0203
{
	float x;
	float y;
	float angle;
} ext_robot_pos_t;

//机器人增益
typedef __packed struct  //0x0204
{
	uint8_t recovery_buff;
	uint8_t cooling_buff;
	uint8_t defence_buff;
	uint8_t vulnerability_buff;
	uint16_t attack_buff;
}ext_buff_t;

//空中机器人能量状态
typedef __packed struct  //0x0205
{
	uint8_t airforce_status;
	uint8_t time_remain;
} ext_air_support_data_t;

//伤害状态
typedef __packed struct //0x0206
{
	uint8_t armor_id : 4;
	uint8_t HP_deduction_reason : 4;
} ext_hurt_data_t;

//实时射击信息
typedef __packed struct  //0x0207
{
	uint8_t bullet_type;  //1:17mm 2:42mm
	uint8_t shooter_number;   //ID:1:1 17mm; 2:2 17mm; 3:42mm;
	uint8_t launching_frequency;
	float initial_speed;	
} ext_robot_shoot_data_t;

//子弹剩余发射数
typedef __packed struct  //0x0208
{
	uint16_t projectile_allowance_17mm; 
	uint16_t projectile_allowance_42mm; 
	uint16_t remaining_gold_coin; 
} ext_projectile_allowance_t;

//机器人RFID状态
typedef __packed struct  //0x0209
{
	uint32_t rfid_status;
} ext_rfid_status_t;

//飞镖机器人客户端指令数据
typedef __packed struct  //0x020A
{
	uint8_t dart_launch_opening_status;
	uint16_t target_change_time;
	uint16_t operate_launch_cmd_time;
} ext_dart_client_cmd_t;

//地面机器人位置数据
typedef __packed struct // 0x020B
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
} ext_ground_robot_position_t;
//雷达标记进度数据
typedef __packed struct // 0x020C
{
	uint8_t mark_hero_progress; 
	uint8_t mark_engineer_progress; 
	uint8_t mark_standard_3_progress;
	uint8_t mark_standard_4_progress; 
	uint8_t mark_standard_5_progress; 
	uint8_t mark_sentry_progress;
} ext_radar_mark_data_t;
//哨兵自主决策信息同步
typedef __packed struct // 0x020D
{ 
	uint32_t sentry_info; 
} ext_sentry_info_t;
//雷达自主决策信息同步
typedef __packed struct // 0x020E
{ 
	uint8_t radar_info; 
} ext_radar_info_t;


//交互数据接收信息
typedef __packed struct //0x0301
{
	uint16_t data_cmd_id;
	uint16_t sender_ID;
	uint16_t receiver_ID;
}ext_robot_interactive_header_data_t;

//机器人间交互数据，内容 ID:0x0200~0x02FF
typedef __packed struct
{
	ext_robot_interactive_header_data_t header_data;
	uint8_t data[113];
} robot_interactive_data_t;//ID:0x0200~0x02FF

//客户端删除图形，内容 ID:0x0100;
typedef __packed struct
{
	//ID:0x0100
	ext_robot_interactive_header_data_t header_data;
	uint8_t operate_tpye; 
	uint8_t layer;
} ext_interaction_layer_delete_t;

//图形数据
typedef __packed struct
{
	uint8_t graphic_name[3];
	uint32_t operate_tpye : 3; //bit 0 - 2:
	uint32_t graphic_tpye : 3;//Bit 3-5:
	uint32_t layer : 4;// Bit 6 - 9:,0~9
	uint32_t color : 4;//Bit 10-13:
	uint32_t details_a : 9;//Bit 14-22:[0,360];
	uint32_t details_b : 9;//Bit 23-31:[0,360]
	uint32_t width : 10;//Bit 0-9:;
	uint32_t start_x : 11;//Bit 10-20:;
	uint32_t start_y : 11;//Bit 21-31:;
	uint32_t details_c : 10;//Bit 0-9:;
	uint32_t details_d : 11;//Bit 10-20:;
	uint32_t details_e : 11;//Bit 21-31:;
} graphic_data_struct_t;

//客户端绘制一个图形
typedef __packed struct
{
	//ID:0x0101
	ext_robot_interactive_header_data_t header_data;
	graphic_data_struct_t graphic_data_struct;
} ext_interaction_figure_t;

//客户端绘制两个图形
typedef __packed struct
{
	//ID:0x0102
	ext_robot_interactive_header_data_t header_data;
	graphic_data_struct_t graphic_data_struct[2];
} ext_interaction_figure_double_t;

//客户端绘制五个图形
typedef __packed struct
{
	//ID:0x0103
	//ext_robot_interactive_header_data_t header_data;
	graphic_data_struct_t graphic_data_struct[5];
} ext_interaction_figure_five_t;

//客户端绘制七个图形
typedef __packed struct
{
	//ID:0x0104
	//ext_robot_interactive_header_data_t header_data;
	graphic_data_struct_t graphic_data_struct[7];//1~7
} ext_interaction_figure_seven_t;

//客户端绘制字符
typedef __packed struct
{
	//ID:0x0110
	graphic_data_struct_t graphic_data_struct;
	uint8_t data[30];
} ext_client_custom_character_t;
typedef __packed struct
{
	//ID:0x0120
	ext_robot_interactive_header_data_t header_data;//??????????????
	uint32_t sentry_cmd;				   
} ext_sentry_cmd_t;

typedef __packed struct
{
	//ID:0x0121
	ext_robot_interactive_header_data_t header_data;//??????????????
	uint8_t radat_cmd;				   
} ext_radar_cmd_t;
/*----------0x0301结束------------*/

 typedef __packed struct //0x0302
{
	uint8_t data[30]; 
} ext_custom_robot_data_t;

//小地图下发信息标识(0x0303)/*发送频率：触发时发送.*/
typedef __packed struct  //0x0303
{
	float target_position_x;
	float target_position_y;
	float target_position_z;
	uint8_t commd_keyboard;
	uint16_t target_robot_ID;
	uint8_t cmd_source;
} ext_map_command_t;
    
typedef __packed struct{
	//图传遥控器0x0304 发送频率30Hz 
	int16_t mouse_x;
	int16_t mouse_y;
	int16_t mouse_z;
	uint8_t left_button_down;
	uint8_t right_button_down;
	uint16_t keyboard_value;
	uint16_t reserved;
} ext_remote_control_t;

//小地图接收信息标识
typedef __packed struct   //0x0305
{
	uint16_t target_robot_ID; 
	float target_position_x;
	float target_position_y;
} ext_client_map_command_t;

typedef __packed struct // 0x0306
{
	uint16_t key_value; 
	uint16_t x_position:12;
	uint16_t mouse_left:4;
	uint16_t y_position:12;
	uint16_t mouse_right:4; 
	uint16_t reserved;
} ext_custom_client_data_t;

typedef __packed struct // 0x0307
{
	uint8_t intention;
	uint16_t start_position_x;
	uint16_t start_position_y;
	int8_t delta_x[49];
	int8_t delta_y[49];
	uint16_t sender_id;
} ext_map_data_t;

typedef __packed struct // 0x0308
{ 
	uint16_t sender_id;
	uint16_t receiver_id;
	uint8_t user_data[30];
} ext_custom_info_t;

typedef __packed struct {     //????
	uint8_t     sof;                    /*!< Fixed value 0xA5 */
	uint16_t    data_length;            /*!< Length of next data pack */
	uint8_t     seq;                    /*!< Pack sequene id */
	uint8_t     crc8;                   /*!< CRC checksum for frame header pack */
} ext_frame_header_t;

typedef __packed struct
{
	ext_frame_header_t header;
	uint16_t cmd_id;

	uint16_t data_id; /*!< fixed value 0xD180 */
	uint16_t sender_id;
	uint16_t client_id;
	float data[3];
	//    ext_client_custom_data_mask_t   masks;

	uint16_t crc16;
} ext_client_custom_data_t;
typedef __packed struct
{
	ext_frame_header_t header;
	uint16_t cmd_id;

	uint16_t data_id; /*!< range 0x200~0x2FF */
	uint16_t sender_id;
	uint16_t robot_id;
	graphic_data_struct_t graphic_data; /*!< max data length = 13byte */

	uint16_t crc16;
} ext_robot_graphic_data_t;

typedef __packed struct
{
	ext_frame_header_t header;
	uint16_t cmd_id;

	uint16_t data_id; /*!< range 0x200~0x2FF */
	uint16_t sender_id;
	uint16_t robot_id;
	ext_interaction_figure_double_t graphic_data; /*!< max data length = 13byte */

	uint16_t crc16;
} ext_robot_two_graphic_data_t;

typedef __packed struct
{
	ext_frame_header_t header;
	uint16_t cmd_id;

	uint16_t data_id; /*!< range 0x200~0x2FF */
	uint16_t sender_id;
	uint16_t robot_id;
	ext_interaction_figure_five_t graphic_data; /*!< max data length = 13byte */

	uint16_t crc16;
} ext_robot_five_graphic_data_t;

typedef __packed struct
{
	ext_frame_header_t header;
	uint16_t cmd_id;

	uint16_t data_id; /*!< range 0x200~0x2FF */
	uint16_t sender_id;
	uint16_t robot_id;
	ext_interaction_figure_seven_t graphic_data; /*!< max data length = 13byte */

	uint16_t crc16;
} ext_robot_seven_graphic_data_t;

typedef __packed struct
{
	ext_frame_header_t header;
	uint16_t cmd_id;

	uint16_t data_id; /*!< range 0x200~0x2FF */
	uint16_t sender_id;
	uint16_t robot_id;
	ext_client_custom_character_t character_data; /*!< max data length = 43byte */

	uint16_t crc16;
} ext_robot_character_data_t;


typedef enum {
	game_status = 0x0001,     /*!< frequency = 1Hz */
	game_result = 0x0002,     /*!< send at game ending */
	game_robot_HP = 0x0003,     /*!< frequency = 1Hz */

	event_data = 0x0101,     /*!< send at event changing */
	supply_projectile_action = 0x0102,     /*!< send at action */

	referee_warning = 0x0104,
	dart_info = 0x0105,
	robot_status = 0x0201,     /*!< frequency = 10Hz */
	power_heat_data = 0x0202,     /*!< frequency = 50Hz */
	robot_pos = 0x0203,     /*!< frequency = 10Hz */
	buff = 0x0204,     /*!< send at changing */
	air_support_data = 0x0205,     /*!< frequency = 10Hz, only for aerial robot */
	hurt_data = 0x0206,     /*!< send at hurting */
	robot_shoot_Data = 0x0207,     /*!< send at shooting */
	projectile_allowance = 0x0208,
	rfid_status = 0x0209,
	dart_client_cmd = 0x020A,
	ground_robot_position = 0x020B,
	radar_mark_data = 0X020C,
	sentry_info = 0x020D,
	radar_info = 0x020E,
	robot_interactive_header = 0x0301, /*!< send by user, max frequency = 10Hz */
	custom_robot_data = 0x0302,
	map_command = 0x0303,
	remote_control = 0x0304
} ext_cmd_id_t;

typedef enum {
	robotid_red_hero = 1,
	robotid_red_engineer = 2,
	robotid_red_infantry_1 = 3,
	robotid_red_infantry_2 = 4,
	robotid_red_infantry_3 = 5,
	robotid_red_aerial = 6,
	robotid_red_sentry = 7,
	robotid_red_radar = 9,         
	robotid_blue_hero = 101,
	robotid_blue_engineer = 102,
	robotid_blue_infantry_1 = 103,
	robotid_blue_infantry_2 = 104,
	robotid_blue_infantry_3 = 105,
	robotid_blue_aerial = 106,
	robotid_blue_sentry = 107,
	robotid_blue_radar = 109,

	clientid_red_hero = 0x0101,
	clientid_red_engineer = 0x0102,
	clientid_red_infantry_1 = 0x0103,
	clientid_red_infantry_2 = 0x0104,
	clientid_red_infantry_3 = 0x0105,
	clientid_red_aerial = 0x0106,
	clientid_blue_hero = 0x0165,
	clientid_blue_engineer = 0x0166,
	clientid_blue_infantry_1 = 0x0167,
	clientid_blue_infantry_2 = 0x0168,
	clientid_blue_infantry_3 = 0x0169,
	clientid_blue_aerial = 0x016A,
} ext_id_t;



extern ext_power_heat_data_t ext_power_heat_data;
extern ext_robot_status_t ext_robot_status;
extern ext_robot_shoot_data_t ext_robot_shoot_data;
extern ext_remote_control_t ext_remote_control;

//读取底盘功率
void get_chassis_power_and_buffer(fp32 *power, fp32 *buffer);

//读取枪口射速
void get_shoot_speed(float *speed);

#endif
