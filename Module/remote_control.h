#ifndef _REMOTE_CONTROL_H_
#define _REMOTE_CONTROL_H_

#include "struct_typedef.h"
#include "gimbal_task.h"
#include "referee_task.h"

#define SBUS_RX_BUF_NUM 36u

#define RC_FRAME_LENGTH 18u

typedef struct
{
struct
{
	unsigned short k0;
	unsigned short ch0;
	unsigned short ch1;
	unsigned short ch2;
	unsigned short ch3;
	unsigned short ch4;//×óÉÏ½Ç¹öÂÖ
	unsigned char s1;
	unsigned char s2;
  
}rc;

struct
{
	int16_t x;
	int16_t y;
	int16_t z;
	unsigned char press_l;
	unsigned char press_r;
	
}mouse;

struct
{
	unsigned short v;
}key;

}Rc_ctrl;

//Ò£ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
#define ZeroCh ((uint16_t)50)

/* ----------------------- RC Channel Definition---------------------------- */

#define    RC_CH_VALUE_MIN       ((uint16_t)364 )
#define    RC_CH_VALUE_OFFSET    ((uint16_t)1024)
#define    RC_CH_VALUE_MAX       ((uint16_t)1684)


/* ----------------------- RC Switch Definition----------------------------- */

#define    RC_SW_UP              ((uint16_t)1)
#define    RC_SW_MID             ((uint16_t)3)
#define    RC_SW_DOWN            ((uint16_t)2)
#define switch_is_down(s)       (s == RC_SW_DOWN)
#define switch_is_mid(s)        (s == RC_SW_MID)
#define switch_is_up(s)         (s == RC_SW_UP)


/* ----------------------- PC Key Definition-------------------------------- */

#define    KEY_PRESSED_OFFSET_W        ((uint16_t)0x01<<0)
#define    KEY_PRESSED_OFFSET_S        ((uint16_t)0x01<<1)
#define    KEY_PRESSED_OFFSET_A        ((uint16_t)0x01<<2)
#define    KEY_PRESSED_OFFSET_D        ((uint16_t)0x01<<3)
#define    KEY_PRESSED_OFFSET_SHIFT    ((uint16_t)0x01<<4)
#define    KEY_PRESSED_OFFSET_CTRL     ((uint16_t)0x01<<5)
#define    KEY_PRESSED_OFFSET_Q        ((uint16_t)0x01<<6)
#define    KEY_PRESSED_OFFSET_E        ((uint16_t)0x01<<7)
#define    KEY_PRESSED_OFFSET_R        ((uint16_t)0x01<<8)
#define    KEY_PRESSED_OFFSET_F        ((uint16_t)0x01<<9)
#define    KEY_PRESSED_OFFSET_G        ((uint16_t)0x01<<10)
#define    KEY_PRESSED_OFFSET_Z        ((uint16_t)0x01<<11)
#define    KEY_PRESSED_OFFSET_X        ((uint16_t)0x01<<12)
#define    KEY_PRESSED_OFFSET_C        ((uint16_t)0x01<<13)
#define    KEY_PRESSED_OFFSET_V        ((uint16_t)0x01<<14)
#define    KEY_PRESSED_OFFSET_B        ((uint16_t)0x01<<15)

/* ï¿½ï¿½È¡Ò£ï¿½ï¿½ï¿½ï¿½Ò¡ï¿½ï¿½Æ«ï¿½ï¿½Öµ 
   RLRÊÇÓÒÒ£¸Ë×óÓÒ  LUDÊÇ×óÒ¡¸ËÇ°ºó	*/
#define    RC_CH0_RLR_OFFSET    (ctl.rc.ch0 - RC_CH_VALUE_OFFSET)
#define    RC_CH1_RUD_OFFSET  	(ctl.rc.ch1 - RC_CH_VALUE_OFFSET)
#define    RC_CH2_LLR_OFFSET  	(ctl.rc.ch2 - RC_CH_VALUE_OFFSET)
#define    RC_CH3_LUD_OFFSET  	(ctl.rc.ch3 - RC_CH_VALUE_OFFSET)


/* sw1ÊÇ×ó²¦¸Ë£¬sw2ÊÇÓÒ²¦¸Ë */
#define    IF_RC_SW1_UP      (ctl.rc.s1 == RC_SW_UP)  //×ó²¦¸ËÉÏ
#define    IF_RC_SW1_MID     (ctl.rc.s1 == RC_SW_MID) //×ó²¦¸ËÖÐ
#define    IF_RC_SW1_DOWN    (ctl.rc.s1 == RC_SW_DOWN)	//×ó²¦¸Ëdown
#define    IF_RC_SW2_UP      (ctl.rc.s2 == RC_SW_UP)	//ÓÒ²¦¸ËÉÏ
#define    IF_RC_SW2_MID     (ctl.rc.s2 == RC_SW_MID)	//ÓÒ²¦¸ËÖÐ
#define    IF_RC_SW2_DOWN    (ctl.rc.s2 == RC_SW_DOWN)	//ÓÒ²¦¸Ëdown

#ifdef KEY_MOD_1

/* ï¿½ï¿½È¡ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Æ¶ï¿½ï¿½Ù¶ï¿½ */
#define    MOUSE_X_MOVE_SPEED    (ctl.mouse.x)
#define    MOUSE_Y_MOVE_SPEED    (ctl.mouse.y)
#define    MOUSE_Z_MOVE_SPEED    (ctl.mouse.z)


/* ï¿½ï¿½ï¿½ï¿½ï¿½ê°´ï¿½ï¿½×´Ì¬ 
   ï¿½ï¿½ï¿½ï¿½Îª1ï¿½ï¿½Ã»ï¿½ï¿½ï¿½ï¿½Îª0*/
#define    IF_MOUSE_PRESSED_LEFT    (ctl.mouse.press_l == 1)
#define    IF_MOUSE_PRESSED_RIGHT    (ctl.mouse.press_r == 1)


/* ï¿½ï¿½ï¿½ï¿½ï¿½Ì°ï¿½ï¿½ï¿½×´Ì¬ 
   ï¿½ï¿½ï¿½ï¿½Ó¦ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Â£ï¿½ï¿½ï¿½ï¿½ß¼ï¿½ï¿½ï¿½ï¿½ï¿½Ê½ï¿½ï¿½ÖµÎª1ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Îª0 */
#define    IF_KEY_PRESSED         (  ctl.key.v  )
#define    IF_KEY_PRESSED_W       ( (ctl.key.v & KEY_PRESSED_OFFSET_W)    != 0 )
#define    IF_KEY_PRESSED_S       ( (ctl.key.v & KEY_PRESSED_OFFSET_S)    != 0 )
#define    IF_KEY_PRESSED_A       ( (ctl.key.v & KEY_PRESSED_OFFSET_A)    != 0 )
#define    IF_KEY_PRESSED_D       ( (ctl.key.v & KEY_PRESSED_OFFSET_D)    != 0 )
#define    IF_KEY_PRESSED_Q       ( (ctl.key.v & KEY_PRESSED_OFFSET_Q)    != 0 )
#define    IF_KEY_PRESSED_E       ( (ctl.key.v & KEY_PRESSED_OFFSET_E)    != 0 )
#define    IF_KEY_PRESSED_G       ( (ctl.key.v & KEY_PRESSED_OFFSET_G)    != 0 )
#define    IF_KEY_PRESSED_X       ( (ctl.key.v & KEY_PRESSED_OFFSET_X)    != 0 )
#define    IF_KEY_PRESSED_Z       ( (ctl.key.v & KEY_PRESSED_OFFSET_Z)    != 0 )
#define    IF_KEY_PRESSED_C       ( (ctl.key.v & KEY_PRESSED_OFFSET_C)    != 0 )
#define    IF_KEY_PRESSED_B       ( (ctl.key.v & KEY_PRESSED_OFFSET_B)    != 0 )
#define    IF_KEY_PRESSED_V       ( (ctl.key.v & KEY_PRESSED_OFFSET_V)    != 0 )
#define    IF_KEY_PRESSED_F       ( (ctl.key.v & KEY_PRESSED_OFFSET_F)    != 0 )
#define    IF_KEY_PRESSED_R       ( (ctl.key.v & KEY_PRESSED_OFFSET_R)    != 0 )
#define    IF_KEY_PRESSED_CTRL    ( (ctl.key.v & KEY_PRESSED_OFFSET_CTRL) != 0 )
#define    IF_KEY_PRESSED_SHIFT   ( (ctl.key.v & KEY_PRESSED_OFFSET_SHIFT) != 0 )

#endif

#ifdef KEY_MOD_2

/* ï¿½ï¿½È¡ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Æ¶ï¿½ï¿½Ù¶ï¿½ */
#define    MOUSE_X_MOVE_SPEED    (ext_robot_command.mouse_x)
#define    MOUSE_Y_MOVE_SPEED    (ext_robot_command.mouse_y)
#define    MOUSE_Z_MOVE_SPEED    (ext_robot_command.mouse_z)


/* ï¿½ï¿½ï¿½ï¿½ï¿½ê°´ï¿½ï¿½×´Ì¬ 
   ï¿½ï¿½ï¿½ï¿½Îª1ï¿½ï¿½Ã»ï¿½ï¿½ï¿½ï¿½Îª0*/
#define    IF_MOUSE_PRESSED_LEFT    (ext_robot_command.left_button_down == 1)
#define    IF_MOUSE_PRESSED_RIGHT    (ext_robot_command.right_button_down == 1)


/* ï¿½ï¿½ï¿½ï¿½ï¿½Ì°ï¿½ï¿½ï¿½×´Ì¬ 
   ï¿½ï¿½ï¿½ï¿½Ó¦ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Â£ï¿½ï¿½ï¿½ï¿½ß¼ï¿½ï¿½ï¿½ï¿½ï¿½Ê½ï¿½ï¿½ÖµÎª1ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Îª0 */
#define    IF_KEY_PRESSED         (  ext_robot_command.keyboard_value  )
#define    IF_KEY_PRESSED_W       ( (ext_robot_command.keyboard_value & KEY_PRESSED_OFFSET_W)    != 0 )
#define    IF_KEY_PRESSED_S       ( (ext_robot_command.keyboard_value & KEY_PRESSED_OFFSET_S)    != 0 )
#define    IF_KEY_PRESSED_A       ( (ext_robot_command.keyboard_value & KEY_PRESSED_OFFSET_A)    != 0 )
#define    IF_KEY_PRESSED_D       ( (ext_robot_command.keyboard_value & KEY_PRESSED_OFFSET_D)    != 0 )
#define    IF_KEY_PRESSED_Q       ( (ext_robot_command.keyboard_value & KEY_PRESSED_OFFSET_Q)    != 0 )
#define    IF_KEY_PRESSED_E       ( (ext_robot_command.keyboard_value & KEY_PRESSED_OFFSET_E)    != 0 )
#define    IF_KEY_PRESSED_G       ( (ext_robot_command.keyboard_value & KEY_PRESSED_OFFSET_G)    != 0 )
#define    IF_KEY_PRESSED_X       ( (ext_robot_command.keyboard_value & KEY_PRESSED_OFFSET_X)    != 0 )
#define    IF_KEY_PRESSED_Z       ( (ext_robot_command.keyboard_value & KEY_PRESSED_OFFSET_Z)    != 0 )
#define    IF_KEY_PRESSED_C       ( (ext_robot_command.keyboard_value & KEY_PRESSED_OFFSET_C)    != 0 )
#define    IF_KEY_PRESSED_B       ( (ext_robot_command.keyboard_value & KEY_PRESSED_OFFSET_B)    != 0 )
#define    IF_KEY_PRESSED_V       ( (ext_robot_command.keyboard_value & KEY_PRESSED_OFFSET_V)    != 0 )
#define    IF_KEY_PRESSED_F       ( (ext_robot_command.keyboard_value & KEY_PRESSED_OFFSET_F)    != 0 )
#define    IF_KEY_PRESSED_R       ( (ext_robot_command.keyboard_value & KEY_PRESSED_OFFSET_R)    != 0 )
#define    IF_KEY_PRESSED_CTRL    ( (ext_robot_command.keyboard_value & KEY_PRESSED_OFFSET_CTRL) != 0 )
#define    IF_KEY_PRESSED_SHIFT   ( (ext_robot_command.keyboard_value & KEY_PRESSED_OFFSET_SHIFT) != 0 )

#endif

void remote_control_init(void);

extern Rc_ctrl ctl;

extern uint16_t rc_receive_num;


#endif
