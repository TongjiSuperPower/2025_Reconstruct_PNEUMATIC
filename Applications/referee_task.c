#include "referee_task.h"
#include "cmsis_os.h"
#include "stdio.h"
#include "struct_typedef.h"
#include "can.h"
#include "main.h"
#include "bsp_usart.h"
#include "protocol.h"
#include "usart.h"
#include "shoot_task.h"
#include "can_task.h"
#include "string.h"
#include "fifo.h"
#include "CRC8_CRC16.h"
#include "bsp_usart.h"
#include "stdbool.h"
#include "gimbal_task.h"
#include "chassis_task.h"
#include "supercap_task.h"


void init_referee_struct_data(void);
void referee_unpack_fifo_data(void);
void referee_data_solve(uint8_t *frame);
void referee_data_solve(uint8_t *frame);
void send_multi_static_graphic(void);
void send_multi_dynamic_graphic(void);
void referee_send_multi_graphic(ext_id_t target_id, ext_interaction_figure_seven_t* graphic_draw);
void referee_send_five_graphic(ext_id_t target_id, ext_interaction_figure_five_t* graphic_draw);
void send_double_text(void);
void send_capvol_graphic(int capvols);
void referee_send_client_graphic(ext_id_t target_id, graphic_data_struct_t* graphic_draw);
void send_rub_graphic(char graphname[3], int x, int y, int color, int type);
void send_string(char* str, char* name, int x, int y, int upd, int colour);
void send_line_graphic(char name[3], uint32_t start_x, uint32_t start_y, uint32_t end_x, uint32_t end_y);
void send_rectangle_graphic(char name[3], uint32_t x, uint32_t y, uint32_t end_x, uint32_t end_y);
void send_circle_graphic(char name[3], uint32_t x, uint32_t y, uint32_t radius);
void send_spinning_graphic(char name[3], int x, int y, int color, int type);
void send_shoot_state(void);
void send_autoaim_state(void);
void get_robot_id(void);
bool capdraw = true;
bool auto_state = true;
bool shoot = true;

ext_id_t MY_CLIENT_ID = clientid_blue_infantry_1;
int MY_ROBOT_ID = robotid_blue_infantry_1;

uint16_t cmd_id;  // ���ݰ�ID

ext_game_status_t                          ext_game_status;// ����״̬���ݣ�0x0001��
ext_game_result_t                          ext_game_result;//�����������?(0x0002)
ext_game_robot_HP_t                 	   ext_game_robot_HP;//������Ѫ�����ݣ�0x0003��


ext_event_data_t                           ext_event_data;//�����¼����ݣ�0x0101��
ext_supply_projectile_action_t             ext_supply_projectile_action;//����վ������ʶ���ݣ�0x0102��

ext_referee_warning_t                      ext_referee_warning;//���о�������(0x0104)
ext_dart_info_t                  		   ext_dart_info;//���ڷ����������?(0x0105)


ext_robot_status_t                    	   ext_robot_status;//������������ϵ����(0x0201)
ext_power_heat_data_t                      ext_power_heat_data;//ʵʱ���̹��ʺ�ǹ���������ݣ�0x0202��
ext_robot_pos_t                       	   ext_robot_pos;//������λ�����ݣ�0x0203��
ext_buff_t                                 ext_buff;//�������������ݣ�0x0204��
ext_air_support_data_t                     ext_air_support_data;//����֧Ԯʱ�����ݣ�0x0205��
ext_hurt_data_t                            ext_hurt_data;//�˺�״̬���ݣ�0x0206��
ext_robot_shoot_data_t                     ext_robot_shoot_data;//ʵʱ������ݣ�?0x0207��
ext_projectile_allowance_t                 ext_projectile_allowance;//����������(0x0208)
ext_rfid_status_t                          ext_rfid_status;//������ RFID ģ��״̬(0x0209)
ext_dart_client_cmd_t                      ext_dart_client_cmd;//����ѡ�ֶ�ָ������(0x020A)
ext_ground_robot_position_t                ext_ground_robot_position;//���������λ������?(0x020B)
ext_radar_mark_data_t                      ext_radar_mark_data;//�״��ǽ�������(0x020C)
ext_sentry_info_t                          ext_sentry_info;//�ڱ�����������Ϣͬ��(0x020D)
ext_radar_info_t                      	   ext_radar_info;//�״�����������Ϣͬ��(0x020E)

//-------------0x0301���ֿ�ʼ-------------------
ext_robot_interactive_header_data_t        ext_robot_interactive_header_data;//�����˽������ݣ�0x0301��
robot_interactive_data_t                   robot_interactive_data;//�����˼佻�����ݣ����� ID:0x0200~0x02FF
ext_interaction_layer_delete_t             ext_interaction_layer_delete;//�ͻ���ɾ��ͼ�Σ����� ID:0x0100;
graphic_data_struct_t                      graphic_data_struct;//ͼ������
ext_interaction_figure_t         		   ext_interaction_figure;//�ͻ��˻���һ��ͼ��
ext_interaction_figure_double_t            ext_interaction_figure_double;//�ͻ��˻�������ͼ��
ext_interaction_figure_five_t              ext_interaction_figure_five;//�ͻ��˻������ͼ��?
ext_interaction_figure_seven_t             ext_interaction_figure_seven;//�ͻ��˻����߸�ͼ��
ext_client_custom_character_t              ext_client_custom_character;//�ͻ��˻����ַ�
//-------------0x0301���ֽ���-------------------


ext_custom_robot_data_t                    ext_custom_robot_data;//�Զ��������������˽�������(0x0302)
ext_map_command_t                          ext_map_command; //ѡ�ֶ�С��ͼ��������(0x0303)/*����Ƶ�ʣ�����ʱ����.*/
ext_remote_control_t                       ext_remote_control;//����ң������(0x0304)
ext_client_map_command_t                   ext_client_map_command; //ѡ�ֶ�С��ͼ�����״�����(0x0305)
ext_custom_client_data_t                   ext_custom_client_data; //�Զ����������ѡ�ֶ˽�������?(0x0306)
ext_map_data_t                             ext_map_data; //ѡ�ֶ�С��ͼ�����ڱ�����(0x0307)
ext_custom_info_t                          ext_custom_info; //ѡ�ֶ�С��ͼ���ջ���������(0x0308)

frame_header_struct_t ext_referee_receive_header;
frame_header_struct_t ext_referee_send_header;

fifo_s_t referee_fifo;
uint8_t referee_fifo_buf[REFEREE_FIFO_BUF_LENGTH];
unpack_data_t referee_unpack_obj;

uint8_t usart6_buf[2][USART_RX_BUF_LENGHT];

uint8_t USART6_dma[80];		//DMA��������
uint8_t Personal_Data[128];	//DMA��������


void Referee_Task(void const * argument){
    
    init_referee_struct_data();//Ϊ����ʢװ���ݵĸ����ṹ�����ռ䡣
    fifo_s_init(&referee_fifo, referee_fifo_buf, REFEREE_FIFO_BUF_LENGTH);//��ʼ�����ڽ�������ϵͳ��Ϣ�Ķ��л���
    usart6_init(usart6_buf[0], usart6_buf[1], USART_RX_BUF_LENGHT);//ͨ��6�Ŵ����շ����Ե�Դ����ģ������ݣ���������ģ���?��WIFI��������?�š�
	//USART6ͨ�������жϽ������ݡ�
	
	uint16_t UI_PushUp_Counter = 0;
	uint16_t UI_static_Counter = 0;
	/* ����ϵͳ��ʼ�� */
	vTaskDelay(300);

	while(1){
		referee_unpack_fifo_data();//��������

	    vTaskDelay(10);
		get_robot_id();

		UI_PushUp_Counter++;
				
		if(UI_PushUp_Counter > 10){
			UI_PushUp_Counter = 0;
		}
		
		UI_static_Counter++;
		
		if(UI_static_Counter >= 100){
			UI_static_Counter = 0;
		}
		
		if(IF_KEY_PRESSED_R && (capdraw != true || auto_state != true || shoot != true)){
		    //send_multi_graphic();
		    capdraw = true;
		    auto_state = true;
			shoot = true;
		    continue;
		}

        if(UI_static_Counter == 57 && IF_KEY_PRESSED_R){
		    send_multi_static_graphic();//�߸�
			continue;
	    }

		//��������
		if(UI_PushUp_Counter == 7){
			//REST ENERGY
            send_capvol_graphic(cap_data.Capacity);
			continue;
		}

		//���ද̬����UI
		if(UI_PushUp_Counter == 1){
			send_multi_dynamic_graphic();//�߸�
			continue;
		}
	}
}



void init_referee_struct_data(void)
{
    memset(&ext_referee_send_header, 0, sizeof(frame_header_struct_t));

    memset(&ext_game_status, 0, sizeof(ext_game_status_t));
    memset(&ext_game_result, 0, sizeof(ext_game_result_t));
    memset(&ext_game_robot_HP, 0, sizeof(ext_game_robot_HP_t));

    memset(&ext_event_data, 0, sizeof(ext_event_data_t));
    memset(&ext_supply_projectile_action, 0, sizeof(ext_supply_projectile_action_t));
    memset(&ext_referee_warning, 0, sizeof(ext_referee_warning_t));
	memset(&ext_dart_info,             0, sizeof(ext_dart_info));

    memset(&ext_robot_status, 0, sizeof(ext_robot_status_t));
    memset(&ext_power_heat_data, 0, sizeof(ext_power_heat_data_t));
    memset(&ext_robot_pos, 0, sizeof(ext_robot_pos_t));
    memset(&ext_buff, 0, sizeof(ext_buff));
    memset(&ext_air_support_data, 0, sizeof(air_support_data));
    memset(&ext_hurt_data, 0, sizeof(ext_hurt_data_t));
    memset(&ext_robot_shoot_data, 0, sizeof(ext_robot_shoot_data_t));
    memset(&ext_projectile_allowance, 0, sizeof(ext_projectile_allowance_t));
	memset(&ext_rfid_status,                     0, sizeof(ext_rfid_status));
	memset(&ext_dart_client_cmd,                 0, sizeof(ext_dart_client_cmd));	
	memset(&ext_ground_robot_position,         0, sizeof(ext_ground_robot_position));
	memset(&ext_radar_mark_data,                 0, sizeof(ext_radar_mark_data));
	memset(&ext_sentry_info,                 0, sizeof(ext_sentry_info));
	memset(&ext_radar_info,                 0, sizeof(ext_radar_info));
				

    memset(&ext_robot_interactive_header_data, 0, sizeof(ext_robot_interactive_header_data_t));
	memset(&robot_interactive_data, 0, sizeof(robot_interactive_data));
	memset(&ext_custom_robot_data, 0, sizeof(ext_custom_robot_data));
	memset(&ext_map_command, 0, sizeof(ext_map_command));
	memset(&ext_remote_control, 0, sizeof(ext_remote_control));
}


void referee_unpack_fifo_data(void)
{
  uint8_t byte = 0;
  uint8_t sof = HEADER_SOF;
  unpack_data_t *p_obj = &referee_unpack_obj;

  while ( fifo_s_used(&referee_fifo) )
  {
    byte = fifo_s_get(&referee_fifo);
    switch(p_obj->unpack_step)
    {
      case STEP_HEADER_SOF:
      {
        if(byte == sof)
        {
          p_obj->unpack_step = STEP_LENGTH_LOW;
          p_obj->protocol_packet[p_obj->index++] = byte;
        }
        else
        {
          p_obj->index = 0;
        }
      }break;
      
      case STEP_LENGTH_LOW:
      {
        p_obj->data_len = byte;
        p_obj->protocol_packet[p_obj->index++] = byte;
        p_obj->unpack_step = STEP_LENGTH_HIGH;
      }break;
      
      case STEP_LENGTH_HIGH:
      {
        p_obj->data_len |= (byte << 8);
        p_obj->protocol_packet[p_obj->index++] = byte;

        if(p_obj->data_len < (REF_PROTOCOL_FRAME_MAX_SIZE - REF_HEADER_CRC_CMDID_LEN))
        {
          p_obj->unpack_step = STEP_FRAME_SEQ;
        }
        else
        {
          p_obj->unpack_step = STEP_HEADER_SOF;
          p_obj->index = 0;
        }
      }break;
      case STEP_FRAME_SEQ:
      {
        p_obj->protocol_packet[p_obj->index++] = byte;
        p_obj->unpack_step = STEP_HEADER_CRC8;
      }break;

      case STEP_HEADER_CRC8:
      {
        p_obj->protocol_packet[p_obj->index++] = byte;

        if (p_obj->index == REF_PROTOCOL_HEADER_SIZE)
        {
          if ( verify_CRC8_check_sum(p_obj->protocol_packet, REF_PROTOCOL_HEADER_SIZE) )
          {
            p_obj->unpack_step = STEP_DATA_CRC16;
          }
          else
          {
            p_obj->unpack_step = STEP_HEADER_SOF;
            p_obj->index = 0;
          }
        }
      }break;  
      
      case STEP_DATA_CRC16:
      {
        if (p_obj->index < (REF_HEADER_CRC_CMDID_LEN + p_obj->data_len))
        {
           p_obj->protocol_packet[p_obj->index++] = byte;  
        }
        if (p_obj->index >= (REF_HEADER_CRC_CMDID_LEN + p_obj->data_len))
        {
          p_obj->unpack_step = STEP_HEADER_SOF;
          p_obj->index = 0;

          if ( verify_CRC16_check_sum(p_obj->protocol_packet, REF_HEADER_CRC_CMDID_LEN + p_obj->data_len) )
          {
            referee_data_solve(p_obj->protocol_packet);
          }
        }
      }break;

      default:
      {
        p_obj->unpack_step = STEP_HEADER_SOF;
        p_obj->index = 0;
      }break;
    }
  }
}

void referee_data_solve(uint8_t *frame)
{
    uint16_t cmd_id = 0;

    uint8_t index = 0;

    memcpy(&ext_referee_receive_header, frame, sizeof(frame_header_struct_t));

    index += sizeof(frame_header_struct_t);

    memcpy(&cmd_id, frame + index, sizeof(uint16_t));
    index += sizeof(uint16_t);

    switch (cmd_id)
    {
        case GAME_STATE_CMD_ID:
        {
            memcpy(&ext_game_status, frame + index, sizeof(ext_game_status_t));
        }
        break;
        case GAME_RESULT_CMD_ID:
        {
            memcpy(&ext_game_result, frame + index, sizeof(ext_game_result));
        }
        break;
        case GAME_ROBOT_HP_CMD_ID:
        {
            memcpy(&ext_game_robot_HP, frame + index, sizeof(ext_game_robot_HP_t));
        }
        break;

        case EVENTS_DATA_CMD_ID:
        {
            memcpy(&ext_event_data, frame + index, sizeof(ext_event_data_t));
        }
        break;
        case SUPPLY_PROJECTILE_ACTION_CMD_ID:
        {
            memcpy(&ext_supply_projectile_action, frame + index, sizeof(ext_supply_projectile_action_t));
        }
        break;

        case REFEREE_WARNING_CMD_ID:
        {
            memcpy(&ext_referee_warning, frame + index, sizeof(ext_referee_warning_t));
        }
        break;

        case ROBOT_STATUS_CMD_ID:
        {
            memcpy(&ext_robot_status, frame + index, sizeof(ext_robot_status_t));
        }
        break;
        case POWER_HEAT_DATA_CMD_ID:
        {
            memcpy(&ext_power_heat_data, frame + index, sizeof(ext_power_heat_data_t));
        }
        break;
        case ROBOT_POS_CMD_ID:
        {
            memcpy(&ext_robot_pos, frame + index, sizeof(ext_robot_pos_t));
        }
        break;
        case BUFF_CMD_ID:
        {
            memcpy(&ext_buff, frame + index, sizeof(ext_buff_t));
        }
        break;
        case AIR_SUPPORT_DATA_CMD_ID:
        {
            memcpy(&ext_air_support_data, frame + index, sizeof(ext_air_support_data_t));
        }
        break;
        case HURT_DATA_CMD_ID:
        {
            memcpy(&ext_hurt_data, frame + index, sizeof(ext_hurt_data_t));
        }
        break;
        case ROBOT_SHOOT_DATA_CMD_ID:
        {
            memcpy(&ext_robot_shoot_data, frame + index, sizeof(ext_robot_shoot_data_t));
        }
        break;
        case BULLET_REMAINING_CMD_ID:
        {
            memcpy(&ext_projectile_allowance, frame + index, sizeof(ext_projectile_allowance_t));
        }
        break;
        case RFID_STATUS_CMD_ID:
        {
            memcpy(&ext_rfid_status, frame + index, sizeof(ext_rfid_status_t));
        }
        break;
				case GROUND_ROBOT_POSITION_CMD_ID: // 0x020B
        {
            memcpy(&ext_ground_robot_position, frame + index, sizeof(ext_ground_robot_position_t));
        }
        break;
				case RADAR_MARK_DATA_CMD_ID: // 0x020C
        {
            memcpy(&ext_radar_mark_data, frame + index, sizeof(ext_radar_mark_data_t));
        }
        break;
        case SENTRY_INFO_CMD_ID: // 0x020D
        {
            memcpy(&ext_sentry_info, frame + index, sizeof(ext_sentry_info_t));
        }
        break;
				case RADAR_INFO_CMD_ID: // 0x020E
        {
            memcpy(&ext_radar_info, frame + index, sizeof(ext_radar_info_t));
        }
        break;
				case ROBOT_INTERACTIVE_DATA_CMD_ID: // 0x0301
        {
            memcpy(&ext_robot_interactive_header_data, frame + index, sizeof(ext_robot_interactive_header_data_t));
        }
				case CUSTOM_ROBOT_DATA_CMD_ID: // 0x0302
        {
            memcpy(&ext_custom_robot_data , frame + index, sizeof(ext_custom_robot_data_t));
        }
        break;
				case MAP_COMMAND_CMD_ID: // 0x0303
        {
            memcpy(&ext_map_command, frame + index, sizeof(ext_map_command_t));
        }
        break;
				case ROBOT_COMMAND_CMD_ID: // 0x0304
        {
            memcpy(&ext_remote_control, frame + index, sizeof(ext_remote_control_t));
        }
        break;
        default:
        {
            break;
        }
    }
}

void get_robot_id()
{
		switch(ext_robot_status.robot_id)
		{
			case robotid_red_hero:{
					MY_CLIENT_ID = clientid_red_hero;
					MY_ROBOT_ID = robotid_red_hero;	
				  break;
			}
			case robotid_red_engineer:{
					MY_CLIENT_ID = clientid_red_engineer;
					MY_ROBOT_ID = robotid_red_engineer;	
				  break;
			}
			case robotid_red_infantry_1:{
					MY_CLIENT_ID = clientid_red_infantry_1;
					MY_ROBOT_ID = robotid_red_infantry_1;	
				  break;
			}
			case robotid_red_infantry_2:{
					MY_CLIENT_ID = clientid_red_infantry_2;
					MY_ROBOT_ID = robotid_red_infantry_2;	
				  break;					
			}
			case robotid_red_infantry_3:{
					MY_CLIENT_ID = clientid_red_infantry_3;
					MY_ROBOT_ID = robotid_red_infantry_3;
				  break;					
			}
			case robotid_red_aerial:{
					MY_CLIENT_ID = clientid_red_aerial;
					MY_ROBOT_ID = robotid_red_aerial;
				  break;					
			} 
		
			case robotid_blue_hero:{
					MY_CLIENT_ID = clientid_blue_hero;
					MY_ROBOT_ID = robotid_blue_hero;	
				  break;
			}
				case robotid_blue_engineer:{
					MY_CLIENT_ID = clientid_blue_engineer;
					MY_ROBOT_ID = robotid_blue_engineer;	
				  break;
			}
			case robotid_blue_infantry_1:{
					MY_CLIENT_ID = clientid_blue_infantry_1;
					MY_ROBOT_ID = robotid_blue_infantry_1;	
				  break;
			}
			case robotid_blue_infantry_2:{
					MY_CLIENT_ID = clientid_blue_infantry_2;
					MY_ROBOT_ID = robotid_blue_infantry_2;
				  break;					
			}
			case robotid_blue_infantry_3:{
					MY_CLIENT_ID = clientid_blue_infantry_3;
					MY_ROBOT_ID = robotid_blue_infantry_3;
				  break;					
			}
			case robotid_blue_aerial:{
					MY_CLIENT_ID = clientid_blue_aerial;
					MY_ROBOT_ID = robotid_blue_aerial;
				  break;					
			} 
 
		}
}

void send_multi_static_graphic(void)//��ͻ��˷��������ͼ�ε����ݡ�ÿ��ͼ�ΰ�������յ�?�������͡���ɫ���߿������ԡ�
{

	ext_interaction_figure_seven_t graphic_draw;
	switch(ext_robot_status.robot_id){
		case robotid_red_hero:{
			MY_CLIENT_ID = clientid_red_hero;
			MY_ROBOT_ID = robotid_red_hero;	
			break;
		}
		case robotid_red_infantry_1:{
			MY_CLIENT_ID = clientid_red_infantry_1;
			MY_ROBOT_ID = robotid_red_infantry_1;	
			break;
		}
		case robotid_red_infantry_2:{
			MY_CLIENT_ID = clientid_red_infantry_2;
			MY_ROBOT_ID = robotid_red_infantry_2;	
			break;					
		}
		case robotid_red_infantry_3:{
			MY_CLIENT_ID = clientid_red_infantry_3;
			MY_ROBOT_ID = robotid_red_infantry_3;
			break;					
		}
	
		case robotid_blue_hero:{
			MY_CLIENT_ID = clientid_blue_hero;
			MY_ROBOT_ID = robotid_blue_hero;	
			break;
		}
		case robotid_blue_infantry_1:{
			MY_CLIENT_ID = clientid_blue_infantry_1;
			MY_ROBOT_ID = robotid_blue_infantry_1;	
			break;
		}
		case robotid_blue_infantry_2:{
			MY_CLIENT_ID = clientid_blue_infantry_2;
			MY_ROBOT_ID = robotid_blue_infantry_2;
			break;					
		}
		case robotid_blue_infantry_3:{
			MY_CLIENT_ID = clientid_blue_infantry_3;
			MY_ROBOT_ID = robotid_blue_infantry_3;
			break;					
		}
	}
	
	//׼��
	//(952,509)
	//����
	//����
	graphic_draw.graphic_data_struct[0].graphic_name[0] = '2';
	graphic_draw.graphic_data_struct[0].graphic_name[1] = '4';
	graphic_draw.graphic_data_struct[0].graphic_name[2] = '1';

	graphic_draw.graphic_data_struct[0].operate_tpye = 1;

	graphic_draw.graphic_data_struct[0].graphic_tpye = 0;
	graphic_draw.graphic_data_struct[0].layer = 2;

	graphic_draw.graphic_data_struct[0].color = 8;
	graphic_draw.graphic_data_struct[0].width = 2;

    graphic_draw.graphic_data_struct[0].start_x = 936;
	graphic_draw.graphic_data_struct[0].start_y = 537;
	graphic_draw.graphic_data_struct[0].details_d = 936;
	graphic_draw.graphic_data_struct[0].details_e = 497;
	//����
    graphic_draw.graphic_data_struct[1].graphic_name[0] = '2';
	graphic_draw.graphic_data_struct[1].graphic_name[1] = '4';
	graphic_draw.graphic_data_struct[1].graphic_name[2] = '2';

	graphic_draw.graphic_data_struct[1].operate_tpye = 1;

	graphic_draw.graphic_data_struct[1].graphic_tpye = 0;
	graphic_draw.graphic_data_struct[1].layer = 2;

	graphic_draw.graphic_data_struct[1].color = 8;
	graphic_draw.graphic_data_struct[1].width = 2;

    graphic_draw.graphic_data_struct[1].start_x = 916;
	graphic_draw.graphic_data_struct[1].start_y = 517;
	graphic_draw.graphic_data_struct[1].details_d = 956;
	graphic_draw.graphic_data_struct[1].details_e = 517;


	//Բ��
	graphic_draw.graphic_data_struct[2].graphic_name[0] = '2';
	graphic_draw.graphic_data_struct[2].graphic_name[1] = '3';
	graphic_draw.graphic_data_struct[2].graphic_name[2] = '1';

	graphic_draw.graphic_data_struct[2].operate_tpye = 1;

	graphic_draw.graphic_data_struct[2].graphic_tpye = 2;
	graphic_draw.graphic_data_struct[2].layer = 0;

	graphic_draw.graphic_data_struct[2].color = 8;
	graphic_draw.graphic_data_struct[2].width = 2;

    graphic_draw.graphic_data_struct[2].start_x = 1800;
	graphic_draw.graphic_data_struct[2].start_y = 600;
	graphic_draw.graphic_data_struct[2].details_c = 33;

    //Բ��
	graphic_draw.graphic_data_struct[3].graphic_name[0] = '2';
	graphic_draw.graphic_data_struct[3].graphic_name[1] = '3';
	graphic_draw.graphic_data_struct[3].graphic_name[2] = '2';

	graphic_draw.graphic_data_struct[3].operate_tpye = 1;

	graphic_draw.graphic_data_struct[3].graphic_tpye = 2;
	graphic_draw.graphic_data_struct[3].layer = 0;

	graphic_draw.graphic_data_struct[3].color = 8;
	graphic_draw.graphic_data_struct[3].width = 2;

    graphic_draw.graphic_data_struct[3].start_x = 1800;
	graphic_draw.graphic_data_struct[3].start_y = 800;
	graphic_draw.graphic_data_struct[3].details_c = 33;

	//���ݿ�
	graphic_draw.graphic_data_struct[4].graphic_name[0] = '2';
	graphic_draw.graphic_data_struct[4].graphic_name[1] = '3';
	graphic_draw.graphic_data_struct[4].graphic_name[2] = '3';

	graphic_draw.graphic_data_struct[4].operate_tpye = 1;

	graphic_draw.graphic_data_struct[4].graphic_tpye = 1;
	graphic_draw.graphic_data_struct[4].layer = 0;

	graphic_draw.graphic_data_struct[4].color = 8;
	graphic_draw.graphic_data_struct[4].width = 1;

    graphic_draw.graphic_data_struct[4].start_x = 760;
	graphic_draw.graphic_data_struct[4].start_y = 80;
	graphic_draw.graphic_data_struct[4].details_d = 1172;
	graphic_draw.graphic_data_struct[4].details_e = 60;

    //ʾ������
	graphic_draw.graphic_data_struct[5].graphic_name[0] = '2';
	graphic_draw.graphic_data_struct[5].graphic_name[1] = '8';
	graphic_draw.graphic_data_struct[5].graphic_name[2] = '1';

	graphic_draw.graphic_data_struct[5].operate_tpye = 1;

	graphic_draw.graphic_data_struct[5].graphic_tpye = 0;
	graphic_draw.graphic_data_struct[5].layer = 0;

	graphic_draw.graphic_data_struct[5].color = 2;
	graphic_draw.graphic_data_struct[5].width = 2;

    graphic_draw.graphic_data_struct[5].start_x = 1300;
	graphic_draw.graphic_data_struct[5].start_y = 0;
	graphic_draw.graphic_data_struct[5].details_d = 1142;
	graphic_draw.graphic_data_struct[5].details_e = 255;

    //ʾ������
	graphic_draw.graphic_data_struct[6].graphic_name[0] = '2';
	graphic_draw.graphic_data_struct[6].graphic_name[1] = '8';
	graphic_draw.graphic_data_struct[6].graphic_name[2] = '2';

	graphic_draw.graphic_data_struct[6].operate_tpye = 1;

	graphic_draw.graphic_data_struct[6].graphic_tpye = 0;
	graphic_draw.graphic_data_struct[6].layer = 0;

	graphic_draw.graphic_data_struct[6].color = 2;
	graphic_draw.graphic_data_struct[6].width = 2;

    graphic_draw.graphic_data_struct[6].start_x = 620;
	graphic_draw.graphic_data_struct[6].start_y = 0;
	graphic_draw.graphic_data_struct[6].details_d = 778;
	graphic_draw.graphic_data_struct[6].details_e = 255;

	referee_send_multi_graphic(MY_CLIENT_ID, &graphic_draw);
}

void send_multi_dynamic_graphic(void)//��ͻ��˷����߸����?ͼ�ε����ݡ�
{

	ext_interaction_figure_seven_t graphic_draw;
	switch(ext_robot_status.robot_id){
		case robotid_red_hero:{
			MY_CLIENT_ID = clientid_red_hero;
			MY_ROBOT_ID = robotid_red_hero;	
			break;
		}
		case robotid_red_infantry_1:{
			MY_CLIENT_ID = clientid_red_infantry_1;
			MY_ROBOT_ID = robotid_red_infantry_1;	
			break;
		}
		case robotid_red_infantry_2:{
			MY_CLIENT_ID = clientid_red_infantry_2;
			MY_ROBOT_ID = robotid_red_infantry_2;	
			break;					
		}
		case robotid_red_infantry_3:{
			MY_CLIENT_ID = clientid_red_infantry_3;
			MY_ROBOT_ID = robotid_red_infantry_3;
			break;					
		}
			
		case robotid_blue_hero:{
			MY_CLIENT_ID = clientid_blue_hero;
			MY_ROBOT_ID = robotid_blue_hero;	
			break;
		}
		case robotid_blue_infantry_1:{
			MY_CLIENT_ID = clientid_blue_infantry_1;
			MY_ROBOT_ID = robotid_blue_infantry_1;	
			break;
		}
		case robotid_blue_infantry_2:{
			MY_CLIENT_ID = clientid_blue_infantry_2;
			MY_ROBOT_ID = robotid_blue_infantry_2;
			break;					
		}
		case robotid_blue_infantry_3:{
			MY_CLIENT_ID = clientid_blue_infantry_3;
			MY_ROBOT_ID = robotid_blue_infantry_3;
			break;					
		}
	}

    //�������?
	if(1 == 0){
		graphic_draw.graphic_data_struct[0].operate_tpye = 1;
	}
	else{
		graphic_draw.graphic_data_struct[0].operate_tpye = 3;
	}
	graphic_draw.graphic_data_struct[0].graphic_name[0] = '0';
	graphic_draw.graphic_data_struct[0].graphic_name[1] = '1';
	graphic_draw.graphic_data_struct[0].graphic_name[2] = '3';
	
	graphic_draw.graphic_data_struct[0].color = 4;
	graphic_draw.graphic_data_struct[0].graphic_tpye = 2;
	
	graphic_draw.graphic_data_struct[0].layer = 0;
	graphic_draw.graphic_data_struct[0].width = 20; //��������
	
	graphic_draw.graphic_data_struct[0].start_x = 1800;
	graphic_draw.graphic_data_struct[0].start_y = 500;
	
	graphic_draw.graphic_data_struct[0].details_c = 20;
        
	//�������� ��������ӳ���?
	graphic_draw.graphic_data_struct[1].graphic_name[0] = '4';
	graphic_draw.graphic_data_struct[1].graphic_name[1] = '0';
	graphic_draw.graphic_data_struct[1].graphic_name[2] = '9';
	if(shoot){
	    graphic_draw.graphic_data_struct[1].operate_tpye = 1;
	    shoot = false;
	}
	else
	{
	    graphic_draw.graphic_data_struct[1].operate_tpye = 2;
	}
	if(gimbal_data.gimbal_behaviour == GIMBAL_AUTO){
		if(autoaim_measure.vision_state == 1){
		    graphic_draw.graphic_data_struct[1].color = 2;
		}
		else{
		    graphic_draw.graphic_data_struct[1].color = 3;
		}
	}
	else{
		graphic_draw.graphic_data_struct[1].color = 8;
	}

	graphic_draw.graphic_data_struct[1].graphic_tpye = 1;
	graphic_draw.graphic_data_struct[1].layer = 0;

	graphic_draw.graphic_data_struct[1].width = 1;

    graphic_draw.graphic_data_struct[1].start_x = 640;
	graphic_draw.graphic_data_struct[1].start_y = 270;
	graphic_draw.graphic_data_struct[1].details_d = 1280;
	graphic_draw.graphic_data_struct[1].details_e = 770;

	/*Ħ���ֿ���ui*/
   /*  
	if (shoot_data.fric_state == FRIC_ON)
	{
		graphic_draw.graphic_data_struct[2].operate_tpye = 1;
	}
	else
	{
		graphic_draw.graphic_data_struct[2].operate_tpye = 3;
	}
	graphic_draw.graphic_data_struct[2].graphic_name[0] = '0';
	graphic_draw.graphic_data_struct[2].graphic_name[1] = '0';
	graphic_draw.graphic_data_struct[2].graphic_name[2] = '4';
	
	graphic_draw.graphic_data_struct[2].color = 2;
	graphic_draw.graphic_data_struct[2].graphic_tpye = 2;
	
	graphic_draw.graphic_data_struct[2].layer = 0;
	graphic_draw.graphic_data_struct[2].width = 20; //��������
	
	graphic_draw.graphic_data_struct[2].start_x = 1800;
	graphic_draw.graphic_data_struct[2].start_y = 600;
	
	graphic_draw.graphic_data_struct[2].details_c = 20; */
        
	//�Ƿ���С����
	if (chassis_move_data.chassis_mode == chassis_spin){
		graphic_draw.graphic_data_struct[3].color = 2;
		graphic_draw.graphic_data_struct[3].operate_tpye = 1;
	}
	else{
		graphic_draw.graphic_data_struct[3].color = 3;
		graphic_draw.graphic_data_struct[3].operate_tpye = 3;
	}
	graphic_draw.graphic_data_struct[3].graphic_name[0] = '0';
	graphic_draw.graphic_data_struct[3].graphic_name[1] = '0';
	graphic_draw.graphic_data_struct[3].graphic_name[2] = '3';
	
	graphic_draw.graphic_data_struct[3].graphic_tpye = 2;
	
	graphic_draw.graphic_data_struct[3].layer = 0;
	graphic_draw.graphic_data_struct[3].width = 20; //��������
	
	graphic_draw.graphic_data_struct[3].start_x = 1800;
	graphic_draw.graphic_data_struct[3].start_y = 800;
	
	graphic_draw.graphic_data_struct[3].details_c = 20;

	//���ģ�?
	graphic_draw.graphic_data_struct[4].graphic_name[0] = '1';
	graphic_draw.graphic_data_struct[4].graphic_name[1] = '0';
	graphic_draw.graphic_data_struct[4].graphic_name[2] = '9';
	if(auto_state){
	    graphic_draw.graphic_data_struct[4].operate_tpye = 1;
	    auto_state = false;
	}
	else
	{
	    graphic_draw.graphic_data_struct[4].operate_tpye = 2;
	}

	graphic_draw.graphic_data_struct[4].graphic_tpye = 1;
	graphic_draw.graphic_data_struct[4].layer = 0;

	/* if(shoot_data.shoot_mode == SHOOT_READY_COUNTINUE){
		graphic_draw.graphic_data_struct[4].color = 2;
	}
    else if(shoot_data.shoot_mode == SHOOT_READY_SINGLE){
		graphic_draw.graphic_data_struct[4].color = 3;
    }
 	else{
		graphic_draw.graphic_data_struct[4].color = 8;
	}
	
	graphic_draw.graphic_data_struct[4].width = 40; 
	graphic_draw.graphic_data_struct[4].start_x = 1775;
	graphic_draw.graphic_data_struct[4].start_y = 400;
	
	graphic_draw.graphic_data_struct[4].details_d = 1825;
	graphic_draw.graphic_data_struct[4].details_e = 400;


	graphic_draw.graphic_data_struct[5].graphic_name[0] = '2';
	graphic_draw.graphic_data_struct[5].graphic_name[1] = '7';
	graphic_draw.graphic_data_struct[5].graphic_name[2] = '3';

	graphic_draw.graphic_data_struct[5].operate_tpye = 1;

	graphic_draw.graphic_data_struct[5].graphic_tpye = 0;
	graphic_draw.graphic_data_struct[5].layer = 0;

	graphic_draw.graphic_data_struct[5].color = 8;
	graphic_draw.graphic_data_struct[5].width = 1;

    graphic_draw.graphic_data_struct[5].start_x = 760;
	graphic_draw.graphic_data_struct[5].start_y = 80;
	graphic_draw.graphic_data_struct[5].details_d = 1172;
	graphic_draw.graphic_data_struct[5].details_e = 60; */


	graphic_draw.graphic_data_struct[6].graphic_name[0] = '2';
	graphic_draw.graphic_data_struct[6].graphic_name[1] = '7';
	graphic_draw.graphic_data_struct[6].graphic_name[2] = '4';

	graphic_draw.graphic_data_struct[6].operate_tpye = 1;

	graphic_draw.graphic_data_struct[6].graphic_tpye = 0;
	graphic_draw.graphic_data_struct[6].layer = 0;

	graphic_draw.graphic_data_struct[6].color = 8;
	graphic_draw.graphic_data_struct[6].width = 1;

    graphic_draw.graphic_data_struct[6].start_x = 760;
	graphic_draw.graphic_data_struct[6].start_y = 80;
	graphic_draw.graphic_data_struct[6].details_d = 1172;
	graphic_draw.graphic_data_struct[6].details_e = 60;

	referee_send_multi_graphic(MY_CLIENT_ID, &graphic_draw);
}

void send_capvol_graphic(int capvols)
{
	graphic_data_struct_t graphic_draw;
	char capname[3] = "209";
	graphic_draw.graphic_name[0] = capname[0];
	graphic_draw.graphic_name[1] = capname[1];
	graphic_draw.graphic_name[2] = capname[2];
	if(capdraw){
	    graphic_draw.operate_tpye = 1;
	    capdraw = false;
	}
	else
	{
	    graphic_draw.operate_tpye = 2;
	}

	if(cap_FSM == cap_auto_charge){
		graphic_draw.color = 2;
	}
	else{
		graphic_draw.color = 1;
	}

	graphic_draw.graphic_tpye = 0;
	graphic_draw.layer = 1;

	graphic_draw.width = 18; //��������
	graphic_draw.start_x = 761;
	graphic_draw.start_y = 70;
	
	graphic_draw.details_d  = 761 + (capvols - 6) * 18;
	graphic_draw.details_e  = 70;
	
	//memcpy(graphic_draw.graphic_name, (uint8_t*)name, strlen(name));
	referee_send_client_graphic(MY_CLIENT_ID, &graphic_draw);
}


void referee_send_multi_graphic(ext_id_t target_id, ext_interaction_figure_seven_t* graphic_draw){

	static ext_robot_seven_graphic_data_t robot_data;

	robot_data.header.sof = REFEREE_FRAME_HEADER_SOF;
	robot_data.header.seq++;
	//robot_data.header.data_length = sizeof(robot_data) - sizeof(robot_data.header) - sizeof(robot_data.cmd_id) - sizeof(robot_data.crc16);
	robot_data.header.data_length = 6 + 7 *15;
	append_CRC8_check_sum((uint8_t*)&robot_data.header, sizeof(robot_data.header));
	
	robot_data.cmd_id = robot_interactive_header;
	robot_data.data_id = 0x0104;
	robot_data.sender_id = MY_ROBOT_ID;
	robot_data.robot_id = target_id;

	robot_data.graphic_data = *graphic_draw;
	append_CRC16_check_sum((uint8_t*)&robot_data, sizeof(robot_data));

	memcpy(Personal_Data, (uint8_t*)&robot_data, sizeof(robot_data));
	usart6_tx_dma_enable(Personal_Data, sizeof(robot_data));//�����ƺõ�ͼ�����ݴ�����?

}

void referee_send_five_graphic(ext_id_t target_id, ext_interaction_figure_five_t* graphic_draw){

	static ext_robot_five_graphic_data_t robot_data;

	robot_data.header.sof = REFEREE_FRAME_HEADER_SOF;
	robot_data.header.seq++;
	//robot_data.header.data_length = sizeof(robot_data) - sizeof(robot_data.header) - sizeof(robot_data.cmd_id) - sizeof(robot_data.crc16);
	robot_data.header.data_length = 6 + 5 *15;
	append_CRC8_check_sum((uint8_t*)&robot_data.header, sizeof(robot_data.header));
	
	robot_data.cmd_id = robot_interactive_header;
	robot_data.data_id = 0x0103;
	robot_data.sender_id = MY_ROBOT_ID;
	robot_data.robot_id = target_id;

	robot_data.graphic_data = *graphic_draw;
	append_CRC16_check_sum((uint8_t*)&robot_data, sizeof(robot_data));

	memcpy(Personal_Data, (uint8_t*)&robot_data, sizeof(robot_data));
	usart6_tx_dma_enable(Personal_Data, sizeof(robot_data));//�����ƺõ�ͼ�����ݴ�����?

}

void referee_send_two_graphic(ext_id_t target_id, ext_interaction_figure_double_t* graphic_draw) {

	static ext_robot_two_graphic_data_t robot_data;

	robot_data.header.sof = REFEREE_FRAME_HEADER_SOF;
	robot_data.header.seq++;
	//robot_data.header.data_length = sizeof(robot_data) - sizeof(robot_data.header) - sizeof(robot_data.cmd_id) - sizeof(robot_data.crc16);
	robot_data.header.data_length = 6 + 2 * 15;
	append_CRC8_check_sum((uint8_t*)&robot_data.header, sizeof(robot_data.header));
	
	robot_data.cmd_id = robot_interactive_header;
	robot_data.data_id = 0x0102;
	robot_data.sender_id = MY_ROBOT_ID;
	robot_data.robot_id = target_id;

	robot_data.graphic_data = *graphic_draw;
	append_CRC16_check_sum((uint8_t*)&robot_data, sizeof(robot_data));

	memcpy(Personal_Data, (uint8_t*)&robot_data, sizeof(robot_data));
	usart6_tx_dma_enable(Personal_Data, sizeof(robot_data));//�����ƺõ�ͼ�����ݴ�����?
}

void send_double_text(void)
{
	ext_interaction_figure_double_t graphic_draw;
	char name1[3] = "221";
	graphic_draw.graphic_data_struct[0].graphic_name[0] = name1[0];
	graphic_draw.graphic_data_struct[0].graphic_name[1] = name1[1];
	graphic_draw.graphic_data_struct[0].graphic_name[2] = name1[2];
	graphic_draw.graphic_data_struct[0].operate_tpye = 1;
	graphic_draw.graphic_data_struct[0].graphic_tpye = 0;
	graphic_draw.graphic_data_struct[0].layer = 0;
	graphic_draw.graphic_data_struct[0].color = 2;
	graphic_draw.graphic_data_struct[0].width =2;
	graphic_draw.graphic_data_struct[0].start_x =556;
	graphic_draw.graphic_data_struct[0].start_y =0;
	graphic_draw.graphic_data_struct[0].details_d =706;
	graphic_draw.graphic_data_struct[0].details_e =240;
	
	char name2[3] = "223";
	graphic_draw.graphic_data_struct[1].graphic_name[0] = name2[0];
	graphic_draw.graphic_data_struct[1].graphic_name[1] = name2[1];
	graphic_draw.graphic_data_struct[1].graphic_name[2] = name2[2];
	graphic_draw.graphic_data_struct[1].operate_tpye = 1;
	graphic_draw.graphic_data_struct[1].graphic_tpye = 0;
	graphic_draw.graphic_data_struct[1].layer = 0;
	graphic_draw.graphic_data_struct[1].color = 2;
	graphic_draw.graphic_data_struct[1].width =2;
	graphic_draw.graphic_data_struct[1].start_x =1364;
	graphic_draw.graphic_data_struct[1].start_y =0;
	graphic_draw.graphic_data_struct[1].details_d =1214;
	graphic_draw.graphic_data_struct[1].details_e =240;
	
	referee_send_two_graphic(MY_CLIENT_ID, &graphic_draw);
	
}


void referee_send_client_graphic(ext_id_t target_id, graphic_data_struct_t* graphic_draw) 
{
	static ext_robot_graphic_data_t robot_data;

	robot_data.header.sof = REFEREE_FRAME_HEADER_SOF;
	robot_data.header.seq++;
	robot_data.header.data_length = sizeof(robot_data) - sizeof(robot_data.header) - sizeof(robot_data.cmd_id) - sizeof(robot_data.crc16);
	
	append_CRC8_check_sum((uint8_t*)&robot_data.header, sizeof(robot_data.header));
	
	robot_data.cmd_id = robot_interactive_header;
	robot_data.data_id = 0x0101;
	robot_data.sender_id = MY_ROBOT_ID;
	robot_data.robot_id = target_id;
	
	robot_data.graphic_data = *graphic_draw;
	append_CRC16_check_sum((uint8_t*)&robot_data, sizeof(robot_data));//֡ͷCRCУ��

	memcpy(Personal_Data, (uint8_t*)&robot_data, sizeof(robot_data));
	usart6_tx_dma_enable(Personal_Data, sizeof(robot_data));

}

void referee_send_client_character(ext_id_t target_id, ext_client_custom_character_t* character_data) {


	static ext_robot_character_data_t robot_data;

	robot_data.header.sof = REFEREE_FRAME_HEADER_SOF;
	robot_data.header.seq++;
	robot_data.header.data_length = sizeof(robot_data) - sizeof(robot_data.header) - sizeof(robot_data.cmd_id) - sizeof(robot_data.crc16);
	append_CRC8_check_sum((uint8_t*)&robot_data.header, sizeof(robot_data.header));

	robot_data.cmd_id = robot_interactive_header;
	robot_data.data_id = 0x0110;
	robot_data.sender_id = MY_ROBOT_ID;
	robot_data.robot_id = target_id;
	
	robot_data.character_data = *character_data;
	append_CRC16_check_sum((uint8_t*)&robot_data, sizeof(robot_data));

	memcpy(Personal_Data, (uint8_t*)&robot_data, sizeof(robot_data));
	usart6_tx_dma_enable(Personal_Data, sizeof(robot_data));

}

//�ַ����ͺ���
void  send_string(char* str, char* name, int x, int y, int upd, int colour)
{
	int len = strlen(str) + 1;
	ext_client_custom_character_t character_data;
	character_data.graphic_data_struct.graphic_tpye = 7;//string

	character_data.graphic_data_struct.operate_tpye = upd;//modify

	character_data.graphic_data_struct.layer = 0;

	character_data.graphic_data_struct.color = colour;//green

	character_data.graphic_data_struct.width = 2;
	character_data.graphic_data_struct.details_a = 20;//size
	character_data.graphic_data_struct.details_b = len;//length
	character_data.graphic_data_struct.start_x = x;
	character_data.graphic_data_struct.start_y = y;

	memcpy(character_data.data, (uint8_t*)str, len);
	memcpy(character_data.graphic_data_struct.graphic_name, (uint8_t*)name, strlen(name));

	referee_send_client_character(MY_CLIENT_ID, &character_data);
}

void send_rub_graphic(char graphname[3], int x, int y, int color, int type)
{
	
	graphic_data_struct_t graphic_draw;
	graphic_draw.graphic_name[0] = graphname[0];
	graphic_draw.graphic_name[0] = graphname[1];
	graphic_draw.graphic_name[0] = graphname[2];
	
	graphic_draw.operate_tpye = type;
	
	graphic_draw.color = color;
	graphic_draw.graphic_tpye = 2;
	
	graphic_draw.layer = 0;
	graphic_draw.width = 20; //��������
	
	graphic_draw.start_x = x;
	graphic_draw.start_y = y;
	
	graphic_draw.details_c = 20;
	
	//memcpy(graphic_draw.graphic_name, (uint8_t*)name, strlen(name));

	referee_send_client_graphic(MY_CLIENT_ID, &graphic_draw);
}

//��һ��ֱ��
void send_line_graphic(char name[3], uint32_t start_x, uint32_t start_y, uint32_t end_x, uint32_t end_y){

    graphic_data_struct_t graphic_draw;

	graphic_draw.graphic_name[0] = name[0];
	graphic_draw.graphic_name[1] = name[1];
	graphic_draw.graphic_name[2] = name[2];

	graphic_draw.operate_tpye = 1;

	graphic_draw.graphic_tpye = 0;
	graphic_draw.layer = 2;

	graphic_draw.color = 8;
	graphic_draw.width = 2;

    graphic_draw.start_x = start_x;
	graphic_draw.start_y = start_y;
	graphic_draw.details_d = end_x;
	graphic_draw.details_e = end_y;

	referee_send_client_graphic(MY_CLIENT_ID, &graphic_draw);
}

//����
void send_rectangle_graphic(char name[3], uint32_t x, uint32_t y, uint32_t end_x, uint32_t end_y){
	graphic_data_struct_t graphic_draw;

	graphic_draw.graphic_name[0] = name[0];
	graphic_draw.graphic_name[1] = name[1];
	graphic_draw.graphic_name[2] = name[2];

	graphic_draw.operate_tpye = 1;

	graphic_draw.graphic_tpye = 1;
	graphic_draw.layer = 0;

	graphic_draw.color = 8;
	graphic_draw.width = 1
	;

    graphic_draw.start_x = x;
	graphic_draw.start_y = y;
	graphic_draw.details_d = end_x;
	graphic_draw.details_e = end_y;

	referee_send_client_graphic(MY_CLIENT_ID, &graphic_draw);
}

//ϸԲ
void send_circle_graphic(char name[3], uint32_t x, uint32_t y, uint32_t radius){

	graphic_data_struct_t graphic_draw;

	graphic_draw.graphic_name[0] = name[0];
	graphic_draw.graphic_name[1] = name[1];
	graphic_draw.graphic_name[2] = name[2];

	graphic_draw.operate_tpye = 1;

	graphic_draw.graphic_tpye = 2;
	graphic_draw.layer = 0;

	graphic_draw.color = 8;
	graphic_draw.width = 2;

    graphic_draw.start_x = x;
	graphic_draw.start_y = y;
	graphic_draw.details_c = radius;

	referee_send_client_graphic(MY_CLIENT_ID, &graphic_draw);
}

//��Բ
void send_spinning_graphic(char name[3], int x, int y, int color, int type)
{
	
	graphic_data_struct_t graphic_draw;
	
	graphic_draw.graphic_name[0] =name[0];
	graphic_draw.graphic_name[1] =name[1];
	graphic_draw.graphic_name[2] =name[2];
	
	
	graphic_draw.operate_tpye = type;
	
	graphic_draw.color = color;
	graphic_draw.graphic_tpye = 2;
	
	graphic_draw.layer = 0;
	graphic_draw.width = 20; //��������
	
	graphic_draw.start_x = x;
	graphic_draw.start_y = y;
	
	graphic_draw.details_c = 20;
	
	//memcpy(graphic_draw.graphic_name, (uint8_t*)name, strlen(name));

	referee_send_client_graphic(MY_CLIENT_ID, &graphic_draw);
}


void USART6_IRQHandler(void)
{
    static volatile uint8_t res;
    if(USART6->SR & UART_FLAG_IDLE)
    {
        __HAL_UART_CLEAR_PEFLAG(&huart6);

        static uint16_t this_time_rx_len = 0;

        if ((huart6.hdmarx->Instance->CR & DMA_SxCR_CT) == RESET)
        {
            __HAL_DMA_DISABLE(huart6.hdmarx);
            this_time_rx_len = USART_RX_BUF_LENGHT - __HAL_DMA_GET_COUNTER(huart6.hdmarx);
            __HAL_DMA_SET_COUNTER(huart6.hdmarx, USART_RX_BUF_LENGHT);
            huart6.hdmarx->Instance->CR |= DMA_SxCR_CT;
            __HAL_DMA_ENABLE(huart6.hdmarx);
            fifo_s_puts(&referee_fifo, (char*)usart6_buf[0], this_time_rx_len);
        }
        else
        {
            __HAL_DMA_DISABLE(huart6.hdmarx);
            this_time_rx_len = USART_RX_BUF_LENGHT - __HAL_DMA_GET_COUNTER(huart6.hdmarx);
            __HAL_DMA_SET_COUNTER(huart6.hdmarx, USART_RX_BUF_LENGHT);
            huart6.hdmarx->Instance->CR &= ~(DMA_SxCR_CT);
            __HAL_DMA_ENABLE(huart6.hdmarx);
            fifo_s_puts(&referee_fifo, (char*)usart6_buf[1], this_time_rx_len);
        }
    }
}


void send_autoaim_state(void){
	graphic_data_struct_t graphic_draw;
	char capname[3] = "409";
	graphic_draw.graphic_name[0] = capname[0];
	graphic_draw.graphic_name[1] = capname[1];
	graphic_draw.graphic_name[2] = capname[2];
	if(shoot){
	graphic_draw.operate_tpye = 1;
	shoot = false;
	}
	else
	{
	graphic_draw.operate_tpye = 2;
	}
	graphic_draw.graphic_tpye = 1;
	graphic_draw.layer = 0;
	if(gimbal_data.gimbal_behaviour == GIMBAL_AUTO){
		if(autoaim_measure.vision_state == 1){
		  graphic_draw.color = 2;
		}
		else{
		  graphic_draw.color = 3;
		}
	}
	else{
		graphic_draw.color = 8;
	}
	graphic_draw.width = 40; 
	graphic_draw.start_x = 1775;
	graphic_draw.start_y = 700;
	
	graphic_draw.details_d = 1825;
	graphic_draw.details_e = 700;
	referee_send_client_graphic(MY_CLIENT_ID, &graphic_draw);
}

void send_shoot_state(void){
	graphic_data_struct_t graphic_draw;
	char capname[3] = "109";
	graphic_draw.graphic_name[0] = capname[0];
	graphic_draw.graphic_name[1] = capname[1];
	graphic_draw.graphic_name[2] = capname[2];
	if(auto_state){
	graphic_draw.operate_tpye = 1;
	auto_state = false;
	}
	else
	{
	graphic_draw.operate_tpye = 2;
	}
	graphic_draw.graphic_tpye = 1;
	graphic_draw.layer = 0;
	if(shoot_control.shoot_mode == SHOOT_READY){
		  graphic_draw.color = 2;
	}
  else if(shoot_control.shoot_mode == SHOOT_BULLET){
		  graphic_draw.color = 3;
  }
	else{
		graphic_draw.color = 8;
	}
	graphic_draw.width = 40; 
	graphic_draw.start_x = 1775;
	graphic_draw.start_y = 400;
	
	graphic_draw.details_d = 1825;
	graphic_draw.details_e = 400;
	referee_send_client_graphic(MY_CLIENT_ID, &graphic_draw);
}

//��ȡ���̹���
void get_chassis_power_and_buffer(fp32 *power, fp32 *buffer)
{
    *power = ext_power_heat_data.chassis_power;
    *buffer = ext_power_heat_data.buffer_energy;
}

//��ȡǹ������
void get_shoot_speed(float *speed){
	*speed = ext_robot_shoot_data.initial_speed;;
}
