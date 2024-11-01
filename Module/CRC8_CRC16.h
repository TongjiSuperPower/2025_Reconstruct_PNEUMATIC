#ifndef _CRC8_CRC16_H_
#define _CRC8_CRC16_H_

#include "main.h"
#include "struct_typedef.h"


/**
  * @brief          ����CRC8
  * @param[in]      pch_message: ����
  * @param[in]      dw_length: ���ݺ�У��ĳ���
  * @param[in]      ucCRC8:��ʼCRC8
  * @retval         �������CRC8
  */
uint8_t get_CRC8_check_sum(unsigned char *pchMessage,unsigned int dwLength,unsigned char ucCRC8);


/**
  * @brief          CRC8У�麯��
  * @param[in]      pch_message: ����
  * @param[in]      dw_length: ���ݺ�У��ĳ���
  * @retval         ����߼�
  */
uint32_t verify_CRC8_check_sum(unsigned char *pchMessage, unsigned int dwLength);


/**
  * @brief          ���CRC8�����ݵĽ�β
  * @param[in]      pch_message: ����
  * @param[in]      dw_length: ���ݺ�У��ĳ���
  * @retval         none
  */
void append_CRC8_check_sum(unsigned char *pchMessage, unsigned int dwLength);


/**
  * @brief          ����CRC16
  * @param[in]      pch_message: ����
  * @param[in]      dw_length: ���ݺ�У��ĳ���
  * @param[in]      wCRC:��ʼCRC16
  * @retval         �������CRC16
  */
uint16_t get_CRC16_check_sum(uint8_t *pchMessage,uint32_t dwLength,uint16_t wCRC);


/**
  * @brief          CRC16У�麯��
  * @param[in]      pch_message: ����
  * @param[in]      dw_length: ���ݺ�У��ĳ���
  * @retval         ����߼�
  */
uint32_t verify_CRC16_check_sum(uint8_t *pchMessage, uint32_t dwLength);


/**
  * @brief          ���CRC16�����ݵĽ�β
  * @param[in]      pch_message: ����
  * @param[in]      dw_length: ���ݺ�У��ĳ���
  * @retval         none
  */
void append_CRC16_check_sum(uint8_t * pchMessage,uint32_t dwLength);

#endif
