/**
 * @file Init.cpp
 * @author qingyu
 * @brief 
 * @version 0.1
 * @date 2025-10-14
 * 
 * @copyright Copyright (c) 2025
 * 
 */
/* Includes ------------------------------------------------------------------*/

#include "Init.h"
#include "Robot.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

Robot robot_;

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/**
 * @brief can1回调函数
 * 
 * @param CAN_RxMessage 
 */
void can1_callback_function(CanRxBuffer *CAN_RxMessage)
{
    switch (CAN_RxMessage->header.StdId)
    {
        case (0x201):
        {
            robot_.shoot_.motor_shoot_1_.CanRxCpltCallback(CAN_RxMessage->data);
            break;
        }
        case (0x202):
        {
            robot_.shoot_.motor_shoot_2_.CanRxCpltCallback(CAN_RxMessage->data);
            break;
        }
        case (0x203):
        {
            robot_.shoot_.motor_shoot_3_.CanRxCpltCallback(CAN_RxMessage->data);
            break;
        }
        case (0x204):
        {
            robot_.shoot_.motor_shoot_4_.CanRxCpltCallback(CAN_RxMessage->data);
            break;
        }
    }
}

/**
 * @brief can2回调函数
 * 
 * @param CAN_RxMessage 
 */
void can2_callback_function(CanRxBuffer* CAN_RxMessage)
{
    switch (CAN_RxMessage->header.StdId) 
    {
        case (0x05):
        {
            robot_.gimbal_.motor_pitch_.CanRxCpltCallback(CAN_RxMessage->data);
            break;
        }
    }
}

/**
 * @brief remote回调函数
 * 
 * @param buffer 
 * @param length 
 */
void uart3_callback_function(uint8_t* buffer, uint16_t length) 
{	
	robot_.remote_dr16_.UartRxCpltCallback(buffer);

    robot_.mcu_comm_.send_chassis_data_.start_of_frame   = 0xAA;
    robot_.mcu_comm_.send_chassis_data_.chassis_speed_x  = robot_.remote_dr16_.output_.chassis_x;
    robot_.mcu_comm_.send_chassis_data_.chassis_speed_y  = robot_.remote_dr16_.output_.chassis_y;
    robot_.mcu_comm_.send_chassis_data_.rotation         = robot_.remote_dr16_.output_.rotation;

    robot_.mcu_comm_.send_comm_data_.start_of_frame      = 0xAB;
    robot_.mcu_comm_.send_comm_data_.switch_l            = robot_.remote_dr16_.output_.switch_l;
    robot_.mcu_comm_.send_comm_data_.switch_r            = robot_.remote_dr16_.output_.switch_r;
    robot_.mcu_comm_.send_comm_data_.supercap            = 0;
}

/**
 * @bief USB接收完成回调函数
 *
 * @param len 接收到的数据长度
 */
void usb_rx_callback(uint16_t len)
{
    robot_.pc_comm_.RxCpltCallback();
}

/**
 * @bief USB发送完成回调函数
 *
 * @param len 发送的数据长度
 */
void usb_tx_callback(uint16_t len)
{
    
}

/* Function prototypes -------------------------------------------------------*/

void Init()
{
    usb_init(usb_tx_callback, usb_rx_callback);
    can_init(&hcan1, can1_callback_function);
    can_init(&hcan2, can2_callback_function);
    robot_.Init();
}
