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
#include "dvc_remote_dr16.h"

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
 * @brief dr16回调函数
 * 
 * @param buffer 
 * @param length 
 */
void uart1_callback_function(uint8_t* buffer, uint16_t length) 
{	
	robot_.remote_vt02_.UartRxCpltCallback(buffer);

    robot_.mcu_comm_.send_chassis_data_.start_of_frame = 0xAA;

    // x方向
    if(robot_.remote_vt02_.output_.keyboard.keycode.w)
    {
        robot_.mcu_comm_.send_chassis_data_.chassis_speed_y = MAX_REMOTE_DR16_CHANNLE;
    }
    else if(robot_.remote_vt02_.output_.keyboard.keycode.s)
    {
        robot_.mcu_comm_.send_chassis_data_.chassis_speed_y = MIN_REMOTE_DR16_CHANNLE;
    }
    else
    {
        robot_.mcu_comm_.send_chassis_data_.chassis_speed_y = MID_REMOTE_DR16_CHANNLE;
    }

    // y方向
    if(robot_.remote_vt02_.output_.keyboard.keycode.a)
    {
        robot_.mcu_comm_.send_chassis_data_.chassis_speed_x = MIN_REMOTE_DR16_CHANNLE;
    }
    else if(robot_.remote_vt02_.output_.keyboard.keycode.d)
    {
        robot_.mcu_comm_.send_chassis_data_.chassis_speed_x = MAX_REMOTE_DR16_CHANNLE;
    }
    else
    {
        robot_.mcu_comm_.send_chassis_data_.chassis_speed_x = MID_REMOTE_DR16_CHANNLE;
    }

    robot_.mcu_comm_.send_chassis_data_.switch_lr.all = MID_REMOTE_DR16_SWITCH_LR;          // VT02模式下未用，设置默认值

    // yaw轴
    robot_.mcu_comm_.send_chassis_data_.rotation = (1684 + 1320 * (robot_.remote_vt02_.output_.mouse_x - 32767) / 65535);


    robot_.mcu_comm_.send_command_data_.start_of_frame = 0xAB;

    // 键盘
    robot_.mcu_comm_.send_command_data_.keyboard.all = robot_.remote_vt02_.output_.keyboard.all;

    // 鼠标按键
    robot_.mcu_comm_.send_command_data_.mouse_lr.all = robot_.remote_vt02_.output_.mouse_lr.all;
}

/**
 * @brief dr16回调函数
 * 
 * @param buffer 
 * @param length 
 */
void uart3_callback_function(uint8_t* buffer, uint16_t length) 
{	
	robot_.remote_dr16_.UartRxCpltCallback(buffer);

    // printf("%f,%f,%f\n", robot_.remote_dr16_.output_.mouse.mouse_x, robot_.remote_dr16_.output_.mouse.mouse_y, robot_.remote_dr16_.output_.mouse.mouse_z);

    // uint16_t temp_x = 0, temp_y = 0;

    // robot_.mcu_comm_.send_chassis_data_.start_of_frame   = 0xAA;

    // if(robot_.remote_dr16_.output_.keyboard.keycode.w == REMOTE_VT02_KEY_STATUS_PRESS)
    // {
    //     temp_x = MAX_REMOTE_DR16_CHANNLE;
    // }
    // else if(robot_.remote_dr16_.output_.keyboard.keycode.s)
    // {
    //     temp_x = MIN_REMOTE_DR16_CHANNLE;
    // }
    // else
    // {
    //     temp_x = MID_REMOTE_DR16_CHANNLE;
    // }

    // if(robot_.remote_dr16_.output_.keyboard.keycode.a)
    // {
    //     temp_y = MIN_REMOTE_DR16_CHANNLE;
    // }
    // else if(robot_.remote_dr16_.output_.keyboard.keycode.d)
    // {
    //     temp_y = MAX_REMOTE_DR16_CHANNLE;
    // }
    // else
    // {
    //     temp_y = MID_REMOTE_DR16_CHANNLE;
    // }

    // robot_.mcu_comm_.send_chassis_data_.chassis_speed_x  = robot_.remote_dr16_.output_.remote.chassis_x;
    // robot_.mcu_comm_.send_chassis_data_.chassis_speed_y  = robot_.remote_dr16_.output_.remote.chassis_y;
    // robot_.mcu_comm_.send_chassis_data_.rotation         = robot_.remote_dr16_.output_.remote.rotation;

    // robot_.mcu_comm_.send_chassis_data_.switch_lr.switchcode.switch_l = robot_.remote_dr16_.output_.remote.switch_l;
    // robot_.mcu_comm_.send_chassis_data_.switch_lr.switchcode.switch_r = robot_.remote_dr16_.output_.remote.switch_r;


    // robot_.mcu_comm_.send_command_data_.start_of_frame   = 0xAB;

    // robot_.mcu_comm_.send_command_data_.keyboard.all     = robot_.remote_dr16_.output_.keyboard.all;
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
