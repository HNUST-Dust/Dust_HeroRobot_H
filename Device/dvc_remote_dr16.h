/**
 * @file dvc_remote_dji.h
 * @author qingyu
 * @brief 
 * @version 0.1
 * @date 2025-10-18
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#ifndef __DVC_REMOTE_DJI_DR16_H__
#define __DVC_REMOTE_DJI_DR16_H__

/* Includes ------------------------------------------------------------------*/

#include "bsp_uart.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"

/* Exported macros -----------------------------------------------------------*/

#define MAX_REMOTE_DR16_CHANNLE      1684
#define MID_REMOTE_DR16_CHANNLE      1024
#define MIN_REMOTE_DR16_CHANNLE      364

#define MID_REMOTE_DR16_SWITCH_LR    15

/* Exported types ------------------------------------------------------------*/

/**
 * @brief DR16在线状态枚举
 * 
 */
enum RemoteDR16AliveStatus
{
    REMOTE_DR16_ALIVE_STATUS_DISABLE = 0,
    REMOTE_DR16_ALIVE_STATUS_ENABLE  = 1,
};

/**
 * @brief DR16拨杆状态枚举
 * 
 */
enum RemoteDR16SwitchStatus
{
    SWITCH_UP    = (uint8_t)1,
    SWITCH_MID   = (uint8_t)3,
    SWITCH_DOWN  = (uint8_t)2,
};

/**
 * @brief DR16原始数据结构体
 * 
 */
struct RemoteDR16RawData
{
    struct 
    {
        uint16_t ch0, ch1, ch2, ch3;
        uint8_t s1, s2;
    } rc;

    struct
    {
        int16_t x, y, z;
        uint8_t l, r;
    } mouse;

    struct 
    {
        uint16_t all;
    } keyboard;
};

/**
 * @brief DR16输出数据结构体
 * 
 */
struct RemoteDR16OutputData
{
    struct 
    {
        float chassis_x, chassis_y, rotation, pitch;
        uint8_t switch_l, switch_r;
    } remote;

    struct
    {
        float mouse_x, mouse_y, mouse_z;
        uint8_t mouse_l, mouse_r;
    } mouse;

    union
    {
        uint16_t all;
        struct
        {
            uint16_t w : 1; 
            uint16_t s : 1;
            uint16_t a : 1;
            uint16_t d : 1;
            uint16_t q : 1; 
            uint16_t e : 1;
            uint16_t shift : 1;
            uint16_t ctrl : 1;
            uint16_t reserved : 8;
        } keycode;
    } keyboard;
};

/**
 * @brief DR16遥控器
 * 
 */
class RemoteDjiDR16
{
public:
    // 遥控器输出数据
    RemoteDR16OutputData output_;

    // 遥控器状态
    RemoteDR16AliveStatus remote_dr16_alive_status = REMOTE_DR16_ALIVE_STATUS_DISABLE;

    void Init(UART_HandleTypeDef *huart, Uart_Callback callback_function, uint16_t rx_buffer_length);

    void Task();

    void AlivePeriodElapsedCallback();

    void UartRxCpltCallback(uint8_t* buffer);

    static void TaskEntry(void *param);  // FreeRTOS 入口，静态函数

private:
    // uart管理模块
    UartManageObject* uart_manage_object_;

    // 原始数据
    RemoteDR16RawData raw_data_;

    // 当前时刻flag
    uint32_t flag_ = 0;

    // 前一时刻flag
    uint32_t pre_flag_ = 0;

    // 掉线清理数据函数

    void ClearData();

    // 内部数据处理函数

    void DataProcess(uint8_t* rx_data);
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/


#endif