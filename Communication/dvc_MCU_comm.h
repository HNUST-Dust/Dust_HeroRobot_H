/**
 * @file dvc_MCU_comm.h
 * @author qingyu
 * @brief 
 * @version 0.1
 * @date 2025-11-26
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#pragma once

/* Includes ------------------------------------------------------------------*/

#include "bsp_can.h"
#include "dvc_PC_comm.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 转换联合体
 * 
 */
union McuConv
{
    uint8_t b[4];
    float f;
};

/**
 * @brief Mcu存活状态枚举
 * 
 */
enum McuAliveState
{
    MCU_ALIVE_STATE_ENABLE = 0,
    MCU_ALIVE_STATE_DISABLE,
};

/**
 * @brief Mcu底盘数据结构体
 * 
 */
struct McuChassisData
{
    uint8_t          start_of_frame = 0xAA;     // 帧头
    uint16_t         chassis_speed_x;           // 平移方向：左、右
    uint16_t         chassis_speed_y;           // 平移方向：前、后
    uint16_t         rotation;                  // 旋转方向：不转、顺时针转、逆时针转
    union
    {
        uint8_t all;
        struct
        {
            uint8_t pause : 1;
            uint8_t cns : 2;
            uint8_t fn1 : 1;
            uint8_t fn2 : 1;
            uint8_t trigger : 1;
            uint8_t reserved : 2;
        };
    };
};

/**
 * @brief Mcu通用数据结构体
 * 
 */
struct McuCommandData
{
    uint8_t start_of_frame = 0xAB;

    union 
    {
        uint8_t all;
        struct 
        {
            uint8_t mouse_l : 2;
            uint8_t mouse_r : 2;
            uint8_t reserved : 4;
        };
    } mouse_lr;
    
    union
    {
        uint16_t all;
        struct
        {
            uint8_t w : 1;
            uint8_t s : 1;
            uint8_t a : 1;
            uint8_t d : 1;
            uint8_t shift : 1;
            uint8_t ctrl : 1;
            uint8_t q : 1;
            uint8_t e : 1;
            uint8_t r : 1;
            uint8_t f : 1;
            uint8_t g : 1;
            uint8_t z : 1;
            uint8_t x : 1;
            uint8_t c : 1;
            uint8_t v : 1;
            uint8_t b : 1;
        };
    } keyboard;

    float imu_yaw;                    // yaw轴角度
};

/**
 * @brief Mcu接收自瞄数据结构体
 * 
 */
struct McuSendAutoaimData
{
    uint8_t start_of_frame = 0xAC;
    uint8_t mode;
    float autoaim_yaw_angle;          // 自瞄yaw轴角度
    uint8_t is_autoaim_start = 0;
};

/**
 * @brief Mcu接收裁判系统数据结构体
 * 
 */
struct McuRecvRefereeData
{
    uint8_t start_of_yaw_frame = 0xAF;
    float bullet_speed;
};

/**
 * @brief Mcu通讯类
 * 
 */
class McuComm
{
public:

    McuChassisData send_chassis_data_ = 
    {
        0xAA,
        1024,
        1024,
        1024,
        15,
    };
    
    McuCommandData send_command_data_ = 
    {
        0xAB,
        0,
        0,
        0.0f,
    };

    McuSendAutoaimData send_autoaim_data_ = 
    {   0xAC,
        0,
        0.0f,
        0,
    };

    McuRecvRefereeData recv_referee_data_ = 
    {
        0xAF,
        0.0f,
    };

    void Init(CAN_HandleTypeDef *hcan, uint8_t can_rx_id, uint8_t can_tx_id);

    void Task();

    void ClearData();

    void CanSendChassisData();

    void CanSendCommandData();
    
    void CanSendAutoaimData();

    void CanRxCpltCallback(uint8_t *rx_data);

    inline McuAliveState GetMcuAliveState();

protected:

    CanManageObject *can_manage_object_;

    uint16_t can_rx_id_;

    uint16_t can_tx_id_;

    uint8_t tx_data_[8];

    uint32_t flag_ = 0;

    uint32_t pre_flag_ = 0;

    uint32_t alive_count_ = 0;

    McuAliveState mcu_alive_state_ = MCU_ALIVE_STATE_DISABLE;

    void DataProcess(uint8_t* rx_data);

    void AlivePeriodElapsedCallback();
    
    // FreeRTOS 入口，静态函数
    static void TaskEntry(void *param);
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

inline McuAliveState McuComm::GetMcuAliveState()
{
    return (mcu_alive_state_);
}
