/**
 * @file app_robot.cpp
 * @author qingyu
 * @brief 
 * @version 0.1
 * @date 2025-10-21
 * 
 * @copyright Copyright (c) 2025
 * 
 */
/* Includes ------------------------------------------------------------------*/

#include "Robot.h"
#include "dvc_remote_dr16.h"

/* Private macros ------------------------------------------------------------*/

#define INTERVAL_LIMIT(data, max, min)      \
    do{                                     \
         if((data) >= (max)){               \
            (data) = (max);                 \
        } else if((data) <= (min)){         \
            (data) = (min);                 \
        }                                   \
    }while(0)

#define REMOTE_PITCH_RATIO        0.1f
#define AUTOAIM_PITCH_RATIO       350.f

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/**
 * @brief Robot初始化函数
 * 
 */
void Robot::Init()
{
    dwt_init(168);

    // dr16遥控初始化
    remote_dr16_.Init(&huart3, uart3_callback_function, UART_BUFFER_LENGTH);

    // 陀螺仪初始化
    imu_.Init();

    // 上下板通讯组件初始化
    mcu_comm_.Init(&hcan2, 0x00, 0x01);

    // 上位机通讯
    pc_comm_.Init();

    // 云台初始化
    gimbal_.Init();

    // 摩擦轮初始化
    shoot_.Init();
    
    static const osThreadAttr_t kRobotTaskAttr = 
    {
        .name = "robot_task",
        .stack_size = 1024,
        .priority = (osPriority_t) osPriorityNormal
    };
    // 启动任务，将 this 传入
    osThreadNew(Robot::TaskEntry, this, &kRobotTaskAttr);
}

/**
 * @brief 任务入口（静态函数）—— osThreadNew 需要这个原型
 * 
 * @param argument 
 */
void Robot::TaskEntry(void *argument)
{
    Robot *self = static_cast<Robot *>(argument);  // 还原 this 指针
    self->Task();  // 调用成员函数
}

/**
 * @brief Robot任务函数
 * 
 */
void Robot::Task()
{
    for(;;)
    {
        /****************************   MCUcomm   ****************************/


        mcu_comm_.UpdataAutoaimData(&pc_comm_.recv_autoaim_data);

        mcu_comm_.CanSendAutoaimData();


        /****************************   PCcomm   ****************************/

        
        

        
        /****************************   Gimbal   ****************************/


        if(remote_dr16_.output_.remote.switch_r == SWITCH_UP || remote_dr16_.output_.mouse.press_r == REMOTE_DR16_KEY_STATUS_PRESS)
        {
            if(pc_comm_.recv_autoaim_data.mode == PC_AUTOAIM_MODE_IDIE)
            {
                remote_radian = remote_dr16_.output_.remote.pitch + remote_dr16_.output_.mouse.mouse_y;
            }
            else 
            {
                float filtered_autoaim =  gimbal_.pitch_autoaim_filter_.Update(pc_comm_.recv_autoaim_data.pitch.pitch_ang);
                
                remote_radian -= filtered_autoaim / AUTOAIM_PITCH_RATIO;
            }

            INTERVAL_LIMIT(remote_radian, MAX_PITCH_RADIAN, MIN_PITCH_RADIAN);

            gimbal_.SetTargetPitchRadian(remote_radian);
        }
        else if(remote_dr16_.output_.remote.switch_r == SWITCH_DOWN || remote_dr16_.output_.mouse.press_r == REMOTE_DR16_KEY_STATUS_FREE)
        {
            remote_radian = remote_dr16_.output_.remote.pitch + remote_dr16_.output_.mouse.mouse_y;

            INTERVAL_LIMIT(remote_radian, MAX_PITCH_RADIAN, MIN_PITCH_RADIAN);

            gimbal_.SetTargetPitchRadian(remote_radian);
        }
    
        gimbal_.SetImuPitchRadian(normalize_angle_pm_pi(imu_.GetRollAngle()));


        /****************************   Mode   ****************************/


        if(remote_dr16_.output_.remote.switch_r == SWITCH_UP || remote_dr16_.output_.mouse.press_r || remote_dr16_.output_.keyboard.keycode.e)
        {
            shoot_.SetTargetShootOmega(MAX_SHOOT_OMEGA);
        }
        else
        {
            shoot_.SetTargetShootOmega(0);
        }


        osDelay(pdMS_TO_TICKS(1));
    }
}



