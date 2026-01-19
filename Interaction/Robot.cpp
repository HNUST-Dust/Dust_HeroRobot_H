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

/* Private macros ------------------------------------------------------------*/

#define INTERVAL_LIMIT(data, max, min)      \
    do{                                     \
         if((data) >= (max)){               \
            (data) = (max);                 \
        } else if((data) <= (min)){         \
            (data) = (min);                 \
        }}while(0)

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

    // vt02遥控初始化
    remote_vt02_.Init(&huart1, uart1_callback_function, UART_BUFFER_LENGTH);

    // 陀螺仪初始化
    imu_.Init();

    // 上下板通讯组件初始化
    mcu_comm_.Init(&hcan2, 0x00, 0x01);

    // 上位机通讯
    pc_comm_.Init();

    // 等待云台yaw角回正
    osDelay(pdMS_TO_TICKS(5000));

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

        
        

        
        /****************************   云台   ****************************/


        if(remote_vt02_.output_.mouse_r == REMOTE_VT02_KEY_STATUS_FREE)
        {
            remote_radian += (remote_vt02_.output_.mouse_y * PI) * REMOTE_PITCH_RATIO;

            INTERVAL_LIMIT(remote_radian, MAX_PITCH_RADIAN, MIN_PITCH_RADIAN);

            gimbal_.SetTargetPitchRadian(remote_radian);
        }

        if(remote_vt02_.output_.mouse_r == REMOTE_VT02_KEY_STATUS_PRESS)
        {
            switch (pc_comm_.recv_autoaim_data.mode)
            {
                case(AUTOAIM_MODE_IDIE):
                {
                    remote_radian += (remote_vt02_.output_.mouse_y * PI) * REMOTE_PITCH_RATIO;

                    INTERVAL_LIMIT(remote_radian, MAX_PITCH_RADIAN, MIN_PITCH_RADIAN);

                    gimbal_.SetTargetPitchRadian(remote_radian);

                    shoot_.SetTargetShootOmega(MAX_SHOOT_OMEGA);

                    break;
                }
                case(AUTOAIM_MODE_FOLLOW):
                {
                    float filtered_autoaim =  gimbal_.pitch_autoaim_filter_.Update(pc_comm_.recv_autoaim_data.pitch.pitch_ang);

                    remote_radian -= filtered_autoaim / AUTOAIM_PITCH_RATIO;

                    INTERVAL_LIMIT(remote_radian, MAX_PITCH_RADIAN, MIN_PITCH_RADIAN);

                    gimbal_.SetTargetPitchRadian(remote_radian);

                    shoot_.SetTargetShootOmega(MAX_SHOOT_OMEGA);

                    break;
                }
                case(AUTOAIM_MODE_FIRE):
                {
                    float filtered_autoaim =  gimbal_.pitch_autoaim_filter_.Update(pc_comm_.recv_autoaim_data.pitch.pitch_ang);

                    remote_radian -= filtered_autoaim / AUTOAIM_PITCH_RATIO;

                    INTERVAL_LIMIT(remote_radian, MAX_PITCH_RADIAN, MIN_PITCH_RADIAN);

                    gimbal_.SetTargetPitchRadian(remote_radian);

                    shoot_.SetTargetShootOmega(MAX_SHOOT_OMEGA);

                    break;
                }
            }
        }
        else if(remote_vt02_.output_.keyboard.keycode.f == REMOTE_VT02_KEY_STATUS_PRESS)
        {
            shoot_.SetTargetShootOmega(MAX_SHOOT_OMEGA);
        }
        else
        {
            shoot_.SetTargetShootOmega(0);
        }

        gimbal_.SetImuPitchRadian(normalize_angle_pm_pi(imu_.GetRollAngle()));


        /****************************   调试   ****************************/


        osDelay(pdMS_TO_TICKS(1));
    }
}



