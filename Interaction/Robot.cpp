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

constexpr float REMOTE_PITCH_RATIO  = 0.1f;
constexpr float AUTOAIM_PITCH_RATIO = 350.f;

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

    // 遥控初始化
    remote_vt03_.Init(&huart6, uart6_callback_function, UART_BUFFER_LENGTH);

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
    McuRecvRefereeData mcu_referee_data_local;
    mcu_referee_data_local.bullet_speed = 0.0f;

    for(;;)
    {
        /****************************   MCUcomm   ****************************/


        __disable_irq();
        mcu_referee_data_local = *(static_cast<McuRecvRefereeData*>(&(mcu_comm_.recv_referee_data_)));
        __enable_irq();

        if (!remote_vt03_.remote_alive_status) {
            mcu_comm_.ClearData();
        }

        if (!pc_comm_.pc_alive_state) {
            mcu_comm_.send_autoaim_data_.is_autoaim_start = 0;
        }

        mcu_comm_.CanSendAutoaimData();


        /****************************   PCcomm   ****************************/

        
        pc_comm_.UpdataAutoaimData(mcu_referee_data_local);

        
        /****************************   Gimbal   ****************************/


        if(remote_vt03_.output_.remote.fn2 || remote_vt03_.output_.mouse.mouse_r)
        {
            if(pc_comm_.recv_autoaim_data.mode == PC_AUTOAIM_MODE_IDIE)
            {
                remote_radian = remote_vt03_.output_.remote.pitch + remote_vt03_.output_.mouse.mouse_y;
            }
            else 
            {
                float filtered_autoaim =  gimbal_.pitch_autoaim_filter_.Update(pc_comm_.recv_autoaim_data.pitch.pitch_ang);

                remote_radian -= filtered_autoaim / AUTOAIM_PITCH_RATIO;
            }

            remote_radian = std::clamp(remote_radian, MIN_PITCH_RADIAN, MAX_PITCH_RADIAN);

            gimbal_.SetTargetPitchRadian(remote_radian);
        }
        else if(!remote_vt03_.output_.remote.fn2 || !remote_vt03_.output_.mouse.mouse_r)
        {
            remote_radian = remote_vt03_.output_.remote.pitch + remote_vt03_.output_.mouse.mouse_y;

            remote_radian = std::clamp(remote_radian, MIN_PITCH_RADIAN, MAX_PITCH_RADIAN);

            gimbal_.SetTargetPitchRadian(remote_radian);
        }
    
        gimbal_.SetImuPitchRadian(normalize_angle_pm_pi(imu_.GetRollAngle()));


        /****************************   Mode   ****************************/


        if(remote_vt03_.output_.remote.fn2 || remote_vt03_.output_.keyboard.f)
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



