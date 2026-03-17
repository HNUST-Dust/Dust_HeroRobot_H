/**
 * @file dvc_remote_vt03.cpp
 * @author qingyu
 * @brief 
 * @version 0.1
 * @date 2026-01-17
 * 
 * @copyright Copyright (c) 2026
 * 
 */
/* Includes ------------------------------------------------------------------*/

#include "dvc_remote_vt03.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief VT03清理数据函数
 * 
 */
void RemoteDjiVT03::ClearData()
{
    output_.remote.chassis_x = 1024;
    output_.remote.chassis_y = 1024;
    output_.remote.rotation = 1024;
    output_.remote.all = 0;

    output_.mouse.mouse_x = 0;
    output_.mouse.mouse_y = 0;
    output_.mouse.mouse_z = 0;

    output_.mouse.mouse_l = REMOTE_KEY_STATUS_FREE;
    output_.mouse.mouse_r = REMOTE_KEY_STATUS_FREE;

    output_.keyboard.all = REMOTE_KEY_STATUS_FREE;
}

/**
 * @brief VT03数据处理函数
 * 
 */
void RemoteDjiVT03::DataProcess(uint8_t* buffer)
{
    raw_data_ = reinterpret_cast<RmoteVT03RawData const*>(buffer);

    if (raw_data_->start_of_frame_1 != 0xA9 && raw_data_->start_of_frame_2 != 0x53) {
        return;
    }

    output_.remote.pitch = K_PITCH * raw_data_->channel_2 + C_PITCH;
    output_.remote.pitch = CLAMP(output_.remote.pitch, MIN_PITCH_RADIAN, MAX_PITCH_RADIAN);

    output_.remote.chassis_y = raw_data_->channel_0;
    output_.remote.chassis_x = raw_data_->channel_1;
    output_.remote.rotation = raw_data_->channel_3;
    output_.remote.thumbwheel = raw_data_->wheel;

    output_.remote.cns = raw_data_->cns;
    output_.remote.fn1 = raw_data_->fn_1;
    output_.remote.fn2 = raw_data_->fn_2;
    output_.remote.trigger = raw_data_->trigger;
    output_.remote.pause = raw_data_->pause;

    int16_t dx = std::clamp(raw_data_->mouse_x * 30, INT16_MIN, INT16_MAX);
    int16_t dy = std::clamp(raw_data_->mouse_y * 2, INT16_MIN, INT16_MAX);
    
    output_.mouse.mouse_x = (int16_t)(1683 + 1320 * ((int16_t)dx - 32767) / 65535);
    output_.mouse.mouse_y += (float)dy / (float)INT16_MAX;
    output_.mouse.mouse_y = std::clamp(output_.mouse.mouse_y, MIN_PITCH_RADIAN, MAX_PITCH_RADIAN);
    // output_.mouse.mouse_z = (int16_t)raw_data_->mouse_z;

    output_.mouse.mouse_l = raw_data_->mouse_l;
    output_.mouse.mouse_r = raw_data_->mouse_r;

    Process_Keyboard_Toggle(&output_.keyboard, raw_data_->keyboard);
}