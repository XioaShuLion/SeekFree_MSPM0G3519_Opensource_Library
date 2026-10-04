
/*********************************************************************************************************************
* 接线定义：
*                   ------------------------------------
*                   模块管脚            单片机管脚
*                   PUL+                查看 STEPPER_M415B_PUL_PIN 宏定义
*                   DIR+                查看 STEPPER_M415B_DIR_PIN 宏定义
*                   ENA+                查看 STEPPER_M415B_ENA_PIN 宏定义
*                   电源                 5V / 24V（根据电机选择）
*                   GND                 共地
*                   ------------------------------------
********************************************************************************************************************/

#ifndef _stepper_m415b_h_
#define _stepper_m415b_h_

#include "zf_common_headfile.h"

#include "zf_driver_delay.h"
#include "zf_driver_gpio.h"

// 定义步进电机控制引脚 用户可根据实际接线修改
#define STEPPER_M415B_PUL_PIN                   ( B3  )                         // 脉冲信号引脚
#define STEPPER_M415B_DIR_PIN                   ( B4  )                         // 方向信号引脚（避开SWD调试口）
#define STEPPER_M415B_ENA_PIN                   ( A12  )                         // 使能信号引脚

// M415B 驱动器时序要求 (微秒)
#define STEPPER_M415B_DIR_SETUP_TIME            ( 5    )                        // DIR 信号建立时间 > 5us
#define STEPPER_M415B_ENA_RESPONSE_TIME         ( 10   )                        // ENA 信号响应时间

typedef enum
{
    STEPPER_M415B_DIR_CW    = 0,                                                // 正转方向
    STEPPER_M415B_DIR_CCW   = 1,                                                // 反转方向
}stepper_m415b_dir_enum;

void                stepper_m415b_init           (void);
void                stepper_m415b_set_enable     (uint8 enable);
void                stepper_m415b_set_dir        (stepper_m415b_dir_enum dir);
void                stepper_m415b_step_once      (uint32 delay_us);
void                stepper_m415b_move           (uint32 total_steps, stepper_m415b_dir_enum dir, uint32 min_delay_us, uint32 max_delay_us, uint32 accel_steps);

#endif