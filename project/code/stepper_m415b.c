
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

#include "zf_common_headfile.h"

#include "stepper_m415b.h"

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     步进电机 M415B 初始化
// 参数说明     void
// 返回参数     void
// 使用示例     stepper_m415b_init();
// 备注信息     初始化控制引脚并默认使能驱动器
//-------------------------------------------------------------------------------------------------------------------
void stepper_m415b_init (void)
{
    gpio_init(STEPPER_M415B_PUL_PIN, GPO, 0, GPO_PUSH_PULL);
    gpio_init(STEPPER_M415B_DIR_PIN, GPO, 0, GPO_PUSH_PULL);
    gpio_init(STEPPER_M415B_ENA_PIN, GPO, 0, GPO_PUSH_PULL);

    // 上电默认使能驱动器 (M415B: ENA+ 低电平 = 驱动器工作)
    stepper_m415b_set_enable(ZF_ENABLE);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     控制 M415B 使能状态
// 参数说明     enable          使能状态 ZF_ENABLE: 驱动器工作  ZF_DISABLE: 脱机/断电
// 返回参数     void
// 使用示例     stepper_m415b_set_enable(ZF_ENABLE);
// 备注信息     M415B 共阴极接法下 ENA+ 高电平使能 低电平脱机（部分版本相反）
//-------------------------------------------------------------------------------------------------------------------
void stepper_m415b_set_enable (uint8 enable)
{
    if(ZF_ENABLE == enable)
    {
        gpio_set_level(STEPPER_M415B_ENA_PIN, 1);                              // 高电平使能
    }
    else
    {
        gpio_set_level(STEPPER_M415B_ENA_PIN, 0);                              // 低电平脱机
    }
    system_delay_us(STEPPER_M415B_ENA_RESPONSE_TIME);                           // 等待光耦响应
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置步进电机旋转方向
// 参数说明     dir             旋转方向 STEPPER_M415B_DIR_CW / STEPPER_M415B_DIR_CCW
// 返回参数     void
// 使用示例     stepper_m415b_set_dir(STEPPER_M415B_DIR_CW);
// 备注信息     M415B 要求 DIR 建立时间 > 5us
//-------------------------------------------------------------------------------------------------------------------
void stepper_m415b_set_dir (stepper_m415b_dir_enum dir)
{
    if(STEPPER_M415B_DIR_CW == dir)
    {
        gpio_set_level(STEPPER_M415B_DIR_PIN, 0);
    }
    else
    {
        gpio_set_level(STEPPER_M415B_DIR_PIN, 1);
    }
    system_delay_us(STEPPER_M415B_DIR_SETUP_TIME);                              // M415B 要求 DIR 建立时间 > 5us
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     输出单个步进脉冲
// 参数说明     delay_us        脉冲半周期延时 (微秒) 决定当前转速
// 返回参数     void
// 使用示例     stepper_m415b_step_once(200);
// 备注信息     M415B 脉冲宽度要求 > 2.5us
//-------------------------------------------------------------------------------------------------------------------
void stepper_m415b_step_once (uint32 delay_us)
{
    gpio_set_level(STEPPER_M415B_PUL_PIN, 1);
    system_delay_us(delay_us);                                                  // 高电平保持时间
    gpio_set_level(STEPPER_M415B_PUL_PIN, 0);
    system_delay_us(delay_us);                                                  // 低电平保持时间
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     带梯形加减速的步进电机移动控制
// 参数说明     total_steps     总移动步数
// 参数说明     dir             旋转方向 STEPPER_M415B_DIR_CW / STEPPER_M415B_DIR_CCW
// 参数说明     min_delay_us    最高速度时的脉冲半周期 (微秒 数值越小速度越快 如 200us)
// 参数说明     max_delay_us    起始/终止速度的脉冲半周期 (微秒 数值越大速度越慢 如 1000us)
// 参数说明     accel_steps     加减速所用的步数 (如 100 步)
// 返回参数     void
// 使用示例     stepper_m415b_move(800, STEPPER_M415B_DIR_CW, 200, 1000, 200);
// 备注信息     加减速阶段使用线性变速 匀速阶段保持最高速
//-------------------------------------------------------------------------------------------------------------------
void stepper_m415b_move (uint32 total_steps, stepper_m415b_dir_enum dir, uint32 min_delay_us, uint32 max_delay_us, uint32 accel_steps)
{
    if(0 == total_steps)
    {
        return;
    }

    stepper_m415b_set_dir(dir);

    // 调整加减速步数 防止总步数不足以完成完整的加减速过程
    if(accel_steps * 2 > total_steps)
    {
        accel_steps = total_steps / 2;
    }

    uint32 speed_range = max_delay_us - min_delay_us;                           // 速度变化范围
    uint32 i = 0;
    for(i = 0; total_steps > i; i++)
    {
        uint32 current_delay = min_delay_us;

        // 1. 加速阶段: 先乘后除 避免整数截断导致速度突变
        if(i < accel_steps)
        {
            current_delay = max_delay_us - (speed_range * i) / accel_steps;
        }
        // 2. 减速阶段
        else if(i >= (total_steps - accel_steps))
        {
            uint32 decel_index = total_steps - 1 - i;
            current_delay = max_delay_us - (speed_range * decel_index) / accel_steps;
        }
        // 3. 匀速阶段: 保持 min_delay_us (最高速)

        // 发送脉冲
        stepper_m415b_step_once(current_delay);
    }
}