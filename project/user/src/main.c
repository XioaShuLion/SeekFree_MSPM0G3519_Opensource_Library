#include "zf_common_headfile.h"
//#include "pid_opensourse.h"
#include "gray_track.h"
// 打开新的工程或者工程移动了位置务必执行以下操作
// 第一步 关闭上面所有打开的文件
// 第二步 project->clean  等待下方进度条走完

// **************************** 代码区域 ****************************
//#define PIT_CH                  (PIT_TIM_G12)                                   // 使用的周期中断编号 需与 isr.c 中 TIMG12_IRQHandler 对应
//#define PIT_PRIORITY            (TIMG12_INT_IRQn)                               // 对应周期中断的中断编号

//void pit_handler(uint32 state, void *ptr);

int main (void)
{
    clock_init(SYSTEM_CLOCK_80M);                                               // 时钟配置及系统初始化<务必保留>
    debug_init();                                                               // 调试串口信息初始化
    
    // 此处编写用户代码 例如外设初始化代码等
    track_car_init();                                                           
    motor_enable = 0;                                                           // 使能电机标志位初始化为 0（停止）
    //pit_ms_init(PIT_CH, 10, pit_handler, NULL);                                // 初始化 PIT 10ms 周期中断
    //interrupt_set_priority(PIT_PRIORITY, 0);                                   // 设置中断优先级为最高
    // 此处编写用户代码 例如外设初始化代码等

    while(true)
    {
    track_car_loop();                                // 循迹小车主循环（每 5ms 调用一次）

    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     PIT 10ms 周期中断回调函数 由 TIMG12_IRQHandler 调用
// 参数说明     state               触发中断的事件
// 参数说明     *ptr                回调参数指针
// 返回参数     void
//-------------------------------------------------------------------------------------------------------------------
//void pit_handler(uint32 state, void *ptr)
//{
//                                                              
//}
// **************************** 代码区域 ****************************
