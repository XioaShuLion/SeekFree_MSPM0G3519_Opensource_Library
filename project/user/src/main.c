#include "zf_common_headfile.h"
#include "gray_track.h"
#include "menu_ui.h"
// 打开新的工程或者工程移动了位置务必执行以下操作
// 第一步 关闭上面所有打开的文件
// 第二步 project->clean  等待下方进度条走完

// **************************** 代码区域 ****************************
int main (void)
{
    clock_init(SYSTEM_CLOCK_80M);                                               // 时钟配置及系统初始化<务必保留>
    debug_init();                                                               // 调试串口信息初始化

    // 此处编写用户代码 例如外设初始化代码等
    track_car_init();                                                           // 初始化传感器 + 电机
    motor_enable = 0;                                                           // 使能电机标志位初始化为 0（停止）

    menu_ui_init();                                                             // 初始化屏幕 + 按键 + 菜单

    while(true)
    {
        track_car_loop();                                                       // 循迹控制（传感器→电机）
        menu_ui_run();                                                          // 菜单 UI（非堵塞按键检测 + 显示）
    }
}

// **************************** 代码区域 ****************************
