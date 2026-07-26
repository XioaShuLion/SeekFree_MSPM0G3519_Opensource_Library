/*********************************************************************************************************************
* 文件名称          menu_ui
* 描述              非堵塞按键检测 + 菜单状态机 + IPS200 界面渲染（三合一模块）
* 适用平台          MSPM0G3519 + IPS200 320x240 + 4 按键
* 备注              不可重入，仅在主循环中调用 menu_ui_run()
********************************************************************************************************************/

#ifndef _menu_ui_h_
#define _menu_ui_h_

#include "zf_common_headfile.h"
#include "zf_device_key.h"
#include "zf_device_ips200.h"

//===================================================================
// 外部引用（来自 gray_track.h 的可调参数与标定函数）
//===================================================================
extern uint8_t  gray_threshold;         // 循迹二值化阈值 0~100
extern uint16_t target_turns;           // 目标圈数×4
extern uint8_t  motor_enable;           // 电机使能标志
extern float    gray_deviation;         // 当前偏差
extern int16_t  motor_left_pwm;         // 左轮 PWM
extern int16_t  motor_right_pwm;        // 右轮 PWM
extern uint16_t turn_count;             // 已转弯次数
extern uint8_t  lost_line_flag;         // 丢线标志

// 灰度标定函数
extern void gray_max_min_reset(void);
extern void gray_save_max_min_to_array(void);
extern void gray_calculate_threshold(void);
extern void motor_stop(void);

// 灰度标定数据（实时显示用）
extern uint16 gray_max[8];
extern uint16 gray_min[8];

// 菜单可调参数（定义在 menu_ui.c）
extern uint16_t g_base_speed;           // 基础速度
extern uint16_t g_lost_recovery_speed;  // 丢线恢复速度

//===================================================================
// 菜单项类型
//===================================================================
typedef enum
{
    ITEM_UINT8,         // uint8_t  参数
    ITEM_UINT16,        // uint16_t 参数
    ITEM_INT16,         // int16_t  参数
    ITEM_FLOAT,         // float   参数（精度 0.1）
    ITEM_ACTION,        // 动作项（如"恢复默认"）
} menu_item_type_t;

//===================================================================
// 单个菜单项
//===================================================================
typedef struct
{
    const char      *name;              // 显示名称（建议 ≤10 字符）
    menu_item_type_t type;              // 数据类型
    void            *value_ptr;         // 指向实际变量的指针
    int16_t          min;               // 最小值（float 类型：实际值×10）
    int16_t          max;               // 最大值（float 类型：实际值×10）
    int16_t          step;              // 步进
    const char      *unit;              // 单位字符串
} menu_item_t;

//===================================================================
// 页面状态
//===================================================================
typedef enum
{
    PAGE_MAIN,          // 主信息页（循迹状态总览）
    PAGE_MENU,          // 菜单列表页
    PAGE_SETTING,       // 参数编辑页
    PAGE_CALIBRATE,     // 灰度传感器校准页
} page_t;

//===================================================================
// 公开 API
//===================================================================
void menu_ui_init(void);                                    // 初始化菜单（会初始化按键和屏幕）
void menu_ui_run(void);                                     // 非堵塞运行（每 5ms 主循环调用一次）

#endif
