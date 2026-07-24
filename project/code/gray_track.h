/*********************************************************************************************************************
* 文件名称          gray_track
* 功能描述          八路灰度循迹模块：标定、偏差、电机控制、丢线恢复、圈数计数
* 适用平台          MSPM0G3519 + GS08RA + TB6612 电机驱动
********************************************************************************************************************/

#ifndef __GRAY_TRACK_H__
#define __GRAY_TRACK_H__

#include "zf_common_headfile.h"

//===================================================================
// 电机引脚配置 (参照 pid_opensourse.c DRV8701E 单方向引脚)
//===================================================================
// ---- 电机A（左轮）----
#define MOTOR_L_PWM_PIN     PWM_TIM_A0_CH0_A0   // 左轮 PWM 引脚
#define MOTOR_L_DIR_PIN     A1                   // 左轮方向引脚

// ---- 电机B（右轮）----
#define MOTOR_R_PWM_PIN     PWM_TIM_A0_CH2_B12  // 右轮 PWM 引脚
#define MOTOR_R_DIR_PIN     B13                  // 右轮方向引脚

#define PWM_FREQ            17000                // PWM 频率 17kHz
#define MOTOR_PWM_MAX       8000                 // 电机 PWM 最大占空比

//===================================================================
// 全局变量声明
//===================================================================
extern uint16 gray_max[8];
extern uint16 gray_min[8];
extern uint8  gray_threshold;
extern float  gray_deviation;

// 电机控制
extern int16_t motor_left_pwm;
extern int16_t motor_right_pwm;
extern uint8_t motor_enable;

// 丢线与圈数
extern uint8_t  lost_line_flag;
extern uint16_t turn_count;
extern uint16_t target_turns;

// 电池电压
extern float battery_voltage;

//===================================================================
// 函数声明
//===================================================================
// ---- 灰度标定与偏差 ----
void  gray_max_min_update(void);
void  gray_max_min_reset(void);
void  gray_save_max_min_to_array(void);
void  gray_calculate_threshold(void);       // 根据 max/min 自动算阈值
float gray_calculate_deviation(void);

// ---- 电机控制 ----
void motor_init(void);
void motor_set_pwm(uint8_t motor, int16_t pwm);   // motor: 1=左轮, 2=右轮
void motor_stop(void);

// ---- 循迹控制 ----
void track_control(void);      // 偏差→差速PWM + 丢线恢复
void track_car_init(void);
void track_car_loop(void);

#endif  // __GRAY_TRACK_H__
