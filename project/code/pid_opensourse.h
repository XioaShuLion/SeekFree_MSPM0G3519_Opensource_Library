#ifndef _PID_OPENSOURSE_H_
#define _PID_OPENSOURSE_H_

#include "zf_common_headfile.h"
#include "zf_common_typedef.h"


// ---------------------------------------------------------
// 结构体定义
// ---------------------------------------------------------
typedef struct {
    float error;
    float last_error;
    float last_last_error;
} PID_State;

typedef struct {
    float error;
    float last_error;
} Turn_PID_State;

// ---------------------------------------------------------
// 全局变量声明
// ---------------------------------------------------------
extern int16 templ_plues;
extern int16 tempr_plues;
extern float gyro_z;
extern float left_pwm;
extern float right_pwm;


extern PID_State speed_pid_left;
extern PID_State speed_pid_right;
extern Turn_PID_State turn_pid;

// 如果代码中用到了未声明的 output，也补充一下
extern float output;

// ---------------------------------------------------------
// 宏定义与参数配置 (根据你实际情况微调)
// ---------------------------------------------------------

// --- 控制器参数 ---
#define KP_TURN       9.6f //12.0f
#define KD_TURN       16.0f //30.0f
#define GYRO          0.0f
#define MAX_TURN_DIFF 500.0f // 转向最大差速限制

#define KP_SPEED_LEFT  10.0f   // 增加比例增益，提升响应速度
#define KI_SPEED_LEFT  0.1f    // 保持积分增益不变
#define KD_SPEED_LEFT  0.0f    // 增加微分增益，进一步抑制振荡
#define KP_SPEED_RIGHT 10.0f   // 同步调整右侧比例增益
#define KI_SPEED_RIGHT 0.10f    // 同步调整右侧积分增益
#define KD_SPEED_RIGHT 0.0f    // 同步调整右侧微分增益
#define MAX_SPEED      5000.0f   // 保持最大速度不变

#define MOTOR_PWM_MAX 5000

float GetGyroZ();
void imu_init(void);
void gyroOffset_init(void);
void ICM_getValues();
void Pid_Init(void);
void Encoder_Motor(void);
float Turn_Control_Positional(float line_error);
float Speed_Control_Incremental(float target, float actual, PID_State* pid_state, float kp, float ki, float kd);
void Motor_Cascade_Control(float target_speed_left, float target_speed_right, float line_error);



// ---------------------------------------------------------
// 外部需要实现的函数声明 (通常在 main 或是传感器读取的地方)
// ---------------------------------------------------------
extern float Calculate_Erro_Init(void);

// 限幅函数
inline float limit_value(float value, float min, float max) {
    if (value > max) return max;
    if (value < min) return min;
    return value;
}


#endif // _PID_OPENSOURSE_H_