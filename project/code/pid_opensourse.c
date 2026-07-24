#include "pid_opensourse.h"
//#include "tcp_echo_client.h"
//#include "image.h"
//extern TcpEchoClient my_tcp_client;

//zf_device_imu imu_dev;

// ---------------------------------------------------------
// 引脚定义 (参照 E3_04 电机驱动例程 和 E2_01 编码器例程)
// ---------------------------------------------------------
// 电机1 (左轮) —— 对应 E3 例程 MOTOR1
#define PWM_L_PIN   PWM_TIM_A0_CH0_A0    // MOTOR1_PWM → B12 的 PWM (TIM_A0 CH0)
#define DIR_L_PIN   A1                    // MOTOR1_DIR
// 电机2 (右轮) —— 对应 E3 例程 MOTOR2
#define PWM_R_PIN   PWM_TIM_A0_CH2_B12   // MOTOR2_PWM (TIM_A0 CH2)
#define DIR_R_PIN   B13                   // MOTOR2_DIR

// 编码器 (参照 E2_01 正交解码例程)
// 左编码器 —— 对应 E2 例程 ENCODER1 (TIM_G8, A26, A27)
#define ENCODER_L_TIMER     TIM_G8
#define ENCODER_L_CH1       TIMG8_ENCODER1_CH1_A26
#define ENCODER_L_CH2       TIMG8_ENCODER1_CH2_A27
// 右编码器 —— 对应 E2 例程 ENCODER2 (TIM_G9, B7, B9)
#define ENCODER_R_TIMER     TIM_G9
#define ENCODER_R_CH1       TIMG9_ENCODER1_CH1_B7
#define ENCODER_R_CH2       TIMG9_ENCODER1_CH2_B9

#define PI 3.1415926535f

int16 templ_plues = 0;
int16 tempr_plues = 0;
int16 imu_gyro_z = 0;

float left_pwm = 0.0f;
float right_pwm = 0.0f;
float output = 0.0f;
float gyro_z = 0.0f; 
float GyroOffset = 0.0f;

PID_State speed_pid_left = {0, 0, 0};
PID_State speed_pid_right = {0, 0, 0};
Turn_PID_State turn_pid = {0, 0};

// ---------------------------------------------------------
// 如果外部没有实现 Calculate_Erro_Init 函数，你可以在这里提供一个默认实现
// ---------------------------------------------------------
float Calculate_Erro_Init(void) {
    return 0.0f; // 默认返回 0
}

/********************************************************PID 初始化******************************************************/
void Pid_Init(void) {
    // 1. 初始化 GPIO 方向引脚 (参照 E3 电机例程)
    gpio_init(DIR_L_PIN, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    gpio_init(DIR_R_PIN, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    
    // 2. 初始化 PWM (参照 E3 电机例程, 频率 17KHz)
    pwm_init(PWM_L_PIN, 17000, 0);
    pwm_init(PWM_R_PIN, 17000, 0);
    
    // 3. 初始化正交编码器 (参照 E2 编码器例程)
    encoder_quad_init(ENCODER_L_TIMER, ENCODER_L_CH1, ENCODER_L_CH2);
    encoder_quad_init(ENCODER_R_TIMER, ENCODER_R_CH1, ENCODER_R_CH2);
    encoder_clear_count(ENCODER_L_TIMER);
    encoder_clear_count(ENCODER_R_TIMER);
    
    // 4. 清除所有历史数据和积分误差
    templ_plues = 0;
    tempr_plues = 0;
    left_pwm = 0.0f;
    right_pwm = 0.0f;
    
    speed_pid_left.last_error = 0;
    speed_pid_left.last_last_error = 0;
    speed_pid_right.last_error = 0;
    speed_pid_right.last_last_error = 0;
    
    turn_pid.error = 0;
    turn_pid.last_error = 0;
}
/********************************************************PID 初始化******************************************************/

/********************************************************编码器********************************************************/
void Encoder_Motor(void) 
{

  int16 encoder_l_count = encoder_get_count(ENCODER_L_TIMER);   // 读取左编码器计数值
  int16 encoder_r_count = -encoder_get_count(ENCODER_R_TIMER);  // 读取右编码器计数值 (取反匹配方向)

  float alpha = 1.0f; // 定义低通滤波器的系数
  templ_plues = (int16)(alpha * encoder_l_count + (1.0f - alpha) * templ_plues);    // 对左编码器计数值进行低通滤波
  tempr_plues = (int16)(alpha * encoder_r_count + (1.0f - alpha) * tempr_plues);   // 对右编码器计数值进行低通滤波

  encoder_clear_count(ENCODER_L_TIMER);
  encoder_clear_count(ENCODER_R_TIMER);   // 清除编码器的计数值，以便下一次计数
}
/********************************************************编码器********************************************************/


/********************************************************陀螺仪******************************************************/

void imu_init(void) {
    //imu_dev.init(); 
    gyroOffset_init();
}

float GetGyroZ() {
    //imu_gyro_z = imu_dev.get_gyro_z();  
    return (float)imu_gyro_z;
}

void gyroOffset_init(void) {
    GyroOffset = 0;
    printf("陀螺仪零偏校准中，请勿移动小车...\n");
    for (uint16_t i = 0; i < 2000; ++i) {
         GetGyroZ(); // 获取当前陀螺仪Z轴数据并更新全局变量
        GyroOffset += imu_gyro_z;
        system_delay_ms(1); 
    }
    GyroOffset /= 2000.0f;
    
    printf("校准完成!Z轴零偏: %.2f\n", GyroOffset);
}

void ICM_getValues() {
    GetGyroZ();
    // 角速度计算：(原始值 - 零偏) -> 转化为 弧度/秒 (rad/s)
    gyro_z = ((float)imu_gyro_z - GyroOffset) * PI / 16.4f/180.0f;
}
/********************************************************陀螺仪******************************************************/


/********************************************************转向环********************************************************/

float Turn_Control_Positional(float line_error) 
{
  ICM_getValues(); 
  // 如果误差为0，可能说明完美在中心，但陀螺仪阻尼还得起作用，直接 return 0 有点冒险，建议去掉这个 if
   if (line_error == 0) return 0; 

  turn_pid.last_error = turn_pid.error;
  turn_pid.error = line_error;

  // PD计算
  float P = KP_TURN * turn_pid.error;
  float D = KD_TURN * (turn_pid.error - turn_pid.last_error);
  float gyro_comp = GYRO * gyro_z; 
  
  output = P + D - gyro_comp;
  return func_limit_ab(output, -MAX_TURN_DIFF, MAX_TURN_DIFF);
}

/********************************************************转向环********************************************************/




/********************************************************速度环********************************************************/
float Speed_Control_Incremental(float target, float actual, PID_State* pid_state, float kp, float ki, float kd) 
{
  float error = target - actual;
  float delta_u = kp * (error - pid_state->last_error)
                + ki * error
                + kd * (error - 2*pid_state->last_error + pid_state->last_last_error);
  
  pid_state->last_last_error = pid_state->last_error;
  pid_state->last_error = error;
  return delta_u;
}
/********************************************************速度环********************************************************/



/********************************************************串 级*********************************************************/
void Motor_Cascade_Control(float target_speed_left,   // 独立左轮目标速度
                              float target_speed_right,  // 独立右轮目标速度
                              float line_error)     // 转向误差
{        
  // === 【新增】实时更新陀螺仪数据 ===
  // 1. 外环：位置式PD计算转向差速
  float turn_diff = Turn_Control_Positional(line_error);

  // 2. 内环：增量式PID计算电机输出
  left_pwm += Speed_Control_Incremental
    (
        target_speed_left + turn_diff, 
        templ_plues, 
        &speed_pid_left,
        KP_SPEED_LEFT, KI_SPEED_LEFT, KD_SPEED_LEFT
    );
  right_pwm += Speed_Control_Incremental
    (
        target_speed_right - turn_diff,
        tempr_plues,
        &speed_pid_right, 
        KP_SPEED_RIGHT, KI_SPEED_RIGHT, KD_SPEED_RIGHT
    );

  // PWM限幅
  left_pwm = func_limit_ab(left_pwm, -MAX_SPEED, MAX_SPEED);
  right_pwm = func_limit_ab(right_pwm, -MAX_SPEED, MAX_SPEED);

  // 设置左右轮电机方向和占空比 (参照 E3 电机例程风格)
  if (left_pwm == 0.0f) {
        pwm_set_duty(PWM_L_PIN, 0);  
    } else {
        gpio_set_level(DIR_L_PIN, left_pwm < 0.0f ? GPIO_LOW : GPIO_HIGH);
        uint16_t duty_l = (uint16_t)(left_pwm > 0.0f ? left_pwm : -left_pwm);
        if (duty_l > MOTOR_PWM_MAX)  duty_l = MOTOR_PWM_MAX;
        pwm_set_duty(PWM_L_PIN, duty_l);
    }

    // --- 右轮逻辑 ---
    if (right_pwm == 0.0f) {
        pwm_set_duty(PWM_R_PIN, 0);
    } else {
        gpio_set_level(DIR_R_PIN, right_pwm > 0.0f ? GPIO_HIGH : GPIO_LOW);
        uint16_t duty_r = (uint16_t)(right_pwm > 0.0f ? right_pwm : -right_pwm);
        if (duty_r > MOTOR_PWM_MAX) duty_r = MOTOR_PWM_MAX;
        pwm_set_duty(PWM_R_PIN, duty_r);
    }
  // 打印左编码器值和目标速度
 
  printf("%.2f,%.2f,%.2f,%.2f\r\n", (double)templ_plues,(double)tempr_plues,(double)100.0f, (double)gyro_z);
/*
  
  // 发送数据到TCP (可自定义数据发送间隔)
  static int send_interval_count = 0;
  int target_interval_ms = 100; // 目标发送间隔，单位毫秒。可根据需要修改此值
  int timer_period_ms = 10;     // 定时器中断周期，当前为 10ms

  if( ++send_interval_count >= (target_interval_ms / timer_period_ms)) {
      send_interval_count = 0;
      char send_buf[64];
      // 注意：末尾使用 \r\n，很多上位机如果没有收到 \r，会把所有数据当成一行，导致 UI 卡死闪退。
      int len = snprintf(send_buf, sizeof(send_buf), "%.2f,%.2f,%.2f,%.2f,%.2f\r\n", (double)templ_plues, (double)tempr_plues, (double)g_target_speed, (double)gyro_z, (double)line_error);
      if (len > 0) {
          my_tcp_client.send_data((const uint8_t*)send_buf, len);
      }
  }
  */
}

