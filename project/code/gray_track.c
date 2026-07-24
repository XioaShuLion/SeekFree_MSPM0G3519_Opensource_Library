/*********************************************************************************************************************
* 文件名称          gray_track
* 功能描述          八路灰度循迹模块：标定、偏差计算
* 适用平台          MSPM0G3519 + GS08RA
********************************************************************************************************************/

#include "gray_track.h"

//-------------------------------------------------------------------
// 全局变量定义
//-------------------------------------------------------------------
// 灰度传感器预标定值 (黑线 min / 白线 max, RAW 空间)
// 灰度传感器预标定值 (黑线 min / 白线 max, RAW 空间, 8bit ADC 0~255)
uint16 gray_max[8] = {250, 250, 239, 247, 233, 240, 249, 216};   // 用户实测白线最大值
uint16 gray_min[8] = { 51,  59,  40,  45,  37,  42,  52,  35};   // 用户实测黑线最小值
uint8  gray_threshold = 45;       // 灰度二值化阈值 (DEAL 空间 0~100)
float  gray_deviation = 0.0f;     // 黑线偏差（负=偏左，正=偏右，0=居中）

//-------------------------------------------------------------------
// 采集并更新光电管最大最小值
//-------------------------------------------------------------------
void gray_max_min_update(void)
{
    uint8 i;
    for(i = 0; i < 8; i++)
    {
        if(gs08ra_raw_val[i] > gray_max[i]) gray_max[i] = gs08ra_raw_val[i];
        if(gs08ra_raw_val[i] < gray_min[i]) gray_min[i] = gs08ra_raw_val[i];
    }
}

//-------------------------------------------------------------------
// 重置最大最小值
//-------------------------------------------------------------------
void gray_max_min_reset(void)
{
    uint8 i;
    for(i = 0; i < 8; i++)
    {
        gray_max[i] = 0;
        gray_min[i] = 4095;
    }
}

//-------------------------------------------------------------------
// 将采集的最大最小值写入 gs08ra 标定数组
//-------------------------------------------------------------------
void gray_save_max_min_to_array(void)
{
    uint8 i;
    for(i = 0; i < 8; i++)
    {
        gs08ra_max_val[i] = gray_max[i];
        gs08ra_min_val[i] = gray_min[i];
    }
}

//-------------------------------------------------------------------
// 根据 max/min 自动计算阈值 (DEAL 归一化后 0~100, 取 45 偏黑侧)
//-------------------------------------------------------------------
void gray_calculate_threshold(void)
{
    gray_threshold = 45;
    gs08ra_set_threshold(gray_threshold);
}

//-------------------------------------------------------------------
// 计算黑线偏差（BIN 二值化重心法）
// bin_val：0=黑(不亮)，1=白(亮)
// 返回值：负=偏左，正=偏右，0=居中
//-------------------------------------------------------------------
float gray_calculate_deviation(void)
{
    int32 sum_index = 0;
    uint8 count = 0;
    uint8 i;

    for(i = 0; i < 8; i++)
    {
        if(gs08ra_bin_val[i] == 0)    // 检测到黑线
        {
            sum_index += i;
            count++;
        }
    }

    if(count == 0)
    {
        return 0.0f;    // 没检测到黑线
    }

    // 黑线中心位置，减去 3.5 得到偏差
    float position  = (float)sum_index / (float)count;
    float deviation = position - 3.5f;

    return deviation;
}

//===================================================================
// 电机控制
//===================================================================

int16_t motor_left_pwm  = 0;
int16_t motor_right_pwm = 0;
uint8_t motor_enable    = 0;    // 0=停止, 1=使能

//-------------------------------------------------------------------
// 电机初始化 (DRV8701E 单方向引脚风格，参照 pid_opensourse.c)
//-------------------------------------------------------------------
void motor_init(void)
{
    // 方向引脚初始化为输出
    gpio_init(MOTOR_L_DIR_PIN, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    gpio_init(MOTOR_R_DIR_PIN, GPO, GPIO_HIGH, GPO_PUSH_PULL);

    // PWM 初始化
    pwm_init(MOTOR_L_PWM_PIN, PWM_FREQ, 0);
    pwm_init(MOTOR_R_PWM_PIN, PWM_FREQ, 0);
}

//-------------------------------------------------------------------
// 电机设置 PWM (DRV8701E 单方向引脚: GPIO_HIGH=正转, GPIO_LOW=反转)
// motor: 1=左轮, 2=右轮, pwm: -MOTOR_PWM_MAX ~ +MOTOR_PWM_MAX
//-------------------------------------------------------------------
void motor_set_pwm(uint8_t motor, int16_t pwm)
{
    pwm_channel_enum pwm_pin;
    gpio_pin_enum     dir_pin;

    if(motor == 1)
    {
        pwm_pin = MOTOR_L_PWM_PIN;
        dir_pin = MOTOR_L_DIR_PIN;
    }
    else
    {
        pwm_pin = MOTOR_R_PWM_PIN;
        dir_pin = MOTOR_R_DIR_PIN;
    }

    // 限幅
    if(pwm >  MOTOR_PWM_MAX) pwm =  MOTOR_PWM_MAX;
    if(pwm < -MOTOR_PWM_MAX) pwm = -MOTOR_PWM_MAX;

    // 方向控制
    if(pwm > 0)
    {
        gpio_set_level(dir_pin, GPIO_HIGH);
        pwm_set_duty(pwm_pin, (uint32)pwm);
    }
    else if(pwm < 0)
    {
        gpio_set_level(dir_pin, GPIO_LOW);
        pwm_set_duty(pwm_pin, (uint32)(-pwm));
    }
    else
    {
        pwm_set_duty(pwm_pin, 0);
    }
}

//-------------------------------------------------------------------
// 电机停止
//-------------------------------------------------------------------
void motor_stop(void)
{
    motor_set_pwm(1, 0);
    motor_set_pwm(2, 0);
}

//===================================================================
// 循迹控制：偏差→差速PWM + 丢线恢复 + 圈数计数
//===================================================================

uint8_t  lost_line_flag = 0;       // 丢线标志：0=正常, 1=丢线中
uint16_t turn_count     = 0;       // 已转弯次数
uint16_t target_turns   = 0;       // 目标圈数×4

static float   last_deviation  = 0.0f;   // 丢线前最后偏差
static uint8_t turn_dir_locked = 0;      // 丢线锁定方向
static uint8_t turn_debounce   = 0;      // 转弯去抖

//-------------------------------------------------------------------
// 偏差→差速 PWM 查表（基于川鱼方案，适配 gray_deviation）
//-------------------------------------------------------------------
static int16_t deviation_to_difpwm(float dev)
{
    // dev 范围 -3.5~+3.5，映射 DifPWM -4000~+4000 (适配 MOTOR_PWM_MAX=8000)
    int16_t dif = (int16_t)(dev * 1140.0f);   // 3.5*1140≈4000

    // 限幅
    if(dif >  4000) dif =  4000;
    if(dif < -4000) dif = -4000;
    return dif;
}

//-------------------------------------------------------------------
// 循迹控制：偏差→电机差速
//-------------------------------------------------------------------
void track_control(void)
{
    int16_t ave_pwm, dif_pwm;

    // ---------- 丢线检测 ----------
    // 检查是否所有传感器都看到白色（全白=丢线）
    uint8_t all_white = 1;
    uint8_t i;
    for(i = 0; i < 8; i++)
    {
        if(gs08ra_bin_val[i] == 0) { all_white = 0; break; }   // 有黑线→未丢线
    }

    if(all_white && !lost_line_flag)
    {
        // 刚丢线：记录丢线前偏差，锁定方向
        lost_line_flag = 1;
        last_deviation = gray_deviation;

        if(turn_dir_locked == 0)
        {
            turn_dir_locked = 1;
        }
    }
    else if(!all_white && lost_line_flag)
    {
        // 恢复：计数+1，去抖
        if(turn_debounce > 5)
        {
            lost_line_flag = 0;
            turn_count++;
            turn_debounce = 0;
        }
        else
        {
            turn_debounce++;
        }
    }
    else if(!all_white)
    {
        turn_debounce = 0;
    }

    // ---------- 电机控制 ----------
    if(!motor_enable)
    {
        motor_stop();
        return;
    }

    if(lost_line_flag)
    {
        // ---- 丢线恢复：原地旋转找线 ----
        // 根据丢线前偏差方向，只朝一侧转
        float turn_dev = (turn_dir_locked && last_deviation != 0) 
                         ? last_deviation : -1.0f;

        if(turn_dev < 0)
        {
            motor_set_pwm(1, -1600);   // 左轮反转
            motor_set_pwm(2,  3600);   // 右轮正转 → 左转找线
        }
        else
        {
            motor_set_pwm(1,  3600);   // 左轮正转
            motor_set_pwm(2, -1600);   // 右轮反转 → 右转找线
        }
        return;
    }

    // ---- 正常循迹 ----
    // 更新丢线方向记录
    last_deviation = gray_deviation;
    turn_dir_locked = 0;

    // 差速 PWM（偏差越大，差速越大）
    dif_pwm = deviation_to_difpwm(gray_deviation);

    // 基础速度（偏差越大，速度越低）
    {
        float abs_dev = gray_deviation;
        if(abs_dev < 0) abs_dev = -abs_dev;
        ave_pwm = 3000 - (int16_t)(abs_dev * 500.0f);
        if(ave_pwm < 1200) ave_pwm = 1200;   // 最低速度保护
    }

    motor_left_pwm  = ave_pwm + dif_pwm;
    motor_right_pwm = ave_pwm - dif_pwm;

    // 限幅
    if(motor_left_pwm  >  MOTOR_PWM_MAX) motor_left_pwm  =  MOTOR_PWM_MAX;
    if(motor_left_pwm  < -MOTOR_PWM_MAX) motor_left_pwm  = -MOTOR_PWM_MAX;
    if(motor_right_pwm >  MOTOR_PWM_MAX) motor_right_pwm =  MOTOR_PWM_MAX;
    if(motor_right_pwm < -MOTOR_PWM_MAX) motor_right_pwm = -MOTOR_PWM_MAX;

    motor_set_pwm(1, motor_left_pwm);
    motor_set_pwm(2, motor_right_pwm);

    // ---------- 圈数到自动停止 ----------
    if(target_turns > 0 && turn_count >= target_turns)
    {
        motor_enable = 0;
        turn_count  = 0;
        motor_stop();
    }
}

//===================================================================
// 电池电压检测 (参照 E1_02 Battery ADC 例程)
//===================================================================

#define BATTERY_ADC_PIN     ADC0_CH7_A22    // 电池电压检测引脚
float battery_voltage = 0.0f;

static void battery_read(void)
{
    uint16 adc_val = adc_convert(BATTERY_ADC_PIN);
    battery_voltage = 37.0f * adc_val / 256.0f;    // 矫正系数 11.84/11.62≈1.019
}

//-------------------------------------------------------------------
// 循迹小车初始化
//-------------------------------------------------------------------
void track_car_init(void)
{
    gs08ra_init();      // 初始化八路灰度传感器
    key_init(10);       // 初始化按键（10ms 扫描周期）
    motor_init();       // 初始化电机 PWM

    // 加载预标定值到 gs08ra 库
    gray_save_max_min_to_array();
    gray_calculate_threshold();

    // 屏幕初始化 (参照 E5_04 IPS200 例程)
    ips200_set_dir(IPS200_PORTAIT);
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    ips200_init(IPS200_TYPE_SPI);
    ips200_clear();
    ips200_show_string(0, 0, "Track Ready");
}

//-------------------------------------------------------------------
// 屏幕显示更新 (参照 E5_04 IPS200 例程)
//-------------------------------------------------------------------
static void track_display_update(void)
{
    char buf[32];
    uint8 i;

    // 第1行: BIN 指示灯 (0=黑, 1=白)
    ips200_show_string(0, 16, "BIN:");
    for(i = 0; i < 8; i++)
    {
        buf[i] = gs08ra_bin_val[i] ? '0' : '1';
    }
    buf[8] = '\0';
    ips200_show_string(48, 16, buf);

    // 第2-3行: 灰度最大值 (分两行, 每行4个)
    snprintf(buf, sizeof(buf), "Mx:%4d%4d%4d%4d", gray_max[0], gray_max[1], gray_max[2], gray_max[3]);
    ips200_show_string(0, 32, buf);
    snprintf(buf, sizeof(buf), "   %4d%4d%4d%4d", gray_max[4], gray_max[5], gray_max[6], gray_max[7]);
    ips200_show_string(0, 48, buf);

    // 第4-5行: 灰度最小值
    snprintf(buf, sizeof(buf), "Mn:%4d%4d%4d%4d", gray_min[0], gray_min[1], gray_min[2], gray_min[3]);
    ips200_show_string(0, 64, buf);
    snprintf(buf, sizeof(buf), "   %4d%4d%4d%4d", gray_min[4], gray_min[5], gray_min[6], gray_min[7]);
    ips200_show_string(0, 80, buf);

    // 第7行: 偏差 + 阈值
    ips200_show_string(0, 100, "Dev:");
    ips200_show_float(48, 100, gray_deviation, 2, 2);
    ips200_show_string(128, 100, "Thr:");
    ips200_show_int(168, 100, gray_threshold, 3);

    // 第8行: 电机 PWM
    ips200_show_string(0, 116, "L:");
    ips200_show_int(24, 116, motor_left_pwm, 5);
    ips200_show_string(104, 116, "R:");
    ips200_show_int(128, 116, motor_right_pwm, 5);

    // 第9行: 状态 + 圈数
    ips200_show_string(0, 132, lost_line_flag ? "LOST" : "TRACK");
    ips200_show_string(80, 132, "Turns:");
    ips200_show_int(144, 132, turn_count, 3);

    // 第10行: 电池电压
    ips200_show_string(0, 148, "Bat:");
    ips200_show_float(48, 148, battery_voltage, 2, 2);
    ips200_show_string(120, 148, "V");
}

//-------------------------------------------------------------------
// 循迹小车主循环（每 5ms 调用一次）
//-------------------------------------------------------------------
void track_car_loop(void)
{
    static int16 cnt = 0;
    cnt++;
    gs08ra_scan_read();                             // 采集八路灰度
    gray_max_min_update();                          // 更新临时最大最小值
    gray_deviation = gray_calculate_deviation();    // 计算黑线偏差
    track_control();                                // 循迹控制（偏差→电机）

    // ---------- 串口调试输出 + 屏幕显示（每 20 次 = 100ms） ----------
    if(cnt % 10 == 0)
    {
        printf("RAW:%d,%d,%d,%d,%d,%d,%d,%d\r\n",
               gs08ra_raw_val[0], gs08ra_raw_val[1], gs08ra_raw_val[2], gs08ra_raw_val[3],
               gs08ra_raw_val[4], gs08ra_raw_val[5], gs08ra_raw_val[6], gs08ra_raw_val[7]);
        printf("DEAL:%d,%d,%d,%d,%d,%d,%d,%d\r\n",
               gs08ra_deal_val[0], gs08ra_deal_val[1], gs08ra_deal_val[2], gs08ra_deal_val[3],
               gs08ra_deal_val[4], gs08ra_deal_val[5], gs08ra_deal_val[6], gs08ra_deal_val[7]);
        printf("BIN:%d,%d,%d,%d,%d,%d,%d,%d\r\n",
               gs08ra_bin_val[0], gs08ra_bin_val[1], gs08ra_bin_val[2], gs08ra_bin_val[3],
               gs08ra_bin_val[4], gs08ra_bin_val[5], gs08ra_bin_val[6], gs08ra_bin_val[7]);
        printf("Deviation:%.2f  Threshold:%d\r\n", gray_deviation, gray_threshold);
        printf("Motor L:%d R:%d  Lost:%d  Turns:%d\r\n",
               motor_left_pwm, motor_right_pwm, lost_line_flag, turn_count);
        track_display_update();                     // 更新屏幕显示
    }
    if(cnt % 50 == 0)
    {
        battery_read();                             // 每 250ms 读取电池电压
    }
    // ---------- 按键处理 ----------
    key_scanner();

    if(key_get_state(KEY_1) == KEY_SHORT_PRESS)
    {
        gray_max_min_reset();
        printf("//==== Reset Max and Min ====//\r\n");
        system_delay_ms(500);
    }

    if(key_get_state(KEY_2) == KEY_SHORT_PRESS)
    {
        gray_save_max_min_to_array();
        gray_calculate_threshold();                         // 自动计算阈值
        printf("//==== Saved & Thr=%d ====//\r\n", gray_threshold);
        system_delay_ms(500);
    }

    if(key_get_state(KEY_3) == KEY_SHORT_PRESS)
    {
        gray_calculate_threshold();                         // 重新自动算阈值
        printf("//==== Thr=%d ====//\r\n", gray_threshold);
    }

    if(key_get_state(KEY_4) == KEY_SHORT_PRESS)
    {
        // 按键4：使能/失能电机（川鱼方式）
        motor_enable = !motor_enable;
        if(!motor_enable)
        {
            motor_stop();
            turn_count = 0;
            printf("//==== Motor Disabled ====//\r\n");
        }
        else
        {
            printf("//==== Motor Enabled ====//\r\n");
        }
        system_delay_ms(500);
    }
    system_delay_ms(5);   // 5ms 循环周期
}
