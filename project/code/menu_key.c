/*********************************************************************************************************************
* 文件名称          menu_key
* 描述              非堵塞按键检测模块（用于菜单系统）
* 备注              基于 zf_device_key 封装，不可重入，仅在主循环中调用
********************************************************************************************************************/

#include "menu_key.h"

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     非堵塞按键检测
// 参数说明     press_type      输出参数，记录按键类型（短按 / 长按）
// 返回参数     key_index_enum  按下的按键索引，如果当前无按键动作则返回 KEY_NUMBER
// 使用示例     
//              key_state_enum type;
//              key_index_enum key = menu_key_detect(&type);
//              if(KEY_NUMBER != key)
//              {
//                  if(KEY_SHORT_PRESS == type) { ... }  // 短按处理
//                  if(KEY_LONG_PRESS  == type) { ... }  // 长按处理
//              }
// 备注信息     该函数非堵塞，调用后立即返回；每次调用只返回一个按键事件
//              key_scanner() 需要以固定周期被调用（放在 PIT 中断或主循环中）
//              本函数读取到按键事件后会自动清除对应按键状态，避免重复触发
//-------------------------------------------------------------------------------------------------------------------
key_index_enum menu_key_detect(key_state_enum *press_type)
{
    uint8 i;

    // 遍历所有按键，检查是否有新事件
    for(i = 0; i < KEY_NUMBER; i++)
    {
        key_state_enum state = key_get_state((key_index_enum)i);

        if(KEY_SHORT_PRESS == state || KEY_LONG_PRESS == state)
        {
            // 输出按键类型
            if(NULL != press_type)
            {
                *press_type = state;
            }

            // 清除该按键状态，确保每次按下只响应一次
            key_clear_state((key_index_enum)i);

            return (key_index_enum)i;
        }
    }

    // 无按键事件
    return KEY_NUMBER;
}
