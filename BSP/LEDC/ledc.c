#include "ledc.h"
//计算二的n次方
uint32_t ledc_duty_pow(uint32_t duty,uint32_t m,uint32_t n){
    uint32_t result=1;
    while(n--){
        result*=m;
    }
    return (result * duty)/100;//把百分比换算成计数值
}

void ledc_Init(ledc_config_t *ledc_config){
    ledc_config->duty=ledc_duty_pow(ledc_config->duty,2,ledc_config->duty_resolution);
    ledc_timer_config_t ledc_timer={
        .speed_mode=LEDC_LOW_SPEED_MODE,//高速定时器还是低速定时器
        .duty_resolution=ledc_config->duty_resolution,//配置分辨率
        .clk_cfg=ledc_config->clk_cfg, 
        .freq_hz          = ledc_config->freq_hz,     // PWM输出频率（单位Hz）
        .timer_num        = ledc_config->timer_num   // 使用的定时器编号（0-3）

    };
      ledc_timer_config(&ledc_timer);

    // 2. 配置LEDC通道参数
    ledc_channel_config_t ledc_channel = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,      // 与定时器模式保持一致
        .intr_type        = LEDC_INTR_DISABLE,        // 禁用LEDC中断
        .channel          = ledc_config->channel,     // 使用的通道编号（0-7，取决于芯片型号）
        .duty             = ledc_config->duty,         // 初始占空比（需要先换算为计数值）
        .gpio_num         = ledc_config->gpio_num,    // 绑定的GPIO引脚号
        .hpoint           = 0,                         // 计数器高点（通常设为0即可）
        .timer_sel        = ledc_config->timer_num    // 绑定到上面配置好的定时器
    };
    // 配置并设置LEDC通道
    ledc_channel_config(&ledc_channel);
}

void ledc_pwm_set_duty(ledc_config_t *ledc_config, uint16_t duty)
 {
     // 1. 把百分比占空比换算为硬件计数值
     // 公式：计数值 = 占空比百分比 × 2^分辨率 / 100
     ledc_config->duty = ledc_duty_pow(duty, 2, ledc_config->duty_resolution);
     // 2. 设置LEDC通道的新占空比（
     ledc_set_duty(LEDC_LOW_SPEED_MODE, ledc_config->channel, ledc_config->duty);
     // 3. 更新占空比配置，让设置立即生效
     ledc_update_duty(LEDC_LOW_SPEED_MODE, ledc_config->channel);
 }