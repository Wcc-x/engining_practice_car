#include <stdio.h>
#include "ledc.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
void app_main(void)
{
  uint8_t dir = 1;                // 占空比变化方向：1=增加，0=减少
    uint16_t ledpwmval = 0;         // 占空比变量（百分比形式，0-100）

    // 1. 动态分配LEDC配置结构体
    ledc_config_t *ledc_config = malloc(sizeof(ledc_config_t));

    // 2. 初始化LEDC配置参数
    ledc_config->clk_cfg = LEDC_AUTO_CLK;          // 自动选择时钟源
    ledc_config->timer_num = LEDC_TIMER_0;         // 使用定时器0
    ledc_config->freq_hz = 1000;                   // PWM频率设为1kHz
    ledc_config->duty_resolution = LEDC_TIMER_14_BIT; // 14位分辨率（最大占空比计数值2^14=16384）
    ledc_config->channel = LEDC_CHANNEL_0;        // 使用通道0
    ledc_config->duty = 0;                         // 初始占空比为0%
    ledc_config->gpio_num = LEDC_PWM_CHO_GPIO;     // 绑定到指定GPIO引脚


     ledc_config_t *ledc_config_2 = malloc(sizeof(ledc_config_t));
      ledc_config->clk_cfg = LEDC_AUTO_CLK;          
    ledc_config->timer_num = LEDC_TIMER_0;         
    ledc_config->freq_hz = 1000;                   
    ledc_config->duty_resolution = LEDC_TIMER_14_BIT; 
    ledc_config->channel = LEDC_CHANNEL_2;        
    ledc_config->duty = 0;                         
    ledc_config->gpio_num = LEDC_PWM_CHO_GPIO_2;    
    
    

      ledc_config_t *ledc_config_3 = malloc(sizeof(ledc_config_t));
      ledc_config->clk_cfg = LEDC_AUTO_CLK;          
    ledc_config->timer_num = LEDC_TIMER_1;         
    ledc_config->freq_hz = 1000;                   
    ledc_config->duty_resolution = LEDC_TIMER_14_BIT; 
    ledc_config->channel = LEDC_CHANNEL_3;        
    ledc_config->duty = 0;                         
    ledc_config->gpio_num = LEDC_PWM_CHO_GPIO_3;

    ledc_config_t *ledc_config_4 = malloc(sizeof(ledc_config_t));
      ledc_config->clk_cfg = LEDC_AUTO_CLK;          
    ledc_config->timer_num = LEDC_TIMER_1;         
    ledc_config->freq_hz = 1000;                   
    ledc_config->duty_resolution = LEDC_TIMER_14_BIT; 
    ledc_config->channel = LEDC_CHANNEL_4;        
    ledc_config->duty = 0;                         
    ledc_config->gpio_num = LEDC_PWM_CHO_GPIO_4;
    // 3. 调用初始化函数，配置定时器和通道
    ledc_init(ledc_config);
    ledc_init(ledc_config_2);
    ledc_init(ledc_config_3);
    ledc_init(ledc_config_4);
    while(1){
        vTaskDelay(50 / portTICK_PERIOD_MS);  // 延时50ms，控制呼吸灯变化速度
         
        // 更新PWM占空比
        ledc_pwm_set_duty(ledc_config, ledpwmval);
    }
}
