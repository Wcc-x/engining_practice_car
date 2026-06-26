#ifndef _LEDC_H
#define _LEDC_H
#include "driver/gpio.h"
#include "driver/ledc.h"

#define LEDC_PWM_TIMER LEDC_TIMER_0
#define LEDC_PWM_CHO_GPIO  GPIO_NUM_23   /* 原 GPIO1=UART0_TX 冲突，换至 GPIO23 */
#define LEDC_PWM_CHANNEL  LEDC_CHANNEL_0


#define LEDC_PWM_CHANNEL_2  LEDC_CHANNEL_2
#define LEDC_PWM_CHO_GPIO_2  GPIO_NUM_2

#define LEDC_PWM_TIMER_1 LEDC_TIMER_1
#define LEDC_PWM_CHANNEL_3  LEDC_CHANNEL_3
#define LEDC_PWM_CHO_GPIO_3  GPIO_NUM_26   /* 原 GPIO10=Flash 冲突，换至 GPIO26 */

#define LEDC_PWM_CHANNEL_4  LEDC_CHANNEL_4
#define LEDC_PWM_CHO_GPIO_4  GPIO_NUM_27   /* 原 GPIO11=Flash 冲突，换至 GPIO27 *//*
*--------------------------------
这一部分根据布线合不合理可以更改
------------------------------------
*/
typedef struct{
    ledc_clk_cfg_t clk_cfg;
    ledc_timer_t timer_num;
    uint32_t freq_hz;
    ledc_timer_bit_t duty_resolution;
    ledc_channel_t channel;
    uint32_t duty;
    int gpio_num;
}ledc_config_t;
uint32_t ledc_duty_pow(uint32_t duty,uint32_t m,uint32_t n);
void ledc_Init(ledc_config_t *ledc_config);
void ledc_pwm_set_duty(ledc_config_t *ledc_config, uint16_t duty);
#endif