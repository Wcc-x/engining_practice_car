#include <stdio.h>
#include "ledc.h"
#include "pcnt_encoder.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void app_main(void)
{
    /* ============================================================
     *  第一部分：LEDC PWM 呼吸灯初始化（4路）
     * ============================================================ */
    

    // 通道0
    ledc_config_t *ledc_config = malloc(sizeof(ledc_config_t));
    ledc_config->clk_cfg = LEDC_AUTO_CLK;
    ledc_config->timer_num = LEDC_TIMER_0;
    ledc_config->freq_hz = 1000;
    ledc_config->duty_resolution = LEDC_TIMER_14_BIT;
    ledc_config->channel = LEDC_CHANNEL_0;
    ledc_config->duty = 0;
    ledc_config->gpio_num = LEDC_PWM_CHO_GPIO;

    // 通道2
    ledc_config_t *ledc_config_2 = malloc(sizeof(ledc_config_t));
    ledc_config_2->clk_cfg = LEDC_AUTO_CLK;
    ledc_config_2->timer_num = LEDC_TIMER_0;
    ledc_config_2->freq_hz = 1000;
    ledc_config_2->duty_resolution = LEDC_TIMER_14_BIT;
    ledc_config_2->channel = LEDC_CHANNEL_2;
    ledc_config_2->duty = 0;
    ledc_config_2->gpio_num = LEDC_PWM_CHO_GPIO_2;

    // 通道3
    ledc_config_t *ledc_config_3 = malloc(sizeof(ledc_config_t));
    ledc_config_3->clk_cfg = LEDC_AUTO_CLK;
    ledc_config_3->timer_num = LEDC_TIMER_1;
    ledc_config_3->freq_hz = 1000;
    ledc_config_3->duty_resolution = LEDC_TIMER_14_BIT;
    ledc_config_3->channel = LEDC_CHANNEL_3;
    ledc_config_3->duty = 0;
    ledc_config_3->gpio_num = LEDC_PWM_CHO_GPIO_3;

    // 通道4
    ledc_config_t *ledc_config_4 = malloc(sizeof(ledc_config_t));
    ledc_config_4->clk_cfg = LEDC_AUTO_CLK;
    ledc_config_4->timer_num = LEDC_TIMER_1;
    ledc_config_4->freq_hz = 1000;
    ledc_config_4->duty_resolution = LEDC_TIMER_14_BIT;
    ledc_config_4->channel = LEDC_CHANNEL_4;
    ledc_config_4->duty = 0;
    ledc_config_4->gpio_num = LEDC_PWM_CHO_GPIO_4;

    ledc_Init(ledc_config);
    ledc_Init(ledc_config_2);
    ledc_Init(ledc_config_3);
    ledc_Init(ledc_config_4);

    /* ============================================================
     *  第二部分：PCNT 霍尔编码器初始化（2路）
     * ============================================================ */

    // --- 编码器1（A/B 相正交解码） ---
    pcnt_encoder_config_t *encoder_1 = malloc(sizeof(pcnt_encoder_config_t));
    encoder_1->a_gpio_num    = PCNT_ENCODER_1_A_GPIO;   // A相 → 边沿检测
    encoder_1->b_gpio_num    = PCNT_ENCODER_1_B_GPIO;   // B相 → 方向控制
    encoder_1->pulses_per_rev = PCNT_ENCODER_PULSES_PER_REV;
    encoder_1->glitch_ns      = PCNT_ENCODER_GLITCH_NS;
    pcnt_encoder_init(encoder_1);

    // --- 编码器2（A/B 相正交解码） ---
    pcnt_encoder_config_t *encoder_2 = malloc(sizeof(pcnt_encoder_config_t));
    encoder_2->a_gpio_num    = PCNT_ENCODER_2_A_GPIO;   // A相 → 边沿检测
    encoder_2->b_gpio_num    = PCNT_ENCODER_2_B_GPIO;   // B相 → 方向控制
    encoder_2->pulses_per_rev = PCNT_ENCODER_PULSES_PER_REV;
    encoder_2->glitch_ns      = PCNT_ENCODER_GLITCH_NS;
    pcnt_encoder_init(encoder_2);

    // --- 编码器3（A/B 相正交解码） ---
    pcnt_encoder_config_t *encoder_3 = malloc(sizeof(pcnt_encoder_config_t));
    encoder_3->a_gpio_num    = PCNT_ENCODER_3_A_GPIO;   // A相 → 边沿检测
    encoder_3->b_gpio_num    = PCNT_ENCODER_3_B_GPIO;   // B相 → 方向控制
    encoder_3->pulses_per_rev = PCNT_ENCODER_PULSES_PER_REV;
    encoder_3->glitch_ns      = PCNT_ENCODER_GLITCH_NS;
    pcnt_encoder_init(encoder_3);

    // --- 编码器4（A/B 相正交解码） ---
    pcnt_encoder_config_t *encoder_4 = malloc(sizeof(pcnt_encoder_config_t));
    encoder_4->a_gpio_num    = PCNT_ENCODER_4_A_GPIO;   // A相 → 边沿检测
    encoder_4->b_gpio_num    = PCNT_ENCODER_4_B_GPIO;   // B相 → 方向控制
    encoder_4->pulses_per_rev = PCNT_ENCODER_PULSES_PER_REV;
    encoder_4->glitch_ns      = PCNT_ENCODER_GLITCH_NS;
    pcnt_encoder_init(encoder_4);

    /* ============================================================
     *  第三部分：主循环
     *   - 周期性更新编码器读数，计算速度(RPM)和角度(°)
     *   - 周期性更新PWM占空比
     * ============================================================ */
    while (1) {
        // ---------- 更新编码器读数（每隔一次循环刷新） ----------
        pcnt_encoder_update(encoder_1);
        pcnt_encoder_update(encoder_2);
        pcnt_encoder_update(encoder_3);
        pcnt_encoder_update(encoder_4);

        // ---------- 打印编码器数据 ----------
        // total_pulse_count 由 update() 写入，直接读字段避免二次读硬件
        printf("E1| total:%8lld  rpm:%6.1f  angle:%6.1f° || "
               "E2| total:%8lld  rpm:%6.1f  angle:%6.1f° || "
               "E3| total:%8lld  rpm:%6.1f  angle:%6.1f° || "
               "E4| total:%8lld  rpm:%6.1f  angle:%6.1f°\n",
               encoder_1->total_pulse_count,
               pcnt_encoder_get_speed_rpm(encoder_1),
               pcnt_encoder_get_angle_deg(encoder_1),
               encoder_2->total_pulse_count,
               pcnt_encoder_get_speed_rpm(encoder_2),
               pcnt_encoder_get_angle_deg(encoder_2),
               encoder_3->total_pulse_count,
               pcnt_encoder_get_speed_rpm(encoder_3),
               pcnt_encoder_get_angle_deg(encoder_3),
               encoder_4->total_pulse_count,
               pcnt_encoder_get_speed_rpm(encoder_4),
               pcnt_encoder_get_angle_deg(encoder_4));

        // ---------- 更新PWM占空比 ----------
        // ledc_pwm_set_duty(ledc_config, ledpwmval);

       

    /* ============================================================
     *  清理资源（正常情况下不会执行到这里，while(1) 永不退出）
     * ============================================================ */
    pcnt_encoder_deinit(encoder_1);
    pcnt_encoder_deinit(encoder_2);
    free(encoder_1);
    free(encoder_2);

    // LEDC 同理释放（此处省略 ledc_deinit，按需补充）
    free(ledc_config);
    free(ledc_config_2);
    free(ledc_config_3);
    free(ledc_config_4);
}
}
