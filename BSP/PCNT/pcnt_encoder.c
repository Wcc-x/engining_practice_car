#include "pcnt_encoder.h"
#include "esp_timer.h"

/*   
 *  初始化 — 双通道 4x 正交解码
 *
 *  通道A: edge=MA, level=MB
 *          A↓→-1  A↑→+1 （B=HIGH保持 / B=LOW翻转）
 *  通道B: edge=MB, level=MA  （与A互补，构成4x解码）
 *          B↓→+1  B↑→-1 （A=HIGH保持 / A=LOW翻转）
 *
 *  正转 (A超前B) 每AB周期: A↓(+1) + B↑(+1) + A↑(+1) + B↓(+1) = +4
 *  反转 (B超前A) 每AB周期: A↓(-1) + B↑(-1) + A↑(-1) + B↓(-1) = -4
 *
 *  accum_count=true → 驱动内部自动累加溢出，无需 ISR
 *    */
void pcnt_encoder_init(pcnt_encoder_config_t *enc)
{
    /* ---------- 1. 创建 PCNT 单元（accum_count 自动处理溢出） ---------- */
    pcnt_unit_config_t unit_config = {
        .high_limit = PCNT_ACCUM_HIGH_LIMIT,
        .low_limit  = PCNT_ACCUM_LOW_LIMIT,
        .flags.accum_count = 1,     /* 驱动内部 ISR 自动累加，用户无感知 */
    };
    pcnt_new_unit(&unit_config, &enc->pcnt_unit);

    /* ---------- 2. 毛刺滤波器 ---------- */
    pcnt_glitch_filter_config_t filter_cfg = {
        .max_glitch_ns = enc->glitch_ns,
    };
    pcnt_unit_set_glitch_filter(enc->pcnt_unit, &filter_cfg);

    /* ---------- 3. 通道A：边沿=MA，电平=MB ---------- */
    pcnt_chan_config_t chan_a_cfg = {
        .edge_gpio_num  = enc->a_gpio_num,
        .level_gpio_num = enc->b_gpio_num,
    };
    pcnt_new_channel(enc->pcnt_unit, &chan_a_cfg, &enc->pcnt_chan_a);
    pcnt_channel_set_edge_action(enc->pcnt_chan_a,
        PCNT_CHANNEL_EDGE_ACTION_DECREASE,   /* A↓ → -1 */
        PCNT_CHANNEL_EDGE_ACTION_INCREASE);  /* A↑ → +1 */
    pcnt_channel_set_level_action(enc->pcnt_chan_a,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP,      /* B=HIGH → 保持   */
        PCNT_CHANNEL_LEVEL_ACTION_INVERSE);  /* B=LOW  → 翻转  */

    /* ---------- 4. 通道B：边沿=MB，电平=MA（与A互补） ---------- */
    pcnt_chan_config_t chan_b_cfg = {
        .edge_gpio_num  = enc->b_gpio_num,
        .level_gpio_num = enc->a_gpio_num,
    };
    pcnt_new_channel(enc->pcnt_unit, &chan_b_cfg, &enc->pcnt_chan_b);
    pcnt_channel_set_edge_action(enc->pcnt_chan_b,
        PCNT_CHANNEL_EDGE_ACTION_INCREASE,   /* B↓ → +1 */
        PCNT_CHANNEL_EDGE_ACTION_DECREASE);  /* B↑ → -1 */
    pcnt_channel_set_level_action(enc->pcnt_chan_b,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP,      /* A=HIGH → 保持   */
        PCNT_CHANNEL_LEVEL_ACTION_INVERSE);  /* A=LOW  → 翻转  */

    /* ---------- 5. 启用 & 清零 & 启动 ---------- */
    pcnt_unit_enable(enc->pcnt_unit);
    pcnt_unit_clear_count(enc->pcnt_unit);
    pcnt_unit_start(enc->pcnt_unit);

    /* ---------- 6. 运行时状态初始化 ---------- */
    enc->total_pulse_count = 0;
    enc->speed_rpm         = 0.0f;
    enc->position_deg      = 0.0f;
    enc->last_update_us    = esp_timer_get_time();
    enc->last_pulse_count  = 0;
}

/*   
 *  读累积脉冲数（正转+ / 反转-）
 *
 *  accum_count=true 下，pcnt_unit_get_count 直接返回累加后的
 *  32 位有符号值，无需任何溢出修正。
 *    */
int64_t pcnt_encoder_get_pulse_count(pcnt_encoder_config_t *enc)
{
    int count = 0;
    pcnt_unit_get_count(enc->pcnt_unit, &count);
    enc->total_pulse_count = (int64_t)count;
    return enc->total_pulse_count;
}

/*   
 *  更新转速(RPM)和角度(°)
 *
 *  4x 解码：每 AB 周期计 4 次 → 每转计数 = 4 × pulses_per_rev
 *
 *  RPM = (Δ脉冲 / 每转计数) / (Δ微秒 / 60,000,000)
 *  角度 = (总脉冲 % 每转计数) × 360° / 每转计数
 *    */
void pcnt_encoder_update(pcnt_encoder_config_t *enc)
{
    /* ---------- 1. 读当前脉冲 ---------- */
    int64_t total_count = pcnt_encoder_get_pulse_count(enc);

    /* ---------- 2. 时间差（分钟） ---------- */
    int64_t now_us    = esp_timer_get_time();
    float   delta_min = (float)(now_us - enc->last_update_us) / 60000000.0f;

    /* ---------- 3. 脉冲差 ---------- */
    int32_t delta_pulse = (int32_t)(total_count - enc->last_pulse_count);

    /* ---------- 4. 转速 ---------- */
    if (delta_min > 0.0f && enc->pulses_per_rev > 0) {
        int   cpr = PCNT_4X_COUNTS_PER_REV(enc->pulses_per_rev);
        float rev = (float)delta_pulse / (float)cpr;
        enc->speed_rpm = rev / delta_min;
    } else {
        enc->speed_rpm = 0.0f;
    }

    /* ---------- 5. 角度 0~360° ---------- */
    int     cpr        = PCNT_4X_COUNTS_PER_REV(enc->pulses_per_rev);
    int64_t normalized = total_count % (int64_t)cpr;
    if (normalized < 0) {
        normalized += cpr;
    }
    enc->position_deg = (float)normalized * 360.0f / (float)cpr;

    /* ---------- 6. 保存 ---------- */
    enc->last_pulse_count = (int32_t)total_count;
    enc->last_update_us   = now_us;
}

/*   
 *  Getter
 *    */
float pcnt_encoder_get_speed_rpm(const pcnt_encoder_config_t *enc)
{
    return enc->speed_rpm;
}

float pcnt_encoder_get_angle_deg(const pcnt_encoder_config_t *enc)
{
    return enc->position_deg;
}

/*   
 *  归零
 *    */
void pcnt_encoder_reset(pcnt_encoder_config_t *enc)
{
    pcnt_unit_clear_count(enc->pcnt_unit);
    enc->total_pulse_count = 0;
    enc->last_pulse_count  = 0;
    enc->speed_rpm         = 0.0f;
    enc->position_deg      = 0.0f;
    enc->last_update_us    = esp_timer_get_time();
}

/*   
 *  反初始化
 *    */
void pcnt_encoder_deinit(pcnt_encoder_config_t *enc)
{
    if (enc->pcnt_unit) {
        pcnt_unit_stop(enc->pcnt_unit);
        pcnt_unit_disable(enc->pcnt_unit);
        pcnt_del_channel(enc->pcnt_chan_a);
        pcnt_del_channel(enc->pcnt_chan_b);
        pcnt_del_unit(enc->pcnt_unit);
        enc->pcnt_unit   = NULL;
        enc->pcnt_chan_a = NULL;
        enc->pcnt_chan_b = NULL;
    }
}
