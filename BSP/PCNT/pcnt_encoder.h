#ifndef _PCNT_ENCODER_H
#define _PCNT_ENCODER_H

#include "driver/gpio.h"
#include "driver/pulse_cnt.h"

#ifdef __cplusplus
extern "C" {
#endif

/*   
 *  引脚 & 参数宏（按实际硬件修改）
 *    */

/* ---------- 编码器1 A/B 相 GPIO ---------- */
#define PCNT_ENCODER_1_A_GPIO        GPIO_NUM_4
#define PCNT_ENCODER_1_B_GPIO        GPIO_NUM_5

/* ---------- 编码器2 A/B 相 GPIO ---------- */
#define PCNT_ENCODER_2_A_GPIO        GPIO_NUM_6
#define PCNT_ENCODER_2_B_GPIO        GPIO_NUM_7

#define PCNT_ENCODER_3_A_GPIO        GPIO_NUM_8
#define PCNT_ENCODER_3_B_GPIO        GPIO_NUM_9

#define PCNT_ENCODER_4_A_GPIO        GPIO_NUM_10
#define PCNT_ENCODER_4_B_GPIO        GPIO_NUM_11
/* ---------- 编码器物理参数 ---------- */
#define PCNT_ENCODER_PULSES_PER_REV  11      /* 电机每转单相霍尔脉冲数          */
#define PCNT_ENCODER_GLITCH_NS       1000    /* 毛刺滤波时间（纳秒）             */

/* ---------- PCNT 累加器限幅（accum_count=true，驱动内部自动处理溢出） ---------- */
#define PCNT_ACCUM_HIGH_LIMIT        20000
#define PCNT_ACCUM_LOW_LIMIT        -20000

/* ---------- 4x 正交解码：每 AB 周期计 4 次 ---------- */
#define PCNT_4X_COUNTS_PER_REV(ppr)  (4 * (ppr))


/*   
 *  编码器实例结构体
 *
 *  使用方法：
 *    1. malloc / 静态分配
 *    2. 填 a_gpio_num, b_gpio_num, pulses_per_rev, glitch_ns
 *    3. pcnt_encoder_init()
 *    4. 主循环中周期性 pcnt_encoder_update()
 *    5. 通过 getter 读取速度、角度、脉冲数
 *
 *  设计要点：
 *    - 双通道 4x 正交解码，无需 ISR
 *    - accum_count=true，PCNT 驱动内部自动累加溢出
 *    - 直接 pcnt_unit_get_count() 读值，零中断、零临界区
 *    */
typedef struct {
    /* ===== 用户配置（init 前填写） ===== */
    int      a_gpio_num;                 /* A相 GPIO（边沿 + 电平检测）         */
    int      b_gpio_num;                 /* B相 GPIO（边沿 + 电平检测）         */
    int      pulses_per_rev;             /* 电机每转单相脉冲数                 */
    uint32_t glitch_ns;                  /* 毛刺滤波时间（纳秒）               */

    /* ===== 硬件句柄（init 后自动填充） ===== */
    pcnt_unit_handle_t    pcnt_unit;     /* PCNT 单元句柄                      */
    pcnt_channel_handle_t pcnt_chan_a;   /* A 相通道句柄                       */
    pcnt_channel_handle_t pcnt_chan_b;   /* B 相通道句柄（4x 解码互补通道）     */

    /* ===== 运行时状态（update 后更新，用户只读） ===== */
    int64_t total_pulse_count;           /* 当前累积脉冲数（正转+ / 反转-）    */
    float   speed_rpm;                   /* 当前转速 RPM（正=正转，负=反转）    */
    float   position_deg;                /* 当前角度 0.0 ~ 360.0°              */

    /* ===== 内部状态（库维护，用户勿改） ===== */
    int64_t last_update_us;              /* 上次更新时间戳（微秒）             */
    int32_t last_pulse_count;            /* 上次脉冲数（计算 delta 用）         */
} pcnt_encoder_config_t;


/*   
 *  API
 *    */

/** 初始化双通道 4x 正交解码，accum_count 自动处理溢出，无 ISR */
void    pcnt_encoder_init(pcnt_encoder_config_t *enc);

/** 读当前累积脉冲数（正转+ / 反转-），同时更新 enc->total_pulse_count */
int64_t pcnt_encoder_get_pulse_count(pcnt_encoder_config_t *enc);

/** 周期性更新转速(RPM)和角度(°)，建议 10~100ms 间隔 */
void    pcnt_encoder_update(pcnt_encoder_config_t *enc);

/** 获取最近一次 update 的转速 (RPM) */
float   pcnt_encoder_get_speed_rpm(const pcnt_encoder_config_t *enc);

/** 获取最近一次 update 的角度 (0~360°) */
float   pcnt_encoder_get_angle_deg(const pcnt_encoder_config_t *enc);

/** 脉冲计数归零（硬件 + 软件同步清零） */
void    pcnt_encoder_reset(pcnt_encoder_config_t *enc);

/** 反初始化，释放 PCNT 硬件资源 */
void    pcnt_encoder_deinit(pcnt_encoder_config_t *enc);

#ifdef __cplusplus
}
#endif

#endif /* _PCNT_ENCODER_H */
