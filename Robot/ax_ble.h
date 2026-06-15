/**
 * @file    ax_ble.h
 * @brief   BLE 蓝牙接收模块 — 适配江协科技蓝牙串口微信小程序
 *
 * 基于 NimBLE 实现 Nordic UART Service (NUS) 外设角色。
 * 小程序作为 GATT Client 连接后，通过 NUS RX Characteristic 写入数据包，
 * ESP32 解析数据包并提取控制指令（摇杆/滑杆/按键）。
 *
 * 数据包格式（江协科技小程序规定）:
 *   [joystick,lx,ly,rx,ry]  或 [j,lx,ly,rx,ry]      摇杆
 *   [slider,name,value]      或 [s,name,value]        滑杆
 *   [key,name,down/up]       或 [k,name,d/u]          按键
 *   [display,x,y,text,size]  或 [d,x,y,text,size]     显示屏 (暂不处理)
 *   [plot,val1,val2,...]     或 [p,val1,val2,...]     绘图   (暂不处理)
 *
 * NUS UUID (标准 Nordic UART Service):
 *   Service:      6E400001-B5A3-F393-E0A9-E50E24DCCA9E
 *   RX (phone→ESP): 6E400002-B5A3-F393-E0A9-E50E24DCCA9E  (Write)
 *   TX (ESP→phone): 6E400003-B5A3-F393-E0A9-E50E24DCCA9E  (Notify)
 */

#ifndef __AX_BLE_H
#define __AX_BLE_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 *  解析结果结构体
 * ============================================================ */

/** 摇杆数据 */
typedef struct {
    bool    valid;          /* 是否收到有效数据 */
    int16_t left_x;         /* 左摇杆横向值 */
    int16_t left_y;         /* 左摇杆纵向值 */
    int16_t right_x;        /* 右摇杆横向值 */
    int16_t right_y;        /* 右摇杆纵向值 */
} AX_BLE_Joystick;

/** 滑杆数据 */
typedef struct {
    bool    valid;
    int16_t name;           /* 滑杆名称/编号 */
    int16_t value;          /* 滑杆当前值 */
} AX_BLE_Slider;

/** 按键数据 */
typedef struct {
    bool    valid;
    int16_t name;           /* 按键名称/编号 */
    bool    is_down;        /* true=按下, false=松开 */
} AX_BLE_Key;

/* ============================================================
 *  全局数据（实时更新）
 * ============================================================ */
extern AX_BLE_Joystick  ax_ble_joystick;
extern AX_BLE_Slider    ax_ble_slider;
extern AX_BLE_Key       ax_ble_key;
extern bool              ax_ble_connected;    /* 蓝牙连接状态 */
extern uint8_t           ax_ble_rx_buf[];     /* 接收缓冲区 (调试用) */
extern uint16_t          ax_ble_rx_len;       /* 接收数据长度 */

/* ============================================================
 *  API
 * ============================================================ */

/**
 * @brief  初始化 BLE — 启动 NimBLE 主机栈 + GATT Server 广播
 * @param  device_name  蓝牙广播名称 (如 "XTARK-ESP32")
 */
void AX_BLE_Init(const char *device_name);

/**
 * @brief  向已连接的手机发送数据 (通过 NUS TX Notify)
 * @param  data  数据指针
 * @param  len   数据长度
 * @return 发送成功返回 0，未连接返回 -1
 */
int AX_BLE_Send(const uint8_t *data, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* __AX_BLE_H */
