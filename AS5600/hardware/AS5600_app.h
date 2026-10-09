#ifndef __AS5600_APP_H_
#define __AS5600_APP_H_

#include "AS5600_driver.h"
#include "hal_iic.h"
#include "main.h"
#include <stdint.h>

#define AS5600_DEVICE_COUNT 1  //AS5600设备数量

typedef enum {
    AS5600_DEVICE_1 = 0,
    AS5600_DEVICE_2 = 1,
} AS5600_Device_Index_t;

typedef struct as5600_angle{
    float  relative_angle;          //AS5600实时原始角度换算值，范围[0, 360)
    volatile uint8_t  angle_flag;   //获取角度标志位
    volatile uint8_t  writing_flag; //写入标志位
}AS5600_angle_t;

typedef struct as5600_config{
    as5600_power_mode_t             power_mode;
    as5600_hysteresis_t             hysteresis;
    as5600_output_stage_t           output_stage;
    as5600_slow_filter_t            slow_filter;
    as5600_fast_filter_threshold_t  fast_filter_threshold;
    as5600_bool_t                   watchdog_enable;
} as5600_config_t;

typedef struct as5600_device{
    as5600_handle_t handle;       
    AS5600_angle_t  angle_data;   
    iic_bus_t       iic_bus;   
    as5600_config_t  config;   
    uint8_t         is_initialized; 
} as5600_device_t;
extern as5600_device_t g_as5600_devices[AS5600_DEVICE_COUNT];

/*public app function*/
/**
 * @brief 初始化所有AS5600设备
 */
void as5600_init_all(void);

/**
 * @brief 从指定AS5600设备读取RAW_ANGLE并更新滤波后的实时角度
 * @param device_index 设备索引
 */
void as5600_update_angle_data(AS5600_Device_Index_t device_index);

/**
 * @brief AS5600数据调度处理函数，在主循环中根据angle_flag更新角度缓存
 */
void AS5600_Data_Handler(void);

/**
 * @brief 获取指定AS5600设备的角度数据缓存
 * @param device_index 设备索引
 * @return 指定设备的角度数据指针
 */
AS5600_angle_t * ad5600_get_angle(AS5600_Device_Index_t device_index);

/**
 * @brief AS5600调试打印接口
 * @param fmt printf风格格式字符串
 * @param ... 可变参数
 */
void as5600_debug_print(const char *const fmt, ...);

#endif

