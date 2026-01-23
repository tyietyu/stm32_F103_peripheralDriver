#include "AS5600_app.h"
#include <stdarg.h>
#include <stdio.h>
#include <math.h>

extern UART_HandleTypeDef huart5;

// 设备1的回调
static uint8_t as5600_init_1(void)
{
    return IICInit(&g_as5600_devices[AS5600_DEVICE_1].iic_bus);
}
static uint8_t as5600_deinit_1(void)
{
    return IICDeinit(&g_as5600_devices[AS5600_DEVICE_1].iic_bus);
}
static uint8_t as5600_read_1(uint8_t addr, uint8_t reg, uint8_t *data, uint16_t len) 
{ 
    return IIC_Read_Multi_Byte(&g_as5600_devices[AS5600_DEVICE_1].iic_bus, addr, reg, data, len); 
}
static uint8_t as5600_write_1(uint8_t addr, uint8_t reg, uint8_t *data, uint16_t len)
{ 
    return IIC_Write_Multi_Byte(&g_as5600_devices[AS5600_DEVICE_1].iic_bus, addr, reg, data, len); 
}

// 设备2的回调
static uint8_t as5600_init_2(void) 
{ 
    return IICInit(&g_as5600_devices[AS5600_DEVICE_2].iic_bus); 
}
static uint8_t as5600_deinit_2(void) 
{ 
    return IICDeinit(&g_as5600_devices[AS5600_DEVICE_2].iic_bus); 
}
static uint8_t as5600_read_2(uint8_t addr, uint8_t reg, uint8_t *data, uint16_t len) 
{ 
    return IIC_Read_Multi_Byte(&g_as5600_devices[AS5600_DEVICE_2].iic_bus, addr, reg, data, len); 
}
static uint8_t as5600_write_2(uint8_t addr, uint8_t reg, uint8_t *data, uint16_t len) 
{ 
    return IIC_Write_Multi_Byte(&g_as5600_devices[AS5600_DEVICE_2].iic_bus, addr, reg, data, len); 
}

// 设备3的回调
static uint8_t as5600_init_3(void) 
{ 
    return IICInit(&g_as5600_devices[AS5600_DEVICE_3].iic_bus); 
}
static uint8_t as5600_deinit_3(void) 
{ 
    return IICDeinit(&g_as5600_devices[AS5600_DEVICE_3].iic_bus); 
}
static uint8_t as5600_read_3(uint8_t addr, uint8_t reg, uint8_t *data, uint16_t len) 
{ 
    return IIC_Read_Multi_Byte(&g_as5600_devices[AS5600_DEVICE_3].iic_bus, addr, reg, data, len); 
}
static uint8_t as5600_write_3(uint8_t addr, uint8_t reg, uint8_t *data, uint16_t len) 
{ 
    return IIC_Write_Multi_Byte(&g_as5600_devices[AS5600_DEVICE_3].iic_bus, addr, reg, data, len); 
}

as5600_device_t g_as5600_devices[AS5600_DEVICE_COUNT] = {
    [AS5600_DEVICE_1] = {
        .iic_bus = {
            .IIC_SDA_PORT = GPIOB, 
            .IIC_SDA_PIN = GPIO_PIN_7, 
            .IIC_SCL_PORT = GPIOB, 
            .IIC_SCL_PIN = GPIO_PIN_6,
        },
        .handle = {
            .iic_init = as5600_init_1, 
            .iic_deinit = as5600_deinit_1, 
            .iic_read = as5600_read_1, 
            .iic_write = as5600_write_1, 
            .delay_ms = HAL_Delay, 
            .debug_print = as5600_debug_print},
        .angle_data = {
            .zero_position = 0, 
            .stop_position = 4095}, 
        .config ={
            .power_mode            = AS5600_POWER_MODE_NOM,
            .hysteresis            = AS5600_HYSTERESIS_OFF,
            .output_stage          = AS5600_OUTPUT_STAGE_PWM,
            .slow_filter           = AS5600_SLOW_FILTER_2X,
            .fast_filter_threshold = AS5600_FAST_FILTER_THRESHOLD_6LSB,
            .watchdog_enable       = AS5600_BOOL_FALSE,
        },
        .is_initialized = 0
    },
    [AS5600_DEVICE_2] = {
        .iic_bus = {
            .IIC_SDA_PORT = GPIOD,
            .IIC_SDA_PIN = GPIO_PIN_6, 
            .IIC_SCL_PORT = GPIOD,
            .IIC_SCL_PIN = GPIO_PIN_5
        }, 
        .handle = {
            .iic_init = as5600_init_2,
            .iic_deinit = as5600_deinit_2,
            .iic_read = as5600_read_2,
            .iic_write = as5600_write_2, 
            .delay_ms = HAL_Delay, 
            .debug_print = as5600_debug_print}, 
        .angle_data = {
            .zero_position = 0, 
            .stop_position = 4095}, 
        .config ={
            .power_mode            = AS5600_POWER_MODE_NOM,
            .hysteresis            = AS5600_HYSTERESIS_OFF,
            .output_stage          = AS5600_OUTPUT_STAGE_PWM,
            .slow_filter           = AS5600_SLOW_FILTER_2X,
            .fast_filter_threshold = AS5600_FAST_FILTER_THRESHOLD_6LSB,
            .watchdog_enable       = AS5600_BOOL_FALSE,           
        },
        .is_initialized = 0
    },
    [AS5600_DEVICE_3] = {
        .iic_bus = {
            .IIC_SDA_PORT = GPIOC,
            .IIC_SDA_PIN = GPIO_PIN_11,
            .IIC_SCL_PORT = GPIOC,
            .IIC_SCL_PIN = GPIO_PIN_10
        }, 
        .handle = {
            .iic_init = as5600_init_3,
            .iic_deinit = as5600_deinit_3,
            .iic_read = as5600_read_3,
            .iic_write = as5600_write_3, 
            .delay_ms = HAL_Delay, 
            .debug_print = as5600_debug_print}, 
        .angle_data = {
            .zero_position = 0, 
            .stop_position = 4095}, 
        .config ={
            .power_mode            = AS5600_POWER_MODE_NOM,
            .hysteresis            = AS5600_HYSTERESIS_OFF,
            .output_stage          = AS5600_OUTPUT_STAGE_PWM,
            .slow_filter           = AS5600_SLOW_FILTER_2X,
            .fast_filter_threshold = AS5600_FAST_FILTER_THRESHOLD_6LSB,
            .watchdog_enable       = AS5600_BOOL_FALSE,
        },
        .is_initialized = 0
    },
};

static float calculate_relative_angle(float current_angle, uint16_t zero_position)
{
    float zero_angle = (float)zero_position * (360.0f / 4096.0f);
    float relative_angle = current_angle - zero_angle;
    // 归一化到 [0, 360)
    relative_angle = fmodf(relative_angle, 360.0f);
    if (relative_angle < 0.0f)
    {
        relative_angle += 360.0f;
    }
    return relative_angle;
}

static uint8_t as5600_configure_device(AS5600_Device_Index_t device_index)
{
    if (device_index >= AS5600_DEVICE_COUNT) return 1;
    
    as5600_device_t *dev = &g_as5600_devices[device_index];
    const as5600_config_t *conf = &dev->config; 
    
    if (as5600_set_power_mode(&dev->handle, conf->power_mode) != 0) return 1;
    if (as5600_set_hysteresis(&dev->handle, conf->hysteresis) != 0) return 1;
    if (as5600_set_output_stage(&dev->handle, conf->output_stage) != 0) return 1;
    if (as5600_set_slow_filter(&dev->handle, conf->slow_filter) != 0) return 1;
    if (as5600_set_fast_filter_threshold(&dev->handle, conf->fast_filter_threshold) != 0) return 1;
    if (as5600_set_watch_dog(&dev->handle, conf->watchdog_enable) != 0) return 1;
    
    // 起始/停止位置仍然从 angle_data 中获取
    if (as5600_set_start_position(&dev->handle, dev->angle_data.zero_position) != 0) return 1;
    if (as5600_set_stop_position(&dev->handle, dev->angle_data.stop_position) != 0) return 1;
    return 0; 
}

void as5600_update_angle_data(AS5600_Device_Index_t device_index)
{
    static uint32_t last_success_times[AS5600_DEVICE_COUNT] = {0};
    if (device_index >= AS5600_DEVICE_COUNT || !g_as5600_devices[device_index].is_initialized)
    {
        return; 
    }
    
    as5600_device_t *dev = &g_as5600_devices[device_index];
    AS5600_angle_t *angle_data = &dev->angle_data;
    
    uint16_t angle_raw;
    float angle_deg;
    
    if (as5600_read(&dev->handle, &angle_raw, &angle_deg) == 0)
    {
        uint32_t current_time = HAL_GetTick();
        if (last_success_times[device_index] == 0) {
            last_success_times[device_index] = current_time;
            angle_data->current_angle = angle_deg;
            angle_data->last_angle = angle_deg;
            return;
        }
        
        float time_diff = (current_time - last_success_times[device_index]) / 1000.0f;
        if (time_diff < 0.001f) { // 防止除以零或过小的时间间隔
            return;
        }
        
        angle_data->last_angle = angle_data->current_angle;
        angle_data->current_angle = angle_deg;
        
        float angle_diff = angle_data->current_angle - angle_data->last_angle;
        if (angle_diff > 180.0f)
        {
            angle_diff -= 360.0f;
            angle_data->total_turns--; // 逆时针
        }
        else if (angle_diff < -180.0f)
        {
            angle_diff += 360.0f;
            angle_data->total_turns++; // 顺时针
        }
        
        // 计算速度和RPM
        angle_data->angular_velocity = angle_diff / time_diff;             // 度/秒
        angle_data->rpm = (angle_data->angular_velocity / 360.0f) * 60.0f; // 转/分
        
        // 计算相对角度
        angle_data->relative_angle = calculate_relative_angle(angle_data->current_angle,angle_data->zero_position);
        last_success_times[device_index] = current_time;
    }
    else
    {
        as5600_debug_print("AS5600 Device %d read angle error\r\n", device_index + 1);
    }
}

float as5600_get_rpm(AS5600_Device_Index_t device_index)
{
    if (device_index >= AS5600_DEVICE_COUNT) return 0;
    return g_as5600_devices[device_index].angle_data.rpm;
}

int32_t as5600_get_total_turns(AS5600_Device_Index_t device_index)
{
    if (device_index >= AS5600_DEVICE_COUNT) return 0;
    return g_as5600_devices[device_index].angle_data.total_turns;
}

float as5600_get_relative_angle(AS5600_Device_Index_t device_index)
{
    if (device_index >= AS5600_DEVICE_COUNT) return 0;
    return g_as5600_devices[device_index].angle_data.relative_angle;
}

/**
 * @brief timer2中断回调函数
 * 
 * @param htim 
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
        for (int i = 0; i < AS5600_DEVICE_COUNT; i++){
            g_as5600_devices[i].angle_data.angle_flag = 1;
        }
    }
}

void as5600_init_all(void)
{
    for (int i = 0; i < AS5600_DEVICE_COUNT; i++){
        if (as5600_init(&g_as5600_devices[i].handle) == 0){
            if (as5600_configure_device((AS5600_Device_Index_t)i) == 0){
                g_as5600_devices[i].is_initialized = 1;
                as5600_debug_print("AS5600 Device %d initialized and configured successfully.\r\n", i + 1);
            }else{
                as5600_debug_print("AS5600 Device %d configuration failed.\r\n", i + 1);
            }
        }else{
            as5600_debug_print("AS5600 Device %d I2C init failed.\r\n", i + 1);
        }
    }
}

void as5600_debug_print(const char *const fmt, ...)
{
    char buf[256];
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    if (len > 0 && len < sizeof(buf))
    {
        HAL_UART_Transmit(&huart5, (uint8_t *)buf, len, 1000);
    }
}

