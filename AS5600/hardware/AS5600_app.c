#include "AS5600_app.h"
#include "usart.h"
#include <stdarg.h>
#include <stdio.h>

#define AS5600_FILTER_ALPHA_MIN         0.16f   //低速跟随系数，兼顾慢速连续性和静止稳定性
#define AS5600_FILTER_ALPHA_MAX         0.65f   //快速变化跟随系数，数值越大响应越快但滤波越弱
#define AS5600_FILTER_DELTA_LOW_DEG     0.35f   //低速跟随阈值，单位：度
#define AS5600_FILTER_DELTA_HIGH_DEG    3.00f   //快速跟随阈值，单位：度
#define AS5600_STATIC_NOISE_DEG         0.20f   //静止噪声窗口，约等于 2 个 AS5600 LSB
#define AS5600_MOTION_START_DEG         0.25f   //连续低速运动累计到该阈值后退出静止保持
#define AS5600_ANGLE_FULL_SCALE_DEG     360.0f
#define AS5600_ANGLE_HALF_SCALE_DEG     180.0f

static float g_as5600_filtered_angle[AS5600_DEVICE_COUNT] = {0.0f};
static float g_as5600_last_raw_angle[AS5600_DEVICE_COUNT] = {0.0f};
static float g_as5600_motion_accum[AS5600_DEVICE_COUNT] = {0.0f};
static uint8_t g_as5600_filter_initialized[AS5600_DEVICE_COUNT] = {0U};

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
    return IIC_Read_Multi_Byte(&g_as5600_devices[AS5600_DEVICE_1].iic_bus, addr, reg, len, data); 
}
static uint8_t as5600_write_1(uint8_t addr, uint8_t reg, uint8_t *data, uint16_t len)
{ 
    return IIC_Write_Multi_Byte(&g_as5600_devices[AS5600_DEVICE_1].iic_bus, addr, reg, len, data); 
}

/* 单设备配置不编译第二路回调及设备表项，避免数组越界。 */
#if AS5600_DEVICE_COUNT > 1
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
    return IIC_Read_Multi_Byte(&g_as5600_devices[AS5600_DEVICE_2].iic_bus, addr, reg, len, data); 
}
static uint8_t as5600_write_2(uint8_t addr, uint8_t reg, uint8_t *data, uint16_t len) 
{ 
    return IIC_Write_Multi_Byte(&g_as5600_devices[AS5600_DEVICE_2].iic_bus, addr, reg, len, data); 
}
#endif

as5600_device_t g_as5600_devices[AS5600_DEVICE_COUNT] = {
    [AS5600_DEVICE_1] = {
        .iic_bus = {
            .IIC_SDA_PORT = SDA1_GPIO_Port,
            .IIC_SDA_PIN = SDA1_Pin,
            .IIC_SCL_PORT = SCL1_GPIO_Port,
            .IIC_SCL_PIN = SCL1_Pin,
        },
        .handle = {
            .iic_init = as5600_init_1, 
            .iic_deinit = as5600_deinit_1, 
            .iic_read = as5600_read_1, 
            .iic_write = as5600_write_1, 
            .delay_ms = HAL_Delay, 
            .debug_print = as5600_debug_print},
        .angle_data = {
            .relative_angle = 0.0f,
            .angle_flag = 0,
            .writing_flag = 0
        },  
        .config ={
            .power_mode            = AS5600_POWER_MODE_NOM,
            .hysteresis            = AS5600_HYSTERESIS_OFF,
            .output_stage          = AS5600_OUTPUT_STAGE_ANALOG_FULL,
            .slow_filter           = AS5600_SLOW_FILTER_8X,
            .fast_filter_threshold = AS5600_FAST_FILTER_THRESHOLD_SLOW_FILTER_ONLY,
            .watchdog_enable       = AS5600_BOOL_FALSE,
        },
        .is_initialized = 0
    },
#if AS5600_DEVICE_COUNT > 1
    [AS5600_DEVICE_2] = {
        .iic_bus = {
            .IIC_SDA_PORT = SDA2_GPIO_Port,
            .IIC_SDA_PIN = SDA2_Pin,
            .IIC_SCL_PORT = SCL2_GPIO_Port,
            .IIC_SCL_PIN = SCL2_Pin
        }, 
        .handle = {
            .iic_init = as5600_init_2,
            .iic_deinit = as5600_deinit_2,
            .iic_read = as5600_read_2,
            .iic_write = as5600_write_2, 
            .delay_ms = HAL_Delay, 
            .debug_print = as5600_debug_print}, 
        .angle_data = {
            .relative_angle = 0.0f,
            .angle_flag = 0,
            .writing_flag = 0
        },  
        .config ={
            .power_mode            = AS5600_POWER_MODE_NOM,
            .hysteresis            = AS5600_HYSTERESIS_OFF,
            .output_stage          = AS5600_OUTPUT_STAGE_ANALOG_FULL,
            .slow_filter           = AS5600_SLOW_FILTER_8X,
            .fast_filter_threshold = AS5600_FAST_FILTER_THRESHOLD_SLOW_FILTER_ONLY,
            .watchdog_enable       = AS5600_BOOL_FALSE,           
        },
        .is_initialized = 0
    },
#endif
};

static uint8_t as5600_configure_device(AS5600_Device_Index_t device_index)
{
    as5600_device_t *dev = &g_as5600_devices[device_index];
    const as5600_config_t *conf = &dev->config; 
    
    if (as5600_set_power_mode(&dev->handle, conf->power_mode) != 0) return 1;
    if (as5600_set_hysteresis(&dev->handle, conf->hysteresis) != 0) return 1;
    if (as5600_set_output_stage(&dev->handle, conf->output_stage) != 0) return 1;
    if (as5600_set_slow_filter(&dev->handle, conf->slow_filter) != 0) return 1;
    if (as5600_set_fast_filter_threshold(&dev->handle, conf->fast_filter_threshold) != 0) return 1;
    if (as5600_set_watch_dog(&dev->handle, conf->watchdog_enable) != 0) return 1;
    return 0; 
}

static void as5600_reset_angle_filter(AS5600_Device_Index_t device_index)
{
    g_as5600_filtered_angle[device_index] = 0.0f;
    g_as5600_last_raw_angle[device_index] = 0.0f;
    g_as5600_motion_accum[device_index] = 0.0f;
    g_as5600_filter_initialized[device_index] = 0U;
}

static uint8_t as5600_is_magnet_detected(as5600_handle_t *handle)
{
    uint8_t status = 0U;

    if (as5600_get_status(handle, &status) != 0)
    {
        return 0U;
    }

    return ((status & AS5600_STATUS_MD) != 0U) ? 1U : 0U;
}

static float as5600_wrap_angle(float angle)
{
    if (angle >= AS5600_ANGLE_FULL_SCALE_DEG)
    {
        angle -= AS5600_ANGLE_FULL_SCALE_DEG;
    }
    else if (angle < 0.0f)
    {
        angle += AS5600_ANGLE_FULL_SCALE_DEG;
    }

    return angle;
}

static float as5600_abs_float(float value)
{
    if (value < 0.0f)
    {
        return -value;
    }

    return value;
}

static float as5600_angle_delta(float angle, float reference)
{
    float delta = angle - reference;

    if (delta > AS5600_ANGLE_HALF_SCALE_DEG)
    {
        delta -= AS5600_ANGLE_FULL_SCALE_DEG;
    }
    else if (delta < -AS5600_ANGLE_HALF_SCALE_DEG)
    {
        delta += AS5600_ANGLE_FULL_SCALE_DEG;
    }

    return delta;
}

static float as5600_get_adaptive_alpha(float angle_delta_abs)
{
    float ratio;

    if (angle_delta_abs <= AS5600_FILTER_DELTA_LOW_DEG)
    {
        return AS5600_FILTER_ALPHA_MIN;
    }

    if (angle_delta_abs >= AS5600_FILTER_DELTA_HIGH_DEG)
    {
        return AS5600_FILTER_ALPHA_MAX;
    }

    ratio = (angle_delta_abs - AS5600_FILTER_DELTA_LOW_DEG) /
            (AS5600_FILTER_DELTA_HIGH_DEG - AS5600_FILTER_DELTA_LOW_DEG);

    return AS5600_FILTER_ALPHA_MIN + ratio * (AS5600_FILTER_ALPHA_MAX - AS5600_FILTER_ALPHA_MIN);
}

static float as5600_low_pass_filter_angle(AS5600_Device_Index_t device_index, float angle_deg)
{
    float *filtered_angle = &g_as5600_filtered_angle[device_index];
    float raw_delta;
    float angle_delta;
    float angle_delta_abs;
    float raw_delta_abs;
    float alpha;

    if (g_as5600_filter_initialized[device_index] == 0U)
    {
        *filtered_angle = angle_deg;
        g_as5600_last_raw_angle[device_index] = angle_deg;
        g_as5600_motion_accum[device_index] = 0.0f;
        g_as5600_filter_initialized[device_index] = 1U;

        return angle_deg;
    }

    raw_delta = as5600_angle_delta(angle_deg, g_as5600_last_raw_angle[device_index]);
    raw_delta_abs = as5600_abs_float(raw_delta);
    g_as5600_last_raw_angle[device_index] = angle_deg;

    if ((raw_delta * g_as5600_motion_accum[device_index]) < 0.0f)
    {
        g_as5600_motion_accum[device_index] = raw_delta;
    }
    else
    {
        g_as5600_motion_accum[device_index] += raw_delta;
    }

    if (g_as5600_motion_accum[device_index] > AS5600_MOTION_START_DEG)
    {
        g_as5600_motion_accum[device_index] = AS5600_MOTION_START_DEG;
    }
    else if (g_as5600_motion_accum[device_index] < -AS5600_MOTION_START_DEG)
    {
        g_as5600_motion_accum[device_index] = -AS5600_MOTION_START_DEG;
    }

    angle_delta = as5600_angle_delta(angle_deg, *filtered_angle);
    angle_delta_abs = as5600_abs_float(angle_delta);

    if ((raw_delta_abs <= AS5600_STATIC_NOISE_DEG) &&
        (angle_delta_abs <= AS5600_STATIC_NOISE_DEG))
    {
        g_as5600_motion_accum[device_index] = 0.0f;
        return *filtered_angle;
    }

    /* 静止时锁住滤波输出；连续同向的小步进累计后再按低速运动跟随。 */
    if ((raw_delta_abs <= AS5600_STATIC_NOISE_DEG) &&
        (as5600_abs_float(g_as5600_motion_accum[device_index]) < AS5600_MOTION_START_DEG) &&
        (angle_delta_abs <= AS5600_MOTION_START_DEG))
    {
        return *filtered_angle;
    }

    alpha = as5600_get_adaptive_alpha(angle_delta_abs);
    *filtered_angle += alpha * angle_delta;
    *filtered_angle = as5600_wrap_angle(*filtered_angle);

    return *filtered_angle;
}

void as5600_update_angle_data(AS5600_Device_Index_t device_index)
{
    as5600_device_t *dev = &g_as5600_devices[device_index];
    AS5600_angle_t *angle_data = &dev->angle_data;
    uint16_t angle_raw = 0U;
    float angle_deg;

    if (as5600_is_magnet_detected(&dev->handle) == 0U)
    {
        as5600_reset_angle_filter(device_index);
        return;
    }

    if (as5600_read(&dev->handle, &angle_raw, &angle_deg) == 0)
    {
        angle_data->writing_flag = 1;
        angle_data->relative_angle = as5600_low_pass_filter_angle(device_index, angle_deg);
        angle_data->writing_flag = 0;
    }
    (void)angle_raw;
}

AS5600_angle_t * ad5600_get_angle(AS5600_Device_Index_t device_index)
{
	as5600_device_t * dev = &g_as5600_devices[device_index];
    AS5600_angle_t * angle_data = &dev->angle_data;
	return angle_data;
}

void AS5600_Data_Handler(void)
{
    for (int i = 0; i < AS5600_DEVICE_COUNT; i++) 
    {
      if (g_as5600_devices[i].angle_data.angle_flag)
      {
        g_as5600_devices[i].angle_data.angle_flag = 0; 
        as5600_update_angle_data((AS5600_Device_Index_t) i);
      }
    }
}

void as5600_init_all(void)
{
    for (int i = 0; i < AS5600_DEVICE_COUNT; i++){
        if (as5600_init(&g_as5600_devices[i].handle) == 0){
            if (as5600_configure_device((AS5600_Device_Index_t)i) == 0){
                g_as5600_devices[i].is_initialized = 1;
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

