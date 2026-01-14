#ifndef __OV7725_H__
#define __OV7725_H__

#include "ov7725_reg.h"
#include "hal_iic.h"
#include "log.h"
#include "main.h"

static const uint8_t OV7725_init_cfg[65][2] = {
    {OV7725_CLKRC,     0x00},
    {OV7725_COM7,      0x46},
    {OV7725_HSTART,    0x3F},
    {OV7725_HSIZE,     0x50},
    {OV7725_VSTRT,     0x03},
    {OV7725_VSIZE,     0x78},
    {OV7725_HREF,      0x00},
    {OV7725_HOutSize,  0x50},
    {OV7725_VOutSize,  0x78},
    {OV7725_TGT_B,     0x7F},
    {OV7725_FixGain,   0x09},
    {OV7725_AWB_Ctrl0, 0xE0},
    {OV7725_DSP_Ctrl1, 0xFF},
    {OV7725_DSP_Ctrl2, 0x00},
    {OV7725_DSP_Ctrl3, 0x00},
    {OV7725_DSP_Ctrl4, 0x00},
    {OV7725_COM8,      0xF0},
    {OV7725_COM4,      0xC1},
    {OV7725_COM6,      0xC5},
    {OV7725_COM9,      0x11},
    {OV7725_BDBase,    0x7F},
    {OV7725_BDMStep,   0x03},
    {OV7725_AEW,       0x40},
    {OV7725_AEB,       0x30},
    {OV7725_VPT,       0xA1},
    {OV7725_EXHCL,     0x9E},
    {OV7725_AWBCtrl3,  0xAA},
    {OV7725_COM8,      0xFF},
    {OV7725_EDGE1,     0x08},
    {OV7725_DNSOff,    0x01},
    {OV7725_EDGE2,     0x03},
    {OV7725_EDGE3,     0x00},
    {OV7725_MTX1,      0xB0},
    {OV7725_MTX2,      0x9D},
    {OV7725_MTX3,      0x13},
    {OV7725_MTX4,      0x16},
    {OV7725_MTX5,      0x7B},
    {OV7725_MTX6,      0x91},
    {OV7725_MTX_Ctrl,  0x1E},
    {OV7725_BRIGHT,    0x08},
    {OV7725_CNST,      0x20},
    {OV7725_UVADJ0,    0x81},
    {OV7725_SDE,       0x06},
    {OV7725_USAT,      0x65},
    {OV7725_VSAT,      0x65},
    {OV7725_HUECOS,    0x80},
    {OV7725_HUESIN,    0x80},
    {OV7725_GAM1,      0x0C},
    {OV7725_GAM2,      0x16},
    {OV7725_GAM3,      0x2A},
    {OV7725_GAM4,      0x4E},
    {OV7725_GAM5,      0x61},
    {OV7725_GAM6,      0x6F},
    {OV7725_GAM7,      0x7B},
    {OV7725_GAM8,      0x86},
    {OV7725_GAM9,      0x8E},
    {OV7725_GAM10,     0x97},
    {OV7725_GAM11,     0xA4},
    {OV7725_GAM12,     0xAF},
    {OV7725_GAM13,     0xC5},
    {OV7725_GAM14,     0xD7},
    {OV7725_GAM15,     0xE8},
    {OV7725_SLOP,      0x20},
    {OV7725_COM3,      0x50},
    {OV7725_COM5,      0xF5},
};

#define OV7725_WRST(x)                      do{ x ?                                                                     \
                                                HAL_GPIO_WritePin(FIFO_WRST_GPIO_Port, FIFO_WRST_Pin, GPIO_PIN_SET) :   \
                                                HAL_GPIO_WritePin(FIFO_WRST_GPIO_Port, FIFO_WRST_Pin, GPIO_PIN_RESET);  \
                                            }while(0)
#define OV7725_RRST(x)                      do{ x ?                                                                     \
                                                HAL_GPIO_WritePin(FIFO_RRST_GPIO_Port, FIFO_RRST_Pin, GPIO_PIN_SET) :   \
                                                HAL_GPIO_WritePin(FIFO_RRST_GPIO_Port, FIFO_RRST_Pin, GPIO_PIN_RESET);  \
                                            }while(0)
#define OV7725_OE(x)                        do{ x ?                                                                     \
                                                HAL_GPIO_WritePin(FIFO_OE_GPIO_Port, FIFO_OE_Pin, GPIO_PIN_SET) :       \
                                                HAL_GPIO_WritePin(FIFO_OE_GPIO_Port, FIFO_OE_Pin, GPIO_PIN_RESET);      \
                                            }while(0)
#define OV7725_RCLK(x)                      do{ x ?                                                                     \
                                                (FIFO_RCLK_GPIO_Port->BSRR = (uint32_t)FIFO_RCLK_Pin) :                 \
                                                (FIFO_RCLK_GPIO_Port->BSRR = (uint32_t)FIFO_RCLK_Pin << 16);            \
                                            }while(0)
#define OV7725_WEN(x)                       do{ x ?                                                                     \
                                                HAL_GPIO_WritePin(FIFO_WEN_GPIO_Port, FIFO_WEN_Pin, GPIO_PIN_SET) :     \
                                                HAL_GPIO_WritePin(FIFO_WEN_GPIO_Port, FIFO_WEN_Pin, GPIO_PIN_RESET);    \
                                            }while(0)

/* OV7725设备信息枚举 */
typedef enum
{
    OV7725_SCCB_ADDR  = 0x21,   /* SCCB通讯设备地址 */
    OV7725_MID_DEVICE = 0x7FA2,        /* 模块ID */
    OV7725_PID_DEVICE = 0x7721,        /* 产品ID */
}OV7725_Device_Info_t;

/* OV7725模块状态枚举 */
typedef enum
{
    OV7725_OK = 0,       /* 没有错误 */
    OV7725_ERROR = 1,    /* 错误 */
    OV7725_EINVAL = 2,   /* 非法参数 */
    OV7725_EEMPTY = 3,   /* 资源为空 */
} OV7725_Status_t;

/* OV7725模块在不同输出模式下的最大输出分辨率 */
typedef enum
{
    OV7725_VGA_WIDTH_MAX  = 640,
    OV7725_VGA_HEIGHT_MAX  = 480,
    OV7725_QVGA_WIDTH_MAX  = 320,
    OV7725_QVGA_HEIGHT_MAX  = 240,
} ov7725_output_mode_resolution_t;

/* OV7725模块灯光模式枚举 */
typedef enum
{
    OV7725_LIGHT_MODE_AUTO = 0x00,         /* Auto */
    OV7725_LIGHT_MODE_SUNNY,               /* Sunny */
    OV7725_LIGHT_MODE_CLOUDY,              /* Cloudy */
    OV7725_LIGHT_MODE_OFFICE,              /* Office */
    OV7725_LIGHT_MODE_HOME,                /* Home */
    OV7725_LIGHT_MODE_NIGHT,               /* Night */
} ov7725_light_mode_t;

/* OV7725模块色彩饱和度枚举 */
typedef enum
{
    OV7725_COLOR_SATURATION_0 = 0x00,      /* +4 */
    OV7725_COLOR_SATURATION_1,             /* +3 */
    OV7725_COLOR_SATURATION_2,             /* +2 */
    OV7725_COLOR_SATURATION_3,             /* +1 */
    OV7725_COLOR_SATURATION_4,             /* 0 */
    OV7725_COLOR_SATURATION_5,             /* -1 */
    OV7725_COLOR_SATURATION_6,             /* -2 */
    OV7725_COLOR_SATURATION_7,             /* -3 */
    OV7725_COLOR_SATURATION_8,             /* -4 */
} ov7725_color_saturation_t;

/* OV7725模块亮度枚举 */
typedef enum
{
    OV7725_BRIGHTNESS_0 = 0x00,            /* +4 */
    OV7725_BRIGHTNESS_1,                   /* +3 */
    OV7725_BRIGHTNESS_2,                   /* +2 */
    OV7725_BRIGHTNESS_3,                   /* +1 */
    OV7725_BRIGHTNESS_4,                   /* 0 */
    OV7725_BRIGHTNESS_5,                   /* -1 */
    OV7725_BRIGHTNESS_6,                   /* -2 */
    OV7725_BRIGHTNESS_7,                   /* -3 */
    OV7725_BRIGHTNESS_8,                   /* -4 */
} ov7725_brightness_t;

/* OV7725模块对比度枚举 */
typedef enum
{
    OV7725_CONTRAST_0 = 0x00,              /* +4 */
    OV7725_CONTRAST_1,                     /* +3 */
    OV7725_CONTRAST_2,                     /* +2 */
    OV7725_CONTRAST_3,                     /* +1 */
    OV7725_CONTRAST_4,                     /* 0 */
    OV7725_CONTRAST_5,                     /* -1 */
    OV7725_CONTRAST_6,                     /* -2 */
    OV7725_CONTRAST_7,                     /* -3 */
    OV7725_CONTRAST_8,                     /* -4 */
} ov7725_contrast_t;

/* OV7725模块特殊效果枚举 */
typedef enum
{
    OV7725_SPECIAL_EFFECT_NORMAL = 0x00,   /* Normal */
    OV7725_SPECIAL_EFFECT_BW,              /* B&W */
    OV7725_SPECIAL_EFFECT_BLUISH,          /* Bluish */
    OV7725_SPECIAL_EFFECT_SEPIA,           /* Sepia */
    OV7725_SPECIAL_EFFECT_REDISH,          /* Redish */
    OV7725_SPECIAL_EFFECT_GREENISH,        /* Greenish */
    OV7725_SPECIAL_EFFECT_NEGATIVE,        /* Negative */
} ov7725_special_effect_t;

/* OV7725模块输出模式枚举 */
typedef enum
{
    OV7725_OUTPUT_MODE_VGA = 0x00,         /* VGA */
    OV7725_OUTPUT_MODE_QVGA,               /* QVGA */
} ov7725_output_mode_t;

/* OV7725模块获取图像方式枚举 */
typedef enum
{
    OV7725_GET_FRAME_TYPE_NOINC = 0x00,    /* 目的地址不自增 */
    OV7725_GET_FRAME_TYPE_AUTO_INC,        /* 目的地址自增 */
} ov7725_get_frame_type_t;

// OV7725模块捕获状态枚举（支持双帧缓冲）
typedef enum {
    CAPTURE_IDLE = 0,      /* 空闲，可开始新采集 */
    CAPTURE_ONGOING,       /* 正在采集第一帧 */
    CAPTURE_DONE,          /* 一帧采集完成，待处理 */
} OV7725_Capture_State_t;

typedef struct
{
    iic_bus_t *bus;                     /* I2C总线 */
    uint8_t addr;                       /* 设备地址 */
    uint16_t image_width;               /* 宽度 */
    uint16_t image_height;              /* 高度 */              /* 处理图像帧信息 */
} OV7725_Handle_t;

/**
 * @brief 初始化OV7725模块
 * @return uint8_t OV7725_OK: 初始化成功, OV7725_ERROR: 初始化失败
 */
uint8_t ov7725_Init(void);

/**
 * @brief 配置OV7725模块
 * @param width 图像宽度
 * @param height 图像高度
 * @param mode 输出模式
 * @param light_mode 灯光模式
 * @param saturation 色彩饱和度
 * @param brightness 亮度
 * @param contrast 对比度
 * @param effect 特殊效果
 * @return uint8_t OV7725_OK: 配置成功, OV7725_ERROR: 配置失败
 */
uint8_t ov7725_Config(uint16_t width, uint16_t height, ov7725_output_mode_t mode,
                      ov7725_light_mode_t light_mode, ov7725_color_saturation_t saturation,
                      ov7725_brightness_t brightness, ov7725_contrast_t contrast,
                      ov7725_special_effect_t effect);

/**
 * @brief 获取OV7725模块输出的一帧图像数据
 * @param dts 指向存储图像数据的缓冲区指针
 * @param type 获取图像方式
 * @return uint8_t OV7725_OK: 获取成功, OV7725_ERROR: 获取失败
 */
uint8_t ov7725_Get_Frame_Data(volatile uint16_t *dts, ov7725_get_frame_type_t type);

/**
 * @brief 开始读取OV7725帧数据（初始化FIFO读取）
 * @return uint8_t OV7725_OK: 成功, OV7725_EEMPTY: 无帧数据
 */
uint8_t ov7725_Frame_Read_Start(void);

/**
 * @brief 读取OV7725帧数据块
 * @param buffer: 数据缓冲区指针（uint8_t类型）
 * @param pixel_count: 要读取的像素数量
 * @return uint16_t 实际读取的像素数量
 */
uint16_t ov7725_Frame_Read_Chunk(uint8_t *buffer, uint16_t pixel_count);

/**
 * @brief 结束读取OV7725帧数据
 */
void ov7725_Frame_Read_End(void);

/**
 * @brief 检查是否有帧数据可读
 * @return 1: 有数据可读, 0: 无数据
 */
uint8_t ov7725_Frame_Available(void);

#endif // __OV7725_H__
