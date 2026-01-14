#include "ov7725.h"

/* IIC总线实例 */
static iic_bus_t OV7725_IIC_Bus = {
    .IIC_SCL_PORT = OV_SCL_GPIO_Port,
    .IIC_SCL_PIN = OV_SCL_Pin,
    .IIC_SDA_PORT = OV_SDA_GPIO_Port,
    .IIC_SDA_PIN = OV_SDA_Pin,
};

OV7725_Handle_t OV7725_Camera = {
    .bus = &OV7725_IIC_Bus,
    .addr = OV7725_SCCB_ADDR,
    .image_width = OV7725_QVGA_WIDTH_MAX,
    .image_height = OV7725_QVGA_HEIGHT_MAX,
};

/**
 * @brief 向OV7725模块写入一个寄存器
 * @param reg 寄存器地址
 * @param data 要写入的数据
 * @return uint8_t 写入状态，0表示成功，非0表示失败
 */
static uint8_t ov7725_write_reg(uint8_t reg, uint8_t data)
{
    return IIC_Write_One_Byte(OV7725_Camera.bus, OV7725_Camera.addr, reg, data);
}

/**
 * @brief 从OV7725模块读取一个寄存器
 * @param reg 寄存器地址
 * @return uint8_t 读取到的数据
 */
static uint8_t ov7725_read_reg(uint8_t reg)
{
    return IIC_Read_One_Byte(OV7725_Camera.bus, OV7725_Camera.addr, reg);
}

/**
 * @brief 复位OV7725模块
 * @return uint8_t 重置状态，0表示成功，非0表示失败
 */
static void ov7725_reset(void)
{
    ov7725_write_reg(OV7725_COM7, 0x80);
    HAL_Delay(10);
}

/**
 * @brief 获取OV7725的制造商ID和产品ID
 * @param MID 制造商ID指针 (MIDH << 8 | MIDL)
 * @param PID 产品ID指针 (PID << 8 | VER)
 * @return 无
 */
static void ov7725_get_id(uint16_t *MID, uint16_t *PID)
{
    *MID = ov7725_read_reg(OV7725_MIDH) << 8;
    *MID |= ov7725_read_reg(OV7725_MIDL);
    
    *PID = ov7725_read_reg(OV7725_PID) << 8;
    *PID |= ov7725_read_reg(OV7725_VER);
}

/**
 * @brief 初始化OV7725模块寄存器
 * @param 无
 * @retval 无
 */
static void ov7725_init_reg(void)
{
    uint8_t cfg_index;
    uint8_t cfg_size = sizeof(OV7725_init_cfg) / sizeof(OV7725_init_cfg[0]);
    
    for (cfg_index = 0; cfg_index < cfg_size; cfg_index++)
    {
        ov7725_write_reg(OV7725_init_cfg[cfg_index][0], OV7725_init_cfg[cfg_index][1]);
        switch (OV7725_init_cfg[cfg_index][0])
        {
            case OV7725_HSIZE:
            {
                OV7725_Camera.image_width = OV7725_init_cfg[cfg_index][1] << 2;
                break;
            }
            case OV7725_VSIZE:
            {
                OV7725_Camera.image_height = OV7725_init_cfg[cfg_index][1] << 1;
                break;
            }
            default:
                break;
        }
    }
}

/**
 * @brief 设置OV7725模块灯光模式
 * @param mode: OV7725_LIGHT_MODE_AUTO  : Auto
 *              OV7725_LIGHT_MODE_SUNNY : Sunny
 *              OV7725_LIGHT_MODE_CLOUDY: Cloudy
 *              OV7725_LIGHT_MODE_OFFICE: Office
 *              OV7725_LIGHT_MODE_HOME  : Home
 *              OV7725_LIGHT_MODE_NIGHT : Night
 * @return uint8_t OV7725_OK: 设置成功, OV7725_EINVAL: 参数错误
 */
static uint8_t ov7725_set_light_mode(ov7725_light_mode_t mode)
{
    switch (mode)
    {
        case OV7725_LIGHT_MODE_AUTO:
        {
            ov7725_write_reg(OV7725_COM8, 0xFF);
            ov7725_write_reg(OV7725_COM5, 0x65);
            ov7725_write_reg(OV7725_ADVFL, 0x00);
            ov7725_write_reg(OV7725_ADVFH, 0x00);
            break;
        }
        case OV7725_LIGHT_MODE_SUNNY:
        {
            ov7725_write_reg(OV7725_COM8, 0xFD);
            ov7725_write_reg(OV7725_BLUE, 0x5A);
            ov7725_write_reg(OV7725_RED, 0x5C);
            ov7725_write_reg(OV7725_COM5, 0x65);
            ov7725_write_reg(OV7725_ADVFL, 0x00);
            ov7725_write_reg(OV7725_ADVFH, 0x00);
            break;
        }
        case OV7725_LIGHT_MODE_CLOUDY:
        {
            ov7725_write_reg(OV7725_COM8, 0xFD);
            ov7725_write_reg(OV7725_BLUE, 0x58);
            ov7725_write_reg(OV7725_RED, 0x60);
            ov7725_write_reg(OV7725_COM5, 0x65);
            ov7725_write_reg(OV7725_ADVFL, 0x00);
            ov7725_write_reg(OV7725_ADVFH, 0x00);
            break;
        }
        case OV7725_LIGHT_MODE_OFFICE:
        {
            ov7725_write_reg(OV7725_COM8, 0xFD);
            ov7725_write_reg(OV7725_BLUE, 0x84);
            ov7725_write_reg(OV7725_RED, 0x4C);
            ov7725_write_reg(OV7725_COM5, 0x65);
            ov7725_write_reg(OV7725_ADVFL, 0x00);
            ov7725_write_reg(OV7725_ADVFH, 0x00);
            break;
        }
        case OV7725_LIGHT_MODE_HOME:
        {
            ov7725_write_reg(OV7725_COM8, 0xFD);
            ov7725_write_reg(OV7725_BLUE, 0x96);
            ov7725_write_reg(OV7725_RED, 0x40);
            ov7725_write_reg(OV7725_COM5, 0x65);
            ov7725_write_reg(OV7725_ADVFL, 0x00);
            ov7725_write_reg(OV7725_ADVFH, 0x00);
            break;
        }
        case OV7725_LIGHT_MODE_NIGHT:
        {
            ov7725_write_reg(OV7725_COM8, 0xFF);
            ov7725_write_reg(OV7725_COM5, 0xE5);
            break;
        }
        default:
        {
            CAW_LOG_WARN("ov7725 Set Invalid Param: %d", mode);
            return OV7725_EINVAL;
        }
    }
    return OV7725_OK;
}

/**
 * @brief 设置OV7725模块色彩饱和度
 * @param saturation: OV7725_COLOR_SATURATION_0: +4
 *                    OV7725_COLOR_SATURATION_1: +3
 *                    OV7725_COLOR_SATURATION_2: +2
 *                    OV7725_COLOR_SATURATION_3: +1
 *                    OV7725_COLOR_SATURATION_4: 0
 *                    OV7725_COLOR_SATURATION_5: -1
 *                    OV7725_COLOR_SATURATION_6: -2
 *                    OV7725_COLOR_SATURATION_7: -3
 *                    OV7725_COLOR_SATURATION_8: -4
 * @return uint8_t OV7725_OK: 设置成功, OV7725_EINVAL: 参数错误
 */
static uint8_t ov7725_set_color_saturation(ov7725_color_saturation_t saturation)
{
    uint8_t val_table[] = {0x80, 0x70, 0x60, 0x50, 0x40, 0x30, 0x20, 0x10, 0x00};
    if (saturation > OV7725_COLOR_SATURATION_8)
    {
        CAW_LOG_WARN("ov7725 Set Invalid Param: %d", saturation);
        return OV7725_EINVAL;
    }

    ov7725_write_reg(OV7725_USAT, val_table[saturation]);
    ov7725_write_reg(OV7725_VSAT, val_table[saturation]);
    return OV7725_OK;
}

/**
 * @brief 设置OV7725模块亮度
 * @param brightness: OV7725_BRIGHTNESS_0: +4
 *                    OV7725_BRIGHTNESS_1: +3
 *                    OV7725_BRIGHTNESS_2: +2
 *                    OV7725_BRIGHTNESS_3: +1
 *                    OV7725_BRIGHTNESS_4: 0
 *                    OV7725_BRIGHTNESS_5: -1
 *                    OV7725_BRIGHTNESS_6: -2
 *                    OV7725_BRIGHTNESS_7: -3
 *                    OV7725_BRIGHTNESS_8: -4
 * @return uint8_t OV7725_OK: 设置成功, OV7725_EINVAL: 参数错误
 */
static uint8_t ov7725_set_brightness(ov7725_brightness_t brightness)
{
    uint8_t bright_table[] = {0x48, 0x38, 0x28, 0x18, 0x08, 0x08, 0x18, 0x28, 0x38};
    uint8_t sign_table[]   = {0x06, 0x06, 0x06, 0x06, 0x06, 0x0E, 0x0E, 0x0E, 0x0E};
    if (brightness > OV7725_BRIGHTNESS_8)
    {
        CAW_LOG_WARN("ov7725 Set Invalid Param: %d", brightness);
        return OV7725_EINVAL;
    }

    ov7725_write_reg(OV7725_BRIGHT, bright_table[brightness]);
    ov7725_write_reg(OV7725_SIGN, sign_table[brightness]);
    return OV7725_OK;
}

/**
 * @brief 设置OV7725模块对比度
 * @param contrast: OV7725_CONTRAST_0: +4
 *                  OV7725_CONTRAST_1: +3
 *                  OV7725_CONTRAST_2: +2
 *                  OV7725_CONTRAST_3: +1
 *                  OV7725_CONTRAST_4: 0
 *                  OV7725_CONTRAST_5: -1
 *                  OV7725_CONTRAST_6: -2
 *                  OV7725_CONTRAST_7: -3
 *                  OV7725_CONTRAST_8: -4
 * @return uint8_t OV7725_OK: 设置成功, OV7725_EINVAL: 参数错误
 */
static uint8_t ov7725_set_contrast(ov7725_contrast_t contrast)
{
    uint8_t val_table[] = {0x30, 0x2C, 0x28, 0x24, 0x20, 0x1C, 0x18, 0x14, 0x10};
    if (contrast > OV7725_CONTRAST_8)
    {
        CAW_LOG_WARN("ov7725 Set Invalid Param: %d", contrast);
        return OV7725_EINVAL;
    }
    ov7725_write_reg(OV7725_CNST, val_table[contrast]);
    return OV7725_OK;
}

/**
 * @brief 设置OV7725模块特殊效果
 * @param effect: OV7725_SPECIAL_EFFECT_NORMAL  : Normal
 *                OV7725_SPECIAL_EFFECT_BW      : B&W
 *                OV7725_SPECIAL_EFFECT_BLUISH  : Bluish
 *                OV7725_SPECIAL_EFFECT_SEPIA   : Sepia
 *                OV7725_SPECIAL_EFFECT_REDISH  : Redish
 *                OV7725_SPECIAL_EFFECT_GREENISH: Greenish
 *                OV7725_SPECIAL_EFFECT_NEGATIVE: Negative
 * @return uint8_t OV7725_OK: 设置成功, OV7725_EINVAL: 参数错误
 */
static uint8_t ov7725_set_special_effect(ov7725_special_effect_t effect)
{
    switch (effect)
    {
        case OV7725_SPECIAL_EFFECT_NORMAL:
        {
            ov7725_write_reg(OV7725_SDE, 0x06);
            ov7725_write_reg(OV7725_UFIX, 0x80);
            ov7725_write_reg(OV7725_VFIX, 0x80);
            break;
        }
        case OV7725_SPECIAL_EFFECT_BW:
        {
            ov7725_write_reg(OV7725_SDE, 0x26);
            ov7725_write_reg(OV7725_UFIX, 0x80);
            ov7725_write_reg(OV7725_VFIX, 0x80);
            break;
        }
        case OV7725_SPECIAL_EFFECT_BLUISH:
        {
            ov7725_write_reg(OV7725_SDE, 0x1E);
            ov7725_write_reg(OV7725_UFIX, 0xA0);
            ov7725_write_reg(OV7725_VFIX, 0x40);
            break;
        }
        case OV7725_SPECIAL_EFFECT_SEPIA:
        {
            ov7725_write_reg(OV7725_SDE, 0x1E);
            ov7725_write_reg(OV7725_UFIX, 0x40);
            ov7725_write_reg(OV7725_VFIX, 0xA0);
            break;
        }
        case OV7725_SPECIAL_EFFECT_REDISH:
        {
            ov7725_write_reg(OV7725_SDE, 0x1E);
            ov7725_write_reg(OV7725_UFIX, 0x80);
            ov7725_write_reg(OV7725_VFIX, 0xC0);
            break;
        }
        case OV7725_SPECIAL_EFFECT_GREENISH:
        {
            ov7725_write_reg(OV7725_SDE, 0x1E);
            ov7725_write_reg(OV7725_UFIX, 0x60);
            ov7725_write_reg(OV7725_VFIX, 0x60);
            break;
        }
        case OV7725_SPECIAL_EFFECT_NEGATIVE:
        {
            ov7725_write_reg(OV7725_SDE, 0x46);
            break;
        }
        default:
        {
            CAW_LOG_WARN("ov7725 Set Invalid Param: %d", effect);
            return OV7725_EINVAL;
        }
    }
    return OV7725_OK;
}

/**
 * @brief 设置OV7725模块输出模式
 * @param width : 输出图像宽度（VGA，<=640；QVGA，<=320）
 * @param height: 输出图像高度（VGA，<=480；QVGA，<=240）
 * @param mode  : OV7725_OUTPUT_MODE_VGA : VGA输出模式
 *                OV7725_OUTPUT_MODE_QVGA: QVGA输出模式
 * @return uint8_t OV7725_OK: 设置成功, OV7725_EINVAL: 参数错误
 */
static uint8_t ov7725_set_output(uint16_t width, uint16_t height, ov7725_output_mode_t mode)
{
    uint16_t xs, ys;
    uint8_t hstart_raw, vstrt_raw, href_raw;

    switch (mode)
    {
        case OV7725_OUTPUT_MODE_VGA:
        {
            if ((width > OV7725_VGA_WIDTH_MAX) || (height > OV7725_VGA_HEIGHT_MAX)) return OV7725_EINVAL;
            xs = (OV7725_VGA_WIDTH_MAX - width) >> 1;
            ys = (OV7725_VGA_HEIGHT_MAX - height) >> 1;
            ov7725_write_reg(OV7725_COM7, 0x06);
            ov7725_write_reg(OV7725_HSTART, 0x23);
            ov7725_write_reg(OV7725_HSIZE, 0xA0);
            ov7725_write_reg(OV7725_VSTRT, 0x07);
            ov7725_write_reg(OV7725_VSIZE, 0xF0);
            ov7725_write_reg(OV7725_HREF, 0x00);
            ov7725_write_reg(OV7725_HOutSize, 0xA0);
            ov7725_write_reg(OV7725_VOutSize, 0xF0);
            break;
        }
        case OV7725_OUTPUT_MODE_QVGA:
        {
            if ((width > OV7725_QVGA_WIDTH_MAX) || (height > OV7725_QVGA_HEIGHT_MAX)) return OV7725_EINVAL;
            xs = (OV7725_QVGA_WIDTH_MAX - width) >> 1;
            ys = (OV7725_QVGA_HEIGHT_MAX - height) >> 1;
            ov7725_write_reg(OV7725_COM7, 0x46);
            ov7725_write_reg(OV7725_HSTART, 0x3F);
            ov7725_write_reg(OV7725_HSIZE, 0x50);
            ov7725_write_reg(OV7725_VSTRT, 0x03);
            ov7725_write_reg(OV7725_VSIZE, 0x78);
            ov7725_write_reg(OV7725_HREF, 0x00);
            ov7725_write_reg(OV7725_HOutSize, 0x50);
            ov7725_write_reg(OV7725_VOutSize, 0x78);
            break;
        }
        default:
        {
            CAW_LOG_WARN("ov7725 Set Invalid Param: %d", mode);
            return OV7725_EINVAL;
        }
    }

    hstart_raw = ov7725_read_reg(OV7725_HSTART);
    ov7725_write_reg(OV7725_HSTART, hstart_raw + (xs >> 2));
    ov7725_write_reg(OV7725_HSIZE, width >> 2);
    OV7725_Camera.image_width = ov7725_read_reg(OV7725_HSIZE) << 2;

    vstrt_raw = ov7725_read_reg(OV7725_VSTRT);
    ov7725_write_reg(OV7725_VSTRT, vstrt_raw + (ys >> 1));
    ov7725_write_reg(OV7725_VSIZE, height >> 1);
    OV7725_Camera.image_height = ov7725_read_reg(OV7725_VSIZE) << 1;

    href_raw = ov7725_read_reg(OV7725_HREF);
    ov7725_write_reg(OV7725_HREF, ((ys & 0x01) << 6) | ((xs & 0x03) << 4) | ((height & 0x01) << 2) | (width & 0x03) | href_raw);

    ov7725_write_reg(OV7725_HOutSize, width >> 2);
    ov7725_write_reg(OV7725_VOutSize, height >> 1);
    ov7725_write_reg(OV7725_EXHCH, href_raw | (width & 0x03) | ((height & 0x01) << 2));

    return OV7725_OK;
}

/**
 * @brief 使能OV7725模块输出图像
 * @param enable: 1 : 使能输出图像
 *                0 : 禁止输出图像
 * @retval 无
 */
static void ov7725_output_image(bool enable)
{
    if (enable == true)
    {
        OV7725_OE(0);
    }
    else
    {
        OV7725_OE(1);
    }
}

/**
 * @brief 获取OV7725模块输出图像宽度和高度
 * @param width : 图像宽度指针
 * @param height: 图像高度指针
 * @retval 无
 */
static void ov7725_get_output_size(uint16_t *width, uint16_t *height)
{
    *width = OV7725_Camera.image_width;
    *height = OV7725_Camera.image_height;
}

/**
 * @brief 配置OV7725模块运行模式
 * @param width      : 输出图像宽度
 * @param height     : 输出图像高度
 * @param mode       : 输出模式 (VGA/QVGA)
 * @param light_mode : 灯光模式
 * @param saturation : 色彩饱和度
 * @param brightness : 亮度
 * @param contrast   : 对比度
 * @param effect     : 特殊效果
 * @return uint8_t OV7725_OK: 配置成功, OV7725_ERROR: 配置失败
 */
uint8_t ov7725_Config(uint16_t width, uint16_t height, ov7725_output_mode_t mode,
                      ov7725_light_mode_t light_mode, ov7725_color_saturation_t saturation,
                      ov7725_brightness_t brightness, ov7725_contrast_t contrast,
                      ov7725_special_effect_t effect)
{
    uint8_t ret = 0;
    ret  = ov7725_set_output(width, height, mode);
    ret += ov7725_set_light_mode(light_mode);
    ret += ov7725_set_color_saturation(saturation);
    ret += ov7725_set_brightness(brightness);
    ret += ov7725_set_contrast(contrast);
    ret += ov7725_set_special_effect(effect);
    if (ret != 0)
    {
        CAW_LOG_ERROR("ov7725_config Error! : ret = 0x%02X", ret);
        return OV7725_ERROR;
    }

    ov7725_output_image(true);
    return OV7725_OK;
}

/**
 * @brief 初始化OV7725模块
 * @return uint8_t OV7725_OK: 初始化成功, OV7725_ERROR: 初始化失败
 */
uint8_t ov7725_Init(void)
{
    uint16_t mid;
    uint16_t pid;

    OV7725_WRST(1);
    OV7725_RRST(1);
    OV7725_OE(1);
    OV7725_RCLK(1);
    OV7725_WEN(1);

    IICInit(OV7725_Camera.bus);
    ov7725_reset();
    ov7725_get_id(&mid, &pid);
    if (mid != OV7725_MID_DEVICE || pid != OV7725_PID_DEVICE)
    {
        CAW_LOG_ERROR("OV7725_Init Error! : mid = 0x%04X, pid = 0x%04X", mid, pid);
        return OV7725_ERROR;
    }
    else
    {
        CAW_LOG_INFO("OV7725_Init Success! : mid = 0x%04X, pid = 0x%04X", mid, pid);
    }

    ov7725_init_reg();
    return OV7725_OK;
}

/**
 * @brief 获取OV7725 D0-D7一字节数据
 * @param 无
 * @return uint8_t 读取到的数据
 */
static inline uint8_t ov7725_get_byte_data(void)
{
    return (uint8_t)(D0_GPIO_Port->IDR & 0x00FF);
}

/**
 * @brief 获取OV7725模块输出的一帧图像数据
 * @param dts : 图像数据返回首地址
 * @param type: OV7725_GET_FRAME_TYPE_NOINC   : 目的地址固定
 *              OV7725_GET_FRAME_TYPE_AUTO_INC: 目的地址自动自增
 * @return uint8_t OV7725_OK: 获取成功, OV7725_EINVAL: 参数错误, OV7725_EEMPTY: 图像数据为空
 */
uint8_t ov7725_Get_Frame_Data(volatile uint16_t *dts, ov7725_get_frame_type_t type)
{
    uint16_t width_index;
    uint16_t height_index;
    uint16_t dat;

    OV7725_RRST(0);
    OV7725_RCLK(0);
    OV7725_RCLK(1);
    OV7725_RCLK(0);
    OV7725_RRST(1);
    OV7725_RCLK(1);

    for (height_index = 0; height_index < OV7725_Camera.image_height; height_index++)
    {
        for (width_index = 0; width_index < OV7725_Camera.image_width; width_index++)
        {
            OV7725_RCLK(0);
            dat = (ov7725_get_byte_data() << 8);
            OV7725_RCLK(1);
            OV7725_RCLK(0);
            dat |= ov7725_get_byte_data();
            OV7725_RCLK(1);
            *dts = dat;
            switch (type)
            {
                case OV7725_GET_FRAME_TYPE_NOINC:
                {
                    break;
                }
                case OV7725_GET_FRAME_TYPE_AUTO_INC:
                {
                    dts++;
                    break;
                }
                default:
                {
                    CAW_LOG_WARN("ov7725 Invalid Param: type = %d", type);
                    return OV7725_EINVAL;
                }
            }
        }
    }
    
    return OV7725_OK;
}

/**
 * @brief 开始读取OV7725帧数据（初始化FIFO读取）
 * @return uint8_t OV7725_OK: 成功, OV7725_EEMPTY: 无帧数据
 */
uint8_t ov7725_Frame_Read_Start(void)
{
    OV7725_RRST(0);
    OV7725_RCLK(0);
    OV7725_RCLK(1);
    OV7725_RCLK(0);
    OV7725_RRST(1);
    OV7725_RCLK(1);

    return OV7725_OK;
}

/**
 * @brief 读取OV7725帧数据块
 * @param buffer: 数据缓冲区指针（uint8_t类型）
 * @param pixel_count: 要读取的像素数量
 * @return uint16_t 实际读取的像素数量
 */
uint16_t ov7725_Frame_Read_Chunk(uint8_t *buffer, uint16_t pixel_count)
{
    uint16_t i;
    for (i = 0; i < pixel_count; i++)
    {
        OV7725_RCLK(0);
        *buffer++ = ov7725_get_byte_data();  /* 高字节 */
        OV7725_RCLK(1);
        OV7725_RCLK(0);
        *buffer++ = ov7725_get_byte_data();  /* 低字节 */
        OV7725_RCLK(1);
    }
    return i;
}

volatile OV7725_Capture_State_t capture_state = CAPTURE_IDLE;
static uint32_t capture_done_timestamp = 0;
static volatile uint8_t pending_frame_count = 0;  /* 待处理帧计数 */

/**
 * @brief 结束读取OV7725帧数据（双帧缓冲版本）
 * @note 读取完成后，如果有待处理帧，继续处理；否则允许新采集
 */
void ov7725_Frame_Read_End(void)
{
    if (pending_frame_count > 0)
    {
        /* 还有待处理帧，继续保持CAPTURE_DONE状态 */
        pending_frame_count--;
        if (pending_frame_count == 0)
        {
            capture_state = CAPTURE_IDLE;
        }
    }
    else
    {
        capture_state = CAPTURE_IDLE;
    }
}

/**
 * @brief 检查是否有帧数据可读
 * @return 1: 有数据可读, 0: 无数据
 */
uint8_t ov7725_Frame_Available(void)
{
    return (capture_state == CAPTURE_DONE);
}

/**
 * @brief OV7725 VSYNC外部中断服务函数（双帧缓冲版本）
 * @param GPIO_Pin: 触发中断的GPIO引脚
 * @note 支持连续采集：在读取帧1时可以同时采集帧2
 *       FIFO容量384KB，QVGA帧150KB，可存储2帧
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == VSYNC_Pin)
    {
        switch (capture_state)
        {
            case CAPTURE_IDLE:
            {
                /* 空闲状态：开始采集新帧 */
                OV7725_WRST(0);
                OV7725_WEN(1);
                OV7725_WRST(1);
                capture_state = CAPTURE_ONGOING;
                break;
            }
            case CAPTURE_ONGOING:
            {
                /* 采集中：一帧录入完成 */
                OV7725_WEN(0);
                capture_done_timestamp = HAL_GetTick();
                capture_state = CAPTURE_DONE;
                pending_frame_count = 0;
                break;
            }
            case CAPTURE_DONE:
            {
                if (HAL_GetTick() - capture_done_timestamp > 5000)
                {
                    capture_state = CAPTURE_IDLE;
                    pending_frame_count = 0;
                    CAW_LOG_WARN("Frame processing timeout! Reset to IDLE.");
                }
                break;
            }
            case CAPTURE_BUFFERING:
            {
                /* 缓冲采集中：第二帧录入完成 */
                OV7725_WEN(0);
                pending_frame_count++;
                capture_state = CAPTURE_DONE;
                break;
            }
        }
    }
}
