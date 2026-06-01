# oled_F103 OTA 架构设计：外部 SPI Flash staging + bootloader

## 目标

本设计面向 `oled_F103` 工程，芯片型号为 STM32F103C8T6。当前 OLED 功能已经可运行，OTA 改造必须避免覆盖现有 APP、EEPROM 模拟区和 OLED 相关逻辑。

目标是把 OTA 从当前的“云端通知解析和状态上报框架”扩展为可落地的升级方案：

- APP 通过 ESP8266/MQTT 接收 OTA 通知并按块下载固件。
- 新固件先写入外部 SPI Flash staging 区，不直接写内部 Flash。
- bootloader 在复位后校验 staging 固件，再搬运到内部 APP 区。
- 升级失败时保留当前 APP 可继续运行，并上报失败原因。
- 支持升级成功确认和失败回滚，避免设备变砖。

## 当前工程状态

当前 OTA 已具备：

- `otadriver.c/.h`：OTA 上下文、版本上报、升级通知 JSON 解析、进度/错误上报。
- `esp8266.c/.h`：MQTT 连接、订阅、发布和 `+MQTTSUBRECV` payload 提取。
- `flash.c/.h`：内部 Flash 读写函数。
- `main.c`：初始化后调用 `OTA_Init()`，主循环调用 `OTA_Loop()`。

当前 OTA 未具备：

- 独立 bootloader。
- 外部 SPI Flash 驱动和 staging 分区管理。
- bootloader 与 APP 共享的 OTA 元数据结构。
- staging 固件完整性校验、写后校验、断电恢复。
- APP 链接地址调整、向量表重定位和 bootloader 跳转 APP。
- 升级确认、失败回滚、升级状态持久化。

当前 `OTA_ENABLE_INTERNAL_FLASH_STAGING` 默认为 `0U`，内部 Flash 下载写入路径被禁用。这是正确的安全默认值，不应在 C8T6 上直接打开。

## 设计原则

- 不使用 STM32F103C8T6 内部 Flash 做完整固件 staging。
- APP 只负责下载、校验和写入外部 SPI Flash，不负责覆盖内部 APP 区。
- bootloader 是唯一允许擦写内部 APP 区的模块。
- bootloader 尽量小，功能固定，不依赖 OLED、MQTT、动态内存或复杂业务逻辑。
- OTA 元数据采用固定地址、固定结构、带 magic/version/crc，bootloader 和 APP 共用同一份头文件。
- 所有写 Flash 操作必须有返回值、边界检查、写后校验和可恢复错误路径。

## 推荐 Flash 分区

STM32F103C8T6 内部 Flash 共 64KB，页大小 1KB。推荐内部 Flash 分区如下：

| 区域 | 地址范围 | 建议大小 | 用途 |
| --- | --- | ---: | --- |
| bootloader | `0x08000000` - `0x08003FFF` | 16KB | 上电判断、校验、搬运、跳转 APP |
| APP | `0x08004000` - `0x0800EFFF` | 44KB | 当前 OLED + ESP8266 + OTA APP |
| 参数/EEPROM 保留 | `0x0800F000` - `0x0800FFFF` | 4KB | EEPROM 模拟、OTA 小元数据备份 |

说明：

- 当前工程已把 EEPROM/Flash 保存区避开最后 2KB，但 OTA 后建议统一规划最后 4KB，便于参数和 OTA 状态分离。
- APP 起始地址改为 `0x08004000` 后，APP 工程必须修改链接地址和 `SCB->VTOR`。
- 如果 APP 后续超过 44KB，应优先减功能或更换 STM32F103CB/RB，不建议压缩 bootloader 安全能力。

外部 SPI Flash 分区建议：

| 区域 | 偏移 | 建议大小 | 用途 |
| --- | --- | ---: | --- |
| OTA metadata A | `0x000000` | 4KB | OTA 元数据主副本 |
| OTA metadata B | `0x001000` | 4KB | OTA 元数据备份副本 |
| firmware slot | `0x002000` | >= APP 最大大小 | 新固件 staging |
| block bitmap | slot 后 | 4KB 或按需 | 下载块完成标记 |

外部 SPI Flash 容量建议不小于 1MB。常见 W25Qxx 系列可满足，但代码中不要绑定单一型号，先抽象成 `spi_flash_read/write/erase` 接口。

## 固件格式

上传到云端的固件文件建议使用“固件头 + APP binary”格式。

固件头建议字段：

```c
typedef struct {
    uint32_t magic;          /* 固定值，例如 0x4F544131: "OTA1" */
    uint16_t header_version; /* 结构版本 */
    uint16_t header_size;    /* 头部长度 */
    uint32_t image_size;     /* APP binary 长度 */
    uint32_t image_crc32;    /* APP binary CRC32 */
    uint8_t  image_sha256[32];
    uint32_t app_start_addr; /* 期望 APP 地址，例如 0x08004000 */
    uint32_t hw_version;     /* 硬件兼容版本 */
    uint32_t sw_version;     /* 固件版本号，单调递增 */
    uint32_t flags;
    uint32_t header_crc32;   /* 头部 CRC，计算时该字段按 0 处理 */
} ota_image_header_t;
```

第一阶段可实现 CRC32 + MD5/SHA256 完整性校验；量产安全要求更高时，应增加签名校验。MD5 只能防传输损坏，不能防篡改。

## OTA 元数据

bootloader 和 APP 共享 OTA 元数据。元数据建议同时保存在外部 SPI Flash metadata A/B，内部 Flash 最后参数区可只保存一个简短状态镜像，避免频繁擦写内部 Flash。

建议状态：

```c
typedef enum {
    OTA_SLOT_EMPTY = 0,
    OTA_SLOT_DOWNLOADING,
    OTA_SLOT_READY,
    OTA_SLOT_UPDATING,
    OTA_SLOT_PENDING_CONFIRM,
    OTA_SLOT_CONFIRMED,
    OTA_SLOT_FAILED
} ota_slot_state_t;
```

建议元数据字段：

```c
typedef struct {
    uint32_t magic;
    uint16_t struct_version;
    uint16_t struct_size;
    uint32_t sequence;
    uint32_t state;
    uint32_t image_size;
    uint32_t image_crc32;
    uint8_t  image_sha256[32];
    uint32_t app_start_addr;
    uint32_t sw_version;
    uint32_t downloaded_size;
    uint32_t block_size;
    uint32_t total_blocks;
    uint32_t failed_reason;
    uint32_t metadata_crc32;
} ota_metadata_t;
```

metadata A/B 使用 `sequence` 选择最新有效副本。写入时先写新副本并校验，再切换状态，避免断电导致元数据不可用。

## 模块划分

### bootloader 工程

建议新增独立 Keil target 或独立工程，例如：

- `oled_F103_Bootloader`
- 输出地址：`0x08000000`
- 最大大小：16KB

bootloader 职责：

- 初始化最小 HAL、时钟、CRC、SPI Flash。
- 读取并校验 OTA metadata。
- 当状态为 `OTA_SLOT_READY` 时校验 staging 固件。
- 擦除 APP 区并按页搬运新固件到 `0x08004000`。
- 写后逐页校验内部 Flash。
- 搬运成功后设置 `OTA_SLOT_PENDING_CONFIRM`，跳转 APP。
- 搬运失败后设置 `OTA_SLOT_FAILED`，保留或跳转旧 APP。
- 无升级任务时校验 APP 向量表并跳转 APP。

bootloader 不做：

- OLED 显示。
- WiFi/MQTT 通信。
- JSON 解析。
- 云端状态上报。

### APP 工程

APP 仍使用当前 `oled_F103` 工程，职责：

- 维护 OLED 和现有业务功能。
- 初始化 ESP8266/MQTT。
- 订阅 OTA upgrade topic。
- 解析 OTA 通知。
- 按块请求固件并写入外部 SPI Flash。
- 维护下载 bitmap 和 metadata。
- 完整下载后校验固件头、CRC32、SHA256/MD5。
- 校验通过后设置 `OTA_SLOT_READY` 并复位。
- 新固件首次启动后完成自检，调用 `OTA_Confirm_Image()` 设置 `OTA_SLOT_CONFIRMED`。
- 升级失败或 bootloader 写入失败后读取失败状态并上报云端。

### SPI Flash 驱动

新增独立驱动层，避免 OTA 直接依赖具体芯片型号：

- `spi_flash_init()`
- `spi_flash_read(uint32_t addr, uint8_t *buf, uint32_t len)`
- `spi_flash_write(uint32_t addr, const uint8_t *buf, uint32_t len)`
- `spi_flash_erase_sector(uint32_t addr)`
- `spi_flash_erase_range(uint32_t addr, uint32_t len)`
- `spi_flash_read_id()`

接口必须返回错误码。写入前检查地址范围，写入后至少对 metadata 和当前块做读回校验。

### OTA storage 层

新增 `ota_storage.c/.h`，屏蔽 metadata A/B、firmware slot、bitmap 的读写细节：

- `OTA_Storage_Init()`
- `OTA_Storage_Load_Metadata()`
- `OTA_Storage_Save_Metadata()`
- `OTA_Storage_Write_Block()`
- `OTA_Storage_Read_Image()`
- `OTA_Storage_Set_Block_Done()`
- `OTA_Storage_Is_Block_Done()`
- `OTA_Storage_Clear_Slot()`

APP 和 bootloader 都可复用其中的只读/读写子集。bootloader 不需要 bitmap 功能。

### OTA verify 层

新增 `ota_verify.c/.h`：

- 校验固件头 magic、结构版本、目标地址、硬件版本。
- 校验 `image_size` 不超过 APP 区大小。
- 计算 CRC32。
- 可选计算 SHA256/MD5。
- 返回明确错误码。

如果代码空间紧张，bootloader 第一阶段只做 CRC32；APP 下载完成阶段做更完整的 SHA256/MD5 校验。

## 升级流程

### 正常升级

1. APP 启动 ESP8266/MQTT，并调用 `OTA_Init()` 上报当前版本。
2. 云端下发 OTA upgrade notification。
3. APP 解析 `size/version/sign/streamId/signMethod`。
4. APP 检查外部 SPI Flash 容量、APP 最大尺寸、版本号和硬件兼容性。
5. APP 清空 staging slot，metadata 置为 `OTA_SLOT_DOWNLOADING`。
6. APP 按 `OTA_BLOCK_SIZE` 请求固件块。
7. 每个块写入 SPI Flash，读回校验，通过后设置 bitmap。
8. 全部块完成后，APP 校验固件头和完整镜像。
9. APP metadata 置为 `OTA_SLOT_READY`，上报 100% 或 ready，随后复位。
10. bootloader 发现 `OTA_SLOT_READY`，再次校验 staging 固件。
11. bootloader 擦除 APP 区，搬运固件，写后校验。
12. bootloader metadata 置为 `OTA_SLOT_PENDING_CONFIRM`，跳转新 APP。
13. 新 APP 完成最小自检后调用 `OTA_Confirm_Image()`，metadata 置为 `OTA_SLOT_CONFIRMED`。
14. APP 上报升级成功和新版本。

### 下载中断恢复

- APP 启动时读取 metadata。
- 如果状态是 `OTA_SLOT_DOWNLOADING`，检查 bitmap。
- 对已完成块跳过，对缺失块重新请求。
- 如果 metadata 或 bitmap 校验失败，清空 slot 后重新开始。

### bootloader 搬运失败

- 如果失败发生在擦除 APP 前，保留旧 APP，设置 `OTA_SLOT_FAILED`，跳转旧 APP。
- 如果失败发生在 APP 区已被部分擦除后，旧 APP 可能不可用。为降低风险，bootloader 必须在擦除前完成 staging 全量校验，并确保电源稳定。
- C8T6 内部 Flash 空间不足以保留 A/B APP，因此无法做到严格意义上的内部双 APP 回滚。这里的“回滚”主要指：下载失败不影响旧 APP；搬运失败时尽量避免进入搬运，失败后等待重新烧录或重新搬运。

## APP 地址和启动跳转

APP 起始地址改为 `0x08004000` 后，APP 工程需要：

- 修改 Keil IROM1 start：`0x08004000`。
- 修改 IROM1 size：`0x0000B000`，对应 44KB APP 区。
- 启用 `USER_VECT_TAB_ADDRESS`。
- 设置 `VECT_TAB_OFFSET 0x00004000U`。

bootloader 跳转 APP 逻辑：

```c
typedef void (*app_entry_t)(void);

static void boot_jump_to_app(uint32_t app_addr)
{
    uint32_t app_sp = *(volatile uint32_t *)app_addr;
    uint32_t app_reset = *(volatile uint32_t *)(app_addr + 4U);

    if ((app_sp < 0x20000000U) || (app_sp > 0x20005000U)) {
        return;
    }

    __disable_irq();
    HAL_RCC_DeInit();
    HAL_DeInit();
    SCB->VTOR = app_addr;
    __set_MSP(app_sp);
    ((app_entry_t)app_reset)();
}
```

实际代码中还需要关闭 SysTick、外设中断和 NVIC pending 状态。

## 需要补充或调整的代码

### 新增 bootloader

- `oled_F103_Bootloader/Core/Src/main.c`
- `oled_F103_Bootloader/BSP/boot_jump.c/.h`
- `oled_F103_Bootloader/BSP/boot_ota.c/.h`
- `oled_F103_Bootloader/BSP/boot_flash.c/.h`
- bootloader Keil target 或独立 `.uvprojx`

核心函数：

- `BOOT_OTA_Check()`
- `BOOT_OTA_Verify_Staging()`
- `BOOT_OTA_Program_App()`
- `BOOT_OTA_Set_State()`
- `BOOT_Jump_To_App()`
- `BOOT_Is_Valid_App()`

### 新增共享 OTA 基础层

- `oled_F103/BSP/ota_types.h`
- `oled_F103/BSP/ota_storage.c/.h`
- `oled_F103/BSP/ota_verify.c/.h`
- `oled_F103/BSP/crc32.c/.h`

这些文件应尽量不依赖 APP 业务逻辑，便于 bootloader 复用。

### 新增 SPI Flash 驱动

- `oled_F103/BSP/spi_flash.c/.h`
- 如使用 W25Qxx，可新增 `w25qxx.c/.h` 作为具体实现，但外部只暴露通用 `spi_flash_*` 接口。

需要 CubeMX 或手工确认 SPI 引脚、片选 GPIO、时钟频率和与 OLED/其他外设是否冲突。

### 修改 APP OTA

现有 `oled_F103/BSP/otadriver.c/.h` 需要从“内部 Flash staging”改为“外部 SPI Flash staging”：

- 移除或废弃 `OTA_ENABLE_INTERNAL_FLASH_STAGING` 的内部 Flash 写入路径。
- 下载块写入 `OTA_Storage_Write_Block()`。
- 下载完成后调用 `OTA_Verify_Image()`。
- 校验通过后写 metadata `OTA_SLOT_READY`。
- 新增 `OTA_Confirm_Image()`，APP 首次运行自检通过后确认升级。
- 处理 `OTA_SLOT_FAILED`，上报 bootloader 失败原因。

### 修改 ESP8266/MQTT 接收

`ESP8266_receive_msg()` 需要继续加强：

- 保证 `+MQTTSUBRECV` payload 可承载二进制 `0x00`。
- 按长度解析，不依赖字符串结束符。
- 处理连续两条 MQTT 消息。
- 处理 payload 大于接收缓冲时的拒绝和错误上报。

如果 ESP-AT 的 MQTT 二进制透传不稳定，应改用 HTTP 分块下载或让 ESP8266 侧先下载到自身，再通过 UART 分块传给 STM32。

### 修改内部 Flash 写入接口

`flash.c/.h` 建议补充安全版本，不直接替换旧接口，避免影响现有 EEPROM/OLED 逻辑：

- `int flash_erase_page_checked(uint32_t page_addr)`
- `int flash_write_checked(uint32_t addr, const uint8_t *data, uint32_t len)`
- `int flash_read_checked(uint32_t addr, uint8_t *data, uint32_t len)`
- `int flash_verify(uint32_t addr, const uint8_t *data, uint32_t len)`

bootloader 使用 checked 接口，旧 `iap_write_flash()` 暂时保留给现有功能。

### 修改 APP 工程配置

- IROM1 start 改为 `0x08004000`。
- IROM1 size 改为 `0x0000B000`。
- `system_stm32f1xx.c` 启用向量表重定位。
- 编译产物命名区分 bootloader 和 APP。
- 生成 `.bin` 文件供 OTA 上传。

## 错误码和上报

建议统一 OTA 错误码：

| 错误码 | 含义 |
| ---: | --- |
| -1 | 通用升级失败 |
| -2 | 下载失败 |
| -3 | 校验失败 |
| -4 | 内部 Flash 写入失败 |
| -5 | 外部 Flash 不存在或容量不足 |
| -6 | OTA 消息格式错误 |
| -7 | 固件大小超过 APP 分区 |
| -8 | 硬件版本不匹配 |
| -9 | 版本回退 |
| -10 | bootloader 搬运失败 |

APP 上报时应包含：

- 当前版本。
- 目标版本。
- state。
- percent。
- failed_reason。
- 当前 offset 或 block index。

## 验证计划

### 静态验证

- 检查 bootloader 大小不超过 16KB。
- 检查 APP 链接地址为 `0x08004000`。
- 检查 APP map 中 RO/RW 不越界。
- 检查 OTA 分区常量与文档一致。

### 单元级验证

- metadata A/B 断电模拟：只写一半时能选择旧副本。
- CRC32/SHA256 对固定测试向量正确。
- SPI Flash 写入、读回、擦除边界检查正确。
- bitmap 能正确跳过已下载块。

### 板级验证

- 无升级任务：bootloader 能稳定跳转现有 APP，OLED 正常。
- 下载中断：断电后能续传或重新开始。
- 固件校验失败：不擦内部 APP，旧 APP 继续运行。
- staging 正确：bootloader 搬运后新 APP 运行。
- 新 APP 未确认：复位后进入失败策略，不误报成功。
- 新 APP 确认：状态变为 `OTA_SLOT_CONFIRMED`，云端显示成功。

## 实施顺序

1. 新增共享 `ota_types.h`，固定分区、状态、metadata、错误码。
2. 新增 SPI Flash 驱动并完成读写擦除验证。
3. 新增 `ota_storage`，实现 metadata A/B、firmware slot、bitmap。
4. 新增 `ota_verify` 和 CRC32。
5. 修改 APP 的 OTA 下载路径，把 staging 从内部 Flash 改为 SPI Flash。
6. 新增 bootloader 最小版本：无 OTA 时跳转 APP。
7. 调整 APP 链接地址和向量表，验证 OLED 正常。
8. 给 bootloader 增加 staging 校验和搬运。
9. 增加确认、失败状态和云端上报。
10. 做断电、校验失败、重复升级和回退测试。

## 主要风险

- C8T6 内部 Flash 不能做完整 A/B APP，搬运阶段断电仍有不可完全规避的风险。
- APP 改到 `0x08004000` 后，所有中断依赖向量表重定位，必须重点验证串口 DMA、SysTick、OLED 定时逻辑。
- ESP8266 AT MQTT 对二进制大包的处理需要实测，不稳定时要切换 HTTP 或 ESP 侧缓存方案。
- bootloader 代码空间有限，不能引入复杂日志、JSON、OLED 或 MQTT 逻辑。
- 外部 SPI Flash 和 OLED/SPI/I2C 引脚资源必须在硬件上确认，避免总线冲突。

## 当前代码同步状态

截至 2026-06-01，当前工程已经按本架构落地了第一版 OTA 代码骨架：

- APP 工程 IROM 已调整到 `0x08004000`，size 为 `0x0000B000`。
- `system_stm32f1xx.c` 已启用 `USER_VECT_TAB_ADDRESS`，`VECT_TAB_OFFSET` 为 `0x00004000U`。
- 已新增 `ota_types.h`、`crc32.c/.h`、`spi_flash.c/.h`、`ota_storage.c/.h`、`ota_verify.c/.h`。
- APP 侧 `otadriver.c` 已改为外部 SPI Flash staging，支持下载块写入、bitmap、metadata、MD5 包校验、CRC32 镜像校验、ready 后复位。
- 已新增 `oled_F103/BootLoader` 工程，包含 `boot_jump`、`boot_flash`、`boot_ota`，支持检查 staging、搬运 APP、写后校验和 pending confirm 状态。
- `tools/ota_pack.py` 已用于生成 `ota_image_header_t + APP binary` 格式的 OTA 包。
- APP Keil post-build 已配置为生成 `oled_F103.bin` 和 `oled_F103.ota.bin`。
- ESP8266 UART DMA 接收已增加 `process_buff` 快照，避免 OTA payload 在主循环解析前被下一次 DMA 接收覆盖。

仍需板级确认：

- 外部 SPI Flash 实物型号、容量、供电和 SPI1 引脚连接。
- ESP8266 AT 固件对 `+MQTTSUBRECV` 二进制 payload 的实际长度和分包行为。
- bootloader 烧录后无升级任务能稳定跳转 APP，OLED 和串口 DMA 正常。
- OTA 过程中断电、校验失败、未确认启动等失败路径。
