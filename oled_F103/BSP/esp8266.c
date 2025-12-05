#include "esp8266.h"
#include "core_json.h"
#include "usart.h"
#include "otadriver.h"

extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_rx;

static uint8_t ESP8266_restore(void);
static uint8_t ESP8266_sw_reset(void);

// 初始化配置
esp8266_config_t esp8266_config = {
    .wifi = {
        .ssid = "XiaomiPro",
        .password = "123456789l"},
    .device_info = {
        .product_key = "k1644sbngGw", 
        .device_name = "AT_MQTT", 
        .device_secret = "1038f2eead281b6e90427a69d9cd532b"},
    .mqtt = {
        .username = "AT_MQTT&k1644sbngGw", 
        .client_id = "k1644sbngGw.AT_MQTT|securemode=2\\,signmethod=hmacsha256\\,timestamp=1722604000001|", 
        .password = "35d93f05a230fb364ac51438ed45f67c42d0c0fd4a2f792298297d106afdbad3", 
        .broker_address = "k1644sbngGw.iot-as-mqtt.cn-shanghai.aliyuncs.com"},
    .mqtt_topics = {
        .sub_topic = "/k1644sbngGw/AT_MQTT/user/get", 
        .pub_topic = "/sys/k1644sbngGw/AT_MQTT/thing/event/property/post", 
        .json_format = "{\"params\":{\"esp8266_adc_data\":%d,\"LED\":%d},\"version\":\"1.0.0\"}", 
        .json_format_firmware = "{\"id\":\"0011\",\"params\":{\"version\":\"1.0.0\"}}", 
        .device_attributes = "/sys/k1644sbngGw/AT_MQTT/thing/service/property/set"},
    .ota = {
        .upload_info_pub = "/ota/device/inform/k1644sbngGw/AT_MQTT", 
        .download_info_sub = "/ota/device/upgrade/k1644sbngGw/AT_MQTT", 
        .device_active_info_pub = "/sys/k1644sbngGw/AT_MQTT/thing/ota/firmware/get", 
        .device_report_progress_pub = "/ota/device/progress/k1644sbngGw/AT_MQTT", 
        .device_download_file = "/sys/k1644sbngGw/AT_MQTT/thing/file/download_reply", 
        .device_download_file_reply = "/sys/k1644sbngGw/AT_MQTT/thing/file/download"}
};

uart_init_t uart_init = {
    .uart_port = &huart2,
    .delay_ms = HAL_Delay,
    .uart_send = hal_uart_send,
    .uart_receive = hal_uart_receive,
    .esp8266_buffer.send_buff = {0},
    .esp8266_buffer.receive_buff = {0},
    .esp8266_buffer.receive_start = 0,
    .esp8266_buffer.receive_count = 0,
};

esp8266_init_t esp8266_init = {
    .init = ESP8266_init,
    .esp8266_sw_reset = ESP8266_sw_reset,
    .esp8266_restore = ESP8266_restore,
    .connect_wifi = ESP8266_connect_wifi,
    .login_to_cloud = ESP8266_connect_to_cloud,
};

void ESP8266_uart_rx_clear(uint16_t len)
{
    memset((void *)uart_init.esp8266_buffer.receive_buff, 0x00, len);
    uart_init.esp8266_buffer.receive_count = 0;
    uart_init.esp8266_buffer.receive_start = 0;
}

void hal_uart_send(uint8_t *data, size_t length)
{
    if (data == NULL || length == 0)
        return;
    HAL_UART_Transmit(&huart2, data, length, 1000);
}

void hal_uart_receive(uint8_t *data, size_t length)
{
    if (data == NULL || length == 0)
        return;
    HAL_UART_Receive(&huart2, data, length, 1000);
}

void hal_uart2_receiver_handle(UART_HandleTypeDef *huart)
{
    if(__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) != RESET)
    {
        __HAL_UART_CLEAR_FLAG(huart, UART_FLAG_IDLE);
        HAL_UART_DMAStop(huart);
        uint16_t len = ESP8266_RX_BUF_SIZE - __HAL_DMA_GET_COUNTER(huart->hdmarx);
        if(len > 0)
        {
            uart_init.esp8266_buffer.receive_count = len;
            uart_init.esp8266_buffer.receive_start = 1;
            if(len < ESP8266_RX_BUF_SIZE) 
            {
                uart_init.esp8266_buffer.receive_buff[len] = '\0';
            }
        }
        HAL_UART_Receive_DMA(huart, uart_init.esp8266_buffer.receive_buff, ESP8266_RX_BUF_SIZE);
    }
}

uint8_t ESP8266_send_at_cmd(unsigned char *cmd, unsigned char len, const char *ack)
{
    uart_init.uart_send((unsigned char *)cmd, len);
    if(uart_init.esp8266_buffer.receive_start == 1)
    {
        if (strstr((const char *)uart_init.esp8266_buffer.receive_buff, ack))
        {
            ESP8266_uart_rx_clear(uart_init.esp8266_buffer.receive_count);
            return ESP8266_EOK;
        }
    }
    else
    {
        printf("ESP8266_send_at_cmd error\r\n");
        ESP8266_uart_rx_clear(uart_init.esp8266_buffer.receive_count);
        return ESP8266_ERROR;
    }
    return ESP8266_ERROR;
}

static uint8_t ESP8266_sw_reset(void)
{
    const char *cmd = "AT+RST\r\n";
    return ESP8266_send_at_cmd((unsigned char *)cmd, strlen(cmd), "OK"); // 可根据需要调整超时时间
}

static uint8_t ESP8266_restore(void)
{

    const char *cmd = "AT+RESTORE\r\n";
    return ESP8266_send_at_cmd((unsigned char *)cmd, strlen(cmd), "OK"); // 可根据需要调整超时时间
}

static uint8_t ESP8266_set_mode(uint8_t mode)
{
    const char *cmd_template = "AT+CWMODE=%d\r\n";
    char cmd[20];
    uint8_t ret;

    if (mode >= 1 && mode <= 3) // 根据模式设置命令
    {
        snprintf(cmd, sizeof(cmd), cmd_template, mode);
        ret = ESP8266_send_at_cmd((uint8_t *)cmd, strlen(cmd), "OK");
    }
    else
    {
        return ESP8266_EINVAL;
    }
    return (ret == ESP8266_EOK) ? ESP8266_EOK : ESP8266_ERROR;
}

static uint8_t ESP8266_ate_config(uint8_t cfg)
{
    const char *cmd;
    switch (cfg)
    {
    case 0:
        cmd = "ATE0\r\n"; // 关闭回显
        break;
    case 1:
        cmd = "ATE1\r\n"; // 打开回显
        break;
    default:
        return ESP8266_EINVAL; // 返回无效参数错误
    }
    return ESP8266_send_at_cmd((unsigned char *)cmd, strlen(cmd), "OK");
}

uint8_t ESP8266_set_unvarnished_mode(uint8_t enter)
{
    uint8_t ret;
    if (enter)
    {
        // 发送AT命令以进入透传模式
        ret = ESP8266_send_at_cmd((uint8_t *)"AT+CIPMODE=1\r\n", strlen("AT+CIPMODE=1\r\n"), "OK");
        if (ret != ESP8266_EOK)
        {
            return ESP8266_ERROR; // 若第一个命令失败，返回错误
        }
        ret = ESP8266_send_at_cmd((uint8_t *)"AT+CIPSEND\r\n", strlen("AT+CIPSEND\r\n"), ">");
    }
    else
    {
        char *cmd = "+++";
        hal_uart_send((uint8_t *)cmd, strlen(cmd));
        return ESP8266_EOK;
    }

    // 返回结果
    return (ret == ESP8266_EOK) ? ESP8266_EOK : ESP8266_ERROR;
}

uint8_t ESP8266_connect_wifi(const char *wifi_ssid, const char *password)
{
    uint8_t retval = 1; // 默认为失败状态
    uint16_t count = 0;

    // 发送连接WiFi的AT命令
    char cmd[100]; // 预留足够的空间来构建AT命令
    snprintf(cmd, sizeof(cmd), "AT+CWJAP=\"%s\",\"%s\"\r\n", wifi_ssid, password);
    uart_init.uart_send((unsigned char *)cmd, strlen(cmd));

    while ((uart_init.esp8266_buffer.receive_start == 0) && (count < 1000))
    {
        count++;
        uart_init.delay_ms(1);
    }
    // 检查是否超时
    if (count < 1000)
    {
        uart_init.delay_ms(5000); // 等待连接WiFi的时间
        if (strstr((const char *)uart_init.esp8266_buffer.receive_buff, "OK"))
        {
            retval = 0; // 成功连接
        }
    }
    // 清理接收缓冲区
    ESP8266_uart_rx_clear(uart_init.esp8266_buffer.receive_count);
    return retval; // 返回结果
}

uint8_t ESP8266_connect_to_cloud(const char *mqtt_name, const char *mqtt_password, const char *mqtt_client_id, const char *broker_address)
{
    uint8_t retval = 0;
    uint16_t count = 0;

    // 1. 发送 MQTT 用户配置
    char cmd1[128];
    snprintf(cmd1, sizeof(cmd1), "AT+MQTTUSERCFG=0,1,\"NULL\",\"%s\",\"%s\",0,0,\"\"\r\n",
             mqtt_name, mqtt_password);
    uart_init.uart_send((unsigned char *)cmd1, strlen(cmd1));
    uart_init.delay_ms(10);

    count = 0;
    while ((uart_init.esp8266_buffer.receive_start == 0) && (count < 1000))
    {
        count++;
        uart_init.delay_ms(1);
    }
    if (count >= 1000 || strstr((const char *)uart_init.esp8266_buffer.receive_buff, "OK") == NULL)
    {
        return 1;
    }
    ESP8266_uart_rx_clear(uart_init.esp8266_buffer.receive_count);

    // 2. 发送 MQTT 连接命令
    char cmd2[128];
    snprintf(cmd2, sizeof(cmd2), "AT+MQTTCONN=0,\"%s\"\r\n", mqtt_client_id);
    uart_init.uart_send((unsigned char *)cmd2, strlen(cmd2));

    count = 0;
    while ((uart_init.esp8266_buffer.receive_start == 0) && (count < 1000))
    {
        count++;
        uart_init.delay_ms(1);
    }
    if (count >= 1000 || strstr((const char *)uart_init.esp8266_buffer.receive_buff, "OK") == NULL)
    {
        return 1; // 返回错误
    }

    ESP8266_uart_rx_clear(uart_init.esp8266_buffer.receive_count);

    // 3. 发送 TCP 连接命令
    char cmd3[128];
    snprintf(cmd3, sizeof(cmd3), "AT+MQTTCONN=0,\"%s\",1883,0\r\n", broker_address);
    uart_init.uart_send((unsigned char *)cmd3, strlen(cmd3));

    count = 0;
    while ((uart_init.esp8266_buffer.receive_start == 0) && (count < 1000))
    {
        count++;
        uart_init.delay_ms(1);
    }

    if (count >= 1000 || strstr((const char *)uart_init.esp8266_buffer.receive_buff, "OK") == NULL)
    {
        return 1;
    }
    ESP8266_uart_rx_clear(uart_init.esp8266_buffer.receive_count);
    return retval;
}

uint8_t ESP8266_init(uint8_t esp8266_mode, uint8_t esp8266_cfg)
{
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
    HAL_UART_Receive_DMA(&huart2, uart_init.esp8266_buffer.receive_buff, ESP8266_RX_BUF_SIZE);

    const char *at_cmd = "AT\r\n";
    const char *expected_ack = "OK";
    uint8_t max_attempts = 10;
    uint8_t retry = 0;
    uint8_t at_ok = 0;

    for (uint8_t i = 0; i < max_attempts; i++)
    {
        if (ESP8266_send_at_cmd((uint8_t *)at_cmd, strlen(at_cmd), expected_ack) == ESP8266_EOK)
        {
            at_ok = 1;
            break;
        }
        uart_init.delay_ms(500);
    }
    if(!at_ok)return ESP8266_ERROR;

    retry = 0;
    while (ESP8266_set_mode(esp8266_mode))
    {
        if (++retry > 5) return ESP8266_ERROR;
        uart_init.delay_ms(500);
    }

    retry = 0;
    while (ESP8266_ate_config(esp8266_cfg))
    {
        if (++retry > 5) return ESP8266_ERROR;
        uart_init.delay_ms(500);
    }

    retry = 0;
    while (ESP8266_connect_wifi(esp8266_config.wifi.ssid, esp8266_config.wifi.password))
    {
        printf("WiFi Connect Failed, Retrying... %d/5\r\n", retry+1);
        if (++retry > 5) return ESP8266_ERROR;
        uart_init.delay_ms(500);
    }

    retry = 0;
    while (ESP8266_connect_to_cloud(esp8266_config.mqtt.username, 
                                    esp8266_config.mqtt.password, 
                                    esp8266_config.mqtt.client_id, 
                                    esp8266_config.mqtt.broker_address))
    {
        printf("MQTT Connect Failed, Retrying... %d/5\r\n", retry+1);
        if (++retry > 5) return ESP8266_ERROR;
        uart_init.delay_ms(500);
    }
    return ESP8266_EOK;
}

uint8_t ESP8266_send_msg(const char *topic, const char *msg_format, ...)
{
    uint8_t retval = 0;
    uint16_t count = 0;
    static uint8_t error_count = 0;
    unsigned char msg_buf[128];

    va_list args;
    va_start(args, msg_format);
    vsnprintf((char *)msg_buf, sizeof(msg_buf), msg_format, args);
    snprintf((char *)msg_buf, sizeof(msg_buf), "AT+MQTTPUB=0,\"%s\",\"%s\",1,0\r\n", topic, (const char *)msg_buf);
    va_end(args);
    uart_init.uart_send((uint8_t *)msg_buf, strlen((const char *)msg_buf));

    while ((uart_init.esp8266_buffer.receive_start == 0) && (count < 500))
    {
        count++;
        uart_init.delay_ms(1);
    }

    if (count >= 500)
    {
        retval = 1;
    }
    else
    {
        uart_init.delay_ms(50);

        if (strstr((const char *)uart_init.esp8266_buffer.receive_buff, "OK"))
        {
            retval = 0;
            error_count = 0;
        }
        else
        {
            error_count++;

            if (error_count == 5)
            {
                error_count = 0;
                printf("RECONNECT MQTT BROKER!!!\r\n");
            }
        }
    }

    ESP8266_uart_rx_clear(uart_init.esp8266_buffer.receive_count);
    return retval;
}

static uint8_t parse_json_msg(uint8_t *json_msg, uint8_t json_len)
{
    uint8_t retval = 0;
    JSONStatus_t result;
    char query[] = "params.light";
    size_t queryLength = sizeof(query) - 1;
    char *value;
    size_t valueLength;
    result = JSON_Validate((const char *)json_msg, json_len);

    if (result == JSONSuccess)
    {
        result = JSON_Search((char *)json_msg, json_len, query, queryLength, &value, &valueLength);

        if (result == JSONSuccess)
        {
            char save = value[valueLength];
            value[valueLength] = '\0';
            printf("Found: %s %d-> %s\n", query, valueLength, value);
            value[valueLength] = save;
            retval = 0;
        }
        else
        {
            retval = 1;
        }
    }
    else
    {
        retval = 1;
    }

    return retval;
}

uint8_t ESP8266_receive_msg(const char *topic, uint8_t *msg_data, uint16_t msg_len)
{
    uint8_t retval = 0;
    char *rx_buff = (char *)uart_init.esp8266_buffer.receive_buff;
    MQTT_MsgType_t msg_type = MQTT_MSG_TYPE_NORMAL; // 默认为普通消息

    if (uart_init.esp8266_buffer.receive_start == 1)
    {
        uart_init.esp8266_buffer.receive_start = 0;

        char *ptr = strstr(rx_buff, "+MQTTSUBRECV:");
        if (ptr)
        {
            // A. 解析 Topic (提取双引号之间的内容)
            char *topic_start = strchr(ptr, '"');
            if (topic_start == NULL)
			{
				ESP8266_uart_rx_clear(uart_init.esp8266_buffer.receive_count);
			}
            topic_start++; // 跳过左引号

            char *topic_end = strchr(topic_start, '"');
            if (topic_end == NULL)
			{
				ESP8266_uart_rx_clear(uart_init.esp8266_buffer.receive_count);
			}

            // 提取 Topic 到本地缓冲区用于判断
            char recv_topic[128] = {0};
            uint8_t topic_len = topic_end - topic_start;
            if (topic_len >= sizeof(recv_topic)) topic_len = sizeof(recv_topic) - 1;
            memcpy(recv_topic, topic_start, topic_len);

            // B. 解析数据长度
            char *len_start = topic_end + 2; // 跳过 引号和逗号 ",
            int payload_len = atoi(len_start);

            // C. 定位 Payload 数据起始位置 (找到长度后面的逗号)
            char *data_start = strchr(len_start, ',');
            if (data_start == NULL)
			{
				ESP8266_uart_rx_clear(uart_init.esp8266_buffer.receive_count);
			}
            data_start++; // 跳过逗号，指向真实数据

            // === 3. 确定消息类型 ===
            if (strstr(recv_topic, "thing/file/download_reply")) 
            {
                msg_type = MQTT_MSG_TYPE_OTA_NOTIFY;
            }
            else if (strstr(recv_topic, "ota/device/upgrade")) 
            {
                msg_type = MQTT_MSG_TYPE_OTA_NOTIFY;
            }
            else 
            {
                msg_type = MQTT_MSG_TYPE_NORMAL;
            }

            switch (msg_type)
            {
                case MQTT_MSG_TYPE_OTA_NOTIFY:
                    OTA_Process_MQTT_Msg(recv_topic, (uint8_t *)data_start, payload_len);
                    retval = 0;
                    break;

                case MQTT_MSG_TYPE_NORMAL:
                    if (payload_len > msg_len) payload_len = msg_len;
                    memcpy(msg_data, data_start, payload_len);
                    msg_data[payload_len] = '\0';
                    retval = parse_json_msg(msg_data, payload_len);
                    break;
                default:
                    break;
            }
        }
        else
        {
            retval = 1; // 未找到 MQTT 报头
        }
    }
    else
    {
        retval = 1; // 未接收到数据
    }

    ESP8266_uart_rx_clear(uart_init.esp8266_buffer.receive_count);
    return retval;
}

