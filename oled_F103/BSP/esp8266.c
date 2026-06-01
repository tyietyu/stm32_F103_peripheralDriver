#include "esp8266.h"
#include "core_json.h"
#include "usart.h"
#include "otadriver.h"

extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_rx;

#define ESP8266_MQTT_PAYLOAD_SIZE 256U
#define ESP8266_MQTT_TOPIC_SIZE 128U
#define ESP8266_MQTT_ESCAPED_TOPIC_SIZE (ESP8266_MQTT_TOPIC_SIZE * 2U)
#define ESP8266_WIFI_SSID_SIZE 33U
#define ESP8266_WIFI_PASSWORD_SIZE 65U
#define ESP8266_WIFI_ESCAPED_SSID_SIZE (ESP8266_WIFI_SSID_SIZE * 2U)
#define ESP8266_WIFI_ESCAPED_PASSWORD_SIZE (ESP8266_WIFI_PASSWORD_SIZE * 2U)
#define ESP8266_AT_CMD_MAX_SIZE 256U
#define ESP8266_MQTT_PUBLISH_TIMEOUT_MS 10000U

static uint8_t ESP8266_restore(void);
static uint8_t ESP8266_sw_reset(void);
static uint8_t ESP8266_escape_at_quoted_string(const char *src, char *dst, size_t dst_size);
static uint8_t ESP8266_wait_for_ack(const char *ack, uint16_t timeout_ms);
void ESP8266_uart_rx_clear(uint16_t len);

static uint8_t ESP8266_escape_at_quoted_string(const char *src, char *dst, size_t dst_size)
{
    size_t write_index = 0U;

    if ((src == NULL) || (dst == NULL) || (dst_size == 0U)) {
        return ESP8266_EINVAL;
    }

    while (*src != '\0') {
        char ch = *src++;

        if ((ch == '\r') || (ch == '\n')) {
            dst[0] = '\0';
            return ESP8266_EINVAL;
        }

        if ((ch == '"') || (ch == '\\') || (ch == ',')) {
            if ((write_index + 2U) >= dst_size) {
                dst[0] = '\0';
                return ESP8266_EINVAL;
            }
            dst[write_index++] = '\\';
        } else {
            if ((write_index + 1U) >= dst_size) {
                dst[0] = '\0';
                return ESP8266_EINVAL;
            }
        }

        dst[write_index++] = ch;
    }

    dst[write_index] = '\0';
    return ESP8266_EOK;
}

static uint8_t ESP8266_wait_for_ack(const char *ack, uint16_t timeout_ms)
{
    uint16_t count = 0U;

    if ((ack == NULL) || (timeout_ms == 0U)) {
        return ESP8266_EINVAL;
    }

    while (count < timeout_ms) {
        if (uart_init.esp8266_buffer.receive_start == 1U) {
            if (strstr((const char *)uart_init.esp8266_buffer.process_buff, ack) != NULL) {
                return ESP8266_EOK;
            }
            if ((strstr((const char *)uart_init.esp8266_buffer.process_buff, "ERROR") != NULL) ||
                (strstr((const char *)uart_init.esp8266_buffer.process_buff, "FAIL") != NULL)) {
                ESP8266_uart_rx_clear(uart_init.esp8266_buffer.receive_count);
                return ESP8266_ERROR;
            }
            ESP8266_uart_rx_clear(uart_init.esp8266_buffer.receive_count);
        }

        count++;
        uart_init.delay_ms(1);
    }

    return ESP8266_TIMEOUT;
}


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
    .esp8266_buffer.process_buff = {0},
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
    if (len > ESP8266_RX_BUF_SIZE) {
        len = ESP8266_RX_BUF_SIZE;
    }
    if (len == 0U) {
        len = ESP8266_RX_BUF_SIZE;
    }

    memset((void *)uart_init.esp8266_buffer.receive_buff, 0x00, len);
    memset((void *)uart_init.esp8266_buffer.process_buff, 0x00, len);
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
            if(len >= ESP8266_RX_BUF_SIZE) {
                len = ESP8266_RX_BUF_SIZE - 1U;
            }
            if (uart_init.esp8266_buffer.receive_start == 0U) {
                memcpy((void *)uart_init.esp8266_buffer.process_buff,
                       (const void *)uart_init.esp8266_buffer.receive_buff,
                       len);
                uart_init.esp8266_buffer.process_buff[len] = '\0';
                uart_init.esp8266_buffer.receive_count = len;
                uart_init.esp8266_buffer.receive_start = 1;
            }
            memset((void *)uart_init.esp8266_buffer.receive_buff, 0x00, ESP8266_RX_BUF_SIZE);
        }
        HAL_UART_Receive_DMA(huart, uart_init.esp8266_buffer.receive_buff, ESP8266_RX_BUF_SIZE);
    }
}

uint8_t ESP8266_send_at_cmd(unsigned char *cmd, unsigned char len, const char *ack)
{
    uint16_t count = 0;

    if ((cmd == NULL) || (len == 0U) || (ack == NULL)) {
        return ESP8266_EINVAL;
    }

    ESP8266_uart_rx_clear(ESP8266_RX_BUF_SIZE);
    uart_init.uart_send((unsigned char *)cmd, len);

    while ((uart_init.esp8266_buffer.receive_start == 0U) && (count < 1000U))
    {
        count++;
        uart_init.delay_ms(1);
    }

    if(uart_init.esp8266_buffer.receive_start == 1U)
    {
        if (strstr((const char *)uart_init.esp8266_buffer.process_buff, ack))
        {
            ESP8266_uart_rx_clear(uart_init.esp8266_buffer.receive_count);
            return ESP8266_EOK;
        }
        ESP8266_uart_rx_clear(uart_init.esp8266_buffer.receive_count);
    }
    else
    {
        printf("ESP8266_send_at_cmd timeout\r\n");
        ESP8266_uart_rx_clear(uart_init.esp8266_buffer.receive_count);
        return ESP8266_TIMEOUT;
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
    char cmd[ESP8266_AT_CMD_MAX_SIZE];
    char escaped_ssid[ESP8266_WIFI_ESCAPED_SSID_SIZE];
    char escaped_password[ESP8266_WIFI_ESCAPED_PASSWORD_SIZE];
    int cmd_len;

    if ((wifi_ssid == NULL) || (password == NULL)) {
        return ESP8266_EINVAL;
    }

    if ((strlen(wifi_ssid) >= ESP8266_WIFI_SSID_SIZE) ||
        (strlen(password) >= ESP8266_WIFI_PASSWORD_SIZE) ||
        (ESP8266_escape_at_quoted_string(wifi_ssid,
                                         escaped_ssid,
                                         sizeof(escaped_ssid)) != ESP8266_EOK) ||
        (ESP8266_escape_at_quoted_string(password,
                                         escaped_password,
                                         sizeof(escaped_password)) != ESP8266_EOK)) {
        return ESP8266_EINVAL;
    }

    cmd_len = snprintf(cmd,
                       sizeof(cmd),
                       "AT+CWJAP=\"%s\",\"%s\"\r\n",
                       escaped_ssid,
                       escaped_password);
    if ((cmd_len < 0) || ((size_t)cmd_len >= sizeof(cmd))) {
        return ESP8266_EINVAL;
    }

    ESP8266_uart_rx_clear(ESP8266_RX_BUF_SIZE);
    uart_init.uart_send((uint8_t *)cmd, (size_t)cmd_len);
    if (ESP8266_wait_for_ack("OK", ESP8266_MQTT_PUBLISH_TIMEOUT_MS) != ESP8266_EOK) {
        return ESP8266_ERROR;
    }

    ESP8266_uart_rx_clear(uart_init.esp8266_buffer.receive_count);
    return ESP8266_EOK;
}

uint8_t ESP8266_connect_to_cloud(const char *mqtt_name, const char *mqtt_password, const char *mqtt_client_id, const char *broker_address)
{
    char cmd[ESP8266_AT_CMD_MAX_SIZE];
    int cmd_len;

    if ((mqtt_name == NULL) || (mqtt_password == NULL) ||
        (mqtt_client_id == NULL) || (broker_address == NULL)) {
        return ESP8266_EINVAL;
    }

    cmd_len = snprintf(cmd,
                       sizeof(cmd),
                       "AT+MQTTUSERCFG=0,1,\"%s\",\"%s\",\"%s\",0,0,\"\"\r\n",
                       mqtt_client_id,
                       mqtt_name,
                       mqtt_password);
    if ((cmd_len < 0) || ((size_t)cmd_len >= sizeof(cmd))) {
        return ESP8266_EINVAL;
    }

    if (ESP8266_send_at_cmd((uint8_t *)cmd, (unsigned char)cmd_len, "OK") != ESP8266_EOK) {
        return ESP8266_ERROR;
    }

    cmd_len = snprintf(cmd,
                       sizeof(cmd),
                       "AT+MQTTCONN=0,\"%s\",1883,0\r\n",
                       broker_address);
    if ((cmd_len < 0) || ((size_t)cmd_len >= sizeof(cmd))) {
        return ESP8266_EINVAL;
    }

    ESP8266_uart_rx_clear(ESP8266_RX_BUF_SIZE);
    uart_init.uart_send((uint8_t *)cmd, (size_t)cmd_len);
    if (ESP8266_wait_for_ack("OK", ESP8266_MQTT_PUBLISH_TIMEOUT_MS) != ESP8266_EOK) {
        return ESP8266_ERROR;
    }

    ESP8266_uart_rx_clear(uart_init.esp8266_buffer.receive_count);
    return ESP8266_EOK;
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
    uint8_t retval = ESP8266_EOK;
    static uint8_t error_count = 0;
    static char escaped_topic_buf[ESP8266_MQTT_ESCAPED_TOPIC_SIZE];
    char payload_buf[ESP8266_MQTT_PAYLOAD_SIZE];
    unsigned char *msg_buf = uart_init.esp8266_buffer.send_buff;
    va_list args;
    int payload_len;
    int cmd_len;

    if ((topic == NULL) || (msg_format == NULL)) {
        return ESP8266_EINVAL;
    }

    va_start(args, msg_format);
    payload_len = vsnprintf(payload_buf, sizeof(payload_buf), msg_format, args);
    va_end(args);

    if ((payload_len < 0) || ((size_t)payload_len >= sizeof(payload_buf))) {
        return ESP8266_EINVAL;
    }

    if ((strlen(topic) >= ESP8266_MQTT_TOPIC_SIZE) ||
        (ESP8266_escape_at_quoted_string(topic,
                                         escaped_topic_buf,
                                         sizeof(escaped_topic_buf)) != ESP8266_EOK)) {
        return ESP8266_EINVAL;
    }

    cmd_len = snprintf((char *)msg_buf,
                       ESP8266_RX_BUF_SIZE,
                       "AT+MQTTPUBRAW=0,\"%s\",%d,1,0\r\n",
                       escaped_topic_buf,
                       payload_len);
    if ((cmd_len < 0) ||
        ((size_t)cmd_len >= ESP8266_AT_CMD_MAX_SIZE) ||
        ((size_t)cmd_len >= ESP8266_RX_BUF_SIZE)) {
        return ESP8266_EINVAL;
    }

    ESP8266_uart_rx_clear(ESP8266_RX_BUF_SIZE);
    uart_init.uart_send((uint8_t *)msg_buf, (size_t)cmd_len);

    if (ESP8266_wait_for_ack(">", ESP8266_MQTT_PUBLISH_TIMEOUT_MS) != ESP8266_EOK) {
        return ESP8266_ERROR;
    }

    ESP8266_uart_rx_clear(ESP8266_RX_BUF_SIZE);
    uart_init.uart_send((uint8_t *)payload_buf, (size_t)payload_len);

    if (ESP8266_wait_for_ack("+MQTTPUB:OK", ESP8266_MQTT_PUBLISH_TIMEOUT_MS) == ESP8266_EOK) {
        retval = ESP8266_EOK;
        error_count = 0;
    } else {
        retval = ESP8266_ERROR;
        error_count++;

        if (error_count == 5)
        {
            error_count = 0;
            printf("RECONNECT MQTT BROKER!!!\r\n");
        }
    }

    ESP8266_uart_rx_clear(uart_init.esp8266_buffer.receive_count);
    return retval;
}

static uint8_t parse_json_msg(uint8_t *json_msg, uint16_t json_len)
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
    uint8_t retval = ESP8266_ERROR;
    char *rx_buff = (char *)uart_init.esp8266_buffer.process_buff;
    MQTT_MsgType_t msg_type = MQTT_MSG_TYPE_NORMAL; // 默认为普通消息
    char *ptr;
    char *topic_start;
    char *topic_end;
    char recv_topic[128] = {0};
    uint16_t topic_len;
    char *len_start;
    char *payload_comma;
    long payload_len;
    char *data_start;
    uint32_t data_offset;

    if (uart_init.esp8266_buffer.receive_start != 1U) {
        return ESP8266_ERROR;
    }

    uart_init.esp8266_buffer.receive_start = 0U;

    ptr = strstr(rx_buff, "+MQTTSUBRECV:");
    if (ptr == NULL) {
        goto cleanup;
    }

    topic_start = strchr(ptr, '"');
    if (topic_start == NULL) {
        goto cleanup;
    }
    topic_start++;

    topic_end = strchr(topic_start, '"');
    if (topic_end == NULL) {
        goto cleanup;
    }

    topic_len = (uint16_t)(topic_end - topic_start);
    if (topic_len >= sizeof(recv_topic)) {
        topic_len = sizeof(recv_topic) - 1U;
    }
    memcpy(recv_topic, topic_start, topic_len);

    if ((topic != NULL) && (strcmp(topic, recv_topic) != 0)) {
        goto cleanup;
    }

    len_start = topic_end + 2;
    payload_comma = strchr(len_start, ',');
    if (payload_comma == NULL) {
        goto cleanup;
    }

    payload_len = strtol(len_start, NULL, 10);
    if ((payload_len <= 0L) || (payload_len > UINT16_MAX)) {
        goto cleanup;
    }

    data_start = payload_comma + 1;
    data_offset = (uint32_t)(data_start - rx_buff);
    if ((data_start < rx_buff) ||
        ((data_offset + (uint32_t)payload_len) > uart_init.esp8266_buffer.receive_count)) {
        goto cleanup;
    }

    if ((strstr(recv_topic, "thing/file/download_reply") != NULL) ||
        (strstr(recv_topic, "ota/device/upgrade") != NULL)) {
        msg_type = MQTT_MSG_TYPE_OTA_NOTIFY;
    }

    switch (msg_type)
    {
        case MQTT_MSG_TYPE_OTA_NOTIFY:
            OTA_Process_MQTT_Msg(recv_topic, (uint8_t *)data_start, (uint16_t)payload_len);
            retval = ESP8266_EOK;
            break;

        case MQTT_MSG_TYPE_NORMAL:
            if ((msg_data == NULL) || (msg_len == 0U)) {
                goto cleanup;
            }
            if ((uint32_t)payload_len >= msg_len) {
                payload_len = (long)msg_len - 1L;
            }
            memcpy(msg_data, data_start, (size_t)payload_len);
            msg_data[payload_len] = '\0';
            retval = parse_json_msg(msg_data, (uint16_t)payload_len);
            break;

        default:
            break;
    }

cleanup:
    ESP8266_uart_rx_clear(uart_init.esp8266_buffer.receive_count);
    return retval;
}

