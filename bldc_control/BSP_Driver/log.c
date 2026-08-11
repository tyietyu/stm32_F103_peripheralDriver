/*
 * @Author: Rick rick@guaik.io
 * @Date: 2022-12-09 14:15:44
 * @LastEditors: Rick rick@guaik.io
 * @LastEditTime: 2023-01-09 14:15:44
 * @FilePath: BSP_Driver/log.c
 * @Description:
 * Copyright (c) 2022 by Rick email: rick@guaik.io, All Rights Reserved.
 */
#include "log.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

CAW_LOG_T G_CAW_LOG_Instance = {NULL, false};

int CAW_LOG_Init(UART_HandleTypeDef *huart, CAW_LOG_LEVEL level, bool enable_color)
{
  G_CAW_LOG_Instance.uart_ins = huart;
  G_CAW_LOG_Instance.enable_color = enable_color;
  G_CAW_LOG_Instance.level = level;
  return 0;  
}

/* __FILE__ 在 MDK 下是编译时的全路径，直接进日志会挤占正文并拉长发送时间 */
static const char *CAW_LOG_BaseName(const char *path)
{
  const char *base = path;
  const char *p;

  if (path == NULL)
  {
    return "";
  }

  for (p = path; *p != '\0'; ++p)
  {
    if ((*p == '/') || (*p == '\\'))
    {
      base = p + 1;
    }
  }
  return base;
}

void CAW_LOG_Write(const char *fmt, CAW_LOG_LEVEL level, const char *file, int line, const char *func, ...)
{
  if (level < G_CAW_LOG_Instance.level)
    return;
  char tmp[128];
  char buf[192];
  const char *name = CAW_LOG_BaseName(file);
  va_list args;
  va_start(args, func);
  /* 必须用 vsnprintf：格式化结果长度不可控，vsprintf 会直接冲掉相邻栈变量 */
  vsnprintf(tmp, sizeof(tmp), fmt, args);
  va_end(args);
  while (HAL_UART_GetState(G_CAW_LOG_Instance.uart_ins) != HAL_UART_STATE_READY)
    ;
  if (level == LEVEL_DEBUG)
  {
    if (G_CAW_LOG_Instance.enable_color)
      snprintf(buf, sizeof(buf),
               "\033[0;36mCAW-[DEBUG] <%s:%d> <%s>: %s\033[m\r\n", name,
               line, func, tmp);
    else
      snprintf(buf, sizeof(buf), "CAW-[DEBUG] <%s:%d> <%s>: %s\r\n", name,
               line, func, tmp);
  }
  else if (level == LEVEL_INFO)
  {
    if (G_CAW_LOG_Instance.enable_color)
      snprintf(buf, sizeof(buf),
               "\033[0;32mCAW-[INFO] <%s:%d> <%s>: %s\033[m\r\n", name,
               line, func, tmp);
    else
      snprintf(buf, sizeof(buf), "CAW-[INFO] <%s:%d> <%s>: %s\r\n", name,
               line, func, tmp);
  }
  else if (level == LEVEL_WARN)
  {
    if (G_CAW_LOG_Instance.enable_color)
      snprintf(buf, sizeof(buf),
               "\033[0;33mCAW-[WARN] <%s:%d> <%s>: %s\033[m\r\n", name,
               line, func, tmp);
    else
      snprintf(buf, sizeof(buf), "CAW-[WARN] <%s:%d> <%s>: %s\r\n", name,
               line, func, tmp);
  }
  else if (level == LEVEL_ERROR)
  {
    if (G_CAW_LOG_Instance.enable_color)
      snprintf(buf, sizeof(buf),
               "\033[0;31mCAW-[ERROR] <%s:%d> <%s>: %s\033[m\r\n", name,
               line, func, tmp);
    else
      snprintf(buf, sizeof(buf), "CAW-[ERROR] <%s:%d> <%s>: %s\r\n", name,
               line, func, tmp);
  }
  else
  {
    return;
  }

  /* 只发实际字符，不发结尾的 '\0' */
  HAL_UART_Transmit_IT(G_CAW_LOG_Instance.uart_ins, (uint8_t *)buf, strlen(buf));
  while (HAL_UART_GetState(G_CAW_LOG_Instance.uart_ins) == HAL_UART_STATE_BUSY_TX)
    ;
}
