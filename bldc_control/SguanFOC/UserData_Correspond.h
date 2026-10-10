#ifndef __CORRESPOND_H
#define __CORRESPOND_H
/* 电机控制User用户设置·通信协议设定 */
/* 用户自己的CODE BEGIN Includes */
#include "foc_app.h"
/* 用户自己的CODE END Includes */

/* ================= 驱动代码(驱动层) ================= */
static inline void User_CorrespondSet(unsigned char *ch, unsigned short int size){
    /* 上位机 JustFloat 协议已关闭：库仍会组帧/printf，但不再送往 USART1。
       速度给定改由 foc_app.c 的 FocApp_SpeedCmd_RadS 直接决定 */
    (void)ch;
    (void)size;
}


#endif // CORRESPOND_H
