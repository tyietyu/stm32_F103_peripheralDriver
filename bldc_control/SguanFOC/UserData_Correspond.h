#ifndef __CORRESPOND_H
#define __CORRESPOND_H
/* 电机控制User用户设置·通信协议设定 */
/* 用户自己的CODE BEGIN Includes */
#include "foc_app.h"
/* 用户自己的CODE END Includes */

/* ================= 驱动代码(驱动层) ================= */
static inline void User_CorrespondSet(unsigned char *ch, unsigned short int size){
    /* USART1 921600 + TX DMA，上一帧未发完则丢帧，不阻塞主循环 */
    FocApp_Send((uint8_t *)ch,(uint16_t)size);
}


#endif // CORRESPOND_H
