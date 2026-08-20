#ifndef __USERDATA_USERCONTROL_H
#define __USERDATA_USERCONTROL_H
#include "SguanFOC.h"
/* 电机控制User用户设置·实时参数控制页面 */

/* 用户自己的CODE BEGIN Includes */
#include "foc_app.h"
/* 用户自己的CODE END Includes */

static inline void User_UserControl(void){
    /* 15 kHz 环内、控制器运算之前调用：落实挂起的模式切换，再把目标值写进库结构体。
       目标值统一在这里落库，避免 UART 中断直接写 double 型 Target_Pos 造成撕裂 */
    FocApp_ApplyTargets();
}

static inline void User_AO_Adjust(float AO){
    /* AO=xx? 按当前 mode 分发：开环 Uq_in / Target_Iq / Target_Speed / Target_Pos */
    FocApp_SetTarget(AO);
}

static inline void User_BO_Adjust(float BO){
    /* BO=xx? 切换运行模式，切换时清目标并复位各控制器历史量 */
    if (BO < 1.0f){
        FocApp_SetMode(Velocity_OPEN_MODE);
    }
    else if (BO < 2.0f){
        FocApp_SetMode(Current_SINGLE_MODE);
    }
    else if (BO < 3.0f){
        FocApp_SetMode(VelCur_DOUBLE_MODE);
    }
    else{
        FocApp_SetMode(PosVelCur_THREE_MODE);
    }
}

static inline void User_CO_Adjust(float CO){
    /* CO=0? 急停锁定；CO=1? 解除锁定进待机（栅极仍关断）；CO=2? 重新初始化并运行 */
    FocApp_SetControlWord(CO);
}

static inline void User_UserTX(void){
    /* 仅传入主循环printf发送的数据，如TXdata.fdata[0],默认最多12个 */
    Sguan.TXdata.fdata[0] = Sguan.status;
    Sguan.TXdata.fdata[1] = Sguan.encoder.Real_Speed;
    Sguan.TXdata.fdata[2] = Sguan.foc.Target_Speed;
    Sguan.TXdata.fdata[3] = Sguan.current.Real_Id;
    Sguan.TXdata.fdata[4] = Sguan.current.Real_Iq;
    Sguan.TXdata.fdata[5] = Sguan.foc.Target_Id;
    Sguan.TXdata.fdata[6] = Sguan.foc.Target_Iq;
    Sguan.TXdata.fdata[7] = Sguan.foc.Uq_in;
    Sguan.TXdata.fdata[8] = Sguan.current.Real_Ia;
    Sguan.TXdata.fdata[9] = Sguan.encoder.Real_Pos;
    Sguan.TXdata.fdata[10] = Sguan.encoder.Pos_offset;
    Sguan.TXdata.fdata[11] = Sguan.mode;
}


#endif // USERDATA_USERCONTROL_H
