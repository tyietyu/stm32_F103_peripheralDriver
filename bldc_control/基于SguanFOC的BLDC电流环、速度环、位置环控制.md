# 基于SguanFOC的BLDC电流环、速度环、位置环控制

## 1.目标：实现BLDC电流环、速度环、位置环控制

## 2.已确认硬件的实现：

* 电路原理图文件去读取README.md内容。
* 使用SguanFOC驱动库
* MCU采集U、V、W三相电压使用的是ADC的规则转换组 ，在原理图中的网络标签是：VA_FB,VB_FB,VC_FB,对应ADC1的channel 0,1,2
* MCU采集U、V、W三相电流使用的是ADC的注入转换组，在原理图中的网络标签是：IA_FB,IB_FB,IC_FB,其中IA_FB,IB_FB是DRV8301内部的运放输出，IC_FB是通过LM2904差分运放输出，对应 ADC1的channel 3,4,5
* 母线电压是24V
* PWM驱动使用的是TIM1的channel 1，2，3互补通道，channel 4 是PWM Generation No Output,用于触发adc采样
