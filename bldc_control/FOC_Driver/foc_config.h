#ifndef FOC_CONFIG_H
#define FOC_CONFIG_H

/*
 * FOC 工程的唯一编译期配置入口。
 *
 * 这里的数值必须同时满足原理图、定时器/ADC 配置和电机参数的约束；
 * 需要改动时先改本文件，再同步 .ioc 中无法由 C 头文件表达的枚举项。
 */

/* 环路裁剪：位置环 -> 速度环 -> 电流环。默认只运行已完成保护链路的电流环。 */
#define USE_CURRENT_LOOP                 1
#define USE_SPEED_LOOP                   0
#define USE_POSITION_LOOP                0

#if ((USE_CURRENT_LOOP != 0) && (USE_CURRENT_LOOP != 1))
#error "USE_CURRENT_LOOP must be 0 or 1"
#endif
#if ((USE_SPEED_LOOP != 0) && (USE_SPEED_LOOP != 1))
#error "USE_SPEED_LOOP must be 0 or 1"
#endif
#if ((USE_POSITION_LOOP != 0) && (USE_POSITION_LOOP != 1))
#error "USE_POSITION_LOOP must be 0 or 1"
#endif
#if (USE_SPEED_LOOP && !USE_CURRENT_LOOP)
#error "USE_SPEED_LOOP requires USE_CURRENT_LOOP"
#endif
#if (USE_POSITION_LOOP && !USE_SPEED_LOOP)
#error "USE_POSITION_LOOP requires USE_SPEED_LOOP"
#endif

/* TIM1：F407 APB2 定时器时钟为 168 MHz，中心对齐 PWM。 */
#define CKTIM                              168000000U
#define PWM_PRSC                           0U
#define PWM_FREQ                           15000U
#define PWM_PERIOD                         (CKTIM / (2U * PWM_FREQ * (PWM_PRSC + 1U)))

/* ADC1 注入组：PA3/PA4 两相电流，TIM1_CH4/OC4REF 触发。 */
#define ADC_CHANNEL_NUM                   3U
#define ADC_DATA_LEN                      8U
#define ADC_REF_VOLTAGE                   3.3f
#define ADC_RESOLUTION                    4096.0f
#define FOC_ADC_INJECTED_CHANNELS         2U
#define FOC_ADC_SAMPLE_CYCLES             15U
#define FOC_ADC_CONVERSION_CYCLES         12U
#define FOC_ADC_TIMER_TICKS_PER_CYCLE     8U
#define PWM_ADC_SEQUENCE_TICKS            (FOC_ADC_INJECTED_CHANNELS * \
                                           (FOC_ADC_SAMPLE_CYCLES + \
                                            FOC_ADC_CONVERSION_CYCLES) * \
                                           FOC_ADC_TIMER_TICKS_PER_CYCLE)
#define PWM_ADC_END_MARGIN_TICKS          100U
#define PWM_ADC_PEAK_BLANKING_TICKS       200U
#define PWM_MIN_COMPARE                   PWM_ADC_PEAK_BLANKING_TICKS
#define PWM_MAX_COMPARE                   (PWM_PERIOD - PWM_ADC_SEQUENCE_TICKS - \
                                           PWM_ADC_END_MARGIN_TICKS - \
                                           PWM_ADC_PEAK_BLANKING_TICKS)
#define PWM_ADC_TRIGGER_LATEST            (PWM_PERIOD - PWM_ADC_PEAK_BLANKING_TICKS)

/* ADC 普通组只作三相端电压监控，不进入电流环。 */
#define MONITOR_SAMPLE_FREQ               1000U
#define CURRENT_MONITOR_DECIMATION       (PWM_FREQ / MONITOR_SAMPLE_FREQ)
#define VOLTAGE_DIVIDER_RATIO             8.0f
#define ADC_VOLTAGE_FACTOR                (ADC_REF_VOLTAGE / ADC_RESOLUTION * \
                                           VOLTAGE_DIVIDER_RATIO)

/* DRV8301 电流放大器与分流器：必须与 SPI CTRL2 和硬件 BOM 一致。 */
#define DRV8301_SHUNT_GAIN                10U
#define CURRENT_GAIN                      ((float)DRV8301_SHUNT_GAIN)
#define SHUNT_RESISTOR                    0.01f
#define VOLTAGE_TO_CURRENT                (1.0f / (CURRENT_GAIN * SHUNT_RESISTOR))

/* 电机与传感器参数；方向是启动默认值，ALIGN 会用实测运动方向更新它。 */
#define FOC_MOTOR_SUPPLY_VOLTAGE         24.0f
#define FOC_MOTOR_POLE_PAIRS             11
#define FOC_SENSOR_DIRECTION_DEFAULT      1

/* 电流环和启动限制。PI 参数仍需按实测 R/L 整定，不能由本文件自动推断。 */
#define FOC_INITIAL_IQ_REF               0.02f
#define FOC_IQ_REF_LIMIT                 0.30f
#define FOC_CURRENT_P                    0.30f
#define FOC_CURRENT_I                    0.00f
#define FOC_CURRENT_D                    0.00f
#define FOC_CURRENT_OUTPUT_RAMP          0.0f
#define FOC_CURRENT_VOLTAGE_LIMIT        4.0f
#define FOC_CURRENT_ABS_LIMIT_A          1.0f
#define FOC_OUTER_LOOP_PERIOD_S          0.001f

#if USE_SPEED_LOOP
#define FOC_SPEED_P                      0.10f
#define FOC_SPEED_I                      0.00f
#define FOC_SPEED_D                      0.00f
#define FOC_SPEED_OUTPUT_RAMP            2.00f
#define FOC_SPEED_REFERENCE_LIMIT_RAD_S  10.0f
#endif

#if USE_POSITION_LOOP
#define FOC_POSITION_P                   2.00f
#define FOC_POSITION_I                  0.00f
#define FOC_POSITION_D                  0.00f
#define FOC_POSITION_OUTPUT_RAMP        20.0f
#define FOC_POSITION_SPEED_LIMIT_RAD_S  10.0f
#define FOC_POSITION_REFERENCE_LIMIT_RAD 62.83f
#endif

/* 对齐参数。对齐期间仍受 ADC 软件过流检查保护。 */
#define FOC_ALIGNMENT_VOLTAGE            1.50f
#define FOC_ALIGNMENT_CURRENT_LIMIT_A    1.00f
#define FOC_ALIGNMENT_TIMEOUT_MS         3000U
#define FOC_ALIGNMENT_MOVEMENT_RAD       0.01f
#define FOC_ALIGNMENT_STEPS              20U
#define FOC_ALIGNMENT_STEP_MS            5U
#define FOC_ALIGNMENT_SETTLE_MS          200U

/* 观测器：1 kHz 传感器修正、15 kHz 电角度预测；仅外环模式使用。 */
#define FOC_OBSERVER_BANDWIDTH_HZ        40.0f
#define FOC_OBSERVER_ZETA                0.9f
#define FOC_OBSERVER_CORRECT_TS          1.0e-3f
#define FOC_OBSERVER_OMEGA_LIMIT         200.0f
#define FOC_DEFAULT_SENSOR_AGE           5U

/* 两相电流校准与运行期检查。 */
#define FOC_CURRENT_CALIBRATION_VERSION       1U
#define FOC_CURRENT_OFFSET_MAX_SPAN_COUNTS    64.0f
#define FOC_CURRENT_OFFSET_RAIL_MARGIN_COUNTS 16.0f
#define FOC_CURRENT_RAW_MIN_COUNTS             16.0f
#define FOC_CURRENT_RAW_MAX_COUNTS             4079.0f
#define FOC_CURRENT_SIGN_U                     1.0f
#define FOC_CURRENT_SIGN_V                     1.0f

/* 台架专用的开环符号诊断默认关闭，不能作为正常上电流程的一部分。 */
#define FOC_SIGN_VERIFY_TEST             0
#define FOC_SIGN_VERIFY_VOLTAGE          0.50f
#define FOC_SIGN_VERIFY_SETTLE_MS        200U
#define FOC_SIGN_VERIFY_SAMPLES          100U

/* 周期性能日志默认关闭；UART 阻塞不得进入 1 ms 调度预算。 */
#define FOC_RUNTIME_PERF_LOG             0

/* 采样窗口的保守编译期检查：触发沿到转换结束必须仍在低侧共同导通窗内。 */
#if (FOC_ADC_INJECTED_CHANNELS != 2U)
#error "FOC current feedback expects exactly two injected ADC channels"
#endif
#if (PWM_FREQ == 0U)
#error "PWM_FREQ must be non-zero"
#endif
#if ((MONITOR_SAMPLE_FREQ == 0U) || ((PWM_FREQ % MONITOR_SAMPLE_FREQ) != 0U))
#error "MONITOR_SAMPLE_FREQ must divide PWM_FREQ"
#endif
#if ((FOC_SENSOR_DIRECTION_DEFAULT != 1) && (FOC_SENSOR_DIRECTION_DEFAULT != -1))
#error "FOC_SENSOR_DIRECTION_DEFAULT must be +1 or -1"
#endif
#if (FOC_MOTOR_POLE_PAIRS <= 0)
#error "FOC_MOTOR_POLE_PAIRS must be positive"
#endif
#if (PWM_MIN_COMPARE >= PWM_MAX_COMPARE)
#error "PWM compare limits leave no valid current-sampling window"
#endif
#if ((PWM_ADC_TRIGGER_LATEST + PWM_ADC_SEQUENCE_TICKS) > \
     (2U * PWM_PERIOD - PWM_MAX_COMPARE - PWM_ADC_END_MARGIN_TICKS))
#error "ADC injected sequence overruns the low-side conduction window"
#endif

#endif /* FOC_CONFIG_H */
