#include "mpu6050.h"
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "inv_mpu_dmp_motion_driver.h"
#include "inv_mpu.h"

uint8_t u8_mpu6050_id = 0;

iic_bus_t MPU_bus = {
	.IIC_SDA_PORT = GPIOB,
	.IIC_SCL_PORT = GPIOB,
	.IIC_SDA_PIN  = GPIO_PIN_13,
	.IIC_SCL_PIN  = GPIO_PIN_14,
};

static void soft_iic_init(void)
{
	__HAL_RCC_GPIOB_CLK_ENABLE();
	IICInit(&MPU_bus);
}

static uint8_t IIC_Read_Byte(uint8_t reg)
{
	uint8_t value = 0;
	IIC_Read_Len(MPU_ADDR, reg, 1, &value);
	return value;
}

uint8_t IIC_Write_Len(uint8_t addr,uint8_t reg, uint8_t len, uint8_t *buf)
{
	if(IIC_Write_Multi_Byte(&MPU_bus, addr, reg, len, buf) != HAL_OK)
	{
		return 1;
	}
	return 0;
}

uint8_t IIC_Read_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf)
{
	if(IIC_Read_Multi_Byte(&MPU_bus, addr, reg, len, buf) != HAL_OK)
	{
		return 1;
	}
	return 0;
}

short MPU_Get_Temperature(void)
{
    u8 buf[2]; 
    short raw;
		float temp;
		IIC_Read_Len(MPU_ADDR,MPU_TEMP_OUTH_REG,2,buf); 
    raw=((u16)buf[0]<<8)|buf[1];  
    temp=36.53+((double)raw)/340;  
    return temp*100;;
}

uint8_t MUP6050_Init(void)
{
	soft_iic_init();
	u8_mpu6050_id = IIC_Read_Byte(MPU_DEVICE_ID_REG);
	if(u8_mpu6050_id == MPU_ADDR)
	{
		return 0;
	}
	else
	{
		return 1;
	}
}

#define DEFAULT_MPU_HZ  (100)
struct rx_s {
	unsigned char header[3];
	unsigned char cmd;
};
struct hal_s {
	unsigned char sensors;
	unsigned char dmp_on;
	unsigned char wait_for_tap;
	volatile unsigned char new_gyro;
	unsigned short report;
	unsigned short dmp_features;
	unsigned char motion_int_mode;
	struct rx_s rx;
};
static struct hal_s hal = {0};

/* The sensors can be mounted onto the board in any orientation. The mounting
 * matrix seen below tells the MPL how to rotate the raw data from the
 * driver(s).
 * NOTE: Modify the matrices to match the chip-to-body matrix for your set up.
 */
static signed char gyro_orientation[9] = {-1, 0, 0,
                                           0,-1, 0,
                                           0, 0, 1};

/* Every time new gyro data is available, this function is called in an
 * ISR context. In this example, it sets a flag protecting the FIFO read
 * function.
 */
static void gyro_data_ready_cb(void)
{
	hal.new_gyro = 1;
}

/* These next two functions converts the orientation matrix (see
 * gyro_orientation) to a scalar representation for use by the DMP.
 * NOTE: These functions are borrowed from Invensense's MPL.
 */
static inline unsigned short inv_row_2_scale(const signed char *row)
{
    unsigned short b;

    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7;      // error
    return b;
}
static inline unsigned short inv_orientation_matrix_to_scalar(
    const signed char *mtx)
{
    unsigned short scalar;

    /*
       XYZ  010_001_000 Identity Matrix
       XZY  001_010_000
       YXZ  010_000_001
       YZX  000_010_001
       ZXY  001_000_010
       ZYX  000_001_010
     */

    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;

    return scalar;
}

static void mpu_run_self_test_local(void)
{
    int result;
    long gyro[3], accel[3];

    result = mpu_run_self_test(gyro, accel);
    if (result == 0x7) {
        /* Test passed. We can trust the gyro data here, so let's push it down
         * to the DMP.
         */
        float sens;
        unsigned short accel_sens;
        mpu_get_gyro_sens(&sens);
        gyro[0] = (long)(gyro[0] * sens);
        gyro[1] = (long)(gyro[1] * sens);
        gyro[2] = (long)(gyro[2] * sens);
        dmp_set_gyro_bias(gyro);
        mpu_get_accel_sens(&accel_sens);
        accel[0] *= accel_sens;
        accel[1] *= accel_sens;
        accel[2] *= accel_sens;
        dmp_set_accel_bias(accel);
    }
}

#define INT_EXIT_LPM0 12
uint8_t return_value = 0;

uint8_t DMP_Init(void)
{
	soft_iic_init();

	struct int_param_s int_param;
	unsigned short gyro_rate = 0, gyro_fsr = 0;
	unsigned char accel_fsr = 0;

	/* Set up gyro.
	 * Every function preceded by mpu_ is a driver function and can be found
	 * in inv_mpu.h.
	 */
	int_param.cb = gyro_data_ready_cb;
	int_param.pin = 16;
	int_param.lp_exit = INT_EXIT_LPM0;
	int_param.active_low = 1;

	int result = mpu_init();

	if(result == 0)     //mpu init
	{
		if(!mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL))    // set gyro or/and accel
			return_value = 1;

		if(!mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL)) // set fifo
			return_value = 2;

		if(!mpu_set_sample_rate(DEFAULT_MPU_HZ))              // set sample
			return_value = 3;

	    mpu_get_sample_rate(&gyro_rate);
	    mpu_get_gyro_fsr(&gyro_fsr);
	    mpu_get_accel_fsr(&accel_fsr);

		if(!dmp_load_motion_driver_firmware())       // load dmp 
			return_value = 4;

		if(!dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation)))
			return_value = 5;

		if(!dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
				DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
				DMP_FEATURE_GYRO_CAL))
				return_value = 6;

		if(!dmp_set_fifo_rate(DEFAULT_MPU_HZ))    	// set sample
			return_value = 7;

			mpu_run_self_test_local();                // self test

		if(!mpu_set_dmp_state(1))                 	// enable
			return_value = 8;
	}
	return return_value;
}

#define q30 1073741824.0f

uint8_t Read_DMP(float* Pitch,float* Roll,float* Yaw)
{
	short gyro[3], accel[3], sensors;
	float q0=1.0f,q1=0.0f,q2=0.0f,q3=0.0f;
	unsigned long sensor_timestamp;
	unsigned char more;
	long quat[4];

	int ret = dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &more);

	if(ret)
	{
		return 1;
	}

	if (sensors & INV_WXYZ_QUAT)
	{
		q0=quat[0] / q30;
		q1=quat[1] / q30;
		q2=quat[2] / q30;
		q3=quat[3] / q30;
		*Pitch = (float)asinf(-2 * q1 * q3 + 2 * q0* q2)* 57.3f;
		*Roll =  (float)atan2f(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3f; // roll
		*Yaw =   (float)atan2f(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3) * 57.3f;
		return 0;
	}
	else
	{
		return 2;
	}

}
