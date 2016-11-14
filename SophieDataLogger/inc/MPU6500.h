/*
 * MPU6050.h
 *
 *  Created on: 2014�~8��23��
 *      Author: YunKei
 */

#ifndef  MPU6500_H_
#define  MPU6500_H_

#include <Spi.h>
#include <Acceleration.h>
#include <Eigen/Eigen>
using Eigen::Vector3f;

using namespace Communication;

namespace Sensors{
	class MPU6500Configuration{
		public:
			enum SENSITIVITY{SENSITIVITY_250, SENSITIVITY_500, SENSITIVITY_1000, SENSITIVITY_2000};
			MPU6500Configuration(int spiIndex, Communication::Spi* spi,	SENSITIVITY sensitivity, Vector3f AccPos, Vector3f AccNeg, Vector3f OmegaScale, Vector3f OmegaOffset);
			int SpiIndex;
			Communication::Spi* _spi;
			SENSITIVITY Sensitivity;
			Vector3f _AccPos;
			Vector3f _AccNeg;
			Vector3f _OmegaScale;
			Vector3f _OmegaOffset;
		private:
	};

	class MPU6500{
		public:

			enum REG{SELF_TEST_X_GYRO = 0x00,
					 SELF_TEST_Y_GYRO = 0x01,
					 SELF_TEST_Z_GYRO = 0x02,
					 SELF_TEST_X_ACCEL = 0x0D,
					 SELF_TEST_Y_ACCEL = 0x0E,
					 SELF_TEST_Z_ACCEL = 0x0F,
					 XG_OFFSET_H = 0x13,
					 XG_OFFSET_L = 0x14,
					 YG_OFFSET_H = 0x15,
					 YG_OFFSET_L = 0x16,
					 ZG_OFFSET_H = 0x17,
					 ZG_OFFSET_L = 0x18,
					 SMPLRT_DIV = 0x19,
					 CONFIG = 0x1A,
					 GYRO_CONFIG = 0x1B,
					 ACCEL_CONFIG = 0x1C,
					 ACCEL_CONFIG_2 = 0x1D,
					 LP_ACCEL_ODR = 0x1E,
					 WOM_THR = 0x1F,
					 FIFO_EN = 0x23,
					 I2C_MST_CTRL = 0x24,
					 I2C_SLV0_ADDR = 0x25,
					 I2C_SLV0_REG = 0x26,
					 I2C_SLV0_CTRL = 0x27,
					 I2C_SLV1_ADDR = 0x28,
					 I2C_SLV1_REG = 0x29,
					 I2C_SLV1_CTRL = 0x2A,
					 I2C_SLV2_ADDR = 0x2B,
					 I2C_SLV2_REG = 0x2C,
					 I2C_SLV2_CTRL = 0x2D,
					 I2C_SLV3_ADDR = 0x2E,
					 I2C_SLV3_REG = 0x2F,
					 I2C_SLV3_CTRL = 0x30,
					 I2C_SLV4_ADDR = 0x31,
					 I2C_SLV4_REG = 0x32,
					 I2C_SLV4_DO = 0x33,
					 I2C_SLV4_CTRL = 0x34,
					 I2C_SLV4_DI = 0x35,
					 I2C_MST_STATUS = 0x36,
					 INT_PIN_CFG = 0x37,
					 INT_ENABLE = 0x38,
					 INT_STATUS = 0x3A,
					 ACCEL_XOUT_H = 0x3B,
					 ACCEL_XOUT_L = 0x3C,
					 ACCEL_YOUT_H = 0x3D,
					 ACCEL_YOUT_L = 0x3E,
					 ACCEL_ZOUT_H = 0x3F,
					 ACCEL_ZOUT_L = 0x40,
					 TEMP_OUT_H = 0x41,
					 TEMP_OUT_L = 0x42,
					 GYRO_XOUT_H = 0x43,
					 GYRO_XOUT_L = 0x44,
					 GYRO_YOUT_H = 0x45,
					 GYRO_YOUT_L = 0x46,
					 GYRO_ZOUT_H = 0x47,
					 GYRO_ZOUT_L = 0x48,
					 EXT_SENS_DATA_00 = 0x49,
					 EXT_SENS_DATA_01 = 0x4A,
					 EXT_SENS_DATA_02 = 0x4B,
					 EXT_SENS_DATA_03 = 0x4C,
					 EXT_SENS_DATA_04 = 0x4D,
					 EXT_SENS_DATA_05 = 0x4E,
					 EXT_SENS_DATA_06 = 0x4F,
					 EXT_SENS_DATA_07 = 0x50,
					 EXT_SENS_DATA_08 = 0x51,
					 EXT_SENS_DATA_09 = 0x52,
					 EXT_SENS_DATA_10 = 0x53,
					 EXT_SENS_DATA_11 = 0x54,
					 EXT_SENS_DATA_12 = 0x55,
					 EXT_SENS_DATA_13 = 0x56,
					 EXT_SENS_DATA_14 = 0x57,
					 EXT_SENS_DATA_15 = 0x58,
					 EXT_SENS_DATA_16 = 0x59,
					 EXT_SENS_DATA_17 = 0x5A,
					 EXT_SENS_DATA_18 = 0x5B,
					 EXT_SENS_DATA_19 = 0x5C,
					 EXT_SENS_DATA_20 = 0x5D,
					 EXT_SENS_DATA_21 = 0x5E,
					 EXT_SENS_DATA_22 = 0x5F,
					 EXT_SENS_DATA_23 = 0x60,
					 I2C_SLV0_DO = 0x63,
				  	 I2C_SLV1_DO = 0x64,
				  	 I2C_SLV2_DO = 0x65,
				  	 I2C_SLV3_DO = 0x66,
				  	 I2C_MST_DELAY_CTRL = 0x67,
				  	 SIGNAL_PATH_RESET = 0x68,
					 ACCEL_INTEL_CTRL = 0x69,
					 USER_CTRL = 0x6A,
					 PWR_MGMT_1 = 0x6B,
					 PWR_MGMT_2 = 0x6C,
				  	 RA_BANK_SEL = 0x6D,
					 FIFO_COUNT_H = 0x72,
					 FIFO_COUNT_L = 0x73,
					 FIFO_R_W = 0x74,
					 WHO_AM_I = 0x75,
					 XA_OFFSET_H = 0x77,
					 XA_OFFSET_L = 0x78,
					 YA_OFFSET_H = 0x7A,
					 YA_OFFSET_L = 0x7B,
					 ZA_OFFSET_H = 0x7D,
					 ZA_OFFSET_L = 0x7E};

			MPU6500(MPU6500Configuration* conf);
			bool Update();
			void setSensitivity(MPU6500Configuration::SENSITIVITY value);
			void setTemperature(float value);
			float getTemperature();
			void setRawAcc(Vector3f value);
			Vector3f getRawAcc();
			void setRawAccOffset(Vector3f value);
			Vector3f getRawAccOffset();
			void setRawAccScale(Vector3f value);
			Vector3f getRawAccScale();
			void setRawOmega(Vector3f value);
			Vector3f getRawOmega();
			void setRawOmegaOffset(Vector3f value);
			Vector3f getRawOmegaOffset();
			bool getIsValided();
			MPU6500Configuration* Conf;
		private:
			Spi* spix;
			int DevIndex;
			bool isValided;
			Vector3f RawAcc;
			Vector3f RawAccOffset;
			Vector3f RawAccScale;
			Vector3f RawOmega;
			Vector3f RawOmegaScale;
			Vector3f RawOmegaOffset;
			float temperature;
			MPU6500Configuration::SENSITIVITY Sensitivity;
			void FastInitialization();
			float getGyroTemperatureCompensation(int index, int channel, float temp);
	};
};

#endif /*  MPU6050_H_ */
