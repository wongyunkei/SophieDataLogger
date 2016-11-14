/*
 * App.h
 *
 *  Created on: 2015�~11��27��
 *      Author: wongy
 */

#ifndef APP_H_
#define APP_H_

#include <stm32f4xx.h>
#include <inttypes.h>
#include <MathTools.h>
#include <stdio.h>
#include <Delay.h>
#include <Config.h>
#include <Communicating.h>
#include <Ticks.h>
#include <Task.h>
#include <GPIO.h>
#include <UART.h>
#include <PWM.h>
#include <ADConverter.h>
#include <Sonic.h>
#include <Spi.h>
#include <I2C.h>
#include <Kalman.h>
#include <MPU6050.h>
#include <MPU6500.h>
#include <HMC5883L.h>
#include <Acceleration.h>
#include <Omega.h>
#include <Compass.h>
#include <Quaternion.h>
#include <AdditionalTools.h>
#include <Controlling.h>
#include <MovingWindowAverageFilter.h>
#include <Encoder.h>
#include <Localization.h>
#include <EncoderYaw.h>
#include <ExternalInterrupt.h>
#include <Eigen/Eigen>
#include <CAN.h>
#include <InputCapture.h>
#include <nRF24L01.h>

using namespace Time;
using namespace Math;
using namespace Sensors;
using namespace Inertia;
using namespace Control;
using namespace Communication;
using namespace Debug;

namespace System{
	class Config;
	class GPIO;
};

namespace Control{
	class Controlling;
};

namespace Communication{
	class Communicating;
	class Com;
};

namespace Sensors{
	class MPU6050;
	class MPU6500;
	class HMC5883L;
	class Encoder;
	class ADConverter;
	class MPU6050Configuration;
	class MPU6500Configuration;
};

namespace Inertia{
	class Acceleration;
	class Omega;
	class Compass;
	class EncoderYaw;
};

namespace Math{
	class Quaternion;
	class Localization;
};


typedef struct Point{
	float speed;
	float x;
	float y;
	float yaw;
	bool SonicCalFL;
	bool SonicCalFR;
	bool SonicCalL;
	bool SonicCalR;
	float FL;
	float FR;
	float L;
	float R;
	bool CalX;
	bool CalY;
	float CalXValue;
	float CalYValue;
} PT;

namespace System{
	class App{
		public:
			App();
			static void ReceiveTask(Bundle* bundle);
			static void SendTask(Bundle* bundle);
			static void Print(Bundle* bundle);
			static void TaskDurationPrint(Bundle* bundle);
			static int DeviceIndex;
			uint8_t* Channel;
			static App* mApp;
			static Ticks* mTicks;
			static Task* mTask;
			Config* mConfig;
			ExternalInterrupt* mExti[16];
			GPIO* mLed1;
			GPIO* mLed2;
			GPIO* mLed3;
			GPIO* mLed4;
			MPU6050Configuration* mMPU6050Config;
			MPU6500Configuration* mMPU6500Config[4];
			GPIO* mSonicTrigger[16];

			GPIO* mGPIO1;
			GPIO* mGPIO2;
			GPIO* mGPIO3;
			GPIO* mGPIO4;
			GPIO* mGPIO5;
			GPIO* mGPIO6;
			GPIO* mGPIO7;
			GPIO* mGPIO8;
			static UART* mUART1;
			static UART* mUART2;
			static UART* mUART3;
			static UART* mUART4;
			static UART* mUART5;
			static CAN* mCAN1;
			static Com* Com1;
			static Com* Com2;
			static Com* Com3;
			static Communicating* mCommunicating1;
			static Communicating* mCommunicating2;
			static Communicating* mCommunicating3;
			static Communicating* mCommunicating4;
			nRF24L01Configuration* nRF24L01Conf;
			static nRF24L01* mnRF24L01;
			PWM* mPWM;
			static InputCapture* mInputCapture;
			ADConverter* mADC;
			Sonic* mSonic1;
			Sonic* mSonic2;
			Sonic* mSonic3;
			Sonic* mSonic4;
			Sonic* mSonic5;
			Sonic* mSonic6;
			Sonic* mSonic7;
			Sonic* mSonic8;
			Sonic* mSonic9;
			Sonic* mSonic10;
			Sonic* mSonic11;
			Sonic* mSonic12;
			Sonic* mSonic13;
			Sonic* mSonic14;
			Sonic* mSonic15;
			Sonic* mSonic16;
			static Spi* mSpi1;
			static Spi* mSpi2;
			I2C* mI2C1;
			I2C* mI2C2;
			GPIO* mCE;
			GPIO* mIRQ;
			static MPU6500* mMPU6500[4];
			MPU6050* mMPU6050;
			HMC5883L* mHMC5883L;
			Acceleration* mAcceleration;
			static Acceleration* _mAcceleration[4];
			Omega* mOmega;
			static Omega* _mOmega[4];
			Compass* mCompass;
			Quaternion* mQuaternion;
			static Quaternion* _mQuaternion[4];
			Controlling* mControlling;
			MovingWindowAverageFilter* mADCFilter;
			Encoder* mEncoder1;
			Encoder* mEncoder2;
			Encoder* mEncoder3;
			Encoder* mEncoder4;
			Encoder* mEncoder5;
			Encoder* mEncoder6;
			Localization* mLocalization;
			EncoderYaw* mEncoderYaw;
			PT currentPT;
			PT nextPT;
			bool trigger;
			bool arrived;

			int debugCount;

			uint32_t* funcAddr;
			uint32_t funcNum;
			float error;

			struct HeartBeat{
				uint32_t DeviceID;
				uint32_t DeviceSerial;
			} mHeartBeat;

			struct TempInfo{
				uint32_t CMUSerialNum;
				int16_t PCBTemp;
				int16_t CellTemp;
			} mTempInfo[4];

			struct CMUCellVoltage{
				int16_t CellVoltages[8];
			} mCMUCellVoltage[4];

			struct PackStateOfCharge{
				float SOC;
				float SOCPercentage;
			} mPackStateOfCharge;

			struct PackBalanceStateOfCharge{
				float BalanceSOC;
				float BalanceSOCPercentage;
			} mPackBalanceStateOfCharge;

			struct ChargerControlInformation{
				int16_t ChargingCellVoltError;
				int16_t CellTempMargin;
				int16_t DischargingCellVoltError;;
				uint16_t TotalPackCapacity;
			} mChargerControlInformation;

			struct PrechargeStatus{
				uint8_t PrechargeContactorDriverStatus;
				uint8_t PrechargeState;
				uint16_t TwelveVoltContactorSupplyVoltage;
				uint8_t PrechargeTimer;
				uint8_t PrechargeTimerCounter;

			} mPrechargeStatus;

			struct MinMaxCellVoltage{
				uint16_t MinCellVoltage;
				uint16_t MaxCellVoltage;
				uint8_t CMUNumMinCellVolt;
				uint8_t CellNumCMUMinCellVolt;
				uint8_t CMUNumMaxCellVolt;
				uint8_t CellNumCMUMaxCellVolt;
			} mMinMaxCellVoltage;

			struct MinMaxCellTemp{
				uint16_t MinCellTemp;
				uint16_t MaxCellTemp;
				uint8_t CMUNumMinCellTemp;
				uint8_t CMUNumMaxCellTemp;
			} mMinMaxCellTemp;

			struct BatteryPackVoltageCurrent{
				uint32_t BatteryVoltage;
				int32_t BatteryCurrent;
			} mBatteryPackVoltageCurrent;

			struct BatteryPackStatus{
				uint16_t BalanceVoltageThresholdRising;
				uint16_t BalanceVoltageThresholdFalling;
				uint8_t StatusFlags;
				uint8_t BMSCMUCount;
				uint16_t BMSBMUFirmwareBuildNum;
			} mBatteryPackStatus;

			struct BatteryPackFanStatus{
				uint16_t FanSpeed0;
				uint16_t FanSpeed1;
				uint16_t TwelveCurrentConsumptionOfFansPlusContactors;
				uint16_t TwelveCurrentConsumptionOfCMUs;

			} mBatteryPackFanStatus;

			struct ExtendedBatteryPackStatus{
				uint32_t StatusFlags;
				uint8_t BMUHWVersion;
				uint8_t BMUModelID;
			} mExtendedBatteryPackStatus;

			struct EVDriverControlsSwitchPosition{
				uint16_t State;
			} mEVDriverControlsSwitchPosition;

			float BatteryVoltage;
			float BatteryCurrent;
			float BatteryPower;
			float BatteryEnergyConsumption;

			float MotorVoltage;
			float MotorCurrent;
			float MotorPower;
			float MotorEnergyConsumption;

			float SolarVoltage;
			float SolarCurrent;
			float SolarPower;
			float SolarEnergy;

			float Velocity;
			float Distance;



		private:
			void AppInit();
	};
};

#endif /* APP_H_ */
