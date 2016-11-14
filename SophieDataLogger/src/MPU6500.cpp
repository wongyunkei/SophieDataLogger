/*
 * MPU6500.cpp
 *
 *  Created on: 2014¦~8¤ë23¤é
 *      Author: YunKei
 */

#include <MPU6500.h>
#include <Spi.h>
#include <Task.h>
#include <stdio.h>
#include <math.h>
#include <MathTools.h>
#include <Acceleration.h>
#include <Delay.h>
#include <App.h>
#include <UART.h>

MPU6500Configuration::MPU6500Configuration(int spiIndex, Communication::Spi* spi, SENSITIVITY sensitivity,
											Vector3f AccPos, Vector3f AccNeg,
											Vector3f OmegaScale, Vector3f OmegaOffset) : SpiIndex(spiIndex), _spi(spi),
													Sensitivity(sensitivity),
													_AccPos(AccPos),
													_AccNeg(AccNeg),
													_OmegaScale(OmegaScale),
													_OmegaOffset(OmegaOffset){}



MPU6500::MPU6500(MPU6500Configuration* conf) : Conf(conf), Sensitivity(conf->Sensitivity), DevIndex(conf->SpiIndex), spix(conf->_spi), isValided(false){
	for(int i = 0; i < 3; i++){
		RawAccScale[i] = 2.0f * Inertia::Acceleration::Gravity / (conf->_AccPos[i] - conf->_AccNeg[i]);
		RawAccOffset[i] = conf->_AccPos[i] * RawAccScale[i] - Inertia::Acceleration::Gravity;
		RawOmegaScale[i] = conf->_OmegaScale[i];
		RawOmegaOffset[i] = RawOmegaScale[i] * conf->_OmegaOffset[i];;
	}
	FastInitialization();
	Update();
}

void MPU6500::setSensitivity(MPU6500Configuration::SENSITIVITY value){
	Sensitivity = value;
	switch(Sensitivity){
		case MPU6500Configuration::SENSITIVITY::SENSITIVITY_250:
			App::mApp->mTicks->setTimeout(3);
			while(!spix->WriteCmd(DevIndex, GYRO_CONFIG, 0x00)){
				if(App::mApp->mTicks->Timeout()){
					return;
				}
			}
			break;

		case MPU6500Configuration::SENSITIVITY::SENSITIVITY_500:
			App::mApp->mTicks->setTimeout(3);
			while(!spix->WriteCmd(DevIndex, GYRO_CONFIG, 0x08)){
				if(App::mApp->mTicks->Timeout()){
					return;
				}
			}
			break;

		case MPU6500Configuration::SENSITIVITY::SENSITIVITY_1000:
			App::mApp->mTicks->setTimeout(3);
			while(!spix->WriteCmd(DevIndex, GYRO_CONFIG, 0x10)){
				if(App::mApp->mTicks->Timeout()){
					return;
				}
			}
			break;

		case MPU6500Configuration::SENSITIVITY::SENSITIVITY_2000:
			App::mApp->mTicks->setTimeout(3);
			while(!spix->WriteCmd(DevIndex, GYRO_CONFIG, 0x18)){
				if(App::mApp->mTicks->Timeout()){
					return;
				}
			}
			break;
	}
}

void MPU6500::FastInitialization(){

	App::mApp->mTicks->setTimeout(3);
	while(!spix->WriteCmd(DevIndex, PWR_MGMT_1, 0x00)){
		if(App::mApp->mTicks->Timeout()){
			return;
		}
	}

	App::mApp->mTicks->setTimeout(3);
	while(!spix->WriteCmd(DevIndex, SMPLRT_DIV, 0x07)){
		if(App::mApp->mTicks->Timeout()){
			return;
		}
	}

	App::mApp->mTicks->setTimeout(3);
	while(!spix->WriteCmd(DevIndex, CONFIG, 0x02)){
		if(App::mApp->mTicks->Timeout()){
			return;
		}
	}

	App::mApp->mTicks->setTimeout(3);
	while(!spix->WriteCmd(DevIndex, GYRO_CONFIG, 0x18)){
		if(App::mApp->mTicks->Timeout()){
			return;
		}
	}

	App::mApp->mTicks->setTimeout(3);
	while(!spix->WriteCmd(DevIndex, ACCEL_CONFIG, 0x18)){
		if(App::mApp->mTicks->Timeout()){
			return;
		}
	}

	App::mApp->mTicks->setTimeout(3);
	while(!spix->WriteCmd(DevIndex, ACCEL_CONFIG_2, 0x02)){
		if(App::mApp->mTicks->Timeout()){
			return;
		}
	}
}

bool MPU6500::Update(){

	uint8_t data[14];
	int16_t temp;

	float scale;

	switch(Sensitivity){
		case MPU6500Configuration::SENSITIVITY::SENSITIVITY_250:
			scale = 0.0076335877862595f;
			break;
		case MPU6500Configuration::SENSITIVITY::SENSITIVITY_500:
			scale = 0.0152671755725191f;
			break;
		case MPU6500Configuration::SENSITIVITY::SENSITIVITY_1000:
			scale = 0.0304878048780488f;
			break;
		case MPU6500Configuration::SENSITIVITY::SENSITIVITY_2000:
			scale = 0.0609756097560976f;
			break;
	}

	if(!spix->ReadNBytes(DevIndex, ACCEL_XOUT_H | 0x80, 14, data)){
		FastInitialization();
		isValided = false;
		return false;
	}

	for(int i = 0; i < 14; i += 2){

		if(i >= 0 && i <= 5){
			int j = i / 2;
			temp = (data[i + 1] | (data[i] << 8));
			RawAcc[j] = (float)temp * 0.00048828125f * Acceleration::Gravity;
		}
		else if(i >= 6 && i <= 7){
			temp = (data[i + 1] | (data[i] << 8));
			temperature = (float)temp / 333.87f + 21.0f;
		}
		else if(i >= 8 && i <= 13){
			temp = data[i + 1] | (data[i] << 8);
			RawOmega[(i - 8) / 2] = (float)temp * scale;
		}
	}

	for(int i = 0; i < 3; i++){
		if((RawAcc[i] != RawAcc[i]) || (RawOmega[i] != RawOmega[i])){
			isValided = false;
			return false;
		}
	}

	for(int i = 0; i < 3; i++){
		RawAcc[i] *= RawAccScale[i];
		RawAcc[i] -= RawAccOffset[i];
//		RawOmega[i] -= getGyroTemperatureCompensation(i, temperature);
		RawOmega[i] *= RawOmegaScale[i];
		RawOmega[i] -= RawOmegaOffset[i];
		if(RawOmegaOffset[i] != 0.0f){
			RawOmega[i] = MathTools::CutOff(RawOmega[i], 0.0f, 1.0f);
		}
	}

	isValided = true;
	return true;
}

float MPU6500::getGyroTemperatureCompensation(int index, int channel, float temp){
	float value = 0;
	switch(index){
		case 0:
			switch(channel){
				case 0:
					value = -2.0f;//-1.5f;//-4.5f;//0.0005f*temp*temp - 0.0799f*temp + 12.212f;
					break;
				case 1:
					value = 0.8f;//9.0f;//-0.0002f*temp*temp - 0.0039f*temp + 4.6503f;
					break;
				case 2:
					value = -0.55f;//0.00007*temp*temp + 0.0047f*temp - 0.8594f;
					break;
			}
			break;
		case 1:
			switch(channel){
				case 0:
					value = -10.0f;//-0.7f;//-0.6f;//0.001f*temp*temp - 0.0992f*temp + 6.6046f;
					break;
				case 1:
					value = 8.0f;//3.9f;//0.00002f*temp*temp + 0.032f*temp - 0.3928f;
					break;
				case 2:
					value = -0.33f;//0.00006f*temp*temp + 0.0005f*temp - 0.0949f;
					break;
			}
			break;
		case 2:
			switch(channel){
				case 0:
					value = -164.0f;//0.0011f*temp*temp - 0.1f*temp + 7.0212f;
					break;
				case 1:
					value = 5.5f;//-0.0001f*temp*temp + 0.0083f*temp + 0.9077f;
					break;
				case 2:
					value = -0.43f;//-0.00004f*temp*temp + 0.0015f*temp - 0.5488f;
					break;
			}
			break;
		case 3:
			switch(channel){
				case 0:
					value = 0.0003f*temp*temp + 0.0101f*temp + 2.1163f;
					break;
				case 1:
					value = 0.0002f*temp*temp - 0.0083f*temp - 1.0702f;
					break;
				case 2:
					value = -0.000003f*temp*temp - 0.0131f*temp + 0.9225f;
					break;
			}
			break;
		case 4:
			switch(channel){
				case 0:
					value = -100.3f;////0.1f;//0.0003f*temp*temp - 0.0176f*temp + 3.6351f;
					break;
				case 1:
					value = 5.0f;//4.3f;//-0.00005f*temp*temp + 0.0242f*temp - 0.9812f;
					break;
				case 2:
					value = 0.16f;//-0.00005f*temp*temp + 0.0199f*temp - 1.1762f;
					break;
			}
			break;
		case 5:
			switch(channel){
				case 0:
					value = 22.0f;////4.8f;//0.3f;//0.0009f*temp*temp - 0.1001f*temp + 4.3651f;
					break;
				case 1:
					value = -7.0f;//-0.0001f*temp*temp + 0.0385f*temp - 2.0141f;
					break;
				case 2:
					value = -0.00004f*temp*temp + 0.0036f*temp + 1.1112f;
					break;
			}
			break;
	}
	return value;
}

bool MPU6500::getIsValided(){
	return isValided;
}

void MPU6500::setTemperature(float value){
	temperature = value;
}

float MPU6500::getTemperature(){
	return temperature;
}

void MPU6500::setRawOmegaOffset(Vector3f value){
	RawOmegaOffset = value;
}

Vector3f MPU6500::getRawOmegaOffset(){
	return RawOmegaOffset;
}

void MPU6500::setRawOmega(Vector3f value){
	RawOmega = value;
}

Vector3f MPU6500::getRawOmega(){
	return RawOmega;
}

void MPU6500::setRawAcc(Vector3f value){
	RawAcc = value;
}

Vector3f MPU6500::getRawAcc(){
	return RawAcc;
}
