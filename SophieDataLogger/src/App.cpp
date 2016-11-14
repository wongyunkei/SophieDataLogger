/*
 * App.cpp
 *
 *  Created on: 2015¦~11¤ë27¤é
 *      Author: wongy
 */

#include <App.h>
#include <stdio.h>
#include <stdlib.h>
using namespace std;
using namespace System;
using namespace Sensors;
using namespace Utility;

App* App::mApp = 0;
Ticks* App::mTicks = 0;
Task* App::mTask = 0;
UART* App::mUART1 = 0;
UART* App::mUART2 = 0;
UART* App::mUART3 = 0;
UART* App::mUART4 = 0;
UART* App::mUART5 = 0;
Spi* App::mSpi1 = 0;
Spi* App::mSpi2 = 0;
CAN* App::mCAN1 = 0;
Communicating* App::mCommunicating1 = 0;
Communicating* App::mCommunicating2 = 0;
Communicating* App::mCommunicating3 = 0;
Com* App::Com1 = 0;
Com* App::Com2 = 0;
Com* App::Com3 = 0;
nRF24L01* App::mnRF24L01 = 0;
InputCapture* App::mInputCapture = 0;
MPU6500* App::mMPU6500[4] = {0,0,0,0};
Acceleration* App::_mAcceleration[4] = {0,0,0,0};
Omega* App::_mOmega[4] = {0,0,0,0};
Quaternion* App::_mQuaternion[4] = {0,0,0,0};
int App::DeviceIndex = 0;

void App::ReceiveTask(Bundle* bundle){
	mCommunicating1->ReceivePoll();
}

void App::SendTask(Bundle* bundle){
	mCommunicating1->SendPoll();
}

void printRF(Bundle* bundle){
	static int index = 0;
	switch(App::mApp->mCommunicating2->PrintType){
		case 0:
			if(index < 3){
				if(index == 0){
					App::mApp->mCommunicating2->Send(index+3*(App::mApp->DeviceIndex-1), (float)(MathTools::RadianToDegree(App::mApp->mQuaternion->getEuler()[index]) - App::mApp->mControlling->RollOffset));
				}
				else if(index == 1){
					App::mApp->mCommunicating2->Send(index+3*(App::mApp->DeviceIndex-1), (float)(MathTools::RadianToDegree(App::mApp->mQuaternion->getEuler()[index]) - App::mApp->mControlling->PitchOffset));
				}
				else if(index == 2){
					App::mApp->mCommunicating2->Send(index+3*(App::mApp->DeviceIndex-1), (float)(MathTools::RadianToDegree(App::mApp->mQuaternion->getEuler()[index]) - App::mApp->mControlling->YawOffset));
				}
			}
			break;
		case 1:
			if(index < 3){
				App::mApp->mCommunicating2->Send(index+3*(App::mApp->DeviceIndex-1), (float)(App::mApp->mMPU6050->getRawOmega()[index]));
			}
			break;
		case 2:
			if(index == 0){
				App::mApp->mCommunicating2->Send(0+3*(App::mApp->DeviceIndex-1), App::mApp->mControlling->Motor1PWM);
			}
			else if(index == 1){
				App::mApp->mCommunicating2->Send(1+3*(App::mApp->DeviceIndex-1), App::mApp->mControlling->Motor2PWM);
			}
			else if(index == 2){
				App::mApp->mCommunicating2->Send(2+3*(App::mApp->DeviceIndex-1), App::mApp->mControlling->Motor3PWM);
			}
			else if(index == 3){
				App::mApp->mCommunicating2->Send(3+3*(App::mApp->DeviceIndex-1), App::mApp->mControlling->Motor4PWM);
			}
			break;
	}
//	if(index == 4){
//		index = 0;
//	}
//	else{
//		index++;
//	}
}

void StatePrint3(Bundle* bundle){
	static int index = 0;
	switch(App::mApp->mCommunicating1->PrintType){
		case 0:
			if(index < 3){
				if(index == 0){
					App::mApp->mCommunicating1->Send(index, (float)(MathTools::RadianToDegree(App::mApp->mQuaternion->getEuler()[index]) - App::mApp->mControlling->RollOffset));
				}
				else if(index == 1){
					App::mApp->mCommunicating1->Send(index, (float)(MathTools::RadianToDegree(App::mApp->mQuaternion->getEuler()[index]) - App::mApp->mControlling->PitchOffset));
				}
				else if(index == 2){
					App::mApp->mCommunicating1->Send(index, (float)(MathTools::RadianToDegree(App::mApp->mQuaternion->getEuler()[index]) - App::mApp->mControlling->YawOffset));
				}
			}
			break;
		case 1:
			if(index < 3){
				App::mApp->mCommunicating1->Send(index, (float)(App::mApp->mMPU6050->getRawOmega()[index]));
			}
			break;
		case 2:
			if(index == 0){
				App::mApp->mCommunicating1->Send(0, App::mApp->mControlling->Motor1PWM);
			}
			else if(index == 1){
				App::mApp->mCommunicating1->Send(1, App::mApp->mControlling->Motor2PWM);
			}
			else if(index == 2){
				App::mApp->mCommunicating1->Send(2, App::mApp->mControlling->Motor3PWM);
			}
			else if(index == 3){
				App::mApp->mCommunicating1->Send(3, App::mApp->mControlling->Motor4PWM);
			}
			break;
	}
	if(index == 4){
		index = 0;
	}
	else{
		index++;
	}
}

void BatteryPrint(){
	App::mApp->mADCFilter->Update(App::mApp->mADC->getReading() * 3.3 / 4096.0);
	App::mApp->mCommunicating1->Send(0, App::mApp->mADCFilter->getAverage());
//	printf("%g\n", App::mApp->mADC->getReading());
}

void LocalizationUpdate(){
	App::mApp->mLocalization->LocalizationCalc();
}

void CompassCalTask(){
	App::mApp->mHMC5883L->CalibrationPrint();
}

void CompassCalPrintResult(){
	App::mApp->mHMC5883L->CalibrationResultPrint();
}

void printfBufferTask(){
	AdditionalTools::printfBuffer(0, 4);
}

void UpdateTask(Bundle* bundle){
	static float PrevT = -99999;
	for(int i = 0; i < 4; i++){
		App::mApp->mMPU6500[i]->Update();
		App::mApp->_mAcceleration[i]->Update();
		App::mApp->_mOmega[i]->Update();
		App::mApp->_mQuaternion[i]->Update();
	}
	App::mApp->Velocity = 0.667 / App::mApp->mInputCapture->Period1;
	float dt = (App::mApp->mTicks->getTicks() - PrevT) / 1000;
	if(dt > 0){
		App::mApp->Distance = App::mApp->Velocity*dt;
	}
	PrevT = App::mApp->mTicks->getTicks();
}

void ADCUpdateTask(Bundle* bundle){
	static float PrevT = -99999;
	App::mApp->BatteryPower = App::mApp->BatteryVoltage*App::mApp->BatteryCurrent;
	App::mApp->MotorPower = App::mApp->MotorVoltage*App::mApp->MotorCurrent;
	App::mApp->SolarPower = App::mApp->SolarVoltage*App::mApp->SolarCurrent;
	float dt = (App::mApp->mTicks->getTicks() - PrevT) / 1000;
	if(dt > 0){
		App::mApp->BatteryEnergyConsumption += App::mApp->BatteryPower*dt;
		App::mApp->MotorEnergyConsumption += App::mApp->MotorPower*dt;
		App::mApp->SolarEnergy += App::mApp->SolarPower*dt;
	}
	PrevT = App::mApp->mTicks->getTicks();
}

void App::Print(Bundle* bundle){
	mTicks->PrintTime();
}

void App::TaskDurationPrint(Bundle* bundle){
	mTask->printDeration();
}

void CANSendTask(Bundle* bundle){
	App::mApp->mCAN1->SendPoll();
}

void CANReceiveTask(Bundle* bundle){
	App::mApp->mCAN1->ReceivePoll();
}

void CANReadTask(Bundle* bundle){
	App::mApp->mCAN1->RxLength = 8;
	int length = App::mApp->mCAN1->AvailablePackage;
	if(length > 0){
		for(int i = 0; i < length; i++){
			uint8_t ch[32];
			uint32_t id = App::mApp->mCAN1->ReadPackage((uint8_t*)ch);

			if(id == 0x600){
				App::mApp->mHeartBeat.DeviceID = (ch[3] << 24) | (ch[2] << 16) | (ch[1] << 8) | (ch[0] << 0);
				App::mApp->mHeartBeat.DeviceSerial = (ch[7] << 24) | (ch[6] << 16) | (ch[5] << 8) | (ch[4] << 0);
			}
			else if(id >= 0x601 && id <= 0x6EF){
				id -= 0x600;
				id--;
				int InfoIndex = id % 3;
				int CMUIndex = id / 3;
				if(InfoIndex == 0){
					App::mApp->mTempInfo[CMUIndex].CMUSerialNum = (ch[3] << 24) | (ch[2] << 16) | (ch[1] << 8) | (ch[0] << 0);
					App::mApp->mTempInfo[CMUIndex].PCBTemp = (ch[5] << 8) | (ch[4] << 0);
					App::mApp->mTempInfo[CMUIndex].CellTemp = (ch[7] << 8) | (ch[6] << 0);
				}
				else if(InfoIndex == 1){
					App::mApp->mCMUCellVoltage[CMUIndex].CellVoltages[0] = (ch[1] << 8) | (ch[0] << 0);
					App::mApp->mCMUCellVoltage[CMUIndex].CellVoltages[1] = (ch[3] << 8) | (ch[2] << 0);
					App::mApp->mCMUCellVoltage[CMUIndex].CellVoltages[2] = (ch[5] << 8) | (ch[4] << 0);
					App::mApp->mCMUCellVoltage[CMUIndex].CellVoltages[3] = (ch[7] << 8) | (ch[6] << 0);
				}
				else if(InfoIndex == 2){
					App::mApp->mCMUCellVoltage[CMUIndex].CellVoltages[4] = (ch[1] << 8) | (ch[0] << 0);
					App::mApp->mCMUCellVoltage[CMUIndex].CellVoltages[5] = (ch[3] << 8) | (ch[2] << 0);
					App::mApp->mCMUCellVoltage[CMUIndex].CellVoltages[6] = (ch[5] << 8) | (ch[4] << 0);
					App::mApp->mCMUCellVoltage[CMUIndex].CellVoltages[7] = (ch[7] << 8) | (ch[6] << 0);
				}
			}
			else if(id == 0x6F4){
				union{
					uint32_t d;
					float f;
				} x;
				x.d = (ch[3] << 24) | (ch[2] << 16) | (ch[1] << 8) | (ch[0] << 0);
				App::mApp->mPackStateOfCharge.SOC = x.f;
				x.d = (ch[7] << 24) | (ch[6] << 16) | (ch[5] << 8) | (ch[4] << 0);
				App::mApp->mPackStateOfCharge.SOCPercentage = x.f;
			}
			else if(id == 0x6F5){
				union{
					uint32_t d;
					float f;
				} x;
				x.d = (ch[3] << 24) | (ch[2] << 16) | (ch[1] << 8) | (ch[0] << 0);
				App::mApp->mPackBalanceStateOfCharge.BalanceSOC = x.f;
				x.d = (ch[7] << 24) | (ch[6] << 16) | (ch[5] << 8) | (ch[4] << 0);
				App::mApp->mPackBalanceStateOfCharge.BalanceSOCPercentage = x.f;
			}
			else if(id == 0x6F6){
				App::mApp->mChargerControlInformation.ChargingCellVoltError = (ch[1] << 8) | (ch[0] << 0);
				App::mApp->mChargerControlInformation.CellTempMargin = (ch[3] << 8) | (ch[2] << 0);
				App::mApp->mChargerControlInformation.DischargingCellVoltError = (ch[5] << 8) | (ch[4] << 0);
				App::mApp->mChargerControlInformation.TotalPackCapacity = (ch[7] << 8) | (ch[6] << 0);
			}
			else if(id == 0x6F7){
				App::mApp->mPrechargeStatus.PrechargeContactorDriverStatus = ch[0];
				App::mApp->mPrechargeStatus.PrechargeState = ch[1];
				App::mApp->mPrechargeStatus.TwelveVoltContactorSupplyVoltage = (ch[3] << 8) | (ch[2] << 0);
				App::mApp->mPrechargeStatus.PrechargeTimer = ch[6];
				App::mApp->mPrechargeStatus.PrechargeTimerCounter = ch[7];
			}
			else if(id == 0x6F8){
				App::mApp->mMinMaxCellVoltage.MinCellVoltage = (ch[1] << 8) | (ch[0] << 0);
				App::mApp->mMinMaxCellVoltage.MaxCellVoltage = (ch[3] << 8) | (ch[2] << 0);
				App::mApp->mMinMaxCellVoltage.CMUNumMinCellVolt = ch[4];
				App::mApp->mMinMaxCellVoltage.CellNumCMUMinCellVolt = ch[5];
				App::mApp->mMinMaxCellVoltage.CMUNumMaxCellVolt = ch[6];
				App::mApp->mMinMaxCellVoltage.CellNumCMUMaxCellVolt = ch[7];
			}
			else if(id == 0x6F9){
				App::mApp->mMinMaxCellTemp.MinCellTemp = (ch[1] << 8) | (ch[0] << 0);
				App::mApp->mMinMaxCellTemp.MaxCellTemp = (ch[3] << 8) | (ch[2] << 0);
				App::mApp->mMinMaxCellTemp.CMUNumMinCellTemp = ch[4];
				App::mApp->mMinMaxCellTemp.CMUNumMaxCellTemp = ch[6];
			}
			else if(id == 0x6FA){
				App::mApp->mBatteryPackVoltageCurrent.BatteryVoltage = (ch[3] << 24) | (ch[2] << 16) | (ch[1] << 8) | (ch[0] << 0);
				App::mApp->mBatteryPackVoltageCurrent.BatteryCurrent = (ch[7] << 24) | (ch[6] << 16) | (ch[5] << 8) | (ch[4] << 0);
			}
			else if(id == 0x6FB){
				App::mApp->mBatteryPackStatus.BalanceVoltageThresholdRising = (ch[1] << 8) | (ch[0] << 0);
				App::mApp->mBatteryPackStatus.BalanceVoltageThresholdFalling = (ch[3] << 8) | (ch[2] << 0);
				App::mApp->mBatteryPackStatus.StatusFlags = ch[4];
				App::mApp->mBatteryPackStatus.BMSCMUCount = ch[5];
				App::mApp->mBatteryPackStatus.BMSBMUFirmwareBuildNum = (ch[7] << 8) | (ch[6] << 0);
			}
			else if(id == 0x6FC){
				App::mApp->mBatteryPackFanStatus.FanSpeed0 = (ch[1] << 8) | (ch[0] << 0);
				App::mApp->mBatteryPackFanStatus.FanSpeed1 = (ch[3] << 8) | (ch[2] << 0);
				App::mApp->mBatteryPackFanStatus.TwelveCurrentConsumptionOfFansPlusContactors = (ch[5] << 8) | (ch[4] << 0);
				App::mApp->mBatteryPackFanStatus.TwelveCurrentConsumptionOfCMUs = (ch[7] << 8) | (ch[6] << 0);
			}
			else if(id == 0x6FD){
				App::mApp->mExtendedBatteryPackStatus.StatusFlags = (ch[3] << 24) | (ch[2] << 16) | (ch[1] << 8) | (ch[0] << 0);
				App::mApp->mExtendedBatteryPackStatus.BMUHWVersion = ch[4];
				App::mApp->mExtendedBatteryPackStatus.BMUModelID = ch[5];
			}
			else if(id == 0x505){
				App::mApp->mEVDriverControlsSwitchPosition.State = (ch[1] << 8) | (ch[0] << 0);
			}
		}
	}
}

void StatePrint2(Bundle* bundle){
	static int index = 0;
	float data;

	switch(index){
//		case 0:
//			for(int i = 0; i < 4; i++){
//				App::mApp->mCommunicating1->Send(i, MathTools::RadianToDegree(App::mApp->mQuaternion[i]->getEuler()[0]));
//			}


//			App::mApp->mCommunicating1->Send(0, App::mApp->mHeartBeat.DeviceID);
//			App::mApp->mCommunicating1->Send(1, App::mApp->mHeartBeat.DeviceSerial);
//			break;
//		case 1:
//			static int CMUIndex = 0;
//			data = (float)App::mApp->mTempInfo[CMUIndex].CellTemp / 10.0f;
//			App::mApp->mCommunicating1->Send(75 + CMUIndex, data);
//			data = (float)App::mApp->mTempInfo[CMUIndex].PCBTemp / 10.0f;
//			App::mApp->mCommunicating1->Send(75 + CMUIndex, data);
//
//			if(CMUIndex++ >= 3){
//				CMUIndex = 0;
//			}
//			break;
		case 2:
			static int _CMUIndex = 0;
			for(int i = 0; i < 8; i++){
				data = (float)App::mApp->mCMUCellVoltage[_CMUIndex].CellVoltages[i] / 1000.0f;
				App::mApp->mCommunicating1->Send(5 + _CMUIndex*8 + i, data);
			}
			if(_CMUIndex++ >= 3){
				_CMUIndex = 0;
			}
			break;
//		case 3:
//			App::mApp->mCommunicating1->Send(37, App::mApp->mPackStateOfCharge.SOC);
//			App::mApp->mCommunicating1->Send(38, App::mApp->mPackStateOfCharge.SOCPercentage);
//			break;
//		case 4:
//			App::mApp->mCommunicating1->Send(39, App::mApp->mPackBalanceStateOfCharge.BalanceSOC);
//			App::mApp->mCommunicating1->Send(40, App::mApp->mPackBalanceStateOfCharge.BalanceSOCPercentage);
//			break;
		case 5:
			App::mApp->mCommunicating1->Send(41, App::mApp->mChargerControlInformation.TotalPackCapacity);
			App::mApp->mCommunicating1->Send(42, App::mApp->mChargerControlInformation.CellTempMargin);
			App::mApp->mCommunicating1->Send(43, App::mApp->mChargerControlInformation.ChargingCellVoltError);
			App::mApp->mCommunicating1->Send(44, App::mApp->mChargerControlInformation.DischargingCellVoltError);
			break;
		case 6:
			App::mApp->mCommunicating1->Send(45, App::mApp->mPrechargeStatus.PrechargeContactorDriverStatus);
			App::mApp->mCommunicating1->Send(46, App::mApp->mPrechargeStatus.PrechargeState);
			App::mApp->mCommunicating1->Send(47, App::mApp->mPrechargeStatus.PrechargeTimer);
			App::mApp->mCommunicating1->Send(48, App::mApp->mPrechargeStatus.PrechargeTimerCounter);
			App::mApp->mCommunicating1->Send(49, App::mApp->mPrechargeStatus.TwelveVoltContactorSupplyVoltage);
			break;
		case 7:
			App::mApp->mCommunicating1->Send(50, App::mApp->mMinMaxCellVoltage.CMUNumMaxCellVolt);
			App::mApp->mCommunicating1->Send(51, App::mApp->mMinMaxCellVoltage.CMUNumMinCellVolt);
			App::mApp->mCommunicating1->Send(52, App::mApp->mMinMaxCellVoltage.CellNumCMUMaxCellVolt);
			App::mApp->mCommunicating1->Send(53, App::mApp->mMinMaxCellVoltage.CellNumCMUMinCellVolt);
			App::mApp->mCommunicating1->Send(54, (float)App::mApp->mMinMaxCellVoltage.MaxCellVoltage / 1000.0f);
			App::mApp->mCommunicating1->Send(55, (float)App::mApp->mMinMaxCellVoltage.MinCellVoltage / 1000.0f);
			break;
		case 8:
			App::mApp->mCommunicating1->Send(56, App::mApp->mMinMaxCellTemp.CMUNumMaxCellTemp);
			App::mApp->mCommunicating1->Send(57, App::mApp->mMinMaxCellTemp.CMUNumMinCellTemp);
			App::mApp->mCommunicating1->Send(58, (float)App::mApp->mMinMaxCellTemp.MaxCellTemp / 10.0f);
			App::mApp->mCommunicating1->Send(59, (float)App::mApp->mMinMaxCellTemp.MinCellTemp / 10.0f);
			break;
		case 9:
			App::mApp->mCommunicating1->Send(60, (float)App::mApp->mBatteryPackVoltageCurrent.BatteryCurrent / 1000.0f);
			App::mApp->mCommunicating1->Send(61, (float)App::mApp->mBatteryPackVoltageCurrent.BatteryVoltage / 1000.0f);
			break;
		case 10:
			App::mApp->mCommunicating1->Send(62, App::mApp->mBatteryPackStatus.BMSBMUFirmwareBuildNum);
			App::mApp->mCommunicating1->Send(63, App::mApp->mBatteryPackStatus.BMSCMUCount);
			App::mApp->mCommunicating1->Send(64, App::mApp->mBatteryPackStatus.BalanceVoltageThresholdFalling);
			App::mApp->mCommunicating1->Send(65, App::mApp->mBatteryPackStatus.BalanceVoltageThresholdRising);
			App::mApp->mCommunicating1->Send(66, App::mApp->mBatteryPackStatus.StatusFlags);
			break;
		case 11:
			App::mApp->mCommunicating1->Send(67, App::mApp->mBatteryPackFanStatus.FanSpeed0);
			App::mApp->mCommunicating1->Send(68, App::mApp->mBatteryPackFanStatus.FanSpeed1);
			App::mApp->mCommunicating1->Send(69, App::mApp->mBatteryPackFanStatus.TwelveCurrentConsumptionOfCMUs);
			App::mApp->mCommunicating1->Send(70, App::mApp->mBatteryPackFanStatus.TwelveCurrentConsumptionOfFansPlusContactors);
			break;
		case 12:
			App::mApp->mCommunicating1->Send(71, App::mApp->mExtendedBatteryPackStatus.BMUHWVersion);
			App::mApp->mCommunicating1->Send(72, App::mApp->mExtendedBatteryPackStatus.BMUModelID);
			App::mApp->mCommunicating1->Send(73, App::mApp->mExtendedBatteryPackStatus.StatusFlags);
			break;
		case 13:
			App::mApp->mCommunicating1->Send(74, App::mApp->mEVDriverControlsSwitchPosition.State);
			break;
	}
	if(index++ == 13){
		index = 0;
	}
}

void StatePrint1(Bundle* bundle){
	float data;
	for(int i = 0; i < 4; i++){
		App::mApp->mCommunicating1->Send(i+80, MathTools::RadianToDegree(App::mApp->_mQuaternion[i]->getEuler()[0]));
		App::mApp->mCommunicating1->Send(i+86, App::mApp->_mAcceleration[i]->getFilteredAcc()[1]);
	}

	App::mApp->mCommunicating1->Send(100, App::mApp->Velocity);
	App::mApp->mCommunicating1->Send(101, App::mApp->Distance);

	App::mApp->mCommunicating1->Send(109, App::mApp->MotorCurrent);
	App::mApp->mCommunicating1->Send(111, App::mApp->SolarCurrent);

	App::mApp->mCommunicating1->Send(108, App::mApp->MotorVoltage);
	App::mApp->mCommunicating1->Send(110, App::mApp->SolarVoltage);

	App::mApp->mCommunicating1->Send(102, App::mApp->BatteryPower);
	App::mApp->mCommunicating1->Send(103, App::mApp->MotorPower);
	App::mApp->mCommunicating1->Send(104, App::mApp->SolarPower);

	App::mApp->mCommunicating1->Send(105, App::mApp->BatteryEnergyConsumption);
	App::mApp->mCommunicating1->Send(106, App::mApp->MotorEnergyConsumption);
	App::mApp->mCommunicating1->Send(107, App::mApp->SolarEnergy);


	data = (float)App::mApp->mTempInfo[0].CellTemp / 10.0f;
	App::mApp->mCommunicating1->Send(75 + 0, data);
	data = (float)App::mApp->mTempInfo[1].CellTemp / 10.0f;
	App::mApp->mCommunicating1->Send(75 + 1, data);
	data = (float)App::mApp->mTempInfo[2].CellTemp / 10.0f;
	App::mApp->mCommunicating1->Send(75 + 2, data);
	data = (float)App::mApp->mTempInfo[3].CellTemp / 10.0f;
	App::mApp->mCommunicating1->Send(75 + 3, data);

	App::mApp->mCommunicating1->Send(37, App::mApp->mPackStateOfCharge.SOC);
	App::mApp->mCommunicating1->Send(38, App::mApp->mPackStateOfCharge.SOCPercentage);

	App::mApp->mCommunicating1->Send(39, App::mApp->mPackBalanceStateOfCharge.BalanceSOC);
	App::mApp->mCommunicating1->Send(40, App::mApp->mPackBalanceStateOfCharge.BalanceSOCPercentage);

	App::mApp->mCommunicating1->Send(54, (float)App::mApp->mMinMaxCellVoltage.MaxCellVoltage / 1000.0f);
	App::mApp->mCommunicating1->Send(55, (float)App::mApp->mMinMaxCellVoltage.MinCellVoltage / 1000.0f);

	App::mApp->mCommunicating1->Send(60, (float)App::mApp->mBatteryPackVoltageCurrent.BatteryCurrent / 1000.0f);
	App::mApp->mCommunicating1->Send(61, (float)App::mApp->mBatteryPackVoltageCurrent.BatteryVoltage / 1000.0f);
}

void PrintTask(Bundle* bundle){
	App::mApp->mTask->printDeration();
}

void SelectPrintRF(Bundle* bundle){
	static int index = 0;
	App::mApp->mnRF24L01->TxChannel = App::mApp->Channel[index+1];
	if(index == 0){
		App::mApp->mCommunicating2->Send(Communicating::DEV1FB, 0);
	}
	else if(index == 1){
		App::mApp->mCommunicating2->Send(Communicating::DEV2FB, 0);
	}
	else if(index == 2){
		App::mApp->mCommunicating2->Send(Communicating::DEV3FB, 0);
	}
	else if(index == 3){
		App::mApp->mCommunicating2->Send(Communicating::DEV4FB, 0);
	}
	if(index == 3){
		index = 0;
	}
	else{
		index++;
	}
}

void App::AppInit(){

	Delay::DelayMS(100);
	mApp = this;
	for(int i = 0; i < 16; i++){
		mExti[i] = 0;
	}
	mQuaternion = 0;
	mConfig = new Config();

	mTicks = new Ticks(false);
	mTask = new Task();
	mUART1 = new UART(mConfig->UART1Conf1);
	mLed1 = new GPIO(mConfig->GPIO1Conf1);
	mLed2 = new GPIO(mConfig->GPIO2Conf1);
	mLed1->Blink(mLed1,true,2000);
	printf("Starting...\r\n");
}

App::App() : error(0), debugCount(0), arrived(false),
		trigger(false), mCompass(0), mEncoderYaw(0),
		BatteryVoltage(0), BatteryCurrent(0), BatteryPower(0), BatteryEnergyConsumption(0),
		MotorVoltage(0), MotorCurrent(0), MotorPower(0), MotorEnergyConsumption(0),
		SolarVoltage(0), SolarCurrent(0), SolarPower(0), SolarEnergy(0),
		Velocity(0), Distance(0){

	AppInit();
	Com1 = new Com(Com::__UART, (uint32_t)mUART1);
	mCommunicating1 = new Communicating(Com1);
	mSpi1 = new Spi(mConfig->Spi1Conf1);

	Vector3f AccPos;
	AccPos << 1,1,1;
	AccPos*=Acceleration::Gravity;
	Vector3f AccNeg;
	AccNeg << 1,1,1;;
	AccNeg*=-Acceleration::Gravity;
	Vector3f OmegaScale;
	OmegaScale << 1,1,1;;
	Vector3f OmegaOffset[4];

	//DEV1
	OmegaOffset[0] << 1.5,0.95,-0.45;
	//DEV2
	OmegaOffset[1] << 2.0,0.05,-1.0;
	//DEV3
	OmegaOffset[2] << -3.15,0.8,-2.15;
	//DEV4
	OmegaOffset[3] << 0.85,1.6,-0.75;

	for(int i = 0; i < 4; i++){
		mMPU6500Config[i] = new MPU6500Configuration(i, mSpi1, MPU6500Configuration::SENSITIVITY_2000, AccPos, AccNeg, OmegaScale, OmegaOffset[i]);
		mMPU6500[i] = new MPU6500(mMPU6500Config[i]);
		_mAcceleration[i] = new Acceleration(mMPU6500[i]);
		_mOmega[i] = new Omega(mMPU6500[i]);
		_mQuaternion[i] = new Quaternion(_mAcceleration[i], _mOmega[i]);
	}

	mInputCapture = new InputCapture(mConfig->InputCaptureConf1);
	mCAN1 = new CAN(mConfig->CAN1Conf1);

	mTask->Attach(20, UpdateTask, "UpdateTask", true);
	mTask->Attach(20, ADCUpdateTask, "ADCUpdateTask", true);
	mTask->Attach(10, ReceiveTask, "ReceiveTask", true);
	mTask->Attach(10, SendTask, "SendTask", true);
	mTask->Attach(10, CANReceiveTask, "CANReceiveTask", true);
	mTask->Attach(10, CANSendTask, "CANSendTask", true);
	mTask->Attach(100, CANReadTask, "CANReadTask", true);

	mTask->Attach(20, StatePrint1, "StatePrint1", true);
	mTask->Attach(20, StatePrint2, "StatePrint2", true);

	printf("Started\r\n");
	mTask->Run(true);
}


