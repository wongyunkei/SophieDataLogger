/*
 * Config.h
 *
 *  Created on: 2015�~11��27��
 *      Author: wongy
 */

#ifndef CONFIG_H_
#define CONFIG_H_

#include <Configuration.h>
#include <GPIO.h>
#include <UART.h>
#include <Communicating.h>
#include <PWM.h>
#include <ADConverter.h>
#include <I2C.h>
#include <Spi.h>
#include <Sonic.h>
#include <Encoder.h>
#include <CAN.h>
#include <InputCapture.h>

using namespace Debug;
using namespace Communication;
using namespace Control;
using namespace Sensors;

namespace Sensors{
	class ADCConfiguration;
	class ADConverter;
};

namespace System{

	class Config{
		public:
			Config();

			CANConfiguration* CAN1Conf1;
			CANConfiguration* CAN1Conf2;

			Encoder::EncoderConfiguration* Encoder1Conf1;
			Encoder::EncoderConfiguration* Encoder2Conf1;
			Encoder::EncoderConfiguration* Encoder3Conf1;
			Encoder::EncoderConfiguration* Encoder4Conf1;
			Encoder::EncoderConfiguration* Encoder5Conf1;
			Encoder::EncoderConfiguration* Encoder6Conf1;
			GPIO::GPIOConfiguration* LedConf1;
			GPIO::GPIOConfiguration* LedConf2;
			GPIO::GPIOConfiguration* LedConf3;
			GPIO::GPIOConfiguration* LedConf4;
			GPIO::GPIOConfiguration* LedConf5;
			GPIO::GPIOConfiguration* LedConf6;
			GPIO::GPIOConfiguration* LedConf7;
			GPIO::GPIOConfiguration* LedConf8;
			GPIO::GPIOConfiguration* LedConf9;
			GPIO::GPIOConfiguration* LedConf10;
			GPIO::GPIOConfiguration* LedConf11;
			GPIO::GPIOConfiguration* LedConf12;
			GPIO::GPIOConfiguration* LedConf13;
			GPIO::GPIOConfiguration* LedConf14;
			GPIO::GPIOConfiguration* LedConf15;
			GPIO::GPIOConfiguration* LedConf16;
			GPIO::GPIOConfiguration* LedConf17;
			GPIO::GPIOConfiguration* LedConf18;
			GPIO::GPIOConfiguration* LedConf19;
			GPIO::GPIOConfiguration* LedConf20;
			GPIO::GPIOConfiguration* GPIO1Conf1;
			GPIO::GPIOConfiguration* GPIO2Conf1;
			GPIO::GPIOConfiguration* GPIO3Conf1;
			GPIO::GPIOConfiguration* GPIO4Conf1;
			GPIO::GPIOConfiguration* GPIO5Conf1;
			GPIO::GPIOConfiguration* GPIO6Conf1;
			GPIO::GPIOConfiguration* GPIO7Conf1;
			GPIO::GPIOConfiguration* GPIO8Conf1;
			UART::UARTConfiguration* UART1Conf1;
			UART::UARTConfiguration* UART1Conf2;
			UART::UARTConfiguration* UART2Conf1;
			UART::UARTConfiguration* UART2Conf2;
			UART::UARTConfiguration* UART3Conf1;
			UART::UARTConfiguration* UART3Conf2;
			UART::UARTConfiguration* UART4Conf1;
			UART::UARTConfiguration* UART4Conf2;
			UART::UARTConfiguration* UART5Conf1;
			UART::UARTConfiguration* UART5Conf2;
			PWM::PWMConfiguration* mPWMConf1;
			InputCaptureConfiguration* InputCaptureConf1;
			ADCConfiguration* ADCConf1;
			ADCConfiguration* ADCConf2;
			ADCConfiguration* ADCConf3;
			ADCConfiguration* ADCConf4;
			ADCConfiguration* ADCConf5;
			ADCConfiguration* ADCConf6;
			ADCConfiguration* ADCConf7;
			ADCConfiguration* ADCConf8;
			ADCConfiguration* ADCConf9;
			ADCConfiguration* ADCConf10;
			ADCConfiguration* ADCConf11;
			ADCConfiguration* ADCConf12;
			ADCConfiguration* ADCConf13;
			ADCConfiguration* ADCConf14;
			ADCConfiguration* ADCConf15;
			ADCConfiguration* ADCConf16;
			I2C::I2CConfiguration* I2C1Conf1;
			I2C::I2CConfiguration* I2C1Conf2;
			I2C::I2CConfiguration* I2C2Conf1;
			I2C::I2CConfiguration* I2C2Conf2;
			Spi::SpiConfiguration* Spi1Conf1;
			Spi::SpiConfiguration* Spi1Conf2;
			Spi::SpiConfiguration* Spi2Conf1;
			Sonic::SonicConfiguration* SonicConf1;
			Sonic::SonicConfiguration* SonicConf2;
			Sonic::SonicConfiguration* SonicConf3;
			Sonic::SonicConfiguration* SonicConf4;
		private:
	};

};

#endif /* CONFIG_H_ */
