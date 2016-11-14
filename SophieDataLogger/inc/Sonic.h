/*
 * Sonic.h
 *
 *  Created on: 2016�~4��13��
 *      Author: wongy
 */

#ifndef SONIC_H_
#define SONIC_H_

#include <Led.h>
#include <ExternalInterrupt.h>
#include <MovingWindowAverageFilter.h>

using namespace Debug;
using namespace Math;

namespace Sensors{
	class Sonic{
		public:
			class SonicConfiguration{
				public:
					SonicConfiguration(GPIO::GPIOConfiguration* trigger, Configuration* echo);
					GPIO::GPIOConfiguration* Trigger;
					Configuration* Echo;
			};
			Sonic(SonicConfiguration* conf);
			void Update();
			SonicConfiguration* Conf;
			GPIO* Trigger;
			ExternalInterrupt* Echo;
			float Distance;
			static int SonicNum;
			static int OverFlowCount;
			MovingWindowAverageFilter* SonicFilter;
	};
};

#endif /* SONIC_H_ */
