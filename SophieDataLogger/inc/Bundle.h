/*
 * Bundle.h
 *
 *  Created on: 2016�~6��10��
 *      Author: wongy
 */

#ifndef BUNDLE_H_
#define BUNDLE_H_

#include <GPIO.h>
#include <Task.h>

using namespace Time;

namespace System{
	class BlinkObj;
};

namespace Time{
	class Task;
	class TaskObj;
};

namespace System{

	class Bundle{
		public:
			Bundle();
			BlinkObj** mBlinkObj;
			int BlinkObjNum;
			TaskObj* mTaskObj;
			int Cmd;
			float Data;
	};

};

#endif /* BUNDLE_H_ */
