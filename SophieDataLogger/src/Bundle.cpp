/*
 * Bundle.cpp
 *
 *  Created on: 2016�~6��10��
 *      Author: wongy
 */

#include <Bundle.h>

using namespace System;

Bundle::Bundle() : mBlinkObj(0), mTaskObj(0), BlinkObjNum(0), Cmd(0), Data(0) {
	mBlinkObj = new BlinkObj*;
}


