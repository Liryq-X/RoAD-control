#ifndef MOTOR_H
#define MOTOR_H
//#include "cbw.h"
#include <iostream>
#include "Serial.h"

struct Motor {
	CSerial serial;
	Motor();   
	~Motor();

	//void brakeInput(unsigned char datavalue);                              

//	ULONG getCount(int CounterNum);

//	void getCountClear(int CounterNum);

	float getWeight(int com);

	void getWeightClear(int com);
};
#endif