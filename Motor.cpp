#include "Motor.h"


Motor::Motor() {
	serial.Open(2, 9600);
}

Motor::~Motor() {
	if (serial.IsOpened()) {
		serial.Close();
	}
}
//
//void Motor::brakeInput(unsigned char datavalue) {
//	int PortNum =  FIRSTPORTA;
//	int Direction = DIGITALOUT;
//	int BoardNum = 0;
//	int ULStat = 0;
//	ULStat = cbDConfigPort(BoardNum, PortNum, Direction);
//	ULStat = cbDOut(BoardNum, PortNum, datavalue); //brake
//	printf("MC digital output %u V to pin4(brake/run) of pump", datavalue / 255 * 5);            
//	printf("\n");
//}

//ULONG Motor::getCount(int CounterNum) {
//	ULONG Count = 0;
//	int BoardNum = 0;
//	int ULStat = 0;
//	cbCIn32(BoardNum, CounterNum, &Count);
//	return Count;
//}

//void Motor::getCountClear(int CounterNum) {
//	int BoardNum = 0;
//	int ULStat = 0;
//	ULStat = cbCClear(BoardNum, CounterNum);
//	return;
//}

float Motor::getWeight(int com) {
	//reading data
	char lpBuffer[1000];

	if (serial.IsOpened())
	{
		while (true) {
			int num = serial.ReadData(lpBuffer, 1000);
			if (num == 0) break;
		}

		while (1)
		{
			int nBytesRead = serial.ReadData(lpBuffer, 1000);
	
			//	delete[]lpBuffer;						
			for (int i = nBytesRead-1; i >= 0; i--) {
				if (lpBuffer[i] == '\n' && i - 11 >= 0) {
					std::string num;
					for (int j = 10; j >= 4; j--) {
						num.push_back(lpBuffer[i - j]);
					}

					//std::cerr << "num: " << num << std::endl;

					float n = std::stof(num);
					
					return n;
				}
			}
			Sleep(30);
		}
	}
	else
	{
		std::cout << "Failed to open port!" << std::endl;
		return 1000.f;
	}
		
}

void Motor::getWeightClear(int com) {
	char a = 'z';
	if (serial.IsOpened())
	{
		serial.SendData(&a, 1);
		Sleep(100);
		serial.SendData(&a, 1);
	}
	else
		std::cout << "Failed to open port!" << std::endl;
}

