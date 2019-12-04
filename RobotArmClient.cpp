#include "RobotArmClient.h"


RobotArmClient::RobotArmClient()
{
	TCP_ALIVE = true;

	//initial dst pose, no need for Inf
	for (int i = 0; i < 6; i++)
		dst_cartesian_info_array[i] = 0.0;

	result = NULL;

	ptr = NULL;

	// Initialize Winsock ///////////////////////////
	iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
	
	if (iResult != 0) 
		printf("WSAStartup failed with error: %d\n", iResult);

	ZeroMemory(&hints, sizeof(hints));
	hints.ai_family = AF_INET;
	hints.ai_socktype = SOCK_STREAM;
	hints.ai_protocol = IPPROTO_TCP;

	// Resolve the server address and port
	iResult = getaddrinfo("192.168.0.2", DEFAULT_PORT, &hints, &result);
	if (iResult != 0) 
	{
		printf("getaddrinfo failed with error: %d\n", iResult);
		WSACleanup();
	}

	// Attempt to connect to an address until one succeeds
	for (ptr = result; ptr != NULL; ptr = ptr->ai_next) 
	{
		// Create a SOCKET for connecting to server
		ConnectSocket = socket(ptr->ai_family, ptr->ai_socktype, ptr->ai_protocol);

		if (ConnectSocket == INVALID_SOCKET) 
		{
			printf("socket failed with error: %ld\n", WSAGetLastError());
			WSACleanup();
			return;
		}

		// Disable Nagle algorithm for low-latency send
		char value = 1;

		if(setsockopt(ConnectSocket, IPPROTO_TCP, TCP_NODELAY, &value, sizeof(value)) != 0)
			std::cout<<"robot arm socket connect fail"<<std::endl;

		// Connect to server.
		iResult = connect(ConnectSocket, ptr->ai_addr, (int)ptr->ai_addrlen);

		if (iResult == SOCKET_ERROR) 
		{
			closesocket(ConnectSocket);
			ConnectSocket = INVALID_SOCKET;
			continue;
		}
		break;
	}

	freeaddrinfo(result);

	if (ConnectSocket == INVALID_SOCKET) {
		printf("Unable to connect to server!\n");
		WSACleanup();
		return;
	}   //////////////////

	URMsgHandler.push_back(std::thread(&RobotArmClient::startRecvTCP, this));

	Sleep(1000);
}

void RobotArmClient::startRecvTCP()
{
	////////////TCP_ALIVE = true;
	unsigned int cartesianStartID = 0x04650000;

	// Receive until the peer closes the connection
	int i = 0;

	// Initialize Winsock
	/*
	iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);

	if (iResult != 0)
		printf("WSAStartup failed with error: %d\n", iResult);

	ZeroMemory(&hints, sizeof(hints));
	hints.ai_family = AF_INET;
	hints.ai_socktype = SOCK_STREAM;
	hints.ai_protocol = IPPROTO_TCP;

	// Resolve the server address and port
	iResult = getaddrinfo("192.168.0.2", DEFAULT_PORT, &hints, &result);
	if (iResult != 0)
	{
		printf("getaddrinfo failed with error: %d\n", iResult);
		WSACleanup();
	}

	// Attempt to connect to an address until one succeeds
	for (ptr = result; ptr != NULL; ptr = ptr->ai_next)
	{
		// Create a SOCKET for connecting to server
		ConnectSocket = socket(ptr->ai_family, ptr->ai_socktype, ptr->ai_protocol);

		if (ConnectSocket == INVALID_SOCKET)
		{
			printf("socket failed with error: %ld\n", WSAGetLastError());
			WSACleanup();
			return;
		}

		// Disable Nagle algorithm for low-latency send
		char value = 1;

		if (setsockopt(ConnectSocket, IPPROTO_TCP, TCP_NODELAY, &value, sizeof(value)) != 0)
			std::cout << "robot arm socket connect fail" << std::endl;

		// Connect to server.
		iResult = connect(ConnectSocket, ptr->ai_addr, (int)ptr->ai_addrlen);

		if (iResult == SOCKET_ERROR)
		{
			closesocket(ConnectSocket);
			ConnectSocket = INVALID_SOCKET;
			continue;
		}
		break;
	}

	freeaddrinfo(result);

	if (ConnectSocket == INVALID_SOCKET) {
		printf("Unable to connect to server!\n");
		WSACleanup();
		return;
	} */

	while (TCP_ALIVE) 
	{
		//clock_t tic = clock();

		iResult = recv(ConnectSocket, recvbuf, recvbuflen, 0);

		/*if (iResult > 0)
		printf("Bytes received: %d\n", iResult);
		else if (iResult == 0)
		printf("Connection closed\n");
		else
		printf("recv failed with error: %d\n", WSAGetLastError());*/

		//if (iResult == 635) // 10Hz
		if (iResult == 1108)	//125Hz    //v3.5:1108
		{
			updateMutex.lock();
			// index 307 is the cartesion info x start
			getCartesionInfoFromURPackage(recvbuf + 444);	// 125 Hz
			//getCartesionInfoFromURPackage(recvbuf + 307);	// 10 Hz

			getActualJointPosFromURPackage();

			getActualTCPSpeedFromURPackage();

			tcp_speed_ = sqrt(cur_tcp_speed_array[0]* cur_tcp_speed_array[0] + cur_tcp_speed_array[1] * cur_tcp_speed_array[1] + cur_tcp_speed_array[2] * cur_tcp_speed_array[2]);

			distanceToDst_ = EuclideanDistance(cur_cartesian_info_array, dst_cartesian_info_array);

			displacement_ = EuclideanDistance(cur_cartesian_info_array, start_xyz_);

			distanceToDstConfig_ = L1Distance(cur_joint_pos_array, dst_joint_pos_array);

			updateMutex.unlock();

			//std::cout << distanceToDst << std::endl;

			//printCartesianInfo(cur_cartesian_info_array);


			/*unsigned char* tmp_buffer = (unsigned char*)recvbuf;

			for (int j = 0; j < iResult - 4; j++)
			{
			std::cout << (unsigned int)tmp_buffer[j]<<" ";

			if (std::memcmp(recvbuf + j, &cartesianStartID, 4) == 0)
			{
			std::cout << "found" << j <<" "<< (unsigned int)tmp_buffer[j+4] << std::endl;
			}

			if (tmp_buffer[j] == 4)
			std::cout << std::endl;
			}*/

			//break;
		}
		//else
			//std::cerr << "UR msg size wrong:" << iResult << std::endl;

		//clock_t toc = clock();
		//printf("Elapsed: %f ms\n", (double)(toc - tic) / CLOCKS_PER_SEC * 1000.);

		/*if (i++ == 500)
			break;*/
	}


	// cleanup
	closesocket(ConnectSocket);

	WSACleanup();
}


void RobotArmClient::reverse8CharsToDouble(char* end, double* d)
{
	char* tmpCharPtr = (char*)d;

	for (int i = 0; i < 8; i++)
	{
		*tmpCharPtr = *end;
		tmpCharPtr++;
		end--;
	}
}

void RobotArmClient::getCartesionInfoFromURPackage(char* x_start_ptr)
{
	for (int i = 0; i < 6; i++)
	{
		reverse8CharsToDouble(x_start_ptr + 7 + i * 8, cur_cartesian_info_array + i);
	}
}

void RobotArmClient::getActualJointPosFromURPackage()
{
	char* start_ptr = recvbuf + 252;

	for (int i = 0; i < 6; i++)
	{
		reverse8CharsToDouble(start_ptr + 7 + i * 8, cur_joint_pos_array + i);
	}
}

void RobotArmClient::getActualTCPSpeedFromURPackage()
{
	char* start_ptr = recvbuf + 492;

	for (int i = 0; i < 6; i++)
	{
		reverse8CharsToDouble(start_ptr + 7 + i * 8, cur_tcp_speed_array + i);
	}
}

int RobotArmClient::moveHandL(double* dst_cartesian_info, float acceleration, float speed)
{
	char msg[128];

	//updateMutex.lock();
	distanceToDst_.store(1000.);
	//updateMutex.unlock();

	sprintf(msg, "movel(p[%f,%f,%f,%f,%f,%f],a=%f,v=%f)\n", 
		dst_cartesian_info[0], dst_cartesian_info[1], dst_cartesian_info[2], 
		dst_cartesian_info[3], dst_cartesian_info[4], dst_cartesian_info[5], acceleration, speed);

	//std::cout << "len" << strlen(msg)<<std::endl;
	return send(ConnectSocket, msg, strlen(msg), 0);
}

int RobotArmClient::moveHandJ(double* dst_joint_config, float speed, float acceleration, bool wait2dst)
{
	char msg[128];

	updateMutex.lock();
	distanceToDstConfig_ = 1000.;
	updateMutex.unlock();

	sprintf(msg, "movej([%f,%f,%f,%f,%f,%f],a=%f,v=%f)\n",
		dst_joint_config[0], dst_joint_config[1], dst_joint_config[2],
		dst_joint_config[3], dst_joint_config[4], dst_joint_config[5], acceleration, speed);

	//std::cout << "len" << strlen(msg)<<std::endl;
	
	int num_byte = send(ConnectSocket, msg, strlen(msg), 0);

	if (num_byte == SOCKET_ERROR) return SOCKET_ERROR;

	if (wait2dst) {
		
		if (waitTillHandReachDstConfig(dst_joint_config) == -1)
			std::cerr << "waitTillHandReachDstConfig(dst_joint_config) timeout\n";
	
	}

	return num_byte;
}

void RobotArmClient::getCartesianInfo(double* array6)
{
	updateMutex.lock();

	std::memcpy(array6, cur_cartesian_info_array, 6 * 8);

	updateMutex.unlock();
}

void RobotArmClient::getTCPSpeed(double* array6)
{
	updateMutex.lock();

	std::memcpy(array6, cur_tcp_speed_array, 6 * 8);

	updateMutex.unlock();
}

void RobotArmClient::getCurJointPose(double* array6)
{
	updateMutex.lock();

	std::memcpy(array6, cur_joint_pos_array, 6 * 8);

	updateMutex.unlock();
}

void RobotArmClient::printCartesianInfo(double* array6)
{
	std::cerr << "cartesian: ";
	for (int j = 0; j < 6; j++)
		std::cerr << array6[j] << " ";
	std::cerr << '\n';
}

// only compute xyz distance
double RobotArmClient::EuclideanDistance(double* pose6_1, double* pose6_2)
{
	double sum_squares = 0.;

	for (int i = 0; i < 3; i++)
	{
		double diff = pose6_1[i] - pose6_2[i];
		
		sum_squares += diff*diff;
	}

	return sqrt(sum_squares);
}

double RobotArmClient::L1Distance(double* pose6_1, double* pose6_2)
{
	double L1 = 0.;

	for (int i = 0; i < 6; i++)
	{
		double diff = std::abs(pose6_1[i] - pose6_2[i]);

		if (diff > L1)
		{
			L1 = diff;
		}
	}

	return L1;
}

int RobotArmClient::waitTillHandReachDstPose(double* dst_pose6)
{
	int timeout = 0;

	setDstCartesianInfo(dst_pose6);

	int count = 0;

	while (true)
	{
		
		if (distanceToDst_.load() < 0.003)	break;

		Sleep(1);	

		if (distanceToDst_.load() < 0.008) {
			count++;
			Sleep(1);
			if (count % 2000 == 0)
				std::cout << count << "dist: " << distanceToDst_.load() << "  sleep in wait till hand reach dst pose" << std::endl;

			if (count * 1 / 1000 > 30) {

				timeout = -1;

				break;

			}//30 sec
		}

/*		if (distanceToDst_.load()>800) {
			stopRecvTCP();
			URMsgHandler.clear();
			TCP_ALIVE = TRUE;
			startRecvTCP();

			std::ofstream myfile;
			myfile.open("distanceToDstConfig_timeout_output.txt");
			myfile << "get waitTillHandReachDstPose timeout.\n";
			myfile.close();
		}*/
			
	}

	Sleep(200);

	return timeout;
}

int RobotArmClient::waitTillHandReachDstConfig(double* dst_joint_config)
{
	setDstConfigInfo(dst_joint_config);

	int count = 0;

	int timeout = 0;

	while (true)
	{

		if (distanceToDstConfig_ < 0.005)
			break;

		Sleep(4);
		

		if (distanceToDstConfig_ < 0.008) {
			count++;
			Sleep(4);
			
			if (count % 500 == 0)
				std::cout << count << "dist: " << distanceToDst_.load() << " sleep in wait till hand reach dst config" << std::endl;

			if (count * 4 / 1000 >30) {

				timeout = -1;

				break;

			}//30 sec
		}

		/*
		if (distanceToDstConfig_>800) {
			stopRecvTCP();
			URMsgHandler.clear();
			TCP_ALIVE = TRUE;
			URMsgHandler.push_back(std::thread(&RobotArmClient::startRecvTCP, this));

			std::ofstream myfile;
			myfile.open("distanceToDstConfig_timeout_output.txt");
			myfile << "get waitTillHandReachDstPose timeout.\n";
			myfile.close();
		}*/

	}

	Sleep(200);

	return timeout;
}

void RobotArmClient::setDstCartesianInfo(double* array6)
{
	updateMutex.lock();
	std::memcpy(dst_cartesian_info_array, array6, 48);
	distanceToDst_.store(1000.);
	updateMutex.unlock();
}


void RobotArmClient::setDstConfigInfo(double* array6)
{
	updateMutex.lock();
	std::memcpy(dst_joint_pos_array, array6, 48);
	distanceToDstConfig_ = 1000.;
	updateMutex.unlock();
}

void RobotArmClient::getDistanceToDst(double& distance)
{
	//updateMutex.lock();
	distance = distanceToDst_.load(); 
	//updateMutex.unlock();
}

void RobotArmClient::stopRecvTCP(void)
{
	TCP_ALIVE = false;
	URMsgHandler.back().join();
}

void RobotArmClient::setStartPoseXYZ()
{
	double pose[6];
	getCartesianInfo(pose);
	std::memcpy(start_xyz_, pose, 3*sizeof(double));
	displacement_.store(0);
}

void RobotArmClient::waitTillTCPMove()
{
	//int counter = 0;
	while (true)
	{
		//counter++;
		if (tcp_speed_.load() >= 0.01f) break;
		//if (displacement_.load() >= 0.0001)	break;
		Sleep(1);
	}

	getCartesianInfo(sync_pose_);
	getTCPSpeed(tcp_sync_speed_);

	//return counter;
}

void RobotArmClient::rotateJointRelative(int id, double deg, float acceleration, float speed)
{
	if (id < 0 || id > 5)
	{
		std::cerr << "invalid joint id\n";
		return;
	}

	double joints[6];
	getCurJointPose(joints);
	joints[id] += deg*M_PI/180.;
	moveHandJ(joints, speed, acceleration, true);
}

void RobotArmClient::moveHandRelativeTranslate(double x, double y, double z, float acceleration, float speed)
{
	double pose[6];
	getCartesianInfo(pose);
	pose[0] += x; 
	pose[1] += y; 
	pose[2] += z;
	moveHandL(pose, acceleration, speed);
	waitTillHandReachDstPose(pose);
}

//R
int RobotArmClient::setAnalogOutput(int ID, float value) {
	char msg[128];

	sprintf(msg, "set_analog_out(%d,%f)\n",
		ID, value);

	return send(ConnectSocket, msg, strlen(msg), 0);
}

int RobotArmClient::setDigitalOutput(int ID, bool b) {
	char msg[128];
	if (b) {
		sprintf(msg, "set_digital_out(%d,True)\n",ID);
	 }
	else {
		sprintf(msg, "set_digital_out(%d,False)\n",ID);
	}
	return send(ConnectSocket, msg, strlen(msg), 0);
}
