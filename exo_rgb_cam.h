#ifndef EXO_RGB_CAM_H_
#define EXO_RGB_CAM_H

#include "SVU3V.h"
#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <Windows.h>
#include <mutex>

struct EXO_RGB_CAM
{
	CameraContainerClient_handle CameraClient;

	Camera_handle Camera;

	StreamingChannel_handle StreamingChannel;

	static const int SizeX = 1920;
	static const int SizeY = 1200;
	int ImageSize;

	static unsigned char *rgb_buffer_;

	static cv::Mat rgb_frame;

	static bool image_transfer_done;

	static std::mutex RGBMutex;

	float exposure_time_ = 1400000.;	//us

	EXO_RGB_CAM::EXO_RGB_CAM();
	
	EXO_RGB_CAM::~EXO_RGB_CAM();
	
	int init();

	void stop();

	int acquireRGBImage();

	// stream callback function will be registered when creating streaming channel
	//--------------------------------------------------------------------------------------------
	static SVU3V_RETURN __stdcall ImageCallback(Image_handle Image, void* Context);

};
#endif
