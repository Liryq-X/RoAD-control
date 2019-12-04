#include "exo_rgb_cam.h"

unsigned char* EXO_RGB_CAM::rgb_buffer_ = new unsigned char[1920*1200*3];

bool EXO_RGB_CAM::image_transfer_done;

cv::Mat EXO_RGB_CAM::rgb_frame;

std::mutex RGBMutex;

EXO_RGB_CAM::EXO_RGB_CAM()
{
	rgb_frame.create(1200, 1920, CV_8UC3);
}
 
EXO_RGB_CAM::~EXO_RGB_CAM()
{
	stop();
}

int EXO_RGB_CAM::init()
{
	printf("SVU3V-SDK Mini Sample\n"); printf("\n");
	//---------------------------------------------------------------------
	// Discovery
	//---------------------------------------------------------------------

	int Index = 0;
	
	CameraClient = CameraContainer_create();

	if (CameraClient < 0)
	{
		printf("Create CameraContainer  faild !!\n");
		
		return 0;
	}

	CameraContainer_discovery(CameraClient);
	int NumCameras = 0;
	NumCameras = CameraContainer_getNumberOfCameras(CameraClient);
	if (NumCameras < 1)
	{
		printf("no cameras found !!\n");
		return 0;
	}
	printf("Nbr of Cameras found:  %-10d \n", NumCameras);
	// select the first discovered camera
	Camera = CameraContainer_getCamera(CameraClient, Index);
	std::string modelname = Camera_getModelName(Camera);
	printf("Selected Camera: %s \n", modelname.c_str());

	SVU3V_RETURN result;
	result = Camera_openConnection(Camera, 3.0);

	if (result != U3V_STATUS_SUCCESS)
	{
		printf("Open connection failed,  is camera occupied ?  \n");
		return -1;
	}
///	Camera_getSizeX(Camera, &SizeX);
//	Camera_getSizeY(Camera, &SizeY);
	Camera_getImageSize(Camera, &ImageSize);
//	std::cout << SizeX << " " << SizeY <<" "<<ImageSize<< "\n";

	//-----------------------------------------------------------------------

	float red = 0;
	float green = 0;
	float blue = 0;
	Camera_getWhiteBalance(Camera, &red, &green, &blue);

	std::cout << "r: " << red << "g: " << green << "b: " << blue << "\n";

	Camera_setWhiteBalance(Camera, 121.484f, 100.f, 256.641f);

	Camera_setFlippingMode(Camera, SVU3V_REVERSE_X_Y);


	int BufferCount = 3;

	result = StreamingChannel_create(&StreamingChannel, CameraClient, Camera, BufferCount, &ImageCallback, 0);

	//---------------------------------------------------------------------
	// do some camera configurations
	//---------------------------------------------------------------------

	//------------------------------------------------------------------------
	Camera_setAcquisitionMode(Camera, ACQUISITION_MODE_SOFTWARE_TRIGGER);

	Camera_setExposureTime(Camera, exposure_time_);
	float Gain = 0; // 0 db
	Camera_setGain(Camera, Gain);
	/* if the  settings (binning mode, AOI, pixel depth, Flipping mode) have been changed
	after creating Streaming channel, a reopen of streaming channel is required: */

	//---------------------------------------------------------------------
	// start Acquisition: Software Trigger
	//---------------------------------------------------------------------

	Camera_setAcquisitionControl(Camera, ACQUISITION_CONTROL_START);

//	Sleep(1000);

//	acquireRGBImage();

	return 0;
}

void EXO_RGB_CAM::stop()
{
	//---------------------------------------------------------------------
	// close Camera and and unregister messages
	//---------------------------------------------------------------------
	printf("Stop Acquisition...\n");
	Camera_setAcquisitionControl(Camera, ACQUISITION_CONTROL_STOP);
	printf("Delete streaming channel...\n");
	StreamingChannel_delete(StreamingChannel);
	printf("Close connection...\n");
	Camera_closeConnection(Camera);
	printf("Press any key to end...");
}

int EXO_RGB_CAM::acquireRGBImage()
{
	image_transfer_done = false;
	try {
		Camera_softwareTrigger(Camera);
	}
	catch (std::exception e) {
		std::cout << "exception caught in Camera_softwareTrigger\n";
		std::cout << e.what() << std::endl;
	}
	
	
	int count = 0;

	int timeout = 0;

	try {
		while (!image_transfer_done)
		{
			count++;
			Sleep(10);
			if (count % 100 == 0)
				std::cerr << "count: "<<count<<" sleep in acquire RGB image" << std::endl;
			if (count * 10 / 1000 >5) //more than 10 sec //if (count * 10 / 1000 / 60 > 2) 2 minutes
			{
				timeout = -1;
				break;
			}
		}
	}
	catch (std::exception e) {
		std::cout << "exception caught in while loop\n";
		std::cout << e.what() << std::endl;
	}


	return timeout;
}




// stream callback function will be registered when creating streaming channel
//--------------------------------------------------------------------------------------------
SVU3V_RETURN __stdcall EXO_RGB_CAM::ImageCallback(Image_handle Image, void* Context)
{
	//unsigned char *img_buffer_ = Image_getDataPointer(Image);
//	int success = Image_getImageRGB(Image, rgb_buffer_, SizeX*SizeY*3, BAYER_METHOD_HQLINEAR);RGBMutex
//	RGBMutex.lock();
	int success;
	try {

		success = Image_getImageRGB(Image, rgb_frame.ptr<uchar>(0), SizeX*SizeY * 3, BAYER_METHOD_HQLINEAR);
		std::cerr << "success: " << success << std::endl;

	}
	catch (std::exception e) {
		std::cerr << "success: " << success << std::endl;
		std::cout << "exception caught in Image_getImageRGB\n";
		std::cout << e.what() << std::endl;
	}
	//success = Image_getImageRGB(Image, rgb_frame.ptr<uchar>(0), SizeX*SizeY * 3, BAYER_METHOD_HQLINEAR);
//	RGBMutex.lock();

//	std::cout << "return " << success << " ";

	try {
	
		if (success == 0) image_transfer_done = true;
	}
	catch (std::exception e) {
		std::cout << "exception caught in if (success == 0)\n";
		std::cout << e.what() << std::endl;
	}
	
//success = SVU3V_writeImageToBitmapFile("img.bmp", rgb_frame.ptr<uchar>(0), SizeX, SizeY, GVSP_PIX_RGB24);
//	success = SVU3V_writeImageToBitmapFile("img.bmp", rgb_buffer_, SizeX, SizeY, GVSP_PIX_RGB24);
	
	//printf("Callback:  %08x\n", *img_buffer_);
	//std::cout << "return " << success << "\n";
	return U3V_STATUS_SUCCESS;
}

