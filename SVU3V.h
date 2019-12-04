/*******************************************************************************
 * SVS U3V API   Declaration of U3V camera access functions
 *******************************************************************************
 *
 * Version:     2.1.0
 *
 * Copyright:   SVS VISTEK GmbH
 *
 * Function categories:
 * ---------------------------------------------------------
 *    1 - Camera: Discovery and bookkeeping
 *    2 - Camera: Connection
 *    3 - Camera: Information
 *    4 - Stream: Channel creation and control
 *    5 - Stream: Channel info
 *    6 - Stream: Image access
 *    7 - Stream: Image conversion
 *   8 - Stream: Image characteristics
 *   9 - Stream: Image statistics
 *   10 - Controlling camera: Frame rate
 *   11 - Controlling camera: Exposure
 *   12 - Controlling camera: Gain and offset
 *   13 - Controlling camera: Auto gain/exposure
 *   14 - Controlling camera: Acquisition trigger
 *   15 - Controlling camera: Strobe
 *   16 - Controlling camera: Tap balance
 *   17 - Controlling camera: Image parameter
 *   18 - Controlling camera: Image appearance
 *   19 - Special control: IOMux configuration
 *   20 - Special control: IO control
 *   21 - Special control: Serial communication
 *   22 - Special control: Direct register and memory access
 *   23 - Special control: Persistent settings and recovery
 *   24 - General functions
 *   25 - Special control: Lens control
 * ---------------------------------------------------------

  * Revision history:
 *
 ** Version 2.1.0
 * --------------------

 *	clean up in c# interface.
 *  function Camera_getGainMax() is added. 
 *  bug fixing in function Camera_getAcquisitionMode().
 *
 ** Version 2.0.0
 * --------------------
 *  First release for U3V SDK.
 *  All Functions are from GigE SDK imported.
 
 
 *******************************************************************************
 * Detailed function listing
 *******************************************************************************
 * -------------------------------------
 *  isVersionCompliantDLL()
 *
 * 1 - Camera: Discovery and bookkeeping
 * -------------------------------------
 *  CameraContainer_create()
 *  CameraContainer_delete()
 *  CameraContainer_discovery()
 *  CameraContainer_getNumberOfCameras()
 *  CameraContainer_getCamera()
 *  CameraContainer_findCamera()
 *
 * 2 - Camera: Connection
 * -------------------------------------
 *  Camera_openConnection()
 *  Camera_closeConnection()
 *
 * 3 - Camera: Information
 * -----------------------
 *  Camera_getManufacturerName()
 *  Camera_getModelName()
 *  Camera_getDeviceVersion()
 *  Camera_getManufacturerSpecificInformation()
 *  Camera_getSerialNumber()
 *  Camera_setUserDefinedName()
 *  Camera_getUserDefinedName()
 *  Camera_getPixelClock()
 *  Camera_isCameraFeature()
 *  Camera_readXMLFile()
 *  Camera_getSensorTemperature()
 *
 * 4 - Stream: Channel creation and control
 * ----------------------------------------
 *  StreamingChannel_create()
 *  StreamingChannel_delete()
 *
 * 5 - Stream: Channel info
 * ------------------------
 *  StreamingChannel_getPixelType()
 *  StreamingChannel_getImagePitch()
 *  StreamingChannel_getImageSizeX()
 *  StreamingChannel_getImageSizeY()
 *
 * 6 - Stream: Image access
 * ------------------------
 *  Image_getDataPointer()
 *  Image_getSignalType()
 *  Image_getCamera()
 *  Image_release()
 *
 * 7 - Stream: Image conversion
 * ----------------------------
 *  Image_getImageRGB()
 *  Image_getImage12bitAs8bit()
 *  Image_getImage12bitAs16bit()
 *  Image_getImage16bitAs8bit()
 *		
 * 8 - Stream: Image characteristics
 * ----------------------------------
 *  Image_getPixelType()
 *  Image_getImageSize()
 *  Image_getPitch()
 *  Image_getSizeX()
 *  Image_getSizeY()
 *
 * 9 - Stream: Image statistics
 * ----------------------------
 *  Image_getImageID()
 *  Image_getTimestamp()
 * ------------------------------
 *  
 * 10 - Controlling camera: Frame rate
 * -----------------------------------
 *  Camera_setFrameRate()
 *  Camera_getFrameRate()
 *  Camera_getFrameRateMin()
 *  Camera_getFrameRateMax()
 *  Camera_getFrameRateRange()
 *  Camera_getFrameRateIncrement()
 *
 * 11 - Controlling camera: Exposure
 * ---------------------------------
 *  Camera_setExposureTime()
 *  Camera_getExposureTime()
 *  Camera_getExposureTimeMin()
 *  Camera_getExposureTimeMax()
 *  Camera_getExposureTimeRange()
 *  Camera_getExposureTimeIncrement()
 *  Camera_setExposureDelay()
 *  Camera_getExposureDelay()
 *  Camera_getExposureDelayMax()
 *  Camera_getExposureDelayIncrement()
 *
 * 12 - Controlling camera: Gain and offset
 * ----------------------------------------
 *  Camera_setGain()
 *  Camera_getGain()
 *  Camera_getGainMax()
 *  Camera_getGainMaxExtended()
 *  Camera_getGainIncrement()
 *  Camera_setOffset()
 *  Camera_getOffset()
 *  Camera_getOffsetMax()
 *
 * 13 - Controlling camera: Auto gain/exposure
 * -------------------------------------------
 *  Camera_setAutoGainEnabled()
 *  Camera_getAutoGainEnabled()
 *  Camera_setAutoGainBrightness()
 *  Camera_getAutoGainBrightness()
 *  Camera_setAutoGainDynamics()
 *  Camera_getAutoGainDynamics()
 *  Camera_setAutoGainLimits()
 *  Camera_getAutoGainLimits()
 *  Camera_setAutoExposureLimits()
 *  Camera_getAutoExposureLimits()
 *
 * 14 - Controlling camera: Acquisition trigger
 * --------------------------------------------
 *  Camera_setAcquisitionControl()
 *  Camera_getAcquisitionControl()
 *  Camera_setAcquisitionMode()
 *  Camera_getAcquisitionMode()
 *  Camera_softwareTrigger()
 *  Camera_setTriggerPolarity()
 *  Camera_getTriggerPolarity()
 *  Camera_setPivMode()
 *  Camera_getPivMode()
 *  Camera_setDebouncerDuration()
 *  Camera_getDebouncerDuration()
 *  Camera_setPrescalerDevisor()
 *  Camera_getPrescalerDevisor()
 *  Camera_loadSequenceParameters()
 *  Camera_startSequencer()
 *
 * 15 - Controlling camera: Strobe
 * -------------------------------

 *  Camera_setStrobePolarityExtended()
 *  Camera_getStrobePolarityExtended()
 *  Camera_setStrobePositionExtended()
 *  Camera_getStrobePositionExtended()
 *  Camera_getStrobePositionMax()
 *  Camera_getStrobePositionIncrement()
 *  Camera_setStrobeDurationExtended()
 *  Camera_getStrobeDurationExtended()
 *  Camera_getStrobeDurationMax()
 *  Camera_getStrobeDurationIncrement()
 *
 * 16 - Controlling camera: Tap balance
 * ------------------------------------

 *  Camera_setTapConfigurationEx()
 *  Camera_getTapConfigurationEx()
 *  Camera_setAutoTapBalanceMode()
 *  Camera_getAutoTapBalanceMode()
 *  Camera_setTapGain()
 *  Camera_getTapGain()
 *
 * 17 - Controlling camera: Image parameter
 * ---------------------------------------
 *  Camera_getImagerWidth()
 *  Camera_getImagerHeight()
 *  Camera_getImageSize()
 *  Camera_getPitch()
 *  Camera_getSizeX()
 *  Camera_getSizeY()
 *  Camera_setBinningMode()
 *  Camera_getBinningMode()
 *  Camera_setAreaOfInterest()
 *  Camera_getAreaOfInterest()
 *  Camera_getAreaOfInterestRange()
 *  Camera_getAreaOfInterestIncrement()
 *  Camera_setFlippingMode()	
 *	Camera_getFlippingMode()
 *	Camera_setShutterMode()
 *	Camera_getShutterMode()
 *

 * 18 - Controlling camera: Image appearance
 * -----------------------------------------
 *  Camera_getPixelType()
 *  Camera_setPixelDepth()
 *  Camera_getPixelDepth()
 *  Camera_setWhiteBalance()
 *  Camera_getWhiteBalance()
 *  Camera_getWhiteBalanceMax()
 *  Camera_setGammaCorrection()
 *  Camera_setGammaCorrectionExt()
 *  Camera_setLowpassFilter()
 *  Camera_getLowpassFilter()
 *  Camera_setLookupTableMode()
 *  Camera_getLookupTableMode()
 *  Camera_setLookupTable()
 *  Camera_getLookupTable()
 *  Camera_startImageCorrection()
 *  Camera_isIdleImageCorrection()
 *  Camera_setImageCorrection()
 *  Camera_getImageCorrection()
 *  Camera_setPixelsCorrectionMap()
 *  Camera_getPixelsCorrectionMap()
 *  Camera_setPixelsCorrectionControlEnabel()	
 *	Camera_getPixelsCorrectionControlEnabel()
 *	Camera_setPixelsCorrectionControlMark()
 *	Camera_getPixelsCorrectionControlMark()
 *  Camera_setPixelsCorrectionMapOffset()
 *  Camera_getPixelsCorrectionMapOffset()
 *  Camera_getPixelsCorrectionMapSize()
 *  Camera_getMaximalPixelsCorrectionMapSize()
 *  Camera_getMapIndexCoordinate()
 *  Camera_deletePixelCoordinateFromMap()
 *   
 *
 * 19 - Special control: IOMux configuration
 * -------------------------------------------------------
 *  Camera_getMaxIOMuxIN()
 *  Camera_getMaxIOMuxOUT()
 *  Camera_setIOAssignment()
 *  Camera_getIOAssignment()
 *
 * 20 - Special control: IO control
 * -------------------------------------------------------
 *  Camera_setIOMuxIN()
 *  Camera_getIOMuxIN()
 *  Camera_setIO()
 *  Camera_getIO()
 *  Camera_setAcqLEDOverride()
 *  Camera_getAcqLEDOverride()
 *  Camera_setLEDIntensity()
 *  Camera_getLEDIntensity()
 *
 * 21 - Special control: Serial communication
 * -------------------------------------------------------
 *  Camera_setUARTBuffer()
 *  Camera_getUARTBuffer()
 *  Camera_setUARTBaud()
 *  Camera_getUARTBaud()
 *
 * 22 - Special control: Direct register and memory access
 * -------------------------------------------------------
 *  Camera_setU3VCameraRegister()
 *  Camera_getU3VCameraRegister()
 *  Camera_writeU3VCameraMemory()
 *  Camera_readU3VCameraMemory()
 *
 * 23 - Special control: Persistent settings and recovery
 * ------------------------------------------------------
 *  Camera_writeEEPROM()
 *  Camera_readEEPROM()
 *  Camera_restoreFactoryDefaults()
 *  Camera_loadSettingsFromXml()  
 *  Camera_SaveSettingsToXml()  
 *
 * 24 - General functions
 * ----------------------
 *  SVU3V_estimateWhiteBalance()
 *  SVU3V_estimateWhiteBalanceExtended()
 *  SVU3V_writeImageToBitmapFile()
 *  
 * 25 - Special control: Lens control
 * ------------------------------------------------------
 *  Camera_isLensAvailable()
 *  Camera_getLensName()
 *  Camera_setLensFocalLenght()
 *  Camera_getLensFocalLenght()
 *  Camera_getLensFocalLenghtMin()
 *  Camera_getLensFocalLenghtMax()
 *  Camera_setLensFocusUnit()
 *  Camera_getLensFocusUnit() 
 *  Camera_setLensFocus()
 *  Camera_getLensFocus() 
 *  Camera_getLensFocusMin()
 *  Camera_getLensFocusMax() 
 *  Camera_setLensAperture()
 *  Camera_getLensAperture()
 *  Camera_getLensApertureMin()
 *  Camera_getLensApertureMax()
 */
      


#ifndef SVU3VH
#define SVU3VH

#ifdef __BORLANDC__
  // Generate enums of integer size
#pragma option push -b
#endif

#include <stdio.h>
#include <stdlib.h>
#include <tchar.h>

//---------------------------------------------------------------------------

extern "C"
{
  /** Version information
   *  The particular components of the version information will be represented
   *  in the following way
   */
#ifndef SVU3V_VERSION_DEFINED
#define SVU3V_VERSION_DEFINED
  typedef struct
  {
    unsigned char	MajorVersion;
    unsigned char   MinorVersion;
    unsigned char	RevisionVersion;
    unsigned char	BuildVersion;

  } SVU3V_VERSION; 

  /** Version settings
   *  The following version settings apply to current SVU3V SDK:
   */
  #define SVU3V_VERSION_MAJOR			 2
  #define SVU3V_VERSION_MINOR			 1
  #define SVU3V_VERSION_REVISION		 0
  #define SVU3V_VERSION_BUILD			 10
#endif


  /** Camera Container client handle.
   *  A camera container client handle serves as a reference for managing multiple
   *  clients that are connected to a single camera container (which has no notion
   *  about clients). A value of SVU3V_NO_CLIENT serves as an indicator for
   *  an invalid camera container client before obtaining a valid handle.
   */
  typedef int CameraContainerClient_handle;
  #define SVU3V_NO_CLIENT	(CameraContainerClient_handle)-1

  /** Camera handle.
   *  A camera handle serves as a reference for access functions to camera
   *  functionality. A value of SVU3V_NO_CAMERA serves as an indicator for an
   *  invalid camera handle before obtaining a camera from a camera container.
   */
  typedef void * Camera_handle;
  #define SVU3V_NO_CAMERA	(Camera_handle)-1

  /** Streaming channel handle.
   *  A streaming channel handle serves as a reference for an image stream
   */
  typedef void * StreamingChannel_handle;
  #define SVU3V_NO_STREAMING_CHANNEL	(StreamingChannel_handle)-1

  /** Event handle.
   *  An event handle serves as a reference for a messaging channel
   */
 typedef void * Event_handle;
  #define SVU3V_NO_EVENT	(Event_handle)-1

  /** Message handle.
   *  A message handle serves as a reference for a single message
   */
 // typedef int Message_handle;
//  #define SVU3V_NO_MESSAGE	(Message_handle)-1

  /** Image handle.
   *  An image handle serves as a reference for picture access and
   *  processing functions
   */
  typedef int Image_handle;

  /** Function returns.
   *  API Functions may return success or error codes by this data type unless
   *  they return specific values
   */

typedef void * Stream_handle;
typedef void * Camera_handle;     
    

typedef enum
{

 U3V_STATUS_SUCCESS        = 0x0000 ,

// GenCP Status Codes
//GENCP_SUCCESS             = 0x0000
 GENCP_NOT_IMPLEMENTED     = 0x8001,
 GENCP_INVALID_PARAMETER    =0x8002,
 GENCP_INVALID_ADDRESS		= 0x8003,
 GENCP_WRITE_PROTECT		 = 0x8004,
 GENCP_BAD_ALIGNMENT       = 0x8005,
 GENCP_ACCESS_DENIED       = 0x8006,
 GENCP_BUSY                = 0x8007,
 GENCP_MSG_TIMEOUT         = 0x800B,
 GENCP_INVALID_HEADER      = 0x800E,
 GENCP_WRONG_CONFIG        = 0x800F,
 GENCP_ERROR               = 0x8FFF,

// U3V Status Codes
 U3V_STATUS_RESEND_NOT_SUPPORTED          = 0xA001,
 U3V_STATUS_DSI_ENDPOINT_HALTED           = 0xA002,
 U3V_STATUS_SI_PAYLOAD_SIZE_NOT_ALIGNED   = 0xA003,
 U3V_STATUS_SI_REGISTERS_INCONSISTENT     = 0xA004,
 U3V_STATUS_DATA_DISCARDED                = 0xA100,
 U3V_STATUS_DATA_OVERRUN                  = 0xA101,

// Device Specific Status Codes
 DS_STATUS_ERROR_USB_OPEN                    = 0xC001,
 DS_STATUS_ERROR_CONFIG_DESCRIPTOR           = 0xC002,
 DS_STATUS_NO_INTERFACES                     = 0xC003,
 DS_STATUS_ERROR_GET_DESCRIPTOR              = 0xC004,
 DS_STATUS_ERROR_STRING_DESCRIPTOR           = 0xC005,
 DS_STATUS_ERROR_OPEN_CONTROL_INTERFACE      = 0xC006,
 DS_STATUS_CONTROL_INTERFACE_NOT_OPEN        = 0xC007,
 DS_STATUS_WRONG_CONTROL_PREFIX              = 0xC008,
 DS_STATUS_WRONG_ACK                         = 0xC009,
 DS_STATUS_WRONG_REQUEST_ID                  = 0xC00A,
 DS_STATUS_XML_SIZE_IS_0                     = 0xC00B,
 DS_STATUS_XML_READ_MEM_ERROR                = 0xC00C,
 DS_STATUS_ERROR_OPEN_XML_FILE               = 0xC00D,
 DS_STATUS_XML_WRITE_MEM_ERROR               = 0xC00E,
 DS_STATUS_WRITE_MANTFEST_TABLE_ERROR        = 0xC00F,
 DS_STATUS_NO_EVENT_INTERFACE_AVAILABLE      = 0xC010,
 DS_STATUS_ERROR_OPEN_EVENT_INTERFACE        = 0xC011,
 DS_STATUS_EVENT_INTERFACE_NOT_OPEN          = 0xC012,
 DS_STATUS_NO_EVENT_SUPPORTED                = 0xC013,
 DS_STATUS_NO_STREAM_INTERFACE_AVAILABLE     = 0xC014,
 DS_STATUS_ERROR_OPEN_STREAM_INTERFACE       = 0xC015,
 DS_STATUS_STREAM_INTERFACE_NOT_OPEN         = 0xC016,
 DS_STATUS_NO_IMAGE_AVAILABLE                = 0xC017,
 DS_STATUS_IMAGE_TIMEOUT                     = 0xC018,
 DS_STATUS_IMAGE_MISSING_PACKETS             = 0xC019,
 DS_STATUS_INVALID_HANDLE                    = 0xC01A,
 DS_STATUS_ERROR_CALL_FUNCTION               = 0xC01B,
 DS_STATUS_DEVICE_NOT_FOUND                  = 0xC01C,
 DS_STATUS_XML_UNZIP_ERROR                   = 0xC01D,

//SVS Errors
 CAMERA_REGISTER_ACCESS_DENIED              = 0xD001,
 INVALID_PARAMETER                          = 0xD002,
 FEATURE_NOT_SUPPORTED						= 0xD003,
 INSUFFICIENT_DESTINATION_BUFFER_SIZE       = 0xD004,
 REGISTER_ACCESS_DENIED                     = 0xD005,
 XML_PATH_NOT_FOUND							= 0xD006,
 STATUS_XML_PARSE_ERROR						= 0xD007,
 USER_DEFINED_NAME_TOO_LONG					= 0xD008,
 INSUFFICIENT_RGB_BUFFER_PROVIDED			= 0xD009,
 PIXEL_TYPE_NOT_SUPPORTED					= 0xD00A,
 DLL_VERSION_MISMATCH						= 0xD00B,
 LUT_NOT_AVAILABLE				= 0xD00C,
 LUT_SIZE_MISMATCH				= 0xD00D,
 U3V_DLL_NOT_LOADED						= 0xD00E,
 U3V_CAMERA_CONTAINER_NOT_AVAILABLE = 0xD00F,
 U3V_CAMERA_NOT_FOUND =  0xD010,
 U3V_XML_FILE_NOT_AVAILABLE = 0xD011,
 Callback_FUNCTION_ALREADY_REGISTERED =0xD012,
 STREAM_STRUCTURE_OUT_OF_MEMOR =0xD013,
 STREAMING_FUNCTION_ALREADY_REGISTERED =0xD014,
 INVALID_CALLBACK_FUNCTION_POINTER = 0xD015,
 INVALID_STREAMM_HANDEL= 0xD016,
 U3V_STREAMING_NOT_INITIALIZED = 0xD017,
 IMAGE_NOT_AVAILABLE = 0xD018,
 FILE_COULD_NOT_BE_OPENED = 0xD019,
 WHITE_BALANCE_FAILED = 0xD01A,
 BINNING_MODE_NOT_AVAILABLE = 0xD01B,
 SU3V_CALLBACK_IMAGE_DATA_MISSING = 0xD01C,

} SVU3V_RETURN;



  /** Camera feature information.
   *  A camera can support several items from the following set of camera features.
   */
  typedef enum
  {
    CAMERA_FEATURE_SOFTWARE_TRIGGER                    = 0,  // camera can be triggered by software
    CAMERA_FEATURE_HARDWARE_TRIGGER                    = 1,  // hardware trigger supported as well as trigger polarity
    CAMERA_FEATURE_HARDWARE_TRIGGER_EXTERNAL_EXPOSURE  = 2,  // hardware trigger with internal exposure supported as well as trigger polarity
    CAMERA_FEATURE_FRAMERATE_IN_FREERUNNING_MODE       = 3,  // framerate can be adjusted (in free-running mode)
    CAMERA_FEATURE_EXPOSURE_TIME                       = 4,  // exposure time can be adjusted
    CAMERA_FEATURE_EXPOSURE_DELAY                      = 5,  // exposure delay can be adjusted
    CAMERA_FEATURE_STROBE                              = 6,  // strobe is supported (polarity, duration and delay)
    CAMERA_FEATURE_AUTOGAIN                            = 7,  // autogain is supported
    CAMERA_FEATURE_ADCGAIN                             = 8,  // 2009-05-15/deprecated
    CAMERA_FEATURE_GAIN                                = 8,  // the ADC's gain can be adjusted
    CAMERA_FEATURE_AOI                                 = 9,  // image acquisition can be done for an AOI (area of interest)
    CAMERA_FEATURE_BINNING                             = 10, // binning is supported
    CAMERA_FEATURE_UPDATE_REGISTER                     = 11, // streaming channel related registers can be pre-set and then updated at once (e.g. for changing an AOI)
    CAMERA_FEATURE_NOT_USED                            = 12, // not in use
    CAMERA_FEATURE_COLORDEPTH_8BPP                     = 13, // a pixel depth of 8bit is supported
    CAMERA_FEATURE_COLORDEPTH_10BPP                    = 14, // a pixel depth of 10bit is supported
    CAMERA_FEATURE_COLORDEPTH_12BPP                    = 15, // a pixel depth of 12bit is supported
    CAMERA_FEATURE_COLORDEPTH_16BPP                    = 16, // a pixel depth of 16bit is supported
    CAMERA_FEATURE_ADCOFFSET                           = 17, // the ADC's offset can be adjusted
    CAMERA_FEATURE_SENSORDATA                          = 18, // the camera's sensor/ADC settings can be adjusted (for dual tap cameras)
    CAMERA_FEATURE_WHITEBALANCE                        = 19, // a LUT for whitebalancing is available
    CAMERA_FEATURE_LUT_10TO8                           = 20, // a LUT from 10 bit to 8 bit is available
    CAMERA_FEATURE_LUT_12TO8                           = 21, // a LUT from 12 bit to 8 bit is available
	CAMERA_FEATURES_FLAGS           				   = 22, // streaming state and image availability can be queried from camera
	CAMERA_FEATURES_READOUT_CONTROL      			   = 23, // time of image read out from camera can be controlled by application
	CAMERA_FEATURES_TAP_CONFIG         				   = 24, // the tap configuration can be changed (switching between one and two taps)
	CAMERA_FEATURES_ACQUISITION        				   = 25, // acquisition can be controlled by start/stop
	CAMERA_FEATURES_TAPBALANCE						   = 26, // camera is capable of running auto tap balance
	CAMERA_FEATURES_BINNING_V2						   = 27, // camera offers extended binning modes
    CAMERA_FEATURES_ROTATE_180                         = 28, // image is rotated by 180°
    CAMERA_FEATURES_CAMMODE_NG                         = 29, // camera has a next-generation register mapping
    CAMERA_FEATURES_CAMMODE_CLASSIC                    = 30, // camera has a classic register mapping
    CAMERA_FEATURES_NEXT_FEATUREVECTOR                 = 31, // a subsequent feature vector is available
    // Extended feature vector
    CAMERA_FEATURES2_START                             = 32, // first extended camera feature
	CAMERA_FEATURES2_1_TAP			                   = 32, // camera supports a single tap sensor
	CAMERA_FEATURES2_2_TAP			                   = 33, // camera supports a dual tap sensor
	CAMERA_FEATURES2_3_TAP			                   = 34, // camera supports a triple tap sensor
	CAMERA_FEATURES2_4_TAP			                   = 35, // camera supports a quadruple tap sensor
	CAMERA_FEATURES2_REBOOT		                       = 36, // a remote reboot command is supported
	CAMERA_FEATURES2_IOMUX			                   = 37, // IO multiplexer functionality is available
    CAMERA_FEATURES2_SOFTWARE_TRIGGER_ID               = 38, // writing a software trigger ID into images is supported
	CAMERA_FEATURES2_KNEE_POINTS					   = 39, // knee points available
	CAMERA_FEATURES2_NOISEFILTER					   = 40, // noise filter available
	CAMERA_FEATURES2_TRIGGERDEBOUNCE  				   = 41, // trigger bounce can be activated
	CAMERA_FEATURES2_TEMPERATURE_SENSOR				   = 42, // temperature sensor available
	CAMERA_FEATURES2_IOMUX_PWM						   = 43, // PWM A and B signals are available in IO multiplexer
	CAMERA_FEATURES2_IOMUX_STROBE2					   = 44, // STROBE0 and STROBE1 signals are available in IO multiplexer
	CAMERA_FEATURES2_PIV							   = 45, // PIV Mode
	CAMERA_FEATURES2_IOMUX_EXPOSE					   = 46, // EXPOSE signal is available in IO multiplexer
	CAMERA_FEATURES2_IOMUX_READOUT					   = 47, // READOUT signal is available in IO multiplexer
	CAMERA_FEATURES2_FLATFIELDCORRECTION			   = 48, // FLATFIELDCORRECTION is available
	CAMERA_FEATURES2_SHADINGCORRECTION				   = 49, // SHADINGCORRECTION is available
	CAMERA_FEATURES2_DEFECTPIXEL					   = 50, // DEFECTPIXEL treatment is available
    CAMERA_FEATURES2_TRIGGERBOTHEDGES                  = 51, // TRIGGERBOTHEDGES is available
    CAMERA_FEATURES2_USERGAIN                          = 52, // USERGAIN is available
    CAMERA_FEATURES2_USEROFFSET                        = 53, // USEROFFSET is available
    CAMERA_FEATURES2_BINNING_X2                        = 54, // BINNING_X2 is availble
    CAMERA_FEATURES2_BINNING_X3                        = 55, // BINNING_X3 is available
    CAMERA_FEATURES2_BINNING_X4                        = 56, // BINNING_X4 is available
	CAMERA_FEATURES2_IOMUX_LOGIC					   = 57, // IOMUX_LOGIC is available
	CAMERA_FEATURES2_IOMUX_STROBE4                     = 58, // IOMUX_STROBE4 is available
	CAMERA_FEATURES2_LENSCONTROL       				   = 59, // LENSCONTROL is supported
	CAMERA_FEATURES2_1_TAP_1X_1Y			  		   = 60, // camera supports a single tap sensor
	CAMERA_FEATURES2_2_TAP_2XE_1Y			  		   = 61, // camera supports a dual tap left/right sensor
	CAMERA_FEATURES2_2_TAP_1X_2YE			  		   = 62, // camera supports a dual tap top/bottom sensor
	CAMERA_FEATURES2_4_TAP_2XE_2YE					   = 63, // camera supports a quad tap sensor
	// Extended feature vector
	CAMERA_FEATURES3_START               			   = 64, // second extended camera feature 
	CAMERA_FEATURES3_REVERSE_X	              		   = 64, // camera supports horizontal flipping
	CAMERA_FEATURES3_REVERSE_Y	                	   = 65, // camera supports vertical flipping
	CAMERA_FEATURES3_GLOBAL_SHUTTER                    = 66, // camera supports GLOBAL SHUTTER  Mode
	CAMERA_FEATURES3_ROLLING_SHUTTER                   = 67, // camera supports ROLLING SHUTTER Mode
	CAMERA_FEATURES3_MFT_FOCUS_UNIT                    = 68, // MFT focus unit can be changed,  
	} CAMERA_FEATURE;


  /** Look-up table mode.
   *  A camera can be instructed to apply a look-up table. Usually this will
   *  be used for running a gamma correction. However, other goals can also
   *  be implemented by a look-up table, like converting a gray-scale picture 
   *  into a binary black/white picture.
   */
  typedef enum
  {
    LUT_MODE_DISABLED               = 0,
    LUT_MODE_WHITE_BALANCE          = 1,  // 2006-12-20: deactivated, use  
										  // Camera_setWhiteBalance() instead
	LUT_MODE_ENABLED                = 2,

	} LUT_MODE;

	/** Binning mode.
   *  A camera can be set to one of the following pre-defined binning modes
   */
  typedef enum
  {
		BINNING_MODE_OFF			= 0,
		BINNING_MODE_HORIZONTAL		= 1,
		BINNING_MODE_VERTICAL		= 2,
		BINNING_MODE_2x2			= 3,
		//BINNING_MODE_3x3			= 4,
		BINNING_MODE_4x4			= 5,

  } BINNING_MODE;

  /** Lowpass filter.
   *  A lowpass filter can be activated/deactivated according to the
   *  following options.
   */
  typedef enum
  {
    LOWPASS_FILTER_NONE   = 0,
    LOWPASS_FILTER_3X3    = 1,

  } LOWPASS_FILTER;


  /** PIV mode
   *  A  camera can be set to enabled or disabled PIV mode
   */
  typedef enum
  {
    PIV_MODE_DISABLED  = 0,
    PIV_MODE_ENABLED   = 1,
  } PIV_MODE ;

  /** Acquisition mode.
   *  A camera can be set to one of the following acquisition modes
   */
  typedef enum
  {
    ACQUISITION_MODE_NO_ACQUISITION             = 0,  // 2013-07-08: deprecated
    ACQUISITION_MODE_FREE_RUNNING               = 1,  // 2008-05-06: deprecated, replaced by "fixed frequency"
    ACQUISITION_MODE_FIXED_FREQUENCY            = 1,  // 2011-08-29: deprecated, replaced by "programmable framerate"
    ACQUISITION_MODE_PROGRAMMABLE_FRAMERATE     = 1,
    ACQUISITION_MODE_INT_TRIGGER                = 2,  // 2008-05-06: deprecated, replaced by "software trigger"
    ACQUISITION_MODE_SOFTWARE_TRIGGER           = 2,
    ACQUISITION_MODE_EXT_TRIGGER_INT_EXPOSURE   = 3,
    ACQUISITION_MODE_EXT_TRIGGER_EXT_EXPOSURE   = 4,

  } ACQUISITION_MODE;

  /** Acquisition control
   *  A camera can be set to the following acquisition control modes
   */
  typedef enum
	{
    ACQUISITION_CONTROL_STOP    = 0,
    ACQUISITION_CONTROL_START   = 1,
    
  } ACQUISITION_CONTROL;

  /** Trigger polarity.
   *  A camera can be set to positive or negative trigger polarity
   */
  typedef enum
  {
	  TRIGGER_POLARITY_POSITIVE = 0,
	  TRIGGER_POLARITY_NEGATIVE = 1,

  } TRIGGER_POLARITY;

  /** Strobe polarity.
   *  A camera can be set to positive or negative strobe polarity
   */
  typedef enum
  {
    STROBE_POLARITY_POSITIVE = 0,
    STROBE_POLARITY_NEGATIVE = 1,

  } STROBE_POLARITY;

  /** Bayer conversion method
   */
  typedef enum
  {
    BAYER_METHOD_NONE         = -1,
    BAYER_METHOD_NEAREST      = 0,  // 2009-03-30: deprecated, mapped to BAYER_METHOD_SIMPLE
    BAYER_METHOD_SIMPLE       = 1,
    BAYER_METHOD_BILINEAR     = 2,  // 2009-03-30: deprecated, mapped to BAYER_METHOD_HQLINEAR
    BAYER_METHOD_HQLINEAR     = 3,
    BAYER_METHOD_EDGESENSE    = 4,  // 2009-03-30: deprecated, mapped to BAYER_METHOD_HQLINEAR
    BAYER_METHOD_GRAY         = 5,  // 2011-03-08: deprecated, user is sopposed to provide for own conversion

  } BAYER_METHOD;

  /** Pixel depth defines.
   *  The following pixel depths can be supported by a camera
   */
  typedef enum
  {
    SVU3V_PIXEL_DEPTH_8     =0,
	//SVU3V_PIXEL_DEPTH_10  =1, // not supported
    SVU3V_PIXEL_DEPTH_12    =2,
    SVU3V_PIXEL_DEPTH_16    =3,

  }SVU3V_PIXEL_DEPTH;

	/** Correction step.
	 *  While performing image correction, a sequence of
	 *  particular steps is needed as they are defined below .
	 */
	typedef enum
	{
		IMAGE_CORRECTION_IDLE				    = 0,
		IMAGE_CORRECTION_ACQUIRE_BLACK_IMAGE	= 1,
		IMAGE_CORRECTION_ACQUIRE_WHITE_IMAGE	= 2,
		IMAGE_CORRECTION_SAVE_TO_EEPROM		    = 3,

	} IMAGE_CORRECTION_STEP;

	/** Correction mode.
	 *  After a successful image correction run, one of
	 *  the following modes can be activated in order to
	 *  control what image correction method is used.
	 */
	typedef enum
	{
		IMAGE_CORRECTION_NONE		  = 0,
		IMAGE_CORRECTION_OFFSET_ONLY  = 1,
		IMAGE_CORRECTION_ENABLED      = 2,

	} IMAGE_CORRECTION_MODE;


	/** Tap selection defines.
	 *  Each tap of a 2-tap or 4-tap camera can be selected
	 *  by using one of the following defines.
	 */
	typedef enum
	{
		SVU3V_TAP_SELECT_TAP0		= 0,
		SVU3V_TAP_SELECT_TAP1		= 1,
		SVU3V_TAP_SELECT_TAP2		= 2,
		SVU3V_TAP_SELECT_TAP3		= 3,

	}SVU3V_TAP_SELECT;


	/** Tap configuration	selection defines.
	 *  Each configuration  of a 1-tap, 2-tap (horizontal and vertical) or 4-tap can be selected
	 *  by using one of the following defines.
	 */
	typedef enum
	{
		SVU3V_SELECT_SINGLE_TAP	= 0,
		SVU3V_SELECT_DUAL_TAP_H	= 1,
		SVU3V_SELECT_DUAL_TAP_V	= 2,
		SVU3V_SELECT_QUAD		 	= 3,

	}SVU3V_TAP_CONFIGURATION_SELECT;

	
	/** flipping mode selection defines.
	 *  the following modes of flipping are available
	 */
	typedef enum
	{
		SVU3V_REVERSE_OFF	= 0,
		SVU3V_REVERSE_X	= 1,
		SVU3V_REVERSE_Y	= 2,
		SVU3V_REVERSE_X_Y	= 3,
	} SVU3V_FLIPPING_MODE ;
	
	/** Shutter mode selection defines.
	 *  the following modes of Shutter are available
	 */
	typedef enum
	{
		SVU3V_GLOBAL_SHUTTER  = 0,
		SVU3V_ROLLING_SHUTTER = 1,
	}	SVU3V_SHUTTER_MODE ;
	
	
	/** Auto tap balance modes.
	 *  The following modes of auto tap balancing are available
	 */
	typedef enum
	{
		SVU3V_AUTO_TAP_BALANCE_MODE_OFF			=0,
		SVU3V_AUTO_TAP_BALANCE_MODE_ONCE			=1,
		SVU3V_AUTO_TAP_BALANCE_MODE_CONTINUOUS		=2,
		SVU3V_AUTO_TAP_BALANCE_MODE_RESET			=3,				

	}SVU3V_AUTO_TAP_BALANCE_MODE;

	/** Whitebalance Arts.
	 *  The following Art of Whitebalance are available
	 */
	typedef enum
	{
		Histogramm_Based_WB =0,
		Gray_Based_WB =1,

	}SVU3V_Whitebalance_SELECT;


	 /** The following maps for pixels correction are available
	 */
	typedef enum
	{
		Factory_Map =0,
		SVS_Map =1,
		Custom_Map =2,

	} PIXELS_CORRECTION_MAP_SELECT ;
	
	/** following focus units are available
    */
	typedef enum
		{
		 One_mm_Unit = 0,		//  focus  unit: 1 mm 
		 Dec_mm_Unit =1,	   //   focus  unit:  1/10 mm
		} FOCUS_UNIT;

  /** Pixel type defines.
   *  The pixel type as it is defined in the USB3 Vision specification
   */

  // Indicate that pixel is monochrome
  #define GVSP_PIX_MONO                       0x01000000
  #define GVSP_PIX_RGB                        0x02000000

  // Indicate effective number of bits occupied by the pixel (including padding).
  // This can be used to compute amount of memory required to store an image.
  #define GVSP_PIX_OCCUPY8BIT                 0x00080000
  #define GVSP_PIX_OCCUPY12BIT                0x000C0000
  #define GVSP_PIX_OCCUPY16BIT                0x00100000
  #define GVSP_PIX_OCCUPY24BIT                0x00180000

  // Bit masks
  #define GVSP_PIX_COLOR_MASK                 0xFF000000
  #define GVSP_PIX_EFFECTIVE_PIXELSIZE_MASK   0x00FF0000
  #define GVSP_PIX_ID_MASK                    0x0000FFFF

  // Bit shift value
  #define GVSP_PIX_EFFECTIVE_PIXELSIZE_SHIFT  16

  typedef enum
  {
    // Unknown pixel format
    GVSP_PIX_UNKNOWN          = 0x0000,

    // Mono buffer format defines
    GVSP_PIX_MONO8            = (GVSP_PIX_MONO | GVSP_PIX_OCCUPY8BIT  | 0x0001),
    GVSP_PIX_MONO10           = (GVSP_PIX_MONO | GVSP_PIX_OCCUPY16BIT | 0x0003),
    GVSP_PIX_MONO10_PACKED    = (GVSP_PIX_MONO | GVSP_PIX_OCCUPY12BIT | 0x0004),
    GVSP_PIX_MONO12           = (GVSP_PIX_MONO | GVSP_PIX_OCCUPY16BIT | 0x0005),
    GVSP_PIX_MONO12_PACKED    = (GVSP_PIX_MONO | GVSP_PIX_OCCUPY12BIT | 0x0006),
    GVSP_PIX_MONO16           = (GVSP_PIX_MONO | GVSP_PIX_OCCUPY16BIT | 0x0007),

    // Bayer buffer format defines
    GVSP_PIX_BAYGR8           = (GVSP_PIX_MONO | GVSP_PIX_OCCUPY8BIT  | 0x0008),
    GVSP_PIX_BAYRG8           = (GVSP_PIX_MONO | GVSP_PIX_OCCUPY8BIT  | 0x0009),
    GVSP_PIX_BAYGB8           = (GVSP_PIX_MONO | GVSP_PIX_OCCUPY8BIT  | 0x000A),
    GVSP_PIX_BAYBG8           = (GVSP_PIX_MONO | GVSP_PIX_OCCUPY8BIT  | 0x000B),
    GVSP_PIX_BAYGR10          = (GVSP_PIX_MONO | GVSP_PIX_OCCUPY16BIT | 0x000C),
    GVSP_PIX_BAYRG10          = (GVSP_PIX_MONO | GVSP_PIX_OCCUPY16BIT | 0x000D),
    GVSP_PIX_BAYGB10          = (GVSP_PIX_MONO | GVSP_PIX_OCCUPY16BIT | 0x000E),
    GVSP_PIX_BAYBG10          = (GVSP_PIX_MONO | GVSP_PIX_OCCUPY16BIT | 0x000F),
    GVSP_PIX_BAYGR12          = (GVSP_PIX_MONO | GVSP_PIX_OCCUPY16BIT | 0x0010),
    GVSP_PIX_BAYRG12          = (GVSP_PIX_MONO | GVSP_PIX_OCCUPY16BIT | 0x0011),
    GVSP_PIX_BAYGB12          = (GVSP_PIX_MONO | GVSP_PIX_OCCUPY16BIT | 0x0012),
    GVSP_PIX_BAYBG12          = (GVSP_PIX_MONO | GVSP_PIX_OCCUPY16BIT | 0x0013),

    // Color buffer format defines
    GVSP_PIX_RGB24            = (GVSP_PIX_RGB  | GVSP_PIX_OCCUPY24BIT),

    // Define for a gray image that was converted from a bayer coded image
    GVSP_PIX_GRAY8            = (GVSP_PIX_MONO | GVSP_PIX_OCCUPY8BIT),

  } GVSP_PIXEL_TYPE;

  /** Signal types
   *  Each image that is delivered to an application by a callback will have a related signal
   *  which informs about the circumstances why a callback was triggered.
   *  
   *  Usually a complete image will be delivered with the SVU3V_SIGNAL_FRAME_COMPLETED signal.
   */

   typedef enum
  {
  SVU3V_SIGNAL_NONE             						= 0,
  SVU3V_SIGNAL_FRAME_COMPLETED 						= 1,		// new image available, transfer was successful
  SVU3V_SIGNAL_FRAME_INCOMPLETE            = 2,		// a frame could not be properly completed
  SVU3V_SIGNAL_NO_FRAME_AVAILABLE   = 3,  // no image Available
  SVU3V_SIGNAL_CAMERA_CONNECTION_LOST = 4

  } SVU3V_SIGNAL_TYPE;
 

  /** IO multiplexer IN signals (signal sources).
   *  A camera provides for a flexible IO signal handling where one or
   *  multiple IN signals (signal sources) can be assigned to an OUT
   *  signal (signal sink). The following IN signals are defined.
   */
  typedef enum
	{
	SVU3V_IOMUX_IN0       = (1 <<  0),
	SVU3V_IOMUX_IN1       = (1 <<  1),
	SVU3V_IOMUX_IN2       = (1 <<  2),
	SVU3V_IOMUX_IN3       = (1 <<  3),
	SVU3V_IOMUX_IN4       = (1 <<  4),
	SVU3V_IOMUX_IN5       = (1 <<  5),
    SVU3V_IOMUX_IN6       = (1 <<  6),
    SVU3V_IOMUX_IN7       = (1 <<  7),
    SVU3V_IOMUX_IN8       = (1 <<  8),
    SVU3V_IOMUX_IN9       = (1 <<  9),
    SVU3V_IOMUX_IN10      = (1 << 10),
    SVU3V_IOMUX_IN11      = (1 << 11),
    SVU3V_IOMUX_IN12      = (1 << 12),
    SVU3V_IOMUX_IN13      = (1 << 13),
    SVU3V_IOMUX_IN14      = (1 << 14),
    SVU3V_IOMUX_IN15      = (1 << 15),
    SVU3V_IOMUX_IN16      = (1 << 16),
    SVU3V_IOMUX_IN17      = (1 << 17),
    SVU3V_IOMUX_IN18      = (1 << 18),
    SVU3V_IOMUX_IN19      = (1 << 19),
    SVU3V_IOMUX_IN20      = (1 << 20),
    SVU3V_IOMUX_IN21      = (1 << 21),
    SVU3V_IOMUX_IN22      = (1 << 22),
    SVU3V_IOMUX_IN23      = (1 << 23),
    SVU3V_IOMUX_IN24      = (1 << 24),
    SVU3V_IOMUX_IN25      = (1 << 25),
    SVU3V_IOMUX_IN26      = (1 << 26),
    SVU3V_IOMUX_IN27      = (1 << 27),
    SVU3V_IOMUX_IN28      = (1 << 28),
    SVU3V_IOMUX_IN29      = (1 << 29),
    SVU3V_IOMUX_IN30      = (1 << 30),
	SVU3V_IOMUX_IN31      = (1 << 31),
  } SVU3V_IOMux_IN;

  /** Some of the multiplexer's IN signals (signal sources) have a 
   *  pre-defined usage:
   */


	#define SVU3V_IOMux_IN_STROBE3					SVU3V_IOMUX_IN21
  // STROBE0 signal from the camera (same as STROBE)
	#define SVU3V_IOMux_IN_STROBE2					SVU3V_IOMUX_IN20
  // for logic trigger
	#define SVU3V_IOMUX_IN_MASK_LOGIC				SVU3V_IOMUX_IN19
  // pre-scaler for trigger purposes
	#define SVU3V_IOMUX_IN_MASK_PRESCALE			SVU3V_IOMUX_IN17
  // Reject short pulses (debounce)
	#define SVU3V_IOMUX_IN_MASK_DEBOUNCE			SVU3V_IOMUX_IN16
  // PWMA signal (pulse width modulator)
	#define SVU3V_IOMux_IN_PWMD					SVU3V_IOMUX_IN14
  // PWMA signal (pulse width modulator)
	#define SVU3V_IOMux_IN_PWMC				   	SVU3V_IOMUX_IN13
  // READOUT signal from camera
  #define SVU3V_IOMUX_IN_MASK_PULSE				SVU3V_IOMUX_IN12
  // READOUT signal from camera
	#define SVU3V_IOMux_IN_READOUT			  		SVU3V_IOMUX_IN11
  // EXPOSE signal from camera
  #define SVU3V_IOMux_IN_EXPOSE					SVU3V_IOMUX_IN10
  // PWMB signal (pulse width modulator)
	#define SVU3V_IOMux_IN_PWMB				  	SVU3V_IOMUX_IN9
  // PWMA signal (pulse width modulator)
	#define SVU3V_IOMux_IN_PWMA					SVU3V_IOMUX_IN8
  // STROBE1 and STROBE2 signal from the camera
	#define SVU3V_IOMux_IN_STROBE_0and1			(SVU3V_IOMUX_IN6 | SVU3V_IOMUX_IN7)
  // STROBE1 signal from the camera
	#define SVU3V_IOMux_IN_STROBE1					SVU3V_IOMUX_IN7
  // STROBE0 signal from the camera (same as STROBE)
	#define SVU3V_IOMux_IN_STROBE0					SVU3V_IOMUX_IN6
  // STROBE signal from the camera
	#define SVU3V_IOMux_IN_STROBE					SVU3V_IOMUX_IN6
  // Transmitter output from UART
	#define SVU3V_IOMux_IN_UART_OUT				SVU3V_IOMUX_IN5
  // Receiver IO line from camera connector
	#define SVU3V_IOMux_IN_IO_RXD					SVU3V_IOMUX_IN4
  // Fixed High signal (2010-09-16/EKantz: changed from 8 to 31)
	#define SVU3V_IOMux_IN_FIXED_HIGH				SVU3V_IOMUX_IN31
  // Fixed Low signal (2010-09-16/EKantz: changed from 7 to 30)
	#define SVU3V_IOMux_IN_FIXED_LOW				SVU3V_IOMUX_IN30

	/** Signal values for a particular signal
   */
  typedef enum
  {
    IO_SIGNAL_OFF  = 0,
    IO_SIGNAL_ON   = 1
  } IO_SIGNAL;

  /** IO multiplexer OUT signals (signal sinks).
   *  A camera provides for a flexible IO signal handling where one or
   *  multiple IN signals (signal sources) can be assigned to an OUT
   *  signal (signal sink). The following OUT signals are defined.
   */
  typedef enum
  {
    SVU3V_IOMUX_OUT0       =  0,
    SVU3V_IOMUX_OUT1       =  1,
    SVU3V_IOMUX_OUT2       =  2,
    SVU3V_IOMUX_OUT3       =  3,
    SVU3V_IOMUX_OUT4       =  4,
    SVU3V_IOMUX_OUT5       =  5,
    SVU3V_IOMUX_OUT6       =  6,
    SVU3V_IOMUX_OUT7       =  7,
    SVU3V_IOMUX_OUT8       =  8,
    SVU3V_IOMUX_OUT9       =  9,
    SVU3V_IOMUX_OUT10      = 10,
    SVU3V_IOMUX_OUT11      = 11,
    SVU3V_IOMUX_OUT12      = 12,
    SVU3V_IOMUX_OUT13      = 13,
    SVU3V_IOMUX_OUT14      = 14,
    SVU3V_IOMUX_OUT15      = 15,
    SVU3V_IOMUX_OUT16      = 16,
    SVU3V_IOMUX_OUT17      = 17,
    SVU3V_IOMUX_OUT18      = 18,
    SVU3V_IOMUX_OUT19      = 19,
    SVU3V_IOMUX_OUT20      = 20,
    SVU3V_IOMUX_OUT21      = 21,
    SVU3V_IOMUX_OUT22      = 22,
    SVU3V_IOMUX_OUT23      = 23,
    SVU3V_IOMUX_OUT24      = 24,
    SVU3V_IOMUX_OUT25      = 25,
    SVU3V_IOMUX_OUT26      = 26,
    SVU3V_IOMUX_OUT27      = 27,
    SVU3V_IOMUX_OUT28      = 28,
    SVU3V_IOMUX_OUT29      = 29,
    SVU3V_IOMUX_OUT30      = 30,
    SVU3V_IOMUX_OUT31      = 31,
  } SVU3V_IOMux_OUT;


  /** Some of the multiplexer's OUT signals (signal sinks) have a 
   *  pre-defined usage:
   */
  // Trigger signal to camera
  #define SVU3V_IOMux_OUT_TRIGGER    SVU3V_IOMUX_OUT6
  // Receiver input to UART
  #define SVU3V_IOMux_OUT_UART_IN    SVU3V_IOMUX_OUT5
  // Transmitter IO line from camera connector
  #define SVU3V_IOMux_OUT_IO_TXD     SVU3V_IOMUX_OUT4

  /** Baud rate for a camera's UART
   *  A camera's UART can be set to the following pre-defined baud rates.
   */
  typedef enum
  {
    SVU3V_BaudRate_110        =    110,
    SVU3V_BaudRate_300        =    300,
    SVU3V_BaudRate_1200       =   1200,
    SVU3V_BaudRate_2400       =   2400,
    SVU3V_BaudRate_4800       =   4800,
    SVU3V_BaudRate_9600       =   9600,
    SVU3V_BaudRate_19200      =  19200,
    SVU3V_BaudRate_38400      =  38400,
    SVU3V_BaudRate_57600      =  57600,
    SVU3V_BaudRate_115200     = 115200,
  } SVU3V_BaudRate;

  /** Camera buffer structure
	  *  Camera information will be stored and transferred to the application 
	  *  using the following structure
	  */
  typedef struct {

    char manufacturer[32];
    char model[32];  
    char specificInformation[48];
    char deviceVersion[32];
    char serialNumber[16];
    char userName[16];
  } SVU3V_CAMERA;

  /** Image buffer structure
	  *  An image data pointer along with accompanying data will be stored and
	  *  transferred to the application using the following structure
	  */
  typedef struct {
	unsigned char *ImageData;		// pointer to image data
	int ImageSize;					// image buffer size
	unsigned int ImageID;			// image ID assigned by camera
	int ImageCount;					// total number of images
	SVU3V_SIGNAL_TYPE SignalType;	// type of signal
    
  } SVU3V_IMAGE;

  /** Signal structure.
   * A camera signal conveys information about an event that took place
   * during image acquisition. In case of the end-of-transfer signal
   * an image pointer along with accompanying image data will be delivered
   * to the application contained in a data transport structure.
   */
  typedef struct {
	  SVU3V_SIGNAL_TYPE SignalType;
	  void *Data;
	  int DataLength;
  } SVU3V_SIGNAL;  

  /** Streaming channel callback function.
   *  The user-defined streaming channel callback function will be called each
   *  time when an image acquisition from camera has been finished and the image
   *  is ready for processing
   *
   *  NOTE: The callback function will return a NULL pointer in case an image
   *        could not be completely received  due to a timeout, 
   *  
   */
  typedef SVU3V_RETURN (__stdcall *StreamCallback)(Image_handle Image, void* Context);

  
  
#ifdef __USRDLL__
  #define __usrdllexport__  __declspec(dllexport)
#else
  #define __usrdllexport__
#endif


//------------------------------------------------------------------------------
// 0 - U3V DLL 
//------------------------------------------------------------------------------

  /** isVersionCompliant.
   *  The DLL's version at compile time will be checked against an expected
   *  version at runtime. The calling program knows the runtime version that
   *  it needs for proper functioning and can handle a version mismatch, e.g.
   *  by displaying the expected and the found DLL version to the user.
   *
   *  @param DllVersion a pointer to a version structure for the current DLL version
   *  @param ExpectedVersion a pointer to a version structure with the expected DLL version
   *  @return U3V_STATUS_SUCCESS or an appropriate SVU3V error code
   */
  __usrdllexport__ SVU3V_RETURN
  isVersionCompliantDLL(SVU3V_VERSION *DllVersion,
                        SVU3V_VERSION *ExpectedVersion);


	//------------------------internal-use----------------------------------------
  /** isLoadedU3VDLL
   *  A check is performed if the U3V DLL has already been loaded and if this
   *  is not the case it will be tried to load it.
   */
  bool isLoadedU3VDLL();

//------------------------------------------------------------------------------
// 1 - Camera: Discovery and bookkeeping
//------------------------------------------------------------------------------

  /** Create camera container.
   *  A handle is obtained that references a management object for discovering
   *  and selecting U3V cameras connected to the USB interfaces on a computer.
   *
   *  @return on success a handle for subsequent camera container function calls
   *         or -1 if creating a camera container failed
   */
  __usrdllexport__ CameraContainerClient_handle
  CameraContainer_create();

  /** Delete Camera container.
   *  A previously created camera container reference will be released. After
   *  deleting a camera container all camera specific functions are no longer
   *  available
   *
   *  @param hCameraContainer a handle received from CameraContainer_create()
   */
  __usrdllexport__ SVU3V_RETURN
  CameraContainer_delete(CameraContainerClient_handle hCameraContainer);

  /** Device discovery.
   *  All USB Devices that a computer is connected to will be searched for
   *  U3V cameras  
   *
   *  @param hCameraContainer a handle received from CameraContainer_create()
   *  @return success or error code
   */
  __usrdllexport__ SVU3V_RETURN
  CameraContainer_discovery(CameraContainerClient_handle hCameraContainer);

  /** Get number of connected cameras.
   *  The maximal index is returned that cameras can be accessed with in the
   *  camera container
   *
   *  @param hCameraContainer a handle received from CameraContainer_create()
   *  @return number of available cameras
   */
  __usrdllexport__ int
  CameraContainer_getNumberOfCameras(CameraContainerClient_handle hCameraContainer);

  /** Get camera.
   *  A camera handle is obtained accordingly to an index that was specified
   *  as an input parameter
   *
   *  @param hCameraContainer a handle received from CameraContainer_create()
   *  @param CameraIndex camera index from zero to one less the number of available cameras
   *  @return a handle for subsequent camera function calls or NULL in case the
   *         index was specified out of range
   */
  __usrdllexport__ Camera_handle
  CameraContainer_getCamera(CameraContainerClient_handle hCameraContainer,
                            int CameraIndex);

  /** Find camera.
   *  A camera handle is obtained accordingly to a search string that will be
   *  matched against the following items:
   *   - serial number
   *   - user defined name
   *   If one of those items can be matched a valid handle is returned.
   *   Otherwise a SVU3V_NO_CAMERA return value will be generated
   *
   *  @param hCameraContainer a handle received from CameraContainer_create()
   *  @param CameraItem a string for matching it against the above items
   *  @return a handle for subsequent camera function calls or NULL in case the
   *          index was specified out of range
   */
  __usrdllexport__ Camera_handle
  CameraContainer_findCamera(CameraContainerClient_handle hCameraContainer,
                             char *CameraItem);


//------------------------------------------------------------------------------
// 2 - Camera: Connection
//------------------------------------------------------------------------------

  /** Open connection to camera. 
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param Timeout the time without traffic or heartbeat after which a camera drops a connection (default: 3.0 sec.)
   *   NOTE: Values between 0.0 to 0.5 sec. will be mapped to the default value (3.0 sec.)
	 * HINT: It is recommended to use an extended timeout for debug sessions (e.g. 30 sec.). 
   *  @return success or error code
   */
  __usrdllexport__ SVU3V_RETURN 
  Camera_openConnection(Camera_handle hCamera, float Timeout);



  /** Disconnect camera.
   *  control channel will be closed
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @return success or error code
   */
  __usrdllexport__ SVU3V_RETURN
  Camera_closeConnection(Camera_handle hCamera);

 

//------------------------------------------------------------------------------
// 3 - Camera: Information
//------------------------------------------------------------------------------

  /** Get manufacturer name.
   *  The manufacturer name that is obtained from the camera firmware will be
   *  returned
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @return a string containing requested information
   */
  __usrdllexport__ char *
  Camera_getManufacturerName(Camera_handle hCamera);

  /** Get model name.
   *  The model name that is obtained from the camera firmware will be
   *  returned
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @return a string containing requested information
   */
  __usrdllexport__ char *
  Camera_getModelName(Camera_handle hCamera);

  /** Get device version.
   *  The device version that is obtained from the camera firmware will be
   *  returned
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @return a string containing requested information
   */
  __usrdllexport__ char *
  Camera_getDeviceVersion(Camera_handle hCamera);

  /** Get manufacturer specific information.
   *  The manufacturer specific information that is obtained from the camera
   *  firmware will be returned
   *
   *  @see CameraContainer_getCamera()fo
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @return a string containing requested information
   */
  __usrdllexport__ char *
  Camera_getManufacturerSpecificInformation(Camera_handle hCamera);

  /** Get serial number.
   *  The (manufacturer-assigned) serial number will be obtained from the camera
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @return a string containing requested information
   */
  __usrdllexport__ char *
  Camera_getSerialNumber(Camera_handle hCamera);

  /** Set user-defined name
   *  A user-defined name will be uploaded to a camera
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param UserDefinedName the name to be transferred to the camera
   *  @return success or error code
   */
  __usrdllexport__ SVU3V_RETURN
  Camera_setUserDefinedName(Camera_handle hCamera, char *UserDefinedName);

  /** Get user-defined name
   *  A name that has been assigned to a camera by the user will be returned
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @return a string containing requested information
   */
  __usrdllexport__ char *
  Camera_getUserDefinedName(Camera_handle hCamera);


  /** Get pixel clock.
   *  The camera's pixel clock will be returned
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param PixelClock a reference to the pixel clock value
   *  @return success or error code
   */
  __usrdllexport__ SVU3V_RETURN
  Camera_getPixelClock(Camera_handle hCamera,
                       int *PixelClock);

  /** Is camera feature.
   *  The camera will be queried whether a feature is available or not.
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param Feature a feature which availability will be determined
   *  @return a boolean value indicating whether the queried feature is available or not
   */
  __usrdllexport__ bool
  Camera_isCameraFeature(Camera_handle hCamera, CAMERA_FEATURE Feature);

  /** Read XML file.
   *  The camera's XML file will be retrieved and made available for further
   *  processing by an application. The returned pointer to the content of the
   *  XML file will be valid until the function is called again or until
   *  the camera is closed.
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param XML a pointer to a zero-terminated string containing the complete XML file
   *  @return success or error code
   */
  __usrdllexport__ SVU3V_RETURN
  Camera_readXMLFile(Camera_handle hCamera, char **XML);

 /**  Get Sensor temperature.
   * The current camera's Sensor temperature  will be returned.
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param SensorTemperatur  the actual sensor temperature of the camera [°C]
   *  @return success or error code
   */
  __usrdllexport__ SVU3V_RETURN
  Camera_getSensorTemperature(Camera_handle hCamera, unsigned int *SensorTemperatur);

//------------------------------------------------------------------------------
// 4 - Stream: Channel creation and control
//------------------------------------------------------------------------------
 

  /** Create streaming channel.
   *  A UDP streaming channel will be established for image data transfer.
   *  A connection to the camera has to be successfully opened first using
   *  Camera_openConnection() before a streaming channel can be established
   *
   *  @see Camera_openConnection()
   *  @param hStreamingChannel a handle for the streaming channel will be returned
   *  @param hCameraContainer a camera container handle received from CameraContainer_create()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param BufferCount specifies the number of image buffers (default: 3 buffers)
   *                    NOTE: A value of 0 will be mapped to the default value (3 buffers)
   *  @param CallbackFunction user-defined callback function for image processing
   *  @param Context user-defined data that will be returned on each callback
   *  @return success or error code
   */
  __usrdllexport__ SVU3V_RETURN
  StreamingChannel_create(StreamingChannel_handle *hStreamingChannel,
                          CameraContainerClient_handle hCameraContainer,
                          Camera_handle hCamera,
                          int BufferCount,
                          StreamCallback CallbackFunction,
                          void *Context);

  /** Delete streaming channel.
   *  A streaming channel will be closed and all resources will be released that
   *  have been occupied by the streaming channel
   *
   *  @param hStreamingChannel a streaming channel handle received from StreamingChannel_create()
   *  @return success or error code
   */
  __usrdllexport__ SVU3V_RETURN
  StreamingChannel_delete(StreamingChannel_handle hStreamingChannel);

//------------------------------------------------------------------------------
// 5 - Stream: Channel info 
//------------------------------------------------------------------------------

  /** Get pixel type.
	*  The pixel type will be returned that applies to the output image 
	*
	*  @param hStreamingChannel a streaming channel handle received from StreamingChannel_create()
	*  @param PixelType the programmed pixel type that has been set for the output image/view
	*  @return success or error code
   */
  __usrdllexport__ SVU3V_RETURN
  StreamingChannel_getPixelType(StreamingChannel_handle hStreamingChannel,
                                GVSP_PIXEL_TYPE *PixelType);


  /** Get image pitch.
   *  The image pitch will be returned that applies to the output image 
   *
   *  @param hStreamingChannel a streaming channel handle received from StreamingChannel_create()
   *  @param ImagePitch the image pitch for the output image/view
   *  @return success or error code
   */
  __usrdllexport__ SVU3V_RETURN
  StreamingChannel_getImagePitch(StreamingChannel_handle hStreamingChannel,
                                 int *ImagePitch);

  /** Get image size X.
   *  The image size X will be returned that applies to the output image 
   *
   *  @param hStreamingChannel a streaming channel handle received from StreamingChannel_create()
   *  @param ImageSizeX the image size X for the output image/view
   *  @return success or error code
   */
  __usrdllexport__ SVU3V_RETURN
  StreamingChannel_getImageSizeX(StreamingChannel_handle hStreamingChannel,
                                 int *ImageSizeX);

  /** Get image size Y.
   *  The image size Y will be returned that applies to the output image 
   *
   *  @param hStreamingChannel a streaming channel handle received from StreamingChannel_create()
   *  @param ImageSizeY the image size Y for the output image/view
   *  @return success or error code
   */
  __usrdllexport__ SVU3V_RETURN
  StreamingChannel_getImageSizeY(StreamingChannel_handle hStreamingChannel,
                                 int *ImageSizeY);

//------------------------------------------------------------------------------
// 6 - Stream: Image access
//------------------------------------------------------------------------------

  /** Get image pointer.
   *  A pointer to the image data will be returned
   *
   *  @param hImage an image handle that was received from the callback function
   *  @return a pointer to the image data
   */
  __usrdllexport__ unsigned char *
  Image_getDataPointer(Image_handle hImage);


  /** Get signal type (Not Yet implemented)
   *  The signal type that is related to a callback will be returned.
   *
   *  HINT: If the camera cable is disconnected while a connection to that camera
   *        is open, the image callback will return a NULL pointer. When the
   *        Image_getSignalType() function is called in that situation it will
   *        return a value: SVU3V_SIGNAL_CAMERA_CONNECTION_LOST. That signal
   *        value may be used for taking further actions.
   *
   *  @param hImage an image handle that was received from the callback function
   *  @return the signal type which informs why a callback was triggered
   */
	__usrdllexport__ SVU3V_SIGNAL_TYPE
  Image_getSignalType(Image_handle hImage);

  /** Get camera handle
   *  A handle of the camera that captured the image will be returned
   *
   *  @param hImage an image handle that was received from the callback function
   *  @return a camera handle
   */
  __usrdllexport__ Camera_handle
  Image_getCamera(Image_handle hImage);

  /** Release image.
   *  An image which availability was indicated by a StreamCallback function needs
   *  to be released after processing it by a user application in order to free
   *  the occupied buffer space for a subsequent image acquisition.
   *
   *  After releasing an image, none image access function must be called anymore 
   *  which would use the released image handle.
   *
   *  @param hImage an image handle that was received from the callback function
   *  @return success or error code
   */
  __usrdllexport__ SVU3V_RETURN
  Image_release(Image_handle hImage);

//------------------------------------------------------------------------------
// 7 - Stream: Image conversion
//------------------------------------------------------------------------------

  /** Get image RGB
   *
   *  The image will be converted by a selectable Bayer conversion algorithm into
   *  a RGB image. The caller needs to provide a sufficient buffer.
   *
   *  @param hImage an image handle that was received from the callback function
   *  @param BufferRGB a buffer for RGB image data
   *  @param BufferLength the length of the image buffer
   *  @param BayerMethod the a demosaicking method (Bayer method)
   *  @return success or error code
   */
  __usrdllexport__ SVU3V_RETURN
  Image_getImageRGB(Image_handle hImage,
                    unsigned char *BufferRGB,
                    int BufferLength,
                    BAYER_METHOD BayerMethod);
					
  /** Get image 12-bit as 8-bit
   *  A 12-bit image will be converted into a 8-bit image. The caller needs to 
   *  provide for a sufficient buffer for the 8-bit image.
   *
   *  @param hImage an image handle that was received from the callback function
   *  @param Buffer8bit a buffer for 8-bit image data
   *  @param BufferLength the length of the image buffer
   *  @return success or error code
   */
  __usrdllexport__ SVU3V_RETURN
	Image_getImage12bitAs8bit(Image_handle hImage,
                            unsigned char *Buffer8bit,
                            int BufferLength);

  /** Get image 12-bit as 16-bit
   *  A 12-bit image will be converted into a 16-bit image. The caller needs to 
   *  provide for a sufficiently large buffer (2 x width x height bytes) for the 
   *  16-bit image.
   *
   *  @param hImage an image handle that was received from the callback function
   *  @param Buffer16bit a buffer for 16-bit image data
   *  @param BufferLength the length of the image buffer
   *  @return success or error code
	 */
  __usrdllexport__ SVU3V_RETURN
	Image_getImage12bitAs16bit(Image_handle hImage,
                             unsigned char *Buffer16bit,
                             int BufferLength);

  /** Get image 16-bit as 8-bit
   *  A 16-bit image will be converted into a 8-bit image. The caller needs to 
   *  provide for a sufficient buffer for the 8-bit image.
   *
   *  @param hImage an image handle that was received from the callback function
   *  @param Buffer8bit a buffer for 8-bit image data
   *  @param BufferLength the length of the image buffer
   *  @return success or error code
	 */
  __usrdllexport__ SVU3V_RETURN
	Image_getImage16bitAs8bit(Image_handle hImage,
                            unsigned char *Buffer8bit,
                            int BufferLength);

//------------------------------------------------------------------------------
// 8 - Stream: Image characteristics
//------------------------------------------------------------------------------

  /** Get pixel type
   *  The pixel type as indicated by the camera will be returned
   *
   *  @param hImage an image handle that was received from the callback function
   *  @return the pixel type as indicated by the camera
   */
  __usrdllexport__ GVSP_PIXEL_TYPE
  Image_getPixelType(Image_handle hImage);

  /** Get image size.
   *  The number of bytes in the image data field will be returned.
   *
   *  @param hImage an image handle that was received from the callback function
   *  @return the number of bytes in the image data field
   */
  __usrdllexport__ int
  Image_getImageSize(Image_handle hImage);

  /** Get image pitch
   *  The number of bytes in a row (pitch) will be returned as received from the camera
   *
   *  @param hImage an image handle that was received from the callback function
   *  @return the image's pitch as received from the camera
   */
	__usrdllexport__ int
  Image_getPitch(Image_handle hImage);

  /** Get image size X
   *  The horizontal pixel number will be returned as received from the camera
   *
   *  @param hImage an image handle that was received from the callback function
   *  @return the image's size X as received from the camera
   */
  __usrdllexport__ int
  Image_getSizeX(Image_handle hImage);

  /** Get image size Y
   *  The vertical pixel number will be returned as received from the camera
   *
   *  @param hImage an image handle that was received from the callback function
   *  @return the image's size Y as received from the camera
   */
	__usrdllexport__ int
  Image_getSizeY(Image_handle hImage);

//------------------------------------------------------------------------------
// 9 - Stream: Image statistics
//------------------------------------------------------------------------------

  /** Get image identifier.
   *
   *  An image identifier will be returned as it was assigned by the camera.
   *  Usually the camera will assign an increasing sequence of integer numbers
   *  to subsequent images which will wrap at some point and jump back to 1.
   *  The 0 identifier will not be used as it is defined in the U3V Vision
   *  specification
   *
   *  @param hImage an image handle that was received from the callback function
   *  @return an integer number that is unique inside a certain sequence of numbers
   */
  __usrdllexport__ int
  Image_getImageID(Image_handle hImage);

  /** Get image timestamp
   *  The timestamp that was assigned to an image by the camera on image
   *  acquisition will be returned
   *
   *  @param hImage an image handle that was received from the callback function
   *  @return a timestamp as it was received from the camera in ticks elapsed after last camera reboot
   */
  __usrdllexport__ double
  Image_getTimestamp(Image_handle hImage);


  




//------------------------------------------------------------------------------
// 10 - Controlling camera: Frame rate
//------------------------------------------------------------------------------
  
  /** Set frame rate.
   *  The camera will be adjusted to a new frame rate
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param Framerate new frame rate in 1/s
   *  @return success or error code
   */
  __usrdllexport__ SVU3V_RETURN
  Camera_setFrameRate(Camera_handle hCamera,
                      float Framerate);

  /** Get frame rate.
   *  The currently programmed frame rate will be retrieved
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param Framerate currently programmed frame rate in 1/s
   *  @return success or error code
   */
  __usrdllexport__ SVU3V_RETURN
  Camera_getFrameRate(Camera_handle hCamera,
                      float *Framerate);

  /** Get frame rate min.
   *  The minimal available frame rate will be returned
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param MinFramerate the minimal frame rate in 1/s
   *  @return success or error code
   */
  __usrdllexport__ SVU3V_RETURN
  Camera_getFrameRateMin(Camera_handle hCamera,
                         float *MinFramerate);

  /** Get frame rate max.
   *  The maximal available frame rate will be returned
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param MaxFramerate the maximal frame rate in 1/s
   *  @return success or error code
   */
  __usrdllexport__ SVU3V_RETURN
  Camera_getFrameRateMax(Camera_handle hCamera,
                         float *MaxFramerate);

  /** Get frame rate range.
   *  The currently available frame rate range will be retrieved
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param MinFramerate currently available minimal frame rate in 1/s
   *  @param MaxFramerate currently available maximal frame rate in 1/s
   *  @return success or error code
   */
  __usrdllexport__ SVU3V_RETURN
  Camera_getFrameRateRange(Camera_handle hCamera,
                           float *MinFramerate,
                           float *MaxFramerate);

  /** Get frame rate increment.
   *  The frame rate increment will be returned
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param FramerateIncrement the frame rate increment in 1/s
   *  @return success or error code
   */
  __usrdllexport__ SVU3V_RETURN
  Camera_getFrameRateIncrement(Camera_handle hCamera,
                               float *FramerateIncrement);

//------------------------------------------------------------------------------
// 11 - Controlling camera: Exposure 
//------------------------------------------------------------------------------
  
  /** Set exposure time.
   *  The camera will be adjusted to a new exposure time
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param ExposureTime new exposure time in microseconds
   *  @return success or error code
   */
  __usrdllexport__ SVU3V_RETURN
  Camera_setExposureTime(Camera_handle hCamera,
                         float ExposureTime);

  /** Get exposure time.
   *  The currently programmed exposure time will be retrieved
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param ExposureTime the currently programmed exposure time in microseconds
   *  @return success or error code
   */
  __usrdllexport__ SVU3V_RETURN
  Camera_getExposureTime(Camera_handle hCamera,
                         float *ExposureTime);

  /** Get exposure time min.
   *  The minimal setting for exposure time will be returned.
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param MinExposureTime the minimal exposure time setting in microseconds
   *  @return success or error code
   */
  __usrdllexport__ SVU3V_RETURN
  Camera_getExposureTimeMin(Camera_handle hCamera,
                            float *MinExposureTime);

  /** Get exposure time max.
   *  The maximal setting for exposure will be returned.
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param MaxExposureTime the maximal exposure time setting in microseconds
   *  @return success or error code
   */
  __usrdllexport__ SVU3V_RETURN
  Camera_getExposureTimeMax(Camera_handle hCamera,
                            float *MaxExposureTime);


__usrdllexport__ SVU3V_RETURN 
Camera_getExposureTimeRange(Camera_handle hCamera,
                            float *MinExposureTime,
                            float *MaxExposureTime);


  /** Get exposure time increment.
   *  The increment for exposure time will be returned.
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param ExposureTimeIncrement the exposure time increment in microseconds
   *  @return success or error code
   */
  __usrdllexport__ SVU3V_RETURN
  Camera_getExposureTimeIncrement(Camera_handle hCamera,
                            float *ExposureTimeIncrement);

  /** Set exposure delay.
   *  The camera's exposure delay in micro seconds relative to the trigger
   *  pulse will be set to the provided value. The delay will become active
   *  each time an active edge of an internal or external trigger pulse arrives
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param ExposureDelay the new value for exposure delay in microseconds
   *  @return success or error code
   */
  __usrdllexport__ SVU3V_RETURN
  Camera_setExposureDelay(Camera_handle hCamera,
                           float ExposureDelay);

  /** Get exposure delay.
   *  The camera's current exposure delay will be returned in micro seconds
   *  relative to the trigger pulse
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param ExposureDelay the currently programmed value for exposure delay in microseconds
   *  @return success or error code
   */
  __usrdllexport__ SVU3V_RETURN
  Camera_getExposureDelay(Camera_handle hCamera,
                          float *ExposureDelay);

  /** Get maximal exposure delay.
   *  The camera's maximal exposure delay will be returned in micro seconds
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param MaxExposureDelay the maximal value for exposure delay in microseconds
   *  @return success or error code
   */
  __usrdllexport__ SVU3V_RETURN
  Camera_getExposureDelayMax(Camera_handle hCamera,
                             float *MaxExposureDelay);

  /** Get exposure delay increment.
   *  The camera's exposure delay increment will be returned in micro seconds
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param ExposureDelayIncrement the exposure delay increment in microseconds
   *  @return success or error code
   */
  __usrdllexport__ SVU3V_RETURN
  Camera_getExposureDelayIncrement(Camera_handle hCamera,
                                   float *ExposureDelayIncrement);

//------------------------------------------------------------------------------
// 12 - Controlling camera: Gain and offset
//------------------------------------------------------------------------------

  /** Set gain.
   *  The camera will be adjusted to a new analog gain
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param Gain new analogue gain (0..18 dB)
   *  @return success or error code
   */
  __usrdllexport__ SVU3V_RETURN
  Camera_setGain(Camera_handle hCamera,
                 float Gain);

  /** Get gain.
   *  The currently programmed analog gain will be retrieved
   *  Note: A gain of 1.0 is represented as integer 128 in the appropriate camera
   *  register. Thus the gain can be adjusted only in discrete steps.
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param Gain the currently programmed analog gain (0..18 dB)
   *  @return success or error code
   */
  __usrdllexport__ SVU3V_RETURN
  Camera_getGain(Camera_handle hCamera,
                 float *Gain);



    /** Get maximal gain.
   *  The maximal analog gain will be returned.
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param MaxGain the maximal analog gain
   *  @return success or error code
   */
  
    __usrdllexport__ SVU3V_RETURN
  Camera_getGainMax(Camera_handle hCamera,
                            float *MaxGain);
 
  __usrdllexport__ SVU3V_RETURN
  Camera_getGainMaxExtended(Camera_handle hCamera,
                            float *MaxGainExtended);

  /** Get gain increment.
   *  The analog gain increment will be returned.
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param GainIncrement the analog gain increment 
   *  @return success or error code
   */
  __usrdllexport__ SVU3V_RETURN
  Camera_getGainIncrement(Camera_handle hCamera,
                          float *GainIncrement);

  /** Set offset
   *  The ofset value will be set to the provided value
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param Offset the new value for pixel offset (0..255)
   *  @return success or error code
   */
  __usrdllexport__ SVU3V_RETURN
  Camera_setOffset(Camera_handle hCamera,
                   float Offset);

  /** Get offset
   *  The current offset value will be returned
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param Offset the currently programmed value for pixel offset (0..255)
   *  @return success or error code
   */
  __usrdllexport__ SVU3V_RETURN
  Camera_getOffset(Camera_handle hCamera,
                   float *Offset);

  /** Get maximal offset
   *  The maximal offset value will be returned
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param MaxOffset the maximal value for pixel offset
   *  @return success or error code
   */
  __usrdllexport__ SVU3V_RETURN
  Camera_getOffsetMax(Camera_handle hCamera,
                      float *MaxOffset);

//------------------------------------------------------------------------------
// 13 - Controlling camera: Auto gain/exposure
//------------------------------------------------------------------------------

 /** Set auto gain enabled
	*  The auto gain status will be switched on or off.
	*
	*  @param Camera a handle from a camera that has been opened before
	*  @param isAutoGainEnabled whether auto gain has to be enabled or disabled
	*  @return U3V_STATUS_SUCCESS or an appropriate SVU3V error code
  */
  __usrdllexport__ SVU3V_RETURN
	Camera_setAutoGainEnabled(Camera_handle Camera,
														bool isAutoGainEnabled);
			
 /** Get auto gain enabled
	*  Current auto gain status will be returned.
	*
	*  @param Camera a handle from a camera that has been opened before
	*  @param isAutoGainEnabled whether auto gain is enabled or disabled
	*  @return U3V_STATUS_SUCCESS or an appropriate SVU3V error code
  */
  __usrdllexport__ SVU3V_RETURN
	Camera_getAutoGainEnabled(Camera_handle Camera,
														bool *isAutoGainEnabled);
			
 /** Set auto gain brightness
	*  The target brightness (0..255) will be set which the camera tries to 
  *  reach automatically when auto gain/exposure is enabled. The range
  *  0..255 always applies independently from pixel depth.
	*
	*  @param Camera a handle from a camera that has been opened before
	*  @param Brightness the target brightness for auto gain enabled
	*  @return U3V_STATUS_SUCCESS or an appropriate SVU3V error code
	*/
  __usrdllexport__ SVU3V_RETURN
	Camera_setAutoGainBrightness(Camera_handle Camera,
                               float Brightness);
			
 /** Get auto gain brightness
	*  The target brightness (0..255) will be returned that the camera tries 
  *  to reach automatically when auto gain/exposure is enabled.
	*
	*  @param Camera a handle from a camera that has been opened before
	*  @param Brightness the programmed brightness value
	*  @return U3V_STATUS_SUCCESS or an appropriate SVU3V error code
	*/
  __usrdllexport__ SVU3V_RETURN
	Camera_getAutoGainBrightness(Camera_handle Camera,
                               float *Brightness);
			
 /** Set auto gain dynamics
	*  AutoGain PID regulator's time constants for the I (integration) and 
    *  D (differentiation) components will be set to new values.
	*
	*  @param Camera a handle from a camera that has been opened before
	*  @param AutoGainParameterI the I parameter in a PID regulation loop
	*  @param AutoGainParameterD the D parameter in a PID regulation loop
	*  @return U3V_STATUS_SUCCESS or an appropriate SVU3V error code
	*/
  __usrdllexport__ SVU3V_RETURN
	Camera_setAutoGainDynamics(Camera_handle Camera,
                             float AutoGainParameterI,
                             float AutoGainParameterD);

 /** Get auto gain dynamics
	*  AutoGain PID regulator's time constants for the I (integration) and 
    *  D (differentiation) components will be retrieved from the camera.
	*
	*  @param Camera a handle from a camera that has been opened before
	*  @param AutoGainParameterI the programmed I parameter in a PID regulation loop
	*  @param AutoGainParameterD the programmed D parameter in a PID regulation loop
	*  @return U3V_STATUS_SUCCESS or an appropriate SVU3V error code
	*/
  __usrdllexport__ SVU3V_RETURN
	Camera_getAutoGainDynamics(Camera_handle Camera,
                             float *AutoGainParameterI,
                             float *AutoGainParameterD);

 /** Set auto gain limits
	*  The minimal and maximal gain will be determined that the camera
	*  must not exceed in auto gain/exposure mode.
	*
	*  @param Camera a handle from a camera that has been opened before
	*  @param MinGain the minimal allowed gain value
	*  @param MaxGain the maximal allowed gain value
	*  @return U3V_STATUS_SUCCESS or an appropriate SVU3V error code
	*/
  __usrdllexport__ SVU3V_RETURN
	Camera_setAutoGainLimits(Camera_handle Camera,
                           float MinGain,
                           float MaxGain);
			
 /** Get auto gain limits
	*  The minimal and maximal gain will be returned that the camera
	*  must not exceed in auto gain/exposure mode.
	*
	*  @param Camera a handle from a camera that has been opened before
	*  @param MinGain a pointer to the programmed minimal allowed gain value
	*  @param MaxGain a pointer to the programmed maximal allowed gain value
	*  @return U3V_STATUS_SUCCESS or an appropriate SVU3V error code
	*/
  __usrdllexport__ SVU3V_RETURN
	Camera_getAutoGainLimits(Camera_handle Camera,
                           float *MinGain,
                           float *MaxGain);
			
 /** Set auto exposure limits
	*  The minimal and maximal exposure will be determined that the camera
	*  must not exceed in auto gain/exposure mode.
	*
	*  @param Camera a handle from a camera that has been opened before
	*  @param MinExposure the minimal allowed exposure value
	*  @param MaxExposure the maximal allowed exposure value
	*  @return U3V_STATUS_SUCCESS or an appropriate SVU3V error code
	*/
  __usrdllexport__ SVU3V_RETURN
	Camera_setAutoExposureLimits(Camera_handle Camera,
                               float MinExposure,
                               float MaxExposure);
			
 /** Set auto exposure limits
  *  The minimal and maximal exposure will be determined that the camera
  *  must not exceed in auto gain/exposure mode.
  *
  *  @param Camera a handle from a camera that has been opened before
  *  @param MinExposure the programmed minimal allowed exposure value
  *  @param MaxExposure the programmed maximal allowed exposure value
  *  @return U3V_STATUS_SUCCESS or an appropriate SVU3V error code
  */
  __usrdllexport__ SVU3V_RETURN
	Camera_getAutoExposureLimits(Camera_handle Camera,
                               float *MinExposure,
                               float *MaxExposure);

//------------------------------------------------------------------------------
// 14 - Controlling camera: Acquisition trigger
//------------------------------------------------------------------------------

  /** Set acquisition control.
   *  The camera's acquisition will be controlled (start/stop).
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param AcquisitionControl the new setting for acquisition control
   *  @return success or error code
   */
  __usrdllexport__ SVU3V_RETURN
  Camera_setAcquisitionControl(Camera_handle hCamera,
                               ACQUISITION_CONTROL AcquisitionControl);

  /** Get acquisition control.
   *  The camera's current acquisition control (start/stop) will be returned.
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param AcquisitionControl the currently programmed setting for acquisition control
   *  @return success or error code
   */
  __usrdllexport__ SVU3V_RETURN
  Camera_getAcquisitionControl(Camera_handle hCamera,
                               ACQUISITION_CONTROL *AcquisitionControl);

  /** Set acquisition mode and control.
   *  The camera's acquisition mode will be set to the selected value
   *  It can be determined whether camera control should switch to START immediately.
   *  First function always forces a START whereas second function allows to choose
   *  whether acquisition should start immediately or if previous control state
   *  will be preserved.
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param AcquisitionMode the new setting for acquisition mode
   *  @return success or error code
   */
  __usrdllexport__ SVU3V_RETURN
  Camera_setAcquisitionMode(Camera_handle hCamera,
                            ACQUISITION_MODE AcquisitionMode);


  /** Get acquisition mode.
   *  The camera's current acquisition mode will be returned
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param AcquisitionMode the currently programmed setting for acquisition mode
   *  @return success or error code
   */
  __usrdllexport__ SVU3V_RETURN
  Camera_getAcquisitionMode(Camera_handle hCamera,
                            ACQUISITION_MODE *AcquisitionMode);

  /** Software trigger. 
   *  The camera will be triggered for starting a new acquisition cycle.
   *  A mandatory pre-requisition for a successful software trigger is to have 
   *  the camera set to ACQUISITION_MODE_SOFTWARE_TIGGER before.
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @return success or error code
   */
  __usrdllexport__ SVU3V_RETURN
  Camera_softwareTrigger(Camera_handle hCamera);

 

  /** Set trigger polarity
   *  The camera's trigger polarity will be set to the selected value
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param TriggerPolarity the new setting for trigger polarity
   *  @return success or error code
   */
  __usrdllexport__ SVU3V_RETURN
  Camera_setTriggerPolarity(Camera_handle hCamera,
                            TRIGGER_POLARITY TriggerPolarity);

  /** Get trigger polarity
   *  The camera's current trigger polarity will be returned
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param TriggerPolarity the currently programmed setting for trigger polarity
   *  @return success or error code
   */
  __usrdllexport__ SVU3V_RETURN
  Camera_getTriggerPolarity(Camera_handle hCamera,
                            TRIGGER_POLARITY *TriggerPolarity);

   /** Set a new PIV mode
    *  The camera's PIV mode will be enabled or disabled.
    *
    *  @see CameraContainer_getCamera()
    *  @param hCamera a camera handle received from CameraContainer_getCamera()
    *  @param PivMode the new setting for PIV mode
    *  @return success or error code
    */
  __usrdllexport__ SVU3V_RETURN
  Camera_setPivMode(Camera_handle hCamera,
                    PIV_MODE  PivMode);

   /** Get PIV Mode 
    *  Check if camera's PIV mode is enabled or disabled.
    *  The state of camera's current PivMode will be returned
    *
    *  @see CameraContainer_getCamera()
    *  @param hCamera a camera handle received from CameraContainer_getCamera()
    *  @param PivMode the currently programmed setting for PIV mode
    *  @return success or error code
    */
  __usrdllexport__ SVU3V_RETURN
  Camera_getPivMode(Camera_handle hCamera,
                    PIV_MODE *PivMode);

 //---------------- Debouncer----------------------

  /** Set debouncer  duration
   *   The camera's Debouncer duration will be set to the selected value
   *
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param DebouncerDuration the new setting for debouncer Duration in ticks
   *  one tick corresponds to (1/66666666) sec
   *  @return success or error code
   */
  __usrdllexport__ SVU3V_RETURN
  Camera_setDebouncerDuration(Camera_handle hCamera,
                              float  DebouncerDuration); 

 /** Get debouncer  duration
  *  The camera's debouncer  duration will be returned
  *
  *  @see CameraContainer_getCamera()
  *  @param hCamera a camera handle received from CameraContainer_getCamera()
  *  @param DebouncerDuration the currently programmed debouncer duration in ticks
  *	 one tick corresponds to (1/66666666) sec
  *  @return success or error code
  */
  __usrdllexport__ SVU3V_RETURN
  Camera_getDebouncerDuration(Camera_handle hCamera,
                              float *DebouncerDuration);

  //-----------------prescaler-----------------------
  
  /** Set prescaler devisor
   *   The camera's prescaler Devisor will be set to the selected value
   *
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param PrescalerDevisor the new setting for PrescalerDevisor
   *  @return success or error code
   */
   __usrdllexport__ SVU3V_RETURN
   Camera_setPrescalerDevisor(Camera_handle hCamera,
                             unsigned int PrescalerDevisor); 

 /** Get prescaler devisor
  *  The camera's prescaler devisor will be returned
  *
  *  @see CameraContainer_getCamera()
  *  @param hCamera a camera handle received from CameraContainer_getCamera()
  *  @param PrescalerDevisor the currently programmed setting for prescaler devisor. 
  *  @return success or error code
  */
  __usrdllexport__ SVU3V_RETURN
  Camera_getPrescalerDevisor(Camera_handle hCamera,
                             unsigned int *PrescalerDevisor);

  //-----------------Sequencer-----------------------

  /** load Sequence parameters 
   *  New sequence parameters will be loaded from a XML file into the camera  
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param Filename a complete path and file name where to load the settings from
   *  @return success or error code
   */
  __usrdllexport__ SVU3V_RETURN
  Camera_loadSequenceParameters(Camera_handle hCamera,
                                char *Filename);

  /** Start Sequencer
   * Start acquisition using sequencer.
   * This will occur after loading the appropriate sequence parameters. 
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @return success or error code
   */
  __usrdllexport__ SVU3V_RETURN
  Camera_startSequencer(Camera_handle hCamera);

//------------------------------------------------------------------------------
// 15 - Controlling camera: Strobe
//------------------------------------------------------------------------------
  
    /** Set strobe polarity extended
   *  The camera's strobe polarity will be will be set to the selected value
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param StrobePolarity the currently programmed setting for strobe polarity
   *  @param StrobIndex the index of the current strobe channel, it can take a value 1,2,3 or 4. 
   *  @return success or error code
   */
  

  __usrdllexport__ SVU3V_RETURN
   Camera_setStrobePolarityExtended(Camera_handle hCamera,
                                    STROBE_POLARITY StrobePolarity, int StrobIndex);

  /** Get strobe polarity extended
   *  The camera's current strobe polarity will be returned
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param StrobePolarity the currently programmed setting for strobe polarity
   *  @param StrobIndex the index of the current strobe channel, it can take a value 1,2,3 or 4. 
   *  @return success or error code
   *  @return success or error code
   */

  __usrdllexport__ SVU3V_RETURN
  Camera_getStrobePolarityExtended(Camera_handle hCamera,
                                   STROBE_POLARITY *StrobePolarity, int StrobIndex);



  /** Set strobe position extended
   *  The camera's strobe position in micro seconds relative to the trigger
   *  pulse will be set to the selected value
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param StrobePosition the new value for strobe position in microseconds
   *  @param StrobIndex the index of the current strobe channel, it can take a value 1,2,3 or 4. 
   *  @return success or error code
   */
  __usrdllexport__ SVU3V_RETURN
  Camera_setStrobePositionExtended(Camera_handle hCamera,
                                   float StrobePosition, 
                                   int StrobIndex);



  /** Get strobe position extended
   *  The camera's current strobe position will be returned in micro seconds
   *  relative to the trigger pulse
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param StrobePosition the currently programmed value for strobe position in microseconds
   *  @param StrobIndex the index of the current strobe channel it can take a value 1,2,3 or 4. 
   *  @return success or error code
   */
  __usrdllexport__ SVU3V_RETURN
  Camera_getStrobePositionExtended(Camera_handle hCamera,
                                   float *StrobePosition, 
                                   int StrobIndex);

  /** Get maximal strobe position 
   *  The camera's maximal strobe position (delay) will be returned in micro seconds
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param MaxStrobePosition the maximal value for strobe position in microseconds
   *  @return success or error code
   */
  __usrdllexport__ SVU3V_RETURN
  Camera_getStrobePositionMax(Camera_handle hCamera,
                           float *MaxStrobePosition);

  /** Get strobe position increment
   *  The camera's strobe position increment will be returned in micro seconds
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param StrobePositionIncrement the strobe position increment in microseconds
   *  @return success or error code
   */
  __usrdllexport__ SVU3V_RETURN
  Camera_getStrobePositionIncrement(Camera_handle hCamera,
                                    float *StrobePositionIncrement);



  /** Set strobe duration extended
   *  The camera's strobe duration in micro seconds will be set to the selected
   *  value
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param StrobeDuration the new value for strobe duration in microseconds
   *  @param StrobIndex the index of the current strobe channel it can take a value 1,2,3 or 4. 
   *  @return success or error code
   */
  __usrdllexport__ SVU3V_RETURN
  Camera_setStrobeDurationExtended(Camera_handle hCamera,
                                   float StrobeDuration, 
                                   int StrobIndex);



  /** Get strobe duration extended
   *  The camera's current strobe duration in micro seconds will be returned
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param StrobeDuration the currently programmed value for strobe duration in microseconds
   *  @param StrobIndex the index of the current strobe channel it can take a value 1,2,3 or 4. 
   *  @return success or error code
   */
  __usrdllexport__ SVU3V_RETURN
  Camera_getStrobeDurationExtended(Camera_handle hCamera,
                                   float *StrobeDuration,
                                   int StrobIndex);

  /** Get maximal strobe duration
   *  The camera's maximal strobe duration in micro seconds will be returned
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param MaxStrobeDuration the maximal value for strobe duration in microseconds
   *  @return success or error code
   */
  __usrdllexport__ SVU3V_RETURN
  Camera_getStrobeDurationMax(Camera_handle hCamera,
                              float *MaxStrobeDuration);

  /** Get strobe duration increment
   *  The camera's strobe duration increment in micro seconds will be returned
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param StrobeDurationIncrement the strobe duration increment in microseconds
   *  @return success or error code
   */
  __usrdllexport__ SVU3V_RETURN
  Camera_getStrobeDurationIncrement(Camera_handle hCamera,
                                    float *StrobeDurationIncrement);

//------------------------------------------------------------------------------
// 16 - Controlling camera: Tap balance
//------------------------------------------------------------------------------
 

	/**
	 *	Set tap Configuration extended
	 *  The camera will be controlled for working with one of the following tap configurations: 
	 *  SVU3V_SELECT_SINGLE_TAP 
	 *	SVU3V_SELECT_DUAL_TAP_H	
	 *	SVU3V_SELECT_DUAL_TAP_V	
	 *	SVU3V_SELECT_QUAD		 	
	 *  
	 *  @see CameraContainer_getCamera()
     *  @param hCamera a camera handle received from CameraContainer_getCamera()
	 *  @param SelectedTapConfig the selected tap configuration to be used by the camera 
     *  @return success or error code
	 */
  __usrdllexport__ SVU3V_RETURN
  Camera_setTapConfigurationEx(Camera_handle hCamera,
                             SVU3V_TAP_CONFIGURATION_SELECT SelectedTapConfig);

	/**
	 *  getTapConfiguration
	 *  The camera will be queried whether it is working with one of the following configurations:
     *  SVU3V_SELECT_SINGLE_TAP  
	 *	SVU3V_SELECT_DUAL_TAP_H	
	 *	SVU3V_SELECT_DUAL_TAP_V	
	 *	SVU3V_SELECT_QUAD	
	 *
     *  @see CameraContainer_getCamera()
     *  @param hCamera a camera handle received from CameraContainer_getCamera()
	 *  @param TapConfig the tap configuration currently used by the camera is returned 
     *  @return success or error code
	 */
  __usrdllexport__ SVU3V_RETURN
  Camera_getTapConfigurationEx(Camera_handle hCamera,
                             SVU3V_TAP_CONFIGURATION_SELECT *TapConfig);


  /** Set auto tap balance mode
   *  One of the modes (OFF,ONCE,CONTINUOUS,RESET) will be applied for an auto
   *  tap balance procedure.
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param AutoTapBalanceMode the mode for auto tap balancing to be activated
   *  @return success or error code
   */
  __usrdllexport__ SVU3V_RETURN
  Camera_setAutoTapBalanceMode(Camera_handle hCamera,
                               SVU3V_AUTO_TAP_BALANCE_MODE AutoTapBalanceMode);

  /** Get auto tap balance mode
   *  Currently adjusted auto tap balance mode will be returned.
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param AutoTapBalanceMode a pointer to a value for auto tap balancing
   *  @return success or error code
   */
  __usrdllexport__ SVU3V_RETURN
  Camera_getAutoTapBalanceMode(Camera_handle hCamera,
                               SVU3V_AUTO_TAP_BALANCE_MODE *AutoTapBalanceMode);

 

  /** Set tap gain
   *  A provided tap gain in [dB] will be transferred to camera.
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   
   *  @param TapGain  one of the defined tap selectors 
   *  @param TapSelect the new value for tap gain 
   *  @return success or error code
   */
  __usrdllexport__ SVU3V_RETURN
  Camera_setTapGain(Camera_handle hCamera,
										float TapGain, 
                    SVU3V_TAP_SELECT TapSelect);

  /** Get tap gain
   *  Currently adjusted tap gain in [dB] will be queried from camera.
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param TapSelect one of the defined tap selectors 
   *  @param TapGain the new value for tap gain 
   *  @return success or error code
   */
  __usrdllexport__ SVU3V_RETURN
  Camera_getTapGain(Camera_handle hCamera,
										float *TapGain, 
                    SVU3V_TAP_SELECT TapSelect);

//------------------------------------------------------------------------------
// 17 - Controlling camera: Image parameter
//------------------------------------------------------------------------------

  /** Get imager width.
   *  The imager width will be retrieved from the camera
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param ImagerWidth a reference to the imager width value
   *  @return success or error code
   */
  __usrdllexport__ SVU3V_RETURN
  Camera_getImagerWidth(Camera_handle hCamera,
                        int *ImagerWidth);

  /** Get imager height.
   *  The imager height will be retrieved from the camera
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param ImagerHeight a reference to the imager height value
   *  @return success or error code
   */
  __usrdllexport__ SVU3V_RETURN
  Camera_getImagerHeight(Camera_handle hCamera,
                         int *ImagerHeight);

  /** Get image size.
   *  The number of bytes in an image will be returned
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param ImageSize a reference to the image size value
   *  @return success or error code
   */
  __usrdllexport__ SVU3V_RETURN
  Camera_getImageSize(Camera_handle hCamera,
                      int *ImageSize);

  /** Get pitch.
   *  The number of bytes in a row will be returned
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param Pitch a reference to the pitch value
   *  @return success or error code
   */
  __usrdllexport__ SVU3V_RETURN
  Camera_getPitch(Camera_handle hCamera,
                  int *Pitch);

  /** Get size X.
   *  The currently used horizontal picture size X will be retrieved from the camera
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param SizeX a reference to the size X value
   *  @return success or error code
   */
  __usrdllexport__ SVU3V_RETURN
  Camera_getSizeX(Camera_handle hCamera,
                  int *SizeX);

  /** Get size Y.
   *  The currently used vertical picture size Y will be retrieved from the camera
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param SizeY a reference to the size Y value
   *  @return success or error code
   */
  __usrdllexport__ SVU3V_RETURN
  Camera_getSizeY(Camera_handle hCamera,
                  int *SizeY);

  /** Set binning mode.
   *  The camera's binning mode will be set to the selected value
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param BinningMode the new setting for binning mode
   *  @return success or error code
   */
  __usrdllexport__ SVU3V_RETURN
  Camera_setBinningMode(Camera_handle hCamera,
                        BINNING_MODE BinningMode);

  /** Get binning mode.
   *  The camera's current binning mode will be returned
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param BinningMode the currently programmed setting for binning mode
   *  @return success or error code
   */
  __usrdllexport__ SVU3V_RETURN
  Camera_getBinningMode(Camera_handle hCamera,
                        BINNING_MODE *BinningMode);

  /** Set area of interest (AOI)
   *  The camera will be switched to partial scan mode and an AOI will be set
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param SizeX the number of pixels in one row
   *  @param SizeY the number of scan lines
   *  @param OffsetX a left side offset of the scanned area
   *  @param OffsetY an upper side offset of the scanned area
   *  @return success or error code
   */
  __usrdllexport__ SVU3V_RETURN 
  Camera_setAreaOfInterest(Camera_handle hCamera,
                           int SizeX,
                           int SizeY,
                           int OffsetX,
                           int OffsetY);

  /** Get area of interest(AOI)
   *  The currently set parameters for partial scan will be returned
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param SizeX the programmed number of pixels in one row
   *  @param SizeY the programmed number of scan lines
   *  @param OffsetX a programmed left side offset of the scanned area
   *  @param OffsetY an programmed upper side offset of the scanned area
   *  @return success or error code
   */
  __usrdllexport__ SVU3V_RETURN
  Camera_getAreaOfInterest(Camera_handle hCamera,
                           int *SizeX,
                           int *SizeY,
                           int *OffsetX,
                           int *OffsetY);

  /** Get minimal/maximal area of interest(AOI).
   *  The boundaries for partial scan will be returned
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param MinSizeX the minimal AOI width
   *  @param MinSizeY the minimal AOI height
   *  @param MaxSizeX the maximal AOI width
   *  @param MaxSizeY the maximal AOI height
   *  @return success or error code
   */
  __usrdllexport__ SVU3V_RETURN
  Camera_getAreaOfInterestRange(Camera_handle hCamera,
                                int *MinSizeX,
                                int *MinSizeY,
                                int *MaxSizeX,
                                int *MaxSizeY);

  /** Get area of interest(AOI) increment
   *  The increment for partial scan parameters will be returned
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param SizeXIncrement the increment for AOI width
   *  @param SizeYIncrement the increment for AOI height
   *  @param OffsetXIncrement the increment for AOI width offset
   *  @param OffsetYIncrement the increment for AOI height offset
   *  @return success or error code
   */
  __usrdllexport__ SVU3V_RETURN
  Camera_getAreaOfInterestIncrement(Camera_handle hCamera,
                                    int *SizeXIncrement,
                                    int *SizeYIncrement,
                                    int *OffsetXIncrement,
                                    int *OffsetYIncrement);

 


//-----------------------------------------------------------------------------------------------------

  /** Set Flipping mode.
   *   The camera will be controlled for working with the following flipping mode if selected:
   *   REVERSE_OFF (without flipping)
   *   REVERSE_X (vertical flipping)
   *   REVERSE_Y (horizontal flipping)
   *   REVERSE_X_Y( horizontal and vertical flipping) 
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param FlippingMode the new setting for flipping mode
   *  @return success or error code
   */
 __usrdllexport__ SVU3V_RETURN
  Camera_setFlippingMode(Camera_handle hCamera,
                        SVU3V_FLIPPING_MODE  FlippingMode);

  /** Get Flipping mode.
   *  The camera will be queried whether it is working with one of the following flipping mode:
   *   REVERSE_OFF (without flipping)
   *   REVERSE_X (vertical flipping)
   *   REVERSE_Y (horizontal flipping)
   *   REVERSE_X_Y( horizontal and vertical flipping)
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param FlippingMode  the currently programmed setting for flipping mode
   *  @return success or error code
   */
  __usrdllexport__ SVU3V_RETURN
  Camera_getFlippingMode(Camera_handle hCamera,
                        SVU3V_FLIPPING_MODE *FlippingMode);
  

//-----------------------------------------------------------------------------------------------------


  /** Set Shutter mode.
   *  The camera will be controlled for working with the following shutter mode if selected:
   *  GLOBAL_SHUTTER 
   *  ROLLING_SHUTTER
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param ShutterMode the new setting for shutter mode
   *  @return success or error code
   */
 __usrdllexport__ SVU3V_RETURN
  Camera_setShutterMode(Camera_handle hCamera,
                        SVU3V_SHUTTER_MODE  ShutterMode);

  /** Get Shutter mode.
   *  The camera will be queried whether it is working with one of the following shutter mode:
   *  GLOBAL_SHUTTER 
   *  ROLLING_SHUTTER
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param ShutterMode  the currently programmed setting for shutter mode
   *  @return success or error code
   */
  __usrdllexport__ SVU3V_RETURN
  Camera_getShutterMode(Camera_handle hCamera,
                        SVU3V_SHUTTER_MODE *ShutterMode);


//------------------------------------------------------------------------------
// 18 - Controlling camera: Image appearance
//------------------------------------------------------------------------------
 
  /** Get pixel type.
   *  The pixel type will be retrieved from the camera
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param PixelType a reference to the pixel type value
   *  @return success or error code
   */
  __usrdllexport__ SVU3V_RETURN
  Camera_getPixelType(Camera_handle hCamera,
                      GVSP_PIXEL_TYPE *PixelType);

  /** Set pixel depth.
   *  The number of bits for a pixel will be set to 8, 12 or 16 bits. Before this function
   *  is called the camera's feature vector should be queried whether the desired pixel depth
   *  is supported
   *
   *  @see Camera_isCameraFeature()
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param PixelDepth the intended value for pixel depth
   *  @return success or error code
   */
  __usrdllexport__ SVU3V_RETURN
  Camera_setPixelDepth(Camera_handle hCamera,
                       SVU3V_PIXEL_DEPTH PixelDepth);

  /** Get pixel depth.
   *  The camera's current setting for pixel depth will be queried.
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param PixelDepth an enum for the number of bits in a pixel will be returned
   *  @return success or error code
   */
  __usrdllexport__ SVU3V_RETURN
  Camera_getPixelDepth(Camera_handle hCamera,
                       SVU3V_PIXEL_DEPTH *PixelDepth);

  /** setWhiteBalance.
   *  The provided values will be applied for white balance. Color strengths can be
	 *  given with any reference value, e.g. 100 or 1.0 or others. Thus RGB=100/110/130
	 *  is the same as RGB=1.0/1.1/1.3
   *
   *  NOTE: The color component strength for Red, Green and Blue can either be
   *        provided by user or they can conveniently be calculated inside an image
   *        callback using the Image_estimateWhiteBalance() function.
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param Red balanced value for red color
   *  @param Green balanced value for green color
   *  @param Blue balanced value for blue color
   *  @return success or error code
   */
  __usrdllexport__ SVU3V_RETURN
  Camera_setWhiteBalance(Camera_handle hCamera,
                         float Red,
                         float Green ,
                         float Blue);

  /** getWhiteBalance.
   *  Currently set values for white balance will be returned.
   *  Previously adjusted values will be returned either unchanged or adjusted
   *  if necessary. The returned values will be 100 and above where the color
   *  which got 100 assigned will be transferred unchanged, however two
   *  other color components might be enhanced above 100 for each image.
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param Red balanced value for red color
   *  @param Green balanced value for green color
   *  @param Blue balanced value for blue color
   *  @return success or error code
   */
  __usrdllexport__ SVU3V_RETURN
  Camera_getWhiteBalance(Camera_handle hCamera,
                         float *Red,
                         float *Green ,
                         float *Blue);

  /** getWhiteBalanceMax.
   *  The maximal white-balance value for amplifying colors will be returned.
   *  A value of 1.0 is the reference for a balanced situation.
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param WhiteBalanceMax the maximal white-balance (e.g. 4.0 or 2.0)
   *  @return success or error code
   */
  __usrdllexport__ SVU3V_RETURN
  Camera_getWhiteBalanceMax(Camera_handle hCamera,
                            float *WhiteBalanceMax);

  /** setGammaCorrection.
   *  A lookup table will be generated based on given gamma correction.
   *  Subsequently the lookup table will be uploaded to the camera. 
   *  A gamma correction is supported in a range 0.4 - 2.5
   *
   *  @param Camera a handle from a camera that has been opened before
   *  @param GammaCorrection a gamma correction factor
   *  @return U3V_STATUS_SUCCESS or an appropriate SVU3V error code
   */
  __usrdllexport__ SVU3V_RETURN
	Camera_setGammaCorrection(Camera_handle Camera,
												    float GammaCorrection);

  /** setGammaCorrectionExt.
   *  A lookup table will be generated based on given gamma correction.
   *  Additionally, a digital gain and offset will be taken into account.
   *  Subsequently the lookup table will be uploaded to the camera. 
   *  A gamma correction is supported in a range 0.4 - 2.5
   *
   *  @param Camera a handle from a camera that has been opened before
   *  @param GammaCorrection a gamma correction factor
   *  @param DigitalGain a digital gain used 
   *  @param DigitalOffset a digital offset used
   *  @return U3V_STATUS_SUCCESS or an appropriate SVU3V error code
   */
  __usrdllexport__ SVU3V_RETURN
	Camera_setGammaCorrectionExt(Camera_handle Camera,
												       float GammaCorrection,
                               float DigitalGain,
                               float DigitalOffset);

  /** setLowpassFilter.
    *  A filter will be enabled/disabled which smoothes an image inside
    *  a camera accordingly to a given algorithm, e.g. 3x3.
    *
    *  @param Camera a handle from a camera that has been opened before
    *  @param LowpassFilter a control value for activating/deactivating smoothing
    *  @return U3V_STATUS_SUCCESS or an appropriate SVU3V error code
    */
  __usrdllexport__ SVU3V_RETURN
  Camera_setLowpassFilter(Camera_handle Camera,
                          LOWPASS_FILTER LowpassFilter);

  /** getLowpassFilter.
    *  Current mode of a low pass filter will be retrieved from camera.
    *
    *  @param Camera a handle from a camera that has been opened before
    *  @param LowpassFilter the currently programmed low pass filter will be returned
    *  @return U3V_STATUS_SUCCESS or an appropriate SVU3V error code
    */
  __usrdllexport__ SVU3V_RETURN
  Camera_getLowpassFilter(Camera_handle Camera,
                          LOWPASS_FILTER *LowpassFilter);

  /** setLookupTableMode.
   *  The look-up table mode will be switched on or off
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param LUTMode new setting for look-up table mode
   *  @return success or error code
   */
  __usrdllexport__ SVU3V_RETURN
  Camera_setLookupTableMode(Camera_handle hCamera,
                            LUT_MODE LUTMode);

  /** Get look-up table mode.
   *  The currently programmed look-up table mode will be retrieved
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param LUTMode currently programmed look-up table mode
   *  @return success or error code
   */
  __usrdllexport__ SVU3V_RETURN
  Camera_getLookupTableMode(Camera_handle hCamera,
                            LUT_MODE *LUTMode);

  /** setLookupTable.
   *  A user-defined lookup table will be uploaded to the camera. The size has to match
   *  the lookup table size that is supported by the camera (1024 for 10to8 or 4096 for 12to8). 
   *
   *  @see Camera_isCameraFeature()
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param LookupTable an array of user-defined lookup table values (bytes)
   *  @param LookupTableSize the size of the user-defined lookup table
   *  @return success or error code
   */
  __usrdllexport__ SVU3V_RETURN
  Camera_setLookupTable(Camera_handle hCamera,
                        unsigned char *LookupTable,
                        int LookupTableSize);

  /** getLookupTable.
   *  The currently installed lookup table will be downloaded from the camera. The size of the 
   *  reserved download space has to match the lookup table size (1024 for 10to8 or 4096 for 12to8).
   *
   *  @see Camera_isCameraFeature()
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param LookupTable an array for downloading the lookup table from camera
   *  @param LookupTableSize the size of the provided lookup table space
   *  @return success or error code
   */
  __usrdllexport__ SVU3V_RETURN
  Camera_getLookupTable(Camera_handle hCamera,
                        unsigned char *LookupTable,
                        int LookupTableSize);

  /** startImageCorrection.
   *  A particular step inside of acquiring a correction image for either,
	 *  flat field correction (FFC) or shading correction will be started:
	 *    - acquire a black image
	 *    - acquire a white image
	 *    - save a correction image to camera's persistent memory
	 *
   *  @see Camera_isCameraFeature()
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param ImageCorrectionStep a particular step from running an image correction
   *  @return success or error code
	 */
  __usrdllexport__ SVU3V_RETURN
  Camera_startImageCorrection(Camera_handle hCamera,
                              IMAGE_CORRECTION_STEP ImageCorrectionStep);

  /** isIdleImageCorrection.
   *  A launched image correction process will be checked whether a recently
	 *  initiated image correction step has be finished:
	 *    - acquire a black image
	 *    - acquire a white image
	 *    - save a correction image to camera's persistent memory
	 *
   *  @see Camera_isCameraFeature()
   *  @see CameraContainer_getCamera()
   *
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param ImageCorrectionStep a programmed particular step from running an image correction
   *  @param isIdle  whether a recently initiated image correction is Idle
   *  @return success or error code
	 */
  __usrdllexport__ SVU3V_RETURN
  Camera_isIdleImageCorrection(Camera_handle hCamera,
															 IMAGE_CORRECTION_STEP *ImageCorrectionStep,
				 											 bool *isIdle);

  /** setImageCorrection.
   *  A camera will be switched to one of the following image correction modes
	 *    - None (image correction is off)
	 *    - Offset only (available for Flat Field Correction, FFC)
	 *    - Enabled (image correction is on)
	 *  If image correction is enabled, then it depends on the camera whether 
	 *  Flat Field Correction (FFC) is enabled (gain and offset for each pixel) 
	 *  or Shading Correction (gain interpolation for a group of adjacent pixels)
	 *
   *  @see Camera_isCameraFeature()
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param ImageCorrectionMode one of above image correction modes
   *  @return success or error code
	 */
  __usrdllexport__ SVU3V_RETURN
  Camera_setImageCorrection(Camera_handle hCamera,
														IMAGE_CORRECTION_MODE ImageCorrectionMode);

  /** getImageCorrection.
   *  A camera will be queried for current image correction mode, either of:
	 *    - None (image correction is off)
	 *    - Offset only (available for Flat Field Correction, FFC)
	 *    - Enabled (image correction is on)
	 *  If image correction is enabled, then it depends on the camera whether 
	 *  Flat Field Correction (FFC) is enabled (gain and offset for each pixel) 
	 *  or Shading Correction (gain interpolation for a group of adjacent pixels)
	 *
   *  @see Camera_isCameraFeature()
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param ImageCorrectionMode currently programmed correction modes
   *  @return success or error code
	 */
  __usrdllexport__ SVU3V_RETURN
  Camera_getImageCorrection(Camera_handle hCamera,
														IMAGE_CORRECTION_MODE *ImageCorrectionMode);
  /** Set pixels correction Map
   * A camera will be switched to one of the following pixels correction maps if selected:
	  *    - factory map
	  *    - SVS map
	  *    - custom map
      * the pixels correction control for the currently selected map is enabled per default.
	  * 
   *
   *  @see Camera_isCameraFeature()
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param PixelsCorrectionMap one of above pixels correction maps
   *  @return success or error code
	 */
  __usrdllexport__ SVU3V_RETURN
  Camera_setPixelsCorrectionMap(Camera_handle hCamera,
								  PIXELS_CORRECTION_MAP_SELECT PixelsCorrectionMap);

  /** Get pixels correction map 
   *  A camera will be queried for current selected pixels correction map
   *
   *  @see Camera_isCameraFeature()
   *  @see Camera_isCameraFeature()
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param PixelsCorrectionMap a currently selected map 
   *  @return success or error code
	 */
   __usrdllexport__ SVU3V_RETURN
  Camera_getPixelsCorrectionMap(Camera_handle hCamera,
								PIXELS_CORRECTION_MAP_SELECT *PixelsCorrectionMap);

    /** Set Pixels Correction Control enable.
   *  The Pixels Correction Control status will be switched on or off.
   *  @see Camera_isCameraFeature()
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param isPixelsCorrectionEnabled whether Pixels Correction control has to be enabled or disabled    
   *  @return success or error code
	 */
 __usrdllexport__ SVU3V_RETURN
  Camera_setPixelsCorrectionControlEnabel(Camera_handle hCamera,
									bool isPixelsCorrectionEnabled);

  /** Get pixels correction control enabel.
   *  A camera will be queried for current pixels correction control status.
   *  the pixels correction can be enabled or disabled.
   *  @see Camera_isCameraFeature()
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param isPixelsCorrectionEnabled the programmed status of the pixels correction control.
   *  @return success or error code
	 */
 __usrdllexport__ SVU3V_RETURN
  Camera_getPixelsCorrectionControlEnabel(Camera_handle hCamera,
									bool *isPixelsCorrectionEnabled);
									
  /** Set Pixels Correction Control mark.
   *  The defect pixels will be marked or not.
   *  @see Camera_isCameraFeature()
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param isPixelsCorrectionMarked whether defect pixels have to be marked or not  
   *  @return success or error code
	 */
 __usrdllexport__ SVU3V_RETURN
  Camera_setPixelsCorrectionControlMark(Camera_handle hCamera,
									bool isPixelsCorrectionMarked);

  /** Get pixels correction control mark.
   *  A camera will be queried for current pixels correction control status. 
   *  the defect pixels can be marked or not.
   *  
   *  @see Camera_isCameraFeature()
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param isPixelsCorrectionMarked the programmed mark status of the pixels correction control
   *  @return success or error code
	 */
 __usrdllexport__ SVU3V_RETURN
  Camera_getPixelsCorrectionControlMark(Camera_handle hCamera,
									bool *isPixelsCorrectionMarked);


  /** set pixels correction map offset 
   *  The offset x and y of the currently slected map will be set to the provided values
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param OffsetX the new value for pixel offset in x-axis 
   *  @param OffsetY the new value for pixel offset in y-axis 
   *  @return success or error code
   */

  __usrdllexport__ SVU3V_RETURN
 Camera_setPixelsCorrectionMapOffset(Camera_handle hCamera,
										  int  OffsetX, int  OffsetY);

  /** Get pixels correction map offset 
   *  The offset x and y values of the currently slected map will be returned
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param OffsetX the currently programmed value for pixel offset in x-axis 
   *  @param OffsetY the currently programmed value for pixel offset in Y-axis 
   *  @return success or error code
   */
  __usrdllexport__ SVU3V_RETURN
 Camera_getPixelsCorrectionMapOffset(Camera_handle hCamera,
									int *OffsetX, int *OffsetY);
									 

  /** Get pixels correction map size
   *  The currently coordinates number of defect pixels in selected map will be returned
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param MapSize the currently programmed number of pixels coordinates in selected map 
   *  @return success or error code
   */
   __usrdllexport__ SVU3V_RETURN
  Camera_getPixelsCorrectionMapSize(Camera_handle hCamera,
									 unsigned int *MapSize);

  /** Get maximal pixels correction map size
   *  The Maximal pixels coordinates number per map will be returned
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param MaximalMapSize the Maximal programmed number of defect pixels coordinates per map  
   *  @return success or error code
   */
    __usrdllexport__ SVU3V_RETURN
  Camera_getMaximalPixelsCorrectionMapSize(Camera_handle hCamera,
											 unsigned int *MaximalMapSize);
 

  /** Set map index coordinate
   *  write a new X and Y coordinate accordingly to a map index that was specified
   *  as an input parameter
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param MapIndex the map index from zero to one less the Map size 
   *  @param X_Coordinate the new X coordinate to be written
   *  @param Y_Coordinate the new Y coordinate to be written
   *  @return success or error code
   */
   __usrdllexport__ SVU3V_RETURN
  Camera_setMapIndexCoordinate(Camera_handle hCamera,
									unsigned int MapIndex, 
									unsigned int X_Coordinate, unsigned int Y_Coordinate );
									
 
  /** Get map index coordinate 
   *  get X and Y coordinate accordingly to a map index that was specified
   *  as an input parameter
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param MapIndex the map index from zero to one less the map size 
   *  @param CoordinateX the programmed X coordinate accordingly to a map index 
   *  @param CoordinateY the programmed Y coordinate accordingly to a map index 
   *  @return success or error code
   */
   __usrdllexport__ SVU3V_RETURN
  Camera_getMapIndexCoordinate(Camera_handle hCamera,
									unsigned int MapIndex, 
									unsigned int *CoordinateX, unsigned int *CoordinateY );
  /** Delete Pixel coordinate from map 
   *  delete Pixel Coordinate accordingly to a map index that was specified
   *  as an input parameter
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param MapIndex the map index from zero to one less the map size
   *  @return success or error code
   */

   __usrdllexport__ SVU3V_RETURN
  Camera_deletePixelCoordinateFromMap(Camera_handle hCamera, unsigned int MapIndex);	
  

//------------------------------------------------------------------------------
// 19 - Special control: IOMux configuration
//------------------------------------------------------------------------------

  /** getMaxIOMuxIN.
   *  The maximal number of IN signals (signal sources) to the multiplexer will 
   *  be returned that are currently available in the camera for connecting them 
   *  to the multiplexer's OUT signals.
   *
   *  @see Camera_isCameraFeature()
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param MaxIOMuxINSignals the currently supported number of IN signals (signal sources)
   *  @return success or error code
   */
  __usrdllexport__ SVU3V_RETURN
  Camera_getMaxIOMuxIN(Camera_handle hCamera,
                       int *MaxIOMuxINSignals);

  /** getMaxIOMuxOut.
   *  The maximal number of OUT signals (signal sinks) will be returned that
   *  are currently activated in the camera's IO multiplexer.
   *
   *  @see Camera_isCameraFeature()
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param MaxIOMuxOUTSignals the currently supported number of OUT signals (signal sinks)
   *  @return success or error code
   */
  __usrdllexport__ SVU3V_RETURN
  Camera_getMaxIOMuxOUT(Camera_handle hCamera,
                        int *MaxIOMuxOUTSignals);

  /** setIOAssignment.
   *  An OUT signal (signal sink) will get one or multiple IN signals (signal
   *  sources) assigned in a camera's multiplexer. In case of multiple signal
   *  sources (IN signals) those signals will be or'd for combining them to
   *  one 32-bit value that will subsequently be assigned to an OUT signal.
   *
   *  @see Camera_isCameraFeature()
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param IOMuxOUT the multiplexer's OUT signal (signal sink) to be configured
   *  @param SignalIOMuxIN the IN signal vector (signal sources) to be activated
   *  @return success or error code
   */
  __usrdllexport__ SVU3V_RETURN
  Camera_setIOAssignment(Camera_handle hCamera,
                         SVU3V_IOMux_OUT IOMuxOUT,
                         unsigned int SignalIOMuxIN);

  /** getIOAssignment.
   *  Current assignment of IN signals (signal sources) to an OUT signal
   *  (signal sink) will be retrieved from a camera's multiplexer.
   *
   *  @see Camera_isCameraFeature()
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param IOMuxOUT the multiplexer's OUT signal (signal sink) to be queried
   *  @param ProgrammedIOMuxIN the IN signal vector (signal sources) connected to the OUT signal
   *  @return success or error code
   */
  __usrdllexport__ SVU3V_RETURN
  Camera_getIOAssignment(Camera_handle hCamera,
                         SVU3V_IOMux_OUT IOMuxOUT,
                         unsigned int *ProgrammedIOMuxIN);

//------------------------------------------------------------------------------
// 20 - Special control: IO control
//------------------------------------------------------------------------------

  /** setIOMuxIN.
   *  The complete vector of IN signals (source signals, max 32 bits) will be
   *  set in a camera's multiplexer in one go.
   *
   *  @see Camera_isCameraFeature()
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param VectorIOMuxIN the IN signal vector's new state to be assigned
   *  @return success or error code
   */
  __usrdllexport__ SVU3V_RETURN
  Camera_setIOMuxIN(Camera_handle hCamera,
                    unsigned int VectorIOMuxIN);

  /** getIOMuxIN.
   *  The complete vector of IN signals (source signals, max 32 bits) will be
   *  read out from a camera's multiplexer in one go.
   *
   *  @see Camera_isCameraFeature()
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param VectorIOMuxIN the IN signal vector's current state
   *  @return success or error code
   */
  __usrdllexport__ SVU3V_RETURN
  Camera_getIOMuxIN(Camera_handle hCamera,
                    unsigned int *VectorIOMuxIN);

  /** setIO.
   *  A single IN signal (source signal, one out of max 32 bits) will be set.
   *
   *  @see Camera_isCameraFeature()
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param SignalIOMuxIN a particular signal from the IN signal vector
   *  @param SignalValue the signal value to be assigned to the IN signal
   *  @return success or error code
   */
  __usrdllexport__ SVU3V_RETURN
  Camera_setIO(Camera_handle hCamera,
               SVU3V_IOMux_IN SignalIOMuxIN,
               IO_SIGNAL SignalValue);

  /** getIO.
   *  A single IN signal (source signal, one out of max 32 bits) will be read.
   *
   *  @see Camera_isCameraFeature()
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param SignalIOMuxIN a particular signal from the IN signal vector
   *  @param SignalValue  the current value of the selected IN signal
   *  @return success or error code
   */
  __usrdllexport__ SVU3V_RETURN
  Camera_getIO(Camera_handle hCamera,
               SVU3V_IOMux_IN SignalIOMuxIN,
               IO_SIGNAL *SignalValue);

  /** setAcqLEDOverride.
   *  Override default LED mode by an alternative behavior:
   *  - blue:    waiting for trigger
   *  - cyan:    exposure
   *  - magenta: read-out
   *
   *  @see Camera_isCameraFeature()
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param isOverrideActive whether LED override will be activated or deactivated
   *  @return success or error code
   */
  __usrdllexport__ SVU3V_RETURN
  Camera_setAcqLEDOverride(Camera_handle hCamera,
                           bool isOverrideActive);

  /** getAcqLEDOverride.
   *  Check whether default LED mode was overridden by an alternative behavior:
   *  - blue:    waiting for trigger
   *  - cyan:    exposure
   *  - magenta: read-out
   *
   *  @see Camera_isCameraFeature()
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param isOverrideActive a flag indicating whether LED override is currently activated
   *  @return success or error code
   */
  __usrdllexport__ SVU3V_RETURN
  Camera_getAcqLEDOverride(Camera_handle hCamera,
                           bool *isOverrideActive);

  /** setLEDIntensity.
   *  The LED intensity will be controlled in the range 0..255 as follows:
   *  0   - dark
   *  255 - light
   *
   *  @see Camera_isCameraFeature()
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param LEDIntensity the new intensity (0..255=max) to be assigned to the LED
   *  @return success or error code
   */
  __usrdllexport__ SVU3V_RETURN
  Camera_setLEDIntensity(Camera_handle hCamera,
                         int LEDIntensity);

  /** getLEDIntensity.
   *  The LED intensity will be retrieved from camera with the following meaning:
   *  0   - dark
   *  255 - light
   *
   *  @see Camera_isCameraFeature()
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param LEDIntensity currently assigned LED intensity
   *  @return success or error code
   */
  __usrdllexport__ SVU3V_RETURN
  Camera_getLEDIntensity(Camera_handle hCamera,
                         int *LEDIntensity);

//------------------------------------------------------------------------------
// 21 - Special control: Serial communication
//------------------------------------------------------------------------------

  /** setUARTBuffer.
   *  A block of data (max 512 bytes) will be sent to the camera's UART for
   *  transmitting it over the serial line to a receiver.
   *
   *  @see Camera_isCameraFeature()
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param Data a pointer to a block of data to be sent over the camera's UART
   *  @param DataLength the length of the data block (1..512)
   *  @return success or error code
   */
  __usrdllexport__ SVU3V_RETURN
  Camera_setUARTBuffer(Camera_handle hCamera,
                       unsigned char *Data,
                       int DataLength);

  /** getUARTBuffer.
   *  A block of data will be retrieved which has arrived in the camera's UART
   *  receiver buffer. If this function returns the maximal possible byte count 
   *  of the serial buffer (=512 bytes) then there might be more data available 
   *  which should be retrieved by a subsequent call to this function.
   *
   *  NOTE: If DataLengthMax is set to less than the serial buffer size (512 bytes)
   *        and if DataLengthMax is not sufficient for returning all buffered data
   *        then data loss will occur. 
   *
   *  HINT: If more data will be transferred than the serial buffer size (512 bytes),
   *        then DataLengthMax has to match that serial buffer size. Only then a 
   *        seamless transfer of data chunks can be performed, each chunk being the 
   *        size of the serial buffer (512 bytes).
   *
   *  @see Camera_isCameraFeature()
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param Data a pointer to a data buffer
   *  @param DataLengthReceived a pointer to a value for returning actual data read
   *  @param DataLengthMax the maximal data length to be read (1..512)
   *  @param Timeout a time period [s] after which the function returns if no data was received
   *  @return success or error code
   */
  __usrdllexport__ SVU3V_RETURN
  Camera_getUARTBuffer(Camera_handle hCamera,
                       unsigned char *Data,
                       int *DataLengthReceived,
                       int DataLengthMax,
                       float Timeout);

  /** setUARTBaud.
   *  The baud rate of the camera's UART will be set to one out of a set of 
   *  pre-defined baud rates. Alternatively, any baud rate can be provided
   *  as integer which would not have to comply with any pre-defined value.
   *
   *  @see Camera_isCameraFeature()
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param BaudRate the baud rate to be assigned to the UART
   *  @return success or error code
   */
  __usrdllexport__ SVU3V_RETURN
  Camera_setUARTBaud(Camera_handle hCamera,
                     SVU3V_BaudRate BaudRate);

  /** getUARTBaud.
   *  The currently set baud rate in the camera's UART will be returned.
   *
   *  @see Camera_isCameraFeature()
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param BaudRate the UART's currently assigned baud rate
   *  @return success or error code
   */
  __usrdllexport__ SVU3V_RETURN
  Camera_getUARTBaud(Camera_handle hCamera,
                     SVU3V_BaudRate *BaudRate);



//------------------------------------------------------------------------------
// 22 - Controlling camera: Direct register and memory access
//------------------------------------------------------------------------------

  /** Set U3V camera register.
      *  A register of a SVS U3V camera will be directly written to
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param RegisterAddress a valid address of a SVS U3V camera register
   *  @param RegisterValue a value that has to be written to selected register
   *  @param U3VCameraAccessKey a valid key for directly accessing a U3V camera
   *  @return success or error code
   */
  __usrdllexport__ SVU3V_RETURN
  Camera_setU3VCameraRegister(Camera_handle hCamera,
                               unsigned int RegisterAddress,
                               unsigned int RegisterValue,
                               unsigned int U3VCameraAccessKey);

  /** Get U3V camera register.
   *  A value from a SVS U3V camera register will be directly read out
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param RegisterAddress a valid address of a U3V camera register
   *  @param ProgammedRegisterValue the current programmed value will be returned
   *  @param U3VCameraAccessKey a valid key for directly accessing a U3V camera
   *  @return success or error code
   */
  __usrdllexport__ SVU3V_RETURN
  Camera_getU3VCameraRegister(Camera_handle hCamera,
                               unsigned int RegisterAddress,
                               unsigned int *ProgammedRegisterValue,
                               unsigned int U3VCameraAccessKey);

  /** Write U3V camera memory.
   *  A block of data will be written to the memory of a SVS U3V camera
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param MemoryAddress a valid memory address in a SVS U3V camera
   *  @param DataBlock a block of data that has to be written to selected memory
   *  @param DataLength the length of the specified DataBlock
   *  @param U3VCameraAccessKey a valid key for directly accessing a U3V camera
   *  @return success or error code
   */
  __usrdllexport__ SVU3V_RETURN
  Camera_writeU3VCameraMemory(Camera_handle hCamera,
                               unsigned int  MemoryAddress,
                               unsigned char *DataBlock,
                               unsigned int  DataLength,
                               unsigned int  U3VCameraAccessKey);

  /** Read U3V camera memory.
   *  A block of data will be read from the memory of a SVS U3V camera
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param MemoryAddress a valid memory address in a SVS U3V camera
   *  @param DataBlock an address where the data from selected memory will be written to
   *  @param DataLength the data length to be read from the camera's memory
   *  @param U3VCameraAccessKey a valid key for directly accessing a U3V camera
   *  @return success or error code
   */
  __usrdllexport__ SVU3V_RETURN
  Camera_readU3VCameraMemory(Camera_handle hCamera,
                              unsigned int  MemoryAddress,
                              unsigned char *DataBlock,
                              unsigned int  DataLength,
                              unsigned int  U3VCameraAccessKey);
 
//------------------------------------------------------------------------------
// 23 - Controlling camera: Persistent settings and recovery
//------------------------------------------------------------------------------
 
  /** Write EEPROM defaults.
   *  The current settings will be made the EEPROM defaults that will be
   *  restored on each camera start-up
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @return success or error code
   */
  __usrdllexport__ SVU3V_RETURN
  Camera_writeEEPROM(Camera_handle hCamera);

  /** Read EEPROM defaults.
   *  The EEPROM content will be moved to the appropriate camera registers.
   *  This operation will restore the camera settings that were active when
   *  the EEPROM write function was performed
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @return success or error code
   */
  __usrdllexport__ SVU3V_RETURN
  Camera_readEEPROM(Camera_handle hCamera);

  /** Restore factory defaults.
   *  The camera's registers will be restored to the factory defaults and at
   *  the same time those settings will be written as default to EEPROM
   *
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @return success or error code
   */
  __usrdllexport__ SVU3V_RETURN
  Camera_restoreFactoryDefaults(Camera_handle hCamera);

 /** Load settings from XML
  *  New camera settings will be loaded from a XML file.
  *  The  XML file content will be moved to the appropriate camera registers.
  *  In this operation the XML file will be used instead of the EEPROM. 
  *  
  *  @see CameraContainer_getCamera()
  *  @param hCamera a camera handle received from CameraContainer_getCamera()
  *  @param Filename a complete path and filename where to load the settings from
  *  @return success or error code
  */
  __usrdllexport__ SVU3V_RETURN
  Camera_loadSettingsFromXml(Camera_handle hCamera,
                             char *Filename);
   
 /** Save settings to XML
  *  The current settings will be stored in a XML file 
  *  In this operation the XML file will be used instead of the EEPROM. 
  *
  *  @see CameraContainer_getCamera()
  *  @param hCamera a camera handle received from CameraContainer_getCamera()
  *  @param Filename a complete path and filename where to write the new settings.
  *  @return success or error code
  */
  __usrdllexport__ SVU3V_RETURN
  Camera_SaveSettingsToXml(Camera_handle hCamera,
                           char *Filename);

//------------------------------------------------------------------------------
// 24 - General functions
//------------------------------------------------------------------------------

  /** Estimate white balance.
   *  Current image will be investigated for a suitable white balance setting
   *
   *  @param BufferRGB a buffer with current RGB image
   *  @param BufferLength the length of the RGB buffer
   *  @param Red new value for red color
   *  @param Green new value for green color
   *  @param Blue new value for blue color
   *  @return success or error code
   */
  __usrdllexport__ SVU3V_RETURN
  SVU3V_estimateWhiteBalance(unsigned char *BufferRGB,
                              int BufferLength,
                              float *Red,
                              float *Green ,
                              float *Blue);

  /** Estimate white balance with and without using a gray card.
   *  Current image will be investigated for a suitable white balance
   *
   *  @param BufferRGB a buffer with current RGB image
   *  @param PixelNumber of the Current image
   *  @param Red new value for red color
   *  @param Green new value for green color
   *  @param Blue new value for blue color
   *  @param Whitebalance_Art white balance estimation methode used
   *  @return success or error code
   */
  __usrdllexport__ SVU3V_RETURN
  SVU3V_estimateWhiteBalanceExtended(unsigned char *BufferRGB,
													int PixelNumber,
													int &Red,
													int &Green,
													int &Blue,
													SVU3V_Whitebalance_SELECT  Whitebalance_Art );
  
  /** Write image as a bitmap file to disk
   *  An image given by image data, geometry and type will be written to a
   *  specified location on disk.
   *
   *  @param Filename a path and filename for the bitmap file
   *  @param Data a pointer to image data
   *  @param SizeX the width of the image
   *  @param SizeY the height of the image
   *  @param PixelType either GVSP_PIX_MONO8 or GVSP_PIX_RGB24
   *  @return success or error code
   */
  __usrdllexport__ SVU3V_RETURN
  SVU3V_writeImageToBitmapFile(char *Filename, unsigned char *Data, int SizeX, int SizeY, GVSP_PIXEL_TYPE PixelType);

//------------------------------------------------------------------------------
// 25 - Special control: Lens control
//------------------------------------------------------------------------------
  

/** Is Lens available.
   *The camera will be queried whether a MFT Lens is available or not.
   *
   *  @see Camera_isCameraFeature()
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param isAvailable a flag indicating whether Objective is availabel or not
   *  @return success or error code	
   */
 __usrdllexport__ SVU3V_RETURN
  Camera_isLensAvailable(Camera_handle hCamera, bool *isAvailable); 



/** Get lens name.
   *  The lens name that is obtained from the lens firmware will be
   *  returned
   *
   *  @see Camera_isCameraFeature() 
   *  @see CameraContainer_getCamera()
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @return success or error code	
   */
  __usrdllexport__ char*
  Camera_getLensName(Camera_handle hCamera);



/** setLensFocalLenght.
   *
   *  A provided focal length  will be transferred to lens.
   *  @see Camera_isCameraFeature()  
   *  @see CameraContainer_getCamera()  
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param FocalLenght the focal length to be assigned to the Lens (granularity: 1/10 mm, ex. 350 corresponds to 35 mm). 
   *  @return success or error code
   */
 __usrdllexport__ SVU3V_RETURN
	Camera_setLensFocalLenght(Camera_handle hCamera, unsigned int FocalLenght);


/** getLensFocalLenght.
   
   *  The currently set focal length  of the lens will be returned.
   *  @see Camera_isCameraFeature() 
   *  @see CameraContainer_getCamera()  
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param FocalLenght the currently assigned focal length (granularity: 1/10 mm, ex. 350 corresponds to 35 mm).
   *  @return success or error code
   */
 __usrdllexport__ SVU3V_RETURN
	Camera_getLensFocalLenght(Camera_handle hCamera, unsigned int *FocalLenght);
	
/** getLensFocalLenghtMin.
   
   *  Get the minimal focal length that can be used.
   *  @see Camera_isCameraFeature() 
   *  @see CameraContainer_getCamera()  
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param FocalLenghtMin the minimal focal length setting(granularity: 1/10 mm, ex. 140 corresponds to 14 mm). 
   *  @return success or error code
   */
 __usrdllexport__ SVU3V_RETURN
	Camera_getLensFocalLenghtMin(Camera_handle hCamera, unsigned int *FocalLenghtMin);
	
  /** getLensFocalLenghtMax.
   *  Get the maximal focal length that can be used.
   *  @see Camera_isCameraFeature() 
   *  @see CameraContainer_getCamera()  
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param FocalLenghtMax the maximal focal length setting (granularity: 1/10 mm, ex. 420 corresponds to 42 mm). 
   *  @return success or error code
   */
 __usrdllexport__ SVU3V_RETURN
	Camera_getLensFocalLenghtMax(Camera_handle hCamera, unsigned int *FocalLenghtMax);


  /** set focus unit
   *  A selected focus unit will be transferred to lens.
   *
   *  @see Camera_isCameraFeature() 
   *  @see CameraContainer_getCamera()  
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param Selected_unit the focus unit( mm or 1/10 mm) to be assigned to Lens
   *  @return success or error code
   */
 __usrdllexport__ SVU3V_RETURN
	Camera_setLensFocusUnit(Camera_handle hCamera, FOCUS_UNIT Selected_unit );


  /** get focus unit
   *  The currently focus unit will be returned.
   *  @see Camera_isCameraFeature() 
   *  @see CameraContainer_getCamera()  
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param Selected_unit the currently used focus unit.
   *  @return success or error code
   */
 __usrdllexport__ SVU3V_RETURN
  Camera_getLensFocusUnit(Camera_handle hCamera, FOCUS_UNIT *Selected_unit );


  /** setLensFocus.
   *  A provided focus will be transferred to lens.
   *
   *  @see Camera_isCameraFeature() 
   *  @see CameraContainer_getCamera()  
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param Focus the focus in the selected unit to be assigned to lens (default unit is mm) 
   *  @return success or error code
   */
 __usrdllexport__ SVU3V_RETURN
	Camera_setLensFocus(Camera_handle hCamera, unsigned int Focus );


  /** getLensFocus.
   *  The currently set focus of the lens will be returned.
   *
   *  @see Camera_isCameraFeature() 
   *  @see CameraContainer_getCamera()  
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param Focus the currently assigned focus in the selected unit(default unit is mm)
   *  @return success or error code
   */
 __usrdllexport__ SVU3V_RETURN
  Camera_getLensFocus(Camera_handle hCamera, unsigned int *Focus);
  
  /** getLensFocusMin.
   *  Get the minimal focus that can be used.
   *
   *  @see Camera_isCameraFeature() 
   *  @see CameraContainer_getCamera()  
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param FocusMin the minimal focus setting in the selected unit(default unit is mm)
   *  @return success or error code
   */
 __usrdllexport__ SVU3V_RETURN
   Camera_getLensFocusMin(Camera_handle hCamera, unsigned int *FocusMin );
  
  
  /** getLensFocusMax.
   *  Get the maximal focus that can be used.
   *
   *  @see Camera_isCameraFeature() 
   *  @see CameraContainer_getCamera()  
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param FocusMax the maximal focus setting in the selected unit(default unit is mm)
   *  @return success or error code
   */
 __usrdllexport__ SVU3V_RETURN
  Camera_getLensFocusMax(Camera_handle hCamera, unsigned int *FocusMax); 
  

  /** setLensAperture.
   *  A provided aperture will be transferred to lens.
   *
   *  @see Camera_isCameraFeature() 
   *  @see CameraContainer_getCamera()  
   *  @param hCamera a camera handle received from CameraContainer_getCamera() 
   *  @param Aperture the aperture to be assigned to the Lens (granularity: 1/10 , ex. 90 corresponds to 9)
   *  @return success or error code
   */
 __usrdllexport__ SVU3V_RETURN
	Camera_setLensAperture(Camera_handle hCamera, unsigned int Aperture);


  /** getLensAperture.
   *  The currently set aperture of the lens will be returned.
   *
   *  @see Camera_isCameraFeature() 
   *  @see CameraContainer_getCamera()  
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param Aperture the currently assigned aperture (granularity: 1/10 , ex. 90 corresponds to 9)
   *  @return success or error code
   */
 __usrdllexport__ SVU3V_RETURN
	Camera_getLensAperture(Camera_handle hCamera, unsigned int *Aperture);
	
  /** getLensApertureMin.
   *  Get the minimal aperture that can be used.
   *
   *  @see Camera_isCameraFeature() 
   *  @see CameraContainer_getCamera()  
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param ApertureMin the minimal aperture setting (granularity: 1/10 , ex. 35 corresponds to 3.5) 
   *  @return success or error code
   */
 __usrdllexport__ SVU3V_RETURN
	Camera_getLensApertureMin(Camera_handle hCamera, unsigned int *ApertureMin);

  /** getLensApertureMax.
   *  Get the maximal aperture that can be used.
   *
   *  @see Camera_isCameraFeature() 
   *  @see CameraContainer_getCamera()  
   *  @param hCamera a camera handle received from CameraContainer_getCamera()
   *  @param ApertureMax the maximal aperture setting (granularity: 1/10 , ex. 90 corresponds to 9)
   *  @return success or error code
   */
 __usrdllexport__ SVU3V_RETURN
	Camera_getLensApertureMax(Camera_handle hCamera, unsigned int *ApertureMax);


} // extern "C"

#ifdef __BORLANDC__
#pragma option pop
#endif

#endif


	/** 1394-Based Digital Camera Control Library
	 *  Bayer pattern decoding functions
	 *  Copyright (C) Damien Douxchamps <ddouxchamps@users.sf.net>
	 *
	 *  Written by Damien Douxchamps and Frederic Devernay
	 *
	 *  This library is free software; you can redistribute it and/or
	 *  modify it under the terms of the GNU Lesser General Public
	 *  License as published by the Free Software Foundation; either
	 *  version 2.1 of the License, or (at your option) any later version.
	 *
	 *  This library is distributed in the hope that it will be useful,
	 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
	 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
	 *  Lesser General Public License for more details.
	 *
	 *  You should have received a copy of the GNU Lesser General Public
	 *  License along with this library; if not, write to the Free Software
	 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
	 */

#ifndef BAYERH
#define BAYERH

	enum {
	 DC1394_SUCCESS = 0,
	 DC1394_INVALID_BAYER_METHOD
	};

	enum {
	  DC1394_BYTE_ORDER_UYVY=0,
	  DC1394_BYTE_ORDER_YUYV
	};

	enum {
	  DC1394_BAYER_METHOD_NONE  = -1,
	  DC1394_BAYER_METHOD_NEAREST = 0,
	  DC1394_BAYER_METHOD_SIMPLE,
	  DC1394_BAYER_METHOD_BILINEAR,
	  DC1394_BAYER_METHOD_HQLINEAR,
	  DC1394_BAYER_METHOD_EDGESENSE,
	  DC1394_BAYER_METHOD_DOWNSAMPLE,
	};

	#define DC1394_BAYER_METHOD_MIN      DC1394_BAYER_METHOD_NEAREST
	#define DC1394_BAYER_METHOD_MAX      DC1394_BAYER_METHOD_EDGESENSE
	#define DC1394_BAYER_METHOD_NUM     (DC1394_BAYER_METHOD_MAX-DC1394_BAYER_METHOD_MIN+1)

	enum {
	  DC1394_COLOR_FILTER_RGGB = 512,
	  DC1394_COLOR_FILTER_GBRG,
	  DC1394_COLOR_FILTER_GRBG,
	  DC1394_COLOR_FILTER_BGGR
	};

	typedef unsigned short uint16_t;
	typedef unsigned int uint_t;

	int
	dc1394_bayer_decoding_8bit(const unsigned char *bayer, unsigned char *rgb, unsigned int sx, unsigned int sy, unsigned int tile, unsigned int method);

	int
	dc1394_bayer_decoding_16bit(const uint16_t *bayer, uint16_t *rgb, uint_t sx, uint_t sy, uint_t tile, uint_t bits, uint_t method);

#endif


