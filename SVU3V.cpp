/*******************************************************************************
 * SVU3V       Dynamic loading of SVU3V camera access functions
 *******************************************************************************
 *
 * Version:     2.1.0 / february 2016  
 *   
 * Copyright:   SVS VISTEK GmbH
 *
 *******************************************************************************
 *
 * THIS FILE CONTAINS BAYER PATTERN CONVERSION FROM THE FOLLOWING SOURCE: 
 *
 * 1394-Based Digital Camera Control Library
 * Bayer pattern decoding functions
 * Copyright (C) Damien Douxchamps <ddouxchamps@users.sf.net>
 *
 * Written by Damien Douxchamps and Frederic Devernay
 *******************************************************************************
 */
  
#include "windows.h"

#include "SVU3V.h" 

// Define a name for the SVS SVU3V API DLL
#ifdef X64
  #define SVU3V_DLL   _T("SVU3V.x64.dll")
#else
  #define SVU3V_DLL   _T("SVU3V.dll")
#endif

/**
 *  Handle for the U3V DLL
 */
HINSTANCE U3VDLL = NULL;

/**
 *  Enumerate all SVU3V function calls
 */
typedef
  enum
  {
    // 0 - SVU3V DLL (implicitly called)
    Func_isVersionCompliantDLL,
	
    // 1 - Camera: Discovery and bookkeeping
    Func_CameraContainer_create,
    Func_CameraContainer_delete,
    Func_CameraContainer_discovery,
    Func_CameraContainer_getNumberOfCameras,
    Func_CameraContainer_getCamera,
    Func_CameraContainer_findCamera,
    
	// 2 - Camera: Connection
    Func_Camera_openConnection,
    Func_Camera_closeConnection,

    // 3 - Camera: Information
    Func_Camera_getManufacturerName,
    Func_Camera_getModelName,
    Func_Camera_getDeviceVersion,
    Func_Camera_getManufacturerSpecificInformation,
    Func_Camera_getSerialNumber,
    Func_Camera_setUserDefinedName,
    Func_Camera_getUserDefinedName,
    Func_Camera_getPixelClock,
    Func_Camera_isCameraFeature,
    Func_Camera_readXMLFile,
	Func_Camera_getSensorTemperature,

    // 4 - Stream: Channel creation and control
    Func_StreamingChannel_create,
    Func_StreamingChannel_delete,

    // 5 - Stream: Channel info
    Func_StreamingChannel_getPixelType,
    Func_StreamingChannel_getImagePitch,
    Func_StreamingChannel_getImageSizeX,
    Func_StreamingChannel_getImageSizeY,

    // 6 - Stream: Image access
    Func_Image_getDataPointer,
    Func_Image_getSignalType,
    Func_Image_getCamera,
    Func_Image_release,

    // 7 - Stream: Image conversion
    Func_Image_getImageRGB,
    Func_Image_getImage12bitAs8bit,
    Func_Image_getImage12bitAs16bit,
    Func_Image_getImage16bitAs8bit,

    // 8 - Stream: Image characteristics
    Func_Image_getPixelType,
    Func_Image_getImageSize,
    Func_Image_getPitch,
    Func_Image_getSizeX,
    Func_Image_getSizeY,

    // 9 - Stream: Image statistics
    Func_Image_getImageID,
    Func_Image_getTimestamp,
	
    // 10 - Controlling camera: Frame rate
    Func_Camera_setFrameRate,
    Func_Camera_getFrameRate,
    Func_Camera_getFrameRateMin,
    Func_Camera_getFrameRateMax,
    Func_Camera_getFrameRateRange,
    Func_Camera_getFrameRateIncrement,

    // 11 - Controlling camera: Exposure
    Func_Camera_setExposureTime,
    Func_Camera_getExposureTime,
    Func_Camera_getExposureTimeMin,
    Func_Camera_getExposureTimeMax,
    Func_Camera_getExposureTimeRange,
    Func_Camera_getExposureTimeIncrement,
    Func_Camera_setExposureDelay,
    Func_Camera_getExposureDelay,
    Func_Camera_getExposureDelayMax,
    Func_Camera_getExposureDelayIncrement,

    // 12 - Controlling camera: Gain and offset
    Func_Camera_setGain,
    Func_Camera_getGain,
	Func_Camera_getGainMax,
	Func_Camera_getGainMaxExtended,
    Func_Camera_getGainIncrement,
    Func_Camera_setOffset,
    Func_Camera_getOffset,
    Func_Camera_getOffsetMax,

    // 13 - Controlling camera: Auto gain/exposure
    Func_Camera_setAutoGainEnabled,
    Func_Camera_getAutoGainEnabled,
    Func_Camera_setAutoGainBrightness,
    Func_Camera_getAutoGainBrightness,
    Func_Camera_setAutoGainDynamics,
    Func_Camera_getAutoGainDynamics,
    Func_Camera_setAutoGainLimits,
    Func_Camera_getAutoGainLimits,
    Func_Camera_setAutoExposureLimits,
    Func_Camera_getAutoExposureLimits,


    // 14 - Controlling camera: Acquisition trigger
    Func_Camera_setAcquisitionControl,
    Func_Camera_getAcquisitionControl,
    Func_Camera_setAcquisitionMode,
    Func_Camera_getAcquisitionMode,
    Func_Camera_softwareTrigger,
    Func_Camera_setTriggerPolarity,
    Func_Camera_getTriggerPolarity,
	Func_Camera_setPivMode,
	Func_Camera_getPivMode,
	Func_Camera_setDebouncerDuration,
  	Func_Camera_getDebouncerDuration,
	Func_Camera_setPrescalerDevisor,
	Func_Camera_getPrescalerDevisor,
	Func_Camera_loadSequenceParameters,
	Func_Camera_startSequencer,


    // 15 - Controlling camera: Strobe
  	Func_Camera_setStrobePolarityExtended,
	Func_Camera_getStrobePolarityExtended,
	Func_Camera_setStrobePositionExtended,
	Func_Camera_getStrobePositionExtended,
    Func_Camera_getStrobePositionMax,
    Func_Camera_getStrobePositionIncrement,
    Func_Camera_setStrobeDurationExtended,
	Func_Camera_getStrobeDurationExtended,
    Func_Camera_getStrobeDurationMax,
    Func_Camera_getStrobeDurationIncrement,

    // 16 - Controlling camera: Tap balance
	Func_Camera_setTapConfigurationEx,
    Func_Camera_getTapConfigurationEx,
    Func_Camera_setAutoTapBalanceMode,
    Func_Camera_getAutoTapBalanceMode,
	Func_Camera_setTapGain,
    Func_Camera_getTapGain,


    // 17 - Controlling camera: Image parameter
    Func_Camera_getImagerWidth,
    Func_Camera_getImagerHeight,
    Func_Camera_getImageSize,
    Func_Camera_getPitch,
    Func_Camera_getSizeX,
    Func_Camera_getSizeY,
    Func_Camera_setBinningMode,
    Func_Camera_getBinningMode,
    Func_Camera_setAreaOfInterest,
    Func_Camera_getAreaOfInterest,
    Func_Camera_getAreaOfInterestRange,
    Func_Camera_getAreaOfInterestIncrement,
	Func_Camera_setFlippingMode,	
	Func_Camera_getFlippingMode,
	Func_Camera_setShutterMode,
	Func_Camera_getShutterMode,

    // 18 - Controlling camera: Image appearance
    Func_Camera_getPixelType,
    Func_Camera_setPixelDepth,
    Func_Camera_getPixelDepth,
    Func_Camera_setWhiteBalance,
    Func_Camera_getWhiteBalance,
    Func_Camera_getWhiteBalanceMax,
    Func_Camera_setGammaCorrection,
    Func_Camera_setGammaCorrectionExt,
    Func_Camera_setLowpassFilter,
    Func_Camera_getLowpassFilter,
    Func_Camera_setLookupTableMode,
    Func_Camera_getLookupTableMode,
    Func_Camera_setLookupTable,
    Func_Camera_getLookupTable,
    Func_Camera_startImageCorrection,
    Func_Camera_isIdleImageCorrection,
    Func_Camera_setImageCorrection,
    Func_Camera_getImageCorrection,
	Func_Camera_setPixelsCorrectionMap,
	Func_Camera_getPixelsCorrectionMap,
	Func_Camera_setPixelsCorrectionControlEnabel,	
	Func_Camera_getPixelsCorrectionControlEnabel,
	Func_Camera_setPixelsCorrectionControlMark,
	Func_Camera_getPixelsCorrectionControlMark,
	Func_Camera_setPixelsCorrectionMapOffset,
	Func_Camera_getPixelsCorrectionMapOffset,
	Func_Camera_getPixelsCorrectionMapSize,
	Func_Camera_getMaximalPixelsCorrectionMapSize,
	Func_Camera_setMapIndexCoordinate,
	Func_Camera_getMapIndexCoordinate,
	Func_Camera_deletePixelCoordinateFromMap,

    // 19 - Special control: IOMux configuration
    Func_Camera_getMaxIOMuxIN,
    Func_Camera_getMaxIOMuxOUT,
    Func_Camera_setIOAssignment,
    Func_Camera_getIOAssignment,

    // 20 - Special control: IO control
    Func_Camera_setIOMuxIN,
    Func_Camera_getIOMuxIN,
    Func_Camera_setIO,
    Func_Camera_getIO,
    Func_Camera_setAcqLEDOverride,
    Func_Camera_getAcqLEDOverride,
    Func_Camera_setLEDIntensity,
    Func_Camera_getLEDIntensity,

    // 21 - Special control: Serial communication
    Func_Camera_setUARTBuffer,
    Func_Camera_getUARTBuffer,
    Func_Camera_setUARTBaud,
    Func_Camera_getUARTBaud,

    // 22 - Special control: Direct register and memory access
    Func_Camera_setU3VCameraRegister,
    Func_Camera_getU3VCameraRegister,
    Func_Camera_writeU3VCameraMemory,
    Func_Camera_readU3VCameraMemory,

    // 23 - Special control: Persistent settings and recovery
    Func_Camera_writeEEPROM,
    Func_Camera_readEEPROM,
    Func_Camera_restoreFactoryDefaults,
	Func_Camera_loadSettingsFromXml,
    Func_Camera_SaveSettingsToXml,

    // 24 - General functions
    Func_SVU3V_estimateWhiteBalance,
    Func_SVU3V_estimateWhiteBalanceExtended,
    Func_SVU3V_writeImageToBitmapFile,
	

    // 25 - Special control: Lens control
    Func_Camera_isLensAvailable,
    Func_Camera_getLensName,

    Func_Camera_setLensFocalLenght,
    Func_Camera_getLensFocalLenght,
    Func_Camera_getLensFocalLenghtMin,
    Func_Camera_getLensFocalLenghtMax,
    
	Func_Camera_setLensFocusUnit,
    Func_Camera_getLensFocusUnit,
    Func_Camera_setLensFocus,
    Func_Camera_getLensFocus,
    Func_Camera_getLensFocusMin,
    Func_Camera_getLensFocusMax,

    Func_Camera_setLensAperture,
    Func_Camera_getLensAperture,
    Func_Camera_getLensApertureMin,
    Func_Camera_getLensApertureMax,

 

	// 00 - Consistency check
	//
	// The following function will be used to
	// check whether consistency of a loaded 
	// function table is OK.
	//
	Func_isVersionCompliantDLL_consistency_check,
  }                                                   
  SVU3V_FUNCTION;                                    

/**
 *  Array of function pointers that will be obtained from the SVU3V DLL
 */
struct _U3VFunc
{
  FARPROC function_pointer;
  SVU3V_FUNCTION function_id;
  char *function_name;
}
U3VFunc[] =
{
  // 0 - SVU3V DLL (implicitly called)
  NULL, Func_isVersionCompliantDLL,                     "isVersionCompliantDLL",

  // 1 - Camera: Discovery and bookkeeping
  NULL, Func_CameraContainer_create,                    "CameraContainer_create",
  NULL, Func_CameraContainer_delete,                    "CameraContainer_delete",
  NULL, Func_CameraContainer_discovery,                 "CameraContainer_discovery",
  NULL, Func_CameraContainer_getNumberOfCameras,        "CameraContainer_getNumberOfCameras",
  NULL, Func_CameraContainer_getCamera,                 "CameraContainer_getCamera",
  NULL, Func_CameraContainer_findCamera,                "CameraContainer_findCamera",

  // 2 - Camera: Connection
  NULL, Func_Camera_openConnection,                     "Camera_openConnection",
  NULL, Func_Camera_closeConnection,                    "Camera_closeConnection",

  // 3 - Camera: Information
  NULL, Func_Camera_getManufacturerName,                "Camera_getManufacturerName",
  NULL, Func_Camera_getModelName,                       "Camera_getModelName",
  NULL, Func_Camera_getDeviceVersion,                   "Camera_getDeviceVersion",
  NULL, Func_Camera_getManufacturerSpecificInformation, "Camera_getManufacturerSpecificInformation",
  NULL, Func_Camera_getSerialNumber,                    "Camera_getSerialNumber",
  NULL, Func_Camera_setUserDefinedName,                 "Camera_setUserDefinedName",
  NULL, Func_Camera_getUserDefinedName,                 "Camera_getUserDefinedName",
  NULL, Func_Camera_getPixelClock,                      "Camera_getPixelClock",
  NULL, Func_Camera_isCameraFeature,                    "Camera_isCameraFeature",
  NULL, Func_Camera_readXMLFile,                        "Camera_readXMLFile",
  NULL,Func_Camera_getSensorTemperature,                 "Camera_getSensorTemperature",

  // 4 - Stream: Channel creation and control
  NULL, Func_StreamingChannel_create,                   "StreamingChannel_create", 
  NULL, Func_StreamingChannel_delete,                   "StreamingChannel_delete",
  
  // 5 - Stream: Channel info
  NULL, Func_StreamingChannel_getPixelType,             "StreamingChannel_getPixelType",
  NULL, Func_StreamingChannel_getImagePitch,            "StreamingChannel_getImagePitch",
  NULL, Func_StreamingChannel_getImageSizeX,            "StreamingChannel_getImageSizeX",
  NULL, Func_StreamingChannel_getImageSizeY,            "StreamingChannel_getImageSizeY",

  // 6 - Stream: Image access
  NULL, Func_Image_getDataPointer,                      "Image_getDataPointer",
  NULL, Func_Image_getSignalType,                       "Image_getSignalType",
  NULL, Func_Image_getCamera,                           "Image_getCamera",
  NULL, Func_Image_release,                             "Image_release",

  // 7 - Stream: Image conversion
  NULL, Func_Image_getImageRGB,                         "Image_getImageRGB",
  NULL, Func_Image_getImage12bitAs8bit,                 "Image_getImage12bitAs8bit",
  NULL, Func_Image_getImage12bitAs16bit,                "Image_getImage12bitAs16bit",
  NULL, Func_Image_getImage16bitAs8bit,                 "Image_getImage16bitAs8bit",

  // 8 - Stream: Image characteristics
  NULL, Func_Image_getPixelType,                        "Image_getPixelType",
  NULL, Func_Image_getImageSize,                        "Image_getImageSize",
  NULL, Func_Image_getPitch,                            "Image_getPitch",
  NULL, Func_Image_getSizeX,                            "Image_getSizeX",
  NULL, Func_Image_getSizeY,                            "Image_getSizeY",

  // 9 - Stream: Image statistics
  NULL, Func_Image_getImageID,                          "Image_getImageID",
  NULL, Func_Image_getTimestamp,                        "Image_getTimestamp",

  // 10 - Controlling camera: Frame rate
  NULL, Func_Camera_setFrameRate,                       "Camera_setFrameRate",
  NULL, Func_Camera_getFrameRate,                       "Camera_getFrameRate",
  NULL, Func_Camera_getFrameRateMin,                    "Camera_getFrameRateMin",
  NULL, Func_Camera_getFrameRateMax,                    "Camera_getFrameRateMax",
  NULL, Func_Camera_getFrameRateRange,                  "Camera_getFrameRateRange",
  NULL, Func_Camera_getFrameRateIncrement,              "Camera_getFrameRateIncrement",

  // 11 - Controlling camera: Exposure
  NULL, Func_Camera_setExposureTime,                    "Camera_setExposureTime",
  NULL, Func_Camera_getExposureTime,                    "Camera_getExposureTime",
  NULL, Func_Camera_getExposureTimeMin,                 "Camera_getExposureTimeMin",
  NULL, Func_Camera_getExposureTimeMax,                 "Camera_getExposureTimeMax",
  NULL, Func_Camera_getExposureTimeRange,               "Camera_getExposureTimeRange",
  NULL, Func_Camera_getExposureTimeIncrement,           "Camera_getExposureTimeIncrement",
  NULL, Func_Camera_setExposureDelay,                   "Camera_setExposureDelay",
  NULL, Func_Camera_getExposureDelay,                   "Camera_getExposureDelay",
  NULL, Func_Camera_getExposureDelayMax,                "Camera_getExposureDelayMax",
  NULL, Func_Camera_getExposureDelayIncrement,          "Camera_getExposureDelayIncrement",

  // 12 - Controlling camera: Gain and offset
  NULL, Func_Camera_setGain,                            "Camera_setGain",
  NULL, Func_Camera_getGain,                            "Camera_getGain",

  NULL, Func_Camera_getGainMax,                         "Camera_getGainMax",
  NULL, Func_Camera_getGainMaxExtended,                 "Camera_getGainMaxExtended",
  NULL, Func_Camera_getGainIncrement,                   "Camera_getGainIncrement",
  NULL, Func_Camera_setOffset,                          "Camera_setOffset",
  NULL, Func_Camera_getOffset,                          "Camera_getOffset",
  NULL, Func_Camera_getOffsetMax,                       "Camera_getOffsetMax",

  // 13 - Controlling camera: Auto gain/exposure
  NULL, Func_Camera_setAutoGainEnabled,                 "Camera_setAutoGainEnabled",
  NULL, Func_Camera_getAutoGainEnabled,                 "Camera_getAutoGainEnabled",
  NULL, Func_Camera_setAutoGainBrightness,              "Camera_setAutoGainBrightness",
  NULL, Func_Camera_getAutoGainBrightness,              "Camera_getAutoGainBrightness",
  NULL, Func_Camera_setAutoGainDynamics,                "Camera_setAutoGainDynamics",
  NULL, Func_Camera_getAutoGainDynamics,                "Camera_getAutoGainDynamics",
  NULL, Func_Camera_setAutoGainLimits,                  "Camera_setAutoGainLimits",
  NULL, Func_Camera_getAutoGainLimits,                  "Camera_getAutoGainLimits",
  NULL, Func_Camera_setAutoExposureLimits,              "Camera_setAutoExposureLimits",
  NULL, Func_Camera_getAutoExposureLimits,              "Camera_getAutoExposureLimits",

  // 14 - Controlling camera: Acquisition trigger
  NULL, Func_Camera_setAcquisitionControl,              "Camera_setAcquisitionControl",
  NULL, Func_Camera_getAcquisitionControl,              "Camera_getAcquisitionControl",
  NULL, Func_Camera_setAcquisitionMode,                 "Camera_setAcquisitionMode",
 
  NULL, Func_Camera_getAcquisitionMode,                 "Camera_getAcquisitionMode",
  NULL, Func_Camera_softwareTrigger,                    "Camera_softwareTrigger",
  NULL, Func_Camera_setTriggerPolarity,                 "Camera_setTriggerPolarity",
  NULL, Func_Camera_getTriggerPolarity,                 "Camera_getTriggerPolarity",
  NULL, Func_Camera_setPivMode,                         "Camera_setPivMode",
  NULL, Func_Camera_getPivMode,                         "Camera_getPivMode",
  NULL, Func_Camera_setDebouncerDuration,               "Camera_setDebouncerDuration",
  NULL, Func_Camera_getDebouncerDuration,               "Camera_getDebouncerDuration",
  NULL,	Func_Camera_setPrescalerDevisor,                "Camera_setPrescalerDevisor",
  NULL,	Func_Camera_getPrescalerDevisor,                "Camera_getPrescalerDevisor",
  NULL,	Func_Camera_loadSequenceParameters,             "Camera_loadSequenceParameters",
  NULL, Func_Camera_startSequencer,                     "Camera_startSequencer",

  // 15 - Controlling camera: Strobe
  
  NULL, Func_Camera_setStrobePolarityExtended,          "Camera_setStrobePolarityExtended",
  NULL, Func_Camera_getStrobePolarityExtended,          "Camera_getStrobePolarityExtended",
  NULL, Func_Camera_setStrobePositionExtended,          "Camera_setStrobePositionExtended",
  NULL, Func_Camera_getStrobePositionExtended,          "Camera_getStrobePositionExtended",
  NULL, Func_Camera_getStrobePositionMax,               "Camera_getStrobePositionMax",
  NULL, Func_Camera_getStrobePositionIncrement,         "Camera_getStrobePositionIncrement",
  NULL, Func_Camera_setStrobeDurationExtended,          "Camera_setStrobeDurationExtended",
  NULL, Func_Camera_getStrobeDurationExtended,          "Camera_getStrobeDurationExtended",
  NULL, Func_Camera_getStrobeDurationMax,               "Camera_getStrobeDurationMax",
  NULL, Func_Camera_getStrobeDurationIncrement,         "Camera_getStrobeDurationIncrement",

  // 16 - Controlling camera: Tap balance
  NULL, Func_Camera_setTapConfigurationEx,              "Camera_setTapConfigurationEx",
  NULL, Func_Camera_getTapConfigurationEx,              "Camera_getTapConfigurationEx",
  NULL, Func_Camera_setAutoTapBalanceMode,              "Camera_setAutoTapBalanceMode",
  NULL, Func_Camera_getAutoTapBalanceMode,              "Camera_getAutoTapBalanceMode",
  NULL, Func_Camera_setTapGain,                         "Camera_setTapGain",
  NULL, Func_Camera_getTapGain,                         "Camera_getTapGain",

  // 17 - Controlling camera: Image parameter
  NULL, Func_Camera_getImagerWidth,                     "Camera_getImagerWidth",
  NULL, Func_Camera_getImagerHeight,                    "Camera_getImagerHeight",
  NULL, Func_Camera_getImageSize,                       "Camera_getImageSize",
  NULL, Func_Camera_getPitch,                           "Camera_getPitch",
  NULL, Func_Camera_getSizeX,                           "Camera_getSizeX",
  NULL, Func_Camera_getSizeY,                           "Camera_getSizeY",
  NULL, Func_Camera_setBinningMode,                     "Camera_setBinningMode",
  NULL, Func_Camera_getBinningMode,                     "Camera_getBinningMode",
  NULL, Func_Camera_setAreaOfInterest,                  "Camera_setAreaOfInterest",
  NULL, Func_Camera_getAreaOfInterest,                  "Camera_getAreaOfInterest",
  NULL, Func_Camera_getAreaOfInterestRange,             "Camera_getAreaOfInterestRange",
  NULL, Func_Camera_getAreaOfInterestIncrement,         "Camera_getAreaOfInterestIncrement",
  NULL, Func_Camera_setFlippingMode,	                "Camera_setFlippingMode",
  NULL, Func_Camera_getFlippingMode,					"Camera_getFlippingMode",
  NULL,	Func_Camera_setShutterMode,						"Camera_setShutterMode",
  NULL,	Func_Camera_getShutterMode,						"Camera_getShutterMode",
  
  // 18 - Controlling camera: Image appearance
  NULL, Func_Camera_getPixelType,                       "Camera_getPixelType",
  NULL, Func_Camera_setPixelDepth,                      "Camera_setPixelDepth",
  NULL, Func_Camera_getPixelDepth,                      "Camera_getPixelDepth",
  NULL, Func_Camera_setWhiteBalance,                    "Camera_setWhiteBalance",
  NULL, Func_Camera_getWhiteBalance,                    "Camera_getWhiteBalance",
  NULL, Func_Camera_getWhiteBalanceMax,                 "Camera_getWhiteBalanceMax",
  NULL, Func_Camera_setGammaCorrection,                 "Camera_setGammaCorrection",
  NULL, Func_Camera_setGammaCorrectionExt,              "Camera_setGammaCorrectionExt",
  NULL, Func_Camera_setLowpassFilter,                   "Camera_setLowpassFilter",
  NULL, Func_Camera_getLowpassFilter,                   "Camera_getLowpassFilter",
  NULL, Func_Camera_setLookupTableMode,                 "Camera_setLookupTableMode",
  NULL, Func_Camera_getLookupTableMode,                 "Camera_getLookupTableMode",
  NULL, Func_Camera_setLookupTable,                     "Camera_setLookupTable",
  NULL, Func_Camera_getLookupTable,                     "Camera_getLookupTable",
  NULL, Func_Camera_startImageCorrection,               "Camera_startImageCorrection",
  NULL, Func_Camera_isIdleImageCorrection,              "Camera_isIdleImageCorrection",
  NULL, Func_Camera_setImageCorrection,                 "Camera_setImageCorrection",
  NULL, Func_Camera_getImageCorrection,                 "Camera_getImageCorrection",
  NULL, Func_Camera_setPixelsCorrectionMap,				"Camera_setPixelsCorrectionMap",	
  NULL, Func_Camera_getPixelsCorrectionMap,				"Camera_getPixelsCorrectionMap",
  NULL, Func_Camera_setPixelsCorrectionControlEnabel,   "Camera_setPixelsCorrectionControlEnabel",
  NULL, Func_Camera_getPixelsCorrectionControlEnabel,	"Camera_getPixelsCorrectionControlEnabel",
  NULL, Func_Camera_setPixelsCorrectionControlMark,		"Camera_setPixelsCorrectionControlMark",
  NULL, Func_Camera_getPixelsCorrectionControlMark,		"Camera_getPixelsCorrectionControlMark",
  NULL, Func_Camera_setPixelsCorrectionMapOffset,		"Camera_setPixelsCorrectionMapOffset",
  NULL, Func_Camera_getPixelsCorrectionMapOffset,		"Camera_getPixelsCorrectionMapOffset",
  NULL, Func_Camera_getPixelsCorrectionMapSize,			"Camera_getPixelsCorrectionMapSize", 
  NULL, Func_Camera_getMaximalPixelsCorrectionMapSize,	"Camera_getMaximalPixelsCorrectionMapSize",
  NULL, Func_Camera_setMapIndexCoordinate,				"Camera_setMapIndexCoordinate",
  NULL, Func_Camera_getMapIndexCoordinate,				"Camera_getMapIndexCoordinate",
  NULL, Func_Camera_deletePixelCoordinateFromMap,		"Camera_deletePixelCoordinateFromMap",
 
  // 19 - Special control: IOMux configuration
  NULL, Func_Camera_getMaxIOMuxIN,                      "Camera_getMaxIOMuxIN",
  NULL, Func_Camera_getMaxIOMuxOUT,                     "Camera_getMaxIOMuxOUT",
  NULL, Func_Camera_setIOAssignment,                    "Camera_setIOAssignment",
  NULL, Func_Camera_getIOAssignment,                    "Camera_getIOAssignment",

  // 20 - Special control: IO control
  NULL, Func_Camera_setIOMuxIN,                         "Camera_setIOMuxIN",
  NULL, Func_Camera_getIOMuxIN,                         "Camera_getIOMuxIN",
  NULL, Func_Camera_setIO,                              "Camera_setIO",
  NULL, Func_Camera_getIO,                              "Camera_getIO",
  NULL, Func_Camera_setAcqLEDOverride,                  "Camera_setAcqLEDOverride",
  NULL, Func_Camera_getAcqLEDOverride,                  "Camera_getAcqLEDOverride",
  NULL, Func_Camera_setLEDIntensity,                    "Camera_setLEDIntensity",
  NULL, Func_Camera_getLEDIntensity,                    "Camera_getLEDIntensity",

  // 21 - Special control: Serial communication
  NULL, Func_Camera_setUARTBuffer,                      "Camera_setUARTBuffer",
  NULL, Func_Camera_getUARTBuffer,                      "Camera_getUARTBuffer",
  NULL, Func_Camera_setUARTBaud,                        "Camera_setUARTBaud",
  NULL, Func_Camera_getUARTBaud,                        "Camera_getUARTBaud",

  // 22 - Special control: Direct register and memory access
  NULL, Func_Camera_setU3VCameraRegister,              "Camera_setU3VCameraRegister",
  NULL, Func_Camera_getU3VCameraRegister,              "Camera_getU3VCameraRegister",
  NULL, Func_Camera_writeU3VCameraMemory,              "Camera_writeU3VCameraMemory",
  NULL, Func_Camera_readU3VCameraMemory,               "Camera_readU3VCameraMemory",


  // 23 - Special control: Persistent settings and recovery
  NULL, Func_Camera_writeEEPROM,                        "Camera_writeEEPROM",
  NULL, Func_Camera_readEEPROM,                         "Camera_readEEPROM",
  NULL, Func_Camera_restoreFactoryDefaults,             "Camera_restoreFactoryDefaults",
  NULL, Func_Camera_loadSettingsFromXml,                "Camera_loadSettingsFromXml",
  NULL, Func_Camera_SaveSettingsToXml,                  "Camera_SaveSettingsToXml",

  // 24 - General functions
  NULL, Func_SVU3V_estimateWhiteBalance,               "SVU3V_estimateWhiteBalance",
  NULL, Func_SVU3V_estimateWhiteBalanceExtended,       "SVU3V_estimateWhiteBalanceExtended", 
  NULL, Func_SVU3V_writeImageToBitmapFile,             "SVU3V_writeImageToBitmapFile",
  
  // 25 - Special control: Lens conttrol
  NULL, Func_Camera_isLensAvailable,					"Camera_isLensAvailable",
  NULL, Func_Camera_getLensName,					"Camera_getLensName",
  
  NULL, Func_Camera_setLensFocalLenght,					"Camera_setLensFocalLenght",
  NULL, Func_Camera_getLensFocalLenght,					"Camera_getLensFocalLenght",
  NULL, Func_Camera_getLensFocalLenghtMin,				"Camera_getLensFocalLenghtMin",
  NULL, Func_Camera_getLensFocalLenghtMax,				"Camera_getLensFocalLenghtMax",
 
  NULL, Func_Camera_setLensFocusUnit,					"Camera_setLensFocusUnit",
  NULL, Func_Camera_getLensFocusUnit,					"Camera_getLensFocusUnit",
  NULL, Func_Camera_setLensFocus,					"Camera_setLensFocus",
  NULL, Func_Camera_getLensFocus,					"Camera_getLensFocus",
  NULL, Func_Camera_getLensFocusMin,					"Camera_getLensFocusMin",
  NULL, Func_Camera_getLensFocusMax,					"Camera_getLensFocusMax",
  
  
  NULL, Func_Camera_setLensAperture,					"Camera_setLensAperture",
  NULL, Func_Camera_getLensAperture,					"Camera_getLensAperture",
  NULL, Func_Camera_getLensApertureMin,					"Camera_getLensApertureMin",
  NULL, Func_Camera_getLensApertureMax,					"Camera_getLensApertureMax",
  
  
  // 00 - Consistency check
  //
  // The following function pointer will be used to
  // check whether consistency of the whole function
  // table is OK.
  //
  NULL, Func_isVersionCompliantDLL_consistency_check,   "isVersionCompliantDLL",
};

/**
 *  Specify function types to be used for casting function pointers that are retrieved from the DLL
 */

//-----------------------------------------------------------------------------------------
// 0 - SVU3V DLL (also implicitly called)
//-----------------------------------------------------------------------------------------

typedef SVU3V_RETURN
(*TFunc_isVersionCompliantDLL)(SVU3V_VERSION *DllVersion, 
                               SVU3V_VERSION *ExpectedVersion);
							   

//-----------------------------------------------------------------------------------------
// 1 - Camera: Discovery and bookkeeping
//-----------------------------------------------------------------------------------------

typedef CameraContainerClient_handle
(*TFunc_CameraContainer_create)();

typedef SVU3V_RETURN
(*TFunc_CameraContainer_delete)(CameraContainerClient_handle hCameraContainer);

typedef SVU3V_RETURN
(*TFunc_CameraContainer_discovery)(CameraContainerClient_handle hCameraContainer);

typedef int
(*TFunc_CameraContainer_getNumberOfCameras)(CameraContainerClient_handle hCameraContainer);

typedef  Camera_handle
(*TFunc_CameraContainer_getCamera)(CameraContainerClient_handle hCameraContainer,
                                   int CameraIndex);

typedef  Camera_handle
(*TFunc_CameraContainer_findCamera)(CameraContainerClient_handle hCameraContainer,
                                    char *CameraName);

//-----------------------------------------------------------------------------------------
// 2 - Camera: Connection
//-----------------------------------------------------------------------------------------

typedef SVU3V_RETURN
(*TFunc_Camera_openConnection)(Camera_handle hCamera, float Timeout);

typedef SVU3V_RETURN
(*TFunc_Camera_closeConnection)(Camera_handle hCamera);


//-----------------------------------------------------------------------------------------
// 3 - Camera: Information
//-----------------------------------------------------------------------------------------

typedef char *
(*TFunc_Camera_getManufacturerName)(Camera_handle hCamera);

typedef char *
(*TFunc_Camera_getModelName)(Camera_handle hCamera);

typedef char *
(*TFunc_Camera_getDeviceVersion)(Camera_handle hCamera);

typedef char *
(*TFunc_Camera_getManufacturerSpecificInformation)(Camera_handle hCamera);

typedef char *
(*TFunc_Camera_getSerialNumber)(Camera_handle hCamera);

typedef SVU3V_RETURN
(*TFunc_Camera_setUserDefinedName)(Camera_handle hCamera, char *UserDefinedName);

typedef char *
(*TFunc_Camera_getUserDefinedName)(Camera_handle hCamera);


typedef SVU3V_RETURN
(*TFunc_Camera_getPixelClock)(Camera_handle hCamera,
                             int *PixelClock);

typedef bool
(*TFunc_Camera_isCameraFeature)(Camera_handle hCamera,
                                CAMERA_FEATURE Feature);

typedef SVU3V_RETURN
(*TFunc_Camera_readXMLFile)(Camera_handle hCamera,
                            char **XML);


typedef SVU3V_RETURN
(*TFunc_Camera_getSensorTemperature)(Camera_handle hCamera,
                                     unsigned int *SensorTemperature);


//-----------------------------------------------------------------------------------------
// 4 - Stream: Channel creation and control
//-----------------------------------------------------------------------------------------

typedef SVU3V_RETURN
(*TFunc_StreamingChannel_create)(StreamingChannel_handle *hStreamingChannel,
                                 CameraContainerClient_handle hCameraContainer,
                                 Camera_handle hCamera,
                                 int BufferCount,
                                 StreamCallback CallbackFunction,
                                 void *Context);


typedef SVU3V_RETURN
(*TFunc_StreamingChannel_delete)(StreamingChannel_handle hStreamingChannel);



//-----------------------------------------------------------------------------------------
// 5 - Stream: Channel info
//-----------------------------------------------------------------------------------------

typedef SVU3V_RETURN
(*TFunc_StreamingChannel_getPixelType)(StreamingChannel_handle hStreamingChannel,
                                       GVSP_PIXEL_TYPE *ProgrammedPixelType);




typedef SVU3V_RETURN
(*TFunc_StreamingChannel_getImagePitch)(StreamingChannel_handle hStreamingChannel,
                                        int *ImagePitch);

typedef SVU3V_RETURN
(*TFunc_StreamingChannel_getImageSizeX)(StreamingChannel_handle hStreamingChannel,
                                        int *ImageSizeX);

typedef SVU3V_RETURN
(*TFunc_StreamingChannel_getImageSizeY)(StreamingChannel_handle hStreamingChannel,
                                        int *ImageSizeY);


//-----------------------------------------------------------------------------------------
// 6 - Stream: Image access
//-----------------------------------------------------------------------------------------

typedef unsigned char *
(*TFunc_Image_getDataPointer)(Image_handle hImage);


typedef SVU3V_SIGNAL_TYPE
(*TFunc_Image_getSignalType)(Image_handle hImage);

typedef Camera_handle
(*TFunc_Image_getCamera)(Image_handle hImage);

typedef SVU3V_RETURN
(*TFunc_Image_release)(Image_handle hImage);

//-----------------------------------------------------------------------------------------
// 7 - Stream: Image conversion
//-----------------------------------------------------------------------------------------

typedef SVU3V_RETURN
(*TFunc_Image_getImageRGB)(Image_handle hImage,
                           unsigned char *BufferRGB,
                           int BufferLength,
                           BAYER_METHOD BayerMethod);

typedef SVU3V_RETURN
(*TFunc_Image_getImage12bitAs8bit)(Image_handle hImage,
                                   unsigned char *Buffer8bit,
                                   int BufferLength);

typedef SVU3V_RETURN
(*TFunc_Image_getImage12bitAs16bit)(Image_handle hImage,
                                    unsigned char *Buffer16bit,
                                    int BufferLength);

typedef SVU3V_RETURN
(*TFunc_Image_getImage16bitAs8bit)(Image_handle hImage,
                                   unsigned char *Buffer8bit,
                                   int BufferLength);

//-----------------------------------------------------------------------------------------
// 8 - Stream: Image characteristics
//-----------------------------------------------------------------------------------------

typedef GVSP_PIXEL_TYPE
(*TFunc_Image_getPixelType)(Image_handle hImage);

typedef int
(*TFunc_Image_getImageSize)(Image_handle hImage);

typedef	int
(*TFunc_Image_getPitch)(Image_handle hImage);

typedef int
(*TFunc_Image_getSizeX)(Image_handle hImage);

typedef	int
(*TFunc_Image_getSizeY)(Image_handle hImage);

//-----------------------------------------------------------------------------------------
// 9 - Stream: Image statistics
//-----------------------------------------------------------------------------------------

typedef int
(*TFunc_Image_getImageID)(Image_handle hImage);

typedef double
(*TFunc_Image_getTimestamp)(Image_handle hImage);


//-----------------------------------------------------------------------------------------
// 10 - Controlling camera: Frame rate
//-----------------------------------------------------------------------------------------

typedef SVU3V_RETURN
(*TFunc_Camera_setFrameRate)(Camera_handle hCamera,
                             float Framerate);

typedef SVU3V_RETURN
(*TFunc_Camera_getFrameRate)(Camera_handle hCamera,
                             float *ProgrammedFramerate);

typedef SVU3V_RETURN
(*TFunc_Camera_getFrameRateMin)(Camera_handle hCamera,
                                float *MinFramerate);

typedef SVU3V_RETURN
(*TFunc_Camera_getFrameRateMax)(Camera_handle hCamera,
                                float *MaxFramerate);

typedef SVU3V_RETURN
(*TFunc_Camera_getFrameRateRange)(Camera_handle hCamera,
                                  float *MinFramerate,
                                  float *MaxFramerate);

typedef SVU3V_RETURN
(*TFunc_Camera_getFrameRateIncrement)(Camera_handle hCamera,
                                      float *FramerateIncrement);

//-----------------------------------------------------------------------------------------
// 11 - Controlling camera: Exposure
//-----------------------------------------------------------------------------------------

typedef SVU3V_RETURN
(*TFunc_Camera_setExposureTime)(Camera_handle hCamera,
                                float ExposureTime);

typedef SVU3V_RETURN
(*TFunc_Camera_getExposureTime)(Camera_handle hCamera,
                                float *ProgrammedExposureTime);

typedef SVU3V_RETURN
(*TFunc_Camera_getExposureTimeMin)(Camera_handle hCamera,
                                   float *MinExposureTime);

typedef SVU3V_RETURN
(*TFunc_Camera_getExposureTimeMax)(Camera_handle hCamera,
                                   float *MaxExposureTime);

typedef SVU3V_RETURN
(*TFunc_Camera_getExposureTimeRange)(Camera_handle hCamera,
                                     float *MinExposureTime,
                                     float *MaxExposureTime);

typedef SVU3V_RETURN
(*TFunc_Camera_getExposureTimeIncrement)(Camera_handle hCamera,
                                         float *ExposureTimeIncrement);

typedef SVU3V_RETURN
(*TFunc_Camera_setExposureDelay)(Camera_handle hCamera,
                                 float ExposureDelay);

typedef SVU3V_RETURN
(*TFunc_Camera_getExposureDelay)(Camera_handle hCamera,
                                 float *ProgrammedExposureDelay);

typedef SVU3V_RETURN
(*TFunc_Camera_getExposureDelayMax)(Camera_handle hCamera,
                                    float *MaxExposureDelay);

typedef SVU3V_RETURN
(*TFunc_Camera_getExposureDelayIncrement)(Camera_handle hCamera,
                                          float *ExposureDelayIncrement);

//-----------------------------------------------------------------------------------------
// 12 - Controlling camera: Gain and offset
//-----------------------------------------------------------------------------------------

typedef SVU3V_RETURN
(*TFunc_Camera_setGain)(Camera_handle hCamera,
                        float Gain);

typedef SVU3V_RETURN
(*TFunc_Camera_getGain)(Camera_handle hCamera,
                        float *ProgrammedGain);


typedef SVU3V_RETURN
(*TFunc_Camera_getGainMax)(Camera_handle hCamera,
                                   float *MaxGain);

typedef SVU3V_RETURN
(*TFunc_Camera_getGainMaxExtended)(Camera_handle hCamera,
                                   float *MaxGainExtended);

typedef SVU3V_RETURN
(*TFunc_Camera_getGainIncrement)(Camera_handle hCamera,
                                 float *GainIncrement);

typedef SVU3V_RETURN
(*TFunc_Camera_setOffset)(Camera_handle hCamera,
                         float Offset);

typedef SVU3V_RETURN
(*TFunc_Camera_getOffset)(Camera_handle hCamera,
                          float *Offset);

typedef SVU3V_RETURN
(*TFunc_Camera_getOffsetMax)(Camera_handle hCamera,
                             float *MaxOffset);

//-----------------------------------------------------------------------------------------
// 13 - Controlling camera: Auto gain/exposure
//-----------------------------------------------------------------------------------------

typedef SVU3V_RETURN
(*TFunc_Camera_setAutoGainEnabled)(Camera_handle Camera,
														       bool isAutoGainEnabled);

typedef SVU3V_RETURN
(*TFunc_Camera_getAutoGainEnabled)(Camera_handle Camera,
														       bool *isAutoGainEnabled);

typedef SVU3V_RETURN
(*TFunc_Camera_setAutoGainBrightness)(Camera_handle Camera,
                                      float Brightness);

typedef SVU3V_RETURN
(*TFunc_Camera_getAutoGainBrightness)(Camera_handle Camera,
                                      float *ProgrammedBrightness);

typedef SVU3V_RETURN
(*TFunc_Camera_setAutoGainDynamics)(Camera_handle Camera,
                                    float AutoGainParameterI,
                                    float AutoGainParameterD);

typedef SVU3V_RETURN
(*TFunc_Camera_getAutoGainDynamics)(Camera_handle Camera,
                                    float *ProgrammedAutoGainParameterI,
                                    float *ProgrammedAutoGainParameterD);

typedef SVU3V_RETURN
(*TFunc_Camera_setAutoGainLimits)(Camera_handle Camera,
                                  float MinGain,
                                  float MaxGain);

typedef SVU3V_RETURN
(*TFunc_Camera_getAutoGainLimits)(Camera_handle Camera,
                                  float *ProgrammedMinGain,
                                  float *ProgrammedMaxGain);

typedef SVU3V_RETURN
(*TFunc_Camera_setAutoExposureLimits)(Camera_handle Camera,
                                      float MinExposure,
                                      float MaxExposure);

typedef SVU3V_RETURN
(*TFunc_Camera_getAutoExposureLimits)(Camera_handle Camera,
                                      float *ProgrammedMinExposure,
                                      float *ProgrammedMaxExposure);

//-----------------------------------------------------------------------------------------
// 14 - Controlling camera: Acquisition trigger
//-----------------------------------------------------------------------------------------

typedef SVU3V_RETURN
(*TFunc_Camera_setAcquisitionControl)(Camera_handle hCamera,
                                      ACQUISITION_CONTROL AcquisitionControl);

typedef SVU3V_RETURN
(*TFunc_Camera_getAcquisitionControl)(Camera_handle hCamera,
                                      ACQUISITION_CONTROL *ProgrammedAcquisitionControl);

typedef SVU3V_RETURN
(*TFunc_Camera_setAcquisitionMode)(Camera_handle hCamera,
                                  ACQUISITION_MODE AcquisitionMode);


typedef SVU3V_RETURN
(*TFunc_Camera_getAcquisitionMode)(Camera_handle hCamera,
                                   ACQUISITION_MODE *ProgrammedAcquisitionMode);

typedef SVU3V_RETURN
(*TFunc_Camera_softwareTrigger)(Camera_handle hCamera);


typedef SVU3V_RETURN
(*TFunc_Camera_setTriggerPolarity)(Camera_handle hCamera,
                                   TRIGGER_POLARITY TriggerPolarity);

typedef SVU3V_RETURN
(*TFunc_Camera_getTriggerPolarity)(Camera_handle hCamera,
                                   TRIGGER_POLARITY *ProgrammedTriggerPolarity);
								   
//--------------Piv Mode-------------------
typedef SVU3V_RETURN
(*TFunc_Camera_setPivMode)(Camera_handle hCamera,
                           PIV_MODE  SelectPivMode);

typedef SVU3V_RETURN
(*TFunc_Camera_getPivMode)(Camera_handle hCamera,
                           PIV_MODE *ProgrammedPivMode);

//--------------Debouncer-------------------
typedef SVU3V_RETURN
(*TFunc_Camera_setDebouncerDuration)(Camera_handle hCamera,
                                     float  DebouncerDuration); 

typedef SVU3V_RETURN
(*TFunc_Camera_getDebouncerDuration)(Camera_handle hCamera,
                                     float *ProgrammedDuration);

//--------------prescaler-------------------
typedef SVU3V_RETURN
(*TFunc_Camera_setPrescalerDevisor)(Camera_handle hCamera,
                                    unsigned int  PrescalerDevisor); 

typedef SVU3V_RETURN
(*TFunc_Camera_getPrescalerDevisor)(Camera_handle hCamera,
                                    unsigned int *ProgrammedPrescalerDevisor);

//--------------Sequencer-------------------
typedef SVU3V_RETURN
(*TFunc_Camera_loadSequenceParameters)(Camera_handle hCamera,
                                       char *Filename);

typedef SVU3V_RETURN
(*TFunc_Camera_startSequencer)(Camera_handle hCamera);


//-----------------------------------------------------------------------------------------
// 15 - Controlling camera: Strobe
//-----------------------------------------------------------------------------------------


typedef SVU3V_RETURN
(*TFunc_Camera_setStrobePolarityExtended)(Camera_handle hCamera,
                                  STROBE_POLARITY StrobePolarity, int StrobeIndex);


typedef SVU3V_RETURN
(*TFunc_Camera_getStrobePolarityExtended)(Camera_handle hCamera,
                                  STROBE_POLARITY *ProgrammedStrobePolarity, int StrobeIndex );


typedef SVU3V_RETURN
(*TFunc_Camera_setStrobePositionExtended)(Camera_handle hCamera,
                                  float StrobePosition, int StrobeIndex);


typedef SVU3V_RETURN
(*TFunc_Camera_getStrobePositionExtended)(Camera_handle hCamera,
                                  float *ProgrammedStrobePosition,int StrobeIndex );

typedef SVU3V_RETURN
(*TFunc_Camera_getStrobePositionMax)(Camera_handle hCamera,
                                     float *MaxStrobePosition);

typedef SVU3V_RETURN
(*TFunc_Camera_getStrobePositionIncrement)(Camera_handle hCamera,
                                           float *StrobePositionIncrement);


typedef SVU3V_RETURN
(*TFunc_Camera_setStrobeDurationExtended)(Camera_handle hCamera,
                                  float StrobeDuration,int StrobeIndex);


typedef SVU3V_RETURN
(*TFunc_Camera_getStrobeDurationExtended)(Camera_handle hCamera,
                                          float *ProgrammedStrobeDuration,int StrobeIndex);

typedef SVU3V_RETURN
(*TFunc_Camera_getStrobeDurationMax)(Camera_handle hCamera,
                                     float *MaxStrobeDuration);

typedef SVU3V_RETURN
(*TFunc_Camera_getStrobeDurationIncrement)(Camera_handle hCamera,
                                           float *StrobeDurationIncrement);

//-----------------------------------------------------------------------------------------
// 16 - Controlling camera: Tap balance
//-----------------------------------------------------------------------------------------


typedef SVU3V_RETURN
(*TFunc_Camera_setTapConfigurationEx)(Camera_handle hCamera,
                                    SVU3V_TAP_CONFIGURATION_SELECT SelectedTapConfig);

typedef SVU3V_RETURN 
(*TFunc_Camera_getTapConfigurationEx)(Camera_handle hCamera,
                                    SVU3V_TAP_CONFIGURATION_SELECT *ProgrammedTapConfig);


typedef SVU3V_RETURN
(*TFunc_Camera_setAutoTapBalanceMode)(Camera_handle hCamera,
                                      SVU3V_AUTO_TAP_BALANCE_MODE AutoTapBalanceMode);

typedef SVU3V_RETURN
(*TFunc_Camera_getAutoTapBalanceMode)(Camera_handle hCamera,
                                      SVU3V_AUTO_TAP_BALANCE_MODE *AutoTapBalanceMode);


typedef SVU3V_RETURN
(*TFunc_Camera_setTapGain)(Camera_handle hCamera,
													 float TapGain,
 													 SVU3V_TAP_SELECT TapSelect);

typedef SVU3V_RETURN
(*TFunc_Camera_getTapGain)(Camera_handle hCamera,
													 float *TapGain,
													 SVU3V_TAP_SELECT TapSelect);

//-----------------------------------------------------------------------------------------
// 17 - Controlling camera: Image parameter
//-----------------------------------------------------------------------------------------

typedef SVU3V_RETURN
(*TFunc_Camera_getImagerWidth)(Camera_handle hCamera,
                               int *ImagerWidth);

typedef SVU3V_RETURN
(*TFunc_Camera_getImagerHeight)(Camera_handle hCamera,
                                int *ImagerHeight);

typedef SVU3V_RETURN
(*TFunc_Camera_getImageSize)(Camera_handle hCamera,
                             int *ImageSize);

typedef SVU3V_RETURN
(*TFunc_Camera_getPitch)(Camera_handle hCamera,
                         int *Pitch);

typedef SVU3V_RETURN
(*TFunc_Camera_getSizeX)(Camera_handle hCamera,
                         int *SizeX);

typedef SVU3V_RETURN
(*TFunc_Camera_getSizeY)(Camera_handle hCamera,
                         int *SizeY);

typedef SVU3V_RETURN
(*TFunc_Camera_setBinningMode)(Camera_handle hCamera,
                               BINNING_MODE BinningMode);

typedef SVU3V_RETURN
(*TFunc_Camera_getBinningMode)(Camera_handle hCamera,
                              BINNING_MODE *ProgrammedBinningMode);

typedef SVU3V_RETURN
(*TFunc_Camera_setAreaOfInterest)(Camera_handle hCamera,
                                  int SizeX,
                                  int SizeY,
                                  int OffsetX,
                                  int OffsetY);

typedef SVU3V_RETURN
(*TFunc_Camera_getAreaOfInterest)(Camera_handle hCamera,
                                  int *ProgrammedSizeX,
                                  int *ProgrammedSizeY,
                                  int *ProgrammedOffsetX,
                                  int *ProgrammedOffsetY);

typedef SVU3V_RETURN
(*TFunc_Camera_getAreaOfInterestRange)(Camera_handle hCamera,
                                       int *MinSizeX,
                                       int *MinSizeY,
                                       int *MaxSizeX,
                                       int *MaxSizeY);

typedef SVU3V_RETURN
(*TFunc_Camera_getAreaOfInterestIncrement)(Camera_handle hCamera,
                                           int *SizeXIncrement,
                                           int *SizeYIncrement,
                                           int *OffsetXIncrement,
                                           int *OffsetYIncrement);


typedef SVU3V_RETURN
(*TFunc_Camera_setFlippingMode)(Camera_handle hCamera,
                        SVU3V_FLIPPING_MODE  FlippingMode);

 typedef SVU3V_RETURN
(*TFunc_Camera_getFlippingMode)(Camera_handle hCamera,
                        SVU3V_FLIPPING_MODE *ProgrammedFlippingMode);	

  typedef SVU3V_RETURN	
(*TFunc_Camera_setShutterMode)(Camera_handle hCamera,
                        SVU3V_SHUTTER_MODE  ShutterMode);

   typedef SVU3V_RETURN 
(*TFunc_Camera_getShutterMode)(Camera_handle hCamera,
                        SVU3V_SHUTTER_MODE *ProgrammedShutterMode);
//-----------------------------------------------------------------------------------------
// 18 - Controlling camera: Image appearance
//-----------------------------------------------------------------------------------------

typedef SVU3V_RETURN
(*TFunc_Camera_getPixelType)(Camera_handle hCamera,
                             GVSP_PIXEL_TYPE *PixelType);

typedef SVU3V_RETURN
(*TFunc_Camera_setPixelDepth)(Camera_handle hCamera,
                              SVU3V_PIXEL_DEPTH PixelDepth);

typedef SVU3V_RETURN
(*TFunc_Camera_getPixelDepth)(Camera_handle hCamera,
                              SVU3V_PIXEL_DEPTH *PixelDepth);

typedef SVU3V_RETURN
(*TFunc_Camera_setWhiteBalance)(Camera_handle hCamera,
                                float Red,
                                float Green ,
                                float Blue);

typedef SVU3V_RETURN
(*TFunc_Camera_getWhiteBalance)(Camera_handle hCamera,
                                float *Red,
                                float *Green ,
                                float *Blue);

typedef SVU3V_RETURN
(*TFunc_Camera_getWhiteBalanceMax)(Camera_handle hCamera,
                                   float *WhiteBalanceMax);

typedef SVU3V_RETURN
(*TFunc_Camera_setGammaCorrection)(Camera_handle hCamera,
  								float GammaCorrection);

typedef SVU3V_RETURN
(*TFunc_Camera_setGammaCorrectionExt)(Camera_handle hCamera,
  									  float GammaCorrection,
                                      float DigitalGain,
                                      float DigitalOffset);

typedef SVU3V_RETURN
(*TFunc_Camera_setLowpassFilter)(Camera_handle hCamera,
  													     LOWPASS_FILTER LowpassFilter);

typedef SVU3V_RETURN
(*TFunc_Camera_getLowpassFilter)(Camera_handle hCamera,
  													     LOWPASS_FILTER *ProgrammedLowpassFilter);

typedef SVU3V_RETURN
(*TFunc_Camera_setLookupTableMode)(Camera_handle hCamera,
                                   LUT_MODE LUTMode);

typedef SVU3V_RETURN
(*TFunc_Camera_getLookupTableMode)(Camera_handle hCamera,
                                   LUT_MODE *ProgrammedLUTMode);

typedef SVU3V_RETURN
(*TFunc_Camera_setLookupTable)(Camera_handle hCamera,
                               unsigned char *LookupTable,
                               int LookupTableSize);

typedef SVU3V_RETURN
(*TFunc_Camera_getLookupTable)(Camera_handle hCamera,
                               unsigned char *LookupTable,
                               int LookupTableSize);

typedef SVU3V_RETURN
(*TFunc_Camera_startImageCorrection)(Camera_handle hCamera,
									IMAGE_CORRECTION_STEP ImageCorrectionStep);

typedef SVU3V_RETURN
(*TFunc_Camera_isIdleImageCorrection)(Camera_handle hCamera,
									 IMAGE_CORRECTION_STEP *ProgrammedImageCorrectionStep,
			 						 bool *isIdle);

typedef SVU3V_RETURN
(*TFunc_Camera_setImageCorrection)(Camera_handle hCamera,
								IMAGE_CORRECTION_MODE ImageCorrectionMode);

typedef SVU3V_RETURN
(*TFunc_Camera_getImageCorrection)(Camera_handle hCamera,
								 IMAGE_CORRECTION_MODE *ProgrammedImageCorrectionMode);


typedef SVU3V_RETURN
(*TFunc_Camera_setPixelsCorrectionMap)(Camera_handle hCamera,
								  PIXELS_CORRECTION_MAP_SELECT PixelsCorrectionMap);

 typedef SVU3V_RETURN
(*TFunc_Camera_getPixelsCorrectionMap)(Camera_handle hCamera,
								PIXELS_CORRECTION_MAP_SELECT * ProgrammedPixelsCorrectionMap);




typedef SVU3V_RETURN
(*TFunc_Camera_setPixelsCorrectionControlEnabel)(Camera_handle hCamera,
									bool isPixelsCorrectionEnabled);

 
	
typedef SVU3V_RETURN
(*TFunc_Camera_getPixelsCorrectionControlEnabel)(Camera_handle hCamera,
									bool *isPixelsCorrectionEnabled);

	 
 typedef SVU3V_RETURN
(*TFunc_Camera_setPixelsCorrectionControlMark)(Camera_handle hCamera,
									bool isPixelsCorrectionMarked);


	
typedef SVU3V_RETURN
(*TFunc_Camera_getPixelsCorrectionControlMark)(Camera_handle hCamera,
									bool *isPixelsCorrectionMarked);


typedef SVU3V_RETURN
(*TFunc_Camera_setPixelsCorrectionMapOffset)(Camera_handle hCamera,
										 int  OffsetX,   int  OffsetY);


typedef SVU3V_RETURN
(*TFunc_Camera_getPixelsCorrectionMapOffset)(Camera_handle hCamera,
									  int *ProgrammedOffsetX,  int *ProgrammedOffsetY);
									 
  
 typedef SVU3V_RETURN
(*TFunc_Camera_getPixelsCorrectionMapSize)(Camera_handle hCamera,
									 unsigned int *programmedMapSize);

  
 typedef SVU3V_RETURN
(*TFunc_Camera_getMaximalPixelsCorrectionMapSize)(Camera_handle hCamera,
											 unsigned int *MaximalprogrammedMapSize);
 

 typedef SVU3V_RETURN
(*TFunc_Camera_setMapIndexCoordinate)(Camera_handle hCamera,
									unsigned int MapIndex, 
									unsigned int CoordinateX, unsigned int CoordinateY );
									

 typedef SVU3V_RETURN
(*TFunc_Camera_getMapIndexCoordinate)(Camera_handle hCamera,
									unsigned int MapIndex, 
								unsigned int *ProgrammedCoordinateX, unsigned int *ProgrammedCoordinateY );


 typedef SVU3V_RETURN
(*TFunc_Camera_deletePixelCoordinateFromMap)(Camera_handle hCamera, unsigned int MapIndex);	

//-----------------------------------------------------------------------------------------
// 19 - Special control: IOMux configuration
//-----------------------------------------------------------------------------------------

typedef SVU3V_RETURN
(*TFunc_Camera_getMaxIOMuxIN)(Camera_handle hCamera,
                              int *MaxIOMuxINSignals);

typedef SVU3V_RETURN
(*TFunc_Camera_getMaxIOMuxOUT)(Camera_handle hCamera,
                               int *MaxIOMuxOUTSignals);

typedef SVU3V_RETURN
(*TFunc_Camera_setIOAssignment)(Camera_handle hCamera,
                                SVU3V_IOMux_OUT IOMuxOUT,
                                unsigned int SignalIOMuxIN);

typedef SVU3V_RETURN
(*TFunc_Camera_getIOAssignment)(Camera_handle hCamera,
                                SVU3V_IOMux_OUT IOMuxOUT,
                                unsigned int *ProgrammedIOMuxIN);

//-----------------------------------------------------------------------------------------
// 20 - Special control: IO control
//-----------------------------------------------------------------------------------------

typedef SVU3V_RETURN
(*TFunc_Camera_setIOMuxIN)(Camera_handle hCamera,
                           unsigned int VectorIOMuxIN);

typedef SVU3V_RETURN
(*TFunc_Camera_getIOMuxIN)(Camera_handle hCamera,
                           unsigned int *ProgrammedVectorIOMuxIN);

typedef SVU3V_RETURN
(*TFunc_Camera_setIO)(Camera_handle hCamera,
                      SVU3V_IOMux_IN SignalIOMuxIN,
                      IO_SIGNAL SignalValue);

typedef SVU3V_RETURN
(*TFunc_Camera_getIO)(Camera_handle hCamera,
                      SVU3V_IOMux_IN SignalIOMuxIN,
                      IO_SIGNAL *ProgrammedSignalValue);

typedef SVU3V_RETURN
(*TFunc_Camera_setAcqLEDOverride)(Camera_handle hCamera,
                                  bool isOverrideActive);

typedef SVU3V_RETURN
(*TFunc_Camera_getAcqLEDOverride)(Camera_handle hCamera,
                                  bool *isOverrideActive);

typedef SVU3V_RETURN
(*TFunc_Camera_setLEDIntensity)(Camera_handle hCamera,
                                int LEDIntensity);

typedef SVU3V_RETURN
(*TFunc_Camera_getLEDIntensity)(Camera_handle hCamera,
                                int *ProgrammedLEDIntensity);

//-----------------------------------------------------------------------------------------
// 21 - Special control: Serial communication
//-----------------------------------------------------------------------------------------

typedef SVU3V_RETURN
(*TFunc_Camera_setUARTBuffer)(Camera_handle hCamera,
                              unsigned char *Data,
                              int DataLength);

typedef SVU3V_RETURN
(*TFunc_Camera_getUARTBuffer)(Camera_handle hCamera,
                              unsigned char *Data,
                              int *DataLengthReceived,
                              int DataLengthMax,
                              float Timeout);

typedef SVU3V_RETURN
(*TFunc_Camera_setUARTBaud)(Camera_handle hCamera,
                            SVU3V_BaudRate BaudRate);

typedef SVU3V_RETURN
(*TFunc_Camera_getUARTBaud)(Camera_handle hCamera,
                            SVU3V_BaudRate *ProgrammedBaudRate);

//-----------------------------------------------------------------------------------------
// 22 - Special control: Direct register and memory access
//-----------------------------------------------------------------------------------------

typedef SVU3V_RETURN
(*TFunc_Camera_setU3VCameraRegister)(Camera_handle hCamera,
                                      unsigned int RegisterAddress,
                                      unsigned int RegisterValue,
                                      unsigned int SVU3VCameraAccessKey);

typedef SVU3V_RETURN
(*TFunc_Camera_getU3VCameraRegister)(Camera_handle hCamera,
                                      unsigned int RegisterAddress,
                                      unsigned int *ProgrammedRegisterValue,
                                      unsigned int SVU3VCameraAccessKey);

typedef SVU3V_RETURN
(*TFunc_Camera_writeSVU3VCameraMemory)(Camera_handle hCamera,
                                      unsigned int  MemoryAddress,
                                      unsigned char *DataBlock,
                                      unsigned int  DataLength,
                                      unsigned int  SVU3VCameraAccessKey);

typedef SVU3V_RETURN
(*TFunc_Camera_readSVU3VCameraMemory)(Camera_handle hCamera,
                                     unsigned int  MemoryAddress,
                                     unsigned char *DataBlock,
                                     unsigned int  DataLength,
                                     unsigned int  SVU3VCameraAccessKey);



//-----------------------------------------------------------------------------------------
// 23 - Special control: Persistent settings and recovery
//-----------------------------------------------------------------------------------------

typedef SVU3V_RETURN
(*TFunc_Camera_writeEEPROM)(Camera_handle hCamera);

typedef SVU3V_RETURN
(*TFunc_Camera_readEEPROM)(Camera_handle hCamera);

typedef SVU3V_RETURN
(*TFunc_Camera_restoreFactoryDefaults)(Camera_handle hCamera);

typedef SVU3V_RETURN
(*TFunc_Camera_loadSettingsFromXml)(Camera_handle hCamera,
                                    char *Filename);

typedef SVU3V_RETURN
(*TFunc_Camera_SaveSettingsToXml)(Camera_handle hCamera,
                                  char *Filename);

//-----------------------------------------------------------------------------------------
// 24 - General functions
//-----------------------------------------------------------------------------------------

typedef SVU3V_RETURN 
(*TFunc_SVU3V_estimateWhiteBalance)(unsigned char *DataRGB, int DataRGBLength, float *Red, float *Green, float *Blue);

typedef SVU3V_RETURN 
(*TFunc_SVU3V_estimateWhiteBalanceExtended)(unsigned char *DataRGB, int PixelNumber, int &Red, int &Green, int &Blue,SVU3V_Whitebalance_SELECT  Whitebalance_Art );

typedef SVU3V_RETURN 
(*TFunc_SVU3V_writeImageToBitmapFile)(char *Filename, unsigned char *Data, int SizeX, int SizeY, GVSP_PIXEL_TYPE PixelType);

typedef SVU3V_RETURN 
(*TFunc_SVU3V_installFilterDriver)(char *PathToDriverPackage, char *FilenameINF, char *FilenameINF_m);

typedef SVU3V_RETURN 
(*TFunc_SVU3V_uninstallFilterDriver)();


//-----------------------------------------------------------------------------------------
// 25 - Special control: Lens control
//-----------------------------------------------------------------------------------------

typedef SVU3V_RETURN
(*TFunc_Camera_isLensAvailable)(Camera_handle hCamera, bool *isAvailable);

typedef char*
(*TFunc_Camera_getLensName)(Camera_handle hCamera);

//----------------------------------------FocalLenght--------------------------------------
typedef SVU3V_RETURN
(*TFunc_Camera_setLensFocalLenght)(Camera_handle hCamera, unsigned int ProgrammedFocalLenght);

typedef SVU3V_RETURN
(*TFunc_Camera_getLensFocalLenght)(Camera_handle hCamera, unsigned int *ProgrammedFocalLenght);

typedef SVU3V_RETURN
(*TFunc_Camera_getLensFocalLenghtMin)(Camera_handle hCamera, unsigned int *FocalLenghtMin);
  
typedef SVU3V_RETURN
(*TFunc_Camera_getLensFocalLenghtMax)(Camera_handle hCamera, unsigned int *FocalLenghtMax);
  

//----------------------------------------Focus------------------------------------------


typedef SVU3V_RETURN
(*TFunc_Camera_setLensFocusUnit)(Camera_handle hCamera, FOCUS_UNIT Selected_unit );

typedef SVU3V_RETURN
(*TFunc_Camera_getLensFocusUnit)(Camera_handle hCamera, FOCUS_UNIT *Selected_unit );

typedef SVU3V_RETURN
(*TFunc_Camera_setLensFocus)(Camera_handle hCamera, unsigned int ProgrammedFocus);

typedef SVU3V_RETURN
(*TFunc_Camera_getLensFocus)(Camera_handle hCamera, unsigned int *ProgrammedFocus); 

typedef SVU3V_RETURN
(*TFunc_Camera_getLensFocusMin)(Camera_handle hCamera, unsigned int *FocusMin);

typedef SVU3V_RETURN
(*TFunc_Camera_getLensFocusMax)(Camera_handle hCamera, unsigned int *FocusMax);


//----------------------------------------Aperture------------------------------------------

typedef SVU3V_RETURN
(*TFunc_Camera_setLensAperture)(Camera_handle hCamera, unsigned int ProgrammedAperture);

typedef SVU3V_RETURN
(*TFunc_Camera_getLensAperture)(Camera_handle hCamera, unsigned int *ProgrammedAperture);

typedef SVU3V_RETURN
(*TFunc_Camera_getLensApertureMin)(Camera_handle hCamera, unsigned int *ApertureMin);

typedef SVU3V_RETURN
(*TFunc_Camera_getLensApertureMax)(Camera_handle hCamera, unsigned int *ApertureMax);




//--------------------End-of-function-type-declarations------------------------------------


//*****************************************************************************************
// 0 - SVU3V DLL (implicitly called)
//*****************************************************************************************

bool isLoadedU3VDLL()
{
  
    // Try to load SVU3V DLL
    U3VDLL = LoadLibrary(SVU3V_DLL);

    // Check DLL availability
    if( NULL == U3VDLL )
      return false;
 

	// Check if size of function table matches the number of imported functions
	int FunctionCount = sizeof(U3VFunc) / sizeof(struct _U3VFunc);
	if( FunctionCount != Func_isVersionCompliantDLL_consistency_check + 1 )
		return false;

  // Obtain CameraContainer procedure addresses
  bool function_loaded = true;
  for( int function_index = Func_isVersionCompliantDLL;
       function_index < (sizeof(U3VFunc) / sizeof(struct _U3VFunc));
       function_index++
     )
  {
    U3VFunc[function_index].function_pointer = GetProcAddress(U3VDLL, U3VFunc[function_index].function_name);

    // Check if function was found
    if( NULL == U3VFunc[function_index].function_pointer )
      function_loaded = false;
  }

  // Check if all function pointers could successfully be obtained from the DLL
  if( function_loaded == false )
    return false;
  else
    return true;
}

SVU3V_RETURN
isVersionCompliantDLL(SVU3V_VERSION *DllVersion, 
                      SVU3V_VERSION *ExpectedVersion)
{
  // Check DLL availability
  if( NULL == U3VDLL )
  {
    // Try to load SVU3V DLL
    if( !isLoadedU3VDLL() )
      return U3V_DLL_NOT_LOADED;
  }
  // Pass through function call to DLL
    // check consistency of the whole function pointer 
	//table by calling the last function in that table.
	
  return ((TFunc_isVersionCompliantDLL)
  U3VFunc[Func_isVersionCompliantDLL_consistency_check].function_pointer)(DllVersion, ExpectedVersion);
}



//*****************************************************************************************
// 1 - Camera: Discovery and bookkeeping
//*****************************************************************************************

CameraContainerClient_handle
CameraContainer_create()
{
  // Load DLL 
  if( !isLoadedU3VDLL() )
    return SVU3V_NO_CLIENT;

  // Check DLL version against version at compile time
  SVU3V_VERSION DllVersion;
  SVU3V_VERSION ExpectedVersion;
  ExpectedVersion.MajorVersion  = SVU3V_VERSION_MAJOR;
  ExpectedVersion.MinorVersion  = SVU3V_VERSION_MINOR;
  ExpectedVersion.RevisionVersion = SVU3V_VERSION_REVISION;
  ExpectedVersion.BuildVersion  = SVU3V_VERSION_BUILD;
  if( U3V_STATUS_SUCCESS != isVersionCompliantDLL(&DllVersion, &ExpectedVersion) )
    return SVU3V_NO_CLIENT;

  // Pass through function call to DLL
  return ((TFunc_CameraContainer_create)
  U3VFunc[Func_CameraContainer_create].function_pointer)();
}

SVU3V_RETURN
CameraContainer_delete(CameraContainerClient_handle hCameraContainer)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  SVU3V_RETURN Ret =
  ((TFunc_CameraContainer_delete)
  U3VFunc[Func_CameraContainer_delete].function_pointer)(hCameraContainer);

  // Release DLL (reference counter will be decreased)
  FreeLibrary(U3VDLL);

	return Ret; 
}

SVU3V_RETURN
CameraContainer_discovery(CameraContainerClient_handle hCameraContainer)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_CameraContainer_discovery)
    U3VFunc[Func_CameraContainer_discovery].function_pointer)(hCameraContainer);
}

int
CameraContainer_getNumberOfCameras(CameraContainerClient_handle hCameraContainer)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_CameraContainer_getNumberOfCameras)
  U3VFunc[Func_CameraContainer_getNumberOfCameras].function_pointer)(hCameraContainer);
}

Camera_handle
CameraContainer_getCamera(CameraContainerClient_handle hCameraContainer,
                          int CameraIndex)
{
  // Check DLL availablility
  if( NULL == U3VDLL )
    return SVU3V_NO_CAMERA;

  // Pass through function call to DLL
  return ((TFunc_CameraContainer_getCamera)
  U3VFunc[Func_CameraContainer_getCamera].function_pointer)(hCameraContainer, CameraIndex);
}

Camera_handle
CameraContainer_findCamera(CameraContainerClient_handle hCameraContainer,
                           char *CameraItem)
{
  // Check DLL availablility
  if( NULL == U3VDLL )
    return SVU3V_NO_CAMERA;

  // Pass through function call to DLL
  return ((TFunc_CameraContainer_findCamera)
  U3VFunc[Func_CameraContainer_findCamera].function_pointer)(hCameraContainer, CameraItem);
}

//*****************************************************************************************
// 2 - Camera: Connection
//*****************************************************************************************

SVU3V_RETURN
Camera_openConnection(Camera_handle hCamera, float Timeout)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_openConnection)
  U3VFunc[Func_Camera_openConnection].function_pointer)(hCamera, Timeout);
}



SVU3V_RETURN
Camera_closeConnection(Camera_handle hCamera)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_closeConnection)
  U3VFunc[Func_Camera_closeConnection].function_pointer)(hCamera);
}


//*****************************************************************************************
// 3 - Camera: Information
//*****************************************************************************************

char *
Camera_getManufacturerName(Camera_handle hCamera)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return "";

  // Pass through function call to DLL
  return ((TFunc_Camera_getManufacturerName)
  U3VFunc[Func_Camera_getManufacturerName].function_pointer)(hCamera);
}

char *
Camera_getModelName(Camera_handle hCamera)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return "";

  // Pass through function call to DLL
  return ((TFunc_Camera_getModelName)
  U3VFunc[Func_Camera_getModelName].function_pointer)(hCamera);
}

char *
Camera_getDeviceVersion(Camera_handle hCamera)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return "";

  // Pass through function call to DLL
  return ((TFunc_Camera_getDeviceVersion)
  U3VFunc[Func_Camera_getDeviceVersion].function_pointer)(hCamera);
}

char *
Camera_getManufacturerSpecificInformation(Camera_handle hCamera)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return "";

  // Pass through function call to DLL
  return ((TFunc_Camera_getManufacturerSpecificInformation)
  U3VFunc[Func_Camera_getManufacturerSpecificInformation].function_pointer)(hCamera);
}

char *
Camera_getSerialNumber(Camera_handle hCamera)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return "";

  // Pass through function call to DLL
  return ((TFunc_Camera_getSerialNumber)
  U3VFunc[Func_Camera_getSerialNumber].function_pointer)(hCamera);
}

SVU3V_RETURN
Camera_setUserDefinedName(Camera_handle hCamera, char *UserDefinedName)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_setUserDefinedName)
  U3VFunc[Func_Camera_setUserDefinedName].function_pointer)(hCamera, UserDefinedName);
}

char *
Camera_getUserDefinedName(Camera_handle hCamera)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return "";

  // Pass through function call to DLL
  return ((TFunc_Camera_getUserDefinedName)
  U3VFunc[Func_Camera_getUserDefinedName].function_pointer)(hCamera);
}

SVU3V_RETURN
Camera_getPixelClock(Camera_handle hCamera,
                    int *PixelClock)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getPixelClock)
  U3VFunc[Func_Camera_getPixelClock].function_pointer)(hCamera, PixelClock);
}

bool
Camera_isCameraFeature(Camera_handle hCamera,
                       CAMERA_FEATURE Feature)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return false;

  // Pass through function call to DLL
  return ((TFunc_Camera_isCameraFeature)
  U3VFunc[Func_Camera_isCameraFeature].function_pointer)(hCamera, Feature);
}

SVU3V_RETURN
Camera_readXMLFile(Camera_handle hCamera,
                   char **XML)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_readXMLFile)
  U3VFunc[Func_Camera_readXMLFile].function_pointer)(hCamera, XML);
}

SVU3V_RETURN
Camera_getSensorTemperature(Camera_handle hCamera, 
                            unsigned int *SensorTemperature)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getSensorTemperature)
  U3VFunc[Func_Camera_getSensorTemperature].function_pointer)(hCamera, SensorTemperature);
}

//*****************************************************************************************
// 4 - Stream: Channel creation and control
//*****************************************************************************************

SVU3V_RETURN
StreamingChannel_create(StreamingChannel_handle *hStreamingChannel,
                        CameraContainerClient_handle hCameraContainer,
                        Camera_handle hCamera,
                        int BufferCount,
                        StreamCallback CallbackFunction,
                        void *Context)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_StreamingChannel_create)
  U3VFunc[Func_StreamingChannel_create].function_pointer)(hStreamingChannel, hCameraContainer, hCamera, BufferCount, CallbackFunction, Context);
}


SVU3V_RETURN
StreamingChannel_delete(StreamingChannel_handle hStreamingChannel)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_StreamingChannel_delete)
  U3VFunc[Func_StreamingChannel_delete].function_pointer)(hStreamingChannel);
}


//*****************************************************************************************
// 5 - Stream: Channel info
//*****************************************************************************************

SVU3V_RETURN
StreamingChannel_getPixelType(StreamingChannel_handle hStreamingChannel,
                              GVSP_PIXEL_TYPE *ProgrammedPixelType)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_StreamingChannel_getPixelType)
  U3VFunc[Func_StreamingChannel_getPixelType].function_pointer)(hStreamingChannel,ProgrammedPixelType);
}


SVU3V_RETURN
StreamingChannel_getImagePitch(StreamingChannel_handle hStreamingChannel,
                               int *ImagePitch)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_StreamingChannel_getImagePitch)
  U3VFunc[Func_StreamingChannel_getImagePitch].function_pointer)(hStreamingChannel,ImagePitch);
}

SVU3V_RETURN
StreamingChannel_getImageSizeX(StreamingChannel_handle hStreamingChannel,
                               int *ImageSizeX)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_StreamingChannel_getImageSizeX)
  U3VFunc[Func_StreamingChannel_getImageSizeX].function_pointer)(hStreamingChannel,ImageSizeX);
}

SVU3V_RETURN
StreamingChannel_getImageSizeY(StreamingChannel_handle hStreamingChannel,
                               int *ImageSizeY)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_StreamingChannel_getImageSizeY)
  U3VFunc[Func_StreamingChannel_getImageSizeY].function_pointer)(hStreamingChannel,ImageSizeY);
}

//*****************************************************************************************
// 6 - Stream: Image access
//*****************************************************************************************

unsigned char *
Image_getDataPointer(Image_handle hImage)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return NULL;

  // Pass through function call to DLL
  return ((TFunc_Image_getDataPointer)
  U3VFunc[Func_Image_getDataPointer].function_pointer)(hImage);
}



SVU3V_SIGNAL_TYPE
Image_getSignalType(Image_handle hImage)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return SVU3V_SIGNAL_NONE;

  // Pass through function call to DLL
  return ((TFunc_Image_getSignalType)
  U3VFunc[Func_Image_getSignalType].function_pointer)(hImage);
}

Camera_handle 
Image_getCamera(Image_handle hImage)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return SVU3V_NO_CAMERA;

  // Pass through function call to DLL
  return ((TFunc_Image_getCamera)
  U3VFunc[Func_Image_getCamera].function_pointer)(hImage);
}

SVU3V_RETURN
Image_release(Image_handle hImage)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Image_release)
  U3VFunc[Func_Image_release].function_pointer)(hImage);
}

//*****************************************************************************************
// 7 - Stream: Image conversion
//*****************************************************************************************

SVU3V_RETURN
Image_getImageRGB(Image_handle hImage,
                  unsigned char *BufferRGB,
                  int BufferLength,
                  BAYER_METHOD BayerMethod)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Image_getImageRGB)
  U3VFunc[Func_Image_getImageRGB].function_pointer)(hImage, BufferRGB, BufferLength, BayerMethod);
}


SVU3V_RETURN
Image_getImage12bitAs8bit(Image_handle hImage,
                          unsigned char *Buffer8bit,
                          int BufferLength)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Image_getImage12bitAs8bit)
  U3VFunc[Func_Image_getImage12bitAs8bit].function_pointer)(hImage, Buffer8bit, BufferLength);
}

SVU3V_RETURN
Image_getImage12bitAs16bit(Image_handle hImage,
                           unsigned char *Buffer16bit,
                           int BufferLength)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Image_getImage12bitAs16bit)
  U3VFunc[Func_Image_getImage12bitAs16bit].function_pointer)(hImage, Buffer16bit, BufferLength);
}

SVU3V_RETURN
Image_getImage16bitAs8bit(Image_handle hImage,
                          unsigned char *Buffer8bit,
                          int BufferLength)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Image_getImage16bitAs8bit)
  U3VFunc[Func_Image_getImage16bitAs8bit].function_pointer)(hImage, Buffer8bit, BufferLength);
}

//*****************************************************************************************
// 8 - Stream: Image characteristics
//*****************************************************************************************

GVSP_PIXEL_TYPE
Image_getPixelType(Image_handle hImage)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return GVSP_PIX_MONO8;

  // Pass through function call to DLL
  return ((TFunc_Image_getPixelType)
  U3VFunc[Func_Image_getPixelType].function_pointer)(hImage);
}

int
Image_getImageSize(Image_handle hImage)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return 0;

  // Pass through function call to DLL
  return ((TFunc_Image_getImageSize)
  U3VFunc[Func_Image_getImageSize].function_pointer)(hImage);
}

int
Image_getPitch(Image_handle hImage)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return 0;

  // Pass through function call to DLL
  return ((TFunc_Image_getPitch)
  U3VFunc[Func_Image_getPitch].function_pointer)(hImage);
}

int
Image_getSizeX(Image_handle hImage)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return 0;

  // Pass through function call to DLL
  return ((TFunc_Image_getSizeX)
  U3VFunc[Func_Image_getSizeX].function_pointer)(hImage);
}

int
Image_getSizeY(Image_handle hImage)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return 0;

  // Pass through function call to DLL
  return ((TFunc_Image_getSizeY)
  U3VFunc[Func_Image_getSizeY].function_pointer)(hImage);
}

//*****************************************************************************************
// 9 - Stream: Image statistics
//*****************************************************************************************

int
Image_getImageID(Image_handle hImage)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return 0;

  // Pass through function call to DLL
  return ((TFunc_Image_getImageID)
  U3VFunc[Func_Image_getImageID].function_pointer)(hImage);
}

double
Image_getTimestamp(Image_handle hImage)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return 0;

  // Pass through function call to DLL
  return ((TFunc_Image_getTimestamp)
  U3VFunc[Func_Image_getTimestamp].function_pointer)(hImage);
}


//*****************************************************************************************
// 10 - Controlling camera: Frame rate
//*****************************************************************************************

SVU3V_RETURN
Camera_setFrameRate(Camera_handle hCamera,
                    float Framerate)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_setFrameRate)
  U3VFunc[Func_Camera_setFrameRate].function_pointer)(hCamera, Framerate);
}

SVU3V_RETURN
Camera_getFrameRate(Camera_handle hCamera,
                    float *ProgrammedFramerate)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getFrameRate)
  U3VFunc[Func_Camera_getFrameRate].function_pointer)(hCamera, ProgrammedFramerate);
}

SVU3V_RETURN
Camera_getFrameRateMin(Camera_handle hCamera,
                       float *MinFramerate)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getFrameRateMin)
  U3VFunc[Func_Camera_getFrameRateMin].function_pointer)(hCamera, MinFramerate);
}

SVU3V_RETURN
Camera_getFrameRateMax(Camera_handle hCamera,
                       float *MaxFramerate)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getFrameRateMax)
  U3VFunc[Func_Camera_getFrameRateMax].function_pointer)(hCamera, MaxFramerate);
}

SVU3V_RETURN
Camera_getFrameRateRange(Camera_handle hCamera,
                         float *MinFramerate,
                         float *MaxFramerate)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getFrameRateRange)
  U3VFunc[Func_Camera_getFrameRateRange].function_pointer)(hCamera, MinFramerate, MaxFramerate);
}

SVU3V_RETURN
Camera_getFrameRateIncrement(Camera_handle hCamera,
                             float *FramerateIncrement)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getFrameRateIncrement)
  U3VFunc[Func_Camera_getFrameRateIncrement].function_pointer)(hCamera, FramerateIncrement);
}

//*****************************************************************************************
// 11 - Controlling camera: Exposure
//*****************************************************************************************

SVU3V_RETURN
Camera_setExposureTime(Camera_handle hCamera,
                       float ExposureTime)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_setExposureTime)
  U3VFunc[Func_Camera_setExposureTime].function_pointer)(hCamera, ExposureTime);
}

SVU3V_RETURN
Camera_getExposureTime(Camera_handle hCamera,
                       float *ProgrammedExposureTime)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getExposureTime)
  U3VFunc[Func_Camera_getExposureTime].function_pointer)(hCamera, ProgrammedExposureTime);
}

SVU3V_RETURN
Camera_getExposureTimeMin(Camera_handle hCamera,
                          float *MinExposureTime)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getExposureTimeMin)
  U3VFunc[Func_Camera_getExposureTimeMin].function_pointer)(hCamera, MinExposureTime);
}

SVU3V_RETURN
Camera_getExposureTimeMax(Camera_handle hCamera,
                          float *MaxExposureTime)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getExposureTimeMax)
  U3VFunc[Func_Camera_getExposureTimeMax].function_pointer)(hCamera, MaxExposureTime);
}

SVU3V_RETURN
Camera_getExposureTimeRange(Camera_handle hCamera,
                            float *MinExposureTime,
                            float *MaxExposureTime)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getExposureTimeRange)
  U3VFunc[Func_Camera_getExposureTimeRange].function_pointer)(hCamera, MinExposureTime, MaxExposureTime);
}

SVU3V_RETURN
Camera_getExposureTimeIncrement(Camera_handle hCamera,
                                float *ExposureTimeIncrement)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getExposureTimeIncrement)
  U3VFunc[Func_Camera_getExposureTimeIncrement].function_pointer)(hCamera, ExposureTimeIncrement);
}

SVU3V_RETURN
Camera_setExposureDelay(Camera_handle hCamera,
                         float ExposureDelay)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_setExposureDelay)
  U3VFunc[Func_Camera_setExposureDelay].function_pointer)(hCamera, ExposureDelay);
}

SVU3V_RETURN
Camera_getExposureDelay(Camera_handle hCamera,
                        float *ProgrammedExposureDelay)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getExposureDelay)
  U3VFunc[Func_Camera_getExposureDelay].function_pointer)(hCamera, ProgrammedExposureDelay);
}

SVU3V_RETURN
Camera_getExposureDelayMax(Camera_handle hCamera,
                           float *MaxExposureDelay)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getExposureDelayMax)
  U3VFunc[Func_Camera_getExposureDelayMax].function_pointer)(hCamera, MaxExposureDelay);
}

SVU3V_RETURN
Camera_getExposureDelayIncrement(Camera_handle hCamera,
                                 float *ExposureDelayIncrement)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getExposureDelayIncrement)
  U3VFunc[Func_Camera_getExposureDelayIncrement].function_pointer)(hCamera, ExposureDelayIncrement);
}

//*****************************************************************************************
// 12 - Controlling camera: Gain and offset
//*****************************************************************************************

SVU3V_RETURN
Camera_setGain(Camera_handle hCamera,
               float Gain)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_setGain)
  U3VFunc[Func_Camera_setGain].function_pointer)(hCamera, Gain);
}

SVU3V_RETURN
Camera_getGain(Camera_handle hCamera,
               float *ProgrammedGain)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getGain)
  U3VFunc[Func_Camera_getGain].function_pointer)(hCamera, ProgrammedGain);
}



SVU3V_RETURN
Camera_getGainMax(Camera_handle hCamera,
                          float *MaxGain)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getGainMax)
  U3VFunc[Func_Camera_getGainMax].function_pointer)(hCamera, MaxGain);
}


SVU3V_RETURN
Camera_getGainMaxExtended(Camera_handle hCamera,
                          float *MaxGainExtended)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getGainMaxExtended)
  U3VFunc[Func_Camera_getGainMaxExtended].function_pointer)(hCamera, MaxGainExtended);
}

SVU3V_RETURN
Camera_getGainIncrement(Camera_handle hCamera,
                        float *GainIncrement)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getGainIncrement)
  U3VFunc[Func_Camera_getGainIncrement].function_pointer)(hCamera, GainIncrement);
}

SVU3V_RETURN
Camera_setOffset(Camera_handle hCamera,
                 float Offset)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_setOffset)
  U3VFunc[Func_Camera_setOffset].function_pointer)(hCamera, Offset);
}

SVU3V_RETURN
Camera_getOffset(Camera_handle hCamera,
                 float *ProgrammedOffset)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getOffset)
  U3VFunc[Func_Camera_getOffset].function_pointer)(hCamera, ProgrammedOffset);
}

SVU3V_RETURN
Camera_getOffsetMax(Camera_handle hCamera,
                    float *MaxOffset)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getOffsetMax)
  U3VFunc[Func_Camera_getOffsetMax].function_pointer)(hCamera, MaxOffset);
}

//*****************************************************************************************
// 13 - Controlling camera: Auto gain/exposure
//*****************************************************************************************

SVU3V_RETURN
Camera_setAutoGainEnabled(Camera_handle hCamera,
													bool isAutoGainEnabled)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_setAutoGainEnabled)
  U3VFunc[Func_Camera_setAutoGainEnabled].function_pointer)(hCamera, isAutoGainEnabled);
}

SVU3V_RETURN
Camera_getAutoGainEnabled(Camera_handle hCamera,
														bool *isAutoGainEnabled)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getAutoGainEnabled)
  U3VFunc[Func_Camera_getAutoGainEnabled].function_pointer)(hCamera, isAutoGainEnabled);
}

SVU3V_RETURN
Camera_setAutoGainBrightness(Camera_handle hCamera,
                               float Brightness)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_setAutoGainBrightness)
  U3VFunc[Func_Camera_setAutoGainBrightness].function_pointer)(hCamera, Brightness);
}

SVU3V_RETURN
Camera_getAutoGainBrightness(Camera_handle hCamera,
                             float *ProgrammedBrightness)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getAutoGainBrightness)
  U3VFunc[Func_Camera_getAutoGainBrightness].function_pointer)(hCamera, ProgrammedBrightness);
}

SVU3V_RETURN
Camera_setAutoGainDynamics(Camera_handle hCamera,
                           float AutoGainParameterI,
                           float AutoGainParameterD)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_setAutoGainDynamics)
  U3VFunc[Func_Camera_setAutoGainDynamics].function_pointer)(hCamera, AutoGainParameterI, AutoGainParameterD);
}

SVU3V_RETURN
Camera_getAutoGainDynamics(Camera_handle hCamera,
                           float *ProgrammedAutoGainParameterI,
                           float *ProgrammedAutoGainParameterD)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getAutoGainDynamics)
  U3VFunc[Func_Camera_getAutoGainDynamics].function_pointer)(hCamera, ProgrammedAutoGainParameterI, ProgrammedAutoGainParameterD);
}

SVU3V_RETURN
Camera_setAutoGainLimits(Camera_handle hCamera,
                         float MinGain,
                         float MaxGain)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_setAutoGainLimits)
  U3VFunc[Func_Camera_setAutoGainLimits].function_pointer)(hCamera, MinGain, MaxGain);
}

SVU3V_RETURN
Camera_getAutoGainLimits(Camera_handle hCamera,
                         float *ProgrammedMinGain,
                         float *ProgrammedMaxGain)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getAutoGainLimits)
  U3VFunc[Func_Camera_getAutoGainLimits].function_pointer)(hCamera, ProgrammedMinGain, ProgrammedMaxGain);
}

SVU3V_RETURN
Camera_setAutoExposureLimits(Camera_handle hCamera,
                             float MinExposure,
                             float MaxExposure)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_setAutoExposureLimits)
  U3VFunc[Func_Camera_setAutoExposureLimits].function_pointer)(hCamera, MinExposure, MaxExposure);
}

SVU3V_RETURN
Camera_getAutoExposureLimits(Camera_handle hCamera,
                             float *ProgrammedMinExposure,
                             float *ProgrammedMaxExposure)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getAutoExposureLimits)
  U3VFunc[Func_Camera_getAutoExposureLimits].function_pointer)(hCamera, ProgrammedMinExposure, ProgrammedMaxExposure);
}

//*****************************************************************************************
// 14 - Controlling camera: Acquisition trigger
//*****************************************************************************************

SVU3V_RETURN
Camera_setAcquisitionControl(Camera_handle hCamera,
                             ACQUISITION_CONTROL AcquisitionControl)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_setAcquisitionControl)
  U3VFunc[Func_Camera_setAcquisitionControl].function_pointer)(hCamera, AcquisitionControl);
}

SVU3V_RETURN
Camera_getAcquisitionControl(Camera_handle hCamera,
                             ACQUISITION_CONTROL *ProgrammedAcquisitionControl)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getAcquisitionControl)
  U3VFunc[Func_Camera_getAcquisitionControl].function_pointer)(hCamera, ProgrammedAcquisitionControl);
}

SVU3V_RETURN
Camera_setAcquisitionMode(Camera_handle hCamera,
                          ACQUISITION_MODE AcquisitionMode)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_setAcquisitionMode)
  U3VFunc[Func_Camera_setAcquisitionMode].function_pointer)(hCamera, AcquisitionMode);
}


SVU3V_RETURN
Camera_getAcquisitionMode(Camera_handle hCamera,
                          ACQUISITION_MODE *ProgrammedAcquisitionMode)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getAcquisitionMode)
  U3VFunc[Func_Camera_getAcquisitionMode].function_pointer)(hCamera, ProgrammedAcquisitionMode);
}

SVU3V_RETURN
Camera_softwareTrigger(Camera_handle hCamera)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_softwareTrigger)
  U3VFunc[Func_Camera_softwareTrigger].function_pointer)(hCamera);
}


SVU3V_RETURN
Camera_setTriggerPolarity(Camera_handle hCamera,
                          TRIGGER_POLARITY TriggerPolarity)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_setTriggerPolarity)
  U3VFunc[Func_Camera_setTriggerPolarity].function_pointer)(hCamera, TriggerPolarity);
}

SVU3V_RETURN
Camera_getTriggerPolarity(Camera_handle hCamera,
                          TRIGGER_POLARITY *ProgrammedTriggerPolarity)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getTriggerPolarity)
  U3VFunc[Func_Camera_getTriggerPolarity].function_pointer)(hCamera, ProgrammedTriggerPolarity);
}

//---------------PIV-mode--------------------------------
SVU3V_RETURN
Camera_setPivMode(Camera_handle hCamera,
                  PIV_MODE SelectPivMode)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_setPivMode)
  U3VFunc[Func_Camera_setPivMode].function_pointer)(hCamera, SelectPivMode);
}

SVU3V_RETURN
Camera_getPivMode(Camera_handle hCamera,
                  PIV_MODE *ProgrammedPivMode)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getPivMode)
  U3VFunc[Func_Camera_getPivMode].function_pointer)(hCamera, ProgrammedPivMode);
}

//---------------debouncer-------------------------------------
SVU3V_RETURN
Camera_setDebouncerDuration(Camera_handle hCamera,
                            float  DebouncerDuration)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_setDebouncerDuration)
  U3VFunc[Func_Camera_setDebouncerDuration].function_pointer)(hCamera, DebouncerDuration);
}

SVU3V_RETURN
Camera_getDebouncerDuration(Camera_handle hCamera,
                            float *ProgrammedDuration)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getDebouncerDuration)
  U3VFunc[Func_Camera_getDebouncerDuration].function_pointer)(hCamera, ProgrammedDuration);
}

//---------------prescaler-------------------------------------
SVU3V_RETURN
Camera_setPrescalerDevisor(Camera_handle hCamera,
                           unsigned int  PrescalerDevisor)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_setPrescalerDevisor)
  U3VFunc[Func_Camera_setPrescalerDevisor].function_pointer)(hCamera, PrescalerDevisor);
}

SVU3V_RETURN
Camera_getPrescalerDevisor(Camera_handle hCamera,
                            unsigned int *ProgrammedPrescalerDevisor)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getPrescalerDevisor)
  U3VFunc[Func_Camera_getPrescalerDevisor].function_pointer)(hCamera, ProgrammedPrescalerDevisor);
}

//---------------sequencer-------------------------------------
SVU3V_RETURN
 Camera_loadSequenceParameters(Camera_handle hCamera,
                               char *Filename)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_loadSequenceParameters)
  U3VFunc[Func_Camera_loadSequenceParameters].function_pointer)(hCamera, Filename);
}

SVU3V_RETURN
Camera_startSequencer(Camera_handle hCamera)
{
	// Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_startSequencer)
  U3VFunc[Func_Camera_startSequencer].function_pointer)(hCamera);
}

//*****************************************************************************************
// 15 - Controlling camera: Strobe
//*****************************************************************************************



SVU3V_RETURN
Camera_setStrobePolarityExtended(Camera_handle hCamera,
                                 STROBE_POLARITY StrobePolarity, int StrobeIndex )
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_setStrobePolarityExtended)
  U3VFunc[Func_Camera_setStrobePolarityExtended].function_pointer)(hCamera, StrobePolarity,  StrobeIndex);
}



SVU3V_RETURN
Camera_getStrobePolarityExtended(Camera_handle hCamera,
                                 STROBE_POLARITY *ProgrammedStrobePolarity,int StrobeIndex)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getStrobePolarityExtended)
  U3VFunc[Func_Camera_getStrobePolarityExtended].function_pointer)(hCamera, ProgrammedStrobePolarity, StrobeIndex);
}




SVU3V_RETURN
Camera_setStrobePositionExtended(Camera_handle hCamera,
                                 float StrobePosition,int StrobeIndex)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_setStrobePositionExtended)
  U3VFunc[Func_Camera_setStrobePositionExtended].function_pointer)(hCamera, StrobePosition,StrobeIndex);
}



SVU3V_RETURN
Camera_getStrobePositionExtended(Camera_handle hCamera,
                                 float *ProgrammedStrobePosition, int StrobeIndex )
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getStrobePositionExtended)
  U3VFunc[Func_Camera_getStrobePositionExtended].function_pointer)(hCamera, ProgrammedStrobePosition, StrobeIndex);
}

SVU3V_RETURN
Camera_getStrobePositionMax(Camera_handle hCamera,
                            float *MaxStrobePosition)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getStrobePositionMax)
  U3VFunc[Func_Camera_getStrobePositionMax].function_pointer)(hCamera, MaxStrobePosition);
}

SVU3V_RETURN
Camera_getStrobePositionIncrement(Camera_handle hCamera,
                                  float *StrobePositionIncrement)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getStrobePositionIncrement)
  U3VFunc[Func_Camera_getStrobePositionIncrement].function_pointer)(hCamera, StrobePositionIncrement);
}


SVU3V_RETURN
Camera_setStrobeDurationExtended(Camera_handle hCamera,
                                 float StrobeDuration,int StrobeIndex  )
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_setStrobeDurationExtended)
  U3VFunc[Func_Camera_setStrobeDurationExtended].function_pointer)(hCamera, StrobeDuration, StrobeIndex);
}



SVU3V_RETURN
Camera_getStrobeDurationExtended(Camera_handle hCamera,
                                 float *ProgrammedStrobeDuration, int StrobeIndex )
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getStrobeDurationExtended)
  U3VFunc[Func_Camera_getStrobeDurationExtended].function_pointer)(hCamera, ProgrammedStrobeDuration,StrobeIndex);
}

SVU3V_RETURN
Camera_getStrobeDurationMax(Camera_handle hCamera,
                            float *MaxStrobeDuration)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getStrobeDurationMax)
  U3VFunc[Func_Camera_getStrobeDurationMax].function_pointer)(hCamera, MaxStrobeDuration);
}

SVU3V_RETURN
Camera_getStrobeDurationIncrement(Camera_handle hCamera,
                                  float *StrobeDurationIncrement)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getStrobeDurationIncrement)
  U3VFunc[Func_Camera_getStrobeDurationIncrement].function_pointer)(hCamera, StrobeDurationIncrement);
}

//*****************************************************************************************
// 16 - Controlling camera: Tap balance
//*****************************************************************************************


SVU3V_RETURN 
Camera_setTapConfigurationEx(Camera_handle hCamera,
                         SVU3V_TAP_CONFIGURATION_SELECT SelectedTapConfig)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_setTapConfigurationEx)
  U3VFunc[Func_Camera_setTapConfigurationEx].function_pointer)(hCamera, SelectedTapConfig);
}

SVU3V_RETURN 
Camera_getTapConfigurationEx(Camera_handle hCamera,
                         SVU3V_TAP_CONFIGURATION_SELECT *ProgrammedTapConfig)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getTapConfigurationEx)
  U3VFunc[Func_Camera_getTapConfigurationEx].function_pointer)(hCamera, ProgrammedTapConfig);
}

SVU3V_RETURN
Camera_setAutoTapBalanceMode(Camera_handle hCamera,
                             SVU3V_AUTO_TAP_BALANCE_MODE AutoTapBalanceMode)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_setAutoTapBalanceMode)
  U3VFunc[Func_Camera_setAutoTapBalanceMode].function_pointer)(hCamera, AutoTapBalanceMode);
}

SVU3V_RETURN
Camera_getAutoTapBalanceMode(Camera_handle hCamera,
                             SVU3V_AUTO_TAP_BALANCE_MODE *AutoTapBalanceMode)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getAutoTapBalanceMode)
  U3VFunc[Func_Camera_getAutoTapBalanceMode].function_pointer)(hCamera, AutoTapBalanceMode);
}

SVU3V_RETURN
Camera_setTapGain(Camera_handle hCamera,
									float TapGain,
									SVU3V_TAP_SELECT Tap)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_setTapGain)
  U3VFunc[Func_Camera_setTapGain].function_pointer)(hCamera, TapGain, Tap);
}

SVU3V_RETURN
Camera_getTapGain(Camera_handle hCamera,
									float *TapGain,
									SVU3V_TAP_SELECT Tap)
{
// Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getTapGain)
  U3VFunc[Func_Camera_getTapGain].function_pointer)(hCamera, TapGain, Tap);
}



//*****************************************************************************************
// 17 - Controlling camera: Image parameter
//*****************************************************************************************

SVU3V_RETURN
Camera_getImagerWidth(Camera_handle hCamera,
                      int *ImagerWidth)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getImagerWidth)
  U3VFunc[Func_Camera_getImagerWidth].function_pointer)(hCamera, ImagerWidth);
}

SVU3V_RETURN
Camera_getImagerHeight(Camera_handle hCamera,
                       int *ImagerHeight)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getImagerHeight)
  U3VFunc[Func_Camera_getImagerHeight].function_pointer)(hCamera, ImagerHeight);
}

SVU3V_RETURN
Camera_getImageSize(Camera_handle hCamera,
                    int *ImageSize)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getImageSize)
  U3VFunc[Func_Camera_getImageSize].function_pointer)(hCamera, ImageSize);
}

SVU3V_RETURN
Camera_getPitch(Camera_handle hCamera,
                int *Pitch)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getPitch)
  U3VFunc[Func_Camera_getPitch].function_pointer)(hCamera, Pitch);
}

SVU3V_RETURN
Camera_getSizeX(Camera_handle hCamera,
                int *SizeX)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getSizeX)
  U3VFunc[Func_Camera_getSizeX].function_pointer)(hCamera, SizeX);
}

SVU3V_RETURN
Camera_getSizeY(Camera_handle hCamera,
                int *SizeY)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getSizeY)
  U3VFunc[Func_Camera_getSizeY].function_pointer)(hCamera, SizeY);
}

SVU3V_RETURN
Camera_setBinningMode(Camera_handle hCamera,
                      BINNING_MODE BinningMode)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_setBinningMode)
  U3VFunc[Func_Camera_setBinningMode].function_pointer)(hCamera, BinningMode);
}

SVU3V_RETURN
Camera_getBinningMode(Camera_handle hCamera,
                      BINNING_MODE *ProgrammedBinningMode)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getBinningMode)
  U3VFunc[Func_Camera_getBinningMode].function_pointer)(hCamera, ProgrammedBinningMode);
}

SVU3V_RETURN
Camera_setAreaOfInterest(Camera_handle hCamera,
                         int SizeX,
                         int SizeY,
                         int OffsetX,
                         int OffsetY)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_setAreaOfInterest)
  U3VFunc[Func_Camera_setAreaOfInterest].function_pointer)(hCamera, SizeX, SizeY, OffsetX, OffsetY);
}

SVU3V_RETURN
Camera_getAreaOfInterest(Camera_handle hCamera,
                         int *ProgrammedSizeX,
                         int *ProgrammedSizeY,
                         int *ProgrammedOffsetX,
                         int *ProgrammedOffsetY)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getAreaOfInterest)
  U3VFunc[Func_Camera_getAreaOfInterest].function_pointer)(hCamera, ProgrammedSizeX, ProgrammedSizeY, ProgrammedOffsetX, ProgrammedOffsetY);
}

SVU3V_RETURN
Camera_getAreaOfInterestRange(Camera_handle hCamera,
                              int *MinSizeX,
                              int *MinSizeY,
                              int *MaxSizeX,
                              int *MaxSizeY)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getAreaOfInterestRange)
  U3VFunc[Func_Camera_getAreaOfInterestRange].function_pointer)(hCamera, MinSizeX, MinSizeY, MaxSizeX, MaxSizeY);
}

SVU3V_RETURN
Camera_getAreaOfInterestIncrement(Camera_handle hCamera,
                                  int *SizeXIncrement,
                                  int *SizeYIncrement,
                                  int *OffsetXIncrement,
                                  int *OffsetYIncrement)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getAreaOfInterestIncrement)
  U3VFunc[Func_Camera_getAreaOfInterestIncrement].function_pointer)(hCamera, SizeXIncrement, SizeYIncrement, OffsetXIncrement, OffsetYIncrement);
}


SVU3V_RETURN
Camera_setFlippingMode(Camera_handle hCamera, 
						SVU3V_FLIPPING_MODE  FlippingMode)
{
 // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_setFlippingMode)
  U3VFunc[Func_Camera_setFlippingMode].function_pointer)(hCamera, FlippingMode);
}

SVU3V_RETURN
Camera_getFlippingMode(Camera_handle hCamera,
                        SVU3V_FLIPPING_MODE *ProgrammedFlippingMode) 
{
	 // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getFlippingMode)
  U3VFunc[Func_Camera_getFlippingMode].function_pointer)(hCamera, ProgrammedFlippingMode);
}

SVU3V_RETURN
Camera_setShutterMode(Camera_handle hCamera,
                        SVU3V_SHUTTER_MODE  ShutterMode)
{
	// Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_setShutterMode)
  U3VFunc[Func_Camera_setShutterMode].function_pointer)(hCamera, ShutterMode);
 }

 SVU3V_RETURN
Camera_getShutterMode(Camera_handle hCamera,
                        SVU3V_SHUTTER_MODE *ProgrammedShutterMode)  
{
		 // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getShutterMode)
  U3VFunc[Func_Camera_getShutterMode].function_pointer)(hCamera, ProgrammedShutterMode);
}

//*****************************************************************************************
// 18 - Controlling camera: Image appearance
//*****************************************************************************************

SVU3V_RETURN
Camera_getPixelType(Camera_handle hCamera,
                    GVSP_PIXEL_TYPE *PixelType)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getPixelType)
  U3VFunc[Func_Camera_getPixelType].function_pointer)(hCamera, PixelType);
}

SVU3V_RETURN
Camera_setPixelDepth(Camera_handle hCamera,
                     SVU3V_PIXEL_DEPTH PixelDepth)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_setPixelDepth)
  U3VFunc[Func_Camera_setPixelDepth].function_pointer)(hCamera, PixelDepth);
}

SVU3V_RETURN
Camera_getPixelDepth(Camera_handle hCamera,
                     SVU3V_PIXEL_DEPTH *PixelDepth)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getPixelDepth)
  U3VFunc[Func_Camera_getPixelDepth].function_pointer)(hCamera, PixelDepth);
}



SVU3V_RETURN
Camera_setWhiteBalance(Camera_handle hCamera,
                       float Red,
                       float Green,
                       float Blue)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_setWhiteBalance)
  U3VFunc[Func_Camera_setWhiteBalance].function_pointer)(hCamera, Red, Green, Blue);
}

SVU3V_RETURN
Camera_getWhiteBalance(Camera_handle hCamera,
                       float *Red,
                       float *Green,
                       float *Blue)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getWhiteBalance)
  U3VFunc[Func_Camera_getWhiteBalance].function_pointer)(hCamera, Red, Green, Blue);
}

SVU3V_RETURN
Camera_getWhiteBalanceMax(Camera_handle hCamera,
                            float *WhiteBalanceMax)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getWhiteBalanceMax)
  U3VFunc[Func_Camera_getWhiteBalanceMax].function_pointer)(hCamera, WhiteBalanceMax);
}

SVU3V_RETURN
Camera_setGammaCorrection(Camera_handle hCamera,
  										    float GammaCorrection)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_setGammaCorrection)
  U3VFunc[Func_Camera_setGammaCorrection].function_pointer)(hCamera, GammaCorrection);
}

SVU3V_RETURN
Camera_setGammaCorrectionExt(Camera_handle hCamera,
  													 float GammaCorrection,
                             float DigitalGain,
                             float DigitalOffset)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_setGammaCorrectionExt)
  U3VFunc[Func_Camera_setGammaCorrectionExt].function_pointer)(hCamera, GammaCorrection, DigitalGain, DigitalOffset);
}

SVU3V_RETURN
Camera_setLowpassFilter(Camera_handle hCamera,
                        LOWPASS_FILTER LowpassFilter)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_setLowpassFilter)
  U3VFunc[Func_Camera_setLowpassFilter].function_pointer)(hCamera, LowpassFilter);
}

SVU3V_RETURN
Camera_getLowpassFilter(Camera_handle hCamera,
                        LOWPASS_FILTER *ProgrammedLowpassFilter)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getLowpassFilter)
  U3VFunc[Func_Camera_getLowpassFilter].function_pointer)(hCamera, ProgrammedLowpassFilter);
}

SVU3V_RETURN
Camera_setLookupTableMode(Camera_handle hCamera,
                          LUT_MODE LUTMode)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_setLookupTableMode)
  U3VFunc[Func_Camera_setLookupTableMode].function_pointer)(hCamera, LUTMode);
}

SVU3V_RETURN
Camera_getLookupTableMode(Camera_handle hCamera,
                          LUT_MODE *ProgrammedLUTMode)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getLookupTableMode)
  U3VFunc[Func_Camera_getLookupTableMode].function_pointer)(hCamera, ProgrammedLUTMode);
}

SVU3V_RETURN
Camera_setLookupTable(Camera_handle hCamera,
                      unsigned char *LookupTable,
                      int LookupTableSize)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_setLookupTable)
  U3VFunc[Func_Camera_setLookupTable].function_pointer)(hCamera, LookupTable, LookupTableSize);
}

SVU3V_RETURN
Camera_getLookupTable(Camera_handle hCamera,
                      unsigned char *LookupTable,
                      int LookupTableSize)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getLookupTable)
  U3VFunc[Func_Camera_getLookupTable].function_pointer)(hCamera, LookupTable, LookupTableSize);
}

SVU3V_RETURN
Camera_startImageCorrection(Camera_handle hCamera,
                            IMAGE_CORRECTION_STEP ImageCorrectionStep)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_startImageCorrection)
  U3VFunc[Func_Camera_startImageCorrection].function_pointer)(hCamera, ImageCorrectionStep);
}

 SVU3V_RETURN
Camera_isIdleImageCorrection(Camera_handle hCamera,
														 IMAGE_CORRECTION_STEP *ProgrammedImageCorrectionStep,
			 											 bool *isIdle)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_isIdleImageCorrection)
  U3VFunc[Func_Camera_isIdleImageCorrection].function_pointer)(hCamera, ProgrammedImageCorrectionStep, isIdle);
}

 SVU3V_RETURN
Camera_setImageCorrection(Camera_handle hCamera,
													IMAGE_CORRECTION_MODE ImageCorrectionMode)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_setImageCorrection)
  U3VFunc[Func_Camera_setImageCorrection].function_pointer)(hCamera, ImageCorrectionMode);
}

 SVU3V_RETURN
Camera_getImageCorrection(Camera_handle hCamera,
													IMAGE_CORRECTION_MODE *ProgrammedImageCorrectionMode)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getImageCorrection)
  U3VFunc[Func_Camera_getImageCorrection].function_pointer)(hCamera, ProgrammedImageCorrectionMode);
}

 

 SVU3V_RETURN
Camera_setPixelsCorrectionMap(Camera_handle hCamera,
							  PIXELS_CORRECTION_MAP_SELECT PixelsCorrectionMap)
{
// Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_setPixelsCorrectionMap)
  U3VFunc[Func_Camera_setPixelsCorrectionMap].function_pointer)(hCamera,PixelsCorrectionMap );
}

 SVU3V_RETURN
Camera_getPixelsCorrectionMap(Camera_handle hCamera,
								PIXELS_CORRECTION_MAP_SELECT * ProgrammedPixelsCorrectionMap)
{
	// Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getPixelsCorrectionMap)
  U3VFunc[Func_Camera_getPixelsCorrectionMap].function_pointer)(hCamera, ProgrammedPixelsCorrectionMap);
}




 SVU3V_RETURN
Camera_setPixelsCorrectionControlEnabel(Camera_handle hCamera,
										bool isPixelsCorrectionEnabled)
{
	// Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_setPixelsCorrectionControlEnabel)
  U3VFunc[Func_Camera_setPixelsCorrectionControlEnabel].function_pointer)(hCamera, isPixelsCorrectionEnabled);
}

 
	
 SVU3V_RETURN
Camera_getPixelsCorrectionControlEnabel(Camera_handle hCamera,
										 bool *isPixelsCorrectionEnabled)
{
	// Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getPixelsCorrectionControlEnabel)
  U3VFunc[Func_Camera_getPixelsCorrectionControlEnabel].function_pointer)(hCamera,isPixelsCorrectionEnabled );
}

	 
 SVU3V_RETURN
Camera_setPixelsCorrectionControlMark(Camera_handle hCamera,
									  bool isPixelsCorrectionMarked)
{
	// Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_setPixelsCorrectionControlMark)
  U3VFunc[Func_Camera_setPixelsCorrectionControlMark].function_pointer)(hCamera,isPixelsCorrectionMarked );
}


	
 SVU3V_RETURN
Camera_getPixelsCorrectionControlMark(Camera_handle hCamera,
									   bool *isPixelsCorrectionMarked)
{
	// Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getPixelsCorrectionControlMark)
  U3VFunc[Func_Camera_getPixelsCorrectionControlMark].function_pointer)(hCamera,isPixelsCorrectionMarked );
}


 SVU3V_RETURN
Camera_setPixelsCorrectionMapOffset(Camera_handle hCamera,
									 int  OffsetX,   int  OffsetY)
{
	// Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_setPixelsCorrectionMapOffset)
  U3VFunc[Func_Camera_setPixelsCorrectionMapOffset].function_pointer)(hCamera, OffsetX, OffsetY );
}

 SVU3V_RETURN
Camera_getPixelsCorrectionMapOffset(Camera_handle hCamera,
									  int *ProgrammedOffsetX, int *ProgrammedOffsetY)
{
	// Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getPixelsCorrectionMapOffset)
  U3VFunc[Func_Camera_getPixelsCorrectionMapOffset].function_pointer)(hCamera, ProgrammedOffsetX,ProgrammedOffsetY );
}
									 
  
 SVU3V_RETURN
Camera_getPixelsCorrectionMapSize(Camera_handle hCamera,
								   unsigned int *programmedMapSize)
{
	// Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getPixelsCorrectionMapSize)
  U3VFunc[Func_Camera_getPixelsCorrectionMapSize].function_pointer)(hCamera, programmedMapSize );
}

  
 SVU3V_RETURN
Camera_getMaximalPixelsCorrectionMapSize(Camera_handle hCamera,
										 unsigned int *MaximalprogrammedMapSize)
{
	// Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getMaximalPixelsCorrectionMapSize)
  U3VFunc[Func_Camera_getMaximalPixelsCorrectionMapSize].function_pointer)(hCamera, MaximalprogrammedMapSize );
}
 

 SVU3V_RETURN
Camera_setMapIndexCoordinate(Camera_handle hCamera,
									unsigned int MapIndex, 
									unsigned int CoordinateX, unsigned int CoordinateY )
{
	// Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_setMapIndexCoordinate)
  U3VFunc[Func_Camera_setMapIndexCoordinate].function_pointer)(hCamera, MapIndex,  CoordinateX, CoordinateY );
}
									

 SVU3V_RETURN
Camera_getMapIndexCoordinate(Camera_handle hCamera,
									unsigned int MapIndex, 
									unsigned int *ProgrammedCoordinateX, unsigned int *ProgrammedCoordinateY )
{
	// Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getMapIndexCoordinate)
  U3VFunc[Func_Camera_getMapIndexCoordinate].function_pointer)(hCamera, MapIndex, ProgrammedCoordinateX, ProgrammedCoordinateY);
}


 SVU3V_RETURN
Camera_deletePixelCoordinateFromMap(Camera_handle hCamera, unsigned int MapIndex)
{
	// Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_deletePixelCoordinateFromMap)
  U3VFunc[Func_Camera_deletePixelCoordinateFromMap].function_pointer)(hCamera, MapIndex );

}	




//*****************************************************************************************
// 19 - Special control: IOMux configuration
//*****************************************************************************************

SVU3V_RETURN
Camera_getMaxIOMuxIN(Camera_handle hCamera,
                     int *MaxIOMuxINSignals)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getMaxIOMuxIN)
  U3VFunc[Func_Camera_getMaxIOMuxIN].function_pointer)(hCamera, MaxIOMuxINSignals);
}

SVU3V_RETURN
Camera_getMaxIOMuxOUT(Camera_handle hCamera,
                      int *MaxIOMuxOUTSignals)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getMaxIOMuxOUT)
  U3VFunc[Func_Camera_getMaxIOMuxOUT].function_pointer)(hCamera, MaxIOMuxOUTSignals);
}

SVU3V_RETURN
Camera_setIOAssignment(Camera_handle hCamera,
                         SVU3V_IOMux_OUT IOMuxOUT,
                         unsigned int SignalIOMuxIN)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_setIOAssignment)
  U3VFunc[Func_Camera_setIOAssignment].function_pointer)(hCamera, IOMuxOUT, SignalIOMuxIN);
}

SVU3V_RETURN
Camera_getIOAssignment(Camera_handle hCamera,
                       SVU3V_IOMux_OUT IOMuxOUT,
                       unsigned int *ProgrammedIOMuxIN)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getIOAssignment)
  U3VFunc[Func_Camera_getIOAssignment].function_pointer)(hCamera, IOMuxOUT, ProgrammedIOMuxIN);
}

//*****************************************************************************************
// 20 - Special control: IO control
//*****************************************************************************************

SVU3V_RETURN
Camera_setIOMuxIN(Camera_handle hCamera,
                  unsigned int VectorIOMuxIN)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_setIOMuxIN)
  U3VFunc[Func_Camera_setIOMuxIN].function_pointer)(hCamera, VectorIOMuxIN);
}

SVU3V_RETURN
Camera_getIOMuxIN(Camera_handle hCamera,
                  unsigned int *ProgrammedVectorIOMuxIN)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getIOMuxIN)
  U3VFunc[Func_Camera_getIOMuxIN].function_pointer)(hCamera, ProgrammedVectorIOMuxIN);
}

SVU3V_RETURN
Camera_setIO(Camera_handle hCamera,
             SVU3V_IOMux_IN SignalIOMuxIN,
             IO_SIGNAL SignalValue)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_setIO)
  U3VFunc[Func_Camera_setIO].function_pointer)(hCamera, SignalIOMuxIN, SignalValue);
}

SVU3V_RETURN
Camera_getIO(Camera_handle hCamera,
             SVU3V_IOMux_IN SignalIOMuxIN,
             IO_SIGNAL *ProgrammedSignalValue)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getIO)
  U3VFunc[Func_Camera_getIO].function_pointer)(hCamera, SignalIOMuxIN, ProgrammedSignalValue);
}

SVU3V_RETURN
Camera_setAcqLEDOverride(Camera_handle hCamera,
                         bool isOverrideActive)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_setAcqLEDOverride)
  U3VFunc[Func_Camera_setAcqLEDOverride].function_pointer)(hCamera, isOverrideActive);
}

SVU3V_RETURN
Camera_getAcqLEDOverride(Camera_handle hCamera,
                         bool *isOverrideActive)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getAcqLEDOverride)
  U3VFunc[Func_Camera_getAcqLEDOverride].function_pointer)(hCamera, isOverrideActive);
}

SVU3V_RETURN
Camera_setLEDIntensity(Camera_handle hCamera,
                       int LEDIntensity)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_setLEDIntensity)
  U3VFunc[Func_Camera_setLEDIntensity].function_pointer)(hCamera, LEDIntensity);
}

SVU3V_RETURN
Camera_getLEDIntensity(Camera_handle hCamera,
                       int *ProgrammedLEDIntensity)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getLEDIntensity)
  U3VFunc[Func_Camera_getLEDIntensity].function_pointer)(hCamera, ProgrammedLEDIntensity);
}


//*****************************************************************************************
// 21 - Special control: Serial communication
//*****************************************************************************************

SVU3V_RETURN
Camera_setUARTBuffer(Camera_handle hCamera,
                     unsigned char *Data,
                     int DataLength)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_setUARTBuffer)
  U3VFunc[Func_Camera_setUARTBuffer].function_pointer)(hCamera, Data, DataLength);
}

SVU3V_RETURN
Camera_getUARTBuffer(Camera_handle hCamera,
                     unsigned char *Data,
                     int *DataLengthReceived,
                     int DataLengthMax,
                     float Timeout)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getUARTBuffer)
  U3VFunc[Func_Camera_getUARTBuffer].function_pointer)(hCamera, Data, DataLengthReceived, DataLengthMax, Timeout);
}

SVU3V_RETURN
Camera_setUARTBaud(Camera_handle hCamera,
                   SVU3V_BaudRate BaudRate)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_setUARTBaud)
  U3VFunc[Func_Camera_setUARTBaud].function_pointer)(hCamera, BaudRate);
}

SVU3V_RETURN
Camera_getUARTBaud(Camera_handle hCamera,
                   SVU3V_BaudRate *ProgrammedBaudRate)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getUARTBaud)
  U3VFunc[Func_Camera_getUARTBaud].function_pointer)(hCamera, ProgrammedBaudRate);
}

//*****************************************************************************************
// 22 - Special control: Direct register and memory access
//*****************************************************************************************

SVU3V_RETURN
Camera_setU3VCameraRegister(Camera_handle hCamera,
                             unsigned int RegisterAddress,
                             unsigned int RegisterValue,
                             unsigned int U3VCameraAccessKey)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_setU3VCameraRegister)
  U3VFunc[Func_Camera_setU3VCameraRegister].function_pointer)(hCamera, RegisterAddress, RegisterValue, U3VCameraAccessKey);
}

SVU3V_RETURN
Camera_getU3VCameraRegister(Camera_handle hCamera,
                             unsigned int RegisterAddress,
                             unsigned int *ProgramedRegisterValue,
                             unsigned int U3VCameraAccessKey)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getU3VCameraRegister)
  U3VFunc[Func_Camera_getU3VCameraRegister].function_pointer)(hCamera, RegisterAddress, ProgramedRegisterValue, U3VCameraAccessKey);
}

SVU3V_RETURN
Camera_writeU3VCameraMemory(Camera_handle hCamera,
                             unsigned int  MemoryAddress,
                             unsigned char *DataBlock,
                             unsigned int  DataLength,
                             unsigned int  U3VCameraAccessKey)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_writeSVU3VCameraMemory)
  U3VFunc[Func_Camera_writeU3VCameraMemory].function_pointer)(hCamera, MemoryAddress, DataBlock, DataLength, U3VCameraAccessKey);
}

SVU3V_RETURN
Camera_readU3VCameraMemory(Camera_handle hCamera,
                            unsigned int  MemoryAddress,
                            unsigned char *DataBlock,
                            unsigned int  DataLength,
                            unsigned int  U3VCameraAccessKey)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_readSVU3VCameraMemory)
  U3VFunc[Func_Camera_readU3VCameraMemory].function_pointer)(hCamera, MemoryAddress, DataBlock, DataLength, U3VCameraAccessKey);
}



//*****************************************************************************************
// 23 - Special control: Persistent settings and recovery
//*****************************************************************************************

SVU3V_RETURN
Camera_writeEEPROM(Camera_handle hCamera)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_writeEEPROM)
  U3VFunc[Func_Camera_writeEEPROM].function_pointer)(hCamera);
}

SVU3V_RETURN
Camera_readEEPROM(Camera_handle hCamera)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_readEEPROM)
  U3VFunc[Func_Camera_readEEPROM].function_pointer)(hCamera);
}

SVU3V_RETURN
Camera_restoreFactoryDefaults(Camera_handle hCamera)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_restoreFactoryDefaults)
  U3VFunc[Func_Camera_restoreFactoryDefaults].function_pointer)(hCamera);
}


SVU3V_RETURN
Camera_loadSettingsFromXml(Camera_handle hCamera,
                           char *Filename)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_loadSettingsFromXml)
  U3VFunc[Func_Camera_loadSettingsFromXml].function_pointer)(hCamera,
                                Filename);
}


SVU3V_RETURN
Camera_SaveSettingsToXml(Camera_handle hCamera,
                         char *Filename)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_SaveSettingsToXml)
  U3VFunc[Func_Camera_SaveSettingsToXml].function_pointer)(hCamera,Filename );

}


//*****************************************************************************************
// 24 - General functions
//*****************************************************************************************

SVU3V_RETURN 
SVU3V_estimateWhiteBalance(unsigned char *DataRGB, int DataRGBLength, float *Red, float *Green, float *Blue)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_SVU3V_estimateWhiteBalance)
  U3VFunc[Func_SVU3V_estimateWhiteBalance].function_pointer)(DataRGB, DataRGBLength, Red, Green, Blue);
}

SVU3V_RETURN 
SVU3V_estimateWhiteBalanceExtended(unsigned char *DataRGB, int PixelNumber, int &Red, int &Green, int &Blue,SVU3V_Whitebalance_SELECT  Whitebalance_Art )
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_SVU3V_estimateWhiteBalanceExtended)
  U3VFunc[Func_SVU3V_estimateWhiteBalanceExtended].function_pointer)(DataRGB, PixelNumber, Red, Green, Blue,Whitebalance_Art);
}


SVU3V_RETURN 
SVU3V_writeImageToBitmapFile(char *Filename, unsigned char *Data, int SizeX, int SizeY, GVSP_PIXEL_TYPE PixelType)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_SVU3V_writeImageToBitmapFile)
  U3VFunc[Func_SVU3V_writeImageToBitmapFile].function_pointer)(Filename, Data, SizeX, SizeY, PixelType);
}


//*****************************************************************************************
// 25 - Lens Control
//*****************************************************************************************


SVU3V_RETURN Camera_isLensAvailable(Camera_handle hCamera, bool *isAvailable)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_isLensAvailable)
  U3VFunc[Func_Camera_isLensAvailable].function_pointer)(hCamera, isAvailable);
}

char* Camera_getLensName(Camera_handle hCamera)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return NULL;

  // Pass through function call to DLL
  return ((TFunc_Camera_getLensName)
  U3VFunc[Func_Camera_getLensName].function_pointer)(hCamera);
}

SVU3V_RETURN	Camera_setLensFocalLenght(Camera_handle hCamera, unsigned int ProgrammedFocalLenght)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_setLensFocalLenght)
  U3VFunc[Func_Camera_setLensFocalLenght].function_pointer)(hCamera, ProgrammedFocalLenght);
}

SVU3V_RETURN	Camera_getLensFocalLenght(Camera_handle hCamera, unsigned int *ProgrammedFocalLenght)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getLensFocalLenght)
  U3VFunc[Func_Camera_getLensFocalLenght].function_pointer)(hCamera, ProgrammedFocalLenght);
}

SVU3V_RETURN Camera_getLensFocalLenghtMin(Camera_handle hCamera, unsigned int *FocalLenghtMin)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getLensFocalLenghtMin)
  U3VFunc[Func_Camera_getLensFocalLenghtMin].function_pointer)(hCamera, FocalLenghtMin);
}

SVU3V_RETURN	Camera_getLensFocalLenghtMax(Camera_handle hCamera, unsigned int *FocalLenghtMax)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getLensFocalLenghtMax)
  U3VFunc[Func_Camera_getLensFocalLenghtMax].function_pointer)(hCamera, FocalLenghtMax);
}


SVU3V_RETURN Camera_setLensFocusUnit(Camera_handle hCamera, FOCUS_UNIT Selected_unit )
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_setLensFocusUnit)
  U3VFunc[Func_Camera_setLensFocusUnit].function_pointer)(hCamera, Selected_unit);
}

SVU3V_RETURN Camera_getLensFocusUnit(Camera_handle hCamera, FOCUS_UNIT *Selected_unit )
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getLensFocusUnit)
  U3VFunc[Func_Camera_getLensFocusUnit].function_pointer)(hCamera, Selected_unit);
}


SVU3V_RETURN Camera_setLensFocus(Camera_handle hCamera, unsigned int ProgrammedFocus)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_setLensFocus)
  U3VFunc[Func_Camera_setLensFocus].function_pointer)(hCamera, ProgrammedFocus);
}

SVU3V_RETURN Camera_getLensFocus(Camera_handle hCamera, unsigned int *ProgrammedFocus)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getLensFocus)
  U3VFunc[Func_Camera_getLensFocus].function_pointer)(hCamera, ProgrammedFocus);
}

SVU3V_RETURN Camera_getLensFocusMin(Camera_handle hCamera, unsigned int *FocusMin)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getLensFocusMin)
  U3VFunc[Func_Camera_getLensFocusMin].function_pointer)(hCamera, FocusMin);
}

SVU3V_RETURN Camera_getLensFocusMax(Camera_handle hCamera, unsigned int *FocusMax)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getLensFocusMax)
  U3VFunc[Func_Camera_getLensFocusMax].function_pointer)(hCamera, FocusMax);
}

SVU3V_RETURN	Camera_setLensAperture(Camera_handle hCamera, unsigned int ProgrammedAperture)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_setLensAperture)
  U3VFunc[Func_Camera_setLensAperture].function_pointer)(hCamera, ProgrammedAperture);
}

SVU3V_RETURN	Camera_getLensAperture(Camera_handle hCamera, unsigned int *ProgrammedAperture)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getLensAperture)
  U3VFunc[Func_Camera_getLensAperture].function_pointer)(hCamera, ProgrammedAperture);
}

SVU3V_RETURN	Camera_getLensApertureMin(Camera_handle hCamera, unsigned int *ApertureMin)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getLensApertureMin)
  U3VFunc[Func_Camera_getLensApertureMin].function_pointer)(hCamera, ApertureMin);
}

SVU3V_RETURN	Camera_getLensApertureMax(Camera_handle hCamera, unsigned int *ApertureMax)
{
  // Check DLL availability
  if( NULL == U3VDLL )
    return U3V_DLL_NOT_LOADED;

  // Pass through function call to DLL
  return ((TFunc_Camera_getLensApertureMax)
  U3VFunc[Func_Camera_getLensApertureMax].function_pointer)(hCamera, ApertureMax);
}



/******************************************************************************************
 * 1394-Based Digital Camera Control Library
 * Bayer pattern decoding functions
 * Copyright (C) Damien Douxchamps <ddouxchamps@users.sf.net>
 *
 * Written by Damien Douxchamps and Frederic Devernay
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *****************************************************************************************/

#ifdef BAYER_CONVERSION_ALGORITHMS

#define CLIP(in, out)\
   in = in < 0 ? 0 : in;\
   in = in > 255 ? 255 : in;\
   out=in;
  
#define CLIP16(in, out, bits)\
   in = in < 0 ? 0 : in;\
   in = in > ((1<<bits)-1) ? ((1<<bits)-1) : in;\
   out=in;
  
void
ClearBorders(unsigned char *rgb, int sx, int sy, int w)
{
    int i, j;
    // black edges are added with a width w:
    i = 3 * sx * w - 1;
    j = 3 * sx * sy - 1;
    while (i >= 0) {
	rgb[i--] = 0;
	rgb[j--] = 0;
    }
    i = sx * (sy - 1) * 3 - 1 + w * 3;
    while (i > sx) {
	j = 6 * w;
	while (j > 0) {
	    rgb[i--] = 0;
	    j--;
	}
	i -= (sx - 2 * w) * 3;
    }
}

void
ClearBorders_uint16(uint16_t * rgb, int sx, int sy, int w)
{
    int i, j;

    // black edges:
    i = 3 * sx * w - 1;
    j = 3 * sx * sy - 1;
    while (i >= 0) {
	rgb[i--] = 0;
	rgb[j--] = 0;
    }

    i = sx * (sy - 1) * 3 - 1 + w * 3;
    while (i > sx) {
	j = 6 * w;
	while (j > 0) {
	    rgb[i--] = 0;
	    j--;
	}
	i -= (sx - 2 * w) * 3;
    }

}

/**************************************************************
 *     Color conversion functions for cameras that can        * 
 * output raw-Bayer pattern images, such as some Basler and   *
 * Point Grey camera. Most of the algos presented here come   *
 * from http://www-ise.stanford.edu/~tingchen/ and have been  *
 * converted from Matlab to C and extended to all elementary  *
 * patterns.                                                  *
 **************************************************************/

/* 8-bits versions */
/* insprired by OpenCV's Bayer decoding */

void
dc1394_bayer_NearestNeighbor(const unsigned char *bayer, unsigned char *rgb, int sx, int sy, int tile)
{
    const int bayerStep = sx;
    const int rgbStep = 3 * sx;
    int width = sx;
    int height = sy;
    int blue = tile == DC1394_COLOR_FILTER_BGGR
	|| tile == DC1394_COLOR_FILTER_GBRG ? -1 : 1;
    int start_with_green = tile == DC1394_COLOR_FILTER_GBRG
	|| tile == DC1394_COLOR_FILTER_GRBG;
    int i, imax, iinc;

    /* add black border */
    imax = sx * sy * 3;
    for (i = sx * (sy - 1) * 3; i < imax; i++) {
	rgb[i] = 0;
    }
    iinc = (sx - 1) * 3;
    for (i = (sx - 1) * 3; i < imax; i += iinc) {
	rgb[i++] = 0;
	rgb[i++] = 0;
	rgb[i++] = 0;
    }

    rgb += 1;
    width -= 1;
    height -= 1;

    for (; height--; bayer += bayerStep, rgb += rgbStep) {
      //int t0, t1;
	const unsigned char *bayerEnd = bayer + width;

        if (start_with_green) {
            rgb[-blue] = bayer[1];
            rgb[0] = bayer[bayerStep + 1];
            rgb[blue] = bayer[bayerStep];
            bayer++;
            rgb += 3;
        }

        if (blue > 0) {
            for (; bayer <= bayerEnd - 2; bayer += 2, rgb += 6) {
                rgb[-1] = bayer[0];
                rgb[0] = bayer[1];
                rgb[1] = bayer[bayerStep + 1];

                rgb[2] = bayer[2];
                rgb[3] = bayer[bayerStep + 2];
                rgb[4] = bayer[bayerStep + 1];
            }
        } else {
            for (; bayer <= bayerEnd - 2; bayer += 2, rgb += 6) {
                rgb[1] = bayer[0];
                rgb[0] = bayer[1];
                rgb[-1] = bayer[bayerStep + 1];

                rgb[4] = bayer[2];
                rgb[3] = bayer[bayerStep + 2];
                rgb[2] = bayer[bayerStep + 1];
            }
        }

        if (bayer < bayerEnd) {
            rgb[-blue] = bayer[0];
            rgb[0] = bayer[1];
            rgb[blue] = bayer[bayerStep + 1];
            bayer++;
            rgb += 3;
        }

	bayer -= width;
	rgb -= width * 3;

	blue = -blue;
	start_with_green = !start_with_green;
    }
}

/* OpenCV's Bayer decoding */
void
dc1394_bayer_Bilinear(const unsigned char *bayer, unsigned char *rgb, int sx, int sy, int tile)
{
    const int bayerStep = sx;
    const int rgbStep = 3 * sx;
    int width = sx;
    int height = sy;
    /*
       the two letters  of the OpenCV name are respectively
       the 4th and 3rd letters from the blinky name,
       and we also have to switch R and B (OpenCV is BGR)

       CV_BayerBG2BGR <-> DC1394_COLOR_FILTER_BGGR
       CV_BayerGB2BGR <-> DC1394_COLOR_FILTER_GBRG
       CV_BayerGR2BGR <-> DC1394_COLOR_FILTER_GRBG

       int blue = tile == CV_BayerBG2BGR || tile == CV_BayerGB2BGR ? -1 : 1;
       int start_with_green = tile == CV_BayerGB2BGR || tile == CV_BayerGR2BGR;
     */
    int blue = tile == DC1394_COLOR_FILTER_BGGR
	|| tile == DC1394_COLOR_FILTER_GBRG ? -1 : 1;
    int start_with_green = tile == DC1394_COLOR_FILTER_GBRG
	|| tile == DC1394_COLOR_FILTER_GRBG;

    ClearBorders(rgb, sx, sy, 1);
    rgb += rgbStep + 3 + 1;
    height -= 2;
    width -= 2;

    for (; height--; bayer += bayerStep, rgb += rgbStep) {
	int t0, t1;
	const unsigned char *bayerEnd = bayer + width;

	if (start_with_green) {
	    /* OpenCV has a bug in the next line, which was
	       t0 = (bayer[0] + bayer[bayerStep * 2] + 1) >> 1; */
	    t0 = (bayer[1] + bayer[bayerStep * 2 + 1] + 1) >> 1;
	    t1 = (bayer[bayerStep] + bayer[bayerStep + 2] + 1) >> 1;
	    rgb[-blue] = (unsigned char) t0;
	    rgb[0] = bayer[bayerStep + 1];
	    rgb[blue] = (unsigned char) t1;
	    bayer++;
	    rgb += 3;
	}

	if (blue > 0) {
	    for (; bayer <= bayerEnd - 2; bayer += 2, rgb += 6) {
		t0 = (bayer[0] + bayer[2] + bayer[bayerStep * 2] +
		      bayer[bayerStep * 2 + 2] + 2) >> 2;
		t1 = (bayer[1] + bayer[bayerStep] +
		      bayer[bayerStep + 2] + bayer[bayerStep * 2 + 1] +
		      2) >> 2;
		rgb[-1] = (unsigned char) t0;
		rgb[0] = (unsigned char) t1;
		rgb[1] = bayer[bayerStep + 1];

		t0 = (bayer[2] + bayer[bayerStep * 2 + 2] + 1) >> 1;
		t1 = (bayer[bayerStep + 1] + bayer[bayerStep + 3] +
		      1) >> 1;
		rgb[2] = (unsigned char) t0;
		rgb[3] = bayer[bayerStep + 2];
		rgb[4] = (unsigned char) t1;
	    }
	} else {
	    for (; bayer <= bayerEnd - 2; bayer += 2, rgb += 6) {
		t0 = (bayer[0] + bayer[2] + bayer[bayerStep * 2] +
		      bayer[bayerStep * 2 + 2] + 2) >> 2;
		t1 = (bayer[1] + bayer[bayerStep] +
		      bayer[bayerStep + 2] + bayer[bayerStep * 2 + 1] +
		      2) >> 2;
		rgb[1] = (unsigned char) t0;
		rgb[0] = (unsigned char) t1;
		rgb[-1] = bayer[bayerStep + 1];

		t0 = (bayer[2] + bayer[bayerStep * 2 + 2] + 1) >> 1;
		t1 = (bayer[bayerStep + 1] + bayer[bayerStep + 3] +
		      1) >> 1;
		rgb[4] = (unsigned char) t0;
		rgb[3] = bayer[bayerStep + 2];
		rgb[2] = (unsigned char) t1;
	    }
	}

	if (bayer < bayerEnd) {
	    t0 = (bayer[0] + bayer[2] + bayer[bayerStep * 2] +
		  bayer[bayerStep * 2 + 2] + 2) >> 2;
	    t1 = (bayer[1] + bayer[bayerStep] +
		  bayer[bayerStep + 2] + bayer[bayerStep * 2 + 1] +
		  2) >> 2;
	    rgb[-blue] = (unsigned char) t0;
	    rgb[0] = (unsigned char) t1;
	    rgb[blue] = bayer[bayerStep + 1];
	    bayer++;
	    rgb += 3;
	}

	bayer -= width;
	rgb -= width * 3;

	blue = -blue;
	start_with_green = !start_with_green;
    }
}

/* High-Quality Linear Interpolation For Demosaicing Of
   Bayer-Patterned Color Images, by Henrique S. Malvar, Li-wei He, and
   Ross Cutler, in ICASSP'04 */
void
dc1394_bayer_HQLinear(const unsigned char *bayer, unsigned char *rgb, int sx, int sy, int tile)
{
    const int bayerStep = sx;
    const int rgbStep = 3 * sx;
    int width = sx;
    int height = sy;
    int blue = tile == DC1394_COLOR_FILTER_BGGR
	|| tile == DC1394_COLOR_FILTER_GBRG ? -1 : 1;
    int start_with_green = tile == DC1394_COLOR_FILTER_GBRG
	|| tile == DC1394_COLOR_FILTER_GRBG;

    ClearBorders(rgb, sx, sy, 2);
    rgb += 2 * rgbStep + 6 + 1;
    height -= 4;
    width -= 4;

    /* We begin with a (+1 line,+1 column) offset with respect to bilinear decoding, so start_with_green is the same, but blue is opposite */
    blue = -blue;

    for (; height--; bayer += bayerStep, rgb += rgbStep) {
	int t0, t1;
	const unsigned char *bayerEnd = bayer + width;
	const int bayerStep2 = bayerStep * 2;
	const int bayerStep3 = bayerStep * 3;
	const int bayerStep4 = bayerStep * 4;

	if (start_with_green) {
	    /* at green pixel */
	    rgb[0] = bayer[bayerStep2 + 2];
	    t0 = rgb[0] * 5
		+ ((bayer[bayerStep + 2] + bayer[bayerStep3 + 2]) << 2)
		- bayer[2]
		- bayer[bayerStep + 1]
		- bayer[bayerStep + 3]
		- bayer[bayerStep3 + 1]
		- bayer[bayerStep3 + 3]
		- bayer[bayerStep4 + 2]
		+ ((bayer[bayerStep2] + bayer[bayerStep2 + 4] + 1) >> 1);
	    t1 = rgb[0] * 5 +
		((bayer[bayerStep2 + 1] + bayer[bayerStep2 + 3]) << 2)
		- bayer[bayerStep2]
		- bayer[bayerStep + 1]
		- bayer[bayerStep + 3]
		- bayer[bayerStep3 + 1]
		- bayer[bayerStep3 + 3]
		- bayer[bayerStep2 + 4]
		+ ((bayer[2] + bayer[bayerStep4 + 2] + 1) >> 1);
	    t0 = (t0 + 4) >> 3;
	    CLIP(t0, rgb[-blue]);
	    t1 = (t1 + 4) >> 3;
	    CLIP(t1, rgb[blue]);
	    bayer++;
	    rgb += 3;
	}

	if (blue > 0) {
	    for (; bayer <= bayerEnd - 2; bayer += 2, rgb += 6) {
		/* B at B */
		rgb[1] = bayer[bayerStep2 + 2];
		/* R at B */
		t0 = ((bayer[bayerStep + 1] + bayer[bayerStep + 3] +
		       bayer[bayerStep3 + 1] + bayer[bayerStep3 + 3]) << 1)
		    -
		    (((bayer[2] + bayer[bayerStep2] +
		       bayer[bayerStep2 + 4] + bayer[bayerStep4 +
						     2]) * 3 + 1) >> 1)
		    + rgb[1] * 6;
		/* G at B */
		t1 = ((bayer[bayerStep + 2] + bayer[bayerStep2 + 1] +
		       bayer[bayerStep2 + 3] + bayer[bayerStep3 + 2]) << 1)
		    - (bayer[2] + bayer[bayerStep2] +
		       bayer[bayerStep2 + 4] + bayer[bayerStep4 + 2])
		    + (rgb[1] << 2);
		t0 = (t0 + 4) >> 3;
		CLIP(t0, rgb[-1]);
		t1 = (t1 + 4) >> 3;
		CLIP(t1, rgb[0]);
		/* at green pixel */
		rgb[3] = bayer[bayerStep2 + 3];
		t0 = rgb[3] * 5
		    + ((bayer[bayerStep + 3] + bayer[bayerStep3 + 3]) << 2)
		    - bayer[3]
		    - bayer[bayerStep + 2]
		    - bayer[bayerStep + 4]
		    - bayer[bayerStep3 + 2]
		    - bayer[bayerStep3 + 4]
		    - bayer[bayerStep4 + 3]
		    +
		    ((bayer[bayerStep2 + 1] + bayer[bayerStep2 + 5] +
		      1) >> 1);
		t1 = rgb[3] * 5 +
		    ((bayer[bayerStep2 + 2] + bayer[bayerStep2 + 4]) << 2)
		    - bayer[bayerStep2 + 1]
		    - bayer[bayerStep + 2]
		    - bayer[bayerStep + 4]
		    - bayer[bayerStep3 + 2]
		    - bayer[bayerStep3 + 4]
		    - bayer[bayerStep2 + 5]
		    + ((bayer[3] + bayer[bayerStep4 + 3] + 1) >> 1);
		t0 = (t0 + 4) >> 3;
		CLIP(t0, rgb[2]);
		t1 = (t1 + 4) >> 3;
		CLIP(t1, rgb[4]);
	    }
	} else {
	    for (; bayer <= bayerEnd - 2; bayer += 2, rgb += 6) {
		/* R at R */
		rgb[-1] = bayer[bayerStep2 + 2];
		/* B at R */
		t0 = ((bayer[bayerStep + 1] + bayer[bayerStep + 3] +
		       bayer[bayerStep3 + 1] + bayer[bayerStep3 + 3]) << 1)
		    -
		    (((bayer[2] + bayer[bayerStep2] +
		       bayer[bayerStep2 + 4] + bayer[bayerStep4 +
						     2]) * 3 + 1) >> 1)
		    + rgb[-1] * 6;
		/* G at R */
		t1 = ((bayer[bayerStep + 2] + bayer[bayerStep2 + 1] +
		       bayer[bayerStep2 + 3] + bayer[bayerStep * 3 +
						     2]) << 1)
		    - (bayer[2] + bayer[bayerStep2] +
		       bayer[bayerStep2 + 4] + bayer[bayerStep4 + 2])
		    + (rgb[-1] << 2);
		t0 = (t0 + 4) >> 3;
		CLIP(t0, rgb[1]);
		t1 = (t1 + 4) >> 3;
		CLIP(t1, rgb[0]);

		/* at green pixel */
		rgb[3] = bayer[bayerStep2 + 3];
		t0 = rgb[3] * 5
		    + ((bayer[bayerStep + 3] + bayer[bayerStep3 + 3]) << 2)
		    - bayer[3]
		    - bayer[bayerStep + 2]
		    - bayer[bayerStep + 4]
		    - bayer[bayerStep3 + 2]
		    - bayer[bayerStep3 + 4]
		    - bayer[bayerStep4 + 3]
		    +
		    ((bayer[bayerStep2 + 1] + bayer[bayerStep2 + 5] +
		      1) >> 1);
		t1 = rgb[3] * 5 +
		    ((bayer[bayerStep2 + 2] + bayer[bayerStep2 + 4]) << 2)
		    - bayer[bayerStep2 + 1]
		    - bayer[bayerStep + 2]
		    - bayer[bayerStep + 4]
		    - bayer[bayerStep3 + 2]
		    - bayer[bayerStep3 + 4]
		    - bayer[bayerStep2 + 5]
		    + ((bayer[3] + bayer[bayerStep4 + 3] + 1) >> 1);
		t0 = (t0 + 4) >> 3;
		CLIP(t0, rgb[4]);
		t1 = (t1 + 4) >> 3;
		CLIP(t1, rgb[2]);
	    }
	}

	if (bayer < bayerEnd) {
	    /* B at B */
	    rgb[blue] = bayer[bayerStep2 + 2];
	    /* R at B */
	    t0 = ((bayer[bayerStep + 1] + bayer[bayerStep + 3] +
		   bayer[bayerStep3 + 1] + bayer[bayerStep3 + 3]) << 1)
		-
		(((bayer[2] + bayer[bayerStep2] +
		   bayer[bayerStep2 + 4] + bayer[bayerStep4 +
						 2]) * 3 + 1) >> 1)
		+ rgb[blue] * 6;
	    /* G at B */
	    t1 = (((bayer[bayerStep + 2] + bayer[bayerStep2 + 1] +
		    bayer[bayerStep2 + 3] + bayer[bayerStep3 + 2])) << 1)
		- (bayer[2] + bayer[bayerStep2] +
		   bayer[bayerStep2 + 4] + bayer[bayerStep4 + 2])
		+ (rgb[blue] << 2);
	    t0 = (t0 + 4) >> 3;
	    CLIP(t0, rgb[-blue]);
	    t1 = (t1 + 4) >> 3;
	    CLIP(t1, rgb[0]);
	    bayer++;
	    rgb += 3;
	}

	bayer -= width;
	rgb -= width * 3;

	blue = -blue;
	start_with_green = !start_with_green;
    }
}

/* coriander's Bayer decoding (GPL) */
/* Edge Sensing Interpolation II from http://www-ise.stanford.edu/~tingchen/ */
/*   (Laroche,Claude A.  "Apparatus and method for adaptively
     interpolating a full color image utilizing chrominance gradients"
     U.S. Patent 5,373,322) */
void
dc1394_bayer_EdgeSense(const unsigned char *bayer, unsigned char *rgb, int sx, int sy, int tile)
{
    unsigned char *outR, *outG, *outB;
    register int i, j;
    int dh, dv;
    int tmp;

    // sx and sy should be even
    switch (tile) {
    case DC1394_COLOR_FILTER_GRBG:
    case DC1394_COLOR_FILTER_BGGR:
	outR = &rgb[0];
	outG = &rgb[1];
	outB = &rgb[2];
	break;
    case DC1394_COLOR_FILTER_GBRG:
    case DC1394_COLOR_FILTER_RGGB:
	outR = &rgb[2];
	outG = &rgb[1];
	outB = &rgb[0];
	break;
    default:
 //	fprintf(stderr, "Bad bayer pattern ID: %d\n", tile);
 //	return;
	break;
    }

    switch (tile) {
    case DC1394_COLOR_FILTER_GRBG:	//---------------------------------------------------------
    case DC1394_COLOR_FILTER_GBRG:
	// copy original RGB data to output images
      for (i = 0; i < sy*sx; i += (sx<<1)) {
	for (j = 0; j < sx; j += 2) {
	  outG[(i + j) * 3] = bayer[i + j];
	  outG[(i + sx + (j + 1)) * 3] = bayer[i + sx + (j + 1)];
	  outR[(i + j + 1) * 3] = bayer[i + j + 1];
	  outB[(i + sx + j) * 3] = bayer[i + sx + j];
	}
      }
      // process GREEN channel
      for (i = 3*sx; i < (sy - 2)*sx; i += (sx<<1)) {
	for (j = 2; j < sx - 3; j += 2) {
	  dh = abs(((outB[(i + j - 2) * 3] +
		     outB[(i + j + 2) * 3]) >> 1) -
		   outB[(i + j) * 3]);
	  dv = abs(((outB[(i - (sx<<1) + j) * 3] +
		     outB[(i + (sx<<1) + j) * 3]) >> 1)  -
		   outB[(i + j) * 3]);
	  if (dh < dv)
	    tmp = (outG[(i + j - 1) * 3] +
		   outG[(i + j + 1) * 3]) >> 1;
	  else {
	    if (dh > dv)
	      tmp = (outG[(i - sx + j) * 3] +
		     outG[(i + sx + j) * 3]) >> 1;
	    else
	      tmp = (outG[(i + j - 1) * 3] +
		     outG[(i + j + 1) * 3] +
		     outG[(i - sx + j) * 3] +
		     outG[(i + sx + j) * 3]) >> 2;
	  }
	  CLIP(tmp, outG[(i + j) * 3]);
	}
      }
	
      for (i = 2*sx; i < (sy - 3)*sx; i += (sx<<1)) {
	for (j = 3; j < sx - 2; j += 2) {
	  dh = abs(((outR[(i + j - 2) * 3] +
		     outR[(i + j + 2) * 3]) >>1 ) -
		   outR[(i + j) * 3]);
	  dv = abs(((outR[(i - (sx<<1) + j) * 3] +
		     outR[(i + (sx<<1) + j) * 3]) >>1 ) -
		   outR[(i + j) * 3]);
	  if (dh < dv)
	    tmp = (outG[(i + j - 1) * 3] +
		   outG[(i + j + 1) * 3]) >> 1;
	  else {
	    if (dh > dv)
	      tmp = (outG[(i - sx + j) * 3] +
		     outG[(i + sx + j) * 3]) >> 1;
	    else
	      tmp = (outG[(i + j - 1) * 3] +
		     outG[(i + j + 1) * 3] +
		     outG[(i - sx + j) * 3] +
		     outG[(i + sx + j) * 3]) >> 2;
	  }
	  CLIP(tmp, outG[(i + j) * 3]);
	}
      }
      // process RED channel
      for (i = 0; i < (sy - 1)*sx; i += (sx<<1)) {
	for (j = 2; j < sx - 1; j += 2) {
	  tmp = outG[(i + j) * 3] +
	      ((outR[(i + j - 1) * 3] -
		outG[(i + j - 1) * 3] +
		outR[(i + j + 1) * 3] -
		outG[(i + j + 1) * 3]) >> 1);
	  CLIP(tmp, outR[(i + j) * 3]);
	}
      }
      for (i = sx; i < (sy - 2)*sx; i += (sx<<1)) {
	for (j = 1; j < sx; j += 2) {
	  tmp = outG[(i + j) * 3] +
	      ((outR[(i - sx + j) * 3] -
		outG[(i - sx + j) * 3] +
		outR[(i + sx + j) * 3] -
		outG[(i + sx + j) * 3]) >> 1);
	  CLIP(tmp, outR[(i + j) * 3]);
	}
	for (j = 2; j < sx - 1; j += 2) {
	  tmp = outG[(i + j) * 3] +
	      ((outR[(i - sx + j - 1) * 3] -
		outG[(i - sx + j - 1) * 3] +
		outR[(i - sx + j + 1) * 3] -
		outG[(i - sx + j + 1) * 3] +
		outR[(i + sx + j - 1) * 3] -
		outG[(i + sx + j - 1) * 3] +
		outR[(i + sx + j + 1) * 3] -
		outG[(i + sx + j + 1) * 3]) >> 2);
	  CLIP(tmp, outR[(i + j) * 3]);
	}
      }

      // process BLUE channel
      for (i = sx; i < sy*sx; i += (sx<<1)) {
	for (j = 1; j < sx - 2; j += 2) {
	  tmp = outG[(i + j) * 3] +
	      ((outB[(i + j - 1) * 3] -
		outG[(i + j - 1) * 3] +
		outB[(i + j + 1) * 3] -
		outG[(i + j + 1) * 3]) >> 1);
	  CLIP(tmp, outB[(i + j) * 3]);
	}
      }
      for (i = 2*sx; i < (sy - 1)*sx; i += (sx<<1)) {
	for (j = 0; j < sx - 1; j += 2) {
	  tmp = outG[(i + j) * 3] +
	      ((outB[(i - sx + j) * 3] -
		outG[(i - sx + j) * 3] +
		outB[(i + sx + j) * 3] -
		outG[(i + sx + j) * 3]) >> 1);
	  CLIP(tmp, outB[(i + j) * 3]);
	}
	for (j = 1; j < sx - 2; j += 2) {
	  tmp = outG[(i + j) * 3] +
	      ((outB[(i - sx + j - 1) * 3] -
		outG[(i - sx + j - 1) * 3] +
		outB[(i - sx + j + 1) * 3] -
		outG[(i - sx + j + 1) * 3] +
		outB[(i + sx + j - 1) * 3] -
		outG[(i + sx + j - 1) * 3] +
		outB[(i + sx + j + 1) * 3] -
		outG[(i + sx + j + 1) * 3]) >> 2);
	  CLIP(tmp, outB[(i + j) * 3]);
	}
      }
      break;

    case DC1394_COLOR_FILTER_BGGR:	//---------------------------------------------------------
    case DC1394_COLOR_FILTER_RGGB:
	// copy original RGB data to output images
      for (i = 0; i < sy*sx; i += (sx<<1)) {
	for (j = 0; j < sx; j += 2) {
	  outB[(i + j) * 3] = bayer[i + j];
	  outR[(i + sx + (j + 1)) * 3] = bayer[i + sx + (j + 1)];
	  outG[(i + j + 1) * 3] = bayer[i + j + 1];
	  outG[(i + sx + j) * 3] = bayer[i + sx + j];
	}
      }
      // process GREEN channel
      for (i = 2*sx; i < (sy - 2)*sx; i += (sx<<1)) {
	for (j = 2; j < sx - 3; j += 2) {
	  dh = abs(((outB[(i + j - 2) * 3] +
		    outB[(i + j + 2) * 3]) >> 1) -
		   outB[(i + j) * 3]);
	  dv = abs(((outB[(i - (sx<<1) + j) * 3] +
		    outB[(i + (sx<<1) + j) * 3]) >> 1) -
		   outB[(i + j) * 3]);
	  if (dh < dv)
	    tmp = (outG[(i + j - 1) * 3] +
		   outG[(i + j + 1) * 3]) >> 1;
	  else {
	    if (dh > dv)
	      tmp = (outG[(i - sx + j) * 3] +
		     outG[(i + sx + j) * 3]) >> 1;
	    else
	      tmp = (outG[(i + j - 1) * 3] +
		     outG[(i + j + 1) * 3] +
		     outG[(i - sx + j) * 3] +
		     outG[(i + sx + j) * 3]) >> 2;
	  }
	  CLIP(tmp, outG[(i + j) * 3]);
	}
      }
      for (i = 3*sx; i < (sy - 3)*sx; i += (sx<<1)) {
	for (j = 3; j < sx - 2; j += 2) {
	  dh = abs(((outR[(i + j - 2) * 3] +
		    outR[(i + j + 2) * 3]) >> 1) -
		   outR[(i + j) * 3]);
	  dv = abs(((outR[(i - (sx<<1) + j) * 3] +
		    outR[(i + (sx<<1) + j) * 3]) >> 1) -
		   outR[(i + j) * 3]);
	  if (dh < dv)
	    tmp = (outG[(i + j - 1) * 3] +
		   outG[(i + j + 1) * 3]) >>1;
	  else {
	    if (dh > dv)
	      tmp = (outG[(i - sx + j) * 3] +
		     outG[(i + sx + j) * 3]) >>1;
	    else
	      tmp = (outG[(i + j - 1) * 3] +
		     outG[(i + j + 1) * 3] +
		     outG[(i - sx + j) * 3] +
		     outG[(i + sx + j) * 3]) >>2;
	  }
	  CLIP(tmp, outG[(i + j) * 3]);
	}
      }
      // process RED channel
      for (i = sx; i < (sy - 1)*sx; i += (sx<<1)) {	// G-points (1/2)
	for (j = 2; j < sx - 1; j += 2) {
	  tmp = outG[(i + j) * 3] +
	      ((outR[(i + j - 1) * 3] -
		outG[(i + j - 1) * 3] +
		outR[(i + j + 1) * 3] -
		outG[(i + j + 1) * 3]) >>1);
	  CLIP(tmp, outR[(i + j) * 3]);
	}
      }
      for (i = 2*sx; i < (sy - 2)*sx; i += (sx<<1)) {
	for (j = 1; j < sx; j += 2) {	// G-points (2/2)
	  tmp = outG[(i + j) * 3] +
	      ((outR[(i - sx + j) * 3] -
		outG[(i - sx + j) * 3] +
		outR[(i + sx + j) * 3] -
		outG[(i + sx + j) * 3]) >> 1);
	  CLIP(tmp, outR[(i + j) * 3]);
	}
	for (j = 2; j < sx - 1; j += 2) {	// B-points
	  tmp = outG[(i + j) * 3] +
	      ((outR[(i - sx + j - 1) * 3] -
		outG[(i - sx + j - 1) * 3] +
		outR[(i - sx + j + 1) * 3] -
		outG[(i - sx + j + 1) * 3] +
		outR[(i + sx + j - 1) * 3] -
		outG[(i + sx + j - 1) * 3] +
		outR[(i + sx + j + 1) * 3] -
		outG[(i + sx + j + 1) * 3]) >> 2);
	  CLIP(tmp, outR[(i + j) * 3]);
	}
      }
      
      // process BLUE channel
      for (i = 0; i < sy*sx; i += (sx<<1)) {
	for (j = 1; j < sx - 2; j += 2) {
	  tmp = outG[(i + j) * 3] +
	      ((outB[(i + j - 1) * 3] -
		outG[(i + j - 1) * 3] +
		outB[(i + j + 1) * 3] -
		outG[(i + j + 1) * 3]) >> 1);
	  CLIP(tmp, outB[(i + j) * 3]);
	}
      }
      for (i = sx; i < (sy - 1)*sx; i += (sx<<1)) {
	for (j = 0; j < sx - 1; j += 2) {
	  tmp = outG[(i + j) * 3] +
	      ((outB[(i - sx + j) * 3] -
		outG[(i - sx + j) * 3] +
		outB[(i + sx + j) * 3] -
		outG[(i + sx + j) * 3]) >> 1);
	  CLIP(tmp, outB[(i + j) * 3]);
	}
	for (j = 1; j < sx - 2; j += 2) {
	  tmp = outG[(i + j) * 3] +
	      ((outB[(i - sx + j - 1) * 3] -
		outG[(i - sx + j - 1) * 3] +
		outB[(i - sx + j + 1) * 3] -
		outG[(i - sx + j + 1) * 3] +
		outB[(i + sx + j - 1) * 3] -
		outG[(i + sx + j - 1) * 3] +
		outB[(i + sx + j + 1) * 3] -
		outG[(i + sx + j + 1) * 3]) >> 2);
	  CLIP(tmp, outB[(i + j) * 3]);
	}
      }
      break;
    default:			//---------------------------------------------------------
//      fprintf(stderr, "Bad bayer pattern ID: %d\n", tile);
//      return;
      break;
    }
    
    ClearBorders(rgb, sx, sy, 3);
}

/* coriander's Bayer decoding (GPL) */
void
dc1394_bayer_Downsample(const unsigned char *bayer, unsigned char *rgb, int sx, int sy, int tile)
{
    unsigned char *outR, *outG, *outB;
    register int i, j;
    int tmp;

    sx *= 2;
    sy *= 2;

    switch (tile) {
    case DC1394_COLOR_FILTER_GRBG:
    case DC1394_COLOR_FILTER_BGGR:
	outR = &rgb[0];
	outG = &rgb[1];
	outB = &rgb[2];
	break;
    case DC1394_COLOR_FILTER_GBRG:
    case DC1394_COLOR_FILTER_RGGB:
	outR = &rgb[2];
	outG = &rgb[1];
	outB = &rgb[0];
	break;
    default:
//	fprintf(stderr, "Bad Bayer pattern ID: %d\n", tile);
//	return;
	break;
    }

    switch (tile) {
    case DC1394_COLOR_FILTER_GRBG:	//---------------------------------------------------------
    case DC1394_COLOR_FILTER_GBRG:
	for (i = 0; i < sy*sx; i += (sx<<1)) {
	    for (j = 0; j < sx; j += 2) {
		tmp =
		    ((bayer[i + j] + bayer[i + sx + j + 1]) >> 1);
		CLIP(tmp, outG[((i >> 2) + (j >> 1)) * 3]);
		tmp = bayer[i + sx + j + 1];
		CLIP(tmp, outR[((i >> 2) + (j >> 1)) * 3]);
		tmp = bayer[i + sx + j];
		CLIP(tmp, outB[((i >> 2) + (j >> 1)) * 3]);
	    }
	}
	break;
    case DC1394_COLOR_FILTER_BGGR:	//---------------------------------------------------------
    case DC1394_COLOR_FILTER_RGGB:
	for (i = 0; i < sy*sx; i += (sx<<1)) {
	    for (j = 0; j < sx; j += 2) {
		tmp =
		    ((bayer[i + sx + j] + bayer[i + j + 1]) >> 1);
		CLIP(tmp, outG[((i >> 2) + (j >> 1)) * 3]);
		tmp = bayer[i + sx + j + 1];
		CLIP(tmp, outR[((i >> 2) + (j >> 1)) * 3]);
		tmp = bayer[i + j];
		CLIP(tmp, outB[((i >> 2) + (j >> 1)) * 3]);
	    }
	}
	break;
    default:			//---------------------------------------------------------
//	fprintf(stderr, "Bad Bayer pattern ID: %d\n", tile);
//	return;
	break;
    }

}

/* this is the method used inside AVT cameras. See AVT docs. */
void
dc1394_bayer_Simple(const unsigned char *bayer, unsigned char *rgb, int sx, int sy, int tile)
{
    const int bayerStep = sx;
    const int rgbStep = 3 * sx;
    int width = sx;
    int height = sy;
    int blue = tile == DC1394_COLOR_FILTER_BGGR
        || tile == DC1394_COLOR_FILTER_GBRG ? -1 : 1;
    int start_with_green = tile == DC1394_COLOR_FILTER_GBRG
        || tile == DC1394_COLOR_FILTER_GRBG;
    int i, imax, iinc;

    /* add black border */
    imax = sx * sy * 3;
    for (i = sx * (sy - 1) * 3; i < imax; i++) {
        rgb[i] = 0;
    }
    iinc = (sx - 1) * 3;
    for (i = (sx - 1) * 3; i < imax; i += iinc) {
        rgb[i++] = 0;
        rgb[i++] = 0;
        rgb[i++] = 0;
    }

    rgb += 1;
    width -= 1;
    height -= 1;

    for (; height--; bayer += bayerStep, rgb += rgbStep) {
        const unsigned char *bayerEnd = bayer + width;

        if (start_with_green) {
            rgb[-blue] = bayer[1];
            rgb[0] = (bayer[0] + bayer[bayerStep + 1] + 1) >> 1;
            rgb[blue] = bayer[bayerStep];
            bayer++;
            rgb += 3;
        }

        if (blue > 0) {
            for (; bayer <= bayerEnd - 2; bayer += 2, rgb += 6) {
                rgb[-1] = bayer[0];
                rgb[0] = (bayer[1] + bayer[bayerStep] + 1) >> 1;
                rgb[1] = bayer[bayerStep + 1];

                rgb[2] = bayer[2];
                rgb[3] = (bayer[1] + bayer[bayerStep + 2] + 1) >> 1;
                rgb[4] = bayer[bayerStep + 1];
            }
        } else {
            for (; bayer <= bayerEnd - 2; bayer += 2, rgb += 6) {
                rgb[1] = bayer[0];
                rgb[0] = (bayer[1] + bayer[bayerStep] + 1) >> 1;
                rgb[-1] = bayer[bayerStep + 1];

                rgb[4] = bayer[2];
                rgb[3] = (bayer[1] + bayer[bayerStep + 2] + 1) >> 1;
                rgb[2] = bayer[bayerStep + 1];
            }
        }

        if (bayer < bayerEnd) {
            rgb[-blue] = bayer[0];
            rgb[0] = (bayer[1] + bayer[bayerStep] + 1) >> 1;
            rgb[blue] = bayer[bayerStep + 1];
            bayer++;
            rgb += 3;
        }

        bayer -= width;
        rgb -= width * 3;

        blue = -blue;
        start_with_green = !start_with_green;
    }
}

/* 16-bits versions */

/* insprired by OpenCV's Bayer decoding */
void
dc1394_bayer_NearestNeighbor_uint16(const uint16_t *bayer, uint16_t *rgb, int sx, int sy, int tile, int bits)
{
    const int bayerStep = sx;
    const int rgbStep = 3 * sx;
    int width = sx;
    int height = sy;
    int blue = tile == DC1394_COLOR_FILTER_BGGR
	|| tile == DC1394_COLOR_FILTER_GBRG ? -1 : 1;
    int start_with_green = tile == DC1394_COLOR_FILTER_GBRG
	|| tile == DC1394_COLOR_FILTER_GRBG;
    int i, iinc, imax;

    /* add black border */
    imax = sx * sy * 3;
    for (i = sx * (sy - 1) * 3; i < imax; i++) {
	rgb[i] = 0;
    }
    iinc = (sx - 1) * 3;
    for (i = (sx - 1) * 3; i < imax; i += iinc) {
	rgb[i++] = 0;
	rgb[i++] = 0;
	rgb[i++] = 0;
    }

    rgb += 1;
    height -= 1;
    width -= 1;

    for (; height--; bayer += bayerStep, rgb += rgbStep) {
      //int t0, t1;
	const uint16_t *bayerEnd = bayer + width;

        if (start_with_green) {
            rgb[-blue] = bayer[1];
            rgb[0] = bayer[bayerStep + 1];
            rgb[blue] = bayer[bayerStep];
            bayer++;
            rgb += 3;
        }

        if (blue > 0) {
            for (; bayer <= bayerEnd - 2; bayer += 2, rgb += 6) {
                rgb[-1] = bayer[0];
                rgb[0] = bayer[1];
                rgb[1] = bayer[bayerStep + 1];

                rgb[2] = bayer[2];
                rgb[3] = bayer[bayerStep + 2];
                rgb[4] = bayer[bayerStep + 1];
            }
        } else {
            for (; bayer <= bayerEnd - 2; bayer += 2, rgb += 6) {
                rgb[1] = bayer[0];
                rgb[0] = bayer[1];
                rgb[-1] = bayer[bayerStep + 1];

                rgb[4] = bayer[2];
                rgb[3] = bayer[bayerStep + 2];
                rgb[2] = bayer[bayerStep + 1];
            }
        }

        if (bayer < bayerEnd) {
            rgb[-blue] = bayer[0];
            rgb[0] = bayer[1];
            rgb[blue] = bayer[bayerStep + 1];
            bayer++;
            rgb += 3;
        }

	bayer -= width;
	rgb -= width * 3;

	blue = -blue;
	start_with_green = !start_with_green;
    }
}
/* OpenCV's Bayer decoding */
void
dc1394_bayer_Bilinear_uint16(const uint16_t *bayer, uint16_t *rgb, int sx, int sy, int tile, int bits)
{
    const int bayerStep = sx;
    const int rgbStep = 3 * sx;
    int width = sx;
    int height = sy;
    int blue = tile == DC1394_COLOR_FILTER_BGGR
	|| tile == DC1394_COLOR_FILTER_GBRG ? -1 : 1;
    int start_with_green = tile == DC1394_COLOR_FILTER_GBRG
	|| tile == DC1394_COLOR_FILTER_GRBG;

    rgb += rgbStep + 3 + 1;
    height -= 2;
    width -= 2;

    for (; height--; bayer += bayerStep, rgb += rgbStep) {
	int t0, t1;
	const uint16_t *bayerEnd = bayer + width;

	if (start_with_green) {
	    /* OpenCV has a bug in the next line, which was
	       t0 = (bayer[0] + bayer[bayerStep * 2] + 1) >> 1; */
	    t0 = (bayer[1] + bayer[bayerStep * 2 + 1] + 1) >> 1;
	    t1 = (bayer[bayerStep] + bayer[bayerStep + 2] + 1) >> 1;
	    rgb[-blue] = (uint16_t) t0;
	    rgb[0] = bayer[bayerStep + 1];
	    rgb[blue] = (uint16_t) t1;
	    bayer++;
	    rgb += 3;
	}

	if (blue > 0) {
	    for (; bayer <= bayerEnd - 2; bayer += 2, rgb += 6) {
		t0 = (bayer[0] + bayer[2] + bayer[bayerStep * 2] +
		      bayer[bayerStep * 2 + 2] + 2) >> 2;
		t1 = (bayer[1] + bayer[bayerStep] +
		      bayer[bayerStep + 2] + bayer[bayerStep * 2 + 1] +
		      2) >> 2;
		rgb[-1] = (uint16_t) t0;
		rgb[0] = (uint16_t) t1;
		rgb[1] = bayer[bayerStep + 1];

		t0 = (bayer[2] + bayer[bayerStep * 2 + 2] + 1) >> 1;
		t1 = (bayer[bayerStep + 1] + bayer[bayerStep + 3] +
		      1) >> 1;
		rgb[2] = (uint16_t) t0;
		rgb[3] = bayer[bayerStep + 2];
		rgb[4] = (uint16_t) t1;
	    }
	} else {
	    for (; bayer <= bayerEnd - 2; bayer += 2, rgb += 6) {
		t0 = (bayer[0] + bayer[2] + bayer[bayerStep * 2] +
		      bayer[bayerStep * 2 + 2] + 2) >> 2;
		t1 = (bayer[1] + bayer[bayerStep] +
		      bayer[bayerStep + 2] + bayer[bayerStep * 2 + 1] +
		      2) >> 2;
		rgb[1] = (uint16_t) t0;
		rgb[0] = (uint16_t) t1;
		rgb[-1] = bayer[bayerStep + 1];

		t0 = (bayer[2] + bayer[bayerStep * 2 + 2] + 1) >> 1;
		t1 = (bayer[bayerStep + 1] + bayer[bayerStep + 3] +
		      1) >> 1;
		rgb[4] = (uint16_t) t0;
		rgb[3] = bayer[bayerStep + 2];
		rgb[2] = (uint16_t) t1;
	    }
	}

	if (bayer < bayerEnd) {
	    t0 = (bayer[0] + bayer[2] + bayer[bayerStep * 2] +
		  bayer[bayerStep * 2 + 2] + 2) >> 2;
	    t1 = (bayer[1] + bayer[bayerStep] +
		  bayer[bayerStep + 2] + bayer[bayerStep * 2 + 1] +
		  2) >> 2;
	    rgb[-blue] = (uint16_t) t0;
	    rgb[0] = (uint16_t) t1;
	    rgb[blue] = bayer[bayerStep + 1];
	    bayer++;
	    rgb += 3;
	}

	bayer -= width;
	rgb -= width * 3;

	blue = -blue;
	start_with_green = !start_with_green;
    }
}

/* High-Quality Linear Interpolation For Demosaicing Of
   Bayer-Patterned Color Images, by Henrique S. Malvar, Li-wei He, and
   Ross Cutler, in ICASSP'04 */
void
dc1394_bayer_HQLinear_uint16(const uint16_t *bayer, uint16_t *rgb, int sx, int sy, int tile, int bits)
{
    const int bayerStep = sx;
    const int rgbStep = 3 * sx;
    int width = sx;
    int height = sy;
    /*
       the two letters  of the OpenCV name are respectively
       the 4th and 3rd letters from the blinky name,
       and we also have to switch R and B (OpenCV is BGR)

       CV_BayerBG2BGR <-> DC1394_COLOR_FILTER_BGGR
       CV_BayerGB2BGR <-> DC1394_COLOR_FILTER_GBRG
       CV_BayerGR2BGR <-> DC1394_COLOR_FILTER_GRBG

       int blue = tile == CV_BayerBG2BGR || tile == CV_BayerGB2BGR ? -1 : 1;
       int start_with_green = tile == CV_BayerGB2BGR || tile == CV_BayerGR2BGR;
     */
    int blue = tile == DC1394_COLOR_FILTER_BGGR
	|| tile == DC1394_COLOR_FILTER_GBRG ? -1 : 1;
    int start_with_green = tile == DC1394_COLOR_FILTER_GBRG
	|| tile == DC1394_COLOR_FILTER_GRBG;

    ClearBorders_uint16(rgb, sx, sy, 2);
    rgb += 2 * rgbStep + 6 + 1;
    height -= 4;
    width -= 4;

    /* We begin with a (+1 line,+1 column) offset with respect to bilinear decoding, so start_with_green is the same, but blue is opposite */
    blue = -blue;

    for (; height--; bayer += bayerStep, rgb += rgbStep) {
	int t0, t1;
	const uint16_t *bayerEnd = bayer + width;
	const int bayerStep2 = bayerStep * 2;
	const int bayerStep3 = bayerStep * 3;
	const int bayerStep4 = bayerStep * 4;

	if (start_with_green) {
	    /* at green pixel */
	    rgb[0] = bayer[bayerStep2 + 2];
	    t0 = rgb[0] * 5
		+ ((bayer[bayerStep + 2] + bayer[bayerStep3 + 2]) << 2)
		- bayer[2]
		- bayer[bayerStep + 1]
		- bayer[bayerStep + 3]
		- bayer[bayerStep3 + 1]
		- bayer[bayerStep3 + 3]
		- bayer[bayerStep4 + 2]
		+ ((bayer[bayerStep2] + bayer[bayerStep2 + 4] + 1) >> 1);
	    t1 = rgb[0] * 5 +
		((bayer[bayerStep2 + 1] + bayer[bayerStep2 + 3]) << 2)
		- bayer[bayerStep2]
		- bayer[bayerStep + 1]
		- bayer[bayerStep + 3]
		- bayer[bayerStep3 + 1]
		- bayer[bayerStep3 + 3]
		- bayer[bayerStep2 + 4]
		+ ((bayer[2] + bayer[bayerStep4 + 2] + 1) >> 1);
	    t0 = (t0 + 4) >> 3;
	    CLIP16(t0, rgb[-blue], bits);
	    t1 = (t1 + 4) >> 3;
	    CLIP16(t1, rgb[blue], bits);
	    bayer++;
	    rgb += 3;
	}

	if (blue > 0) {
	    for (; bayer <= bayerEnd - 2; bayer += 2, rgb += 6) {
		/* B at B */
		rgb[1] = bayer[bayerStep2 + 2];
		/* R at B */
		t0 = ((bayer[bayerStep + 1] + bayer[bayerStep + 3] +
		       bayer[bayerStep3 + 1] + bayer[bayerStep3 + 3]) << 1)
		    -
		    (((bayer[2] + bayer[bayerStep2] +
		       bayer[bayerStep2 + 4] + bayer[bayerStep4 +
						     2]) * 3 + 1) >> 1)
		    + rgb[1] * 6;
		/* G at B */
		t1 = ((bayer[bayerStep + 2] + bayer[bayerStep2 + 1] +
		       bayer[bayerStep2 + 3] + bayer[bayerStep * 3 +
						     2]) << 1)
		    - (bayer[2] + bayer[bayerStep2] +
		       bayer[bayerStep2 + 4] + bayer[bayerStep4 + 2])
		    + (rgb[1] << 2);
		t0 = (t0 + 4) >> 3;
		CLIP16(t0, rgb[-1], bits);
		t1 = (t1 + 4) >> 3;
		CLIP16(t1, rgb[0], bits);
		/* at green pixel */
		rgb[3] = bayer[bayerStep2 + 3];
		t0 = rgb[3] * 5
		    + ((bayer[bayerStep + 3] + bayer[bayerStep3 + 3]) << 2)
		    - bayer[3]
		    - bayer[bayerStep + 2]
		    - bayer[bayerStep + 4]
		    - bayer[bayerStep3 + 2]
		    - bayer[bayerStep3 + 4]
		    - bayer[bayerStep4 + 3]
		    +
		    ((bayer[bayerStep2 + 1] + bayer[bayerStep2 + 5] +
		      1) >> 1);
		t1 = rgb[3] * 5 +
		    ((bayer[bayerStep2 + 2] + bayer[bayerStep2 + 4]) << 2)
		    - bayer[bayerStep2 + 1]
		    - bayer[bayerStep + 2]
		    - bayer[bayerStep + 4]
		    - bayer[bayerStep3 + 2]
		    - bayer[bayerStep3 + 4]
		    - bayer[bayerStep2 + 5]
		    + ((bayer[3] + bayer[bayerStep4 + 3] + 1) >> 1);
		t0 = (t0 + 4) >> 3;
		CLIP16(t0, rgb[2], bits);
		t1 = (t1 + 4) >> 3;
		CLIP16(t1, rgb[4], bits);
	    }
	} else {
	    for (; bayer <= bayerEnd - 2; bayer += 2, rgb += 6) {
		/* R at R */
		rgb[-1] = bayer[bayerStep2 + 2];
		/* B at R */
		t0 = ((bayer[bayerStep + 1] + bayer[bayerStep + 3] +
		       bayer[bayerStep * 3 + 1] + bayer[bayerStep3 +
							3]) << 1)
		    -
		    (((bayer[2] + bayer[bayerStep2] +
		       bayer[bayerStep2 + 4] + bayer[bayerStep4 +
						     2]) * 3 + 1) >> 1)
		    + rgb[-1] * 6;
		/* G at R */
		t1 = ((bayer[bayerStep + 2] + bayer[bayerStep2 + 1] +
		       bayer[bayerStep2 + 3] + bayer[bayerStep3 + 2]) << 1)
		    - (bayer[2] + bayer[bayerStep2] +
		       bayer[bayerStep2 + 4] + bayer[bayerStep4 + 2])
		    + (rgb[-1] << 2);
		t0 = (t0 + 4) >> 3;
		CLIP16(t0, rgb[1], bits);
		t1 = (t1 + 4) >> 3;
		CLIP16(t1, rgb[0], bits);

		/* at green pixel */
		rgb[3] = bayer[bayerStep2 + 3];
		t0 = rgb[3] * 5
		    + ((bayer[bayerStep + 3] + bayer[bayerStep3 + 3]) << 2)
		    - bayer[3]
		    - bayer[bayerStep + 2]
		    - bayer[bayerStep + 4]
		    - bayer[bayerStep3 + 2]
		    - bayer[bayerStep3 + 4]
		    - bayer[bayerStep4 + 3]
		    +
		    ((bayer[bayerStep2 + 1] + bayer[bayerStep2 + 5] +
		      1) >> 1);
		t1 = rgb[3] * 5 +
		    ((bayer[bayerStep2 + 2] + bayer[bayerStep2 + 4]) << 2)
		    - bayer[bayerStep2 + 1]
		    - bayer[bayerStep + 2]
		    - bayer[bayerStep + 4]
		    - bayer[bayerStep3 + 2]
		    - bayer[bayerStep3 + 4]
		    - bayer[bayerStep2 + 5]
		    + ((bayer[3] + bayer[bayerStep4 + 3] + 1) >> 1);
		t0 = (t0 + 4) >> 3;
		CLIP16(t0, rgb[4], bits);
		t1 = (t1 + 4) >> 3;
		CLIP16(t1, rgb[2], bits);
	    }
	}

	if (bayer < bayerEnd) {
	    /* B at B */
	    rgb[blue] = bayer[bayerStep2 + 2];
	    /* R at B */
	    t0 = ((bayer[bayerStep + 1] + bayer[bayerStep + 3] +
		   bayer[bayerStep3 + 1] + bayer[bayerStep3 + 3]) << 1)
		-
		(((bayer[2] + bayer[bayerStep2] +
		   bayer[bayerStep2 + 4] + bayer[bayerStep4 +
						 2]) * 3 + 1) >> 1)
		+ rgb[blue] * 6;
	    /* G at B */
	    t1 = (((bayer[bayerStep + 2] + bayer[bayerStep2 + 1] +
		    bayer[bayerStep2 + 3] + bayer[bayerStep3 + 2])) << 1)
		- (bayer[2] + bayer[bayerStep2] +
		   bayer[bayerStep2 + 4] + bayer[bayerStep4 + 2])
		+ (rgb[blue] << 2);
	    t0 = (t0 + 4) >> 3;
	    CLIP16(t0, rgb[-blue], bits);
	    t1 = (t1 + 4) >> 3;
	    CLIP16(t1, rgb[0], bits);
	    bayer++;
	    rgb += 3;
	}

	bayer -= width;
	rgb -= width * 3;

	blue = -blue;
	start_with_green = !start_with_green;
    }
}

/* coriander's Bayer decoding (GPL) */
void
dc1394_bayer_EdgeSense_uint16(const uint16_t *bayer, uint16_t *rgb, int sx, int sy, int tile, int bits)
{
    uint16_t *outR, *outG, *outB;
    register int i, j;
    int dh, dv;
    int tmp;

    // sx and sy should be even
    switch (tile) {
    case DC1394_COLOR_FILTER_GRBG:
    case DC1394_COLOR_FILTER_BGGR:
	outR = &rgb[0];
	outG = &rgb[1];
	outB = &rgb[2];
	break;
    case DC1394_COLOR_FILTER_GBRG:
    case DC1394_COLOR_FILTER_RGGB:
	outR = &rgb[2];
	outG = &rgb[1];
	outB = &rgb[0];
	break;
    default:
//	fprintf(stderr, "Bad bayer pattern ID: %d\n", tile);
//	return;
	break;
    }

    switch (tile) {
    case DC1394_COLOR_FILTER_GRBG:	//---------------------------------------------------------
    case DC1394_COLOR_FILTER_GBRG:
	// copy original RGB data to output images
      for (i = 0; i < sy*sx; i += (sx<<1)) {
	for (j = 0; j < sx; j += 2) {
	  outG[(i + j) * 3] = bayer[i + j];
	  outG[(i + sx + (j + 1)) * 3] = bayer[i + sx + (j + 1)];
	  outR[(i + j + 1) * 3] = bayer[i + j + 1];
	  outB[(i + sx + j) * 3] = bayer[i + sx + j];
	}
      }
      // process GREEN channel
      for (i = 3*sx; i < (sy - 2)*sx; i += (sx<<1)) {
	for (j = 2; j < sx - 3; j += 2) {
	  dh = abs(((outB[(i + j - 2) * 3] +
		     outB[(i + j + 2) * 3]) >> 1) -
		   outB[(i + j) * 3]);
	  dv = abs(((outB[(i - (sx<<1) + j) * 3] +
		     outB[(i + (sx<<1) + j) * 3]) >> 1)  -
		   outB[(i + j) * 3]);
	  if (dh < dv)
	    tmp = (outG[(i + j - 1) * 3] +
		   outG[(i + j + 1) * 3]) >> 1;
	  else {
	    if (dh > dv)
	      tmp = (outG[(i - sx + j) * 3] +
		     outG[(i + sx + j) * 3]) >> 1;
	    else
	      tmp = (outG[(i + j - 1) * 3] +
		     outG[(i + j + 1) * 3] +
		     outG[(i - sx + j) * 3] +
		     outG[(i + sx + j) * 3]) >> 2;
	  }
	  CLIP16(tmp, outR[(i + j) * 3], bits);
	}
      }
	
      for (i = 2*sx; i < (sy - 3)*sx; i += (sx<<1)) {
	for (j = 3; j < sx - 2; j += 2) {
	  dh = abs(((outR[(i + j - 2) * 3] +
		     outR[(i + j + 2) * 3]) >>1 ) -
		   outR[(i + j) * 3]);
	  dv = abs(((outR[(i - (sx<<1) + j) * 3] +
		     outR[(i + (sx<<1) + j) * 3]) >>1 ) -
		   outR[(i + j) * 3]);
	  if (dh < dv)
	    tmp = (outG[(i + j - 1) * 3] +
		   outG[(i + j + 1) * 3]) >> 1;
	  else {
	    if (dh > dv)
	      tmp = (outG[(i - sx + j) * 3] +
		     outG[(i + sx + j) * 3]) >> 1;
	    else
	      tmp = (outG[(i + j - 1) * 3] +
		     outG[(i + j + 1) * 3] +
		     outG[(i - sx + j) * 3] +
		     outG[(i + sx + j) * 3]) >> 2;
	  }
	  CLIP16(tmp, outR[(i + j) * 3], bits);
	}
      }
      // process RED channel
      for (i = 0; i < (sy - 1)*sx; i += (sx<<1)) {
	for (j = 2; j < sx - 1; j += 2) {
	  tmp = outG[(i + j) * 3] +
	      ((outR[(i + j - 1) * 3] -
		outG[(i + j - 1) * 3] +
		outR[(i + j + 1) * 3] -
		outG[(i + j + 1) * 3]) >> 1);
	  CLIP16(tmp, outR[(i + j) * 3], bits);
	}
      }
      for (i = sx; i < (sy - 2)*sx; i += (sx<<1)) {
	for (j = 1; j < sx; j += 2) {
	  tmp = outG[(i + j) * 3] +
	      ((outR[(i - sx + j) * 3] -
		outG[(i - sx + j) * 3] +
		outR[(i + sx + j) * 3] -
		outG[(i + sx + j) * 3]) >> 1);
	  CLIP16(tmp, outR[(i + j) * 3], bits);
	}
	for (j = 2; j < sx - 1; j += 2) {
	  tmp = outG[(i + j) * 3] +
	      ((outR[(i - sx + j - 1) * 3] -
		outG[(i - sx + j - 1) * 3] +
		outR[(i - sx + j + 1) * 3] -
		outG[(i - sx + j + 1) * 3] +
		outR[(i + sx + j - 1) * 3] -
		outG[(i + sx + j - 1) * 3] +
		outR[(i + sx + j + 1) * 3] -
		outG[(i + sx + j + 1) * 3]) >> 2);
	  CLIP16(tmp, outR[(i + j) * 3], bits);
	}
      }

      // process BLUE channel
      for (i = sx; i < sy*sx; i += (sx<<1)) {
	for (j = 1; j < sx - 2; j += 2) {
	  tmp = outG[(i + j) * 3] +
	      ((outB[(i + j - 1) * 3] -
		outG[(i + j - 1) * 3] +
		outB[(i + j + 1) * 3] -
		outG[(i + j + 1) * 3]) >> 1);
	  CLIP16(tmp, outR[(i + j) * 3], bits);
	}
      }
      for (i = 2*sx; i < (sy - 1)*sx; i += (sx<<1)) {
	for (j = 0; j < sx - 1; j += 2) {
	  tmp = outG[(i + j) * 3] +
	      ((outB[(i - sx + j) * 3] -
		outG[(i - sx + j) * 3] +
		outB[(i + sx + j) * 3] -
		outG[(i + sx + j) * 3]) >> 1);
	  CLIP16(tmp, outR[(i + j) * 3], bits);
	}
	for (j = 1; j < sx - 2; j += 2) {
	  tmp = outG[(i + j) * 3] +
	      ((outB[(i - sx + j - 1) * 3] -
		outG[(i - sx + j - 1) * 3] +
		outB[(i - sx + j + 1) * 3] -
		outG[(i - sx + j + 1) * 3] +
		outB[(i + sx + j - 1) * 3] -
		outG[(i + sx + j - 1) * 3] +
		outB[(i + sx + j + 1) * 3] -
		outG[(i + sx + j + 1) * 3]) >> 2);
	  CLIP16(tmp, outR[(i + j) * 3], bits);
	}
      }
      break;

    case DC1394_COLOR_FILTER_BGGR:	//---------------------------------------------------------
    case DC1394_COLOR_FILTER_RGGB:
	// copy original RGB data to output images
      for (i = 0; i < sy*sx; i += (sx<<1)) {
	for (j = 0; j < sx; j += 2) {
	  outB[(i + j) * 3] = bayer[i + j];
	  outR[(i + sx + (j + 1)) * 3] = bayer[i + sx + (j + 1)];
	  outG[(i + j + 1) * 3] = bayer[i + j + 1];
	  outG[(i + sx + j) * 3] = bayer[i + sx + j];
	}
      }
      // process GREEN channel
      for (i = 2*sx; i < (sy - 2)*sx; i += (sx<<1)) {
	for (j = 2; j < sx - 3; j += 2) {
	  dh = abs(((outB[(i + j - 2) * 3] +
		    outB[(i + j + 2) * 3]) >> 1) -
		   outB[(i + j) * 3]);
	  dv = abs(((outB[(i - (sx<<1) + j) * 3] +
		    outB[(i + (sx<<1) + j) * 3]) >> 1) -
		   outB[(i + j) * 3]);
	  if (dh < dv)
	    tmp = (outG[(i + j - 1) * 3] +
		   outG[(i + j + 1) * 3]) >> 1;
	  else {
	    if (dh > dv)
	      tmp = (outG[(i - sx + j) * 3] +
		     outG[(i + sx + j) * 3]) >> 1;
	    else
	      tmp = (outG[(i + j - 1) * 3] +
		     outG[(i + j + 1) * 3] +
		     outG[(i - sx + j) * 3] +
		     outG[(i + sx + j) * 3]) >> 2;
	  }
	  CLIP16(tmp, outR[(i + j) * 3], bits);
	}
      }
      for (i = 3*sx; i < (sy - 3)*sx; i += (sx<<1)) {
	for (j = 3; j < sx - 2; j += 2) {
	  dh = abs(((outR[(i + j - 2) * 3] +
		    outR[(i + j + 2) * 3]) >> 1) -
		   outR[(i + j) * 3]);
	  dv = abs(((outR[(i - (sx<<1) + j) * 3] +
		    outR[(i + (sx<<1) + j) * 3]) >> 1) -
		   outR[(i + j) * 3]);
	  if (dh < dv)
	    tmp = (outG[(i + j - 1) * 3] +
		   outG[(i + j + 1) * 3]) >>1;
	  else {
	    if (dh > dv)
	      tmp = (outG[(i - sx + j) * 3] +
		     outG[(i + sx + j) * 3]) >>1;
	    else
	      tmp = (outG[(i + j - 1) * 3] +
		     outG[(i + j + 1) * 3] +
		     outG[(i - sx + j) * 3] +
		     outG[(i + sx + j) * 3]) >>2;
	  }
	  CLIP16(tmp, outR[(i + j) * 3], bits);
	}
      }
      // process RED channel
      for (i = sx; i < (sy - 1)*sx; i += (sx<<1)) {	// G-points (1/2)
	for (j = 2; j < sx - 1; j += 2) {
	  tmp = outG[(i + j) * 3] +
	      ((outR[(i + j - 1) * 3] -
		outG[(i + j - 1) * 3] +
		outR[(i + j + 1) * 3] -
		outG[(i + j + 1) * 3]) >>1);
	  CLIP16(tmp, outR[(i + j) * 3], bits);
	}
      }
      for (i = 2*sx; i < (sy - 2)*sx; i += (sx<<1)) {
	for (j = 1; j < sx; j += 2) {	// G-points (2/2)
	  tmp = outG[(i + j) * 3] +
	      ((outR[(i - sx + j) * 3] -
		outG[(i - sx + j) * 3] +
		outR[(i + sx + j) * 3] -
		outG[(i + sx + j) * 3]) >> 1);
	  CLIP16(tmp, outR[(i + j) * 3], bits);
	}
	for (j = 2; j < sx - 1; j += 2) {	// B-points
	  tmp = outG[(i + j) * 3] +
	      ((outR[(i - sx + j - 1) * 3] -
		outG[(i - sx + j - 1) * 3] +
		outR[(i - sx + j + 1) * 3] -
		outG[(i - sx + j + 1) * 3] +
		outR[(i + sx + j - 1) * 3] -
		outG[(i + sx + j - 1) * 3] +
		outR[(i + sx + j + 1) * 3] -
		outG[(i + sx + j + 1) * 3]) >> 2);
	  CLIP16(tmp, outR[(i + j) * 3], bits);
	}
      }
      
      // process BLUE channel
      for (i = 0; i < sy*sx; i += (sx<<1)) {
	for (j = 1; j < sx - 2; j += 2) {
	  tmp = outG[(i + j) * 3] +
	      ((outB[(i + j - 1) * 3] -
		outG[(i + j - 1) * 3] +
		outB[(i + j + 1) * 3] -
		outG[(i + j + 1) * 3]) >> 1);
	  CLIP16(tmp, outR[(i + j) * 3], bits);
	}
      }
      for (i = sx; i < (sy - 1)*sx; i += (sx<<1)) {
	for (j = 0; j < sx - 1; j += 2) {
	  tmp = outG[(i + j) * 3] +
	      ((outB[(i - sx + j) * 3] -
		outG[(i - sx + j) * 3] +
		outB[(i + sx + j) * 3] -
		outG[(i + sx + j) * 3]) >> 1);
	  CLIP16(tmp, outR[(i + j) * 3], bits);
	}
	for (j = 1; j < sx - 2; j += 2) {
	  tmp = outG[(i + j) * 3] +
	      ((outB[(i - sx + j - 1) * 3] -
		outG[(i - sx + j - 1) * 3] +
		outB[(i - sx + j + 1) * 3] -
		outG[(i - sx + j + 1) * 3] +
		outB[(i + sx + j - 1) * 3] -
		outG[(i + sx + j - 1) * 3] +
		outB[(i + sx + j + 1) * 3] -
		outG[(i + sx + j + 1) * 3]) >> 2);
	  CLIP16(tmp, outR[(i + j) * 3], bits);
	}
      }
      break;
    default:			//---------------------------------------------------------
//      fprintf(stderr, "Bad bayer pattern ID: %d\n", tile);
//      return;
      break;
    }
   
    ClearBorders_uint16(rgb, sx, sy, 3);
}

/* coriander's Bayer decoding (GPL) */
void
dc1394_bayer_Downsample_uint16(const uint16_t *bayer, uint16_t *rgb, int sx, int sy, int tile, int bits)
{
    uint16_t *outR, *outG, *outB;
    register int i, j;
    int tmp;

    sx *= 2;
    sy *= 2;

    switch (tile) {
    case DC1394_COLOR_FILTER_GRBG:
    case DC1394_COLOR_FILTER_BGGR:
	outR = &rgb[0];
	outG = &rgb[1];
	outB = &rgb[2];
	break;
    case DC1394_COLOR_FILTER_GBRG:
    case DC1394_COLOR_FILTER_RGGB:
	outR = &rgb[2];
	outG = &rgb[1];
	outB = &rgb[0];
	break;
    default:
//	fprintf(stderr, "Bad Bayer pattern ID: %d\n", tile);
//	return;
	break;
    }

    switch (tile) {
    case DC1394_COLOR_FILTER_GRBG:	//---------------------------------------------------------
    case DC1394_COLOR_FILTER_GBRG:
	for (i = 0; i < sy*sx; i += (sx<<1)) {
	    for (j = 0; j < sx; j += 2) {
		tmp =
		    ((bayer[i + j] + bayer[i + sx + j + 1]) >> 1);
		CLIP16(tmp, outG[((i >> 2) + (j >> 1)) * 3], bits);
		tmp = bayer[i + sx + j + 1];
		CLIP16(tmp, outR[((i >> 2) + (j >> 1)) * 3], bits);
		tmp = bayer[i + sx + j];
		CLIP16(tmp, outB[((i >> 2) + (j >> 1)) * 3], bits);
	    }
	}
	break;
    case DC1394_COLOR_FILTER_BGGR:	//---------------------------------------------------------
    case DC1394_COLOR_FILTER_RGGB:
	for (i = 0; i < sy*sx; i += (sx<<1)) {
	    for (j = 0; j < sx; j += 2) {
		tmp =
		    ((bayer[i + sx + j] + bayer[i + j + 1]) >> 1);
		CLIP16(tmp, outG[((i >> 2) + (j >> 1)) * 3], bits);
		tmp = bayer[i + sx + j + 1];
		CLIP16(tmp, outR[((i >> 2) + (j >> 1)) * 3], bits);
		tmp = bayer[i + j];
		CLIP16(tmp, outB[((i >> 2) + (j >> 1)) * 3], bits);
	    }
	}
	break;
    default:			//---------------------------------------------------------
//	fprintf(stderr, "Bad Bayer pattern ID: %d\n", tile);
//	return;
	break;
    }

}

/* coriander's Bayer decoding (GPL) */
void dc1394_bayer_Simple_uint16(const uint16_t *bayer, uint16_t *rgb, int sx, int sy, int tile, int bits)
{
    uint16_t *outR, *outG, *outB;
    register int i, j;
    int tmp, base;

    // sx and sy should be even
    switch (tile) {
    case DC1394_COLOR_FILTER_GRBG:
    case DC1394_COLOR_FILTER_BGGR:
//	outR = &rgb[0];
//	outG = &rgb[1];
//	outB = &rgb[2];
	break;
    case DC1394_COLOR_FILTER_GBRG:
    case DC1394_COLOR_FILTER_RGGB:
//	outR = &rgb[2];
//	outG = &rgb[1];
//	outB = &rgb[0];
	break;
    default:
//	fprintf(stderr, "Bad bayer pattern ID: %d\n", tile);
//	return;
	break;
    }

    switch (tile) {
    case DC1394_COLOR_FILTER_GRBG:
    case DC1394_COLOR_FILTER_BGGR:
	outR = &rgb[0];
	outG = &rgb[1];
	outB = &rgb[2];
	break;
    case DC1394_COLOR_FILTER_GBRG:
    case DC1394_COLOR_FILTER_RGGB:
	outR = &rgb[2];
	outG = &rgb[1];
	outB = &rgb[0];
	break;
    default:
	outR = NULL;
	outG = NULL;
	outB = NULL;
	break;
    }

    switch (tile) {
    case DC1394_COLOR_FILTER_GRBG:	//---------------------------------------------------------
    case DC1394_COLOR_FILTER_GBRG:
	for (i = 0; i < sy - 1; i += 2) {
	    for (j = 0; j < sx - 1; j += 2) {
		base = i * sx + j;
		tmp = ((bayer[base] + bayer[base + sx + 1]) >> 1);
		CLIP16(tmp, outG[base * 3], bits);
		tmp = bayer[base + 1];
		CLIP16(tmp, outR[base * 3], bits);
		tmp = bayer[base + sx];
		CLIP16(tmp, outB[base * 3], bits);
	    }
	}
	for (i = 0; i < sy - 1; i += 2) {
	    for (j = 1; j < sx - 1; j += 2) {
		base = i * sx + j;
		tmp = ((bayer[base + 1] + bayer[base + sx]) >> 1);
		CLIP16(tmp, outG[(base) * 3], bits);
		tmp = bayer[base];
		CLIP16(tmp, outR[(base) * 3], bits);
		tmp = bayer[base + 1 + sx];
		CLIP16(tmp, outB[(base) * 3], bits);
	    }
	}
	for (i = 1; i < sy - 1; i += 2) {
	    for (j = 0; j < sx - 1; j += 2) {
		base = i * sx + j;
		tmp = ((bayer[base + sx] + bayer[base + 1]) >> 1);
		CLIP16(tmp, outG[base * 3], bits);
		tmp = bayer[base + sx + 1];
		CLIP16(tmp, outR[base * 3], bits);
		tmp = bayer[base];
		CLIP16(tmp, outB[base * 3], bits);
	    }
	}
	for (i = 1; i < sy - 1; i += 2) {
	    for (j = 1; j < sx - 1; j += 2) {
		base = i * sx + j;
		tmp = ((bayer[base] + bayer[base + 1 + sx]) >> 1);
		CLIP16(tmp, outG[(base) * 3], bits);
		tmp = bayer[base + sx];
		CLIP16(tmp, outR[(base) * 3], bits);
		tmp = bayer[base + 1];
		CLIP16(tmp, outB[(base) * 3], bits);
	    }
	}
	break;
    case DC1394_COLOR_FILTER_BGGR:	//---------------------------------------------------------
    case DC1394_COLOR_FILTER_RGGB:
	for (i = 0; i < sy - 1; i += 2) {
	    for (j = 0; j < sx - 1; j += 2) {
		base = i * sx + j;
		tmp = ((bayer[base + sx] + bayer[base + 1]) >> 1);
		CLIP16(tmp, outG[base * 3], bits);
		tmp = bayer[base + sx + 1];
		CLIP16(tmp, outR[base * 3], bits);
		tmp = bayer[base];
		CLIP16(tmp, outB[base * 3], bits);
	    }
	}
	for (i = 1; i < sy - 1; i += 2) {
	    for (j = 0; j < sx - 1; j += 2) {
		base = i * sx + j;
		tmp = ((bayer[base] + bayer[base + 1 + sx]) >> 1);
		CLIP16(tmp, outG[(base) * 3], bits);
		tmp = bayer[base + 1];
		CLIP16(tmp, outR[(base) * 3], bits);
		tmp = bayer[base + sx];
		CLIP16(tmp, outB[(base) * 3], bits);
	    }
	}
	for (i = 0; i < sy - 1; i += 2) {
	    for (j = 1; j < sx - 1; j += 2) {
		base = i * sx + j;
		tmp = ((bayer[base] + bayer[base + sx + 1]) >> 1);
		CLIP16(tmp, outG[base * 3], bits);
		tmp = bayer[base + sx];
		CLIP16(tmp, outR[base * 3], bits);
		tmp = bayer[base + 1];
		CLIP16(tmp, outB[base * 3], bits);
	    }
	}
	for (i = 1; i < sy - 1; i += 2) {
	    for (j = 1; j < sx - 1; j += 2) {
		base = i * sx + j;
		tmp = ((bayer[base + 1] + bayer[base + sx]) >> 1);
		CLIP16(tmp, outG[(base) * 3], bits);
		tmp = bayer[base];
		CLIP16(tmp, outR[(base) * 3], bits);
		tmp = bayer[base + 1 + sx];
		CLIP16(tmp, outB[(base) * 3], bits);
	    }
	}
	break;
    default:			//---------------------------------------------------------
//	fprintf(stderr, "Bad bayer pattern ID: %d\n", tile);
//	return;
	break;
    }

    /* add black border */
    for (i = sx * (sy - 1) * 3; i < sx * sy * 3; i++) {
	rgb[i] = 0;
    }
    for (i = (sx - 1) * 3; i < sx * sy * 3; i += (sx - 1) * 3) {
	rgb[i++] = 0;
	rgb[i++] = 0;
	rgb[i++] = 0;
    }
}

int dc1394_bayer_decoding_8bit(const unsigned char *bayer, unsigned char *rgb, unsigned int sx, unsigned int sy, unsigned int tile, unsigned int method)
{
  switch (method) {
  case DC1394_BAYER_METHOD_NEAREST:
    dc1394_bayer_NearestNeighbor(bayer, rgb, sx, sy, tile);
    return DC1394_SUCCESS;
  case DC1394_BAYER_METHOD_SIMPLE:
    dc1394_bayer_Simple(bayer, rgb, sx, sy, tile);
    return DC1394_SUCCESS;
  case DC1394_BAYER_METHOD_BILINEAR:
    dc1394_bayer_Bilinear(bayer, rgb, sx, sy, tile);
    return DC1394_SUCCESS;
  case DC1394_BAYER_METHOD_HQLINEAR:
    dc1394_bayer_HQLinear(bayer, rgb, sx, sy, tile);
    return DC1394_SUCCESS;
  case DC1394_BAYER_METHOD_DOWNSAMPLE:
    dc1394_bayer_Downsample(bayer, rgb, sx, sy, tile);
    return DC1394_SUCCESS;
  case DC1394_BAYER_METHOD_EDGESENSE:
    dc1394_bayer_EdgeSense(bayer, rgb, sx, sy, tile);
    return DC1394_SUCCESS;
  }

  return DC1394_INVALID_BAYER_METHOD;
}

int
dc1394_bayer_decoding_16bit(const uint16_t *bayer, uint16_t *rgb, uint_t sx, uint_t sy, uint_t tile, uint_t bits, uint_t method)
{
  switch (method) {
  case DC1394_BAYER_METHOD_NEAREST:
    dc1394_bayer_NearestNeighbor_uint16(bayer, rgb, sx, sy, tile, bits);
    return DC1394_SUCCESS;
  case DC1394_BAYER_METHOD_SIMPLE:
    dc1394_bayer_Simple_uint16(bayer, rgb, sx, sy, tile, bits);
    return DC1394_SUCCESS;
  case DC1394_BAYER_METHOD_BILINEAR:
    dc1394_bayer_Bilinear_uint16(bayer, rgb, sx, sy, tile, bits);
    return DC1394_SUCCESS;
  case DC1394_BAYER_METHOD_HQLINEAR:
    dc1394_bayer_HQLinear_uint16(bayer, rgb, sx, sy, tile, bits);
    return DC1394_SUCCESS;
  case DC1394_BAYER_METHOD_DOWNSAMPLE:
    dc1394_bayer_Downsample_uint16(bayer, rgb, sx, sy, tile, bits);
    return DC1394_SUCCESS;
  case DC1394_BAYER_METHOD_EDGESENSE:
    dc1394_bayer_EdgeSense_uint16(bayer, rgb, sx, sy, tile, bits);
    return DC1394_SUCCESS;
  }

  return DC1394_INVALID_BAYER_METHOD;
}

#endif

