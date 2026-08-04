//Copyright (c) 2024 KEYENCE CORPORATION. All rights reserved.
/** @file
@brief	LJS8_IF_linux Header
*/

#ifndef _LJS8_IF_LINUX_H
#define _LJS8_IF_LINUX_H

#define MAX_LJS_DEVICENUM	(6U)
#define MAX_LJS_XDATANUM	(3200U)
#define MAX_LJS_YDATANUM	(6400U)

/// Setting value storage level designation
typedef enum {
	LJS8IF_SETTING_DEPTH_WRITE		= 0x00,		// Write settings area
	LJS8IF_SETTING_DEPTH_RUNNING	= 0x01,		// Running settings area
	LJS8IF_SETTING_DEPTH_SAVE		= 0x02,		// Save area
} LJS8IF_SETTING_DEPTH;

/// Initialization target setting item designation
typedef enum {
	LJS8IF_INIT_SETTING_TARGET_PRG0		= 0x00,		// Program 0
	LJS8IF_INIT_SETTING_TARGET_PRG1		= 0x01,		// Program 1
	LJS8IF_INIT_SETTING_TARGET_PRG2		= 0x02,		// Program 2
	LJS8IF_INIT_SETTING_TARGET_PRG3		= 0x03,		// Program 3
	LJS8IF_INIT_SETTING_TARGET_PRG4		= 0x04,		// Program 4
	LJS8IF_INIT_SETTING_TARGET_PRG5		= 0x05,		// Program 5
	LJS8IF_INIT_SETTING_TARGET_PRG6		= 0x06,		// Program 6
	LJS8IF_INIT_SETTING_TARGET_PRG7		= 0x07,		// Program 7
	LJS8IF_INIT_SETTING_TARGET_PRG8		= 0x08,		// Program 8
	LJS8IF_INIT_SETTING_TARGET_PRG9		= 0x09,		// Program 9
	LJS8IF_INIT_SETTING_TARGET_PRG10	= 0x0A,		// Program 10
	LJS8IF_INIT_SETTING_TARGET_PRG11	= 0x0B,		// Program 11
	LJS8IF_INIT_SETTING_TARGET_PRG12	= 0x0C,		// Program 12
	LJS8IF_INIT_SETTING_TARGET_PRG13	= 0x0D,		// Program 13
	LJS8IF_INIT_SETTING_TARGET_PRG14	= 0x0E,		// Program 14
	LJS8IF_INIT_SETTING_TARGET_PRG15	= 0x0F,		// Program 15
} LJS8IF_INIT_SETTING_TARGET;

/// Get height image data position specification method designation
typedef enum {
	LJS8IF_HEIGHT_IMAGE_POSITION_CURRENT		= 0x00,		// From current
	LJS8IF_HEIGHT_IMAGE_POSITION_SPEC			= 0x02,		// Specify position
	LJS8IF_HEIGHT_IMAGE_POSITION_COMMITTED		= 0x03,		// From current after height image commitment
} LJS8IF_HEIGHT_IMAGE_POSITION;

/// Version info structure
typedef struct {
	int	nMajorNumber;		// Major number
	int	nMinorNumber;		// Minor number
	int	nRevisionNumber;	// Revision number
	int	nBuildNumber;		// Build number
} LJS8IF_VERSION_INFO;

/// Ethernet settings structure
typedef struct {
	unsigned char	abyIpAddress[4];	// The IP address of the head to connect to.
	unsigned short	wPortNo;			// The port number of the head to connect to.
	unsigned char	reserve[2];			// Reserved
} LJS8IF_ETHERNET_CONFIG;

/// Setting item designation structure
typedef struct {
	unsigned char	byType;			// Setting type
	unsigned char	byCategory;		// Category
	unsigned char	byItem;			// Setting item
	unsigned char	reserve;		// Reserved
	unsigned char	byTarget1;		// Setting Target 1
	unsigned char	byTarget2;		// Setting Target 2
	unsigned char	byTarget3;		// Setting Target 3
	unsigned char	byTarget4;		// Setting Target 4
} LJS8IF_TARGET_SETTING;


/// HeightImage information structure
typedef struct {
	unsigned short	wXPointNum;			// Number of data in X direction.
	unsigned short	wYLineNum;			// Number of lines in Y direction.
	unsigned char	byLuminanceOutput;	// Whether luminance output is on.
	unsigned char	reserve[3];			// Reserved
	int				nXStart;			// X coordinate of 1st data.
	unsigned int	dwPitchX;			// Data pitch in X direction.
	int				nYStart;			// Y coordinate of 1st data.
	unsigned int	dwPitchY;			// Data pitch in Y direction.
	unsigned char	reserve2[4];		// Reserved
	unsigned int	dwPitchZ;			// Data pitch in Z direction.
} LJS8IF_HEIGHT_IMAGE_INFO;

/// Profile header information structure
typedef struct {
	unsigned int	reserve;		// Reserved
	unsigned int	dwHeightImageNo;// The height image number
	unsigned int	dwProfileNo;	// The profile number
	unsigned char	byProcTimeout;	// Indicates that a timeout occurred in stray light suppression processing.
	unsigned char	reserve2[3];	// Reserved
	unsigned int	reserve3[2];	// Reserved
} LJS8IF_PROFILE_HEADER;

/// Profile footer information structure
typedef struct {
	unsigned int	reserve;	// Reserved
} LJS8IF_PROFILE_FOOTER;

/// Get profile request structure
typedef struct {
	unsigned char	reserve;			// Reserved
	unsigned char	byPositionMode;		// The get profile position specification method
	unsigned char	reserve2[2];		// Reserved
	unsigned int	dwGetHeightImageNo;	// The height image number for the profile to get
	unsigned int	dwGetProfileNo;		// The profile number to start getting profiles from in the specified height image number.
	unsigned short	wGetProfileCount;	// The number of profiles to read.
	unsigned char	byErase;			// Specifies whether to erase the profile data that was read and the profile data older than that.
	unsigned char	reserve3;			// Reserved
} LJS8IF_GET_HEIGHT_IMAGE_PROFILE_REQUEST;

/// Get profile response structure
typedef struct {
	unsigned int	dwCurrentHeightImageNo;				// The height image number at the current point in time.
	unsigned int	dwCurrentHeightImageProfileCount;	// The number of profiles in the newest height image.
	unsigned int	dwOldestHeightImageNo;				// The height image number for the oldest height image held by the head.
	unsigned int	dwOldestHeightImageProfileCount;	// The number of profiles in the oldest height image held by the head.
	unsigned int	dwGetHeightImageNo;					// The height image number that was read this time.
	unsigned int	dwGetHeightImageProfileCount;		// The number of profiles in the height image that was read this time.
	unsigned int	dwGetHeightImageTopProfileNo;		// The oldest profile number in the height image out of the profiles that were read this time.
	unsigned short	wGetProfileCount;					// The number of profiles that were read this time.
	unsigned char	byCurrentHeightImageCommitted;		// The height image measurements for the newest height image number has finished.
	unsigned char	reserve;							// Reserved
} LJS8IF_GET_HEIGHT_IMAGE_PROFILE_RESPONSE;

/// High-speed communication prep start request structure
typedef struct {
	unsigned char	bySendPosition;			// Send start position
	unsigned char	reserve[3];				// Reserved
} LJS8IF_HIGH_SPEED_PRE_START_REQ;


/**
Callback function interface for high-speed data communication
@param	pProfileHeaderArray		A pointer to the buffer that stores the header data array.
@param	pHeightProfileArray		A pointer to the buffer that stores the profile data array.
@param	pLuminanceProfileArray		A pointer to the buffer that stores the luminance profile data array.
@param	dwLuminanceEnable		The value indicating whether luminance data output is enable or not.
@param	dwProfileDataCount		The data count of one profile.
@param	dwCount		The number of profile or header data stored in buffer.
@param	dwNotify	Notification of an interruption in high-speed communication or a break in height image measurements.
@param	dwUser		User information
*/
typedef void(*LJS8IF_CALLBACK_SIMPLE_ARRAY)(LJS8IF_PROFILE_HEADER* pProfileHeaderArray, unsigned short* pHeightProfileArray, unsigned char* pLuminanceProfileArray, unsigned int dwLuminanceEnable, unsigned int dwProfileDataCount, unsigned int dwCount, unsigned int dwNotify, unsigned int dwUser);



extern "C"
{
	/**
	Ethernet communication connection
	@param	nDeviceId		The communication device to communicate with.
	@param	pEthernetConfig	Ethernet communication settings
	@return	Return code
	*/
	int LJS8IF_EthernetOpen(int nDeviceId, LJS8IF_ETHERNET_CONFIG* pEthernetConfig);
	
	/**
	Disconnect communication path
	@param	nDeviceId	The communication device to communicate with.
	@return	Return code
	*/
	int LJS8IF_CommunicationClose(int nDeviceId);

	// System control
	/**
	Reboot the head
	@param	nDeviceId	The communication device to communicate with.
	@return	Return code
	*/
	int LJS8IF_Reboot(int nDeviceId);

	/**
	Return to factory state
	@param	nDeviceId	The communication device to communicate with.
	@return	Return code
	*/
	int LJS8IF_ReturnToFactorySetting(int nDeviceId);

	/**
	Control Laser
	@param	nDeviceId	The communication device to communicate with.
	@param	byState		Laser state
	@return	Return code
	*/
	int LJS8IF_ControlLaser(int nDeviceId, unsigned char byState);

	/**
	Get system error information
	@param	nDeviceId		The communication device to communicate with.
	@param	byReceivedMax	The maximum amount of system error information to receive
	@param	pbyErrCount		The buffer to receive the amount of system error information.
	@param	pwErrCode		The buffer to receive the system error information.
	@return	Return code
	*/
	int LJS8IF_GetError(int nDeviceId, unsigned char byReceivedMax, unsigned char* pbyErrCount, unsigned short* pwErrCode);

	/**
	Clear system error
	@param	nDeviceId	The communication device to communicate with.
	@param	wErrCode	The error code for the error you wish to clear.
	@return	Return code
	*/
	int LJS8IF_ClearError(int nDeviceId, unsigned short wErrCode);

	/**
	Get head temperature
	@param	nDeviceId				The communication device to communicate with.
	@param	pnSensorTemperature		The buffer to receive sensor temperature.
	@param	pnProcessor1Temperature	The buffer to receive processor1 temperature.
	@param	pnProcessor2Temperature	The buffer to receive processor2 temperature.
	@param	pnCaseTemperature		The buffer to receive case temperature.
	@param	pnDriveUnitTemperature	The buffer to receive drive unit temperature.
	@return	Return code
	*/
	int LJS8IF_GetHeadTemperature(int nDeviceId, short* pnSensorTemperature, short* pnProcessor1Temperature, short* pnProcessor2Temperature, short* pnCaseTemperature, short* pnDriveUnitTemperature);

	/**
	Get head model
	@param	nDeviceId			The communication device to communicate with.
	@param	pHeadModel			The buffer to receive head model.
	@return	Return code
	*/
	int LJS8IF_GetHeadModel(int nDeviceId, char* pHeadModel);

	/**
	Get serial Number
	@param	nDeviceId			The communication device to communicate with.
	@param	pSerialNo			The buffer to receive serial number
	@return	Return code
	*/
	int LJS8IF_GetSerialNumber(int nDeviceId, char* pSerialNo);

	/**
	Get current attention status value
	@param	nDeviceId			The communication device to communicate with.
	@param	pwAttentionStatus	The buffer to receive attention status
	@return	Return code
	*/
	int LJS8IF_GetAttentionStatus(int nDeviceId, unsigned short* pwAttentionStatus);

	// Measurement control
	/**
	Trigger
	@param	nDeviceId	The communication device to communicate with.
	@return	Return code
	*/
	int LJS8IF_Trigger(int nDeviceId);

	/**
	Clear memory
	@param	nDeviceId	The communication device to communicate with.
	@return	Return code
	*/
	int LJS8IF_ClearMemory(int nDeviceId);

	// Functions related to modifying or reading settings
	/**
	Send setting
	@param	nDeviceId		The communication device to communicate with.
	@param	byDepth			The level to reflect the setting value
	@param	TargetSetting	The item that is the target
	@param	pData			The buffer that stores the setting data
	@param	dwDataSize		The size in BYTEs of the setting data
	@param	pdwError		Detailed setting error
	@return	Return code
	*/
	int LJS8IF_SetSetting(int nDeviceId, unsigned char byDepth, LJS8IF_TARGET_SETTING TargetSetting, void* pData, unsigned int dwDataSize, unsigned int* pdwError);

	/**
	Get setting
	@param	nDeviceId		The communication device to communicate with.
	@param	byDepth			The level of the setting value to get.
	@param	TargetSetting	The item that is the target
	@param	pData			The buffer to receive the setting data
	@param	dwDataSize		The size of the buffer to receive the acquired data in BYTEs.
	@return	Return code
	*/
	int LJS8IF_GetSetting(int nDeviceId, unsigned char byDepth, LJS8IF_TARGET_SETTING TargetSetting, void* pData, unsigned int dwDataSize);

	/**
	Initialize setting
	@param	nDeviceId	The communication device to communicate with.
	@param	byDepth		The level to reflect the initialized setting.
	@param	byTarget	The setting that is the target for initialization.
	@return	Return code
	*/
	int LJS8IF_InitializeSetting(int nDeviceId, unsigned char byDepth, unsigned char byTarget);

	/**
	Request to reflect settings in the write settings area
	@param	nDeviceId	The communication device to communicate with.
	@param	byDepth		The level to reflect the setting value
	@param	pdwError	Detailed setting error
	@return	Return code
	*/
	int LJS8IF_ReflectSetting(int nDeviceId, unsigned char byDepth, unsigned int* pdwError);

	/**
	Update write settings area
	@param	nDeviceId	The communication device to communicate with.
	@param	byDepth		The level of the settings to update the write settings area with.
	@return	Return code
	*/
	int LJS8IF_RewriteTemporarySetting(int nDeviceId, unsigned char byDepth);

	/**
	Check the status of saving to the save area
	@param	nDeviceId	The communication device to communicate with.
	@param	pbyBusy		Other than 0: Accessing the save area, 0: no access.
	@return	Return code
	*/
	int LJS8IF_CheckMemoryAccess(int nDeviceId, unsigned char* pbyBusy);

	/**
	Change program
	@param	nDeviceId	The communication device to communicate with.
	@param	byProgramNo	Program number after the change.
	@return	Return code
	*/
	int LJS8IF_ChangeActiveProgram(int nDeviceId, unsigned char byProgramNo);

	/**
	Get the active program number
	@param	nDeviceId		The communication device to communicate with.
	@param	pbyProgramNo	The buffer to receive the active program number.
	@return	Return code
	*/
	int LJS8IF_GetActiveProgram(int nDeviceId, unsigned char* pbyProgramNo);

	/**
	Get height image profiles by simple array format
	@param	nDeviceId				The communication device to communicate with.
	@param	pReq					The position, etc., of the profiles to get.
	@param  byUsePCImageFilter      Specifies whether to use PC image filters.
	@param	pRsp					The position, etc., of the profiles that were actually acquired.
	@param	pHeightImageInfo		The information for the acquired height images.
	@param  pProfileHeaderArray		The buffer to get array of header.
	@param  pHeightProfileArray		The buffer to get array of profile data.
	@param  pLuminanceProfileArray	The buffer to get array of luminance profile data.
	@return	Return code
	*/
	int LJS8IF_GetHeightImageSimpleArray(int nDeviceId, LJS8IF_GET_HEIGHT_IMAGE_PROFILE_REQUEST* pReq, unsigned char byUsePCImageFilter, LJS8IF_GET_HEIGHT_IMAGE_PROFILE_RESPONSE* pRsp, LJS8IF_HEIGHT_IMAGE_INFO* pHeightImageInfo, LJS8IF_PROFILE_HEADER* pProfileHeaderArray, unsigned short* pHeightProfileArray, unsigned char* pLuminanceProfileArray);

	/**
	Initialize Ethernet high-speed data communication for simple array
	@param	nDeviceId				The communication device to communicate with.
	@param	pEthernetConfig			The Ethernet settings used in high-speed communication.
	@param	wHighSpeedPortNo		The port number used in high-speed communication.
	@param	pCallBackSimpleArray	The callback function to call when data is received by high-speed communication.
	@param	dwThreadId				Thread ID.
	@return	Return code
	*/
	int LJS8IF_InitializeHighSpeedDataCommunicationSimpleArray(int nDeviceId, LJS8IF_ETHERNET_CONFIG* pEthernetConfig, unsigned short wHighSpeedPortNo,
		LJS8IF_CALLBACK_SIMPLE_ARRAY pCallBackSimpleArray, unsigned int dwThreadId);

	/**
	Request preparation before starting high-speed data communication
	@param	nDeviceId			The communication device to communicate with.
	@param	pReq				What data to send high-speed communication from.
	@param  byUsePCImageFilter  Specifies whether to use PC image filters.
	@param	pHeightImageInfo	Stores the height-image information.
	@return	Return code
	*/
	int LJS8IF_PreStartHighSpeedDataCommunication(int nDeviceId, LJS8IF_HIGH_SPEED_PRE_START_REQ* pReq, unsigned char byUsePCImageFilter, LJS8IF_HEIGHT_IMAGE_INFO* pHeightImageInfo);

	/**
	Start high-speed data communication
	@param	nDeviceId	The communication device to communicate with.
	@return	Return code
	*/
	int LJS8IF_StartHighSpeedDataCommunication(int nDeviceId);

	/**
	Stop high-speed data communication
	@param	nDeviceId	The communication device to communicate with.
	@return	Return code
	*/
	int LJS8IF_StopHighSpeedDataCommunication(int nDeviceId);

	/**
	Finalize high-speed data communication
	@param	nDeviceId	The communication device to communicate with.
	@return	Return code
	*/
	int LJS8IF_FinalizeHighSpeedDataCommunication(int nDeviceId);
	
};
#endif /* _LJS8_IF_LINUX_H */