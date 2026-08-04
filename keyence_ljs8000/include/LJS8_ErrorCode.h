//Copyright (c) 2019 KEYENCE CORPORATION. All rights reserved.
/** @file
@brief	LJS8_ErrorCode Header
*/

#define LJS8IF_RC_OK						0x0000	// Normal termination
#define LJS8IF_RC_ERR_OPEN					0x1000	// Failed to open the communication path
#define LJS8IF_RC_ERR_NOT_OPEN				0x1001	// The communication path was not established.
#define LJS8IF_RC_ERR_SEND					0x1002	// Failed to send the command.
#define LJS8IF_RC_ERR_RECEIVE				0x1003	// Failed to receive a response.
#define LJS8IF_RC_ERR_TIMEOUT				0x1004	// A timeout occurred while waiting for the response.
#define LJS8IF_RC_ERR_NOMEMORY				0x1005	// Failed to allocate memory.
#define LJS8IF_RC_ERR_PARAMETER				0x1006	// An invalid parameter was passed.
#define LJS8IF_RC_ERR_RECV_FMT				0x1007	// The received response data was invalid

#define LJS8IF_RC_ERR_HISPEED_NO_DEVICE		0x1009	// High-speed communication initialization could not be performed.
#define LJS8IF_RC_ERR_HISPEED_OPEN_YET		0x100A	// High-speed communication was initialized.
#define LJS8IF_RC_ERR_HISPEED_RECV_YET		0x100B	// Error already occurred during high-speed communication (for high-speed communication)
#define LJS8IF_RC_ERR_BUFFER_SHORT			0x100C	// The buffer size passed as an argument is insufficient. 

#define LJS8IF_RC_ERR_FILTER_NOMEMORY		0x4005	// Failed to allocate memory for the PC image filter.
#define LJS8IF_RC_ERR_FILTER_PARAMETER		0x4006	// The PC image filter settings read from the sensor head are incorrect.
#define LJS8IF_RC_ERR_FILTER_UNSUPPORTED	0x4007  // PC image filter is not supported.
#define LJS8IF_RC_ERR_FILTER_INIT			0x4009	// Initialization function has not been called yet.
