//Copyright (c) 2024 KEYENCE CORPORATION. All rights reserved.
/** @file
@brief	LJS Image Acquisition Library Header
*/


#ifndef _LJS_ACQ_H
#define _LJS_ACQ_H

#define TRG_READY (0x0020U)		// Trigger ready

typedef struct {
	int		interpolateLines;			// Number of Interpolate lines.
	int		timeout_ms;					// Timeout error occurs if the acquiring process exceeds the set value.
	int		use_external_trigger;		// Set "1" if you controll the trigger timing externally. (e.g. terminal input)
} LJS_ACQ_SETPARAM;

typedef struct {
	int		luminance_enabled;			// Luminance data presence, with luminance: 1, without luminance: 0.
	int		x_pointnum;					// Number of X direction points.
	int		y_pointnum;					// Number of Y direction points.
	float	x_pitch_um;					// Data pitch of X data.
	float	y_pitch_um;					// Data pitch of Y data.
	float	z_pitch_um;					// Data pitch of Z data.
	int		isProcTimeoutOccurred;		// Indicates that a timeout occurred in stray light suppression processing.
} LJS_ACQ_GETPARAM;

extern "C"
{
int LJS_ACQ_OpenDevice(int nDeviceId, LJS8IF_ETHERNET_CONFIG *EthernetConfig, int HighSpeedPortNo);

void LJS_ACQ_CloseDevice(int nDeviceId);

//Blocking I/F
int LJS_ACQ_Acquire(int nDeviceId, unsigned short **heightImage, unsigned char **luminanceImage, LJS_ACQ_SETPARAM *setParam, LJS_ACQ_GETPARAM *getParam);

//Non-blocking I/F
int LJS_ACQ_StartAsync(int nDeviceId, LJS_ACQ_SETPARAM *setParam);
int LJS_ACQ_AcquireAsync(int nDeviceId, unsigned short **heightImage, unsigned char **luminanceImage, LJS_ACQ_SETPARAM *setParam, LJS_ACQ_GETPARAM *getParam);

}
#endif /* _LJS_ACQ_H */