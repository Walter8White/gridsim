//Copyright (c) 2024 KEYENCE CORPORATION. All rights reserved.
/** @file
@brief	LJS Image Acquisition Library Implementation
*/

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <chrono>

#include "LJS8_IF_Linux.h"
#include "LJS8_ErrorCode.h"

#include "LJS_ACQ.h"

// Static variable
static LJS8IF_ETHERNET_CONFIG _ethernetConfig[ MAX_LJS_DEVICENUM ];
static int _highSpeedPortNo[ MAX_LJS_DEVICENUM ];
static int _imageAvailable[ MAX_LJS_DEVICENUM ];
static int _yImageSizeAcquired[ MAX_LJS_DEVICENUM ];
static LJS_ACQ_GETPARAM _getParam[ MAX_LJS_DEVICENUM ];
static unsigned short* _heightImage[ MAX_LJS_DEVICENUM ];
static unsigned char* _luminanceImage[ MAX_LJS_DEVICENUM ];
static int _headTimeoutOccurred[ MAX_LJS_DEVICENUM ];
static int _waitTrigger_ms = 0;

// Function prototype
static void myCallbackFuncSimpleArray(LJS8IF_PROFILE_HEADER* pProfileHeaderArray, unsigned short* pHeightProfileArray, unsigned char* pLuminanceProfileArray, unsigned int dwLuminanceEnable, unsigned int dwProfileDataCount, unsigned int dwCount, unsigned int dwNotify, unsigned int dwUser);

int LJS_ACQ_OpenDevice(int nDeviceId, LJS8IF_ETHERNET_CONFIG *EthernetConfig, int HighSpeedPortNo){
	int errCode = LJS8IF_EthernetOpen(nDeviceId,EthernetConfig);
	
	_ethernetConfig[ nDeviceId ] = *EthernetConfig;
	_highSpeedPortNo[ nDeviceId ] = HighSpeedPortNo;
	printf("[@(LJS_ACQ_OpenDevice) Open device](0x%x)\n",errCode);
	
	return errCode;
}

void LJS_ACQ_CloseDevice(int nDeviceId){
	LJS8IF_FinalizeHighSpeedDataCommunication(nDeviceId);
	LJS8IF_CommunicationClose(nDeviceId);
	printf("[@(LJS_ACQ_CloseDevice) Close device]\n");
}

int LJS_ACQ_Acquire(int nDeviceId, unsigned short **heightImage, unsigned char **luminanceImage, LJS_ACQ_SETPARAM *setParam, LJS_ACQ_GETPARAM *getParam){
	int errCode;
	
	int timeout_ms = setParam->timeout_ms;
	int use_external_trigger = setParam->use_external_trigger;
	
	//Initialize
	errCode = LJS8IF_InitializeHighSpeedDataCommunicationSimpleArray(nDeviceId, &_ethernetConfig[nDeviceId], _highSpeedPortNo[nDeviceId], &myCallbackFuncSimpleArray, nDeviceId);
	printf("[@(LJS_ACQ_Acquire) Initialize HighSpeed](0x%x)\n",errCode);
	if (errCode != LJS8IF_RC_OK) return errCode;

	//PreStart
	LJS8IF_HIGH_SPEED_PRE_START_REQ startReq;
	startReq.bySendPosition = 2;
	LJS8IF_HEIGHT_IMAGE_INFO profileInfo;
	errCode = LJS8IF_PreStartHighSpeedDataCommunication(nDeviceId, &startReq, 0, &profileInfo);
	printf("[@(LJS_ACQ_Acquire) PreStart](0x%x)\n",errCode);
	if (errCode != LJS8IF_RC_OK) return errCode;
	
	//Allocate memory
	if ((_heightImage[nDeviceId] != NULL) || (_luminanceImage[nDeviceId] != NULL)) {
		if(_heightImage[nDeviceId] != NULL) {
			free(_heightImage[nDeviceId]);
			_heightImage[nDeviceId] = NULL;
		}
		if (_luminanceImage[nDeviceId] != NULL) {
			free(_luminanceImage[nDeviceId]);
			_luminanceImage[nDeviceId] = NULL;
		}
	}
	int xDataNum = profileInfo.wXPointNum;
	int yDataNum = profileInfo.wYLineNum;
	int copyLength = xDataNum * yDataNum;
	_heightImage[nDeviceId] = (unsigned short*)malloc(copyLength * sizeof(short));
	_luminanceImage[nDeviceId] = (unsigned char*)malloc(copyLength * sizeof(char));
	if ((_heightImage[nDeviceId] == NULL) || (_luminanceImage[nDeviceId] == NULL)) {
		if (_heightImage[nDeviceId] != NULL) {
			free(_heightImage[nDeviceId]);
			_heightImage[nDeviceId] = NULL;
		}
		if (_luminanceImage[nDeviceId] != NULL) {
			free(_luminanceImage[nDeviceId]);
			_luminanceImage[nDeviceId] = NULL;
		}
		return LJS8IF_RC_ERR_NOMEMORY;
	}

	//Start HighSpeed
	_imageAvailable[nDeviceId] = 0;
	_yImageSizeAcquired[ nDeviceId ] = 0;
	_headTimeoutOccurred[ nDeviceId ] = 0;
	
	errCode = LJS8IF_StartHighSpeedDataCommunication(nDeviceId);
	printf("[@(LJS_ACQ_Acquire) Start HighSpeed](0x%x)\n",errCode);
	if (errCode != LJS8IF_RC_OK) {
		if (_heightImage[nDeviceId] != NULL) {
			free(_heightImage[nDeviceId]);
			_heightImage[nDeviceId] = NULL;
		}
		if (_luminanceImage[nDeviceId] != NULL) {
			free(_luminanceImage[nDeviceId]);
			_luminanceImage[nDeviceId] = NULL;
		}
		return errCode;
	}
	
	//StartMeasure
	_waitTrigger_ms = 0;
	if ( use_external_trigger == 0 ){
		// Check the status of the sensor head.
		ushort status = 0;
		bool isTriggerReady = false;
		auto startTime = std::chrono::high_resolution_clock::now();
		while (true) {
			LJS8IF_GetAttentionStatus(nDeviceId, &status);
			isTriggerReady = (status & TRG_READY) > 0;
			if (isTriggerReady) {
				//Trigger is ready.
				break;
			}
			auto nowTime = std::chrono::high_resolution_clock::now();
			_waitTrigger_ms = std::chrono::duration_cast<std::chrono::milliseconds>(nowTime - startTime).count();
			if (_waitTrigger_ms > timeout_ms) break;
		}
		errCode = LJS8IF_Trigger(nDeviceId);
		printf("[@(LJS_ACQ_Acquire) Trigger](0x%x)\n", errCode);
		if (errCode != LJS8IF_RC_OK) {
			if (_heightImage[nDeviceId] != NULL) {
				free(_heightImage[nDeviceId]);
				_heightImage[nDeviceId] = NULL;
			}
			if (_luminanceImage[nDeviceId] != NULL) {
				free(_luminanceImage[nDeviceId]);
				_luminanceImage[nDeviceId] = NULL;
			}
			if(_waitTrigger_ms > timeout_ms)
				printf("Timeout occurred while waiting for trigger to be ready.\n");
			return errCode;
		}
	}

	printf(" [@(LJS_ACQ_Acquire) acquring image...]\n");
	for(int i = 0; i < timeout_ms - _waitTrigger_ms; ++i){
		usleep(1000);
		if (_imageAvailable[nDeviceId] > 0) break;
	}
	
	if(_imageAvailable[nDeviceId] == 0){
		printf(" [@(LJS_ACQ_Acquire) timeout]\n");
		
		//Stop HighSpeed
		errCode = LJS8IF_StopHighSpeedDataCommunication(nDeviceId);
		printf("[@(LJS_ACQ_Acquire) Stop HighSpeed](0x%x)\n",errCode);
		
		//Free memory
		if((_heightImage[nDeviceId] == NULL) || (_luminanceImage[nDeviceId] == NULL)){
			if (_heightImage[nDeviceId] != NULL) {
				free(_heightImage[nDeviceId]);
				_heightImage[nDeviceId] = NULL;
			}
			if (_luminanceImage[nDeviceId] != NULL) {
				free(_luminanceImage[nDeviceId]);
				_luminanceImage[nDeviceId] = NULL;
			}
		}
		return LJS8IF_RC_ERR_TIMEOUT;
	}
	printf(" [@(LJS_ACQ_Acquire) done]\n");

	if (_headTimeoutOccurred[nDeviceId] != 0) {
		printf(" [Timeout occurred in head processing]\n");
	}

	//Stop HighSpeed
	errCode = LJS8IF_StopHighSpeedDataCommunication(nDeviceId);
	printf("[@(LJS_ACQ_Acquire) Stop HighSpeed](0x%x)\n",errCode);

//---------------------------------------------------------------------
//  Organize parameters related to the acquired image 
//---------------------------------------------------------------------
	
	_getParam[nDeviceId].luminance_enabled 	= profileInfo.byLuminanceOutput;
	_getParam[nDeviceId].x_pointnum			= profileInfo.wXPointNum;
	_getParam[nDeviceId].y_pointnum			= profileInfo.wYLineNum * setParam->interpolateLines;
	_getParam[nDeviceId].x_pitch_um			= profileInfo.dwPitchX * 0.01f;
	_getParam[nDeviceId].y_pitch_um			= profileInfo.dwPitchY * 0.01f / setParam->interpolateLines;
	_getParam[nDeviceId].z_pitch_um			= profileInfo.dwPitchZ * 0.01f;
	
	*getParam = _getParam[nDeviceId];

//---------------------------------------------------------------------
//  Copy internal buffer to user buffer
//---------------------------------------------------------------------
	unsigned short* wDataBuf = &_heightImage[nDeviceId][0];
	int interpolateLines = setParam->interpolateLines;

	//Allocate user memory
	if ((*heightImage != NULL) || (*luminanceImage != NULL)) {
		if (*heightImage != NULL) {
			free(*heightImage);
			*heightImage = NULL;
		}
		if (*luminanceImage != NULL) {
			free(*luminanceImage);
			*luminanceImage = NULL;
		}
	}
	*heightImage = (unsigned short*)malloc(sizeof(short) * xDataNum * _getParam[nDeviceId].y_pointnum);
	*luminanceImage = (unsigned char*)malloc(sizeof(char) * xDataNum * _getParam[nDeviceId].y_pointnum);
	if ((*heightImage == NULL) || (*luminanceImage == NULL)) {
		// failed to allocate user memory
		if (*heightImage != NULL) {
			free(*heightImage);
			*heightImage = NULL;
		}
		if (*luminanceImage != NULL) {
			free(*luminanceImage);
			*luminanceImage = NULL;
		}
		if (_heightImage[nDeviceId] != NULL) {
			free(_heightImage[nDeviceId]);
			_heightImage[nDeviceId] = NULL;
		}
		if (_luminanceImage[nDeviceId] != NULL) {
			free(_luminanceImage[nDeviceId]);
			_luminanceImage[nDeviceId] = NULL;
		}
		return LJS8IF_RC_ERR_NOMEMORY;
	}

	for (int y = 0; y < _yImageSizeAcquired[nDeviceId]; ++y) {
		for (int dy = 0; dy < interpolateLines; ++dy) {
			memcpy(&(*heightImage)[(y * interpolateLines + dy) * xDataNum], &wDataBuf[y * xDataNum], xDataNum * sizeof(unsigned short));
		}
	}

	if (profileInfo.byLuminanceOutput > 0) {
		unsigned char* byDataBuf = (unsigned char*)&_luminanceImage[nDeviceId][0];
		for (int y = 0; y < _yImageSizeAcquired[nDeviceId]; ++y) {
			for (int dy = 0; dy < interpolateLines; ++dy) {
				memcpy(&(*luminanceImage)[(y * interpolateLines + dy) * xDataNum], &byDataBuf[y * xDataNum], xDataNum * sizeof(unsigned char));
			}
		}
	}
	
	
	//Free memory
	if (_heightImage[nDeviceId] != NULL) {
		free(_heightImage[nDeviceId]);
		_heightImage[nDeviceId] = NULL;
	}
	if (_luminanceImage[nDeviceId] != NULL) {
		free(_luminanceImage[nDeviceId]);
		_luminanceImage[nDeviceId] = NULL;
	}
	return LJS8IF_RC_OK;
}

int LJS_ACQ_StartAsync(int nDeviceId, LJS_ACQ_SETPARAM *setParam){
	int errCode;
	
	int timeout_ms = setParam->timeout_ms;
	int use_external_trigger = setParam->use_external_trigger;
	
	//Initialize
	errCode = LJS8IF_InitializeHighSpeedDataCommunicationSimpleArray(nDeviceId, &_ethernetConfig[nDeviceId], _highSpeedPortNo[nDeviceId], &myCallbackFuncSimpleArray, nDeviceId);
	printf("[@(LJS_ACQ_StartAsync) Initialize HighSpeed](0x%x)\n",errCode);
	if (errCode != LJS8IF_RC_OK) return errCode;
	
	//PreStart
	LJS8IF_HIGH_SPEED_PRE_START_REQ startReq;
	startReq.bySendPosition = 2;
	LJS8IF_HEIGHT_IMAGE_INFO profileInfo;
	
	errCode = LJS8IF_PreStartHighSpeedDataCommunication(nDeviceId, &startReq, 0, &profileInfo);
	printf("[@(LJS_ACQ_StartAsync) PreStart](0x%x)\n",errCode);
	if (errCode != LJS8IF_RC_OK) return errCode;
	
	//Allocate memory
	if ((_heightImage[nDeviceId] != NULL) || (_luminanceImage[nDeviceId] != NULL)) {
		if(_heightImage[nDeviceId] != NULL) {
			free(_heightImage[nDeviceId]);
			_heightImage[nDeviceId] = NULL;
		}
		if (_luminanceImage[nDeviceId] != NULL) {
			free(_luminanceImage[nDeviceId]);
			_luminanceImage[nDeviceId] = NULL;
		}
	}
	int xDataNum = profileInfo.wXPointNum;
	int yDataNum = profileInfo.wYLineNum;
	int copyLength = xDataNum * yDataNum;
	_heightImage[nDeviceId] = (unsigned short*)malloc(copyLength * sizeof(short));
	_luminanceImage[nDeviceId] = (unsigned char*)malloc(copyLength * sizeof(char));
	if ((_heightImage[nDeviceId] == NULL) || (_luminanceImage[nDeviceId] == NULL)) {
		if (_heightImage[nDeviceId] != NULL) {
			free(_heightImage[nDeviceId]);
			_heightImage[nDeviceId] = NULL;
		}
		if (_luminanceImage[nDeviceId] != NULL) {
			free(_luminanceImage[nDeviceId]);
			_luminanceImage[nDeviceId] = NULL;
		}
		return LJS8IF_RC_ERR_NOMEMORY;
	}

	//Start HighSpeed
	_imageAvailable[nDeviceId] = 0;
	_yImageSizeAcquired[ nDeviceId ] = 0;
	_headTimeoutOccurred[nDeviceId] = 0;
	
	errCode = LJS8IF_StartHighSpeedDataCommunication(nDeviceId);
	printf("[@(LJS_ACQ_StartAsync) Start HighSpeed](0x%x)\n",errCode);
	if (errCode != LJS8IF_RC_OK) {
		if (_heightImage[nDeviceId] != NULL) {
			free(_heightImage[nDeviceId]);
			_heightImage[nDeviceId] = NULL;
		}
		if (_luminanceImage[nDeviceId] != NULL) {
			free(_luminanceImage[nDeviceId]);
			_luminanceImage[nDeviceId] = NULL;
		}
		return errCode;
	}
	
	//StartMeasure
	_waitTrigger_ms = 0;
	if (use_external_trigger == 0) {
		// Check the status of the sensor head.
		ushort status = 0;
		bool isTriggerReady = false;
		auto startTime = std::chrono::high_resolution_clock::now();
		while(true) {
			LJS8IF_GetAttentionStatus(nDeviceId, &status);
			isTriggerReady = (status & TRG_READY) > 0;
			if (isTriggerReady) {
				//Trigger is ready.
				break;
			}
			auto nowTime = std::chrono::high_resolution_clock::now();
			_waitTrigger_ms = std::chrono::duration_cast<std::chrono::milliseconds>(nowTime - startTime).count();
			if (_waitTrigger_ms > timeout_ms) break;
		}
		errCode = LJS8IF_Trigger(nDeviceId);
		printf("[@(LJS_ACQ_Acquire) Trigger](0x%x)\n", errCode);
		if (errCode != LJS8IF_RC_OK) {
			if (_heightImage[nDeviceId] != NULL) {
				free(_heightImage[nDeviceId]);
				_heightImage[nDeviceId] = NULL;
			}
			if (_luminanceImage[nDeviceId] != NULL) {
				free(_luminanceImage[nDeviceId]);
				_luminanceImage[nDeviceId] = NULL;
			}
			if(_waitTrigger_ms > timeout_ms)
				printf("Timeout occurred while waiting for trigger to be ready.\n");
			return errCode;
		}
	}
	
//---------------------------------------------------------------------
//  Organize parameters related to the acquired image
//---------------------------------------------------------------------
	
	_getParam[nDeviceId].luminance_enabled = profileInfo.byLuminanceOutput;
	_getParam[nDeviceId].x_pointnum = profileInfo.wXPointNum;
	_getParam[nDeviceId].y_pointnum = profileInfo.wYLineNum * setParam->interpolateLines;
	_getParam[nDeviceId].x_pitch_um = profileInfo.dwPitchX * 0.01f;
	_getParam[nDeviceId].y_pitch_um = profileInfo.dwPitchY * 0.01f / setParam->interpolateLines;
	_getParam[nDeviceId].z_pitch_um = profileInfo.dwPitchZ * 0.01f;
	
	return LJS8IF_RC_OK;
}

int LJS_ACQ_AcquireAsync(int nDeviceId, unsigned short **heightImage, unsigned char **luminanceImage, LJS_ACQ_SETPARAM *setParam, LJS_ACQ_GETPARAM *getParam){
	int errCode;
	
	//Allocated memory?
	if ((_heightImage[nDeviceId] == NULL) || (_luminanceImage[nDeviceId] == NULL)) {
		if (_heightImage[nDeviceId] != NULL) {
			free(_heightImage[nDeviceId]);
			_heightImage[nDeviceId] = NULL;
		}
		if (_luminanceImage[nDeviceId] != NULL) {
			free(_luminanceImage[nDeviceId]);
			_luminanceImage[nDeviceId] = NULL;
		}
		return LJS8IF_RC_ERR_NOMEMORY;
	}
	
	if(_imageAvailable[nDeviceId] == 0){
		//printf(" [No image has been acquired]\n");
		return LJS8IF_RC_ERR_TIMEOUT;
	}
	printf(" [@(LJS_ACQ_AcquireAsync) done]\n");

	//Stop HighSpeed
	errCode = LJS8IF_StopHighSpeedDataCommunication(nDeviceId);
	printf("[@(LJS_ACQ_AcquireAsync) Stop HighSpeed](0x%x)\n",errCode);

	_getParam[nDeviceId].isProcTimeoutOccurred = _headTimeoutOccurred[nDeviceId];
	*getParam = _getParam[nDeviceId];

//---------------------------------------------------------------------
//  Copy internal buffer to user buffer
//---------------------------------------------------------------------
	unsigned short* wDataBuf = &_heightImage[nDeviceId][0];
	int xDataNum = _getParam[nDeviceId].x_pointnum;
	int interpolateLines = setParam->interpolateLines;

	//Allocate user memory
	if ((*heightImage != NULL) || (*luminanceImage != NULL)) {
		if (*heightImage != NULL) {
			free(*heightImage);
			*heightImage = NULL;
		}
		if (*luminanceImage != NULL) {
			free(*luminanceImage);
			*luminanceImage = NULL;
		}
	}
	*heightImage = (unsigned short*)malloc(sizeof(short) * xDataNum * _getParam[nDeviceId].y_pointnum);
	*luminanceImage = (unsigned char*)malloc(sizeof(char) * xDataNum * _getParam[nDeviceId].y_pointnum);
	if ((*heightImage == NULL) || (*luminanceImage == NULL)) {
		// failed to allocate user memory
		if (*heightImage != NULL) {
			free(*heightImage);
			*heightImage = NULL;
		}
		if (*luminanceImage != NULL) {
			free(*luminanceImage);
			*luminanceImage = NULL;
		}
		if (_heightImage[nDeviceId] != NULL) {
			free(_heightImage[nDeviceId]);
			_heightImage[nDeviceId] = NULL;
		}
		if (_luminanceImage[nDeviceId] != NULL) {
			free(_luminanceImage[nDeviceId]);
			_luminanceImage[nDeviceId] = NULL;
		}
		return LJS8IF_RC_ERR_NOMEMORY;
	}

	for (int y = 0; y < _yImageSizeAcquired[nDeviceId]; ++y) {
		for (int dy = 0; dy < interpolateLines; ++dy) {
			memcpy(&(*heightImage)[(y * interpolateLines + dy) * xDataNum], &wDataBuf[y * xDataNum], xDataNum * sizeof(unsigned short));
		}
	}

	if (_getParam[nDeviceId].luminance_enabled > 0) {
		unsigned char* byDataBuf = (unsigned char*)&_luminanceImage[nDeviceId][0];
		for (int y = 0; y < _yImageSizeAcquired[nDeviceId]; ++y) {
			for (int dy = 0; dy < interpolateLines; ++dy) {
				memcpy(&(*luminanceImage)[(y * interpolateLines + dy) * xDataNum], &byDataBuf[y * xDataNum], xDataNum * sizeof(unsigned char));
			}
		}
	}
	

	//Free memory
	if (_heightImage[nDeviceId] != NULL) {
		free(_heightImage[nDeviceId]);
		_heightImage[nDeviceId] = NULL;
	}
	if (_luminanceImage[nDeviceId] != NULL) {
		free(_luminanceImage[nDeviceId]);
		_luminanceImage[nDeviceId] = NULL;
	}
	return LJS8IF_RC_OK;
}

static void myCallbackFuncSimpleArray(LJS8IF_PROFILE_HEADER* pProfileHeaderArray, unsigned short* pHeightProfileArray, unsigned char* pLuminanceProfileArray, unsigned int dwLuminanceEnable, unsigned int dwProfileDataCount, unsigned int dwCount, unsigned int dwNotify, unsigned int dwUser) {
	printf(" [@Callback func] dwCount:%d notify:%d\n", (int)dwCount, (int)dwNotify);
	
	if ((dwNotify == 0) || ((dwNotify & 0x10000) > 0)){
		if (_imageAvailable[dwUser] == 0){
			if (dwCount > 0){
				unsigned int sampleCount = dwProfileDataCount * dwCount;
				unsigned int nonzeroHeightCount = 0;
				unsigned short minHeight = 65535;
				unsigned short maxHeight = 0;
				for (unsigned int i = 0; i < sampleCount; ++i) {
					unsigned short value = pHeightProfileArray[i];
					if (value != 0)
						++nonzeroHeightCount;
					if (value < minHeight)
						minHeight = value;
					if (value > maxHeight)
						maxHeight = value;
				}
				printf(" [@Callback stats] height min:%u max:%u nonzero:%u/%u first:%u,%u,%u,%u,%u\n",
					minHeight,
					maxHeight,
					nonzeroHeightCount,
					sampleCount,
					pHeightProfileArray[0],
					sampleCount > 1 ? pHeightProfileArray[1] : 0,
					sampleCount > 2 ? pHeightProfileArray[2] : 0,
					sampleCount > 3 ? pHeightProfileArray[3] : 0,
					sampleCount > 4 ? pHeightProfileArray[4] : 0);

				//flag for head processing timeout
				_headTimeoutOccurred[dwUser] = pProfileHeaderArray->byProcTimeout;
				//height data
				memcpy(&_heightImage[dwUser][0], pHeightProfileArray, sizeof(unsigned short) * dwProfileDataCount * dwCount);

				//luminance data
				if (dwLuminanceEnable != 0){
					memcpy(&_luminanceImage[dwUser][0], pLuminanceProfileArray, sizeof(unsigned char) * dwProfileDataCount * dwCount);
				}

				_yImageSizeAcquired[dwUser] = (int)dwCount;
				_imageAvailable[dwUser] = 1;
			}
		}
	}
}

