//Copyright (c) 2024 KEYENCE CORPORATION. All rights reserved.
/** @file
@brief	LJS8_IF_Linux Implementation
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <errno.h>
#include <new>

#include "LJS8_IF_Linux.h"
#include "LJS8_ErrorCode.h"

#define LJS_CONNECT_TIMEOUT_SEC	(3U)
#define WRITE_DATA_SIZE				(20U * 1024U)
#define SEND_BUF_SIZE				(WRITE_DATA_SIZE + 16U)
#define RECEIVE_BUF_SIZE 			(3000U * 1024U)
#define FAST_RECEIVE_BUF_SIZE		(64U * 1024U)
#define	PARITY_ERROR				(0x40000000)

// Profile length
#define PROF_HEADER_LEN				(6U * sizeof(unsigned int))
#define PROF_CHECKSUM_LEN			(sizeof(unsigned int))
#define PROF_NOT_DATA_LEN			(PROF_HEADER_LEN + PROF_CHECKSUM_LEN)

#define RES_ADRESS_OFFSET_ERROR		(17U)
#define	PREFIX_ERROR_HEAD		(0x8000)

/* definition related to raw level packet */
#define DEF_NOTIFY_MAGIC_NO			(0xFFFFFFFFU)
#define DEF_PROFILE_DATA_MAGIC_NO	(0xFFFFFFFEU)

#define BRIGHTNESS_VALUE			(0x40)
const static unsigned int MASK_CURRENT_HEIGHT_IMAGE_COMMITED = 0x80000000;

/* NOTIFY packet */
typedef struct {
	unsigned int	dwMagicNo;	/* constant 0xFFFFFFFF */
	unsigned int	dwNotify;	/* NOTIFY */
} ST_CIND_HEIGHT_IMAGE_NOTIFY;

typedef struct {
	unsigned char*	ProfileBuffer;				// Buffer for passing data to the user finally in the callback function.
	unsigned char*	pbyCommunicationBuffer;		// Buffer to store newly received raw data buffer.
	unsigned char*	pbyConvertingBuffer;		// Buffer to store converting profile.
	unsigned int	dwConvertingBufferIndex;	// How many data remains in the pbyConvertingBuffer, in bytes.
	unsigned char*	pbyConvertedBuffer;			// Buffer to store converted profile.
	int				bIsEnabled;					// Indicate that the thread is active.
	void			(*pFunc)(unsigned char*, unsigned int, unsigned int, unsigned int, unsigned int);
												// Callback function pointer.
	unsigned int	dwProfileMax;				// When this number of profiles acquired, it wake up the callback function.
	unsigned int	dwProfileLoopCnt;			// Number of profiles collected so far.
	unsigned int	dwProfileCnt;				// Number of data points for each profile.
	unsigned int	dwProfileSize;				// Size of each profile, in bytes.
	unsigned char	byKind;						// Kind of profile (e.g. luminance output is enabled)
	unsigned int	dwThreadId;					// Thread identifier
	//extended
	unsigned int	dwDeviceId;					// Device identifier (e.g. which number of head)
	int				nBufferSize;				// Size of ProfileBuffer, in bytes.
	int				stopFlag;					// (Internal use) flag to terminate thread. 
	void			(*pFuncSimpleArray)(LJS8IF_PROFILE_HEADER* pProfileHeaderArray, unsigned short* pHeightProfileArray, unsigned char* pLuminanceProfileArray, unsigned int dwLuminanceEnable, unsigned int dwProfileDataCount, unsigned int dwCount, unsigned int dwNotify, unsigned int dwUser);
												// Callback function pointer(for Simple Array method)
} THREAD_PARAM_FAST;

// Static variable
static int sockfd[ MAX_LJS_DEVICENUM ];
static int sockfdHighSpeed[ MAX_LJS_DEVICENUM ];
static unsigned char mysendBuf[ MAX_LJS_DEVICENUM ][ SEND_BUF_SIZE ];
static unsigned char myrecvline[ MAX_LJS_DEVICENUM ][ RECEIVE_BUF_SIZE ];
static int  startcode[ MAX_LJS_DEVICENUM ];
static pthread_t pthread[ MAX_LJS_DEVICENUM ];
THREAD_PARAM_FAST m_ThreadParamFast[MAX_LJS_DEVICENUM];

// Function prototype
static int myAnyCommand(int nDeviceId, unsigned char * senddata, int dataLength);
static int sendSingleCommand(int nDeviceId, unsigned char CommandCode);
static int ConvertProfileData(const void* pInData, const unsigned short wXPointNum, bool isBrightness, unsigned short dataCount, LJS8IF_PROFILE_HEADER* pProfileHeaderArray, unsigned short* pHeightProfileArray, unsigned char* pLuminanceProfileArray, unsigned int dwDataSize);
static void *receiveThread(void *args);
static void InitThreadParamFast(int nDeviceId);
static void SetThreadInitParamFast(int nDeviceId, void (*pCallBack)(unsigned char*, unsigned int, unsigned int, unsigned int, unsigned int), unsigned int dwThreadId);
static int SetThreadParamFast(int nDeviceId, unsigned int dwProfileCnt, unsigned int dwLineCnt, unsigned char byKind);

static unsigned int ReceiveDataCheck(unsigned char* pbyRecvData, unsigned int dwRecvSize, unsigned int dwProfSize, unsigned char* pbyConvData, unsigned int* pdwConvSize, unsigned int* pdwNotify);

static void CallbackFuncWrapper( unsigned char *Buf, unsigned int dwSize ,unsigned int dwCount, unsigned int dwNotify, unsigned int  dwUser);

extern "C"
{
//-----------------------------------------------------------------
// LJS8_IF funcitons implementation
//-----------------------------------------------------------------
int LJS8IF_EthernetOpen(int nDeviceId, LJS8IF_ETHERNET_CONFIG* pEthernetConfig)
{
	char ip[15];
	unsigned char *ipn;
	struct sockaddr_in servaddr;
	
	if(pEthernetConfig == NULL)
		return LJS8IF_RC_ERR_PARAMETER;
	
	ipn= pEthernetConfig->abyIpAddress;
	
	sprintf(ip,"%d.%d.%d.%d",ipn[0],ipn[1],ipn[2],ipn[3]);
	
	// Create socket
	sockfd[nDeviceId] = socket(AF_INET,SOCK_STREAM,0);
	if( sockfd[nDeviceId] <0){
		return LJS8IF_RC_ERR_OPEN;
	}
	servaddr.sin_family = AF_INET;
	servaddr.sin_addr.s_addr=inet_addr(ip);
	servaddr.sin_port=htons(pEthernetConfig->wPortNo);

	// Connect socket in non-blocking mode to determine timeout
	// set non-blocking mode
	int flag;
	if( (flag = fcntl(sockfd[nDeviceId], F_GETFL, NULL)) <0){
		return LJS8IF_RC_ERR_OPEN;
	}
	if( fcntl(sockfd[nDeviceId], F_SETFL, flag | O_NONBLOCK) <0){
		return LJS8IF_RC_ERR_OPEN;
	}
	
	if( connect(sockfd[nDeviceId], (struct sockaddr *)&servaddr, sizeof(servaddr)) <0){
		if(errno  != EINPROGRESS){
			return LJS8IF_RC_ERR_OPEN;
		}
		
		struct timeval tv;
		int res;
		fd_set fds;

		tv.tv_sec = LJS_CONNECT_TIMEOUT_SEC;
		tv.tv_usec = 0;
		FD_ZERO(&fds);
		FD_SET(sockfd[nDeviceId], &fds);

		res = select(sockfd[nDeviceId]+1, NULL, &fds, NULL, &tv);

		if(res <= 0){ // Connection timeout
			return LJS8IF_RC_ERR_OPEN;
		}
		else if(res >0){
			socklen_t optlen=sizeof(servaddr);
			
			// Check if socket is writable
			if(getpeername(sockfd[nDeviceId], (struct sockaddr *)&servaddr, &optlen) <0){
				return LJS8IF_RC_ERR_OPEN;
			}
		}
	}
	
	// Blocking mode
	if( fcntl(sockfd[nDeviceId], F_SETFL, flag) <0){
		return LJS8IF_RC_ERR_OPEN;
	}
	
  	return LJS8IF_RC_OK;
}

int LJS8IF_CommunicationClose(int nDeviceId)
{
	// close socket
	shutdown(sockfd[nDeviceId], SHUT_WR);
	
	struct timeval tv;
	int res;
	char buffer[1];

	tv.tv_sec = 1;
	tv.tv_usec = 0;
	fd_set fds;
	FD_ZERO(&fds);
	FD_SET(sockfd[nDeviceId], &fds);

	res = select(sockfd[nDeviceId] + 1, &fds, NULL, NULL, &tv);

	if (res > 0) {
		res = recv(sockfd[nDeviceId], buffer, sizeof(buffer), 0);
		if (res != 0) {
			printf("Shutdown(SHUT_WR, LowSpeed) failed\n");
		}
	}
	else {
		res = -1;
	}

	if (res != 0) {
		shutdown(sockfd[nDeviceId], SHUT_RD);

		tv.tv_sec = 1;
		tv.tv_usec = 0;
		FD_ZERO(&fds);
		FD_SET(sockfd[nDeviceId], &fds);
		res = select(sockfd[nDeviceId] + 1, &fds, NULL, NULL, &tv);

		if (res > 0) {
			res = recv(sockfd[nDeviceId], buffer, sizeof(buffer), 0);
			if (res != 0) {
				printf("Shutdown(SHUT_RD, LowSpeed) failed\n");
			}
		}
		// Terminate communication
	}

	close(sockfd[nDeviceId]);
	return LJS8IF_RC_OK;
}

int LJS8IF_Reboot(int nDeviceId)
{
	return sendSingleCommand(nDeviceId, 0x02);
}

int LJS8IF_ReturnToFactorySetting(int nDeviceId)
{
	return sendSingleCommand(nDeviceId, 0x03);
}

int LJS8IF_ControlLaser(int nDeviceId, unsigned char byState)
{
	unsigned char senddata[] = { 0x2A, 0x00, 0x00, 0x00, byState , 0x00, 0x00, 0x00 };
	return myAnyCommand(nDeviceId, senddata, sizeof(senddata));
}

int LJS8IF_GetError(int nDeviceId, unsigned char byReceivedMax, unsigned char* pbyErrCount, unsigned short* pwErrCode)
{
	int res = sendSingleCommand(nDeviceId, 0x04);
	if(LJS8IF_RC_OK != res) return res;
	
	unsigned char numberOfError = myrecvline[nDeviceId][28];
	unsigned char copyNum;
	
	// Number of errors
	*pbyErrCount = numberOfError;
	
	// Copy error infomation
	if(numberOfError >= 1){
		if(byReceivedMax > numberOfError){
			copyNum = numberOfError;
		}
		else{
			copyNum = byReceivedMax;
		}
		memcpy(pwErrCode, &myrecvline[nDeviceId][32], sizeof(unsigned short) * copyNum );
	}

	return LJS8IF_RC_OK;
}

int LJS8IF_ClearError(int nDeviceId, unsigned short wErrCode)
{
	unsigned char *abyDat = (unsigned char*)&wErrCode;
	unsigned char senddata[] = { 0x05, 0x00, 0x00, 0x00, abyDat[0], abyDat[1], 0x00, 0x00 };
	return myAnyCommand(nDeviceId, senddata, sizeof(senddata));
}

int LJS8IF_GetHeadTemperature(int nDeviceId, short* pnSensorTemperature, short* pnProcessor1Temperature, short* pnProcessor2Temperature, short* pnCaseTemperature, short* pnDriveUnitTemperature)
{
	int res = sendSingleCommand(nDeviceId, 0x4C);
	if(LJS8IF_RC_OK != res) return res;

	*pnSensorTemperature		= *(short*)(&myrecvline[nDeviceId][28]);
	*pnProcessor1Temperature	= *(short*)(&myrecvline[nDeviceId][30]);
	*pnProcessor2Temperature	= *(short*)(&myrecvline[nDeviceId][32]);
	*pnCaseTemperature			= *(short*)(&myrecvline[nDeviceId][34]);
	*pnDriveUnitTemperature		= *(short*)(&myrecvline[nDeviceId][36]);
	
	return LJS8IF_RC_OK;
}

int LJS8IF_GetHeadModel(int nDeviceId, char* pHeadModel)
{
	int res = sendSingleCommand(nDeviceId, 0x01);
	if(LJS8IF_RC_OK != res) return res;
	
	memcpy(pHeadModel, &myrecvline[nDeviceId][44], 32);
	
	return LJS8IF_RC_OK;
}

int LJS8IF_GetSerialNumber(int nDeviceId, char* pSerialNo)
{
	int res = sendSingleCommand(nDeviceId, 0x06);
	if(LJS8IF_RC_OK != res) return res;
	
	memcpy(pSerialNo, &myrecvline[nDeviceId][92], 16);
		
	return LJS8IF_RC_OK;
}

int LJS8IF_GetAttentionStatus(int nDeviceId, unsigned short* pwAttentionStatus)
{
	int res = sendSingleCommand(nDeviceId, 0x65);
	if(LJS8IF_RC_OK != res) return res;
	
	*pwAttentionStatus 	= *(unsigned short*)(&myrecvline[nDeviceId][28]);
	
	return LJS8IF_RC_OK;
}

int LJS8IF_Trigger(int nDeviceId)
{
	return sendSingleCommand(nDeviceId, 0x24);
}

int LJS8IF_ClearMemory(int nDeviceId)
{
	return sendSingleCommand(nDeviceId, 0x27);
}

int LJS8IF_SetSetting(int nDeviceId, unsigned char byDepth, LJS8IF_TARGET_SETTING TargetSetting, void* pData, unsigned int dwDataSize, unsigned int* pdwError)
{
	LJS8IF_TARGET_SETTING *tset = &TargetSetting;
	
	unsigned char header_dat[] = { 0x32, 0x00, 0x00, 0x00, 
								byDepth, 0x00, 0x00, 0x00, 
								tset->byType,
								tset->byCategory,
								tset->byItem,
								0x00,
								tset->byTarget1,
								tset->byTarget2,
								tset->byTarget3,
								tset->byTarget4};
								
	struct
	{
		unsigned char header[sizeof(header_dat)];
		unsigned char body[WRITE_DATA_SIZE];
	} stCommand;
	
	if(dwDataSize > WRITE_DATA_SIZE) return LJS8IF_RC_ERR_PARAMETER;
	
	bzero(&stCommand, sizeof(stCommand));
	memcpy(stCommand.header, header_dat, sizeof(stCommand.header));
	memcpy(stCommand.body, pData, dwDataSize);
	
	int res = myAnyCommand(nDeviceId, &stCommand.header[0], sizeof(stCommand.header)+ dwDataSize);
	if(LJS8IF_RC_OK != res) return res;
	
	*pdwError 	= *(unsigned int*)(&myrecvline[nDeviceId][28]);
	
	return LJS8IF_RC_OK;
}

int LJS8IF_GetSetting(int nDeviceId, unsigned char byDepth, LJS8IF_TARGET_SETTING TargetSetting, void* pData, unsigned int dwDataSize)
{
	LJS8IF_TARGET_SETTING *tset = &TargetSetting;
	
	unsigned char senddata[] = { 0x31, 0x00, 0x00, 0x00, 
								0x00, 0x00, 0x00, 0x00,
								byDepth, 0x00, 0x00, 0x00, 
								tset->byType,
								tset->byCategory,
								tset->byItem,
								0x00,
								tset->byTarget1,
								tset->byTarget2,
								tset->byTarget3,
								tset->byTarget4};
								
	int res = myAnyCommand(nDeviceId, senddata, sizeof(senddata));
	if(LJS8IF_RC_OK != res) return res;

	// Copy received setting
	memcpy(pData, &myrecvline[nDeviceId][28], dwDataSize);
	
	return LJS8IF_RC_OK;
}

int LJS8IF_InitializeSetting(int nDeviceId, unsigned char byDepth, unsigned char byTarget)
{
	unsigned char senddata[] = { 0x3D, 0x00, 0x00, 0x00, 
								byDepth, 0x00, 0x00, 0x00, 
								0x03, byTarget, 0x00, 0x00};
	return myAnyCommand(nDeviceId, senddata, sizeof(senddata));
}

int LJS8IF_ReflectSetting(int nDeviceId, unsigned char byDepth, unsigned int* pdwError)
{
	unsigned char senddata[] = { 0x33, 0x00, 0x00, 0x00, byDepth, 0x00, 0x00, 0x00 };
	int res = myAnyCommand(nDeviceId, senddata, sizeof(senddata));
	if(LJS8IF_RC_OK != res) return res;

	*pdwError 	= *(unsigned int*)(&myrecvline[nDeviceId][28]);
	
	return LJS8IF_RC_OK;
}

int LJS8IF_RewriteTemporarySetting(int nDeviceId, unsigned char byDepth)
{
	if(byDepth == 1 || byDepth == 2){
		byDepth -=1;
	}
	else{
		byDepth = 0xFF;
	}
	unsigned char senddata[] = { 0x35, 0x00, 0x00, 0x00, byDepth, 0x00, 0x00, 0x00 };
	return myAnyCommand(nDeviceId, senddata, sizeof(senddata));
}

int LJS8IF_CheckMemoryAccess(int nDeviceId, unsigned char* pbyBusy)
{
	int res = sendSingleCommand(nDeviceId, 0x34);
	if(LJS8IF_RC_OK != res) return res;
	
	*pbyBusy 	= *(unsigned char*)(&myrecvline[nDeviceId][28]);
	
	return LJS8IF_RC_OK;
}

int LJS8IF_ChangeActiveProgram(int nDeviceId, unsigned char byProgramNo)
{
	unsigned char senddata[] = { 0x39, 0x00, 0x00, 0x00, byProgramNo, 0x00, 0x00, 0x00 };
	return myAnyCommand(nDeviceId, senddata, sizeof(senddata));
}

int LJS8IF_GetActiveProgram(int nDeviceId, unsigned char* pbyProgramNo)
{
	//There is no TCP/IP command to get the active program number directly.
	//Instead, use the "GetAttentionStatus command to get the information stored in the response.
	int res = sendSingleCommand(nDeviceId, 0x3F);
	if(LJS8IF_RC_OK != res) return res;
	
	*pbyProgramNo 	= *(unsigned char*)(&myrecvline[nDeviceId][24]);
	
	return LJS8IF_RC_OK;
}

int LJS8IF_GetHeightImageSimpleArray(int nDeviceId, LJS8IF_GET_HEIGHT_IMAGE_PROFILE_REQUEST* pReq, unsigned char byUsePCImageFilter, LJS8IF_GET_HEIGHT_IMAGE_PROFILE_RESPONSE* pRsp, LJS8IF_HEIGHT_IMAGE_INFO* pHeightImageInfo, LJS8IF_PROFILE_HEADER* pProfileHeaderArray, unsigned short* pHeightProfileArray, unsigned char* pLuminanceProfileArray)
{
	unsigned char* abyHeightImageNo = (unsigned char*)(&pReq->dwGetHeightImageNo);
	unsigned char* abyProfNo = (unsigned char*)(&pReq->dwGetProfileNo);

	unsigned char senddata[] = { 0x42, 0x00, 0x00, 0x00,
								0x00, 0x00, 0x00, 0x00,
								0x00,
								pReq->byPositionMode,
								0x00, 0x00,
								abyHeightImageNo[0], abyHeightImageNo[1], abyHeightImageNo[2], abyHeightImageNo[3],
								abyProfNo[0], abyProfNo[1], abyProfNo[2], abyProfNo[3],
								(unsigned char)(pReq->wGetProfileCount & 0xFF),
								(unsigned char)((pReq->wGetProfileCount >> 8) & 0xFF),
								pReq->byErase,
								0x00};

	if (byUsePCImageFilter != 0) return LJS8IF_RC_ERR_FILTER_UNSUPPORTED;

	int res = myAnyCommand(nDeviceId, senddata, sizeof(senddata));
	if (LJS8IF_RC_OK != res) return res;

	//*pRsp
	pRsp->dwCurrentHeightImageNo = *(unsigned int*)(&myrecvline[nDeviceId][28]);
	pRsp->dwCurrentHeightImageProfileCount = *(unsigned int*)(&myrecvline[nDeviceId][32]) & ~MASK_CURRENT_HEIGHT_IMAGE_COMMITED;
	pRsp->dwOldestHeightImageNo = *(unsigned int*)(&myrecvline[nDeviceId][36]);
	pRsp->dwOldestHeightImageProfileCount = *(unsigned int*)(&myrecvline[nDeviceId][40]);
	pRsp->dwGetHeightImageNo = *(unsigned int*)(&myrecvline[nDeviceId][44]);
	pRsp->dwGetHeightImageProfileCount = *(unsigned int*)(&myrecvline[nDeviceId][48]);
	pRsp->dwGetHeightImageTopProfileNo = *(unsigned int*)(&myrecvline[nDeviceId][52]);
	pRsp->wGetProfileCount = *(unsigned short*)(&myrecvline[nDeviceId][56]);
	pRsp->byCurrentHeightImageCommitted = ((*(unsigned int*)(&myrecvline[nDeviceId][32]) & MASK_CURRENT_HEIGHT_IMAGE_COMMITED) > 0) ? 1 : 0;

	//*pHeightImageInfo
	unsigned char kind = myrecvline[nDeviceId][64];
	pHeightImageInfo->wXPointNum = *(unsigned short*)(&myrecvline[nDeviceId][68]);
	pHeightImageInfo->wYLineNum = *(unsigned short*)(&myrecvline[nDeviceId][70]);
	pHeightImageInfo->byLuminanceOutput = (BRIGHTNESS_VALUE & kind) > 0 ? 1 : 0;
	pHeightImageInfo->nXStart = *(int*)(&myrecvline[nDeviceId][72]);
	pHeightImageInfo->dwPitchX = *(unsigned int*)(&myrecvline[nDeviceId][76]);
	pHeightImageInfo->nYStart = *(int*)(&myrecvline[nDeviceId][80]);
	pHeightImageInfo->dwPitchY = *(unsigned int*)(&myrecvline[nDeviceId][84]);
	pHeightImageInfo->dwPitchZ = *(unsigned int*)(&myrecvline[nDeviceId][92]);

	//*pdwProfileData
	unsigned char* beforeConvData = &myrecvline[nDeviceId][108];
	unsigned int isBrightness = pHeightImageInfo->byLuminanceOutput;
	int nMultipleValue = isBrightness ? 3 : 2;
	
	const int MaxProfileCount = 3200;
	auto bufferSize = pReq->wGetProfileCount * (MaxProfileCount * nMultipleValue + sizeof(LJS8IF_PROFILE_HEADER) + sizeof(LJS8IF_PROFILE_FOOTER));

	res = ConvertProfileData(
		beforeConvData,
		pHeightImageInfo->wXPointNum,
		pHeightImageInfo->byLuminanceOutput,
		pRsp->wGetProfileCount,
		pProfileHeaderArray,
		pHeightProfileArray,
		pLuminanceProfileArray,
		bufferSize);

	return res;
}

int LJS8IF_PreStartHighSpeedDataCommunication(int nDeviceId, LJS8IF_HIGH_SPEED_PRE_START_REQ* pReq, unsigned char byUsePCImageFilter, LJS8IF_HEIGHT_IMAGE_INFO* pHeightImageInfo)
{
	unsigned char profileKind;

	unsigned char senddata[] = { 0x47, 0x00, 0x00, 0x00, (pReq->bySendPosition), 0x00, 0x00, 0x00 };

	if (byUsePCImageFilter != 0) return LJS8IF_RC_ERR_FILTER_UNSUPPORTED;

	int res = myAnyCommand(nDeviceId, senddata, sizeof(senddata));
	if (LJS8IF_RC_OK != res) return res;

	startcode[nDeviceId] = *(int*)(&myrecvline[nDeviceId][32]);
	profileKind = myrecvline[nDeviceId][40];

	pHeightImageInfo->wXPointNum = *(unsigned short*)(&myrecvline[nDeviceId][44]);
	pHeightImageInfo->wYLineNum = *(unsigned short*)(&myrecvline[nDeviceId][46]);
	pHeightImageInfo->byLuminanceOutput = (BRIGHTNESS_VALUE & profileKind) > 0 ? 1 : 0;
	pHeightImageInfo->nXStart = *(int*)(&myrecvline[nDeviceId][48]);
	pHeightImageInfo->dwPitchX = *(unsigned int*)(&myrecvline[nDeviceId][52]);
	pHeightImageInfo->nYStart = *(int*)(&myrecvline[nDeviceId][56]);
	pHeightImageInfo->dwPitchY = *(unsigned int*)(&myrecvline[nDeviceId][60]);
	pHeightImageInfo->dwPitchZ = *(unsigned int*)(&myrecvline[nDeviceId][68]);
	
	SetThreadParamFast(nDeviceId, pHeightImageInfo->wXPointNum, pHeightImageInfo->wYLineNum, profileKind);

	return LJS8IF_RC_OK;
}

int LJS8IF_StartHighSpeedDataCommunication(int nDeviceId)
{
	unsigned char *abyDat = (unsigned char*)&startcode[nDeviceId];
	unsigned char senddata[] = { 0xA0, 0x00, 0x00, 0x00, 0x47, 0x00, 0x00, 0x00, abyDat[0], abyDat[1], abyDat[2], abyDat[3] };
	return myAnyCommand(nDeviceId, senddata, sizeof(senddata));
}

int LJS8IF_StopHighSpeedDataCommunication(int nDeviceId)
{
	return sendSingleCommand(nDeviceId, 0x48);
}

int LJS8IF_InitializeHighSpeedDataCommunicationSimpleArray(int nDeviceId, LJS8IF_ETHERNET_CONFIG* pEthernetConfig, unsigned short wHighSpeedPortNo, LJS8IF_CALLBACK_SIMPLE_ARRAY pCallBackSimpleArray, unsigned int dwThreadId)
{
	char ip[15];
	unsigned char* ipn;
	struct sockaddr_in servaddr;

	int rc = LJS8IF_RC_OK;

	m_ThreadParamFast[nDeviceId].pFuncSimpleArray = pCallBackSimpleArray;

	if (pEthernetConfig == NULL)
		return LJS8IF_RC_ERR_PARAMETER;

	ipn = pEthernetConfig->abyIpAddress;

	sprintf(ip, "%d.%d.%d.%d", ipn[0], ipn[1], ipn[2], ipn[3]);
	sockfdHighSpeed[nDeviceId] = socket(AF_INET, SOCK_STREAM, 0);

	if (sockfdHighSpeed[nDeviceId] < 0) {
		return LJS8IF_RC_ERR_OPEN;
	}

	linger linger_opt;
	linger_opt.l_onoff = 1;
	linger_opt.l_linger = 1;

	if (setsockopt(sockfdHighSpeed[nDeviceId], SOL_SOCKET, SO_LINGER, reinterpret_cast<char*>(&linger_opt), sizeof(linger_opt)) < 0) {
		close(sockfdHighSpeed[nDeviceId]);
		return LJS8IF_RC_ERR_OPEN;
	}
	
	servaddr.sin_family = AF_INET;
	servaddr.sin_addr.s_addr = inet_addr(ip);
	servaddr.sin_port = htons(wHighSpeedPortNo);

	// Connect socket in non-blocking mode to determine timeout
	// set non-blocking mode
	int flag;
	if ((flag = fcntl(sockfdHighSpeed[nDeviceId], F_GETFL, NULL)) < 0) {
		return LJS8IF_RC_ERR_OPEN;
	}
	if (fcntl(sockfdHighSpeed[nDeviceId], F_SETFL, flag | O_NONBLOCK) < 0) {
		return LJS8IF_RC_ERR_OPEN;
	}

	if (connect(sockfdHighSpeed[nDeviceId], (struct sockaddr*)&servaddr, sizeof(servaddr)) < 0) {
		if (errno != EINPROGRESS) {
			return LJS8IF_RC_ERR_OPEN;
		}

		struct timeval tv;
		int res;
		fd_set fds;

		tv.tv_sec = LJS_CONNECT_TIMEOUT_SEC;
		tv.tv_usec = 0;
		FD_ZERO(&fds);
		FD_SET(sockfdHighSpeed[nDeviceId], &fds);

		res = select(sockfdHighSpeed[nDeviceId] + 1, NULL, &fds, NULL, &tv);

		if (res <= 0) { // Connection timeout
			return LJS8IF_RC_ERR_OPEN;
		}
		else if (res > 0) {
			socklen_t optlen = sizeof(servaddr);

			// Check if socket is writable
			if (getpeername(sockfdHighSpeed[nDeviceId], (struct sockaddr*)&servaddr, &optlen) < 0) {
				return LJS8IF_RC_ERR_OPEN;
			}
		}
	}

	// Blocking mode
	if (fcntl(sockfdHighSpeed[nDeviceId], F_SETFL, flag) < 0) {
		return LJS8IF_RC_ERR_OPEN;
	}

	InitThreadParamFast(nDeviceId);

	SetThreadInitParamFast(nDeviceId, &CallbackFuncWrapper, dwThreadId);

	do {
		//Allocate buffer for raw data
		if (m_ThreadParamFast[nDeviceId].pbyCommunicationBuffer == NULL) {
			m_ThreadParamFast[nDeviceId].pbyCommunicationBuffer = new(std::nothrow) unsigned char[FAST_RECEIVE_BUF_SIZE];
			if (m_ThreadParamFast[nDeviceId].pbyCommunicationBuffer == NULL) {
				rc = LJS8IF_RC_ERR_NOMEMORY;
				break;
			}
		}
		//Allocate buffer for converting data
		if (m_ThreadParamFast[nDeviceId].pbyConvertingBuffer == NULL) {
			m_ThreadParamFast[nDeviceId].pbyConvertingBuffer = new(std::nothrow) unsigned char[static_cast<int>(FAST_RECEIVE_BUF_SIZE * 2)];
			if (m_ThreadParamFast[nDeviceId].pbyConvertingBuffer == NULL) {
				rc = LJS8IF_RC_ERR_NOMEMORY;
				break;
			}
		}
		//Allocate buffer for converted data
		if (m_ThreadParamFast[nDeviceId].pbyConvertedBuffer == NULL) {
			m_ThreadParamFast[nDeviceId].pbyConvertedBuffer = new(std::nothrow) unsigned char[FAST_RECEIVE_BUF_SIZE];
			if (m_ThreadParamFast[nDeviceId].pbyConvertedBuffer == NULL) {
				rc = LJS8IF_RC_ERR_NOMEMORY;
				break;
			}
		}

		//Create a thread for receiving high-speed data
		m_ThreadParamFast[nDeviceId].stopFlag = 0;
		int res = pthread_create(&pthread[nDeviceId], NULL, &receiveThread, (void*)&m_ThreadParamFast[nDeviceId]);
		usleep(1000);
		if (res != 0) {
			rc = LJS8IF_RC_ERR_OPEN;
			break;
		}else{
			// Wait for the thread to start (max 100ms)
			rc = LJS8IF_RC_ERR_OPEN;
			for (int i = 0; i < 10; i++) {
				if (m_ThreadParamFast[nDeviceId].bIsEnabled) {
					rc = LJS8IF_RC_OK;
					break;
				}
				usleep(10000);
			}
			if (rc != LJS8IF_RC_OK) {
				// If an error occurs, terminate the thread
				if (pthread[nDeviceId] != 0) {
					pthread_cancel(pthread[nDeviceId]);
					pthread[nDeviceId] = 0;
				}
			}
		}
	} while (0);

	return 	(int)rc;
}

int LJS8IF_FinalizeHighSpeedDataCommunication(int nDeviceId)
{	
	//Stop high-speed communication
	LJS8IF_StopHighSpeedDataCommunication(nDeviceId);
	
	//Terminate the thread for receiving high-speed data
	m_ThreadParamFast[nDeviceId].stopFlag = 1;
	if (pthread[nDeviceId] != 0) {
		pthread_join(pthread[nDeviceId], NULL);
		pthread[nDeviceId] = 0;
	}
	
	//Close high speed communication socket
	shutdown(sockfdHighSpeed[nDeviceId], SHUT_WR);

	struct timeval tv;
	int res;
	char buffer[1];

	tv.tv_sec = 1;
	tv.tv_usec = 0;
	fd_set fds;
	FD_ZERO(&fds);
	FD_SET(sockfdHighSpeed[nDeviceId], &fds);

	res = select(sockfdHighSpeed[nDeviceId] + 1, &fds, NULL, NULL, &tv);

	if (res > 0) {
		res = recv(sockfdHighSpeed[nDeviceId], buffer, sizeof(buffer), 0);
		if (res != 0) {
			printf("Shutdown(SHUT_WR, HighSpeed) failed\n");
		}
	}
	else {
		res = -1;
	}
	
	if (res != 0) {
		shutdown(sockfdHighSpeed[nDeviceId], SHUT_RD);

		tv.tv_sec = 1;
		tv.tv_usec = 0;
		FD_ZERO(&fds);
		FD_SET(sockfdHighSpeed[nDeviceId], &fds);
		res = select(sockfdHighSpeed[nDeviceId] + 1, &fds, NULL, NULL, &tv);

		if (res > 0) {
			res = recv(sockfdHighSpeed[nDeviceId], buffer, sizeof(buffer), 0);
			if (res != 0) {
				printf("Shutdown(SHUT_RD, HighSpeed) failed\n");
			}
		}
		// Terminate communication
	}

	close(sockfdHighSpeed[nDeviceId]);

	//Release buffer
	if (m_ThreadParamFast[nDeviceId].ProfileBuffer != NULL) {
		free(m_ThreadParamFast[nDeviceId].ProfileBuffer);
		m_ThreadParamFast[nDeviceId].ProfileBuffer = NULL;
	}
	if (m_ThreadParamFast[nDeviceId].pbyCommunicationBuffer != NULL) {
		delete[] m_ThreadParamFast[nDeviceId].pbyCommunicationBuffer;
		m_ThreadParamFast[nDeviceId].pbyCommunicationBuffer = NULL;
	}
	if (m_ThreadParamFast[nDeviceId].pbyConvertingBuffer != NULL) {
		delete[] m_ThreadParamFast[nDeviceId].pbyConvertingBuffer;
		m_ThreadParamFast[nDeviceId].pbyConvertingBuffer = NULL;
	}
	if (m_ThreadParamFast[nDeviceId].pbyConvertedBuffer != NULL) {
		delete[] m_ThreadParamFast[nDeviceId].pbyConvertedBuffer;
		m_ThreadParamFast[nDeviceId].pbyConvertedBuffer = NULL;
	}
	
	return LJS8IF_RC_OK;
}

}//extern "C"


//-----------------------------------------------------------------
// Internal funcitons implementation
//-----------------------------------------------------------------
static void *receiveThread(void *args)
{	
	THREAD_PARAM_FAST *pThreadParam = (THREAD_PARAM_FAST *) args;
	int nDeviceId = pThreadParam->dwDeviceId;
	pThreadParam->bIsEnabled = 1;
	
	int m_nNumData = 0;
	int m_nWriteIndex =0;
	
	int dwNumReceived;
	
	//for select
	struct timeval tv;
	fd_set fds, readfds;
	FD_ZERO(&readfds);
	FD_SET(sockfdHighSpeed[nDeviceId],&readfds);

	while(1){ //loop1
		if(pThreadParam->stopFlag){
			break;
		}
		
		// Use select for non-blocking recv
		tv.tv_sec = 1;
		tv.tv_usec = 0;
		memcpy(&fds, &readfds, sizeof(fd_set));
		select(sockfdHighSpeed[nDeviceId]+1, &fds, NULL, NULL, &tv);
		dwNumReceived = 0;

		if( FD_ISSET(sockfdHighSpeed[nDeviceId], &fds)){
			dwNumReceived = recv(sockfdHighSpeed[nDeviceId],(unsigned char*)pThreadParam->pbyCommunicationBuffer, FAST_RECEIVE_BUF_SIZE, 0);
		}
		
		if (dwNumReceived <= 0) {
			continue;
		}

		if(pThreadParam->stopFlag){
			break;
		}
		
		// Receive some data,
		// Copy data from newly received buffer to pre-conversion buffer.
		if (FAST_RECEIVE_BUF_SIZE * 2 < pThreadParam->dwConvertingBufferIndex + dwNumReceived) {
			// Irregular case.
			printf("***Highspeed Batch Receive thread*** : ERROR pThreadParam->pbyConvertingBuffer over flow.%d\n", dwNumReceived);
		}
		memcpy(pThreadParam->pbyConvertingBuffer + pThreadParam->dwConvertingBufferIndex, pThreadParam->pbyCommunicationBuffer, dwNumReceived);
		pThreadParam->dwConvertingBufferIndex += dwNumReceived;
		unsigned char* pbyConvBefore = pThreadParam->pbyConvertingBuffer;
		unsigned int dwNotify = 0;
		
		int rc = 1;
			
		while (1) { //loop2
			while (1) { //loop3
				// Check the received data.
				unsigned int dwConvSize = 0;
				unsigned int dwNumUsed = ReceiveDataCheck(pbyConvBefore, pThreadParam->dwConvertingBufferIndex, pThreadParam->dwProfileSize,
					pThreadParam->pbyConvertedBuffer, &dwConvSize, &dwNotify);

				// Received data lacks. Wait for next data.
				if (dwNumUsed == 0) break;

				// In case of "notify" packet, notify upper function.
				// if you get parity error, read all.
				if ((dwNotify & ~PARITY_ERROR) != 0) {
					pThreadParam->dwConvertingBufferIndex -= dwNumUsed;
					pbyConvBefore += dwNumUsed;
					break;
				}

				// The requested number of profiles has been collected.
				if (pThreadParam->dwProfileMax <= pThreadParam->dwProfileLoopCnt) break;

				int nSizeOfData = dwConvSize;
				unsigned char* m_pBuffer = pThreadParam->ProfileBuffer;
				unsigned char* pbyCopyData = pThreadParam->pbyConvertedBuffer;

				// Check if the buffer size is not exceeded.
				if (m_nNumData + nSizeOfData <= pThreadParam->nBufferSize) {
					if (m_nWriteIndex + nSizeOfData > pThreadParam->nBufferSize) {
						memcpy(m_pBuffer + m_nWriteIndex, pbyCopyData, pThreadParam->nBufferSize - m_nWriteIndex);
						pbyCopyData += (pThreadParam->nBufferSize - m_nWriteIndex);
						nSizeOfData -= (pThreadParam->nBufferSize - m_nWriteIndex);
						m_nNumData += (pThreadParam->nBufferSize - m_nWriteIndex);
						m_nWriteIndex = 0;
					}
					if (nSizeOfData > 0) {
						memcpy(m_pBuffer + m_nWriteIndex, pbyCopyData, nSizeOfData);
						m_nWriteIndex = (m_nWriteIndex + nSizeOfData) % pThreadParam->nBufferSize;
						m_nNumData += nSizeOfData;
					}
					rc = 1;
				}
				else {	// Irregular case. The buffer size is exceeded.
					printf("Irregular case. The buffer size is exceeded.\n");
					rc = 0;
					break;
				}

				pThreadParam->dwConvertingBufferIndex -= dwNumUsed;
				pbyConvBefore += dwNumUsed;
				pThreadParam->dwProfileLoopCnt++;
			} //loop3

			// Irregular case. Fail to copy data to buffer.
			if (rc == 0) {
				printf("Irregular case. Fail to copy data to buffer.\n");
				break;
			}

			// If profile data is received and there is remaining received data, pad them in the conversion buffer.
			if ((pThreadParam->dwConvertingBufferIndex) && (pThreadParam->pbyConvertingBuffer != pbyConvBefore)) {
				memcpy(pThreadParam->pbyConvertingBuffer, pbyConvBefore, pThreadParam->dwConvertingBufferIndex);
				pbyConvBefore = pThreadParam->pbyConvertingBuffer;
			}
			
			// Received data lacks. Wait for next data.
			if (dwNotify == 0 && pThreadParam->dwProfileLoopCnt < pThreadParam->dwProfileMax) break;

			// "Notify" packet or the requested number of profiles has been collected.(If the height image does not have all the lines, it is treated as empty data and a callback is notified.)
			unsigned int dwLineCnt = pThreadParam->dwProfileMax == pThreadParam->dwProfileLoopCnt ? pThreadParam->dwProfileLoopCnt : 0;
			unsigned char* pbyBuffer = pThreadParam->ProfileBuffer;
			pThreadParam->pFunc(pbyBuffer, pThreadParam->dwProfileSize, dwLineCnt, dwNotify, pThreadParam->dwThreadId);

			m_nNumData = 0;
			m_nWriteIndex = 0;
			pThreadParam->dwProfileLoopCnt = 0;

			// "Notify" packet and "stop continuous" transmission
			if (dwNotify & 0x0000ffff) {
				pThreadParam->dwProfileSize = 0;
				pThreadParam->dwConvertingBufferIndex = 0;
				break;
			}
		} //loop2

		if (!rc) {
			// Irregular case. The buffer size is exceeded or parity error.
			break;
		}
	}//loop1
	
	//Terminate thread
	pThreadParam->bIsEnabled = 0;
	
	pthread_exit(0);
}

static int sendSingleCommand(int nDeviceId, unsigned char CommandCode)
{
	unsigned char senddata[] = { CommandCode, 0x00, 0x00, 0x00 };
	return myAnyCommand(nDeviceId, senddata, sizeof(senddata));
}

static int myAnyCommand(int nDeviceId, unsigned char* senddata, int dataLength)
{
	int dataOffset = 16;
	int tcpLength = dataOffset + dataLength -4;
	int *lengthPtr;
	
	lengthPtr = (int*)(&mysendBuf[nDeviceId][0]);
	*lengthPtr = tcpLength;
	
	mysendBuf[nDeviceId][4] = 0x05;
	mysendBuf[nDeviceId][5] = 0x00;
	mysendBuf[nDeviceId][6] = 0xF0;
	mysendBuf[nDeviceId][7] = 0x01;
		
	lengthPtr = (int*)(&mysendBuf[nDeviceId][12]);
	*lengthPtr = dataLength;

	// Clear receive buffer before send data.
	{
		// for select
		struct timeval tv;
		fd_set fds, readfds;
		FD_ZERO(&readfds);
		FD_SET(sockfd[nDeviceId],&readfds);

		// use select for non-blocking recv
		tv.tv_sec = 0;
		tv.tv_usec = 1;
		memcpy(&fds, &readfds, sizeof(fd_set));
		select(sockfd[nDeviceId]+1, &fds, NULL, NULL, &tv);

		if( FD_ISSET(sockfd[nDeviceId], &fds)){
			recv(sockfd[nDeviceId],&myrecvline[nDeviceId][0],RECEIVE_BUF_SIZE, 0);
		}
		
		//clear buffer
		bzero(&myrecvline[nDeviceId][0],RECEIVE_BUF_SIZE);
	}


	// Send
	memcpy(&mysendBuf[nDeviceId][0] + dataOffset, senddata, dataLength);
	auto res = send(sockfd[nDeviceId], &mysendBuf[nDeviceId][0], tcpLength + 4, MSG_DONTROUTE); //send command
	if(res == -1){
		return LJS8IF_RC_ERR_SEND;
	}
	
	// Check if the minimum data has arrived
	int n = 0;
	while( n < 4 ){
		n = recv(sockfd[nDeviceId], &myrecvline[nDeviceId][0], RECEIVE_BUF_SIZE, MSG_PEEK);
		if(n==-1){
			return LJS8IF_RC_ERR_RECEIVE;
		}
	}
	// Receive rest of the data
	int recvLength = *(int *)&myrecvline[nDeviceId][0];
	int targetLength = recvLength + 4 ;
	int tryBytes	 = 0;
	int receivedBytes = 0;
	
	// Receivable size at one time is limited due to the recv function.
	// Divide and receive repeatedly.
	while( receivedBytes < targetLength ){
		
		tryBytes = targetLength - receivedBytes;
		if(tryBytes > 65535)
		{
			tryBytes = 65535;
		}
		
		n = 0;
		while( n < tryBytes ){
			n = recv(sockfd[nDeviceId], &myrecvline[nDeviceId][receivedBytes], tryBytes, MSG_PEEK);
			if(n==-1){
				return LJS8IF_RC_ERR_RECEIVE;
			}
		}
		
		recv(sockfd[nDeviceId], &myrecvline[nDeviceId][receivedBytes], tryBytes, MSG_WAITALL);
		receivedBytes += tryBytes;
		
		//printf("<%u>/<%u>\n",receivedBytes,targetLength);
	}
	myrecvline[nDeviceId][targetLength]=0; //add string termination character
	
	// Check response code
	unsigned char errCode= myrecvline[nDeviceId][RES_ADRESS_OFFSET_ERROR];

	if (errCode != 0){
		return ( PREFIX_ERROR_HEAD + errCode );
	}
	return LJS8IF_RC_OK;
}

static int ConvertProfileData(
	const void* pInData, 
	const unsigned short wXPointNum,
	bool isBrightness, 
	unsigned short dataCount,
	LJS8IF_PROFILE_HEADER* pProfileHeaderArray,
	unsigned short* pHeightProfileArray, 
	unsigned char* pLuminanceProfileArray,
	unsigned int dwDataSize)
{
	// Calculate profile size
	int nMultipleValue = (isBrightness) ? 3 : 2;
	
	int nProfileDataSize = wXPointNum * nMultipleValue * sizeof(unsigned char);
	int nHeightDataSize = wXPointNum * sizeof(unsigned short);
	int nLuminanceDataSize = wXPointNum * sizeof(unsigned char);

	int nHeaderSize = sizeof(LJS8IF_PROFILE_HEADER);
	int nFooterSize = sizeof(LJS8IF_PROFILE_FOOTER);


	// Check if the user buffer size is enough
	if (dwDataSize < (unsigned int)((nProfileDataSize + nHeaderSize + nFooterSize) * dataCount)) return LJS8IF_RC_ERR_BUFFER_SHORT;

	const unsigned char* pProfileRawData = (const unsigned char*)pInData;
	LJS8IF_PROFILE_HEADER* pProfileHeaderArrayOut = pProfileHeaderArray;
	unsigned short* pHeightOut = pHeightProfileArray;
	unsigned char* pLuminanceOut = pLuminanceProfileArray;

	for (unsigned short i = 0; i < dataCount; i++)
	{
		// temporary buffer for one profile
		unsigned char* aProfileData = new(std::nothrow) unsigned char[nProfileDataSize + nHeaderSize + nFooterSize];
		if (aProfileData == NULL) return LJS8IF_RC_ERR_NOMEMORY;
		memcpy(aProfileData, pProfileRawData, nProfileDataSize + nHeaderSize + nFooterSize);

		//*pProfileHeaderArray
		memcpy(pProfileHeaderArrayOut, aProfileData, nHeaderSize);
		pProfileHeaderArrayOut++;

		//*pHeightProfileArray
		memcpy(pHeightOut, aProfileData + nHeaderSize, nHeightDataSize);
		pHeightOut += wXPointNum;

		if(isBrightness == true){
			//*pLuminanceProfileArray
			memcpy(pLuminanceOut, aProfileData + nHeaderSize + nHeightDataSize, nLuminanceDataSize);
			pLuminanceOut += wXPointNum;
		}

		pProfileRawData += nHeaderSize + nProfileDataSize + nFooterSize;
		delete[] aProfileData;
		aProfileData = NULL;
	}

	return LJS8IF_RC_OK;
}

/**
  @brief	Checking received data. Used in high speed receiving thread.
  @param	pbyRecvData			:Received data
  @param	dwRecvSize			:Received data size
  @param	dwProfSize			:data size of each profile
  @param	pbyConvData			:Converted data
  @param	pdwConvSize			:Converted data size
  @param	pdwNotify			:"notify" content

  @return	used size
*/
static unsigned int ReceiveDataCheck(unsigned char* pbyRecvData, unsigned int dwRecvSize, unsigned int dwProfSize, unsigned char* pbyConvData, unsigned int* pdwConvSize, unsigned int* pdwNotify)
{
	ST_CIND_HEIGHT_IMAGE_NOTIFY* pstNotify = reinterpret_cast<ST_CIND_HEIGHT_IMAGE_NOTIFY*>(pbyRecvData);
	unsigned int dwUsedSize = 0;
	unsigned int dwNotify = 0;
	unsigned int dwConvSize = 0;

	if ((pbyRecvData == nullptr) || (pbyConvData == nullptr) || (pdwConvSize == nullptr) || (pdwNotify == nullptr)) {
		return dwUsedSize;
	}

	// If "Notify" packet is received, set its content.
	if ((sizeof(ST_CIND_HEIGHT_IMAGE_NOTIFY) <= dwRecvSize) && (DEF_NOTIFY_MAGIC_NO == pstNotify->dwMagicNo)) {
		dwNotify = pstNotify->dwNotify;
		dwUsedSize = sizeof(ST_CIND_HEIGHT_IMAGE_NOTIFY);
	}
	else if (dwProfSize <= dwRecvSize) {
		// Receive profile data.
		if (DEF_PROFILE_DATA_MAGIC_NO == pstNotify->dwMagicNo) {
			memcpy(pbyConvData, pbyRecvData, dwProfSize);
			dwConvSize = dwProfSize;
			dwUsedSize = dwProfSize;
		}
		else {
			// Irregular case.
			printf("***Highspeed Batch Receive thread*** : ERROR Invalid received data.\n");
		}
	}

	// Update "Notify" content
	*pdwNotify = dwNotify;
	
	*pdwConvSize = dwConvSize;
	
	return dwUsedSize;
}

/**
Setting initial parameters for highspeed receiving thread
  @param	nDeviceId		: device identifier
  @param	pCallBack		: receive completion callback function
  @param	dwThreadId		: thread identifier
  @return	nothing
*/
static void SetThreadInitParamFast(int nDeviceId, void (*pCallBack)(unsigned char*, unsigned int, unsigned int, unsigned int, unsigned int), unsigned int dwThreadId)
{
	m_ThreadParamFast[nDeviceId].pFunc	= pCallBack;
	m_ThreadParamFast[nDeviceId].dwThreadId = dwThreadId;
	m_ThreadParamFast[nDeviceId].dwDeviceId = nDeviceId;
}

/**
Initialize parameters for highspeed receiving thread
  @param	nDeviceId		: device identifier
  @return	nothing
*/
static void InitThreadParamFast(int nDeviceId)
{
	m_ThreadParamFast[nDeviceId].ProfileBuffer = NULL;	//Allocate in SetThreadParamFat funtion.

	m_ThreadParamFast[nDeviceId].pbyCommunicationBuffer = NULL;
	m_ThreadParamFast[nDeviceId].pbyConvertingBuffer = NULL;
	m_ThreadParamFast[nDeviceId].dwConvertingBufferIndex = 0;
	m_ThreadParamFast[nDeviceId].pbyConvertedBuffer = NULL;
	m_ThreadParamFast[nDeviceId].bIsEnabled = 0;

	m_ThreadParamFast[nDeviceId].pFunc = NULL;

	m_ThreadParamFast[nDeviceId].dwProfileMax = 0;
	m_ThreadParamFast[nDeviceId].dwProfileLoopCnt = 0;
	m_ThreadParamFast[nDeviceId].dwProfileCnt = 0;
	m_ThreadParamFast[nDeviceId].dwProfileSize = 0;
	m_ThreadParamFast[nDeviceId].byKind = 0;

	m_ThreadParamFast[nDeviceId].dwThreadId = 0;
	m_ThreadParamFast[nDeviceId].dwDeviceId = 0;

	m_ThreadParamFast[nDeviceId].nBufferSize = 0;
	m_ThreadParamFast[nDeviceId].stopFlag = 1;
}

/**
Setting parameters for highspeed receiving thread
  @param	nDeviceId		: device identifier
  @param	dwProfileCnt	: number of data points for each profile
  @param	dwLineCnt		: number of lines to wake receive completion callback function
  @param	byKind			: kind of profile (e.g. luminance output is enabled)
  @return
*/
static int SetThreadParamFast(int nDeviceId, unsigned int dwProfileCnt, unsigned int dwLineCnt, unsigned char byKind)
{
	// Check if high speed receiving thread is enabled.
	if (!m_ThreadParamFast[nDeviceId].bIsEnabled) {
		printf("High speed receiving thread is not enabled.\n");
		return LJS8IF_RC_ERR_NOT_OPEN;
	}

	// Store the kind of profile.
	m_ThreadParamFast[nDeviceId].byKind = byKind;

	// Store the data points for each profile.
	m_ThreadParamFast[nDeviceId].dwProfileCnt = dwProfileCnt;

	// Store the maximum profile lines to receive.
	m_ThreadParamFast[nDeviceId].dwProfileMax = dwLineCnt;

	// Store the size of each profile
	// Calculate the size of profile depending on luminance output is enabled.
	unsigned int dwSize = 0;
	bool isBrightness = (byKind & BRIGHTNESS_VALUE) == BRIGHTNESS_VALUE;
	
	if (isBrightness) {
		dwSize = (dwProfileCnt) * (sizeof(unsigned short) + sizeof(unsigned char)) + PROF_HEADER_LEN + PROF_CHECKSUM_LEN;
	}
	else {
		dwSize = (dwProfileCnt) * sizeof(unsigned short) + PROF_HEADER_LEN + PROF_CHECKSUM_LEN;
	}
	
	m_ThreadParamFast[nDeviceId].dwProfileSize = dwSize;

	// Create buffer for receive completion callback function.
	if (m_ThreadParamFast[nDeviceId].ProfileBuffer != NULL) {
		free(m_ThreadParamFast[nDeviceId].ProfileBuffer);
		m_ThreadParamFast[nDeviceId].ProfileBuffer = NULL;
	}
	m_ThreadParamFast[nDeviceId].nBufferSize = dwSize * dwLineCnt;
	m_ThreadParamFast[nDeviceId].ProfileBuffer = (unsigned char*)malloc(dwSize * dwLineCnt);

	if (m_ThreadParamFast[nDeviceId].ProfileBuffer == NULL) {
		return LJS8IF_RC_ERR_NOMEMORY;
	}

	return LJS8IF_RC_OK;
}

static void CallbackFuncWrapper(unsigned char* Buf, unsigned int dwSize, unsigned int dwCount, unsigned int dwNotify, unsigned int  dwUser)
{
	unsigned int nDeviceId = dwUser;

	auto header = new LJS8IF_PROFILE_HEADER[dwCount];
	auto height = new unsigned short[m_ThreadParamFast[nDeviceId].dwProfileCnt * dwCount];
	auto luminance = new unsigned char[m_ThreadParamFast[nDeviceId].dwProfileCnt * dwCount];

	int isBrightness = (m_ThreadParamFast[nDeviceId].byKind & BRIGHTNESS_VALUE) == BRIGHTNESS_VALUE;

	auto bufferSize = dwSize * dwCount;

	ConvertProfileData(
		Buf, 
		m_ThreadParamFast[nDeviceId].dwProfileCnt,
		isBrightness,
		dwCount,
		header,
		height,
		luminance,
		bufferSize);

	m_ThreadParamFast[nDeviceId].pFuncSimpleArray(header, height, luminance, (unsigned int)isBrightness, m_ThreadParamFast[nDeviceId].dwProfileCnt, dwCount, dwNotify, dwUser);

	delete[] header;
	delete[] height;
	delete[] luminance;

	header = NULL;
	height = NULL;
	luminance = NULL;
}