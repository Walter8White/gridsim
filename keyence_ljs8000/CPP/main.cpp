//Copyright (c) 2024 KEYENCE CORPORATION. All rights reserved.
/** @file
@brief	Example for the usage of the LJ-S image acquisition library (LJS_ACQ)
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>
#include <time.h>

#include "LJS8_IF_Linux.h"
#include "LJS8_ErrorCode.h"
#include "LJS_ACQ.h"

#ifdef OPENCV_EN
#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/highgui.hpp>
#endif

#ifdef PNG_EN
#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/imgcodecs.hpp>
#endif

#ifdef VIZ_EN
#include <opencv4/opencv2/viz/viz3d.hpp>
#endif

typedef struct {
	int invalidCount;
	unsigned short minHeight;
	unsigned short maxHeight;
} HEIGHT_STATS;

static void print_usage(const char* programName)
{
	printf("usage: %s [--save-raw] [--save-invalid-image] [--save-all]\n", programName);
	printf("  --save-raw            Save height/luminance raw files and metadata CSV.\n");
	printf("  --save-invalid-image  Save PNG image with invalid pixels in red.\n");
	printf("  --save-all            Save raw files, metadata, and invalid-pixel image.\n");
}

static HEIGHT_STATS compute_height_stats(unsigned short* heightImage, const LJS_ACQ_GETPARAM* getParam)
{
	HEIGHT_STATS stats;
	stats.invalidCount = 0;
	stats.minHeight = 65535;
	stats.maxHeight = 0;

	int pointCount = getParam->x_pointnum * getParam->y_pointnum;
	for (int i = 0; i < pointCount; ++i) {
		unsigned short value = heightImage[i];
		if (value == 0)
			++stats.invalidCount;
		if (value < stats.minHeight)
			stats.minHeight = value;
		if (value > stats.maxHeight)
			stats.maxHeight = value;
	}

	return stats;
}

static int prepare_capture_base_path(char* basePath, size_t basePathSize)
{
	const char* outDir = "captures";
	mkdir(outDir, 0775);

	time_t now = time(NULL);
	struct tm tmNow;
	localtime_r(&now, &tmNow);

	if (strftime(basePath, basePathSize, "captures/ljs_%Y%m%d_%H%M%S", &tmNow) == 0) {
		printf("Failed to prepare capture path.\n");
		return 1;
	}

	return 0;
}

static int save_invalid_image(unsigned short* heightImage, const LJS_ACQ_GETPARAM* getParam, const HEIGHT_STATS* stats, const char* basePath)
{
#ifndef PNG_EN
	printf("PNG support was not enabled at build time.\n");
	printf("Rebuild with OpenCV imgcodecs support to use --save-invalid-image.\n");
	return 1;
#else
	int pointCount = getParam->x_pointnum * getParam->y_pointnum;
	unsigned short minValid = 65535;
	unsigned short maxValid = 0;
	for (int i = 0; i < pointCount; ++i) {
		unsigned short value = heightImage[i];
		if (value != 0) {
			if (value < minValid)
				minValid = value;
			if (value > maxValid)
				maxValid = value;
		}
	}
	if (maxValid <= minValid)
		maxValid = minValid + 1;

	cv::Mat image(getParam->y_pointnum, getParam->x_pointnum, CV_8UC3);
	for (int y = 0; y < getParam->y_pointnum; ++y) {
		for (int x = 0; x < getParam->x_pointnum; ++x) {
			int i = y * getParam->x_pointnum + x;
			cv::Vec3b& pixel = image.at<cv::Vec3b>(y, x);
			unsigned short value = heightImage[i];
			if (value == 0) {
				pixel = cv::Vec3b(0, 0, 255);
			} else {
				int gray = (int)((value - minValid) * 255.0 / (maxValid - minValid));
				if (gray < 0)
					gray = 0;
				if (gray > 255)
					gray = 255;
				pixel = cv::Vec3b((unsigned char)gray, (unsigned char)gray, (unsigned char)gray);
			}
		}
	}

	char imagePath[300];
	snprintf(imagePath, sizeof(imagePath), "%s_invalid_red.png", basePath);
	if (!cv::imwrite(imagePath, image)) {
		printf("Failed to write %s.\n", imagePath);
		return 1;
	}

	printf(" Saved invalid image : %s\n", imagePath);
	printf(" Invalid pixels shown in red: %d/%d (%.4f%%)\n",
		stats->invalidCount, pointCount, 100.0 * stats->invalidCount / pointCount);

	return 0;
#endif
}

static int save_raw_acquisition(unsigned short* heightImage, unsigned char* luminanceImage, const LJS_ACQ_GETPARAM* getParam, const HEIGHT_STATS* stats, const char* basePath)
{
	int pointCount = getParam->x_pointnum * getParam->y_pointnum;
	char heightPath[300];
	snprintf(heightPath, sizeof(heightPath), "%s_height_u16le.raw", basePath);
	FILE* fp = fopen(heightPath, "wb");
	if (fp == NULL) {
		printf("Failed to open %s for writing.\n", heightPath);
		return 1;
	}
	fwrite(heightImage, sizeof(unsigned short), pointCount, fp);
	fclose(fp);

	char luminancePath[300] = "";
	if (getParam->luminance_enabled) {
		snprintf(luminancePath, sizeof(luminancePath), "%s_luminance_u8.raw", basePath);
		fp = fopen(luminancePath, "wb");
		if (fp == NULL) {
			printf("Failed to open %s for writing.\n", luminancePath);
			return 1;
		}
		fwrite(luminanceImage, sizeof(unsigned char), pointCount, fp);
		fclose(fp);
	}

	char metaPath[300];
	snprintf(metaPath, sizeof(metaPath), "%s_meta.csv", basePath);
	fp = fopen(metaPath, "w");
	if (fp == NULL) {
		printf("Failed to open %s for writing.\n", metaPath);
		return 1;
	}
	fprintf(fp, "key,value\n");
	fprintf(fp, "height_file,%s\n", heightPath);
	if (getParam->luminance_enabled)
		fprintf(fp, "luminance_file,%s\n", luminancePath);
	fprintf(fp, "x_pointnum,%d\n", getParam->x_pointnum);
	fprintf(fp, "y_pointnum,%d\n", getParam->y_pointnum);
	fprintf(fp, "luminance_enabled,%d\n", getParam->luminance_enabled);
	fprintf(fp, "x_pitch_um,%f\n", getParam->x_pitch_um);
	fprintf(fp, "y_pitch_um,%f\n", getParam->y_pitch_um);
	fprintf(fp, "z_pitch_um,%f\n", getParam->z_pitch_um);
	fprintf(fp, "height_formula_mm,(stored_value - 32768) * z_pitch_um / 1000\n");
	fprintf(fp, "invalid_height_value,0\n");
	fprintf(fp, "height_min_raw,%u\n", stats->minHeight);
	fprintf(fp, "height_max_raw,%u\n", stats->maxHeight);
	fprintf(fp, "invalid_pixel_count,%d\n", stats->invalidCount);
	fprintf(fp, "invalid_pixel_percent,%f\n", 100.0 * stats->invalidCount / pointCount);
	fclose(fp);

	printf(" Saved height    : %s\n", heightPath);
	if (getParam->luminance_enabled)
		printf(" Saved luminance : %s\n", luminancePath);
	printf(" Saved metadata  : %s\n", metaPath);

	return 0;
}

int main(int argc, char *argv[]){
//=====================================================================
// Acquire LJS image
//=====================================================================
// Basically you can acquire LJ-S image in just 3 steps.
//
//  Step1. Open device
//  Step2. Acquire images
//  Step3. Close device

	unsigned short *heightImage = NULL;		// Height image
	unsigned char *luminanceImage = NULL;	// Luminance image
	int saveRaw = 0;
	int saveInvalidImage = 0;

	for (int i = 1; i < argc; ++i) {
		if (strcmp(argv[i], "--save-raw") == 0) {
			saveRaw = 1;
		} else if (strcmp(argv[i], "--save-invalid-image") == 0) {
			saveInvalidImage = 1;
		} else if (strcmp(argv[i], "--save-all") == 0) {
			saveRaw = 1;
			saveInvalidImage = 1;
		} else if (strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-h") == 0) {
			print_usage(argv[0]);
			return 0;
		} else {
			printf("Unknown argument: %s\n", argv[i]);
			print_usage(argv[0]);
			return 1;
		}
	}
	
//-----------------------------------------------------------------
// CHANGE THIS BLOCK TO MATCH YOUR SENSOR SETTINGS (FROM HERE)
//-----------------------------------------------------------------
	int deviceId 			=	0;		// Set "0" if you use only 1 head.
	int	interpolateLines	=	1;		// Scale factor of Y lines(up to 8).
	int	timeout_ms			= 	5000;	// Timeout value for the acquiring image (in milisecond).
	int use_external_trigger = 0;		// Set "1" if you control the trigger timing externally. (e.g. terminal input)
	
	
	LJS8IF_ETHERNET_CONFIG EthernetConfig =
	{
		{ 192, 168, 0, 1},				// IP address
		24691							// Port number
	};
	int HighSpeedPortNo = 24692;		// Port number for high-speed communication
	
//-----------------------------------------------------------------
// CHANGE THIS BLOCK TO MATCH YOUR SENSOR SETTINGS (TO HERE)
//-----------------------------------------------------------------

	// Prepare setting parameter
	LJS_ACQ_SETPARAM setParam;
	{
		setParam.interpolateLines 	= interpolateLines;
		setParam.timeout_ms 		= timeout_ms;
		setParam.use_external_trigger = use_external_trigger;
	}
	
	// Variable to store information of the acquired image
	LJS_ACQ_GETPARAM getParam;

	// Check User setting parameters
	if(interpolateLines < 1 || interpolateLines > 8){
		printf("Invalid interpolateLines value. Please set a value between 1 and 8.\n");
		return (1);
	}

	//------------------------------------------------------------
	// Step1. Open device
	//------------------------------------------------------------
	int errCode = LJS_ACQ_OpenDevice( deviceId, &EthernetConfig, HighSpeedPortNo );
	
	if(errCode != LJS8IF_RC_OK){
		printf("Failed to open device.\n");
		return (1);
	}
	
	//------------------------------------------------------------
	// Step2. Acquire image
	//------------------------------------------------------------
	// There are two methods you can use.
	//
	// (1) Synchronous method
	//	"Acquire" function does not return unless the acquisition is completed or a timeout occurs. 
	//	This is an easy method because you only call one function.
	//  But it blocks execution of other code during acquisition.
	//
	// (2) Asynchronous methods
	//  "Start" the acquisition first, and "Acquire" later.
	//  "Acquire" function returns even if the acquisition is not completed.
	//  It doesn't block other code.

	
#if 0	// Synchronous acquisition
	errCode = LJS_ACQ_Acquire( deviceId, &heightImage, &luminanceImage, &setParam, &getParam);
	
	if (errCode != LJS8IF_RC_OK) {
		printf("Failed to acquire image.\n");
		//Free user memory
		if (heightImage != NULL) {
			free(heightImage);
			heightImage = NULL;
		}
		if (luminanceImage != NULL) {
			free(luminanceImage);
			luminanceImage = NULL;
		}

		return (1);
	}
#else	// Asynchronous acquisition
	// Start asynchronous acquire
	errCode = LJS_ACQ_StartAsync(deviceId, &setParam);

	if (errCode != LJS8IF_RC_OK) {
		printf("Failed to acquire image.\n");
		return (1);
	}

	// Acquire images. Polling to confirm completion.
	printf(" [acquring image...(waiting in usercode)]\n");
	for (int i = 0; i < timeout_ms; ++i) {
		usleep(1000);
		errCode = LJS_ACQ_AcquireAsync(deviceId, &heightImage, &luminanceImage, &setParam, &getParam);
		if (errCode == LJS8IF_RC_OK)
			break;
	}

	if (errCode != LJS8IF_RC_OK) {
		printf("Failed to acquire image (timeout).\n");
		//Free user memory
		if (heightImage != NULL) {
			free(heightImage);
			heightImage = NULL;
		}
		if (luminanceImage != NULL) {
			free(luminanceImage);
			luminanceImage = NULL;
		}

		return (1);
	}

#endif

	//------------------------------------------------------------
	// Step3. Close device
	//------------------------------------------------------------
	LJS_ACQ_CloseDevice(deviceId);
	
	// Information of the acquired image
	printf("----------------------------------------\n");
	printf(" Luminance output       : %d\n", getParam.luminance_enabled);
	printf(" Number of X points     : %d\n", getParam.x_pointnum);
	printf(" Number of Y points     : %d\n", getParam.y_pointnum);
	printf(" X pitch in micrometer  : %f\n", getParam.x_pitch_um);
	printf(" Y pitch in micrometer  : %f\n", getParam.y_pitch_um);
	printf(" Z pitch in micrometer  : %f\n", getParam.z_pitch_um);
	printf(" Head Processing Timeout: %d\n", getParam.isProcTimeoutOccurred);
	printf("----------------------------------------\n");

	HEIGHT_STATS stats = compute_height_stats(heightImage, &getParam);
	int pointCount = getParam.x_pointnum * getParam.y_pointnum;
	printf(" Height raw stats       : min=%u max=%u invalid=%d/%d (%.4f%%)\n",
		stats.minHeight, stats.maxHeight, stats.invalidCount, pointCount, 100.0 * stats.invalidCount / pointCount);
	if (stats.invalidCount == pointCount)
		printf(" WARNING: all height pixels are invalid (raw value 0).\n");

	if (saveRaw || saveInvalidImage) {
		char basePath[256];
		if (prepare_capture_base_path(basePath, sizeof(basePath)) != 0)
			return 1;
		if (saveRaw && save_raw_acquisition(heightImage, luminanceImage, &getParam, &stats, basePath) != 0)
			return 1;
		if (saveInvalidImage && save_invalid_image(heightImage, &getParam, &stats, basePath) != 0)
			return 1;
	} else {
		printf(" No files saved. Use --save-raw, --save-invalid-image, or --save-all to write captures.\n");
	}
	
//=====================================================================
// 2D and 3D display using OpenCV and VIZ module
//=====================================================================
#ifdef OPENCV_EN
	int x_pointnum = getParam.x_pointnum;
	int y_linenum = getParam.y_pointnum;

	// Convert LJS image data to OpenCV image format
	cv::Mat imgHeight 	= 	cv::Mat::zeros(y_linenum, x_pointnum, CV_16UC1);
	cv::Mat imgLumi 	= 	cv::Mat::zeros(y_linenum, x_pointnum, CV_8UC1);
	for(int y=0; y < y_linenum; ++y){
		for(int x=0; x < x_pointnum; ++x){
			
			imgHeight.at<ushort>(y, x)	= heightImage[y * x_pointnum + x];
			
			if( getParam.luminance_enabled )
				imgLumi.at<uchar>(y, x) 	= luminanceImage[y * x_pointnum + x];
		}
	}
	cv::namedWindow("Height", cv::WINDOW_FREERATIO);
	cv::resizeWindow("Height", 400, 400);
	cv::imshow("Height", imgHeight);
	
	if( getParam.luminance_enabled ){
		cv::namedWindow("Luminance", cv::WINDOW_FREERATIO);
		cv::resizeWindow("Luminance", 400, 400);
		cv::imshow("Luminance", imgLumi);
	}

	cv::waitKey(2000);
#ifdef VIZ_EN

	// Reduce the point density.
	// Point clouds may be too large to be processed by VIZ.
	const int reducePoints = 4;

	// Convert LJS image data to OpenCV point cloud format
	cv::Mat pCloud(y_linenum/reducePoints, x_pointnum/reducePoints, CV_32FC3);
	
	cv::Mat imgLumiReduced = cv::Mat::zeros(y_linenum/reducePoints, x_pointnum/reducePoints, CV_8UC1);

  	for(int ry=0, y=0; ry < y_linenum/reducePoints; ++ry, y+=reducePoints){
		for(int rx=0, x=0; rx < x_pointnum/reducePoints; ++rx, x+=reducePoints){
			float fX = x * getParam.x_pitch_um;
			float fY = y * getParam.y_pitch_um;
			
			if(heightImage[ y * x_pointnum + x] !=0){
				float fZ = (heightImage[ y * x_pointnum + x] -32768)* getParam.z_pitch_um;
				pCloud.at<cv::Vec3f>(ry,rx) = cv::Vec3f(fX,-fY,fZ);
			}
			else{
				pCloud.at<cv::Vec3f>(ry,rx) = cv::Vec3f(0.f, 0.f, 0.f);
			}
			
			if( getParam.luminance_enabled )
				imgLumiReduced.at<uchar>(ry, rx) = luminanceImage[y * x_pointnum + x];
			
		}
	}

	cv::viz::Viz3d myWindow("PointCloud");
	
	cv::viz::WCloud wcloudWithLumi(pCloud, imgLumiReduced);
	cv::viz::WCloud wcloud(pCloud);
	
	myWindow.showWidget("Coordinate", cv::viz::WCoordinateSystem());
	
	if( getParam.luminance_enabled )
		myWindow.showWidget("data", wcloudWithLumi);
	else
		myWindow.showWidget("data", wcloud);
	
	printf("\nPress 'q' key to exit the program...\n");	
	myWindow.spin();
	
#else

	printf("\nPress any key to exit the program...\n");
	cv::waitKey(0);	
#endif // VIZ_EN

	cv::destroyAllWindows();
#endif // OPENCV_EN

	
	//Free user memory
	free( heightImage );
	free( luminanceImage );
	heightImage = NULL;
	luminanceImage = NULL;

	return (0);
}
