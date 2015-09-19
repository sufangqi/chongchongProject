/******************************************************************************
 *     Copyright (c) 2015 PuTao Co., Ltd.
 *     All Rights Reserved.
 ******************************************************************************/
#ifndef ___BASEPT_H__
#define ___BASEPT_H__

#include <stdio.h>
#include <stdlib.h>
#include <cstring>
#include <iostream>

//#include "opencv2/core.hpp"
//#include "opencv2/imgproc.hpp"
//#include "opencv2/imgproc/imgproc_c.h"
//#include "opencv2/calib3d.hpp"

#include<opencv2\opencv.hpp>
//#include <opencv2/objdetect/objdetect.hpp>
//#include <opencv2/video/video.hpp>
#ifdef _SHOW_
#include <opencv2/photo.hpp> //for hdr
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs/imgcodecs_c.h>
#endif


#include <vector>
#include <list>
#include <ctype.h>
#include <limits>

#include "basept.h"
#include "_time.h"

using namespace std;

#define CV_VERSION_ID       CVAUX_STR(CV_MAJOR_VERSION) "." CVAUX_STR(CV_MINOR_VERSION) "." CVAUX_STR(CV_SUBMINOR_VERSION)

typedef enum _COLOR {
  RED      = 0,
  BLACK    = 1,
  YELLOW   = 2,
  GREEN    = 3,
  CYAN     = 4,
  BLUE     = 5,
  PURPLE   = 6,
  COLORNUM = 7,
} COLOR;


typedef enum _SystemEnum {
    PT_APPLE_IPAD2   = 0,
    PT_APPLE_MINI1   = 1,
    PT_APPLE_AIR     = 2,
    PT_APPLE_AIR2    = 3,
    PT_APPLE_MINI3   = 4,

    PT_SYS_NUM       = 5,
} PTSysEnum;

#define iOSMinSDK 6
typedef enum _SDKEnum {
    PT_APPLE_IOS6   = 0,
    PT_APPLE_IOS7   = 1,
    PT_APPLE_IOS8   = 2,
    
    PT_SDK_NUM      = 3,
} PTSDKEnum;


extern const char* PtSysStr[PT_SYS_NUM];

extern const cv::Point2f dst_points_small[PT_SYS_NUM][PT_SDK_NUM][4];
extern const cv::Point2f src_points_small[PT_SYS_NUM][PT_SDK_NUM][4];
extern const double cameraArray[PT_SYS_NUM][PT_SDK_NUM][3*3];
extern const double distCoeffsArray[PT_SYS_NUM][PT_SDK_NUM][5];


typedef struct _PAIR {
    int min;
    int max;
    _PAIR(){min=0; max=0;}
    _PAIR(int _min,int _max) {min = _min; max = _max;}
} PAIR;

typedef struct _STATIConfigData {
  PTS32               nLowerBound;
  PTS32               nUpperBound;
//  bool                bCalcPixelSum;
} STATIConfigData;

typedef struct _STATIResultData {
  PTS32               nLowerBoundNum;//number of pixels which value is lower than nLowerBound
  PTS32               nUpperBoundNum;//number of pixels which value is larger than nUpperBound
  PTU32               nPixelSum;//sum of all (gray) pixles
} STATIResultData;

PTS32 _calcStatistics(const cv::Mat& srcImg, const PTImageFormatEnum eFormat, const STATIConfigData& configData, STATIResultData &stSTATIResultData/*passed by reference*/);

PTS32 _convert(const PTU8* pSrcBuffer, PTU8* pDstBuffer, const int nWidth, const int nHeight, const PTImageFormatEnum eSrcFormat, const PTImageFormatEnum eDstFormat);

PTF64 _distance(const cv::Point& pointA, const cv::Point& pointB);

PTF64 _calcAngle(const cv::Point& pre, const cv::Point& cur, const cv::Point& next);

PTS32 _cvtColor2BGR(PTU8* pPixels, PTS32 nWidth, PTS32 nHeight, PTImageFormatEnum eFormat, cv::Mat& bgrMat);

PTS32 _rotateImg(const cv::Mat srcMat, PTBOOL needRotate, cv::Mat& rotatedMat);

PTS32 _getBinAccordHSV(cv::Mat& bin, const cv::Mat& hsv, COLOR color, const PAIR& hueRange, const PAIR& satRange, const PAIR& valRange);

#endif /* ___BASEPT_H__ */

/* EOF */

