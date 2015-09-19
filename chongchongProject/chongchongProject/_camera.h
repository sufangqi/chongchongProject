/******************************************************************************
 *     Copyright (c) 2015 PuTao Co., Ltd.
 *     All Rights Reserved.
 ******************************************************************************/
#ifndef ___CAMERA_H__
#define ___CAMERA_H__

#include "_basept.h"

#define OsmoHardware
//#define PutaoHardware



//TODO:XUXUXU need to check the air's roi
//left_up -> left_down  clockwise
const int vertNum = 4;
const cv::Point detectRoiVertex[PT_SYS_NUM][PT_SDK_NUM][vertNum] = {
#if defined PutaoHardware
    {cv::Point(0, 345)*RESIZERATIO, cv::Point(959, 345)*RESIZERATIO, cv::Point(959, 1100)*RESIZERATIO, cv::Point(0, 1100)*RESIZERATIO},//ipad mini 3
    {cv::Point(0, 184)*RESIZERATIO, cv::Point(959, 184)*RESIZERATIO, cv::Point(959, 1120)*RESIZERATIO, cv::Point(0, 1120)*RESIZERATIO},//ipad air 2
    #ifdef UseCaptureVedio
    {cv::Point(0, 200)*RESIZERATIO, cv::Point(716, 200)*RESIZERATIO, cv::Point(716, 1080)*RESIZERATIO, cv::Point(0, 1080)*RESIZERATIO}
    #endif
#elif defined OsmoHardware
    {//ipad 2
        {cv::Point(0, 0), cv::Point(0, 0), cv::Point(0, 0), cv::Point(0, 0)},
        {cv::Point(0, 44), cv::Point(239, 44), cv::Point(239, 280), cv::Point(0, 280)},
        {cv::Point(0, 44), cv::Point(239, 44), cv::Point(239, 280), cv::Point(0, 280)},
    },
    {//ipad mini 1
        {cv::Point(0, 44), cv::Point(239, 44), cv::Point(239, 290), cv::Point(0, 290)},
        {cv::Point(0, 0), cv::Point(0, 0), cv::Point(0, 0), cv::Point(0, 0)},
        {cv::Point(0, 44), cv::Point(239, 44), cv::Point(239, 280), cv::Point(0, 280)},
    },
    {//ipad air
        {cv::Point(0, 0), cv::Point(0, 0), cv::Point(0, 0), cv::Point(0, 0)},
        {cv::Point(0, 0), cv::Point(0, 0), cv::Point(0, 0), cv::Point(0, 0)},
        {cv::Point(0, 48), cv::Point(239, 48), cv::Point(239, 260), cv::Point(0, 260)},
    },
    {//ipad air 2
        {cv::Point(0, 0), cv::Point(0, 0), cv::Point(0, 0), cv::Point(0, 0)},
        {cv::Point(0, 0), cv::Point(0, 0), cv::Point(0, 0), cv::Point(0, 0)},
        {cv::Point(0, 46), cv::Point(239, 46), cv::Point(239, 280), cv::Point(0, 280)},
    },
    {//ipad mini 3
        {cv::Point(0, 0), cv::Point(0, 0), cv::Point(0, 0), cv::Point(0, 0)},
        {cv::Point(0, 0), cv::Point(0, 0), cv::Point(0, 0), cv::Point(0, 0)},
        {cv::Point(0, 50), cv::Point(239, 50), cv::Point(239, 270), cv::Point(0, 270)},
    },
    
    #ifdef UseCaptureVedio
    {cv::Point(0, 200)*RESIZERATIO, cv::Point(716, 200)*RESIZERATIO, cv::Point(716, 1080)*RESIZERATIO, cv::Point(0, 1080)*RESIZERATIO}
    #endif
#endif
};


typedef struct _CAMERAPARAM {
    PTS32               nWidth;
    PTS32               nHeight;
    PTImageFormatEnum   eFormat;
    PTBOOL              needRotate;
    
    //map params for remap
    cv::Mat mapx;
    cv::Mat mapy;
    //matrix for perspective transform
    cv::Mat persMatrix;
    //roiDetect for detect boards and roiCheck for check contours
    cv::Rect roiDetect;
    
#if defined PutaoHardware
    vector<cv::Point2f> roiPtsCheck;
#elif defined OsmoHardware
    cv::Rect roiCheck;
#endif
} CAMERAPARAM;

PTS32 _gammaCorr(cv::Mat& hsv_src_dst, double gammaCoeff);

PTS32 _unevenLumiCorr(cv::Mat& hsv_src_dst);

PTS32 _getAffineMatrix(PTSysEnum eSys, PTSDKEnum eSDK, cv::Size imgSize, cv::Mat& affineMapx, cv::Mat& affineMapy);

PTS32 _getPerspectiveMatrix(PTSysEnum eSys, PTSDKEnum eSDK, cv::Size imgSize, cv::Mat& perspectiveMatrix);

#if defined PutaoHardware
PTS32 _getCaptureROI(PTDeviceEnum eDevice, cv::Mat affineMapx, cv::Mat affineMapy, cv::Mat persMatrix, cv::Rect& roiDetect, vector<cv::Point2f>& roiPtsCheck);
#elif defined OsmoHardware
PTS32 _getCaptureROI(PTSysEnum eSys, PTSDKEnum eSDK, cv::Mat affineMapx, cv::Mat affineMapy, cv::Mat persMatrix, cv::Rect& roiDetect, cv::Rect& roiAfterPers);
#endif

PTS32 _warpPerspectiveContour(vector<cv::Point>& contour, const cv::Mat& persMatrix, const cv::Rect& roiDetect);

PTS32 _warpPerspectivePoint(cv::Point& point, const cv::Mat& persMatrix, const cv::Rect& roiDetect);

#endif
