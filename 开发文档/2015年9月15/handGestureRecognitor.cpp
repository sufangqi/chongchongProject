#include"handGestureRecognitor.h"
#include"_utils.h"
#define _VIDEODEBUG_
HandGestureRecognitor::HandGestureRecognitor()
{
   mIsUpTapPointInited   = FALSE;//the up knock base is got
   mIsDownTapPointInited = FALSE;//the down knock
   mIsTapPointInited     = FALSE;
   mStdUpPointArea   = -1.0f;
   mStdDownPointArea = -1.0f;
}

HandGestureRecognitor::~HandGestureRecognitor()
{
//do nothing
}

PTS32 HandGestureRecognitor::init(PTU8* pPixels, PTS32 nWidth, PTS32 nHeight, PTImageFormatEnum eFormat)
{
#ifdef _VIDEODEBUG_
    if(pPixels==NULL || nWidth!=1280 || nHeight!=720) {
       PTDEBUG("Invalid parameters: pPixels[%p], nWidth[%d], nHeight[%d], eFormat[%d]\n", pPixels, nWidth, nHeight, eFormat/*, strFormat[eFormat]*/);
       return PT_RET_INVALIDPARAM;
    }
#else
    if(pPixels==NULL || nWidth!=1280 || nHeight!=960) {
       PTDEBUG("Invalid parameters: pPixels[%p], nWidth[%d], nHeight[%d], eFormat[%d]\n", pPixels, nWidth, nHeight, eFormat/*, strFormat[eFormat]*/);
       return PT_RET_INVALIDPARAM;
    }
#endif

    Mat srcImg;
    _cvtColor2BGR(pPixels, nWidth, nHeight, eFormat, srcImg);

#ifdef _VIDEODEBUG_
//    imshow("srcImg", srcImg);
//    waitKey(0);
#endif

#if 0
    //integrated in iPad
    Mat subImg = srcImg(Rect(380, 0, 580, 960)).clone();
#else
    //video as input
    Mat subImg = srcImg(Rect(380, 0, 640, 720)).clone();
#endif

    resize(subImg, subImg, Size(subImg.cols/4, subImg.rows/4));
    //获取敲击点的位置和坐标
    mUpHandImg   = subImg(Rect(0, 0, subImg.cols, subImg.rows/2));
    mDownHandImg = subImg(Rect(0, subImg.rows/2, subImg.cols, subImg.rows/2));

#ifdef _VIDEODEBUG_
/*    imshow("subImg", subImg);
    imshow("mUpHandImg", mUpHandImg);
    imshow("mDownHandImg", mDownHandImg);
    waitKey();*/
#endif

    return PT_RET_OK;
}

//check wether the knock point is put corect
PTS32  HandGestureRecognitor::getKnockPointStatus(PTBOOL& isTapPointCorrect)
{
    Mat knockMask(mUpHandImg.size(), CV_8UC1);

    /*useless*/
    double konckPointArea = -1.0f;

    Point centerUp = Point(-1, -1);
    _getKnockMask(mUpHandImg, knockMask, ROIDOWN);
    _getContoursAreaCenter(knockMask, konckPointArea, centerUp);

#ifdef _SHOW_
    Mat draw(knockMask.size(), CV_8UC3);
    cvtColor(knockMask, draw, CV_GRAY2BGR);
    circle(draw, centerUp, 4, CV_RGB(0,0,255));
    imshow("knockMask[Up]", draw);
#endif

    PTDEBUG("konckPointArea[%f], centerUp[%d,%d]\n", konckPointArea, centerUp.x, centerUp.y);

    Point centerDown = Point(-1,-1);
    _getKnockMask(mDownHandImg, knockMask, ROIUP);
    _getContoursAreaCenter(knockMask, konckPointArea, centerDown);

#ifdef _SHOW_
    cvtColor(knockMask, draw, CV_GRAY2BGR);
    circle(draw, centerDown, 4, CV_RGB(0,255,0));
    imshow("knockMask[Down]", draw);
#endif

    centerDown.y = centerDown.y + mUpHandImg.rows;
    PTDEBUG("konckPointArea[%f], centerDown[%d,%d]\n", konckPointArea, centerDown.x, centerDown.y);

    const double knockPointAngle = abs(_calcLineAngle(centerUp, centerDown));
    const double distance_up     = centerUp.y;
    const double distance_down   = abs((mUpHandImg.rows+mDownHandImg.rows) - centerDown.y);
    const double distance_right  = MIN(abs(mUpHandImg.cols-centerUp.x), abs(mUpHandImg.cols-centerDown.x));

    /*below parameters should be tuned*/
    PTDEBUG("knockPointAngle[%f], distance_down[%f], distance_right[%f], distance_up[%f]\n", knockPointAngle, distance_down, distance_right, distance_up);
    if(knockPointAngle>15.0f || distance_down<mDownHandImg.rows/5|| distance_right<mDownHandImg.cols/10 || distance_up<mDownHandImg.rows/5) {
       PTDEBUG("Tap points are not placed correct!\n");
       isTapPointCorrect = FALSE;
    } else {
       PTDEBUG("Tap points are placed correct!\n");
       isTapPointCorrect = TRUE;
    }

    //waitKey(0);
    return PT_RET_OK;
}

PTS32 HandGestureRecognitor::studyTwoKnockBase(void)
{
    PTDEBUG("Enter %s\n", __FUNCTION__);

    if(!mIsUpTapPointInited) {
       _getKnockBase(mUpHandImg, ROIUP, mStdUpPointArea, mIsUpTapPointInited);
       PTDEBUG("mStdUpPointArea[%f], mIsUpTapPointInited[%s]\n", mStdUpPointArea, mIsUpTapPointInited?"true":"false");
    } else {
       PTDEBUG("mIsUpTapPointInited[%s], mStdUpPointArea[%f]\n", mIsUpTapPointInited?"true":"false", mStdUpPointArea);
    }

    if(!mIsDownTapPointInited) {
       _getKnockBase(mDownHandImg, ROIDOWN, mStdDownPointArea, mIsDownTapPointInited);
       PTDEBUG("mStdDownPointArea[%f], mIsDownTapPointInited[%s]\n", mStdDownPointArea, mIsDownTapPointInited?"true":"false");
    } else {
       PTDEBUG("mIsDownTapPointInited[%s], mStdDownPointArea[%f]\n", mIsDownTapPointInited?"true":"false", mStdDownPointArea);
    }

    if(mIsUpTapPointInited && mIsDownTapPointInited) {
       PTDEBUG("both mIsUpTapPointInited and mIsDownTapPointInited are true, set mIsTapPointInited to TRUE\n");
       PTDEBUG("mStdUpPointArea[%f], mStdDownPointArea[%f]\n", mStdUpPointArea, mStdDownPointArea);
       mIsTapPointInited = TRUE;
    } else {
       PTDEBUG("one of mIsUpTapPointInited[%s] and mIsDownTapPointInited[%s] is not true, set mIsTapPointInited to FALSE\n",
                mIsUpTapPointInited?"true":"false", mIsDownTapPointInited?"true":"false");
       mIsTapPointInited = FALSE;
    }

    PTDEBUG("Exit %s\n", __FUNCTION__);
    return PT_RET_OK;
}

PTS32 HandGestureRecognitor::getUpHandGesture(PTHandStatus& handStatus)
{
    _getHandRecognitizeGesture(mUpHandImg, mStdUpPointArea, ROIUP, handStatus);
    return PT_RET_OK;
}

PTS32 HandGestureRecognitor::getDownHandGesture(PTHandStatus& handStatus)
{
    //flipping around the x-axis
    flip(mDownHandImg, mDownHandImg, 0);
    _getHandRecognitizeGesture(mDownHandImg, mStdDownPointArea, ROIDOWN, handStatus);
    return PT_RET_OK;
}

