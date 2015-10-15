#include "handGestureRecognitor.h"
#include "_utils.h"

//#define _SHOW_
#ifdef _SHOW_
//global mat for display dubug
#define  DISP_WINDOW   "dispImg"
Mat dispImg;
#endif
//unsigned int gHandID = 0;

#define VideoMode

//FIXME: SHIT global varible
Mat gUpHandImg;
Mat gDownHandImg;

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
#ifdef VideoMode
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

#ifdef _SHOW_
    dispImg.create(srcImg.rows, srcImg.cols, CV_8UC3);
    dispImg.setTo(Scalar::all(127));
    Mat topLeft = dispImg(Rect(0,0,dispImg.cols/2,dispImg.rows/2));
    resize(srcImg, topLeft, Size(srcImg.cols/2,srcImg.rows/2));
#endif

#ifdef VideoMode
    //1280x720 video as input
    Mat subImg = srcImg(Rect(380, 0, 640, 720)).clone();
#else
    //integrated in iPad
    //Mat subImg = srcImg(Rect(380, 0, 580, 960)).clone();
    Mat subImg = srcImg(Rect(230, 0, 650, 960)).clone();
#endif

    resize(subImg, subImg, Size(subImg.cols/4, subImg.rows/4));
    //获取敲击点的位置和坐标
    gUpHandImg   = subImg(Rect(0, 0, subImg.cols, subImg.rows/2));
    gDownHandImg = subImg(Rect(0, subImg.rows/2, subImg.cols, subImg.rows/2));

#ifdef _SHOW_
    Mat topRightUp = dispImg(Rect(dispImg.cols/2, 0, gUpHandImg.cols, gUpHandImg.rows));
    resize(gUpHandImg, topRightUp, gUpHandImg.size());

    Mat topRightDown = dispImg(Rect(dispImg.cols/2, gUpHandImg.rows, gDownHandImg.cols, gDownHandImg.rows));
    resize(gDownHandImg, topRightDown, gDownHandImg.size());
    imshow(DISP_WINDOW, dispImg);
    waitKey(0);
#endif

    return PT_RET_OK;
}

//check wether the knock point is put corect
PTS32  HandGestureRecognitor::getKnockPointStatus(PTBOOL& isTapPointCorrect)
{
    Mat knockMask(gUpHandImg.size(), CV_8UC1);

    /*useless*/
    double konckPointArea = -1.0f;

    Point centerUp = Point(-1, -1);
    _getKnockMask(gUpHandImg, knockMask, ROIUP);
    _getMaxContoursAreaCenter(knockMask, konckPointArea, centerUp);

#ifdef _SHOW_
    Mat draw(gUpHandImg.size(), CV_8UC3);
    cvtColor(knockMask, draw, CV_GRAY2BGR);
    circle(draw, centerUp, 4, CV_RGB(0,0,255));

    Mat topRightUp = dispImg(Rect(dispImg.cols/2+gUpHandImg.cols, 0, knockMask.cols, knockMask.rows));
    resize(draw, topRightUp, draw.size());
#endif

    PTDEBUG("konckPointArea[%f], centerUp[%d,%d]\n", konckPointArea, centerUp.x, centerUp.y);

    Point centerDown = Point(-1,-1);
    _getKnockMask(gDownHandImg, knockMask, ROIDOWN);
    _getMaxContoursAreaCenter(knockMask, konckPointArea, centerDown);

#ifdef _SHOW_
    cvtColor(knockMask, draw, CV_GRAY2BGR);
    circle(draw, centerDown, 4, CV_RGB(0,255,0));

    topRightUp = dispImg(Rect(dispImg.cols/2+gUpHandImg.cols, knockMask.rows, knockMask.cols, knockMask.rows));
    resize(draw, topRightUp, draw.size());

    imshow(DISP_WINDOW, dispImg);
    //waitKey(0);
#endif

    centerDown.y = centerDown.y + gUpHandImg.rows;
    PTDEBUG("konckPointArea[%f], centerDown[%d,%d]\n", konckPointArea, centerDown.x, centerDown.y);

    const double knockPointAngle = abs(_calcLineYAngle(centerUp, centerDown));
    const double distance_up     = centerUp.y;
    const double distance_down   = abs((gUpHandImg.rows+gDownHandImg.rows) - centerDown.y);
    const double distance_right  = MIN(abs(gUpHandImg.cols-centerUp.x), abs(gUpHandImg.cols-centerDown.x));

    /*below parameters should be tuned*/
    PTDEBUG("knockPointAngle[%f], distance_down[%f], distance_right[%f], distance_up[%f]\n", knockPointAngle, distance_down, distance_right, distance_up);
    if(knockPointAngle>15.0f || distance_up<gUpHandImg.rows/5.0f || distance_down<gDownHandImg.rows/5.0f || distance_right<gUpHandImg.cols/10.0f) {
       PTDEBUG("Tap points are not placed correct!\n");
       isTapPointCorrect = FALSE;
    } else {
       PTDEBUG("Tap points are placed correct!\n");
       isTapPointCorrect = TRUE;
    }

    return PT_RET_OK;
}

PTS32 HandGestureRecognitor::studyTwoKnockBase(void)
{
    PTDEBUG("Enter %s\n", __FUNCTION__);

    if(!this->mIsUpTapPointInited) {
       _getKnockBase(gUpHandImg, ROIUP, this->mStdUpPointArea, this->mIsUpTapPointInited);
       PTDEBUG("mStdUpPointArea[%f], mIsUpTapPointInited[%s]\n", this->mStdUpPointArea, this->mIsUpTapPointInited?"true":"false");
    } else {
       PTDEBUG("mIsUpTapPointInited[%s], mStdUpPointArea[%f]\n", this->mIsUpTapPointInited?"true":"false", this->mStdUpPointArea);
    }

    if(!this->mIsDownTapPointInited) {
       _getKnockBase(gDownHandImg, ROIDOWN, this->mStdDownPointArea, this->mIsDownTapPointInited);
       PTDEBUG("mStdDownPointArea[%f], mIsDownTapPointInited[%s]\n", this->mStdDownPointArea, this->mIsDownTapPointInited?"true":"false");
    } else {
       PTDEBUG("mIsDownTapPointInited[%s], mStdDownPointArea[%f]\n", this->mIsDownTapPointInited?"true":"false", this->mStdDownPointArea);
    }

    if(this->mIsUpTapPointInited && this->mIsDownTapPointInited) {
       PTDEBUG("both mIsUpTapPointInited and mIsDownTapPointInited are true, set mIsTapPointInited to TRUE\n");
       PTDEBUG("mStdUpPointArea[%f], mStdDownPointArea[%f]\n", this->mStdUpPointArea, this->mStdDownPointArea);
       this->mIsTapPointInited = TRUE;
    } else {
       PTDEBUG("one of mIsUpTapPointInited[%s] and mIsDownTapPointInited[%s] is not true, set mIsTapPointInited to FALSE\n",
                this->mIsUpTapPointInited?"true":"false", this->mIsDownTapPointInited?"true":"false");
       this->mIsTapPointInited = FALSE;
    }

    PTDEBUG("Exit %s\n", __FUNCTION__);
    return PT_RET_OK;
}

PTS32 HandGestureRecognitor::getUpHandGesture(PTHandStatus& handStatus, int& KnockNumber)
{
    _getHandRecognitizeGestureUp(gUpHandImg, this->mStdUpPointArea, ROIUP, handStatus, KnockNumber);
    return PT_RET_OK;
}

PTS32 HandGestureRecognitor::getDownHandGesture(PTHandStatus& handStatus, int& KnockNumber)
{
    //flipping around the x-axis
    flip(gDownHandImg, gDownHandImg, 0);
    _getHandRecognitizeGestureDown(gDownHandImg, this->mStdDownPointArea, ROIDOWN, handStatus, KnockNumber);
    return PT_RET_OK;
}

