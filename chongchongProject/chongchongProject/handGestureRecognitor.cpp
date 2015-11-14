#include "handGestureRecognitor.h"
#include "_utils.h"

//#define _SHOW_
#ifdef _SHOW_
//global mat for display dubug
#define  DISP_WINDOW   "dispImg"
extern Mat dispImg;
#endif
//unsigned int gHandID = 0;

//FIXME: SHIT global varible
Mat gUpHandImg;
Mat gDownHandImg;
PTSysEnum eBoard;

HandGestureRecognitor::HandGestureRecognitor()
{
   mIsUpTapPointInited   = FALSE;//the up knock base is got
   mIsDownTapPointInited = FALSE;//the down knock
   mIsCheckTapPoint      = FALSE;
   mStdUpPointArea   = 1.0f;
   mStdDownPointArea = 1.0f;
}

HandGestureRecognitor::~HandGestureRecognitor()
{
//do nothing
}

PTS32 HandGestureRecognitor::init(PTU8* pPixels, PTS32 nWidth, PTS32 nHeight, PTImageFormatEnum eFormat, const char* const pSystemInfo)
{
    if(pPixels==NULL || nWidth!=640 || nHeight!=480 || pSystemInfo==NULL) {
       PTDEBUG("Invalid parameters: pPixels[%p], nWidth[%d], nHeight[%d], eFormat[%d]\n", pPixels, nWidth, nHeight, eFormat/*, strFormat[eFormat]*/);
       return PT_RET_INVALIDPARAM;
    }

    Mat srcImg;
    _cvtColor2BGR(pPixels, nWidth, nHeight, eFormat, srcImg);

#ifdef _SHOW_
    dispImg.create(srcImg.rows, srcImg.cols, CV_8UC3);
    dispImg.setTo(Scalar::all(127));
    Mat topLeft = dispImg(Rect(0,0,dispImg.cols/2,dispImg.rows/2));
    resize(srcImg, topLeft, Size(srcImg.cols/2,srcImg.rows/2));
#endif

    //integrated in iPad
    //TODO: this roi should be tuned according to iPad hardware.
    eBoard = PT_SYS_NUM;
    parseSystemInfo(pSystemInfo, eBoard);
	eBoard = PT_APPLE_AIR;
    Mat subImg;
    switch(eBoard) {
      case PT_APPLE_IPAD2:
      case PT_APPLE_IPAD3:
      case PT_APPLE_IPAD4: {
           subImg = srcImg(Rect(125, 0, 400, 480)).clone();
           break;
           }
      case PT_APPLE_MINI1: {
           subImg = srcImg(Rect(110, 0, 440, 480)).clone();
           break;
           }
      case PT_APPLE_MINI2: {
           subImg = srcImg(Rect(160, 0, 390, 480)).clone();
           break;
           }
      case PT_APPLE_MINI3: {
           subImg = srcImg(Rect(150, 0, 350, 480)).clone();
           break;
           }
      case PT_APPLE_AIR  : {
           subImg = srcImg(Rect(145, 0, 355, 480)).clone();
           break;
           }
      case PT_APPLE_AIR2 : {
           subImg = srcImg(Rect(145, 0, 330, 480)).clone();
           break;
           }
      default: {
           subImg = srcImg(Rect(315, 0, 220, 480)).clone();
           break;
      }
    }
	imshow("subImg",subImg);
    resize(subImg, subImg, Size(subImg.cols/2, subImg.rows/2));

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

	_getKnockMask(gUpHandImg, knockMask,eBoard);

    double konckPointArea = -1.0f;
    Point centerUp = Point(-1, -1);
    vector<Point> ContoursOfUpPoint;
    _getMaxContoursAreaCenter(knockMask, konckPointArea, centerUp, ContoursOfUpPoint);
    PTDEBUG("konckPointArea[%f], centerUp[%d,%d]\n", konckPointArea, centerUp.x, centerUp.y);
    
#ifdef _SHOW_
    Mat draw(gUpHandImg.size(), CV_8UC3);
    cvtColor(knockMask, draw, CV_GRAY2BGR);
    circle(draw, centerUp, 4, CV_RGB(0,0,255));
    
    Mat topRightUp = dispImg(Rect(dispImg.cols/2+gUpHandImg.cols, 0, knockMask.cols, knockMask.rows));
    resize(draw, topRightUp, draw.size());
#endif

	_getKnockMask(gDownHandImg, knockMask,eBoard);

    Point centerDown = Point(-1,-1);
    vector<Point> ContoursOfDownPoint;
    _getMaxContoursAreaCenter(knockMask, konckPointArea, centerDown, ContoursOfDownPoint);

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
    const double distance_left   = MIN(centerUp.x, centerDown.x);

    /*below parameters should be tuned*/
    PTDEBUG("knockPointAngle[%f], distance_down[%f], distance_right[%f], distance_up[%f]\n",
             knockPointAngle, distance_down, distance_right, distance_up);

    if(knockPointAngle > 15.0f || distance_up < 20 || distance_down < 20 || distance_right < 10 || distance_left < 40 ) {
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
		_getKnockBase(gUpHandImg, this->mStdUpPointArea, this->mIsUpTapPointInited,eBoard);
       PTDEBUG("mStdUpPointArea[%f], mIsUpTapPointInited[%s]\n", this->mStdUpPointArea, this->mIsUpTapPointInited?"true":"false");
    } else {
       PTDEBUG("mIsUpTapPointInited[%s], mStdUpPointArea[%f]\n", this->mIsUpTapPointInited?"true":"false", this->mStdUpPointArea);
    }

    if(!this->mIsDownTapPointInited) {
		_getKnockBase(gDownHandImg, this->mStdDownPointArea, this->mIsDownTapPointInited,eBoard);
       PTDEBUG("mStdDownPointArea[%f], mIsDownTapPointInited[%s]\n", this->mStdDownPointArea, this->mIsDownTapPointInited?"true":"false");
    } else {
       PTDEBUG("mIsDownTapPointInited[%s], mStdDownPointArea[%f]\n", this->mIsDownTapPointInited?"true":"false", this->mStdDownPointArea);
    }

    if(this->mIsUpTapPointInited && this->mIsDownTapPointInited) {
       PTDEBUG("both mIsUpTapPointInited and mIsDownTapPointInited are true, set mIsTapPointInited to TRUE\n");
       PTDEBUG("mStdUpPointArea[%f], mStdDownPointArea[%f]\n", this->mStdUpPointArea, this->mStdDownPointArea);
       this->mIsCheckTapPoint = TRUE;
    } else {
       PTDEBUG("one of mIsUpTapPointInited[%s] and mIsDownTapPointInited[%s] is not true, set mIsTapPointInited to FALSE\n",
                this->mIsUpTapPointInited?"true":"false", this->mIsDownTapPointInited?"true":"false");
       this->mIsCheckTapPoint = FALSE;
    }

    PTDEBUG("Exit %s\n", __FUNCTION__);
    return PT_RET_OK;
}

PTS32 HandGestureRecognitor::getUpHandGesture(PTHandStatus& handStatus, int& KnockNumber)
{
	int HandType = 0;//right hand
    _getHandRecognitizeGesture(gUpHandImg, this->mStdUpPointArea,eBoard,HandType,handStatus, KnockNumber);
    //imwrite(" gUpHandImg.jpg", gUpHandImg);
    return PT_RET_OK;
}

PTS32 HandGestureRecognitor::getDownHandGesture(PTHandStatus& handStatus, int& KnockNumber)
{
    //flipping around the x-axis
    flip(gDownHandImg, gDownHandImg, 0);
    //imwrite(" gDownHandImg .jpg", gDownHandImg);
	int HandType = 1;//left hand
    _getHandRecognitizeGesture(gDownHandImg, this->mStdDownPointArea, eBoard,HandType,handStatus, KnockNumber);
    return PT_RET_OK;
}

