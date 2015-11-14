#include "_utils.h"
#include "handGestureRecognitor.h"
//#include "opencv2/imgcodecs.hpp"
#define _SHOW_
//#define USEOTSU
#define USE_SKIN_COLOR_DIFF
#ifdef _SHOW_
extern Mat dispImg;
#endif

static const int KNOCKRANGE[2][6] = {
    {20,  37,/*hue*/  60, 255,/*sat*/ 0, 256,} ,/*value*/ // blue
    {130, 170,/*hue*/ 70, 255,/*sat*/ 0, 256,}  ,/*value*/ // purple
};

static vector<double> vKnockArea;

PTS32  _getKnockBase(Mat& srcImg, double& area, PTBOOL& isFinish,PTSysEnum&eBoard)
{
    PTDEBUG("Enter %s\n", __FUNCTION__);
    
    Mat maskImg(srcImg.size(), CV_8UC1);
    //Extract the knock point using HSV
    _getKnockMask(srcImg, maskImg,eBoard);
    
    Point center;
    vector<Point> tmp_contour;
    //Extract the area
    _getMaxContoursAreaCenter(maskImg, area, center, tmp_contour);
    if(vKnockArea.size() < INIT_FRAME_NUM) {
       vKnockArea.push_back(area);
    }
    
    if(vKnockArea.size() == INIT_FRAME_NUM) {
        sort(vKnockArea.begin(), vKnockArea.end());
        area = *(vKnockArea.end()-1);
        PTDEBUG("statisticed from %d frames, knock point std area[%f]\n", (int)vKnockArea.size(), area);
        isFinish = TRUE;
    } else {
        PTDEBUG("statisticed to %d frame, target %d frames\n", (int)vKnockArea.size(), INIT_FRAME_NUM);
        isFinish = FALSE;
    }
    
    PTDEBUG("Exit %s\n", __FUNCTION__);
    return PT_RET_OK;
}

static PTS32 SkinDetectionBasedOnColorDiff(Mat& srcImg, Mat& dstImg)
{
    if(srcImg.channels() != 3){
        PTDEBUG("Error in SkinDetectionBasedOnColorDiff .The Input Image's channel must be 3");
        return PT_RET_INVALIDPARAM;
    }
    
    vector<Mat> YCrCb;
    Mat YcbcrImg;
    
    cvtColor(srcImg, YcbcrImg, CV_BGR2YCrCb);
    split(YcbcrImg, YCrCb);
    Mat imgCr = YCrCb[1];
    Mat imgCb = YCrCb[2];
    
    Mat dst(imgCr.size(), CV_8UC1);
    int rows = imgCr.rows;
    int cols = imgCr.cols * imgCr.channels();
  //  const int channels = imgCr.channels();
    
    if(imgCr.isContinuous() && dst.isContinuous()) {
        cols = cols * rows;
        rows = 1;
    }
    
    for(int i = 0;i < rows;i++){
        uchar *pCr = imgCr.ptr<uchar>(i);
        uchar *pCb = imgCb.ptr<uchar>(i);
        uchar *pDst= dst.ptr<uchar>(i);
        for(int j = 0;j < cols; j++){
            if(pCr[j] - pCb[j] < 10 || pCr[j] < 130) {
                pDst[j] = 0;
            } else {
                pDst[j] = pCr[j] - pCb[j];
            }
        }
    }
    
    normalize(dst, dstImg, 255, 0, NORM_MINMAX);
    
    threshold(dstImg, dstImg, 0, 255, CV_THRESH_OTSU);
    
    return PT_RET_OK;
}

//knock point detection in HSV space, using Hue and Saturation
PTS32 _getKnockMask(Mat& srcImg, Mat& dstImg,PTSysEnum&eBoard)
{
    PTDEBUG("Enter %s\n", __FUNCTION__);
	//select the right desktop corner threshold according to Ipad
	int RightDesktopConerthreshold = 0;
	switch(eBoard) {
      case PT_APPLE_IPAD2:
      case PT_APPLE_IPAD3:
      case PT_APPLE_IPAD4: {
           RightDesktopConerthreshold = 30; 
           break;
           }
      case PT_APPLE_MINI1: {
           RightDesktopConerthreshold = 40; 
           break;
           }
      case PT_APPLE_MINI2: {
           RightDesktopConerthreshold = 40; 
           break;
           }
      case PT_APPLE_MINI3: {
           RightDesktopConerthreshold = 30; 
           break;
           }
      case PT_APPLE_AIR  : {
           RightDesktopConerthreshold = 50;
           break;
           }
      case PT_APPLE_AIR2 : {
           RightDesktopConerthreshold = 50;
           break;
           }
      default: {
            RightDesktopConerthreshold = 30; 
           break;
      }
    }

    Mat temp(srcImg.size(), CV_8UC3);
    cvtColor(srcImg, temp, CV_RGB2HSV);

    vector<Mat> hsv;
    split(temp, hsv);
    Mat hueImg = hsv[0];
    Mat satImg = hsv[1];

    const int hueMin = KNOCKRANGE[0][0];
    const int hueMax = KNOCKRANGE[0][1];
    const int satMin = KNOCKRANGE[0][2];
    const int satMax = KNOCKRANGE[0][3];

    const int hueMin_ = KNOCKRANGE[1][0];
    const int hueMax_ = KNOCKRANGE[1][1];
    const int satMin_ = KNOCKRANGE[1][2];
    const int satMax_ = KNOCKRANGE[1][3];

    PTDEBUG("hueMin[%d], hueMax[%d], satMin[%d], satMax[%d]\n", hueMin, hueMax, satMin, satMax);

    const int rows = hueImg.rows;
    const int cols = hueImg.cols;
    const int channels = hueImg.channels();

    for(int i = 0; i < rows; i++) {
        const uchar* pHue = hueImg.ptr<uchar>(i);
        const uchar* pSat = satImg.ptr<uchar>(i);
        uchar* pDst = dstImg.ptr<uchar>(i);
        for(int j = 0; j < cols; j += channels) {
            const PTU8 hue = pHue[j];
            const PTU8 sat = pSat[j];
            if((cols - j + i > RightDesktopConerthreshold/*exclude top right desktop corner*/)
            && ((hueMin<=hue && hue<=hueMax && satMin<=sat && sat<=satMax) || (hueMin_<=hue && hue<=hueMax_ && satMin_<=sat && sat<=satMax_))) {
                pDst[j] = 255;
            } else {
                pDst[j] = 0;
            }
        }
    }

    //dilate(dstImg, dstImg, Mat());
    dilate(dstImg, dstImg, cv::Mat(), cv::Point(-1, -1), 1);
    PTDEBUG("Exit %s\n", __FUNCTION__);

    return PT_RET_OK;
}

static PTS32 _getContourCenter(const vector<Point>& contour, Point& center)
{
    Moments moment = moments(contour, true/*binary or not*/);
    double m00 = moment.m00;
    double m10 = moment.m10;
    double m01 = moment.m01;
    
    center.x = (int)cvRound(m10/(m00+DBL_MIN));
    center.y = (int)cvRound(m01/(m00+DBL_MIN));
    
    return PT_RET_OK;
}

PTS32  _getMaxContoursAreaCenter(Mat& srcImg, double& area, Point& center, vector<Point>& MaxContour)
{
    PTDEBUG("Enter %s\n", __FUNCTION__);
    
    vector<vector<Point>> contours;
    findContours(srcImg.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
    
    if(!contours.empty()) {
        //find the biggest contour
        int    maxIdx  = 0;
        double maxArea = 0.0f;
        for(size_t i = 0; i < contours.size(); i++ ) {
            double curArea = contourArea(contours[i]);
            if(curArea > maxArea) {
               maxIdx  = i;
               maxArea = curArea;
            }
        }
#ifdef _SHOW_
        imshow("contour", srcImg);
#endif
		srcImg.setTo(0);
        drawContours(srcImg, contours, maxIdx, Scalar(255), CV_FILLED);
        area = maxArea;
        _getContourCenter(contours[maxIdx], center);
        MaxContour = contours[maxIdx];
        PTDEBUG("contour area[%f], contour center[%d,%d]\n", area, center.x, center.y);
    } else {
        PTDEBUG("cann't find contour in srcImg!\n");
    }
    
    PTDEBUG("Exit %s, area[%f], center[%d,%d]\n", __FUNCTION__, area, center.x, center.y);
    return PT_RET_OK;
}

PTF64 _calcLineYAngle(const Point& start,const Point& end)
{
    //calculate the angle of two knock point center
    const double dx = abs(start.x - end.x);
    const double dy = abs(start.y - end.y);
    
    return  atan2(dx,dy+DBL_MIN)*180.0f/CV_PI;
}



PTS32 _getHandRecognitizeGesture(Mat& handImg, double& stdKnockBaseArea,PTSysEnum&eBoard,int HandType,PTHandStatus& handStatus,int& KnockNumber)
{
    PTDEBUG("Enter %s\n", __FUNCTION__);
    
    Mat skinMask(handImg.size(), CV_8UC1);
    
    Mat hand;
    
    Mat knockMask(handImg.size(), CV_8UC1);
	_getKnockMask(handImg, knockMask,eBoard);//Extract the knock point using HSV
    double area = 0.0f;
    Point tmp;
    vector<Point>tmp_contour;
    _getMaxContoursAreaCenter(knockMask, area, tmp,tmp_contour);//Extract the area and the knock center
    static Mat MaskOfKnockStaticDown = Mat::ones(knockMask.size(),CV_8UC1);
	static Mat MaskOfKnockStaticUp = Mat::ones(knockMask.size(),CV_8UC1);;
    KnockNumber = 0;
	cout<<area<<endl;
    if( area / stdKnockBaseArea > 0.8) {
		PTDEBUG("didn't detected hand: data[%p], area[%f], stdKnockBaseArea[%f],HandType[%d]\n", hand.data, area, stdKnockBaseArea,HandType);
            stdKnockBaseArea = area;
			if(HandType == 1)
              knockMask.copyTo(MaskOfKnockStaticDown);
			else
              knockMask.copyTo(MaskOfKnockStaticUp);
    } else {
        PTDEBUG("detected hand, next step is recognize it's gesture...");
        Mat ImgUsedToDetectSkin = Mat::zeros(handImg.size(),CV_8UC1) ;
        handImg.copyTo(ImgUsedToDetectSkin,knockMask < 100);
        SkinDetectionBasedOnColorDiff(ImgUsedToDetectSkin,skinMask);

        Mat SkinAboveOnKnock;
        int SumOfSkinPixel = 0;
		if(HandType == 1){
          skinMask.copyTo(SkinAboveOnKnock,MaskOfKnockStaticDown);
		}
		else{
          skinMask.copyTo(SkinAboveOnKnock,MaskOfKnockStaticUp);
		}
        
        for(int i = 0 ; i < skinMask.rows; i++){
            uchar *pData = SkinAboveOnKnock.ptr<uchar>(i);
            for(int j = 0; j < skinMask.cols; j++){
                if(pData[j] == 255) {
                    SumOfSkinPixel++;
                }
            }
        }
        
#ifdef _SHOW_
        imshow("SKINABOVE",SkinAboveOnKnock);
#endif
       // cout<<"SkinPixel:"<<SumOfSkinPixel / stdKnockBaseArea<<endl;
        PTDEBUG("SumOfSkinPixelRatio[%f]------SumOfSkinPixel[%d], stdKnockBaseArea[%f],Area[%f]\n", SumOfSkinPixel / stdKnockBaseArea, SumOfSkinPixel, stdKnockBaseArea,area);
        if(SumOfSkinPixel / stdKnockBaseArea > 0.3) {
            //  _getGesture(hand, handInfo, handStatus);//recognize fist or palm
            if(handStatus != PALM_ON && area < 50)
            {
               handStatus = HAND_STATUS_COUNT;
                PTDEBUG("The KnockPoint if off from scope{mIsCheckTapPoint\n");
                
            }else{
                handStatus = PALM_ON;
                KnockNumber = 1;
            }
        } else {
            handStatus = HAND_STATUS_COUNT;
        }
    }

#ifdef _SHOW_
	if(HandType ==1 )
	{
		Mat HandInfoShow(Size(knockMask.cols*3,knockMask.rows),CV_8UC1);
		circle(knockMask,tmp,2,Scalar(0));
		knockMask.copyTo(HandInfoShow(Rect(0,0,knockMask.cols,knockMask.rows)));
		skinMask.copyTo(HandInfoShow(Rect(skinMask.cols,0,skinMask.cols,skinMask.rows)));
		MaskOfKnockStaticDown.copyTo(HandInfoShow(Rect(MaskOfKnockStaticDown.cols*2,0,MaskOfKnockStaticDown.cols,MaskOfKnockStaticDown.rows)));
		imshow("HandInfoSHowUp",HandInfoShow);
	} else{
		Mat HandInfoShow(Size(knockMask.cols*3,knockMask.rows),CV_8UC1);
		circle(knockMask,tmp,2,Scalar(0));
		knockMask.copyTo(HandInfoShow(Rect(0,0,knockMask.cols,knockMask.rows)));
		skinMask.copyTo(HandInfoShow(Rect(skinMask.cols,0,skinMask.cols,skinMask.rows)));
		MaskOfKnockStaticUp.copyTo(HandInfoShow(Rect(MaskOfKnockStaticUp.cols*2,0,MaskOfKnockStaticUp.cols,MaskOfKnockStaticUp.rows)));
		imshow("HandInfoSHowDown",HandInfoShow);
	}
#endif
    
    PTDEBUG("Exit %s ---> handStatus[%s]\n", __FUNCTION__, strHandGesture[handStatus]);
    return PT_RET_OK;
}

//parse pDevice,just get first string
PTS32 parseSystemInfo(const char* const pSystemInfo, PTSysEnum& eBoard)
{
    PTDEBUG("Enter %s: pSystemInfo[%s], eBoard[%d].\n", __FUNCTION__, pSystemInfo, (int)eBoard);

    char deviceStr[DEVICESTR_MAX_LEN] = {""};
    strcpy(deviceStr, pSystemInfo);

    //this array should be align with system enum
    const char* stdSysStr[PT_SYS_NUM] = {"ipad_2", "ipad_3", "ipad_4",
                                         "ipad_mini_1", "ipad_mini_2", "ipad_mini_3",
                                         "ipad_air", "ipad_air_2",
                                        };

    char* retStr = strtok(deviceStr, "|");
    //system judgement
    if(retStr != NULL) {
       for(int i = 0; i < PT_SYS_NUM; i++) {
           if(strcmp(retStr, stdSysStr[i])==0) {
              eBoard = (PTSysEnum)i;
              break;
           }
       }
    } else {
       PTDEBUG("zero----invalid parameter[%s]\n", pSystemInfo);
      // return PT_RET_DEVICE_NOTSUPPORTED;
    }

    PTDEBUG("Exit %s: pSystemInfo[%s], eBoard[%d].\n", __FUNCTION__, pSystemInfo, (int)eBoard);
    return PT_RET_OK;
}
