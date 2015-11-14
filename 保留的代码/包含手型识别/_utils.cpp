#include "_utils.h"
#include "handGestureRecognitor.h"
//#include "opencv2/imgcodecs.hpp"
#define _SHOW_
//#define ENABLEADPATIVESKINMOLDE
//#define USEOTSU
#define USE_SKIN_COLOR_DIFF
#ifdef _SHOW_
extern Mat dispImg;
#endif


string osPath;

//#define WriteImg
static const double SKINMODEL[16][8] = {
    0.028239, 0.721932, 0.986365, 0.977145, 0.997612, 0.975892, 0.808283, 0.677762,
    0.040658, 0.401467, 0.891589, 0.835926, 0.953194, 1.000000, 0.746044, 0.697218,
    0.028704, 0.073676, 0.172343, 0.287715, 0.972450, 0.980788, 0.738950, 0.114838,
    0.006940, 0.008198, 0.019683, 0.161006, 0.738414, 0.975587, 0.911177, 0.171402,
    0.002269, 0.003248, 0.008702, 0.032871, 0.030043, 0.093746, 0.638541, 0.343311,
    0.001708, 0.005130, 0.008571, 0.005539, 0.000888, 0.011927, 0.006497, 0.000000,
    0.002076, 0.005760, 0.008659, 0.002474, 0.000060, 0.000855, 0.000000, 0.000000,
    0.002783, 0.002244, 0.002873, 0.000880, 0.000209, 0.017288, 0.000000, 0.000000,
    0.001326, 0.001440, 0.000971, 0.001403, 0.000480, 0.011066, 0.000000, 0.000000,
    0.000661, 0.002243, 0.002444, 0.001194, 0.016745, 0.000000, 0.000000, 0.000000,
    0.000859, 0.001524, 0.001224, 0.001291, 0.029525, 0.003372, 0.000000, 0.000000,
    0.001422, 0.005018, 0.001486, 0.000677, 0.001361, 0.002276, 0.001521, 0.000194,
    0.002894, 0.006812, 0.005115, 0.002061, 0.002241, 0.004565, 0.005651, 0.004617,
    0.003866, 0.021469, 0.046467, 0.029502, 0.033566, 0.071439, 0.128556, 0.115108,
    0.003775, 0.076951, 0.162884, 0.294479, 0.631536, 0.807116, 0.833096, 0.459355,
    0.014402, 0.337849, 0.569603, 0.701440, 0.944551, 0.948559, 0.825167, 0.549554,
};

static const int KNOCKRANGE[2][6] = {
   {85, 110,/*hue*/ 105, 255,/*sat*/ 0, 256,} ,/*value*/ // blue
   {130, 185,/*hue*/  75, 145,/*sat*/ 0, 256,} ,/*value*/ // purple
};

static vector<int> vKnockArea;

PTS32  _getKnockBase(Mat& srcImg, const RoiLocation roiLocation, double& area, PTBOOL& isFinish)
{
    PTDEBUG("Enter %s\n", __FUNCTION__);

    Mat maskImg(srcImg.size(), CV_8UC1);
    //Extract the knock point using HSV
    _getKnockMask(srcImg, maskImg, roiLocation);

    //useless
    Point center;
    //Extract the area
    _getMaxContoursAreaCenter(maskImg, area, center);

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

static PTS32 _mvgetSkinMask(Mat& srcImg, Mat& dstImg, double threshold)//skin detection using skin model.threshold represent threshold of skin
{
    Mat hsvImg(srcImg.size(), CV_8UC3);
    cvtColor(srcImg, hsvImg, CV_BGR2HSV);

    vector<Mat> hsv;
    split(hsvImg, hsv);

    Mat hueImg = hsv[0];
    Mat satImg = hsv[1];

    hueImg.convertTo(hueImg, CV_32F);
    satImg.convertTo(satImg, CV_32F);

    int rows = hueImg.rows;
    int cols = hueImg.cols*hueImg.channels();
    const int channels = hueImg.channels();

    if(hueImg.isContinuous() && satImg.isContinuous() && dstImg.isContinuous()) {
       cols = cols*rows;
       rows = 1;
    }

    for(int i = 0; i < rows; i++) {
        float *pHue = hueImg.ptr<float>(i);
        float *pSat = satImg.ptr<float>(i);
        uchar *pDst = dstImg.ptr<uchar>(i);
        for(int j = 0; j < cols; j+=channels) {
            const float hueVal = pHue[j];
            const float satVal = pSat[j];
            const int hueIdx = (int)floor(hueVal/180.0f*16.0f);
            const int satIdx = (int)floor(satVal/255.0f*8.0f);
            if(SKINMODEL[hueIdx][satIdx] < threshold) {
               pDst[j] = 0;
            } else {
               pDst[j] = 255;
            }
        }
    }

    erode(dstImg, dstImg, Mat());
    medianBlur(dstImg, dstImg, 3);

    return PT_RET_OK;
}

static PTS32 _mvgetSkinMaskOtsu(Mat& srcImg, Mat& dstImg)//skin detection using skin model.threshold represent threshold of skin
{
    Mat hsvImg(srcImg.size(), CV_8UC3);

    cvtColor(srcImg, hsvImg, CV_BGR2HSV);

    vector<Mat> hsv;
    split(hsvImg, hsv);

    Mat hueImg = hsv[0];
    Mat satImg = hsv[1];
    hueImg.convertTo(hueImg, CV_32F);
    satImg.convertTo(satImg, CV_32F);

    int rows = hueImg.rows;
    int cols = hueImg.cols*hueImg.channels();
    const int channels = hueImg.channels();

    if(hueImg.isContinuous() && satImg.isContinuous() && dstImg.isContinuous()) {
       cols = cols*rows;
       rows = 1;
    }

    for(int i = 0; i < rows; i++) {
        float *pHue = hueImg.ptr<float>(i);
        float *pSat = satImg.ptr<float>(i);
        uchar *pDst = dstImg.ptr<uchar>(i);
        for(int j = 0; j < cols; j+=channels) {
            const float hueVal = pHue[j];
            const float satVal = pSat[j];
            const int hueIdx = (int)floor(hueVal/180.0f*16.0f);
            const int satIdx = (int)floor(satVal/255.0f*8.0f);
            pDst[j] = SKINMODEL[hueIdx][satIdx]*256;
        }
    }

    threshold(dstImg, dstImg, 0, 255, CV_THRESH_OTSU);
    erode(dstImg, dstImg, Mat());
    medianBlur(dstImg, dstImg, 3);

    return PT_RET_OK;
}

//skin detection using Cr in YCrCb color space
static PTS32 _getSkinMask(Mat& srcImg, Mat& dstImg)
{
    Mat tempImg(srcImg.size(), CV_8UC3);
    cvtColor(srcImg, tempImg, CV_BGR2YCrCb);

    vector<Mat> yCrCb;
    split(tempImg, yCrCb);
    Mat imgCr = yCrCb[1];

    int rows = imgCr.rows;
    int cols = imgCr.cols*imgCr.channels();
    const int channels = imgCr.channels();

    if(imgCr.isContinuous() && dstImg.isContinuous()) {
       cols = cols*rows;
       rows = 1;
    }

    for(int i = 0; i < rows; i++) {
       uchar* pCr  = imgCr.ptr<uchar>(i);
       uchar* pDst = dstImg.ptr<uchar>(i);
       for(int j = 0; j < cols; j+=channels) {
         if((CR_SKIN_MIN<=pCr[j] && pCr[j]<=CR_SKIN_MAX)) {
            pDst[j] = 255;
         } else {
            pDst[j] = 0;
         }
       }
    }

    erode(dstImg, dstImg, Mat());

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
    const int channels = imgCr.channels();

    if(imgCr.isContinuous() && dst.isContinuous()) {
       cols = cols * rows;
       rows = 1;
    }

    for(int i = 0;i < rows;i++){
       uchar *pCr = imgCr.ptr<uchar>(i);
       uchar *pCb = imgCb.ptr<uchar>(i);
       uchar *pDst= dst.ptr<uchar>(i);
       for(int j = 0;j < cols; j++){
          if(pCr[j] - pCb[j] < 17) {
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
PTS32 _getKnockMask(Mat& srcImg, Mat& dstImg, const RoiLocation roiLocation)
{
    PTDEBUG("Enter %s\n", __FUNCTION__);

    Mat temp(srcImg.size(), CV_8UC3);
    cvtColor(srcImg, temp, CV_BGR2HSV);

    vector<Mat> hsv;
    split(temp, hsv);
    Mat hueImg = hsv[0];
    Mat satImg = hsv[1];

    int rows = hueImg.rows;
    int cols = hueImg.cols;
    const int channels = hueImg.channels();
	Mat dstImgtmp = dstImg.clone();
    if(hueImg.isContinuous() && dstImg.isContinuous()) {
       cols = hueImg.cols * hueImg.rows * hueImg.channels();
       rows = 1;
    }

    const int hueMin = KNOCKRANGE[0][0];
    const int hueMax = KNOCKRANGE[0][1];
    const int satMin = KNOCKRANGE[0][2];
    const int satMax = KNOCKRANGE[0][3];
    const int valMin = KNOCKRANGE[0][4];
    const int valMax = KNOCKRANGE[0][5];

	const int hueMin_ = KNOCKRANGE[1][0];
    const int hueMax_ = KNOCKRANGE[1][1];
    const int satMin_ = KNOCKRANGE[1][2];
    const int satMax_ = KNOCKRANGE[1][3];
    const int valMin_ = KNOCKRANGE[1][4];
    const int valMax_ = KNOCKRANGE[1][5];

    PTDEBUG("roiLocation[%d]->[%s], hueMin[%d], hueMax[%d], satMin[%d], satMax[%d], valMin[%d], valMax[%d]\n",
            (int)roiLocation, strROILocation[roiLocation], hueMin, hueMax, satMin, satMax, valMin, valMax);

    for(int i = 0; i < rows; i++) {
      const uchar* pHue = hueImg.ptr<uchar>(i);
      const uchar* pSat = satImg.ptr<uchar>(i);
      uchar* pDst = dstImg.ptr<uchar>(i);
	  uchar* pDsttmp = dstImgtmp.ptr<uchar>(i);
      for(int j = 0; j < cols; j+=channels) {
        const PTU8 hue = pHue[j];
        const PTU8 sat = pSat[j];
        if(hueMin<=hue && hue<=hueMax && satMin<=sat && sat<=satMax) {
           pDst[j] = 255;
        } else {
           pDst[j] = 0;
        }

		if((hueMin_<=hue && hue<=hueMax_ && satMin_<=sat && sat<=satMax_)) {
           pDsttmp[j] = 255;
        } else {
           pDsttmp[j] = 0;
        }
     }
   }
   max(dstImg,dstImgtmp,dstImg);
   dilate(dstImg, dstImg, Mat());

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

PTS32  _getMaxContoursAreaCenter(Mat& srcImg, double& area, Point& center)
{
    PTDEBUG("Enter %s\n", __FUNCTION__);

    vector<vector<Point>> contours;
    findContours(srcImg.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

    if(!contours.empty()) {
       //find the biggest contour
       int    maxIdx  = 0;
       double maxArea = 0.0f;
       for(int i = 0; i < contours.size(); i++ ) {
         double curArea = contourArea(contours[i]);
         if(curArea > maxArea) {
            maxIdx  = i;
            maxArea = curArea;
         }
       }

       area = maxArea;
       _getContourCenter(contours[maxIdx], center);
       PTDEBUG("contour area[%f], contour center[%d,%d]\n", area, center.x, center.y);
    } else {
       PTDEBUG("cann't find contour in srcImg!\n");
    }

    PTDEBUG("Exit %s, area[%f], center[%d,%d]\n", __FUNCTION__, area, center.x, center.y);
    return PT_RET_OK;
}

static PTS32 _getHandInfo(Mat& skinArea, Mat& hand, HandInfo& handInfo)
{
    Mat gray = skinArea.clone();

    vector<vector<Point>> contours;
    findContours(gray.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    if(!contours.empty()) {

       int    maxIdx  = 0;
       double maxArea = 0.0f;
       for(size_t i = 0; i < contours.size(); i++) {
           const double area = contourArea(contours[i]);
           if(area > maxArea) {
              maxIdx  = i;
              maxArea = area;
           }
       }

       const vector<Point> maxContour = contours[maxIdx];
/*************************************************************************/
       Rect handRect = boundingRect(maxContour);

       Point handCenter;
       _getContourCenter(maxContour, handCenter);
	   
	   vector<Point> TrajectoryLeft;
	   for(size_t i = 0;i < maxContour.size();i++){
		   if(maxContour[i].x <= handCenter.x){
			   TrajectoryLeft.push_back(maxContour[i]);
		   }
	   }
	   vector<Point> PloyCurve;
	   approxPolyDP(TrajectoryLeft,PloyCurve,7,true);
	   vector<Point> HandDefectsNum;
	   for(int i = 1;(i < PloyCurve.size() - 1) && (PloyCurve.size() >=3); i++){
		   Point PrePoint = PloyCurve[i-1];
		   Point CurPoint = PloyCurve[i];
		   Point LastPoint = PloyCurve[i+1];
		   int coss = (PrePoint.x - CurPoint.x) * (LastPoint.y - CurPoint.y) - (LastPoint.x - CurPoint.x) * (PrePoint.y - CurPoint.y);
		   if(coss < 0){
			   HandDefectsNum.push_back(CurPoint);
		   }
	   }
	   if(HandDefectsNum .size() != 0)
	   {
		   handInfo.HandDefectsNum = HandDefectsNum.size();
	   }
#ifdef _SHOW_
	   Mat DrawContoursShow=Mat::zeros(skinArea.size(),CV_8UC3);
	   vector<vector<Point>> tmp;
	   tmp.push_back(TrajectoryLeft);
	   drawContours(DrawContoursShow,tmp,0,Scalar(255));
	   for(size_t i = 0; i <HandDefectsNum.size();i++){
		   circle(DrawContoursShow,HandDefectsNum[i],3,Scalar(0,0,255),2);
	   }
 	   for(vector<Point>::const_iterator itp = PloyCurve.begin(); itp != PloyCurve.end()-1;itp++){
		    Scalar color( 255,255,0 );
		   line(DrawContoursShow,*itp,*(itp+1),color,2);
	   }
	   imshow("DrawContoursShow",DrawContoursShow);
#endif

/*************************************************************************************************************/
       hand = gray(Rect(handRect.x, handRect.y, handRect.width*3/5, handRect.height));

       vector<int> hull;
       convexHull(maxContour, hull, true/*clockwise*/);
       //wether the thumb is exist
       if(Mat(maxContour).checkVector(2, CV_32S) > 3) {
          vector<Vec4i> defects;
          convexityDefects(maxContour, Mat(hull), defects);

          double farY = 0;
          int thumbIndex  = -1;

          for(size_t i = 0; i < defects.size(); i++) {
              const int startIdx = defects[i][0];
              const Point ptStart(maxContour[startIdx]); // point of the contour where the defect begins

              const int endIdx = defects[i][1];
              const Point ptEnd(maxContour[endIdx]); // point of the contour where the defect ends

              const int farIdx = defects[i][2];
              const Point ptFar(maxContour[farIdx]);// the farthest from the convex hull point within the defect

              const double depth = defects[i][3]/256; // distance between the farthest point and the convex hull
			  //The thumb area constraints,TODO
              //if(depth>20.0f and (ptFar.x>ptEnd.x||ptFar.x>ptStart.x) and (ptFar.y-handRect.y)>50 and (skinArea.cols-ptEnd.x)>10) {
              if(depth>10.0f && (ptFar.x>ptEnd.x||ptFar.x>ptStart.x) && (ptFar.y-handRect.y)>30 && (skinArea.cols-ptEnd.x)>10) {
                if(ptEnd.y > farY) {
                   farY = ptFar.y;
                   thumbIndex = i;
                }
             }
          }  

          if(thumbIndex > -1) {
             PTDEBUG("found a thumb, thumbIndex[%d]\n", thumbIndex);
             vector<Point> thumbContour;

             const int startIdx = defects[thumbIndex][0];
             thumbContour.push_back(maxContour[startIdx]);

             const int endIdx = defects[thumbIndex][1];
             thumbContour.push_back(maxContour[endIdx]);

             const int farIdx = defects[thumbIndex][2];
             thumbContour.push_back(maxContour[farIdx]);

			 //cout<<defects[thumbIndex][3]/256<<endl;
             Rect rect = boundingRect(thumbContour);

 #ifdef _SHOW_
             static int handID = 0;
             Mat show(skinArea.size(), CV_8UC3);
             cvtColor(skinArea, show, CV_GRAY2BGR);

			 if(rect.height * rect.width >200){
		     circle(show, handCenter, 4, CV_RGB(0,255,0));
             circle( show, thumbContour[0],   4, Scalar(255,0,100), 2 );
             circle( show, thumbContour[1],   4, Scalar(255,0,100), 2 );
             circle( show, thumbContour[2],   4, Scalar(100,0,255), 2 );
             rectangle(show,rect,Scalar(255),2);
			 }
             imshow(" thumb", show);
             // cvWaitKey(0);
             handID++;
#endif
             double LengthOfHandcenterAndthumb = pow(thumbContour[0].x-handCenter.x,2)+pow(thumbContour[0].y-handCenter.y,2);
             double LengthOfHandcenterAndthumbFar = pow(thumbContour[2].x-handCenter.x,2);
            // cout<<LengthOfHandcenterAndthumb/LengthOfHandcenterAndthumbFar<<"rect.height / rect.width "<<(double)rect.height / rect.width<<endl;
             // if the handCenter is in the right of thumb rect origin
			 if( ((double)rect.height / rect.width < 1.2 || LengthOfHandcenterAndthumb/LengthOfHandcenterAndthumbFar >10) && rect.height * rect.width >200) {
                handInfo.Thumb = 0;
              } else {
                handInfo.Thumb = 1;
              }
          } else {
            handInfo.Thumb = 1;
          }
       }
   } else {
     PTDEBUG("didn't find hand contour in this mask!\n");
   }

   return PT_RET_OK;
}

PTF64 _calcLineYAngle(const Point& start,const Point& end)
{
    //calculate the angle of two knock point center
    const double dx = abs(start.x - end.x);
    const double dy = abs(start.y - end.y);

    return  atan2(dx,dy+DBL_MIN)*180.0f/CV_PI;
}

static PTS32 _getGesture(Mat& hand, const HandInfo& handInfo, PTHandStatus& handStatus)
{
    PTDEBUG("Enter %s, handInfo.Thumb[%d], handInfo.ratio_hull_handarea[%f]\n", __FUNCTION__, handInfo.Thumb, handInfo.ratio_hull_handarea);

    const int step = 3;
    const int lineCount = 10;
    const int rows = hand.rows;
    const int cols = hand.cols;
    const int lineDistance = floor(hand.cols/double(lineCount));

    //The number of finger along a line
    vector<int> fingerNum;

    int num = 0;
    for(int i = 0; i < cols; i += lineDistance+1) {
        int acc = 0;
        PTBOOL startCount = FALSE;
        int prePixel = 0;
        for(int j = 0; j < rows; j++) {
            uchar currentPixel = hand.at<uchar>(j,i);
            //circle(gray,Point(h,w),2,CV_RGB(255,0,245));
            if(prePixel == currentPixel) {
               acc++;
            } else {//change,i.e.boarder
               if(prePixel == 0) {//white to black
                  if(!startCount) {
                     startCount = TRUE;
                  } else {
                     if(acc >= step) {
                        num++;
                     }
                  }
               }
               prePixel = currentPixel;
               acc = 0;
            }
        }
        fingerNum.push_back(num);
    }

    const Scalar meanvalue = mean(fingerNum);
    const int finger = meanvalue(0);

    PTDEBUG("finger[%d], handInfo.Thumb[%d], handInfo.ratio_hull_handarea[%d]\n", finger, handInfo.Thumb, handInfo. HandDefectsNum);
	//printf("finger[%d], handInfo.Thumb[%d], handInfo.HandDefectsNum[%d]\n", finger, handInfo.Thumb, handInfo. HandDefectsNum);
    //if number of finger is greater than 4,then palm
    if(finger > 3) {
       handStatus = PALM_ON;
    } else {
		if(handInfo.HandDefectsNum >1 || handInfo.Thumb == 0)
		{
			handStatus = PALM_ON;
		}else{
			handStatus = FIST_ON;
		}
    }

    PTDEBUG("Exit %s ---> handStatus[%d]\n", __FUNCTION__, handStatus);

    return PT_RET_OK;
}

PTS32 _getHandRecognitizeGestureUp(Mat& handImg, double& stdKnockBaseArea, const RoiLocation roiLocation, PTHandStatus& handStatus,int& KnockNumber)
{
#ifdef WriteImg
    osPath = filename+"up.os.png";
#endif

    PTDEBUG("Enter %s\n", __FUNCTION__);

    Mat skinMask(handImg.size(), CV_8UC1);

    Mat hand;
    Mat knockMask(handImg.size(), CV_8UC1);
    _getKnockMask(handImg, knockMask, roiLocation);//Extract the knock point using HSV

    double area = 0.0f;
    Point tmp;
    _getMaxContoursAreaCenter(knockMask, area, tmp);//Extract the area and the knock center

    static Mat MaskOfKnockStaticUp = Mat::ones(knockMask.size(),CV_8UC1);
    static bool StartToDetectKnockEnd = false;

    KnockNumber = 0;
    if( area/stdKnockBaseArea>0.8) {
       PTDEBUG("didn't detected hand: data[%p], area[%f], stdKnockBaseArea[%f]\n", hand.data, area, stdKnockBaseArea);
       if(area/stdKnockBaseArea > 0.8) {
          stdKnockBaseArea = area;
          //MaskOfKnockStaticUp = knockMask;
          knockMask.copyTo(MaskOfKnockStaticUp);
       }

       if(StartToDetectKnockEnd) {
          StartToDetectKnockEnd = false;
          handStatus = HAND_KNOCK_END;
          //NumberOfKnock++;
       } else {
          handStatus = HAND_STATUS_COUNT;
       }
    } else {
       PTDEBUG("detected hand, next step is recognize it's gesture...");
#ifdef ENABLEADPATIVESKINMOLDE
     _mvgetSkinMask(handImg, skinMask, 0.75);
#else
#ifdef USE_SKIN_COLOR_DIFF
     SkinDetectionBasedOnColorDiff(handImg, skinMask);
#else
     _getSkinMask(handImg, skinMask);
#endif
#endif

#ifdef WriteImg
    imwrite(filename+".up.skinMask.src.png", handImg);
    imwrite(filename+".up.skinMask.dst.png", skinMask);
#endif
	   HandInfo handInfo = HandInfo(0.0f, 0.0f);
       _getHandInfo(skinMask, hand, handInfo);//Extract the hand using YCrCb color space
       Mat SkinAboveOnKnock;
       int SumOfSkinPixel = 0;
       skinMask.copyTo(SkinAboveOnKnock, MaskOfKnockStaticUp);

#ifdef WriteImg
       imwrite(filename+".up.png", skinMask);
#endif
       for(int i = 0 ; i < skinMask.rows; i++){
           uchar *pData = SkinAboveOnKnock.ptr<uchar>(i);
           for(int j = 0; j < skinMask.cols; j++){
               if(pData[j] == 255){
                  SumOfSkinPixel++;
               }
           }
       }
#ifdef WriteImg
       imwrite(filename+".up.png.png", SkinAboveOnKnock);
#endif

#ifdef _SHOW_
       imshow("SKINABOVE", SkinAboveOnKnock);
#endif
      //cout<<"SkinPixel:"<<SumOfSkinPixel / stdKnockBaseArea<<endl;
       PTDEBUG("SumOfSkinPixel[%f]------SumOfSkinPixel[%d], stdKnockBaseArea[%f]\n", SumOfSkinPixel / stdKnockBaseArea, SumOfSkinPixel, stdKnockBaseArea);
       if(SumOfSkinPixel / stdKnockBaseArea > 0.3) {
          _getGesture(hand, handInfo, handStatus);//recognize fist or palm
          StartToDetectKnockEnd = true;
          KnockNumber = 1;
          //KnockNumber = NumberOfKnock;
       } else{
          handStatus = HAND_STATUS_COUNT;
       }
    }

#ifdef _SHOW_
    Mat UpHandInfoShow(Size(knockMask.cols*3,knockMask.rows),CV_8UC1);
	knockMask.copyTo(UpHandInfoShow(Rect(0,0,knockMask.cols,knockMask.rows)));
	skinMask.copyTo(UpHandInfoShow(Rect(skinMask.cols,0,skinMask.cols,skinMask.rows)));
	MaskOfKnockStaticUp.copyTo(UpHandInfoShow(Rect(MaskOfKnockStaticUp.cols*2,0,MaskOfKnockStaticUp.cols,MaskOfKnockStaticUp.rows)));
	
	imshow("UpHandInfoSHow",UpHandInfoShow);
	if(!hand.empty()){
	  imshow("Hand",hand);
	}
#endif

    PTDEBUG("Exit %s ---> handStatus[%s]\n", __FUNCTION__, strHandGesture[handStatus]);
    return PT_RET_OK;
}

PTS32 _getHandRecognitizeGestureDown(Mat& handImg, double& stdKnockBaseArea, const RoiLocation roiLocation, PTHandStatus& handStatus,int& KnockNumber)
{
#ifdef WriteImg
    osPath = filename+"down.os.png";
#endif

    PTDEBUG("Enter %s\n", __FUNCTION__);

    Mat skinMask(handImg.size(), CV_8UC1);

    Mat hand;
    
    Mat knockMask(handImg.size(), CV_8UC1);
    _getKnockMask(handImg, knockMask, roiLocation);//Extract the knock point using HSV
    double area = 0.0f;
    Point tmp;
    _getMaxContoursAreaCenter(knockMask, area, tmp);//Extract the area and the knock center

    static Mat MaskOfKnockStaticDown = Mat::ones(knockMask.size(),CV_8UC1);;
    static bool StartToDetectKnockEnd = false;
    //static int NumberOfKnock = 0;
    //KnockNumber = -1;
    KnockNumber = 0;
    if( area/stdKnockBaseArea>0.8) {

        PTDEBUG("didn't detected hand: data[%p], area[%f], stdKnockBaseArea[%f]\n", hand.data, area, stdKnockBaseArea);
        if (area/stdKnockBaseArea>0.8) {
            stdKnockBaseArea = area;
            //MaskOfKnockStaticDown = knockMask;
            knockMask.copyTo(MaskOfKnockStaticDown);
        }

        if( StartToDetectKnockEnd ){
            StartToDetectKnockEnd = false;
            handStatus = HAND_KNOCK_END;
            //NumberOfKnock++;
        } else {
            handStatus = HAND_STATUS_COUNT;
        }
    } else {
        PTDEBUG("detected hand, next step is recognize it's gesture...");
#ifdef ENABLEADPATIVESKINMOLDE
    _mvgetSkinMask(handImg, skinMask, 0.75);
#else
#ifdef USE_SKIN_COLOR_DIFF
    SkinDetectionBasedOnColorDiff(handImg,skinMask);
#else
    _getSkinMask(handImg, skinMask);
#endif
#endif

#ifdef WriteImg
    imwrite(filename+".down.skinMask.src.png", handImg);
    imwrite(filename+".down.skinMask.dst.png", skinMask);
#endif
	    HandInfo handInfo = HandInfo(0.0f, 0.0f);
        _getHandInfo(skinMask, hand, handInfo);//Extract the hand using YCrCb color space
        Mat SkinAboveOnKnock;
        int SumOfSkinPixel = 0;
        skinMask.copyTo(SkinAboveOnKnock,MaskOfKnockStaticDown);

#ifdef WriteImg
        imwrite(filename+".down.png", skinMask);
#endif
        for(int i = 0 ; i < skinMask.rows; i++){
            uchar *pData = SkinAboveOnKnock.ptr<uchar>(i);
            for(int j = 0; j < skinMask.cols; j++){
                if(pData[j] == 255) {
                   SumOfSkinPixel++;
                }
            }
        }

#ifdef WriteImg
        imwrite(filename+".down.png.png", SkinAboveOnKnock);
#endif

#ifdef _SHOW_
        imshow("SKINABOVE",SkinAboveOnKnock);
#endif
       //cout<<"SkinPixel:"<<SumOfSkinPixel / stdKnockBaseArea<<endl;
       PTDEBUG("SumOfSkinPixel[%f]------SumOfSkinPixel[%d], stdKnockBaseArea[%f]\n", SumOfSkinPixel / stdKnockBaseArea, SumOfSkinPixel, stdKnockBaseArea);
       if(SumOfSkinPixel / stdKnockBaseArea > 0.3) {
          _getGesture(hand, handInfo, handStatus);//recognize fist or palm
          StartToDetectKnockEnd = true;
          //KnockNumber = NumberOfKnock;
          KnockNumber = 1;
       } else {
          handStatus = HAND_STATUS_COUNT;
       }
    }

#ifdef _SHOW_
    Mat DownHandInfoShow(Size(knockMask.cols*3,knockMask.rows),CV_8UC1);
	knockMask.copyTo(DownHandInfoShow(Rect(0,0,knockMask.cols,knockMask.rows)));
	skinMask.copyTo(DownHandInfoShow(Rect(skinMask.cols,0,skinMask.cols,skinMask.rows)));
	MaskOfKnockStaticDown.copyTo(DownHandInfoShow(Rect(MaskOfKnockStaticDown.cols*2,0,MaskOfKnockStaticDown.cols,MaskOfKnockStaticDown.rows)));
	
	imshow("DownHandInfoSHow",DownHandInfoShow);
	if(!hand.empty()){
	  imshow("HandDown",hand);
	}
#endif

    PTDEBUG("Exit %s ---> handStatus[%s]\n", __FUNCTION__, strHandGesture[handStatus]);
    return PT_RET_OK;
}

