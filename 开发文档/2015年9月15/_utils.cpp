#include "_utils.h"
#include "handGestureRecognitor.h"

static const double SKINMODEL[16][8] = {
    0.028239 ,0.721932 ,0.986365 ,0.977145 ,0.997612 ,0.975892 ,0.808283 ,0.677762 ,
    0.040658 ,0.401467 ,0.891589 ,0.835926 ,0.953194 ,1.000000 ,0.746044 ,0.697218 ,
    0.028704 ,0.073676 ,0.172343 ,0.287715 ,0.972450 ,0.980788 ,0.738950 ,0.114838 ,
    0.006940 ,0.008198 ,0.019683 ,0.161006 ,0.738414 ,0.975587 ,0.911177 ,0.171402 ,
    0.002269 ,0.003248 ,0.008702 ,0.032871 ,0.030043 ,0.093746 ,0.638541 ,0.343311 ,
    0.001708 ,0.005130 ,0.008571 ,0.005539 ,0.000888 ,0.011927 ,0.006497 ,0.000000 ,
    0.002076 ,0.005760 ,0.008659 ,0.002474 ,0.000060 ,0.000855 ,0.000000 ,0.000000 ,
    0.002783 ,0.002244 ,0.002873 ,0.000880 ,0.000209 ,0.017288 ,0.000000 ,0.000000 ,
    0.001326 ,0.001440 ,0.000971 ,0.001403 ,0.000480 ,0.011066 ,0.000000 ,0.000000 ,
    0.000661 ,0.002243 ,0.002444 ,0.001194 ,0.016745 ,0.000000 ,0.000000 ,0.000000 ,
    0.000859 ,0.001524 ,0.001224 ,0.001291 ,0.029525 ,0.003372 ,0.000000 ,0.000000 ,
    0.001422 ,0.005018 ,0.001486 ,0.000677 ,0.001361 ,0.002276 ,0.001521 ,0.000194 ,
    0.002894 ,0.006812 ,0.005115 ,0.002061 ,0.002241 ,0.004565 ,0.005651 ,0.004617 ,
    0.003866 ,0.021469 ,0.046467 ,0.029502 ,0.033566 ,0.071439 ,0.128556 ,0.115108 ,
    0.003775 ,0.076951 ,0.162884 ,0.294479 ,0.631536 ,0.807116 ,0.833096 ,0.459355 ,
    0.014402 ,0.337849 ,0.569603 ,0.701440 ,0.944551 ,0.948559 ,0.825167 ,0.549554 ,
};

static const int KNOCKRANGE[2][6] = {
    //{90, 140,/*hue*/ 50, 200,/*sat*/ 0, 256,} ,/*value*/ //up point, blue
    {37, 75,/*hue*/  50, 200,/*sat*/ 0, 256,} ,/*value*/ //down point, green
	{37, 75,/*hue*/  50, 200,/*sat*/ 0, 256,} ,/*value*/ //down point, green
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
    _getContoursAreaCenter(maskImg, area, center);

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
    Mat hsvImg;
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

//skin detection using Cb in YCrCb color space
static PTS32 _getSkinMask(Mat& srcImg, Mat& dstImg)
{
    Mat tempImg;
    cvtColor(srcImg, tempImg, CV_RGB2YCrCb);

    vector<Mat> yCrCb;
    split(tempImg, yCrCb);
    Mat imgCb = yCrCb[2];

    int rows = imgCb.rows;
    int cols = imgCb.cols*imgCb.channels();
    const int channels = imgCb.channels();

    if(imgCb.isContinuous() && dstImg.isContinuous()) {
       cols = cols*rows;
       rows = 1;
    }

    for(int i = 0; i < rows; i++) {
       uchar* pCb  = imgCb.ptr<uchar>(i);
       uchar* pDst = dstImg.ptr<uchar>(i);
       for(int j = 0; j < cols; j+=channels) {
         if((CB_SKIN_MIN<=pCb[j] && pCb[j]<=CB_SKIN_MAX)) {
            pDst[j] = 255;
         } else {
            pDst[j] = 0;
         }
       }
    }

    erode(dstImg, dstImg, Mat());

    return PT_RET_OK;
}

//knock point detection in HSV space, using Hue and Saturation
PTS32 _getKnockMask(Mat& srcImg, Mat& dstImg, const RoiLocation roiLocation)
{
    PTDEBUG("Enter %s\n", __FUNCTION__);

    Mat temp;
    cvtColor(srcImg, temp, CV_BGR2HSV);

    vector<Mat> hsv;
    split(temp, hsv);
    Mat hueImg = hsv[0];
    Mat satImg = hsv[1];

    int rows = hueImg.rows;
    int cols = hueImg.cols;
    const int channels = hueImg.channels();

    if(hueImg.isContinuous() && dstImg.isContinuous()) {
       cols = hueImg.cols * hueImg.rows * hueImg.channels();
       rows = 1;
    }

    const int hueMin = KNOCKRANGE[roiLocation][0];
    const int hueMax = KNOCKRANGE[roiLocation][1];
    const int satMin = KNOCKRANGE[roiLocation][2];
    const int satMax = KNOCKRANGE[roiLocation][3];
    const int valMin = KNOCKRANGE[roiLocation][4];
    const int valMax = KNOCKRANGE[roiLocation][5];

    PTDEBUG("roiLocation[%d], hueMin[%d], hueMax[%d], satMin[%d], satMax[%d], valMin[%d], valMax[%d]\n", roiLocation, hueMin, hueMax, satMin, satMax, valMin, valMax);

    for(int i = 0; i < rows; i++) {
      const uchar* pHue = hueImg.ptr<uchar>(i);
      const uchar* pSat = satImg.ptr<uchar>(i);
      uchar* pDst = dstImg.ptr<uchar>(i);
      for(int j = 0; j < cols; j+=channels) {
        const PTU8 hue = pHue[j];
        const PTU8 sat = pSat[j];
        if(hueMin<=hue && hue<=hueMax && satMin<=sat && sat<=satMax) {
           pDst[j] = 255;
        } else {
           pDst[j] = 0;
        }
     }
   }

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

PTS32  _getContoursAreaCenter(Mat& srcImg, double& area, Point& center)
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

       Rect handRect = boundingRect(maxContour);
       const double perimeter = arcLength(maxContour, true);

       RotatedRect box = minAreaRect(maxContour);
       Point2f vertex[4];
       box.points(vertex);
       const double width  = (pow((vertex[1].x-vertex[2].x), 2.0f) + pow((vertex[1].y-vertex[2].y), 2.0f));
       const double height = (pow((vertex[1].x-vertex[0].x), 2.0f) + pow((vertex[1].y-vertex[0].y), 2.0f));
       const double widthHeightRatio = width/height;

       Point handCenter;
       _getContourCenter(maxContour, handCenter);
       const Point center_gravy = Point(handCenter.x+handRect.x, handCenter.y+handRect.y);//TODO think again

       if(handCenter.x*1.2+1 < handRect.width) {
          hand = gray(Rect(handRect.x, handRect.y, handCenter.x*1.2+1, handRect.height));
       } else {
          hand = gray(handRect);
       }

#ifdef _SHOW_
       static int handID = 0;
       if(handID%2) {
          Mat show(skinArea.size(), CV_8UC3);
          cvtColor(skinArea, show, CV_GRAY2BGR);

          circle(show, handCenter, 4, CV_RGB(0,255,0));
          circle(show, center_gravy, 4, CV_RGB(255,0,0));
          imshow("show_center[Down]", show);

          imshow("hand[Down]", hand);
       } else {
          imshow("hand[Up]", hand);
       }
       handID++;
#endif

       vector<int> hull;
       convexHull(maxContour, hull, true/*clockwise*/);

       vector<Point> hullContour;
       for(size_t i = 0; i < hull.size(); i++) {
           hullContour.push_back(maxContour[hull[i]]);
       }

#ifdef _SHOW_
       Mat show = Mat::zeros(skinArea.size(),CV_8UC3);
       vector<vector<Point>> contours;
       contours.push_back(hullContour);
       if(handID%2) {
          drawContours(show, contours, -1/*all*/, CV_RGB(0,0,255), 2);
          imshow("hull[Up]", show);
       } else {
          drawContours(show, contours, -1/*all*/, CV_RGB(0,255,0), 2);
          imshow("hull[Down]", show);
       }
       waitKey(0);
#endif

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

              const double depth = defects[i][3]/sqrt(width); // distance between the farthest point and the convex hull
              //The thumb area constraints,TODO
            //  if(depth>20.0f && (ptFar.x>ptEnd.x||ptFar.x>ptStart.x) && (ptFar.y-handRect.y)>50 && (skinArea.cols-ptEnd.x)>10) {
			  if(depth>15.0f && (ptFar.x>ptEnd.x||ptFar.x>ptStart.x) && (ptFar.y-handRect.y)>50 && (skinArea.cols-ptEnd.x)>10) {
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


             Rect rect = boundingRect(thumbContour);

             Rect rectThumb = Rect(rect.x, rect.y, abs(handCenter.x-rect.x)+2, rect.height);
             // if the handCenter is in the right of thumb rect origin
             if(handCenter.x > rect.x) {
                PTDEBUG("handCenter.x[%d] > rect.x[%d]\n", handCenter.x, rect.x);
                if(rectThumb.x+abs(handCenter.x-rect.x)+2<skinArea.cols && rectThumb.y+rectThumb.height<skinArea.rows && rectThumb.width>0) {
                   vector<vector<Point>> contoursThumb;
                   double thumbArea = 0.0f;
                   findContours(gray(rectThumb).clone(), contoursThumb, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
                   for(size_t i = 0; i < contoursThumb.size();i++){
                       const double area = contourArea(contoursThumb[i]);
                       if(area > thumbArea) {
                          thumbArea = area;
                       }
                   }

                   if(thumbArea > 0.0f) {
                      handInfo.Thumb = thumbArea/rectThumb.area();
                   } else {
                      handInfo.Thumb = 1.0f;
                   }
                } else {
                      handInfo.Thumb = 1.0f;
                }
             } else {
                PTDEBUG("handCenter.x[%d] <= rect.x[%d]\n", handCenter.x, rect.x);
                Rect rectThumb = rect;
                if(rectThumb.x+rect.width<skinArea.cols && rectThumb.y+rectThumb.height<skinArea.rows && rectThumb.width>0) {
                   vector<vector<Point>> contoursThumb;
                   double thumbArea = 0.0f;
                   findContours(gray(rectThumb).clone(), contoursThumb, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
                   for(size_t i = 0; i < contoursThumb.size(); i++) {
                       const double area = contourArea(contoursThumb[i]);
                       if(area > thumbArea) {
                          thumbArea = area;
                       }
                   }

                   if(thumbArea > 0.0f) {
                      handInfo.Thumb = thumbArea/rectThumb.area();
                   } else {
                      handInfo.Thumb = 1.0f;
                   }
                } else {
                    handInfo.Thumb = 1.0f;
                }
             }
          } else {
            handInfo.Thumb = 1.0f;
          }

          //calculate the hull area
          const double hullArea = contourArea(hullContour);

          const double defectsArea = hullArea - maxArea;
          if(defectsArea > 1.0f) {
             //handInfo.ratio_hull_handarea = pow(widthHeightRatio, 2.0f) * defectsArea / pow(perimeter, 2.0f) / pow(maxArea, 2.0f);
			   handInfo.ratio_hull_handarea = pow(widthHeightRatio, 2.0f) * defectsArea *pow(perimeter, 2.0f) /pow(maxArea,2.0f);
          } else {
             handInfo.ratio_hull_handarea = hullArea/maxArea;
          }
       }
   } else {
     PTDEBUG("didn't find hand contour in this mask!\n");
   }

   return PT_RET_OK;
}

PTF64 _calcLineAngle(const Point& start,const Point& end)
{
    //calculate the angle of two knock point center
    const double dx = abs(start.x - end.x);
    const double dy = abs(start.y - end.y);

    return  atan2(dx,dy+DBL_MIN)*180.0f/CV_PI;
}

static PTS32 _getGesture(Mat& hand, const HandInfo& handInfo, PTHandStatus& handStatus)
{
    PTDEBUG("Enter %s, handInfo.Thumb[%f], handInfo.ratio_hull_handarea[%f]\n", __FUNCTION__, handInfo.Thumb, handInfo.ratio_hull_handarea);

    const int step = 5;
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

    PTDEBUG("finger[%d], handInfo.Thumb[%f], handInfo.ratio_hull_handarea[%f]\n", finger, handInfo.Thumb, handInfo.ratio_hull_handarea);

    //if number of finger is greater than 4,then palm
    if(finger > 4) {
       handStatus = PALM_ON;
    } else {
       //TODO, refine
       if(finger==0 && handInfo.Thumb>0.6) {//if the number of finger is zero and thumb is greater than 0.6,then fist
          handStatus = FIST_ON;
       }

       if(handInfo.ratio_hull_handarea>25 && handInfo.Thumb<0.5) {//ratio_hull_handarea is the the hull ratio
          handStatus = PALM_ON;
       }

       if(handInfo.ratio_hull_handarea<10 && handInfo.Thumb>0.6) {
          handStatus = FIST_ON;
       }

       if(finger==0) {
          handStatus = FIST_ON;
       }

       if(handInfo.Thumb <= 0.45) {
          handStatus = PALM_ON;
       } else {
          handStatus = FIST_ON;
       }
    }

    PTDEBUG("Exit %s ---> handStatus[%d]\n", __FUNCTION__, handStatus);

    return PT_RET_OK;
}

PTS32 _getHandRecognitizeGesture(Mat& handImg, double stdKnockBaseArea, const RoiLocation roiLocation, PTHandStatus& handStatus)
{
    PTDEBUG("Enter %s\n", __FUNCTION__);

    Mat skinMask(handImg.size(), CV_8UC1);

#ifdef ENABLEADPATIVESKINMOLDE
    _mvgetSkinMask(handImg, skinMask, 0.8);
#else
    _getSkinMask(handImg, skinMask);
#endif

#ifdef _SHOW_
    if(roiLocation==ROIUP) {
       imshow("skinMask[Up]", skinMask);
    } else {
       imshow("skinMask[Down]", skinMask);
    }
#endif

    Mat hand;
    HandInfo handInfo = HandInfo(0.0f, 0.0f);
    _getHandInfo(skinMask, hand, handInfo);//Extract the hand using YCrCb color space

    Mat knockMask(handImg.size(), CV_8UC1);
    _getKnockMask(handImg, knockMask, roiLocation);//Extract the knock point using HSV

    double area = -1.0f;
    Point tmp;
    _getContoursAreaCenter(knockMask, area, tmp);//Extract the area and the knock center

    if(!hand.empty() && 0.0f<=area && area/stdKnockBaseArea<0.8) {
       _getGesture(hand, handInfo, handStatus);//recognize fist or palm
    } else {
       PTDEBUG("didn't detected hand\n");
       handStatus = HAND_STATUS_COUNT;
    }

    PTDEBUG("Exit %s ---> handStatus[%d]\n", __FUNCTION__, handStatus);
    return PT_RET_OK;
}

