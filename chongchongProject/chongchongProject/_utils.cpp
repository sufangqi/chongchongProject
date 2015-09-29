#include "_utils.h"
#include "handGestureRecognitor.h"
#define _SHOW_
//#define ENABLEADPATIVESKINMOLDE
//#define USEOTSU
#define USE_SKIN_COLOR_DIFF
#ifdef _SHOW_
extern Mat dispImg;
unsigned int gHandID;
#endif
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
   //{90, 140,/*hue*/ 50, 200,/*sat*/ 0, 256,} ,/*value*/ //up point, blue
    {37, 75,/*hue*/  50, 200,/*sat*/ 0, 256,} ,/*value*/ //down point, green
    {37, 75,/*hue*/  50, 200,/*sat*/ 0, 256,} ,/*value*/ //down point, green
};

static vector<int> vKnockArea;
PTS32 _greyWorldCC(cv::Mat& srcImg)
{
   PTDEBUG("Enter %s\n", __FUNCTION__);

#ifdef _SHOW_
   imshow("before grey cc", srcImg);
#endif

   double whiteB = 0.0f;
   double whiteG = 0.0f;
   double whiteR = 0.0f;

   const int channels = srcImg.channels();
   int rows = srcImg.rows;
   int cols = srcImg.cols*channels;

   if(srcImg.isContinuous()) {
      cols = rows*cols;
      rows = 1;
   }

   for(int i = 0; i < rows; i++) {
     uchar *pRow = srcImg.ptr<uchar>(i);
     for(int j = 0; j < cols; j+=channels) {
         whiteB += pRow[j];
         whiteG += pRow[j+1];
         whiteR += pRow[j+2];
     }
   }


   double sum = sqrt(whiteB*whiteB + whiteG*whiteG + whiteR*whiteR);

   whiteB = whiteB/sum;
   whiteG = whiteG/sum;
   whiteR = whiteR/sum;

   PTDEBUG("white point(%f, %f, %f)\n", whiteR, whiteG, whiteB);

   whiteB = whiteB*sqrt(3.0f);
   whiteG = whiteG*sqrt(3.0f);
   whiteR = whiteR*sqrt(3.0f);

   for(int i = 0; i < rows; i++) {
     uchar *pRow = srcImg.ptr<uchar>(i);
     for(int j = 0; j < cols; j+=channels) {
         pRow[j]   = cv::saturate_cast<uchar>(pRow[j]/whiteB);
         pRow[j+1] = cv::saturate_cast<uchar>(pRow[j+1]/whiteG);
         pRow[j+2] = cv::saturate_cast<uchar>(pRow[j+2]/whiteR);
     }
   }

#ifdef _SHOW_
   imshow("after grey cc", srcImg);
#endif

   return PT_RET_OK;
}
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
PTS32  cvThresholdOtsu(Mat&src)  
{  
    int width = src.cols;  
	int height = src.rows;  
  
    //histogram  
    float histogram[256] = {0};  
    for(int i=0;i < height;i++) {  
		unsigned char* p=(unsigned char*)src.data+src.step[0]*i;  
		//	widthStep*i;  
        for(int j = 0;j < width;j++) {  
            histogram[*p++]++;  
        }  
    }  


    //normalize histogram  
    int size = height*width;  
    for(int i = 0;i< 256;i++) {  
        histogram[i] = histogram[i] / size;  
    }  
    //average pixel value
    float avgValue = 0;  
    for(int i = 0;i < 256;i++) {  
        avgValue+= i * histogram[i];  
    }  


    int thre;    
    float maxVariance = 0;  
    float w = 0,u = 0;  
    for(int i = 0;i < 256; i++) {  
        w+= histogram[i];  
        u+= i*histogram[i];  
  
        float t = avgValue*w-u;  
        float variance = t*t/(w*(1-w));  
        if(variance > maxVariance) {  
            maxVariance = variance;  
            thre = i;  
        }  
    } 
     if(thre < 0)
		 thre = 0;
	threshold(src,src,thre,255,CV_THRESH_BINARY);
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
	cvThresholdOtsu(dstImg);
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
static PTS32 SkinDetectionBasedOnColorDiff(Mat& SrcImg,Mat &DstImg)
{
	if(SrcImg.channels() != 3){
	    PTDEBUG("Error in SkinDetectionBasedOnColorDiff .The Input Image's channel must be 3");
		 return PT_RET_INVALIDPARAM;
	}
	    vector<Mat> YCrCb;
		Mat YcbcrImg;
	
		cvtColor(SrcImg,YcbcrImg,CV_BGR2YCrCb);
		split(YcbcrImg,YCrCb);
		Mat imgCr=YCrCb[1];
		Mat imgCb=YCrCb[2];

		Mat Dst(imgCr.size(),CV_32F);
        int rows = imgCr.rows;
		int cols = imgCr.cols*imgCr.channels();
		const int channels=imgCr.channels();
	
		if(imgCr.isContinuous() && Dst.isContinuous()){
			cols=cols*rows;
			rows = 1;
		}

		for(int i = 0;i < rows;i++){
		   uchar *pCr = imgCr.ptr<uchar>(i);
		   uchar *pCb = imgCb.ptr<uchar>(i);
		   float *pDst = Dst.ptr<float>(i);
		   for(int j = 0;j < cols; j++){
			   if(pCr[j] - pCb[j] < 17)
				   pDst[j] = 0;
			   else
				   pDst[j] = pCr[j] - pCb[j];
		   }
		}
		normalize(Dst,Dst,255,0,NORM_MINMAX);

		
		Dst.convertTo(DstImg,CV_8UC1);
		cvThresholdOtsu(DstImg);
		dilate(DstImg,DstImg,Mat());
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

    PTDEBUG("roiLocation[%d]->[%s], hueMin[%d], hueMax[%d], satMin[%d], satMax[%d], valMin[%d], valMax[%d]\n",
            (int)roiLocation, strROILocation[roiLocation], hueMin, hueMax, satMin, satMax, valMin, valMax);

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

       Rect handRect = boundingRect(maxContour);
       const double perimeter = arcLength(maxContour, true);

       RotatedRect box = minAreaRect(maxContour);
       Point2f vertex[4];
       box.points(vertex);
       const double width  = pow((vertex[1].x-vertex[2].x), 2.0f) + pow((vertex[1].y-vertex[2].y), 2.0f);
       const double height = pow((vertex[1].x-vertex[0].x), 2.0f) + pow((vertex[1].y-vertex[0].y), 2.0f);
       const double widthHeightRatio = width/height;

       Point handCenter;
       _getContourCenter(maxContour, handCenter);
      // const Point center_gravy = Point(handCenter.x+handRect.x, handCenter.y+handRect.y);//TODO think again

	   hand = gray(Rect(handRect.x, handRect.y, handRect.width*3/5, handRect.height));
     

#ifdef _SHOW_
       //Mat draw(skinArea.size(), CV_8UC3);
       //cvtColor(skinArea, draw, CV_GRAY2BGR);

       //circle(draw, handCenter, 4, CV_RGB(0,255,0));
       //circle(draw, center_gravy, 4, CV_RGB(255,0,0));

       //Mat topRightUp;
       //if(gHandID%2 == 0) {
       //   //up hand
       //  // topRightUp = dispImg(Rect(dispImg.cols/2+skinArea.cols*3, skinArea.rows, skinArea.cols, skinArea.rows));
       //} else {
       //   //down hand
       //   //topRightUp = dispImg(Rect(dispImg.cols/2+skinArea.cols*3, 0, skinArea.cols, skinArea.rows));
       //}

       //resize(draw, topRightUp, skinArea.size());
#endif

      
#ifdef _SHOW_
     //  draw = Mat::zeros(skinArea.size(),CV_8UC3);
     //  vector<vector<Point>> contours;
       //only one contour
      // contours.push_back(hullContour);
       //if(gHandID%2 == 0) {
       //   //up hand
       //   drawContours(draw, contours, -1/*all*/, CV_RGB(0,255,0), 2);
       //   topRightUp = dispImg(Rect(dispImg.cols/2, skinArea.rows*3, skinArea.cols, skinArea.rows));
       //} else {
       //   //down hand
       //   drawContours(draw, contours, -1/*all*/, CV_RGB(0,0,255), 2);
       //   topRightUp = dispImg(Rect(dispImg.cols/2, skinArea.rows*2, skinArea.cols, skinArea.rows));
       //}
       //gHandID++;

       //resize(draw, topRightUp, skinArea.size());
#endif

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

              const double depth = defects[i][3]/sqrt(width); // distance between the farthest point and the convex hull
              //The thumb area constraints,TODO
              //if(depth>20.0f && (ptFar.x>ptEnd.x||ptFar.x>ptStart.x) && (ptFar.y-handRect.y)>50 && (skinArea.cols-ptEnd.x)>10) {
              if(depth>15.0f && (ptFar.x>ptEnd.x||ptFar.x>ptStart.x) && (ptFar.y-handRect.y)>30 && (skinArea.cols-ptEnd.x)>10) {
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

 #ifdef _SHOW_
          static int handID = 0;
          Mat show(skinArea.size(), CV_8UC3);
          cvtColor(skinArea, show, CV_GRAY2BGR);

          circle(show, handCenter, 4, CV_RGB(0,255,0));
		  circle( show, thumbContour[0],   4, Scalar(255,0,100), 2 );  
		  circle( show, thumbContour[1],   4, Scalar(255,0,100), 2 );  
		  circle( show, thumbContour[2],   4, Scalar(100,0,255), 2 );
		  rectangle(show,rect,Scalar(255),2);
          imshow(" thumb", show);
	   // cvWaitKey(0);
       handID++;
#endif
	         double LengthOfHandcenterAndthumb = pow(thumbContour[0].x-handCenter.x,2)+pow(thumbContour[0].y-handCenter.y,2);
	         double LengthOfHandcenterAndthumbFar = pow(thumbContour[2].x-handCenter.x,2);
			// cout<<LengthOfHandcenterAndthumb/LengthOfHandcenterAndthumbFar<<"rect.height / rect.width "<<(double)rect.height / rect.width<<endl;
             // if the handCenter is in the right of thumb rect origin
			 if( (double)rect.height / rect.width < 1.2 || LengthOfHandcenterAndthumb/LengthOfHandcenterAndthumbFar >10) {
				handInfo.Thumb = 0.0f;
			  } else {
				  handInfo.Thumb = 1.0f;
			         } 
          } else {
            handInfo.Thumb = 1.0f;
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
    PTDEBUG("Enter %s, handInfo.Thumb[%f], handInfo.ratio_hull_handarea[%f]\n", __FUNCTION__, handInfo.Thumb, handInfo.ratio_hull_handarea);

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

    PTDEBUG("finger[%d], handInfo.Thumb[%f], handInfo.ratio_hull_handarea[%f]\n", finger, handInfo.Thumb, handInfo.ratio_hull_handarea);
	//printf_s("finger[%d], handInfo.Thumb[%f], handInfo.ratio_hull_handarea[%f]\n", finger, handInfo.Thumb, handInfo.ratio_hull_handarea);
    //if number of finger is greater than 4,then palm
    if(finger > 3) {
       handStatus = PALM_ON;
    } else {
       //TODO, refine
       if(finger==0 && handInfo.Thumb == 1) {//if the number of finger is zero and thumb is greater than 0.6,then fist
          handStatus = FIST_ON;
       }else if(handInfo.Thumb == 0)
		   handStatus = PALM_ON;
	   else
		   handStatus = FIST_ON;

    }

    PTDEBUG("Exit %s ---> handStatus[%d]\n", __FUNCTION__, handStatus);

    return PT_RET_OK;
}

PTS32 _getHandRecognitizeGestureUp(Mat& handImg, double stdKnockBaseArea, const RoiLocation roiLocation, PTHandStatus& handStatus,int& KnockNumber)
{
#ifdef _SHOW_
    gHandID++;
#endif

    PTDEBUG("Enter %s\n", __FUNCTION__);

    Mat skinMask(handImg.size(), CV_8UC1);

#ifdef ENABLEADPATIVESKINMOLDE
    _mvgetSkinMask(handImg, skinMask, 0.75);
#else
#ifdef USE_SKIN_COLOR_DIFF
	SkinDetectionBasedOnColorDiff(handImg,skinMask);
#else
    _getSkinMask(handImg, skinMask);
#endif
#endif

    Mat hand;
    HandInfo handInfo = HandInfo(0.0f, 0.0f);
    _getHandInfo(skinMask, hand, handInfo);//Extract the hand using YCrCb color space
    Mat knockMask(handImg.size(), CV_8UC1);
    _getKnockMask(handImg, knockMask, roiLocation);//Extract the knock point using HSV
    double area = 0.0f;
    Point tmp;
    _getMaxContoursAreaCenter(knockMask, area, tmp);//Extract the area and the knock center

	static Mat MaskOfKnockStaticUp ;
	static bool StartToDetectKnockEnd = false;
	static int NumberOfKnock = 0;
	KnockNumber = -1;
	if(hand.empty() || area/stdKnockBaseArea>0.8) {
		stdKnockBaseArea = area;
		PTDEBUG("didn't detected hand\n");
		//std::cout<<StartToDetectKnockEnd<<" "<<handStatus <<endl;
		MaskOfKnockStaticUp = knockMask;
		if(StartToDetectKnockEnd){
        	StartToDetectKnockEnd = false;
			handStatus = HAND_KNOCK_END;
			NumberOfKnock++;
		} else{
		handStatus = HAND_STATUS_COUNT;
		}
	} else {
		PTDEBUG("detected hand, next step is recognize it's gesture...");

		Mat SkinAboveOnKnock;
		int SumOfSkinPixel = 0;
		skinMask.copyTo(SkinAboveOnKnock,MaskOfKnockStaticUp);
		for(int i = 0 ;i <skinMask.rows;i++){
			uchar *pData = SkinAboveOnKnock.ptr<uchar>(i);
			for(int j =0; j<skinMask.cols;j++){
				if(pData[j] == 255){
					SumOfSkinPixel++;
				}
			}
		}
#ifdef _SHOW_
		imshow("SKINABOVE",SkinAboveOnKnock);
#endif
		//cout<<"SkinPixel:"<<SumOfSkinPixel / stdKnockBaseArea<<endl;
		PTDEBUG("SumOfSkinPixel[%f]\n", SumOfSkinPixel / stdKnockBaseArea);
		if(SumOfSkinPixel / stdKnockBaseArea >0.3){
			_getGesture(hand, handInfo, handStatus);//recognize fist or palm
			StartToDetectKnockEnd = true;
			KnockNumber = NumberOfKnock;
		} 
	}
#ifdef _SHOW_
	imshow("KnockMask",knockMask);
	imshow("SkinMask",skinMask);
	if(!hand.empty()){
	  imshow("Hand",hand);
	}
#endif
    PTDEBUG("Exit %s ---> handStatus[%s]\n", __FUNCTION__, strHandGesture[handStatus]);
    return PT_RET_OK;
}
PTS32 _getHandRecognitizeGestureDown(Mat& handImg, double stdKnockBaseArea, const RoiLocation roiLocation, PTHandStatus& handStatus,int& KnockNumber)
{
#ifdef _SHOW_
    gHandID++;
#endif

    PTDEBUG("Enter %s\n", __FUNCTION__);

    Mat skinMask(handImg.size(), CV_8UC1);

#ifdef ENABLEADPATIVESKINMOLDE
    _mvgetSkinMask(handImg, skinMask, 0.75);
#else
#ifdef USE_SKIN_COLOR_DIFF+
	SkinDetectionBasedOnColorDiff(handImg,skinMask);
#else
    _getSkinMask(handImg, skinMask);
#endif
#endif

    Mat hand;
    HandInfo handInfo = HandInfo(0.0f, 0.0f);
    _getHandInfo(skinMask, hand, handInfo);//Extract the hand using YCrCb color space
    Mat knockMask(handImg.size(), CV_8UC1);
    _getKnockMask(handImg, knockMask, roiLocation);//Extract the knock point using HSV
    double area = 0.0f;
    Point tmp;
    _getMaxContoursAreaCenter(knockMask, area, tmp);//Extract the area and the knock center

	static Mat MaskOfKnockStaticDown ;
	static bool StartToDetectKnockEnd = false;
	static int NumberOfKnock = 0;
	KnockNumber = -1;
	if(hand.empty() || area/stdKnockBaseArea>0.8) {
		stdKnockBaseArea = area;
		PTDEBUG("didn't detected hand\n");
		MaskOfKnockStaticDown = knockMask;
		if( StartToDetectKnockEnd ){
        	StartToDetectKnockEnd = false;
			handStatus = HAND_KNOCK_END;
			NumberOfKnock++;
		} else{
		handStatus = HAND_STATUS_COUNT;
		}
	} else {
		PTDEBUG("detected hand, next step is recognize it's gesture...");

		Mat SkinAboveOnKnock;
		int SumOfSkinPixel = 0;
		skinMask.copyTo(SkinAboveOnKnock,MaskOfKnockStaticDown);
		for(int i = 0 ;i <skinMask.rows;i++){
			uchar *pData = SkinAboveOnKnock.ptr<uchar>(i);
			for(int j =0; j<skinMask.cols;j++){
				if(pData[j] == 255){
					SumOfSkinPixel++;
				}
			}
		}
#ifdef _SHOW_
		imshow("SKINABOVE",SkinAboveOnKnock);
#endif
		//cout<<"SkinPixel:"<<SumOfSkinPixel / stdKnockBaseArea<<endl;
		PTDEBUG("SumOfSkinPixel[%f]\n", SumOfSkinPixel / stdKnockBaseArea);
		if(SumOfSkinPixel / stdKnockBaseArea >0.3){
			_getGesture(hand, handInfo, handStatus);//recognize fist or 
			StartToDetectKnockEnd = true;
			KnockNumber = NumberOfKnock;
		} 
	}
#ifdef _SHOW_
	imshow("KnockMask",knockMask);
	imshow("SkinMask",skinMask);
	if(!hand.empty()){
	  imshow("Hand",hand);
	}
#endif
    PTDEBUG("Exit %s ---> handStatus[%s]\n", __FUNCTION__, strHandGesture[handStatus]);
    return PT_RET_OK;
}

