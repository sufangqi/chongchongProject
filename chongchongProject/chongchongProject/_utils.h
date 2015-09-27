#ifndef __UTILS_
#define __UTILS_

#include "basept.h"
#include "_basept.h"

using namespace cv;
using namespace std;

#define CR_SKIN_MIN                  142
#define CR_SKIN_MAX                  255
#define CB_SKIN_MIN                  150
#define CB_SKIN_MAX                  200

//the frame number of decide the base
#define INIT_FRAME_NUM               60

//#define ENABLEADPATIVESKINMOLDE      1

typedef enum _RoiLocation {
   ROIUP   = 0,
   ROIDOWN = 1,
   ROICNT,
} RoiLocation;

const char strROILocation[ROICNT+1][MAX_STRING_LENGTH] = {
  "ROIUP", "ROIDOWN", "NULL",
};

typedef class _HandInfo
{
public:
   _HandInfo() {ratio_hull_handarea = 0.0f; Thumb = 0.0f;}
   _HandInfo(double _ratio_hull_handarea, double _Thumb) {ratio_hull_handarea = _ratio_hull_handarea; Thumb = _Thumb;}

   double ratio_hull_handarea;
   double Thumb;
} HandInfo;

//return the recognized result
PTS32 _getHandRecognitizeGestureUp(Mat& handImg, double stdKnockBaseArea, const RoiLocation roiLocation, PTHandStatus& handStatus,bool IsLongTimeKnock = true);
PTS32 _getHandRecognitizeGestureDown(Mat& handImg, double stdKnockBaseArea, const RoiLocation roiLocation, PTHandStatus& handStatus,bool IsLongTimeKnock = true);

//extract the knock base
PTS32 _getKnockBase(Mat& srcImg, const RoiLocation roiLocation, double& area, PTBOOL& isFinish);

//skin model detection
PTS32 _mvSkinDetectByPoint(Mat& srcImg, Mat& dstImg, double threshold);

//detect knock point
PTS32 _getKnockMask(Mat& srcImg, Mat& dstImg, const RoiLocation roiLocation);

//extract the center of Contour
PTS32 _extractContourInfo(const Mat& skinArea, Point& cogPt);

//extract the area and center of knock point
PTS32 _getMaxContoursAreaCenter(Mat& srcImg, double& area, Point& center);

//calculate the angle of two point
PTF64 _calcLineYAngle(const Point& start, const Point& end);

PTS32 _greyWorldCC(cv::Mat& srcImg);

#endif //!__UTILS_

