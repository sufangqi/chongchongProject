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
#define INIT_FRAME_NUM               1

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
   _HandInfo() {HandDefectsNum = 0.0f; Thumb = 0.0f;}
   _HandInfo(int _HandDefectsNum, int _Thumb) {HandDefectsNum = _HandDefectsNum; Thumb = _Thumb;}

   int HandDefectsNum;
   int Thumb;
} HandInfo;

//return the recognized result
PTS32 _getHandRecognitizeGesture(Mat& handImg, double& stdKnockBaseArea, PTSysEnum& eBoard,int HandType, PTHandStatus& handStatus,int &KnockNumber);


//extract the knock base
PTS32 _getKnockBase(Mat& srcImg, double& area, PTBOOL& isFinish,PTSysEnum& eBoard);

//skin model detection
PTS32 _mvSkinDetectByPoint(Mat& srcImg, Mat& dstImg, double threshold);

//detect knock point
PTS32 _getKnockMask(Mat& srcImg, Mat& dstImg,PTSysEnum&eBoard);

//extract the center of Contour
PTS32 _extractContourInfo(const Mat& skinArea, Point& cogPt);

//extract the area and center of knock point
PTS32 _getMaxContoursAreaCenter(Mat& srcImg, double& area, Point& center,vector<Point>& MaxContour);

//calculate the angle of two point
PTF64 _calcLineYAngle(const Point& start, const Point& end);

PTS32 parseSystemInfo(const char* const pSystemInfo, PTSysEnum& eBoard);

#endif //!__UTILS_

