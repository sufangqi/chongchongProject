#ifndef _HANDGESTURERECOGNITOR_
#define _HANDGESTURERECOGNITOR_

#include "basept.h"

class HandGestureRecognitor {
public:
    HandGestureRecognitor();
    ~HandGestureRecognitor();

    PTS32 init(PTU8* pPixels, PTS32 nWidth, PTS32 nHeight, PTImageFormatEnum eFormat);
    //check wether the knock point is put corect
    PTS32 getKnockPointStatus(PTBOOL& isTapPointCorrect);
    PTS32 studyTwoKnockBase(void);
    PTS32 getUpHandGesture(PTHandStatus& handStatus,bool IsLongTimeKnock = true);
    PTS32 getDownHandGesture(PTHandStatus& handStatus,bool IsLongTimeKnock = true);
    
    //BOOL check_knock_point;
    PTBOOL mIsUpTapPointInited;
    PTBOOL mIsDownTapPointInited;
    PTBOOL mIsTapPointInited;

private:
//    Mat gUpHandImg;
//    Mat gDownHandImg;

    double mStdUpPointArea;
    double mStdDownPointArea;
};

#endif //!_HANDGESTURERECOGNITOR_

