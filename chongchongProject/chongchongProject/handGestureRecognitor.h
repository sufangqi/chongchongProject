#ifndef _HANDGESTURERECOGNITOR_
#define _HANDGESTURERECOGNITOR_

#include "basept.h"

#include <stdio.h>
#include <stdlib.h>
#include <cstring>
#include <iostream>

class HandGestureRecognitor {
public:
    HandGestureRecognitor();
    ~HandGestureRecognitor();

    PTS32 init(PTU8* pPixels, PTS32 nWidth, PTS32 nHeight, PTImageFormatEnum eFormat, const char* const pSystemInfo);
    //check wether the knock point is put corect
    PTS32 getKnockPointStatus(PTBOOL& isTapPointCorrect);
    PTS32 studyTwoKnockBase(void);
    PTS32 getUpHandGesture(PTHandStatus& handStatus, int &KnockNumber);
    PTS32 getDownHandGesture(PTHandStatus& handStatus, int &KnockNumber);
    
    PTBOOL mIsUpTapPointInited;
    PTBOOL mIsDownTapPointInited;
    PTBOOL mIsCheckTapPoint;

private:
    double mStdUpPointArea;
    double mStdDownPointArea;
};

#endif //!_HANDGESTURERECOGNITOR_

