#include "handGestureRecognitor.h"
#include<opencv2\opencv.hpp>
using namespace std;
using namespace cv;

void usage()
{
  PTDEBUG("./a.out videoname\n");
}

int main(int argc, char *argv[])
{
  if(argc < 2) {
     usage();
     return PT_RET_INVALIDPARAM;
  }

  Mat frame;

//  Mat logoPalm = imread("plam.png");
//  Mat logoFist = imread("fist.png");

  VideoCapture cap;
  cap.open(argv[1]);
  if(!cap.isOpened()) {
     PTDEBUG("VideoCapture is not opened!\n");
     return PT_RET_BADRESOURCE;
  } else {
     PTDEBUG("VideoCapture opened successful!\n");
  }


  HandGestureRecognitor recognitor;

  while(TRUE) {
    cap.read(frame);

    if(frame.empty()) {
       PTDEBUG("Cannot grap the frame!\n");
       break;
    } else {
       imshow("video", frame);
       PTDEBUG("\nGrap the frame successful!\n");
       PTDEBUG("frame.cols[%d], frame.rows[%d]\n", frame.cols, frame.rows);
    }

    if(PT_RET_OK != recognitor.init(frame.data, frame.cols, frame.rows, PT_IMG_BGR888)) {
       PTDEBUG("init failed!\n");
       return -2;
    } else {
       PTDEBUG("init successed!\n");
    }

    //check are tap points palced correct or not
    PTBOOL isTapPointCorrect = FALSE;
    if(!isTapPointCorrect) {
	   recognitor.getKnockPointStatus(isTapPointCorrect);
       PTDEBUG("knock point palced wrong!\n");
       putText(frame, "Knock Point ERROR!", Point(100,20), 2, 1, Scalar(255,255,0));
      // continue;
    } else {
       PTDEBUG("knock point palced right!\n");
       putText(frame, "knock point palced right!", Point(10,240), 2, 1, Scalar(255,0,255));
    }

    //get base area of knock point
    if(!recognitor.mIsTapPointInited) {
       PTDEBUG("this frame used to init knock point area\n");
       recognitor.studyTwoKnockBase();
       continue;
    } else {
       PTDEBUG("knock point area inited finished\n");
    }

    PTHandStatus handStatus = HAND_STATUS_COUNT;
    recognitor.getUpHandGesture(handStatus);
    switch(handStatus) {
           case PALM_ON           :
                putText(frame, "Up Plam Knock ON !", Point(10,40), FONT_HERSHEY_PLAIN, 4, Scalar(0,0,255), 2);
                //logoPalm.copyTo(frame(Rect(10, 60, logoPalm.cols, logoPalm.rows)));
                break;
           case FIST_ON           :
                putText(frame, "Up Fist Knock ON !", Point(10,40), FONT_HERSHEY_PLAIN, 4, Scalar(0,0,255), 2);
                //logoFist.copyTo(frame(Rect(FONT_HERSHEY_PLAIN0, 60, logoFist.cols, logoFist.rows)));
                break;
           case HAND_STATUS_COUNT :
               // putText(frame, "Up Hand didn't Knock ON !", Point(10,40), FONT_HERSHEY_PLAIN, 4, Scalar(0,0,255), 2);
                break;
           default                :
                putText(frame, "Up Hand ERROR !", Point(10,40), FONT_HERSHEY_PLAIN, 4, Scalar(0,0,255), 2);
                break;
    }

    handStatus = HAND_STATUS_COUNT;
    recognitor.getDownHandGesture(handStatus);
    switch(handStatus) {
          case PALM_ON            :
               putText(frame, "Down Plam Knock ON !", Point(20,frame.rows-40), FONT_HERSHEY_PLAIN, 4, Scalar(0,0,255), 2);
               //logoPalm.copyTo(frame(Rect(20, frame.rows-logoPalm.rows-40, logoPalm.cols, logoPalm.rows)));
               break;
          case FIST_ON            :
               putText(frame, "Down Fist Knock ON !", Point(20,frame.rows-40), FONT_HERSHEY_PLAIN, 4, Scalar(0,0,255), 2);
               //logoFist.copyTo(frame(Rect(20, frame.rows-logoFist.rows-40, logoFist.cols, logoFist.rows)));
               break;
          case HAND_STATUS_COUNT  :
               //putText(frame, "Down Hand didn't Knock ON !", Point(20,frame.rows-40), FONT_HERSHEY_PLAIN, 4, Scalar(0,0,255), 2);
               break;
          default                 :
               putText(frame, "Down Hand ERROR !", Point(20,frame.rows-40), FONT_HERSHEY_PLAIN, 4, Scalar(0,0,255), 2);
               break;
    }

    PTDEBUG("show--------+++++++++++++\n");
    imshow("frame", frame);
    waitKey(1);
  }

  return 0;
}
