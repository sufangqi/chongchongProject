#include "basept.h"
#include "handGestureRecognitor.h"
#include "_basept.h"

#define _SHOW_
using namespace std;
using namespace cv;

#ifdef _SHOW_
 Mat dispImg;
#endif

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
  Mat frame;

  PTBOOL isTapPointCorrect = FALSE;

  while(TRUE) {
   // waitKey(0);
    cap.read(frame);

    if(frame.empty()) {
       PTDEBUG("Cannot grap the frame!\n");
       break;
    } else {
       PTDEBUG("\nGrap the frame successful!\n");
       PTDEBUG("frame.cols[%d], frame.rows[%d]\n", frame.cols, frame.rows);
    }

    if(PT_RET_OK != recognitor.init(frame.data, frame.cols, frame.rows, PT_IMG_BGR888)) {
       PTDEBUG("init failed!\n");
       return -2;
    } else {
       PTDEBUG("init successed!\n");
    }

    const double fontScale = 2.0f;

    //check are tap points palced correct or not
    if(!isTapPointCorrect) {
       recognitor.getKnockPointStatus(isTapPointCorrect);
       PTDEBUG("knock point palced wrong!\n");
#ifdef _SHOW_
	   dispImg = frame;
       putText(dispImg, "Knock Point ERROR!", Point(100,20), FONT_HERSHEY_PLAIN, fontScale, CV_RGB(255,0,0), 2);
#endif
//       continue;
    } else {
       PTDEBUG("knock point palced right!\n");
#ifdef _SHOW_
       putText(dispImg, "knock point palced right!", Point(10,dispImg.rows/4), FONT_HERSHEY_PLAIN, fontScale, CV_RGB(0,255,0), 2);
#endif
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
	int KnockNumberUp = -1;
    recognitor.getUpHandGesture(handStatus,KnockNumberUp);
    PTDEBUG("recognized UP hand gesture[%s]\n", strHandGesture[handStatus]);
	PTDEBUG("UP hand Number[%d]\n",KnockNumberUp);
#ifdef _SHOW_
    switch(handStatus) {
           case PALM_ON           :
                putText(dispImg, "Up Plam Knock ON !", Point(10,40), FONT_HERSHEY_PLAIN, fontScale, Scalar(0,0,255), 2);
                //logoPalm.copyTo(frame(Rect(10, 60, logoPalm.cols, logoPalm.rows)));
                break;
           case FIST_ON           :
                putText(dispImg, "Up Fist Knock ON !", Point(10,40), FONT_HERSHEY_PLAIN, fontScale, Scalar(0,0,255), 2);
                //logoFist.copyTo(frame(Rect(FONT_HERSHEY_PLAIN0, 60, logoFist.cols, logoFist.rows)));
                break;
           case HAND_STATUS_COUNT :
                putText(dispImg, "Up Hand didn't Knock ON !", Point(10,40), FONT_HERSHEY_PLAIN, fontScale, Scalar(0,0,255), 2);
                break;
		   case HAND_KNOCK_END:
                putText(dispImg, "UP KNOCK ON END!", Point(10,40), FONT_HERSHEY_PLAIN, fontScale, CV_RGB(0,0,255), 2);
                break;
           default                :
                putText(dispImg, "Up Hand ERROR !", Point(10,40), FONT_HERSHEY_PLAIN, fontScale, Scalar(0,0,255), 2);
                break;
    }
#endif

    handStatus = HAND_STATUS_COUNT;
	int KnockNumberDown = -1;
    recognitor.getDownHandGesture(handStatus,KnockNumberDown);
    PTDEBUG("recognized DOWN hand gesture[%s]\n", strHandGesture[handStatus]);
	PTDEBUG("DOWN hand Number[%d]\n", KnockNumberDown);
#ifdef _SHOW_
    switch(handStatus) {
          case PALM_ON            :
               putText(dispImg, "Down Plam Knock ON !", Point(20,dispImg.rows/2-20), FONT_HERSHEY_PLAIN, fontScale, CV_RGB(0,0,255), 2);
               //logoPalm.copyTo(dispImg(Rect(20, dispImg.rows-logoPalm.rows-40, logoPalm.cols, logoPalm.rows)));
               break;
          case FIST_ON            :
               putText(dispImg, "Down Fist Knock ON !", Point(20,dispImg.rows/2-20), FONT_HERSHEY_PLAIN, fontScale, CV_RGB(0,0,255), 2);
               //logoFist.copyTo(dispImg(Rect(20, dispImg.rows-logoFist.rows-40, logoFist.cols, logoFist.rows)));
               break;
          case HAND_STATUS_COUNT  :
               putText(dispImg, "Down Hand didn't Knock ON !", Point(20,dispImg.rows/2-20), FONT_HERSHEY_PLAIN, fontScale, CV_RGB(0,0,255), 2);
               break;
		 case HAND_KNOCK_END:
               putText(dispImg, "DOWN KNOCK ON END", Point(20,dispImg.rows/2-20), FONT_HERSHEY_PLAIN, fontScale, Scalar(0,0,255), 2);
               break;
          default                 :
               putText(dispImg, "Down Hand ERROR !", Point(20,dispImg.rows/2-20), FONT_HERSHEY_PLAIN, fontScale, CV_RGB(0,0,255), 2);
               break;
    }
	cout<<"The Hand Knock Number : Up "<<KnockNumberUp <<" Down "<<KnockNumberDown<<endl;
    imshow("dispImg", dispImg);
    waitKey(0);
#endif
    PTDEBUG("show--------+++++++++++++\n");
  }

  return 0;
}
