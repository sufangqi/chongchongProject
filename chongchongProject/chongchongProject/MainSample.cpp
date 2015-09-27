#include "basept.h"
#include "handGestureRecognitor.h"
#include "_basept.h"
#include<opencv2\opencv.hpp>
using namespace std;
using namespace cv;
#define _SHOW_
//#define _READ_IMAGE_
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
  int imageNum=164;
  cap.open(argv[1]);
  if(!cap.isOpened()) {
     PTDEBUG("VideoCapture is not opened!\n");
     return PT_RET_BADRESOURCE;
  } else {
     PTDEBUG("VideoCapture opened successful!\n");
  }
  HandGestureRecognitor recognitor;
  Mat frame;
 PTBOOL isTapPointCorrect = FALSE;//必须放在最外面
  while(TRUE) {
   // waitKey(0);
#ifndef  _READ_IMAGE_
    cap.read(frame);
#endif
#ifdef _READ_IMAGE_
	char image_name[20]={""};
	sprintf_s(image_name,"img_0%d.jpg",imageNum++);
	string image_name_string=image_name;
	frame=imread("D:\\facedata\\阿里巴巴开放性项目\\实验视频19\\照片2\\"+image_name_string);
	
#endif
    if(frame.empty()) {
       PTDEBUG("Cannot grap the frame!\n");
       break;
    } else {
       PTDEBUG("\nGrap the frame successful!\n");
       PTDEBUG("frame.cols[%d], frame.rows[%d]\n", frame.cols, frame.rows);
    }
#ifdef _READ_IMAGE_//reverse clockwise roate the image 90 degree
	transpose(frame,frame);
    flip(frame,frame,0);
#endif
    if(PT_RET_OK != recognitor.init(frame.data, frame.cols, frame.rows, PT_IMG_BGR888)) {
       PTDEBUG("init failed!\n");
       return -2;
    } else {
       PTDEBUG("init successed!\n");
    }

    const double fontScale = 2.0f;

    //check are tap points palced correct or not
#ifdef _SHOW_
		  	dispImg=frame;
#endif
    if(!isTapPointCorrect) {
       recognitor.getKnockPointStatus(isTapPointCorrect);
       PTDEBUG("knock point palced wrong!\n");
#ifdef _SHOW_
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
    recognitor.getUpHandGesture(handStatus);
    PTDEBUG("recognized UP hand gesture[%s]\n", strHandGesture[handStatus]);
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
               // putText(dispImg, "Up Hand didn't Knock ON !", Point(10,40), FONT_HERSHEY_PLAIN, fontScale, Scalar(0,0,255), 2);
                break;
           default                :
                putText(dispImg, "Up Hand ERROR !", Point(10,40), FONT_HERSHEY_PLAIN, fontScale, Scalar(0,0,255), 2);
                break;
    }
#endif
    handStatus = HAND_STATUS_COUNT;
    recognitor.getDownHandGesture(handStatus);
    PTDEBUG("recognized DOWN hand gesture[%s]\n", strHandGesture[handStatus]);
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
             //  putText(dispImg, "Down Hand didn't Knock ON !", Point(20,dispImg.rows/2-20), FONT_HERSHEY_PLAIN, fontScale, CV_RGB(0,0,255), 2);
               break;
          default                 :
               putText(dispImg, "Down Hand ERROR !", Point(20,dispImg.rows/2-20), FONT_HERSHEY_PLAIN, fontScale, CV_RGB(0,0,255), 2);
               break;
    }
	resize(dispImg,dispImg,Size(640,480));
    imshow("dispImg", dispImg);
	//imwrite("D:\\facedata\\Personal_Data\\facedata\\1080x960\\Result\\"+image_name_string,dispImg);
   waitKey(0);
#endif
    PTDEBUG("show--------+++++++++++++\n");
  }

  return 0;
}
