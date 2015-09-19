/******************************************************************************
 *     Copyright (c) 2015 PuTao Co., Ltd.
 *     All Rights Reserved.
 ******************************************************************************/
#include "basept.h"
#include "_basept.h"

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
