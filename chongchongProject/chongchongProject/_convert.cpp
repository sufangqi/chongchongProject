/******************************************************************************
 *     Copyright (c) 2015 PuTao Co., Ltd.
 *     All Rights Reserved.
 ******************************************************************************/
#include "basept.h"
#include "_basept.h"

PTS32 _convert(const PTU8* pSrcBuffer, PTU8* pDstBuffer, const int nWidth, const int nHeight, const PTImageFormatEnum eSrcFormat, const PTImageFormatEnum eDstFormat)
{
   if(pSrcBuffer == NULL || pDstBuffer == NULL) {
      PTDEBUG("%s: Invalid parameters, pSrcBuffer[%p], pDstBuffer[%p]\n", __FUNCTION__, pSrcBuffer, pDstBuffer);
      return PT_RET_INVALIDPARAM;
   }

   if(eSrcFormat != PT_IMG_ARGB8888 || eDstFormat != PT_IMG_BGR888) {//only support convert from ARGB8888 to BGR888
      PTDEBUG("%s: Invalid parameters, eSrcFormat[%s], eDstFormat[%s]\n", __FUNCTION__, strImageFormat[eSrcFormat], strImageFormat[eDstFormat]);
      return PT_RET_INVALIDPARAM;
   }

   PTDEBUG("%s: pSrcBuffer[%p], pDstBuffer[%p], eSrcFormat[%s], eDstFormat[%s]\n", __FUNCTION__, pSrcBuffer, pDstBuffer, strImageFormat[eSrcFormat], strImageFormat[eDstFormat]);

   const int nSrcChannel = 4;
   const int nDstChannel = 3;
   const int nPixels = nHeight*nWidth;

   for(int i = 0; i < nPixels; i++) {
//       const int A = pSrcBuffer[0];
       pDstBuffer[2] = pSrcBuffer[1];//R
       pDstBuffer[1] = pSrcBuffer[2];//G
       pDstBuffer[0] = pSrcBuffer[3];//B
       pSrcBuffer += nSrcChannel;
       pDstBuffer += nDstChannel;
   }
 
   return PT_RET_OK;
}

PTS32 _cvtColor2BGR(PTU8* pPixels, PTS32 nWidth, PTS32 nHeight, PTImageFormatEnum eFormat, cv::Mat& bgrMat)
{
    if(pPixels == NULL || nWidth < 0 || nHeight < 0) {
        PTDEBUG("%s: Invalid parameters, pPixels[%p], nWidth[%d], nHeight[%d]\n", __FUNCTION__, pPixels, nWidth, nHeight);
        return PT_RET_INVALIDPARAM;
    }
    
    if(eFormat != PT_IMG_BGR888 && eFormat != PT_IMG_RGBA8888 && eFormat != PT_IMG_ARGB8888 && eFormat != PT_IMG_BGRA8888) {
        PTDEBUG("%s: picture format[%d], ie[%s] not supported!\n",__FUNCTION__, (int)eFormat, strImageFormat[eFormat]);
        return PT_RET_INVALIDPARAM;
    }
    
    bgrMat.release();
    
    PTU8* pBuffer = NULL;
    if(eFormat == PT_IMG_ARGB8888) {
        pBuffer = (PTU8*) malloc(3*nWidth*nHeight);//buffer for BGR
        if(pBuffer == NULL) {
            PTDEBUG("%s: no enough memory to support format[%s]\n", __FUNCTION__, strImageFormat[eFormat]);
            return PT_RET_NOMEM;
        }
    }
    
    switch(eFormat) {
        case PT_IMG_BGR888   :{
            bgrMat = cv::Mat(nHeight, nWidth, CV_8UC3, pPixels);
        }
            break;
        case PT_IMG_RGBA8888 :{
            cvtColor(cv::Mat(nHeight, nWidth, CV_8UC4, pPixels), bgrMat, CV_RGBA2BGR);
        }
            break;
        case PT_IMG_ARGB8888 :{
            const int nSrcChannel = 4;
            const int nDstChannel = 3;
            const int nPixels = nHeight*nWidth;
            PTU8* pSrcBuffer = pPixels;
            PTU8* pDstBuffer = pBuffer;
            for(int i = 0; i < nPixels; i++) {
                pDstBuffer[2] = pSrcBuffer[1];//R
                pDstBuffer[1] = pSrcBuffer[2];//G
                pDstBuffer[0] = pSrcBuffer[3];//B
                pSrcBuffer += nSrcChannel;
                pDstBuffer += nDstChannel;
            }
            bgrMat = cv::Mat(nHeight, nWidth, CV_8UC3, pBuffer);
        }
            break;
        case PT_IMG_BGRA8888 :{
            cvtColor(cv::Mat(nHeight, nWidth, CV_8UC4, pPixels), bgrMat, CV_BGRA2BGR);
        }
            break;
        default :{
            PTDEBUG("%s: picture format[%d], ie[%s] not supported!\n", __FUNCTION__, (int)eFormat, strImageFormat[eFormat]);
            return PT_RET_INVALIDPARAM;
        }
            break;
    }
    return PT_RET_OK;
}

//TODO:need add other platforms later
PTS32 _rotateImg(const cv::Mat srcMat, PTBOOL needRotate, cv::Mat& rotatedMat)
{
    if (srcMat.empty() || srcMat.cols < 0 || srcMat.rows < 0 ) {
        PTDEBUG("%s: Invalid parameters, pPixels[%p], nWidth[%d], nHeight[%d]\n", __FUNCTION__, srcMat.data, srcMat.cols, srcMat.rows);
        return  PT_RET_INVALIDPARAM;
    }
    
    PTS32 angle = 0;
    if(needRotate == TRUE){
#if defined(__APPLE__)
        angle = 90;
#endif
    }
    rotatedMat.release();
    PTS32 transposeTime = angle%360;
    if (srcMat.cols <= srcMat.rows
        || transposeTime == 0) {
#if defined(MACOS_X) || defined(__APPLE__)
        PTDEBUG("%s:no need to transpose\n", __FUNCTION__);
        flip(srcMat, rotatedMat, 1);
        return  PT_RET_OK;
#endif
    }
    
    transposeTime = transposeTime/90;
    if (transposeTime == 2) {
        srcMat.copyTo(rotatedMat);
    }
    else{
        transpose(srcMat, rotatedMat);
    }
    
    if (transposeTime == 3) {
        flip(rotatedMat, rotatedMat, 1);
    }
    else if(transposeTime == 2){
        flip(rotatedMat, rotatedMat, -1);
    }
    
    return  PT_RET_OK;
}

