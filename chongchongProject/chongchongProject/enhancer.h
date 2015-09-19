//
//  enhancer.h
//  ImageProcessor
//
//  Created by putao on 15/5/12.
//  Copyright (c) 2015年 putao. All rights reserved.
//

#ifndef __ENHANCER_H__
#define __ENHANCER_H__

#include "basept.h"


typedef enum _EnhanceLevel
{
    PT_EL_NONE = -1,
    PT_EL_LOW = 0,
    PT_EL_MEDIUM = 1,
    PT_EL_HIGH = 2,
    PT_EL_LEVEL_NUMS
}PTEnhanceLevelEnum;

/********************************************************************
 *  Enhance camera data in real time if need
 *  Params:
 *  [IN]:
 *       srcData:       Pointer to source data of camera
 *       srcSize:       the size(width, height) of srcData
 *       srcColorSpace: the colorspace of srcData
 *       dstSize:       the size(width, height) of dstData
 *  [OUT]:
 *       dstData:     Pointer to the data which is enhanced
 *
 *  [RET]:
 *       PT_RET_OK:              success
 *       PT_RET_INVALIDPARAM:    if input is invalid
 *       PT_RET_ERR:             can not process image well
 *
 *********************************************************************/
//PTS32 PTEnhanceCameraPreview(PTU8* srcData, PTSize srcSize, PTImageFormatEnum srcColorSpace,
//                      PTU8* dstData, PTSize dstSize, PTImageFormatEnum dstColorSpace);

/********************************************************************
 *  Enhance an image
 *  Params:
 *  [IN]:
 *       srcData:       Pointer to source data of camera
 *       srcSize:       the size(width, height) of srcData
 *       srcColorSpace: the colorspace of srcData
 *       dstSize:       the size(width, height) of dstData
 *       hdrEnable:     if used hdr before
 *  [OUT]:
 *       dstData:     Pointer to the data which is enhanced
 *
 *  [RET]:
 *       PT_RET_OK:              success
 *       PT_RET_INVALIDPARAM:    if input is invalid
 *       PT_RET_ERR:             can not process image well
 *
 *********************************************************************/
PTS32 PTEnhanceImg(PTU8* srcData, PTSize srcSize, PTImageFormatEnum srcColorSpace,
                       PTU8* dstData, PTSize dstSize, PTImageFormatEnum dstColorSpace, PTU8 hdrEnable);
    

//PTS32 PTEnhanceStartImgEdit(PTU8* srcData, PTSize srcSize, PTImageFormatEnum srcColorSpace,
//                         PTU8* dstData, PTSize dstSize);


//PTS32 PTEnhanceImgEditPreview(PTU8* srcData, PTSize srcSize, PTImageFormatEnum srcColorSpace,
//                           PTU8* dstData, PTSize dstSize, PTEnhanceLevelEnum lType);

    


#endif
