/******************************************************************************
 *     Copyright (c) 2015 PuTao Co., Ltd.
 *     All Rights Reserved.
 ******************************************************************************/

#ifndef __HDR_H__
#define __HDR_H__

#include "basept.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct _HDRInitData {
  PTS32               nWidth;
  PTS32               nHeight;
  PTImageFormatEnum   eFormat;
} PTHDRInitData;

typedef void* PTHDRHandler;

/********************************************************************
 *  Check whether HDR effect is needed for current sence
 *  Params:
 *  [IN]:
 *       pAutoExposedImage:  pointer to buffer of auto exposure image
 *       nWidth, nHeight  :  width and height of the auto exposure image
 *       eFormat          :  format of the auto exposure image
 *  [OUT]:
 *       bNeedHDR         :  pointer to the PTBOOL variable to hold
                             the check result
 *
 *  [RET]:
 *       PT_RET_INVALIDPARAM : some parameter is invalid
 *       PT_RET_OK           : success
 *
 ********************************************************************/

PTS32 PTHDRCheck(PTU8 *pAutoExposedImage, PTS32 nWidth, PTS32 nHeight, PTImageFormatEnum eFormat, PTBOOL *bNeedHDR);

/********************************************************************
 *  High Dynamic Range Engine initialization
 *  Params:
 *  [IN]:
 *       pInitData:     Pointer to PTHDRInitData struct.
 *       ppHandler:     Pointer to hdr handler
 *  [OUT]:
 *       ppHandler:     Pointer to hdr handler, return correct handler on function
 *                      success.
 *  [RET]:
 *       PT_RET_OK:               success
 *       PT_RET_INVALIDPARAM:     invalid parameters in pInitData (if it's not NULL) or
 *                                ppHandler is NULL
 *       PT_RET_NOMEM:            out of memory
 *
 *********************************************************************/

PTS32 PTHDRInit(PTHDRInitData *pInitData, PTHDRHandler *ppHandler);

/********************************************************************
 *  High Dynamic Range Engine de-intialization
 *  Params:
 *  [IN]:
 *       ppHandler:     Pointer to hdr handler
 *  [OUT]:
 *       ppHandler:     NULL written into this address.
 *
 *  [RET]:
 *       PT_RET_OK:              success
 *       PT_RET_INVALIDPARAM:    if ppHandler is NULL
 *
 *********************************************************************/

PTS32 PTHDRDeinit(PTHDRHandler *ppHandler);

/********************************************************************
 *  High Dynamic Range Engine register ONE input images
 *  Params:
 *  [IN]:
 *       pHandler:      hdr handler
 *       pImage:        pointer of one input image
 *       exposureTime:  exposure time of this input image, no unit
 *
 *  [RET]:
 *       PT_RET_OK:              success
 *       PT_RET_INVALIDPARAM:    if pHandler or pImage is NULL, or exposureTime < 0
 *
 ********************************************************************/

PTS32 PTHDRRegisterOneImage(PTHDRHandler pHandler, PTU8 *pImage, PTF32 exposureTime);

/********************************************************************
 *  High Dynamic Range Engine main processing function.
 *  Params:
 *  [IN]:
 *       pHandler:   hdr handler
 *       pOutImage:  pointer to dst image.
 *  [OUT]:
 *       N/A
 *  [RET]:
 *       PT_RET_OK:              success
 *       PT_RET_INVALIDPARAM:    if any pointer is NULL or any input is invalid
 *
 *********************************************************************/

PTS32 PTHDRMerge(PTHDRHandler pHandler, PTU8 *pOutImage);

/********************************************************************
 *  Get High Dynamic Range Engine version infomation
 *  Params:
 *  [IN]:
 *       pVendorInfo:  pointer to buffer to hold version information string
 *  [OUT]:
 *       pVendorInfo:  pointer to buffer to hold version information string
 *
 *  [RET]:
 *       PT_RET_OK:    success
 *
 ********************************************************************/

PTS32 PTHDRGetVendorString(char *pVendorInfo);

#ifdef __cplusplus
}
#endif

#endif /* __HDR_H__ */

/* EOF */

