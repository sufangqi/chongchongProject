//
//  answer_judger.h
//  tangram_predictor
//
//  Created by putao on 15/6/3.
//  Copyright (c) 2015年 putao. All rights reserved.
//

#ifndef tangram_predictor_answer_judger_h
#define tangram_predictor_answer_judger_h
#include <stdio.h>

#include "basept.h"

//#define GetMirrorStateWhenRecogBoards

typedef struct _BOARDINFO
{
    PTU8        boardID;
    PTS32       angle;
    PTBOOL      flip;
    PTPoint2d   centerPoint;
    PTBOOL      visible;
    _BOARDINFO() {boardID=-1; angle=-1; flip=0; centerPoint=PTPoint2d(0,0); visible=FALSE;}
}PTBoardInfo;

typedef struct _CurEnviroState
{
    PTMirrorWarnEnum mirrorWarning;
    PTBOOL enviroIndistinct;
    PTLumiStEnum lumiSt;
    _CurEnviroState() {mirrorWarning=PT_MIRROR_WARN_NUM; enviroIndistinct=FALSE; lumiSt=PT_LUMI_NUM;}
}PTCurEnviroSt;

typedef struct _TangramAnswer
{
#ifdef GetMirrorStateWhenRecogBoards
    PTCurEnviroSt enviroSt;
#endif
    PTS32       tipForMovingAngle;
    PTBOOL      rhomboidNeedFilp;
    PTS32       boardsAmount;
    PTBoardInfo boardInfo[7];
    _TangramAnswer() {tipForMovingAngle=-1; rhomboidNeedFilp=FALSE; boardsAmount=0;}
}PTTangramAnswer;



typedef struct _TangramHint
{
    PTS32       size;
    PTBoardInfo boardInfo[7];
    _TangramHint() {size=-1;}
}PTTangramHint;


typedef void* PTTangramHandler;

//const PAIR redBarHRange = PAIR(170, 8);
//const PAIR redBarSRange = PAIR(255*0.35, 255);
//const PAIR redBarVRange = PAIR(255*0.1, 255);
//
////TODO: XUXUXUXU need to change the parameter according to the UE
////TODO: XUXUXUXU check all _calcVertexes functions for rhomboid
////const PTS32 minExceedAreaPro = 0;
////const PTS32 minPreExceedAreaPro = 0;

//PTBOOL gIfUnevenLumi = FALSE;



typedef enum _GAMEMODE {
    GAME_MODE_COLOR       = 0,
    GAME_MODE_BLACK_GRAY  = 1,
    GAME_MODE_BLACK       = 2,
    GAME_MODE_MODE_NUM    = 3
} PTGameMode;

const char strGameMode[GAME_MODE_MODE_NUM+1][MAX_STRING_LENGTH] = {"GAME_MODE_COLOR", "GAME_MODE_BLACK_GRAY", "GAME_MODE_BLACK", "NULL"};

#define PT_RET_INVALID_RECOG -8//can not do recognition well
#define PT_RET_NO_VALID_ANS  -9//have no available answer after filtering answer according to hint

#ifdef GetMirrorStateWhenRecogBoards
#define PT_RET_INVALID_MIRROR_STATE -10//can not get the mirror state well
#endif


/********************************************************************
 *  Tangram initialization
 *  Params:
 *  [IN]:
 *       pInitData:     Pointer to PTTangramInitData struct.
 *       ppHandler:     Pointer to tangram handler
 *  [OUT]:
 *       ppHandler:     NULL written into this address.
 *
 *  [RET]:
 *       PT_RET_INVALIDPARAM  : if ppHandler is NULL
 *       PT_RET_OK            : success
 *
 *
 ********************************************************************/
PTS32 PTTangramInit(const PTTangramInitData *pInitData, PTTangramHandler *ppHandler);

/********************************************************************
 *  parse content whose format is JSON
 *  Params:
 *  [IN]:
 *       pHandler: tangram handler
 *       content : the content of json file
 *       gameMode: the mode of current game
 *  [OUT]:
 *       defalutAnswer : the default answer that is given when init answer vector
 *
 *  [RET]:
 *       PT_RET_INVALIDPARAM : some parameter is invalid
 *       PT_RET_OK           : success
 *
 ********************************************************************/
PTS32 PTTangramStart(PTTangramHandler pHandler, const char* jsonContent, PTGameMode gameMode,
                     PTTangramAnswer& defalutAnswer);

/********************************************************************
 *  get current result of tangram which palyer makes
 *  Params:
 *  [IN]:
 *       pHandler             :  tangram handler
 *       pPixels              :  pointer to buffer of the input image
 *       hintBoardInfo        :  the struct that store infos for hint board
 *                                      the position of hint board is absolute position
 *       minExceedAreaPro     :  the min proportion of exceed part of the board
 *       minPreExceedAreaPro  :  the min proportion of exceed part of the board(prediction)
 *  [OUT]:
 *       answerBoardsInfo     :  the struct that stores these 7 boards's info
 *
 *  [RET]:
 *       PT_RET_NO_VALID_ANS  : have no available answer currently
 *       PT_RET_INVALID_RECOG : can not do recognition well
 *       PT_RET_INVALIDPARAM  : bad input param
 *       PT_RET_NOMEM         : have no memory when applying for memory
 *       PT_RET_OK            : success
 *
 *
 ********************************************************************/
PTS32 PTGetCurrentResult(PTTangramHandler pHandler, PTU8 *pPixels,PTTangramHint hintBoardInfo, PTTangramAnswer& answerBoardsInfo, PTS32 minExceedAreaPro=0, PTS32 minPreExceedAreaPro=0);


/********************************************************************
 *  get current enviroment state
 *  Params:
 *  [IN]:
 *       pHandler             :  tangram handler
 *       pPixels              :  pointer to buffer of the input image
 *  [OUT]:
 *       curEnviroSt          :  the struct that stores mirror state,luminate state
 *                                  and if the mirror is indistinct
 *
 *  [RET]:
 *       PT_RET_INVALIDPARAM  : bad input param
 *       PT_RET_NOMEM         : have no memory when applying for memory
 *       PT_RET_OK            : success
 *
 *
 ********************************************************************/
PTS32 PTGetCurEnviroState(PTTangramHandler pHandler, PTU8 *pPixels, PTCurEnviroSt& curEnviroSt);


/********************************************************************
 *  Tangram de-intialization
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
PTS32 PTTangramDeinit(PTTangramHandler *ppHandler);

#endif
