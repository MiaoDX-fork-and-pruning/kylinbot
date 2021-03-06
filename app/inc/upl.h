/**
 * Copyright (c) 2011-2016, Jack Mo (mobangjack@foxmail.com).
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
 
#ifndef __UPL_H__
#define __UPL_H__

/*****************************************/
/*         Up-Link Communication         */
/*****************************************/

#ifdef __cplusplus
extern "C" {
#endif

#include "msg.h"
#include "cal.h"
#include "com.h"
#include "cmd.h"
#include "odo.h"
#include "ctl.h"
#include "dci.h"
#include "srs.h"
#include "wdg.h"
#include "ini.h"
#include "wsm.h"
#include "ios.h"

#define UPL_BUF_SIZE 256u

extern MsgType_t uplMsgType;

extern RcpMsg_t uplRcpMsg;
	
extern KylinMsg_t uplKylinMsg;
extern ZGyroMsg_t uplZGyroMsg;
extern SonarMsg_t uplSonarMsg;
extern StatuMsg_t uplStatuMsg;
extern SubscMsg_t uplSubscMsg;;

extern PosCalibMsg_t uplPosCalibMsg;
extern DpiCalibMsg_t uplDpiCalibMsg;
extern EpsCalibMsg_t uplEpsCalibMsg;
extern ComCalibMsg_t uplComCalibMsg;

void Upl_Init(void);
void Upl_Proc(void);

#ifdef __cplusplus
}
#endif

#endif


