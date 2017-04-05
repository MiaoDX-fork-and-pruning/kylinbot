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
 
#ifndef __DNL_H__
#define __DNL_H__

/*****************************************/
/*        Down-Link Communication        */
/*****************************************/

#ifdef __cplusplus
extern "C" {
#endif

#include "ios.h"
#include "msg.h"
#include "cal.h"
#include "dci.h"
#include "cci.h"
#include "fun.h"
	
#define DNL_BUF_SIZE 256u

void Dnl_Init(void);
void Dnl_Proc(void);

extern RcpMsg_t dnlRcpMsg;
extern HcpMsg_t dnlHcpMsg;

extern MsgType_t dnlMsgType;
	
extern DBusMsg_t dnlDBusMsg;
extern CBusMsg_t dnlCBusMsg;

extern KylinMsg_t dnlKylinMsg;
extern SubscMsg_t dnlSubscMsg;
extern CalibMsg_t dnlCalibMsg;

extern PIDCalib_t dnlPIDCalib;
extern IMUCalib_t dnlIMUCalib;
extern MagCalib_t dnlMagCalib;
extern VelCalib_t dnlVelCalib;
extern MecCalib_t dnlMecCalib;
extern PosCalib_t dnlPosCalib;



#ifdef __cplusplus
}
#endif

#endif
