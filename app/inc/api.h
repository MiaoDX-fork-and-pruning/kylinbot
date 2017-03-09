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
 
#ifndef __API_H__
#define __API_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "aci.h"
#include "dci.h"
#include "msg.h"
	
void Api_Init(void);
void Api_Proc(void);

void VRC_Proc(const VirtualRC_t* vrc);
void VHC_Proc(const VirtualHC_t* vhc);
void VDBUS_Proc(const VirtualDBUS_t* vdbus);
void VCBUS_Proc(const VirtualCBUS_t* vcbus);

extern VirtualDBUS_t vdbus;
extern VirtualCBUS_t vcbus;

#ifdef __cplusplus
}
#endif

#endif

