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

#ifndef __CMD_H__
#define __CMD_H__

/**********************************************/
/*          System Command Interface          */
/**********************************************/

#ifdef __cplusplus
extern "C" {
#endif

#include "sys.h"
#include "mec.h"
#include "cfg.h"
#include "fun.h"

void Cmd_Init(void);
void Cmd_Proc(void);

extern WorkingState_t workingStateRef;
extern PeriphsState_t functionalStateRef;
extern ChassisState_t chassisVelocityRef;
extern MecanumState_t mecanumVelocityRef; // Auto-Wired

#ifdef __cplusplus
}
#endif

#endif
