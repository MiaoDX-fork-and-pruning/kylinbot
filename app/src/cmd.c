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

#include "cmd.h"

/**********************************************/
/*          System Command Interface          */
/**********************************************/
Cmd_t cmd;

void Cmd_Init(void)
{
	memset(&cmd, 0, sizeof(Cmd_t));
}

void Cmd_Proc(void)
{
	LIMIT_ABS(cmd.cv.x, cfg.vel.x);
	LIMIT_ABS(cmd.cv.y, cfg.vel.y);
	LIMIT_ABS(cmd.cv.z, cfg.vel.z);
	LIMIT(cmd.gp.e, cfg.pos.el, cfg.pos.eh);
	LIMIT(cmd.gp.c, cfg.pos.cl, cfg.pos.ch);
	
	//cmd.cp.x += cmd.cv.x * SYS_CTL_TSC;
	//cmd.cp.y += cmd.cv.y * SYS_CTL_TSC;
	//cmd.cp.z += cmd.cv.z * SYS_CTL_TSC;
	
	Mec_Decomp((float*)&cmd.cv, (float*)&cmd.mv);
	Mec_Decomp((float*)&cmd.cp, (float*)&cmd.mp);
}


