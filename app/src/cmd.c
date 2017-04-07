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

static void GetPeriphsStateRef(void)
{
}

static void GetChassisStateRef(void)
{
	float dpx = Flag_Get(&cmd.fs, FS_CTL_ABS_MOD) ? cmd.cp.x - odo.cp.x : cmd.cp.x;
	float dpy = Flag_Get(&cmd.fs, FS_CTL_ABS_MOD) ? cmd.cp.y - odo.cp.y : cmd.cp.y;
	float dpz = Flag_Get(&cmd.fs, FS_CTL_ABS_MOD) ? cmd.cp.z - odo.cp.z : cmd.cp.z;
	
	LIMIT_ABS(cmd.cv.x, cfg.vel.x);
	LIMIT_ABS(cmd.cv.y, cfg.vel.y);
	LIMIT_ABS(cmd.cv.z, cfg.vel.z);
	
	cmd.cv.x = map(dpx, -cfg.dpi.x, cfg.dpi.x, -1, 1) * cmd.cv.x;
	cmd.cv.y = map(dpy, -cfg.dpi.y, cfg.dpi.y, -1, 1) * cmd.cv.x;
	cmd.cv.z = map(dpz, -cfg.dpi.z, cfg.dpi.z, -1, 1) * cmd.cv.x;
	
	LIMIT(cmd.cv.x, -cfg.vel.x, cfg.vel.x);
	LIMIT(cmd.cv.y, -cfg.vel.y, cfg.vel.y);
	LIMIT(cmd.cv.z, -cfg.vel.z, cfg.vel.z);
	
	cmd.cp.x += cmd.cv.x * SYS_CTL_TSC;
	cmd.cp.y += cmd.cv.y * SYS_CTL_TSC;
	cmd.cp.z += cmd.cv.z * SYS_CTL_TSC;
	
	Mec_Decomp((float*)&cmd.cv, (float*)&cmd.mv);
	Mec_Decomp((float*)&cmd.cp, (float*)&cmd.mp);
}

static void GetGrabberStateRef(void)
{
	float dpe = Flag_Get(&cmd.fs, FS_CTL_ABS_MOD) ? cmd.gp.e - odo.gp.e : cmd.gp.e;
	float dpc = Flag_Get(&cmd.fs, FS_CTL_ABS_MOD) ? cmd.gp.c - odo.gp.c : cmd.gp.c;
	
	LIMIT(cmd.gp.e, cfg.pos.el, cfg.pos.eh);
	LIMIT(cmd.gp.c, cfg.pos.cl, cfg.pos.ch);
	
	cmd.gv.e = map(dpe, -cfg.dpi.e, cfg.dpi.e, -1, 1) * cmd.gv.e;
	cmd.gv.c = map(dpc, -cfg.dpi.c, cfg.dpi.c, -1, 1) * cmd.gv.c;
	
	cmd.gp.e += cmd.gv.e * SYS_CTL_TSC;
	cmd.gp.c += cmd.gv.c * SYS_CTL_TSC;
}

void Cmd_Proc(void)
{
	GetPeriphsStateRef();
	GetChassisStateRef();
	GetGrabberStateRef();
}


