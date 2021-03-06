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
 
#include "cci.h"

/*************************************************/
/*            CBus Control Interface             */
/*************************************************/

static void GetPeriphsStateRef(const CBus_t* cbus)
{

}

static void GetChassisStateRef(const CBus_t* cbus)
{
	float pxr = cbus->cp.x / CBUS_VALUE_SCALE;
	float pyr = cbus->cp.y / CBUS_VALUE_SCALE;
	float pzr = cbus->cp.z / CBUS_VALUE_SCALE;
	float vxr = cbus->cv.x / CBUS_VALUE_SCALE;
	float vyr = cbus->cv.y / CBUS_VALUE_SCALE;
	float vzr = cbus->cv.z / CBUS_VALUE_SCALE;
	
	float dpx = Flag_Get(&cbus->fs, CBUS_FLAG_BIT_ABS) ? pxr - odo.cp.x : pxr;
	float dpy = Flag_Get(&cbus->fs, CBUS_FLAG_BIT_ABS) ? pyr - odo.cp.y : pyr;
	float dpz = Flag_Get(&cbus->fs, CBUS_FLAG_BIT_ABS) ? pzr - odo.cp.z : pzr;
	
	LIMIT_ABS(vxr, cfg.vel.x);
	LIMIT_ABS(vyr, cfg.vel.y);
	LIMIT_ABS(vzr, cfg.vel.z);
	
	cmd.cv.x = map(dpx, -cfg.dpi.x, cfg.dpi.x, -1, 1) * vxr;
	cmd.cv.y = map(dpy, -cfg.dpi.y, cfg.dpi.y, -1, 1) * vyr;
	cmd.cv.z = map(dpz, -cfg.dpi.z, cfg.dpi.z, -1, 1) * vzr;
	
	//cmd.cv.x = cbus->cv.x / CBUS_VALUE_SCALE;
	LIMIT(cmd.cv.x, -cfg.vel.x, cfg.vel.x);
	cmd.cp.x += cmd.cv.x * SYS_CTL_TSC;
	
	//cmd.cv.y = cbus->cv.y / CBUS_VALUE_SCALE;
	LIMIT(cmd.cv.y, -cfg.vel.y, cfg.vel.y);
	cmd.cp.y += cmd.cv.y * SYS_CTL_TSC;
	
	//cmd.cv.z = cbus->cv.z / CBUS_VALUE_SCALE;
	LIMIT(cmd.cv.z, -cfg.vel.z, cfg.vel.z);
	cmd.cp.z += cmd.cv.z * SYS_CTL_TSC;
}

static void GetGrabberStateRef(const CBus_t* cbus)
{
	float per = cbus->gp.e / CBUS_VALUE_SCALE;
	float pcr = cbus->gp.c / CBUS_VALUE_SCALE;
	float ver = cbus->gv.e / CBUS_VALUE_SCALE;
	float vcr = cbus->gv.c / CBUS_VALUE_SCALE;
	
	float dpe = Flag_Get(&cbus->fs, CBUS_FLAG_BIT_ABS) ? per - odo.gp.e : per;
	float dpc = Flag_Get(&cbus->fs, CBUS_FLAG_BIT_ABS) ? per - odo.gp.c : pcr;
	
	LIMIT(per, cfg.pos.el, cfg.pos.eh);
	LIMIT(pcr, cfg.pos.cl, cfg.pos.ch);
	
	LIMIT_ABS(ver, cfg.vel.e);
	LIMIT_ABS(vcr, cfg.vel.c);
	
	cmd.gv.e = map(dpe, -cfg.dpi.e, cfg.dpi.e, -1, 1) * ver;
	cmd.gv.c = map(dpc, -cfg.dpi.c, cfg.dpi.c, -1, 1) * vcr;
	
	//cmd.gv.e = cbus->gv.e / CBUS_VALUE_SCALE;
	LIMIT(cmd.gv.e, -cfg.vel.e, cfg.vel.e);
	cmd.gp.e += cmd.gv.e * SYS_CTL_TSC;
	LIMIT(cmd.gp.e, cfg.pos.el, cfg.pos.eh);
	
	//cmd.gv.c = cbus->gv.c / CBUS_VALUE_SCALE;
	LIMIT(cmd.gv.c, -cfg.vel.c, cfg.vel.c);
	cmd.gp.c += cmd.gv.c * SYS_CTL_TSC;
	LIMIT(cmd.gp.c, cfg.pos.cl, cfg.pos.ch);
}

void Cci_Init(void)
{
}

void Cci_Proc(const CBus_t* cbus)
{
	GetPeriphsStateRef(cbus);
	GetChassisStateRef(cbus);
	GetGrabberStateRef(cbus);
}


