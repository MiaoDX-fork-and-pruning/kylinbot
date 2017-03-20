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
 
#include "ctl.h"

/**********************************************/
/*              Logic Controller              */
/**********************************************/

Ctl_t ctl;

PID_t CM1AnglePID;
PID_t CM1SpeedPID;
PID_t CM2AnglePID;
PID_t CM2SpeedPID;
PID_t CM3AnglePID;
PID_t CM3SpeedPID;
PID_t CM4AnglePID;
PID_t CM4SpeedPID;
PID_t GMEAnglePID;
PID_t GMESpeedPID;

Rmp_t CM1SpeedRmp;
Rmp_t CM2SpeedRmp;
Rmp_t CM3SpeedRmp;
Rmp_t CM4SpeedRmp;
Rmp_t GMESpeedRmp;
Rmp_t GMCSpeedRmp;

/**********************************************/
/*    Peripherals Functional State Control    */
/**********************************************/
static void FunctionalStateControl(void)
{
	FS_Cpy(&ctl.fs, cmd.fs, FS_ALL);
}

/**********************************************/
/*          Chassis Velocity Control          */
/**********************************************/
static void ChassisVelocityControl(void)
{
	ctl.mv.w1 = PID_Calc(&CM1AnglePID, cmd.mp.w1, odo.mp.w1);
	ctl.mc.w1 = PID_Calc(&CM1SpeedPID, ctl.mv.w1, odo.mv.w1) * Rmp_Calc(&CM1SpeedRmp);
	
	ctl.mv.w2 = PID_Calc(&CM2AnglePID, cmd.mp.w2, odo.mp.w2);
	ctl.mc.w2 = PID_Calc(&CM2SpeedPID, ctl.mv.w2, odo.mv.w2) * Rmp_Calc(&CM2SpeedRmp);
	
	ctl.mv.w3 = PID_Calc(&CM3AnglePID, cmd.mp.w3, odo.mp.w3);
	ctl.mc.w3 = PID_Calc(&CM3SpeedPID, ctl.mv.w3, odo.mv.w3) * Rmp_Calc(&CM3SpeedRmp);
	
	ctl.mv.w4 = PID_Calc(&CM4AnglePID, cmd.mp.w4, odo.mp.w4);
	ctl.mc.w4 = PID_Calc(&CM4SpeedPID, ctl.mv.w4, odo.mv.w4) * Rmp_Calc(&CM4SpeedRmp);
}

static void GrabberPositionControl(void)
{
	ctl.gv.e = PID_Calc(&GMEAnglePID, cmd.gp.e, odo.gp.e); // Elevator motor angle PID
	ctl.gc.e = PID_Calc(&GMESpeedPID, ctl.gv.e, odo.gv.e) * Rmp_Calc(&GMESpeedRmp); // Elevator motor speed PID
	//ctl.gv.c = 10 * (cmd.gp.c - odo.gp.c);
	//ctl.gc.c = odo.gp.c + ctl.gv.c;
	ctl.gc.c = map(cmd.gp.c, cfg.pos.cl, cfg.pos.ch, CLAW_PWM_L, CLAW_PWM_H); // Direct PWM control (1000~2000)/2500, map rad to pwm duty cycle
}

static void Cpl_Init(PID_t* pid)
{
	PID_Config(pid, 
		 cfg.cpl.kp, 
		 cfg.cpl.ki, 
		 cfg.cpl.kd, 
		 cfg.cpl.it,
		 cfg.cpl.Emax,
		 cfg.cpl.Pmax, 
		 cfg.cpl.Imax, 
		 cfg.cpl.Dmax, 
		 cfg.cpl.Omax);
	PID_Reset(pid);
}

static void Cvl_Init(PID_t* pid)
{
	PID_Config(pid, 
		 cfg.cvl.kp, 
		 cfg.cvl.ki, 
		 cfg.cvl.kd, 
		 cfg.cvl.it,
		 cfg.cvl.Emax,
		 cfg.cvl.Pmax, 
		 cfg.cvl.Imax, 
		 cfg.cvl.Dmax, 
		 cfg.cvl.Omax);
	PID_Reset(pid);
}

static void Gvl_Init(PID_t* pid)
{
	PID_Config(pid, 
		 cfg.gvl.kp, 
		 cfg.gvl.ki, 
		 cfg.gvl.kd, 
		 cfg.gvl.it,
		 cfg.gvl.Emax,
		 cfg.gvl.Pmax, 
		 cfg.gvl.Imax, 
		 cfg.gvl.Dmax, 
		 cfg.gvl.Omax);
	PID_Reset(pid);
}

static void Gpl_Init(PID_t* pid)
{
	PID_Config(pid, 
		 cfg.gpl.kp, 
		 cfg.gpl.ki, 
		 cfg.gpl.kd, 
		 cfg.gpl.it,
		 cfg.gpl.Emax,
		 cfg.gpl.Pmax, 
		 cfg.gpl.Imax, 
		 cfg.gpl.Dmax, 
		 cfg.gpl.Omax);
	PID_Reset(pid);
}

static void Rmp_Init(Rmp_t* rmp)
{
	Rmp_Config(rmp, cfg.rmp.cnt);
	Rmp_Reset(rmp);
}

/**********************************************/
/*       Logic Controller Initialization      */
/**********************************************/
void Ctl_Init(void)
{
	Cmd_Init();
	Odo_Init();
	
	Cvl_Init(&CM1SpeedPID);
	Cvl_Init(&CM2SpeedPID);
	Cvl_Init(&CM3SpeedPID);
	Cvl_Init(&CM4SpeedPID);
	
	Cpl_Init(&CM1AnglePID);
	Cpl_Init(&CM2AnglePID);
	Cpl_Init(&CM3AnglePID);
	Cpl_Init(&CM4AnglePID);
	
	Gvl_Init(&GMESpeedPID);
	Gpl_Init(&GMEAnglePID);
	
	Rmp_Init(&CM1SpeedRmp);
	Rmp_Init(&CM2SpeedRmp);
	Rmp_Init(&CM3SpeedRmp);
	Rmp_Init(&CM4SpeedRmp);
	Rmp_Init(&GMESpeedRmp);
	Rmp_Init(&GMCSpeedRmp);
	
	FS_Clr(&ctl.fs, FS_ALL);
	MS_Set(&ctl.mc, 0, 0, 0, 0);
	GS_Set(&ctl.gv, 0, 0);
	GS_Set(&ctl.gc, 0, 0);
}

/**********************************************/
/*          Logic Controller Process          */
/**********************************************/
void Ctl_Proc(void)
{
	Cmd_Proc();
	Odo_Proc();
	
	FunctionalStateControl();
	ChassisVelocityControl();
	GrabberPositionControl();
}

