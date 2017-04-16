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
 
#include "dnl.h"

/*****************************************/
/*        Down-Link Communication        */
/*****************************************/

RcpMsg_t dnlRcpMsg;
HcpMsg_t dnlHcpMsg;

MsgType_t dnlMsgType;

DBusMsg_t dnlDBusMsg;
CBusMsg_t dnlCBusMsg;

KylinMsg_t dnlKylinMsg;
SubscMsg_t dnlSubscMsg;
CalibMsg_t dnlCalibMsg;

PIDCalib_t dnlPIDCalib;
IMUCalib_t dnlIMUCalib;
MagCalib_t dnlMagCalib;
MecCalib_t dnlMecCalib;
PosCalib_t dnlPosCalib;
VelCalib_t dnlVelCalib;
DpiCalib_t dnlDpiCalib;
EpsCalib_t dnlEpsCalib;
ComCalib_t dnlComCalib;

static uint8_t buf[2][DNL_BUF_SIZE];
static FIFO_t fifo;

static void Dnl_ProcRcpMsg(const RcpMsg_t* rcpMsg)
{
	dnlMsgType |= MSG_TYPE_RCP;
	if (Rci_Sw(SW_IDX_R) == SW_DN) {
		Rcp_Dec(&dbus.rcp, rcpMsg->data);
		Rci_Proc(&dbus.rcp);
	}
}

static void Dnl_ProcHcpMsg(const HcpMsg_t* hcpMsg)
{
	dnlMsgType |= MSG_TYPE_HCP;
	if (Rci_Sw(SW_IDX_R) == SW_DN) {
		Hcp_Dec(&dbus.hcp, hcpMsg->data);
		Hci_Proc(&dbus.hcp);
	}
}

static void Dnl_ProcDBusMsg(const DBusMsg_t* dbusMsg)
{
	dnlMsgType |= MSG_TYPE_DBUS;
	// To use this mode, the remote controller must be turned of.
	if (Wdg_HasErr(WDG_ERR_RCV)) {
		DBus_Dec(&dbus, dbusMsg->data);
		Dci_Proc(&dbus);
	}
}

static void Dnl_ProcCBusMsg(const CBusMsg_t* cbusMsg)
{
	dnlMsgType |= MSG_TYPE_CBUS;
	if (Rci_Sw(SW_IDX_R) == SW_DN) {
		Cci_Proc((CBus_t*)cbusMsg->data);
	}
}

static void Dnl_ProcSubscMsg(const SubscMsg_t* subscMsg)
{
	dnlMsgType |= MSG_TYPE_SUBSC;
	// TODO
	cfg.com.msg_type = subscMsg->msg_type;
	cfg_sync_flag = 1;
}

static void Dnl_ProcCalibMsg(const CalibMsg_t* calibMsg)
{
	dnlMsgType |= MSG_TYPE_CALIB;
	if (calibMsg->auto_cali_flag & CALIB_FLAG_BIT_GIM) {
		Cal_ClrFlag(CAL_FLAG_GIM);
	}
	if (calibMsg->auto_cali_flag & CALIB_FLAG_BIT_ODO) {
		Cal_ClrFlag(CAL_FLAG_ODO);
	}
}

static void Dnl_ProcKylinMsg(const KylinMsg_t* kylinMsg)
{
	dnlMsgType |= MSG_TYPE_KYLIN;
	if (Rci_Sw(1) == SW_DN && Wsm_GetWs() == WORKING_STATE_NORMAL) {
		Cci_Proc(&kylinMsg->cbus);
	}
}

static void Dnl_ProcIMUCalib(const IMUCalib_t* IMUCalib)
{
	dnlMsgType |= MSG_TYPE_IMU_CALIB;
	Calib_SetIMU(&cfg.imu, IMUCalib);
	Cfg_SetFlag(CFG_FLAG_IMU);
	cfg_sync_flag = 1;
}

static void Dnl_ProcMagCalib(const MagCalib_t* MagCalib)
{
	dnlMsgType |= MSG_TYPE_MAG_CALIB;
	Calib_SetMag(&cfg.mag, MagCalib);
	Cfg_SetFlag(CFG_FLAG_MAG);
	cfg_sync_flag = 1;
}

static void Dnl_ProcMecCalib(const MecCalib_t* MecCalib)
{
	dnlMsgType |= MSG_TYPE_MEC_CALIB;
	Calib_SetMec(&cfg.mec, MecCalib);
	Cfg_SetFlag(CFG_FLAG_MEC);
	cfg_sync_flag = 1;
}

static void Dnl_ProcPosCalib(const PosCalib_t* PosCalib)
{
	dnlMsgType |= MSG_TYPE_POS_CALIB;
	Calib_SetPos(&cfg.pos, PosCalib);
	Cfg_SetFlag(CFG_FLAG_POS);
	cfg_sync_flag = 1;
}

static void Dnl_ProcVelCalib(const VelCalib_t* VelCalib)
{
	dnlMsgType |= MSG_TYPE_VEL_CALIB;
	Calib_SetVel(&cfg.vel, VelCalib);
	Cfg_SetFlag(CFG_FLAG_VEL);
	cfg_sync_flag = 1;
}

static void Dnl_ProcPIDCalib(const PIDCalib_t* PIDCalib)
{
	dnlMsgType |= MSG_TYPE_PID_CALIB;
	if (PIDCalib->type == PID_CALIB_TYPE_CHASSIS_VELOCITY) {
		Calib_SetPID(&cfg.cvl, PIDCalib);
		Cfg_SetFlag(CFG_FLAG_CVL);
		cfg_sync_flag = 1;
	}
	else if (PIDCalib->type == PID_CALIB_TYPE_GRABBER_VELOCITY) {
		Calib_SetPID(&cfg.gvl, PIDCalib);
		Cfg_SetFlag(CFG_FLAG_GVL);
		cfg_sync_flag = 1;
	}
	else if (PIDCalib->type == PID_CALIB_TYPE_GRABBER_POSITION) {
		Calib_SetPID(&cfg.gpl, PIDCalib);
		Cfg_SetFlag(CFG_FLAG_GPL);
		cfg_sync_flag = 1;
	}
}

static void Dnl_ProcDpiCalib(const DpiCalib_t* dpiCalib)
{
	dnlMsgType |= MSG_TYPE_EPS_CALIB;
	Calib_SetDpi(&cfg.dpi, dpiCalib);
	cfg_sync_flag = 1;
}

static void Dnl_ProcEpsCalib(const EpsCalib_t* epsCalib)
{
	dnlMsgType |= MSG_TYPE_EPS_CALIB;
	Calib_SetEps(&cfg.eps, epsCalib);
	cfg_sync_flag = 1;
}

static void Dnl_ProcComCalib(const ComCalib_t* comCalib)
{
	dnlMsgType |= MSG_TYPE_COM_CALIB;
	memcpy(&cfg.com, comCalib, sizeof(ComCalib_t));
	LIMIT(cfg.com.tdm_tdiv, 1000, 100000);
	cfg_sync_flag = 1;
}

void Dnl_Init(void)
{
	FIFO_Init(&fifo, buf[0], DNL_BUF_SIZE);
}

void Dnl_Proc(void)
{
	// Get fifo free space
	int len = FIFO_GetFree(&fifo);
	// If fifo free space insufficient, pop one element out
	if (len < 1) {
		uint8_t b;
		len = FIFO_Pop(&fifo, &b, 1);
	}
	// Read input stream according to the fifo free space left
	len = IOS_COM_DEV.Read(buf[1], len);
	// If input stream not available, abort
	if (len > 0) {
		// Push stream into fifo
		FIFO_Push(&fifo, buf[1], len);
	}
	// Check if any message received
	if (Msg_Pop(&fifo, buf[1], &msg_head_rcp, &dnlRcpMsg)) {
		Dnl_ProcRcpMsg(&dnlRcpMsg);
	}
	if (Msg_Pop(&fifo, buf[1], &msg_head_hcp, &dnlHcpMsg)) {
		Dnl_ProcHcpMsg(&dnlHcpMsg);
	}
	if (Msg_Pop(&fifo, buf[1], &msg_head_dbus, &dnlDBusMsg)) {
		Dnl_ProcDBusMsg(&dnlDBusMsg);
	}
	if (Msg_Pop(&fifo, buf[1], &msg_head_cbus, &dnlCBusMsg)) {
		Dnl_ProcCBusMsg(&dnlCBusMsg);
	}
	if (Msg_Pop(&fifo, buf[1], &msg_head_kylin, &dnlKylinMsg)) {
		Dnl_ProcKylinMsg(&dnlKylinMsg);
	}
	if (Msg_Pop(&fifo, buf[1], &msg_head_subsc, &dnlSubscMsg)) {
		Dnl_ProcSubscMsg(&dnlSubscMsg);
	}
	if (Msg_Pop(&fifo, buf[1], &msg_head_calib, &dnlCalibMsg)) {
		Dnl_ProcCalibMsg(&dnlCalibMsg);
	}
	if (Msg_Pop(&fifo, buf[1], &msg_head_pid_calib, &dnlPIDCalib)) {
		Dnl_ProcPIDCalib(&dnlPIDCalib);
	}
	if (Msg_Pop(&fifo, buf[1], &msg_head_imu_calib, &dnlIMUCalib)) {
		Dnl_ProcIMUCalib(&dnlIMUCalib);
	}
	if (Msg_Pop(&fifo, buf[1], &msg_head_mag_calib, &dnlMagCalib)) {
		Dnl_ProcMagCalib(&dnlMagCalib);
	}
	if (Msg_Pop(&fifo, buf[1], &msg_head_mec_calib, &dnlMecCalib)) {
		Dnl_ProcMecCalib(&dnlMecCalib);
	}
	if (Msg_Pop(&fifo, buf[1], &msg_head_pos_calib, &dnlPosCalib)) {
		Dnl_ProcPosCalib(&dnlPosCalib);
	}
	if (Msg_Pop(&fifo, buf[1], &msg_head_vel_calib, &dnlVelCalib)) {
		Dnl_ProcVelCalib(&dnlVelCalib);
	}
	if (Msg_Pop(&fifo, buf[1], &msg_head_dpi_calib, &dnlDpiCalib)) {
		Dnl_ProcDpiCalib(&dnlDpiCalib);
	}
	if (Msg_Pop(&fifo, buf[1], &msg_head_eps_calib, &dnlEpsCalib)) {
		Dnl_ProcEpsCalib(&dnlEpsCalib);
	}
	if (Msg_Pop(&fifo, buf[1], &msg_head_com_calib, &dnlComCalib)) {
		Dnl_ProcComCalib(&dnlComCalib);
	}
}



