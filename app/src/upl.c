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
 
#include "upl.h"

/*****************************************/
/*         Up-Link Communication         */
/*****************************************/

MsgType_t uplMsgType = MSG_TYPE_RCP;

RcpMsg_t uplRcpMsg;

KylinMsg_t uplKylinMsg;
ZGyroMsg_t uplZGyroMsg;
SonarMsg_t uplSonarMsg;
StatuMsg_t uplStatuMsg;
SubscMsg_t uplSubscMsg;

PosCalibMsg_t uplPosCalibMsg;
DpiCalibMsg_t uplDpiCalibMsg;
EpsCalibMsg_t uplEpsCalibMsg;
ComCalibMsg_t uplComCalibMsg;

static uint8_t buf[2][UPL_BUF_SIZE];
static FIFO_t fifo;

static void Upl_PushRcpMsg(void)
{
	uplRcpMsg.frame_id++;
	Rcp_Enc(&dbus.rcp, uplRcpMsg.data);
	Msg_Push(&fifo, buf[1], &msg_head_rcp, &uplRcpMsg);
}

static void Upl_PushKylinMsg(void)
{
	uplKylinMsg.frame_id++;
	Flag_Det(&uplKylinMsg.cbus.fs, CBUS_FLAG_BIT_PXC, ABSVAL(pid.cp.x.error[1]) < cfg.eps.x);
	Flag_Det(&uplKylinMsg.cbus.fs, CBUS_FLAG_BIT_PYC, ABSVAL(pid.cp.y.error[1]) < cfg.eps.y);
	Flag_Det(&uplKylinMsg.cbus.fs, CBUS_FLAG_BIT_PZC, ABSVAL(pid.cp.z.error[1]) < cfg.eps.z);
	Flag_Det(&uplKylinMsg.cbus.fs, CBUS_FLAG_BIT_PEC, ABSVAL(pid.gp.e.error[1]) < cfg.eps.e);
	Flag_Det(&uplKylinMsg.cbus.fs, CBUS_FLAG_BIT_PCC, ABSVAL(pid.gp.c.error[1]) < cfg.eps.c);
	uplKylinMsg.cbus.cv.x = odo.cv.x * CBUS_VALUE_SCALE;
	uplKylinMsg.cbus.cv.y = odo.cv.y * CBUS_VALUE_SCALE;
	uplKylinMsg.cbus.cv.z = odo.cv.z * CBUS_VALUE_SCALE;
	uplKylinMsg.cbus.cp.x = odo.cp.x * CBUS_VALUE_SCALE;
	uplKylinMsg.cbus.cp.y = odo.cp.y * CBUS_VALUE_SCALE;
	uplKylinMsg.cbus.cp.z = odo.cp.z * CBUS_VALUE_SCALE;
	uplKylinMsg.cbus.gv.e = odo.gv.e * CBUS_VALUE_SCALE;
	uplKylinMsg.cbus.gp.e = odo.gp.e * CBUS_VALUE_SCALE;
	uplKylinMsg.cbus.gv.c = odo.gv.c * CBUS_VALUE_SCALE;
	uplKylinMsg.cbus.gp.c = odo.gp.c * CBUS_VALUE_SCALE;
	Msg_Push(&fifo, buf[1], &msg_head_kylin, &uplKylinMsg);
}

static void Upl_PushSonarMsg(void)
{
	uint8_t i = 0;
	uplSonarMsg.frame_id++;
	for (i = 0; i < SR04_NUM; i++) {
		uplSonarMsg.data[i] = srs[i].mm_filtered;
	}
	Msg_Push(&fifo, buf[1], &msg_head_sonar, &uplSonarMsg);
}

static void Upl_PushZGyroMsg(void)
{
	uplZGyroMsg.frame_id++;
	uplZGyroMsg.angle = zgyro.angle;
	uplZGyroMsg.rate = zgyro.rate;
	Msg_Push(&fifo, buf[1], &msg_head_zgyro, &uplZGyroMsg);
}

static void Upl_PushStatuMsg(void)
{
	uplStatuMsg.frame_id++;
	uplStatuMsg.wdg = Wdg_GetErr();
	uplStatuMsg.ini = Ini_GetFlag();
	Msg_Push(&fifo, buf[1], &msg_head_statu, &uplStatuMsg);
}

static void Upl_PushSubscMsg(void)
{
	uplSubscMsg.frame_id++;
	// TODO
	uplSubscMsg.msg_type = cfg.com.msg_type;
	Msg_Push(&fifo, buf[1], &msg_head_subsc, &uplSubscMsg);
}

static void Upl_PushPosCalibMsg(void)
{
	uplPosCalibMsg.frame_id++;
	uplPosCalibMsg.data.ch = map(CLAW_PWM_H, 1000, 2000, 0, PI) * POS_CALIB_VALUE_SCALE;
	uplPosCalibMsg.data.cl = map(CLAW_PWM_L, 1000, 2000, 0, PI) * POS_CALIB_VALUE_SCALE;
	uplPosCalibMsg.data.eh = cfg.pos.eh * POS_CALIB_VALUE_SCALE;
	uplPosCalibMsg.data.el = cfg.pos.el * POS_CALIB_VALUE_SCALE;
	Msg_Push(&fifo, buf[1], &msg_head_pos_calib, &uplPosCalibMsg);
}

static void Upl_PushDpiCalibMsg(void)
{
	uplDpiCalibMsg.frame_id++;
	uplDpiCalibMsg.data.x = cfg.dpi.x * DPI_CALIB_VALUE_SCALE;
	uplDpiCalibMsg.data.y = cfg.dpi.y * DPI_CALIB_VALUE_SCALE;
	uplDpiCalibMsg.data.z = cfg.dpi.z * DPI_CALIB_VALUE_SCALE;
	uplDpiCalibMsg.data.e = cfg.dpi.e * DPI_CALIB_VALUE_SCALE;
	uplDpiCalibMsg.data.c = cfg.dpi.c * DPI_CALIB_VALUE_SCALE;
	Msg_Push(&fifo, buf[1], &msg_head_dpi_calib, &uplDpiCalibMsg);
}

static void Upl_PushEpsCalibMsg(void)
{
	uplEpsCalibMsg.frame_id++;
	uplEpsCalibMsg.data.x = cfg.eps.x * EPS_CALIB_VALUE_SCALE;
	uplEpsCalibMsg.data.y = cfg.eps.y * EPS_CALIB_VALUE_SCALE;
	uplEpsCalibMsg.data.z = cfg.eps.z * EPS_CALIB_VALUE_SCALE;
	uplEpsCalibMsg.data.e = cfg.eps.e * EPS_CALIB_VALUE_SCALE;
	uplEpsCalibMsg.data.c = cfg.eps.c * EPS_CALIB_VALUE_SCALE;
	Msg_Push(&fifo, buf[1], &msg_head_eps_calib, &uplEpsCalibMsg);
}

static void Upl_PushComCalibMsg(void)
{
	uplComCalibMsg.frame_id++;
	memcpy(&uplComCalibMsg, &cfg.com, sizeof(ComCalib_t));
	Msg_Push(&fifo, buf[1], &msg_head_com_calib, &uplComCalibMsg);
}

static void Upl_SendMsg(void)
{
	uint8_t data;
	while (!FIFO_IsEmpty(&fifo)) {
		FIFO_Pop(&fifo, &data, 1);
		IOS_COM_DEV.WriteByte(data);
	}
}

static uint8_t Upl_SendRcpMsg(void)
{
	if (IOS_COM_DEV.GetTxFifoFree() >= msg_head_rcp.attr.length + MSG_LEN_EXT) {
		Upl_PushRcpMsg();
		Upl_SendMsg();
		return 1;
	}
	return 0;
}

static uint8_t Upl_SendKylinMsg(void)
{
	if (IOS_COM_DEV.GetTxFifoFree() >= msg_head_kylin.attr.length + MSG_LEN_EXT) {
		Upl_PushKylinMsg();
		Upl_SendMsg();
		return 1;
	}
	return 0;
}

static uint8_t Upl_SendStatuMsg(void)
{
	if (IOS_COM_DEV.GetTxFifoFree() >= msg_head_statu.attr.length + MSG_LEN_EXT) {
		Upl_PushStatuMsg();
		Upl_SendMsg();
		return 1;
	}
	return 0;
}

static uint8_t Upl_SendSubscMsg(void)
{
	if (IOS_COM_DEV.GetTxFifoFree() >= msg_head_subsc.attr.length + MSG_LEN_EXT) {
		Upl_PushSubscMsg();
		Upl_SendMsg();
		return 1;
	}
	return 0;
}

static uint8_t Upl_SendZGyroMsg(void)
{
	if (IOS_COM_DEV.GetTxFifoFree() >= msg_head_zgyro.attr.length + MSG_LEN_EXT) {
		Upl_PushZGyroMsg();
		Upl_SendMsg();
		return 1;
	}
	return 0;
}

static uint8_t Upl_SendSonarMsg(void)
{
	if (IOS_COM_DEV.GetTxFifoFree() >= msg_head_sonar.attr.length + MSG_LEN_EXT) {
		Upl_PushSonarMsg();
		Upl_SendMsg();
		return 1;
	}
	return 0;
}

static uint8_t Upl_SendPosCalibMsg(void)
{
	if (IOS_COM_DEV.GetTxFifoFree() >= msg_head_pos_calib.attr.length + MSG_LEN_EXT) {
		Upl_PushPosCalibMsg();
		Upl_SendMsg();
		return 1;
	}
	return 0;
}

static uint8_t Upl_SendDpiCalibMsg(void)
{
	if (IOS_COM_DEV.GetTxFifoFree() >= msg_head_dpi_calib.attr.length + MSG_LEN_EXT) {
		Upl_PushDpiCalibMsg();
		Upl_SendMsg();
		return 1;
	}
	return 0;
}

static uint8_t Upl_SendEpsCalibMsg(void)
{
	if (IOS_COM_DEV.GetTxFifoFree() >= msg_head_eps_calib.attr.length + MSG_LEN_EXT) {
		Upl_PushEpsCalibMsg();
		Upl_SendMsg();
		return 1;
	}
	return 0;
}

static uint8_t Upl_SendComCalibMsg(void)
{
	if (IOS_COM_DEV.GetTxFifoFree() >= msg_head_com_calib.attr.length + MSG_LEN_EXT) {
		Upl_PushComCalibMsg();
		Upl_SendMsg();
		return 1;
	}
	return 0;
}

void Upl_Init(void)
{
	FIFO_Init(&fifo, buf[0], UPL_BUF_SIZE);
}

#define UPL_SPIN() do { uplMsgType = CROL(uplMsgType, sizeof(MsgType_t), 1) & cfg.com.msg_type; } while (0)
void Upl_Proc(void)
{
	UPL_SPIN();
	switch (uplMsgType) {
		case MSG_TYPE_RCP:
			if (Upl_SendRcpMsg()) {
				UPL_SPIN();
			}
			break;
		case MSG_TYPE_KYLIN:
			if (Upl_SendKylinMsg()) {
				UPL_SPIN();
			}
			break;
		case MSG_TYPE_STATU:
			if (Upl_SendStatuMsg()) {
				UPL_SPIN();
			}
			break;
		case MSG_TYPE_SUBSC:
			if (Upl_SendSubscMsg()) {
				UPL_SPIN();
			}
			break;
		case MSG_TYPE_ZGYRO:
			if (Upl_SendZGyroMsg()) {
				UPL_SPIN();
			}
			break;
		case MSG_TYPE_SONAR:
			if (Upl_SendSonarMsg()) {
				UPL_SPIN();
			}
			break;
		case MSG_TYPE_POS_CALIB:
			if (Upl_SendPosCalibMsg()) {
				UPL_SPIN();
			}
			break;
		case MSG_TYPE_DPI_CALIB:
			if (Upl_SendDpiCalibMsg()) {
				UPL_SPIN();
			}
		case MSG_TYPE_EPS_CALIB:
			if (Upl_SendEpsCalibMsg()) {
				UPL_SPIN();
			}
			break;
		case MSG_TYPE_COM_CALIB:
			if (Upl_SendComCalibMsg()) {
				UPL_SPIN();
			}
			break;
		default:
			UPL_SPIN();
		break;
	}
}


