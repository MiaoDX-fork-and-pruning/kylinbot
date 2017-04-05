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
 
#ifndef __MSG_H__
#define __MSG_H__

/********************************************************/
/*                 KylinBot Msg Type                    */
/*     Basic frame structure:                           */
/*       ________________________________________       */
/*      |id:8|length:8|token:16|data~|checksum:16|      */
/*      |______________________|_____|___________|      */
/*      |         head         |body~|    crc    |      */
/*      |________________________________________|      */
/*                        Msg                           */
/********************************************************/

#ifdef __cplusplus
extern "C" {
#endif

#include "cbus.h"
#include "dbus.h"
#include "fifo.h"
#include "calib.h"
#include "crc16.h"
	
/* Message length minimum/maximum limit */
#define MSG_LEN_MIN 6u
#define MSG_LEN_MAX 256u
#define MSG_LEN_EXT 6u

#pragma pack(1)

/* Message head union typedef */
typedef union MsgHead_t
{
	uint32_t value; // Message head value in 32bit
	struct
	{
		uint8_t id : 8; // Message ID
		uint8_t length : 8; // Message length (unit: byte)
		uint16_t token : 16; // Message CRC token
	}attr; // Message head attributes
}MsgHead_t; // Message head union typedef

typedef struct
{
	uint32_t frame_id;
	uint8_t data[RCP_FRAME_LEN];
}RcpMsg_t;

typedef struct
{
	uint32_t frame_id;
	uint8_t data[HCP_FRAME_LEN];
}HcpMsg_t;

typedef struct
{
	uint32_t frame_id;
	uint8_t data[DBUS_FRAME_LEN];
}DBusMsg_t;

typedef struct
{
	uint32_t frame_id;
	uint8_t data[CBUS_FRAME_LEN];
}CBusMsg_t;

#define MOTOR1_ID 0
#define MOTOR2_ID 1
#define MOTOR3_ID 2
#define MOTOR4_ID 3
#define MOTOR5_ID 4
#define MOTOR6_ID 5
#define MOTOR_ECD_ANGLE_MAX 8191
#define MOTOR_ESC_CURRENT_MAX 13000
typedef struct
{
	uint8_t id; // 0~5
	uint32_t frame_id;
	uint16_t ecd_angle; // Encoder angle, range from 0~8191
	int32_t angle; // Continuous angle, infinite
	int16_t rate; // Rate in ecd_diff/1ms
	int32_t current; // Motor current, max to 13000
}MotorMsg_t;

#define IMU9X_MSG_VALUE_SCALE 1.0f
typedef struct
{
	uint32_t frame_id;
	int16_t ax;
	int16_t ay;
	int16_t az;
	int16_t gx;
	int16_t gy;
	int16_t gz;
	int16_t mx;
	int16_t my;
	int16_t mz;
}IMU9XMsg_t;

#define ZGYRO_ANGLE_RECIP 1e-2f // To scale zgyro angle to deg
#define ZGYRO_RATE_RECIP 1e-5f // To scale zgyro rate to deg/s
typedef struct
{
	uint32_t frame_id;
	int32_t angle; // = (deg*100)
	int16_t rate; // = delta(angle)/1ms
}ZGyroMsg_t;

#define SONAR_NUM 4
#define SONAR_IDX_F 0
#define SONAR_IDX_M 1
#define SONAR_IDX_L 2
#define SONAR_IDX_R 3
typedef struct
{
	uint32_t frame_id;
	int16_t data[SONAR_NUM];
}SonarMsg_t;

#define ODOME_MSG_VALUE_SCALE 1e3f
typedef struct
{
	uint32_t frame_id;
	int32_t px; // Bot position (linear) in x-axis, unit: mm
	int32_t py; // Bot position (linear) in y-axis, unit: mm
	int32_t pz; // Bot position (angular) in z-axis, unit: 1e-3rad
	int16_t vx; // Bot velocity (linear) in x-axis, unit: mm/s
	int16_t vy; // Bot velocity (linear) in y-axis, unit: mm/s
	int16_t vz; // Bot velocity (angular) in z-axis, unit: 1e-3rad/s
}OdomeMsg_t;

#define GRASP_MSG_VALUE_SCALE 1e3f
typedef struct
{
	uint32_t frame_id;
	int16_t pe; // Elevator position
	int16_t ve; // Elevator velocity
	int16_t pc; // Claw position
	int16_t vc; // Claw velocity
}GraspMsg_t;

#define CALIB_FLAG_BIT_IMU (1u<<0)
#define CALIB_FLAG_BIT_MAG (1u<<1)
#define CALIB_FLAG_BIT_GIM (1u<<2)
#define CALIB_FLAG_BIT_ODO (1u<<3)
typedef struct
{
	uint32_t frame_id;
	uint32_t auto_cali_flag; // Auto calibration control bits
}CalibMsg_t;

typedef struct
{
	uint32_t frame_id;
	uint32_t wdg; // Watchdog
	uint32_t ini; // Initialization status
}StatuMsg_t;

typedef struct
{
	uint32_t frame_id;
	uint32_t msg_type;
}SubscMsg_t;

typedef struct
{
	uint32_t frame_id;
	CBus_t cbus; // Control bus
}KylinMsg_t;

typedef struct
{
	uint32_t frame_id;
	PIDCalib_t data;
}PIDCalibMsg_t;

typedef struct
{
	uint32_t frame_id;
	IMUCalib_t data;
}IMUCalibMsg_t;

typedef struct
{
	uint32_t frame_id;
	MagCalib_t data;
}MagCalibMsg_t;

typedef struct
{
	uint32_t frame_id;
	VelCalib_t data;
}VelCalibMsg_t;

typedef struct
{
	uint32_t frame_id;
	MecCalib_t data;
}MecCalibMsg_t;

typedef struct
{
	uint32_t frame_id;
	PosCalib_t data;
}PosCalibMsg_t;

typedef struct
{
	uint32_t frame_id;
	EpsCalib_t data;
}EpsCalibMsg_t;

#define MSG_WRAP_U8(V) ((uint8_t)V)
#define MSG_WRAP_U16(V) ((uint16_t)V)
#define MSG_WRAP_U32(V) ((uint32_t)V)

#define MSG_ID_RCP MSG_WRAP_U8(0x01)
#define MSG_ID_HCP MSG_WRAP_U8(0x02)
#define MSG_ID_DBUS MSG_WRAP_U8(0x03)
#define MSG_ID_CBUS MSG_WRAP_U8(0x04)
#define MSG_ID_ZGYRO MSG_WRAP_U8(0x05)
#define MSG_ID_IMU9X MSG_WRAP_U8(0x06)
#define MSG_ID_MOTOR MSG_WRAP_U8(0x07)
#define MSG_ID_ODOME MSG_WRAP_U8(0x08)
#define MSG_ID_GRASP MSG_WRAP_U8(0x09)
#define MSG_ID_STATU MSG_WRAP_U8(0x0a)
#define MSG_ID_SUBSC MSG_WRAP_U8(0x0b)
#define MSG_ID_CALIB MSG_WRAP_U8(0x0c)
#define MSG_ID_KYLIN MSG_WRAP_U8(0x0d)
#define MSG_ID_SONAR MSG_WRAP_U8(0x0e)
#define MSG_ID_PID_CALIB MSG_WRAP_U8(0x0f)
#define MSG_ID_IMU_CALIB MSG_WRAP_U8(0x10)
#define MSG_ID_MAG_CALIB MSG_WRAP_U8(0x11)
#define MSG_ID_VEL_CALIB MSG_WRAP_U8(0x12)
#define MSG_ID_MEC_CALIB MSG_WRAP_U8(0x13)
#define MSG_ID_POS_CALIB MSG_WRAP_U8(0x14)
#define MSG_ID_EPS_CALIB MSG_WRAP_U8(0x15)

#define MSG_LEN_RCP sizeof(RcpMsg_t)
#define MSG_LEN_HCP sizeof(HcpMsg_t)
#define MSG_LEN_DBUS sizeof(DBusMsg_t)
#define MSG_LEN_CBUS sizeof(CBusMsg_t)
#define MSG_LEN_ZGYRO sizeof(ZGyroMsg_t)
#define MSG_LEN_IMU9X sizeof(IMU9XMsg_t)
#define MSG_LEN_MOTOR sizeof(MotorMsg_t)
#define MSG_LEN_ODOME sizeof(OdomeMsg_t)
#define MSG_LEN_GRASP sizeof(GraspMsg_t)
#define MSG_LEN_STATU sizeof(StatuMsg_t)
#define MSG_LEN_SUBSC sizeof(SubscMsg_t)
#define MSG_LEN_CALIB sizeof(CalibMsg_t)
#define MSG_LEN_KYLIN sizeof(KylinMsg_t)
#define MSG_LEN_SONAR sizeof(SonarMsg_t)
#define MSG_LEN_PID_CALIB sizeof(PIDCalibMsg_t)
#define MSG_LEN_IMU_CALIB sizeof(IMUCalibMsg_t)
#define MSG_LEN_MAG_CALIB sizeof(MagCalibMsg_t)
#define MSG_LEN_VEL_CALIB sizeof(VelCalibMsg_t)
#define MSG_LEN_MEC_CALIB sizeof(MecCalibMsg_t)
#define MSG_LEN_POS_CALIB sizeof(PosCalibMsg_t)
#define MSG_LEN_EPS_CALIB sizeof(EpsCalibMsg_t)
	
#define MSG_TOKEN_RCP MSG_WRAP_U16(0x1234)
#define MSG_TOKEN_HCP MSG_WRAP_U16(0x2345)
#define MSG_TOKEN_DBUS MSG_WRAP_U16(0x3456)
#define MSG_TOKEN_CBUS MSG_WRAP_U16(0x4567)
#define MSG_TOKEN_ZGYRO MSG_WRAP_U16(0x5678)
#define MSG_TOKEN_IMU9X MSG_WRAP_U16(0x6789)
#define MSG_TOKEN_MOTOR MSG_WRAP_U16(0x789a)
#define MSG_TOKEN_ODOME MSG_WRAP_U16(0x89ab)
#define MSG_TOKEN_GRASP MSG_WRAP_U16(0x9abc)
#define MSG_TOKEN_STATU MSG_WRAP_U16(0xabcd)
#define MSG_TOKEN_SUBSC MSG_WRAP_U16(0xbcde)
#define MSG_TOKEN_CALIB MSG_WRAP_U16(0xcdef)
#define MSG_TOKEN_KYLIN MSG_WRAP_U16(0xfedc)
#define MSG_TOKEN_SONAR MSG_WRAP_U16(0xedcb)
#define MSG_TOKEN_PID_CALIB MSG_WRAP_U16(0xdcba)
#define MSG_TOKEN_IMU_CALIB MSG_WRAP_U16(0xcba9)
#define MSG_TOKEN_MAG_CALIB MSG_WRAP_U16(0xba98)
#define MSG_TOKEN_VEL_CALIB MSG_WRAP_U16(0xa987)
#define MSG_TOKEN_MEC_CALIB MSG_WRAP_U16(0x9876)
#define MSG_TOKEN_POS_CALIB MSG_WRAP_U16(0x8765)
#define MSG_TOKEN_EPS_CALIB MSG_WRAP_U16(0x7654)

#define MSG_HEAD_VALUE(ID,LEN,TOKEN) ((MSG_WRAP_U32(TOKEN)<<16) | (MSG_WRAP_U32(LEN)<<8) | MSG_WRAP_U32(ID))
#define MSG_HEAD_VALUE_OF(NAME) MSG_HEAD_VALUE(MSG_ID_##NAME,MSG_LEN_##NAME,MSG_TOKEN_##NAME)

#define MSG_HEAD_VALUE_RCP MSG_HEAD_VALUE_OF(RCP)
#define MSG_HEAD_VALUE_HCP MSG_HEAD_VALUE_OF(HCP)
#define MSG_HEAD_VALUE_DBUS MSG_HEAD_VALUE_OF(DBUS)
#define MSG_HEAD_VALUE_CBUS MSG_HEAD_VALUE_OF(CBUS)
#define MSG_HEAD_VALUE_ZGYRO MSG_HEAD_VALUE_OF(ZGYRO)
#define MSG_HEAD_VALUE_IMU9X MSG_HEAD_VALUE_OF(IMU9X)
#define MSG_HEAD_VALUE_MOTOR MSG_HEAD_VALUE_OF(MOTOR)
#define MSG_HEAD_VALUE_ODOME MSG_HEAD_VALUE_OF(ODOME)
#define MSG_HEAD_VALUE_GRASP MSG_HEAD_VALUE_OF(GRASP)
#define MSG_HEAD_VALUE_STATU MSG_HEAD_VALUE_OF(STATU)
#define MSG_HEAD_VALUE_SUBSC MSG_HEAD_VALUE_OF(SUBSC)
#define MSG_HEAD_VALUE_CALIB MSG_HEAD_VALUE_OF(CALIB)
#define MSG_HEAD_VALUE_KYLIN MSG_HEAD_VALUE_OF(KYLIN)
#define MSG_HEAD_VALUE_SONAR MSG_HEAD_VALUE_OF(SONAR)
#define MSG_HEAD_VALUE_PID_CALIB MSG_HEAD_VALUE_OF(PID_CALIB)
#define MSG_HEAD_VALUE_IMU_CALIB MSG_HEAD_VALUE_OF(IMU_CALIB)
#define MSG_HEAD_VALUE_MAG_CALIB MSG_HEAD_VALUE_OF(MAG_CALIB)
#define MSG_HEAD_VALUE_VEL_CALIB MSG_HEAD_VALUE_OF(VEL_CALIB)
#define MSG_HEAD_VALUE_MEC_CALIB MSG_HEAD_VALUE_OF(MEC_CALIB)
#define MSG_HEAD_VALUE_POS_CALIB MSG_HEAD_VALUE_OF(POS_CALIB)
#define MSG_HEAD_VALUE_EPS_CALIB MSG_HEAD_VALUE_OF(EPS_CALIB)

#define MSG_HEAD_RCP { MSG_HEAD_VALUE_RCP }
#define MSG_HEAD_HCP { MSG_HEAD_VALUE_HCP }
#define MSG_HEAD_DBUS { MSG_HEAD_VALUE_DBUS }
#define MSG_HEAD_CBUS { MSG_HEAD_VALUE_CBUS }
#define MSG_HEAD_ZGYRO { MSG_HEAD_VALUE_ZGYRO }
#define MSG_HEAD_IMU9X { MSG_HEAD_VALUE_IMU9X }
#define MSG_HEAD_MOTOR { MSG_HEAD_VALUE_MOTOR }
#define MSG_HEAD_ODOME { MSG_HEAD_VALUE_ODOME }
#define MSG_HEAD_GRASP { MSG_HEAD_VALUE_GRASP }
#define MSG_HEAD_STATU { MSG_HEAD_VALUE_STATU }
#define MSG_HEAD_SUBSC { MSG_HEAD_VALUE_SUBSC }
#define MSG_HEAD_CALIB { MSG_HEAD_VALUE_CALIB }
#define MSG_HEAD_KYLIN { MSG_HEAD_VALUE_KYLIN }
#define MSG_HEAD_SONAR { MSG_HEAD_VALUE_SONAR }
#define MSG_HEAD_PID_CALIB { MSG_HEAD_VALUE_PID_CALIB }
#define MSG_HEAD_IMU_CALIB { MSG_HEAD_VALUE_IMU_CALIB }
#define MSG_HEAD_MAG_CALIB { MSG_HEAD_VALUE_MAG_CALIB }
#define MSG_HEAD_VEL_CALIB { MSG_HEAD_VALUE_VEL_CALIB }
#define MSG_HEAD_MEC_CALIB { MSG_HEAD_VALUE_MEC_CALIB }
#define MSG_HEAD_POS_CALIB { MSG_HEAD_VALUE_POS_CALIB }
#define MSG_HEAD_EPS_CALIB { MSG_HEAD_VALUE_EPS_CALIB }

#define MSG_TYPE_IDX_RCP 0u
#define MSG_TYPE_IDX_HCP 1u
#define MSG_TYPE_IDX_DBUS 2u
#define MSG_TYPE_IDX_CBUS 3u
#define MSG_TYPE_IDX_ZGYRO 4u
#define MSG_TYPE_IDX_IMU9X 5u
#define MSG_TYPE_IDX_MOTOR 6u
#define MSG_TYPE_IDX_ODOME 7u
#define MSG_TYPE_IDX_GRASP 8u
#define MSG_TYPE_IDX_STATU 9u
#define MSG_TYPE_IDX_SUBSC 10u
#define MSG_TYPE_IDX_CALIB 11u
#define MSG_TYPE_IDX_KYLIN 12u
#define MSG_TYPE_IDX_SONAR 13u
#define MSG_TYPE_IDX_PID_CALIB 14u
#define MSG_TYPE_IDX_IMU_CALIB 15u
#define MSG_TYPE_IDX_MAG_CALIB 16u
#define MSG_TYPE_IDX_VEL_CALIB 17u
#define MSG_TYPE_IDX_MEC_CALIB 18u
#define MSG_TYPE_IDX_POS_CALIB 19u
#define MSG_TYPE_IDX_EPS_CALIB 20u

typedef enum
{
	MSG_TYPE_RCP = 1u << MSG_TYPE_IDX_RCP,
	MSG_TYPE_HCP = 1u << MSG_TYPE_IDX_HCP,
	MSG_TYPE_DBUS = 1u << MSG_TYPE_IDX_DBUS,
	MSG_TYPE_CBUS = 1u << MSG_TYPE_IDX_CBUS,
	MSG_TYPE_ZGYRO = 1u << MSG_TYPE_IDX_ZGYRO,
	MSG_TYPE_IMU9X = 1u << MSG_TYPE_IDX_IMU9X,
	MSG_TYPE_MOTOR = 1u << MSG_TYPE_IDX_MOTOR,
	MSG_TYPE_ODOME = 1u << MSG_TYPE_IDX_ODOME,
	MSG_TYPE_GRASP = 1u << MSG_TYPE_IDX_GRASP,
	MSG_TYPE_STATU = 1u << MSG_TYPE_IDX_STATU,
	MSG_TYPE_SUBSC = 1u << MSG_TYPE_IDX_SUBSC,
	MSG_TYPE_CALIB = 1u << MSG_TYPE_IDX_CALIB,
	MSG_TYPE_KYLIN = 1u << MSG_TYPE_IDX_KYLIN,
	MSG_TYPE_SONAR = 1u << MSG_TYPE_IDX_SONAR,
	MSG_TYPE_PID_CALIB = 1u << MSG_TYPE_IDX_PID_CALIB,
	MSG_TYPE_IMU_CALIB = 1u << MSG_TYPE_IDX_IMU_CALIB,
	MSG_TYPE_MAG_CALIB = 1u << MSG_TYPE_IDX_MAG_CALIB,
	MSG_TYPE_VEL_CALIB = 1u << MSG_TYPE_IDX_VEL_CALIB,
	MSG_TYPE_MEC_CALIB = 1u << MSG_TYPE_IDX_MEC_CALIB,
	MSG_TYPE_POS_CALIB = 1u << MSG_TYPE_IDX_POS_CALIB,
	MSG_TYPE_EPS_CALIB = 1u << MSG_TYPE_IDX_EPS_CALIB,
}MsgType_t;

#pragma pack()

/**
 * @brief: Push a single message to message fifo. 
 * @param fifo Message fifo
 * @param buf Message buffer
 * @param head Message head
 * @param body Message body
 * @return Message length (num of bytes)
 */
uint32_t Msg_Push(FIFO_t* fifo, void* buf, const void* head, const void* body);

/**
 * @brief Pop a single message from message fifo. 
 * @param fifo Message fifo
 * @param buf Message buffer
 * @param head Message head
 * @param body Message body
 * @return Message length (num of bytes)
 */
uint32_t Msg_Pop(FIFO_t* fifo, void* buf, const void* head, void* body);

extern const MsgHead_t msg_head_rcp;
extern const MsgHead_t msg_head_hcp;
extern const MsgHead_t msg_head_dbus;
extern const MsgHead_t msg_head_cbus;
extern const MsgHead_t msg_head_zgyro;
extern const MsgHead_t msg_head_imu9x;
extern const MsgHead_t msg_head_motor;
extern const MsgHead_t msg_head_odome;
extern const MsgHead_t msg_head_grasp;
extern const MsgHead_t msg_head_statu;
extern const MsgHead_t msg_head_subsc;
extern const MsgHead_t msg_head_calib;
extern const MsgHead_t msg_head_kylin;
extern const MsgHead_t msg_head_sonar;
extern const MsgHead_t msg_head_pid_calib;
extern const MsgHead_t msg_head_imu_calib;
extern const MsgHead_t msg_head_mag_calib;
extern const MsgHead_t msg_head_vel_calib;
extern const MsgHead_t msg_head_mec_calib;
extern const MsgHead_t msg_head_pos_calib;
extern const MsgHead_t msg_head_eps_calib;

#ifdef __cplusplus
}
#endif

#endif


