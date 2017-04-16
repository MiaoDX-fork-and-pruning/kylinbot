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

#ifndef __CBUS_H__
#define __CBUS_H__

/**************************************************/
/*             Kylinbot Control Bus               */
/**************************************************/

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <string.h>

typedef struct
{
	int16_t x;
	int16_t y;
	int16_t z;
}ChassisState_c; // Chassis state control typedef

typedef struct
{
	int16_t e;
	int16_t c;
}GrabberState_c; // Grabber state control typedef

#define CBUS_FLAG_BIT_INI (1u<<31) // Initialization flag bit
#define CBUS_FLAG_BIT_ABS (1u<<30) // Absolute control flag bit
#define CBUS_FLAG_BIT_PXC (1u<<29)  // Position X error convergence flag bit
#define CBUS_FLAG_BIT_PYC (1u<<28)  // Position Y error convergence flag bit
#define CBUS_FLAG_BIT_PZC (1u<<27)  // Position Z error convergence flag bit
#define CBUS_FLAG_BIT_PEC (1u<<26)  // Position E error convergence flag bit
#define CBUS_FLAG_BIT_PCC (1u<<25)  // Position C error convergence flag bit
#define CBUS_FLAG_BIT_PSC (CBUS_FLAG_BIT_PXC | CBUS_FLAG_BIT_PYC | CBUS_FLAG_BIT_PZC | CBUS_FLAG_BIT_PEC | CBUS_FLAG_BIT_PCC) // All position convergence flag bits
#define CBUS_VALUE_SCALE 1e3f
#pragma pack(1)
typedef struct
{
	uint32_t fs; // Flag bits
	ChassisState_c cv; // Chassis velocity, unit: linear: mm/s, angular: 1e-3rad/s
	ChassisState_c cp; // Chassis position, unit: linear: mm, angular: 1e-3rad
	GrabberState_c gv; // Grabber velocity, unit: linear: mm/s, angular: 1e-3rad/s
	GrabberState_c gp; // Grabber position, unit: linear: mm, angular: rad
}CBus_t;

#define CBUS_FRAME_LEN sizeof(CBus_t)

#pragma pack()

void CBus_Enc(const CBus_t* cbus, uint8_t* buf);
void CBus_Dec(CBus_t* cbus, const uint8_t* buf);
void CBus_Init(CBus_t* cbus);

#ifdef __cplusplus
}
#endif

#endif
