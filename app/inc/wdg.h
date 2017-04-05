/**
 * Copyright (c) 2016, Jack Mo (mobangjack@foxmail.com).
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

#ifndef __WDG_H__
#define __WDG_H__

/***********************************/
/*            Watch Dog            */
/***********************************/

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef uint32_t WdgErr_t;
	
#define WDG_NUM                          16u

#define WDG_IDX_RCV                      0u
#define WDG_IDX_TTY                      1u
#define WDG_IDX_BTM                      2u
#define WDG_IDX_DBI                      3u
#define WDG_IDX_IMU                      4u
#define WDG_IDX_ZGYRO                    5u
#define WDG_IDX_MOTOR1                   6u
#define WDG_IDX_MOTOR2                   7u
#define WDG_IDX_MOTOR3                   8u
#define WDG_IDX_MOTOR4                   9u
#define WDG_IDX_MOTOR5                   10u
#define WDG_IDX_MOTOR6                   11u
#define WDG_IDX_SONAR1                   12u
#define WDG_IDX_SONAR2                   13u
#define WDG_IDX_SONAR3                   14u
#define WDG_IDX_SONAR4                   15u

#define WDG_ERR_RCV                      (1u<<WDG_IDX_RCV)
#define WDG_ERR_TTY                      (1u<<WDG_IDX_TTY)
#define WDG_ERR_BTM                      (1u<<WDG_IDX_BTM)
#define WDG_ERR_DBI                      (1u<<WDG_IDX_DBI)
#define WDG_ERR_IMU                      (1u<<WDG_IDX_IMU)
#define WDG_ERR_ZGYRO                    (1u<<WDG_IDX_ZGYRO)
#define WDG_ERR_MOTOR1                   (1u<<WDG_IDX_MOTOR1)
#define WDG_ERR_MOTOR2                   (1u<<WDG_IDX_MOTOR2)
#define WDG_ERR_MOTOR3                   (1u<<WDG_IDX_MOTOR3)
#define WDG_ERR_MOTOR4                   (1u<<WDG_IDX_MOTOR4)
#define WDG_ERR_MOTOR5                   (1u<<WDG_IDX_MOTOR5)
#define WDG_ERR_MOTOR6                   (1u<<WDG_IDX_MOTOR6)
#define WDG_ERR_SONAR1                   (1u<<WDG_IDX_SONAR1)
#define WDG_ERR_SONAR2                   (1u<<WDG_IDX_SONAR2)
#define WDG_ERR_SONAR3                   (1u<<WDG_IDX_SONAR3)
#define WDG_ERR_SONAR4                   (1u<<WDG_IDX_SONAR4)

#define WDG_ERR_ALL ( \
		WDG_ERR_RCV | \
		WDG_ERR_TTY | \
		WDG_ERR_BTM | \
		WDG_ERR_DBI | \
		WDG_ERR_IMU | \
		WDG_ERR_ZGYRO | \
		WDG_ERR_MOTOR1 | \
		WDG_ERR_MOTOR2 | \
		WDG_ERR_MOTOR3 | \
		WDG_ERR_MOTOR4 | \
		WDG_ERR_MOTOR5 | \
		WDG_ERR_MOTOR6 | \
		WDG_ERR_SONAR1 | \
		WDG_ERR_SONAR2 | \
		WDG_ERR_SONAR3 | \
		WDG_ERR_SONAR4 \
		)

#define WDG_ERR_SIGNAL (WDG_ERR_RCV)
#define WDG_ERR_MOTORS (WDG_ERR_MOTOR1 | WDG_ERR_MOTOR2 | WDG_ERR_MOTOR3 | WDG_ERR_MOTOR4 | WDG_ERR_MOTOR6)

#define WDG_ERR_FATAL ( \
		WDG_ERR_RCV | \
		WDG_ERR_MOTOR1 | \
		WDG_ERR_MOTOR2 | \
		WDG_ERR_MOTOR3 | \
		WDG_ERR_MOTOR4 \
		)

#define WDG_OVERFLOW_CNT_RCV   	  100u
#define WDG_OVERFLOW_CNT_TTY  		100u
#define WDG_OVERFLOW_CNT_BTM  		100u
#define WDG_OVERFLOW_CNT_DBI  		100u
#define WDG_OVERFLOW_CNT_IMU  		100u
#define WDG_OVERFLOW_CNT_ZGYRO 		100u
#define WDG_OVERFLOW_CNT_MOTOR 		100u
#define WDG_OVERFLOW_CNT_SONAR 		200u

#define WDG_OVERFLOW_CNT_DEFAULT \
{ \
		WDG_OVERFLOW_CNT_RCV, \
		WDG_OVERFLOW_CNT_TTY, \
		WDG_OVERFLOW_CNT_BTM, \
	  WDG_OVERFLOW_CNT_DBI, \
		WDG_OVERFLOW_CNT_IMU, \
		WDG_OVERFLOW_CNT_ZGYRO, \
		WDG_OVERFLOW_CNT_MOTOR, \
		WDG_OVERFLOW_CNT_MOTOR, \
		WDG_OVERFLOW_CNT_MOTOR, \
		WDG_OVERFLOW_CNT_MOTOR, \
		WDG_OVERFLOW_CNT_MOTOR, \
		WDG_OVERFLOW_CNT_MOTOR, \
	  WDG_OVERFLOW_CNT_SONAR, \
    WDG_OVERFLOW_CNT_SONAR, \
    WDG_OVERFLOW_CNT_SONAR, \
    WDG_OVERFLOW_CNT_SONAR, \
}

void Wdg_Init(void);
void Wdg_Proc(void);
void Wdg_Feed(uint8_t i);
void Wdg_Hang(uint8_t i);
WdgErr_t Wdg_GetErr(void);
WdgErr_t Wdg_HasErr(WdgErr_t mask);
WdgErr_t Wdg_HitErr(WdgErr_t mask);
WdgErr_t Wdg_IsOkay(void);

#ifdef __cplusplus
}
#endif

#endif

