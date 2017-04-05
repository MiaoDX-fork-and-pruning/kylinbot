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

#include "cbus.h"

/**************************************************/
/*             Kylinbot Control Bus               */
/**************************************************/

void CBus_Enc(const CBus_t* cbus, uint8_t* buf)
{
	memcpy(buf, cbus, sizeof(CBus_t));
}

void CBus_Dec(CBus_t* cbus, const uint8_t* buf)
{
	memcpy(cbus, buf, sizeof(CBus_t));
}

void CBus_Init(CBus_t* cbus)
{
	memset(cbus, 0, sizeof(CBus_t));
}


