// Copyright 2020 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#ifndef _SOC_ASSIST_DEBUG_REG_H_
#define _SOC_ASSIST_DEBUG_REG_H_


#ifdef __cplusplus
extern "C" {
#endif
#include "soc.h"
#define ASSIST_DEBUG_CORE_0_INTR_ENA_REG          (DR_REG_ASSIST_DEBUG_BASE + 0x000)
/* ASSIST_DEBUG_CORE_0_DRAM0_EXCEPTION_MONITOR_ENA : R/W ;bitpos:[11] ;default: 1'b0 ; */
/*description: */
#define ASSIST_DEBUG_CORE_0_DRAM0_EXCEPTION_MONITOR_ENA  (BIT(11))
#define ASSIST_DEBUG_CORE_0_DRAM0_EXCEPTION_MONITOR_ENA_M  (BIT(11))
#define ASSIST_DEBUG_CORE_0_DRAM0_EXCEPTION_MONITOR_ENA_V  0x1
#define ASSIST_DEBUG_CORE_0_DRAM0_EXCEPTION_MONITOR_ENA_S  11
/* ASSIST_DEBUG_CORE_0_IRAM0_EXCEPTION_MONITOR_ENA : R/W ;bitpos:[10] ;default: 1'b0 ; */
/*description: */
#define ASSIST_DEBUG_CORE_0_IRAM0_EXCEPTION_MONITOR_ENA  (BIT(10))
#define ASSIST_DEBUG_CORE_0_IRAM0_EXCEPTION_MONITOR_ENA_M  (BIT(10))
#define ASSIST_DEBUG_CORE_0_IRAM0_EXCEPTION_MONITOR_ENA_V  0x1
#define ASSIST_DEBUG_CORE_0_IRAM0_EXCEPTION_MONITOR_ENA_S  10
/* ASSIST_DEBUG_CORE_0_SP_SPILL_MAX_ENA : R/W ;bitpos:[9] ;default: 1'b0 ; */
/*description: */
#define ASSIST_DEBUG_CORE_0_SP_SPILL_MAX_ENA  (BIT(9))
#define ASSIST_DEBUG_CORE_0_SP_SPILL_MAX_ENA_M  (BIT(9))
#define ASSIST_DEBUG_CORE_0_SP_SPILL_MAX_ENA_V  0x1
#define ASSIST_DEBUG_CORE_0_SP_SPILL_MAX_ENA_S  9
/* ASSIST_DEBUG_CORE_0_SP_SPILL_MIN_ENA : R/W ;bitpos:[8] ;default: 1'b0 ; */
/*description: */
#define ASSIST_DEBUG_CORE_0_SP_SPILL_MIN_ENA  (BIT(8))
#define ASSIST_DEBUG_CORE_0_SP_SPILL_MIN_ENA_M  (BIT(8))
#define ASSIST_DEBUG_CORE_0_SP_SPILL_MIN_ENA_V  0x1
#define ASSIST_DEBUG_CORE_0_SP_SPILL_MIN_ENA_S  8
/* ASSIST_DEBUG_CORE_0_AREA_PIF_1_WR_ENA : R/W ;bitpos:[7] ;default: 1'b0 ; */
/*description: */
#define ASSIST_DEBUG_CORE_0_AREA_PIF_1_WR_ENA  (BIT(7))
#define ASSIST_DEBUG_CORE_0_AREA_PIF_1_WR_ENA_M  (BIT(7))
#define ASSIST_DEBUG_CORE_0_AREA_PIF_1_WR_ENA_V  0x1
#define ASSIST_DEBUG_CORE_0_AREA_PIF_1_WR_ENA_S  7
/* ASSIST_DEBUG_CORE_0_AREA_PIF_1_RD_ENA : R/W ;bitpos:[6] ;default: 1'b0 ; */
/*description: */
#define ASSIST_DEBUG_CORE_0_AREA_PIF_1_RD_ENA  (BIT(6))
#define ASSIST_DEBUG_CORE_0_AREA_PIF_1_RD_ENA_M  (BIT(6))
#define ASSIST_DEBUG_CORE_0_AREA_PIF_1_RD_ENA_V  0x1
#define ASSIST_DEBUG_CORE_0_AREA_PIF_1_RD_ENA_S  6
/* ASSIST_DEBUG_CORE_0_AREA_PIF_0_WR_ENA : R/W ;bitpos:[5] ;default: 1'b0 ; */
/*description: */
#define ASSIST_DEBUG_CORE_0_AREA_PIF_0_WR_ENA  (BIT(5))
#define ASSIST_DEBUG_CORE_0_AREA_PIF_0_WR_ENA_M  (BIT(5))
#define ASSIST_DEBUG_CORE_0_AREA_PIF_0_WR_ENA_V  0x1
#define ASSIST_DEBUG_CORE_0_AREA_PIF_0_WR_ENA_S  5
/* ASSIST_DEBUG_CORE_0_AREA_PIF_0_RD_ENA : R/W ;bitpos:[4] ;default: 1'b0 ; */
/*description: */
#define ASSIST_DEBUG_CORE_0_AREA_PIF_0_RD_ENA  (BIT(4))
#define ASSIST_DEBUG_CORE_0_AREA_PIF_0_RD_ENA_M  (BIT(4))
#define ASSIST_DEBUG_CORE_0_AREA_PIF_0_RD_ENA_V  0x1
#define ASSIST_DEBUG_CORE_0_AREA_PIF_0_RD_ENA_S  4
/* ASSIST_DEBUG_CORE_0_AREA_DRAM0_1_WR_ENA : R/W ;bitpos:[3] ;default: 1'b0 ; */
/*description: */
#define ASSIST_DEBUG_CORE_0_AREA_DRAM0_1_WR_ENA  (BIT(3))
#define ASSIST_DEBUG_CORE_0_AREA_DRAM0_1_WR_ENA_M  (BIT(3))
#define ASSIST_DEBUG_CORE_0_AREA_DRAM0_1_WR_ENA_V  0x1
#define ASSIST_DEBUG_CORE_0_AREA_DRAM0_1_WR_ENA_S  3
/* ASSIST_DEBUG_CORE_0_AREA_DRAM0_1_RD_ENA : R/W ;bitpos:[2] ;default: 1'b0 ; */
/*description: */
#define ASSIST_DEBUG_CORE_0_AREA_DRAM0_1_RD_ENA  (BIT(2))
#define ASSIST_DEBUG_CORE_0_AREA_DRAM0_1_RD_ENA_M  (BIT(2))
#define ASSIST_DEBUG_CORE_0_AREA_DRAM0_1_RD_ENA_V  0x1
#define ASSIST_DEBUG_CORE_0_AREA_DRAM0_1_RD_ENA_S  2
/* ASSIST_DEBUG_CORE_0_AREA_DRAM0_0_WR_ENA : R/W ;bitpos:[1] ;default: 1'b0 ; */
/*description: */
#define ASSIST_DEBUG_CORE_0_AREA_DRAM0_0_WR_ENA  (BIT(1))
#define ASSIST_DEBUG_CORE_0_AREA_DRAM0_0_WR_ENA_M  (BIT(1))
#define ASSIST_DEBUG_CORE_0_AREA_DRAM0_0_WR_ENA_V  0x1
#define ASSIST_DEBUG_CORE_0_AREA_DRAM0_0_WR_ENA_S  1
/* ASSIST_DEBUG_CORE_0_AREA_DRAM0_0_RD_ENA : R/W ;bitpos:[0] ;default: 1'b0 ; */
/*description: */
#define ASSIST_DEBUG_CORE_0_AREA_DRAM0_0_RD_ENA  (BIT(0))
#define ASSIST_DEBUG_CORE_0_AREA_DRAM0_0_RD_ENA_M  (BIT(0))
#define ASSIST_DEBUG_CORE_0_AREA_DRAM0_0_RD_ENA_V  0x1
#define ASSIST_DEBUG_CORE_0_AREA_DRAM0_0_RD_ENA_S  0

#define ASSIST_DEBUG_CORE_0_INTR_RAW_REG          (DR_REG_ASSIST_DEBUG_BASE + 0x004)
/* ASSIST_DEBUG_CORE_0_DRAM0_EXCEPTION_MONITOR_RAW : RO ;bitpos:[11] ;default: 1'b0 ; */
/*description: */
#define ASSIST_DEBUG_CORE_0_DRAM0_EXCEPTION_MONITOR_RAW  (BIT(11))
#define ASSIST_DEBUG_CORE_0_DRAM0_EXCEPTION_MONITOR_RAW_M  (BIT(11))
#define ASSIST_DEBUG_CORE_0_DRAM0_EXCEPTION_MONITOR_RAW_V  0x1
#define ASSIST_DEBUG_CORE_0_DRAM0_EXCEPTION_MONITOR_RAW_S  11
/* ASSIST_DEBUG_CORE_0_IRAM0_EXCEPTION_MONITOR_RAW : RO ;bitpos:[10] ;default: 1'b0 ; */
/*description: */
#define ASSIST_DEBUG_CORE_0_IRAM0_EXCEPTION_MONITOR_RAW  (BIT(10))
#define ASSIST_DEBUG_CORE_0_IRAM0_EXCEPTION_MONITOR_RAW_M  (BIT(10))
#define ASSIST_DEBUG_CORE_0_IRAM0_EXCEPTION_MONITOR_RAW_V  0x1
#define ASSIST_DEBUG_CORE_0_IRAM0_EXCEPTION_MONITOR_RAW_S  10
/* ASSIST_DEBUG_CORE_0_SP_SPILL_MAX_RAW : RO ;bitpos:[9] ;default: 1'b0 ; */
/*description: */
#define ASSIST_DEBUG_CORE_0_SP_SPILL_MAX_RAW  (BIT(9))
#define ASSIST_DEBUG_CORE_0_SP_SPILL_MAX_RAW_M  (BIT(9))
#define ASSIST_DEBUG_CORE_0_SP_SPILL_MAX_RAW_V  0x1
#define ASSIST_DEBUG_CORE_0_SP_SPILL_MAX_RAW_S  9
/* ASSIST_DEBUG_CORE_0_SP_SPILL_MIN_RAW : RO ;bitpos:[8] ;default: 1'b0 ; */
/*description: */
#define ASSIST_DEBUG_CORE_0_SP_SPILL_MIN_RAW  (BIT(8))
#define ASSIST_DEBUG_CORE_0_SP_SPILL_MIN_RAW_M  (BIT(8))
#define ASSIST_DEBUG_CORE_0_SP_SPILL_MIN_RAW_V  0x1
#define ASSIST_DEBUG_CORE_0_SP_SPILL_MIN_RAW_S  8
/* ASSIST_DEBUG_CORE_0_AREA_PIF_1_WR_RAW : RO ;bitpos:[7] ;default: 1'b0 ; */
/*description: */
#define ASSIST_DEBUG_CORE_0_AREA_PIF_1_WR_RAW  (BIT(7))
#define ASSIST_DEBUG_CORE_0_AREA_PIF_1_WR_RAW_M  (BIT(7))
#define ASSIST_DEBUG_CORE_0_AREA_PIF_1_WR_RAW_V  0x1
#define ASSIST_DEBUG_CORE_0_AREA_PIF_1_WR_RAW_S  7
/* ASSIST_DEBUG_CORE_0_AREA_PIF_1_RD_RAW : RO ;bitpos:[6] ;default: 1'b0 ; */
/*description: */
#define ASSIST_DEBUG_CORE_0_AREA_PIF_1_RD_RAW  (BIT(6))
#define ASSIST_DEBUG_CORE_0_AREA_PIF_1_RD_RAW_M  (BIT(6))
#define ASSIST_DEBUG_CORE_0_AREA_PIF_1_RD_RAW_V  0x1
#define ASSIST_DEBUG_CORE_0_AREA_PIF_1_RD_RAW_S  6
/* ASSIST_DEBUG_CORE_0_AREA_PIF_0_WR_RAW : RO ;bitpos:[5] ;default: 1'b0 ; */
/*description: */
#define ASSIST_DEBUG_CORE_0_AREA_PIF_0_WR_RAW  (BIT(5))
#define ASSIST_DEBUG_CORE_0_AREA_PIF_0_WR_RAW_M  (BIT(5))
#define ASSIST_DEBUG_CORE_0_AREA_PIF_0_WR_RAW_V  0x1
#define ASSIST_DEBUG_CORE_0_AREA_PIF_0_WR_RAW_S  5
/* ASSIST_DEBUG_CORE_0_AREA_PIF_0_RD_RAW : RO ;bitpos:[4] ;default: 1'b0 ; */
/*description: */
#define ASSIST_DEBUG_CORE_0_AREA_PIF_0_RD_RAW  (BIT(4))
#define ASSIST_DEBUG_CORE_0_AREA_PIF_0_RD_RAW_M  (BIT(4))
#define ASSIST_DEBUG_CORE_0_AREA_PIF_0_RD_RAW_V  0x1
#define ASSIST_DEBUG_CORE_0_AREA_PIF_0_RD_RAW_S  4
/* ASSIST_DEBUG_CORE_0_AREA_DRAM0_1_WR_RAW : RO ;bitpos:[3] ;default: 1'b0 ; */
/*description: */
#define ASSIST_DEBUG_CORE_0_AREA_DRAM0_1_WR_RAW  (BIT(3))
#define ASSIST_DEBUG_CORE_0_AREA_DRAM0_1_WR_RAW_M  (BIT(3))
#define ASSIST_DEBUG_CORE_0_AREA_DRAM0_1_WR_RAW_V  0x1
#define ASSIST_DEBUG_CORE_0_AREA_DRAM0_1_WR_RAW_S  3
/* ASSIST_DEBUG_CORE_0_AREA_DRAM0_1_RD_RAW : RO ;bitpos:[2] ;default: 1'b0 ; */
/*description: */
#define ASSIST_DEBUG_CORE_0_AREA_DRAM0_1_RD_RAW  (BIT(2))
#define ASSIST_DEBUG_CORE_0_AREA_DRAM0_1_RD_RAW_M  (BIT(2))
#define ASSIST_DEBUG_CORE_0_AREA_DRAM0_1_RD_RAW_V  0x1
#define ASSIST_DEBUG_CORE_0_AREA_DRAM0_1_RD_RAW_S  2
/* ASSIST_DEBUG_CORE_0_AREA_DRAM0_0_WR_RAW : RO ;bitpos:[1] ;default: 1'b0 ; */
/*description: */
#define ASSIST_DEBUG_CORE_0_AREA_DRAM0_0_WR_RAW  (BIT(1))
#define ASSIST_DEBUG_CORE_0_AREA_DRAM0_0_WR_RAW_M  (BIT(1))
#define ASSIST_DEBUG_CORE_0_AREA_DRAM0_0_WR_RAW_V  0x1
#define ASSIST_DEBUG_CORE_0_AREA_DRAM0_0_WR_RAW_S  1
/* ASSIST_DEBUG_CORE_0_AREA_DRAM0_0_RD_RAW : RO ;bitpos:[0] ;default: 1'b0 ; */
/*description: */
#define ASSIST_DEBUG_CORE_0_AREA_DRAM0_0_RD_RAW  (BIT(0))
#define ASSIST_DEBUG_CORE_0_AREA_DRAM0_0_RD_RAW_M  (BIT(0))
#define ASSIST_DEBUG_CORE_0_AREA_DRAM0_0_RD_RAW_V  0x1
#define ASSIST_DEBUG_CORE_0_AREA_DRAM0_0_RD_RAW_S  0

#define ASSIST_DEBUG_CORE_0_INTR_RLS_REG          (DR_REG_ASSIST_DEBUG_BASE + 0x008)
/* ASSIST_DEBUG_CORE_0_DRAM0_EXCEPTION_MONITOR_RLS : R/W ;bitpos:[11] ;default: 1'b0 ; */
/*description: */
#define ASSIST_DEBUG_CORE_0_DRAM0_EXCEPTION_MONITOR_RLS  (BIT(11))
#define ASSIST_DEBUG_CORE_0_DRAM0_EXCEPTION_MONITOR_RLS_M  (BIT(11))
#define ASSIST_DEBUG_CORE_0_DRAM0_EXCEPTION_MONITOR_RLS_V  0x1
#define ASSIST_DEBUG_CORE_0_DRAM0_EXCEPTION_MONITOR_RLS_S  11
/* ASSIST_DEBUG_CORE_0_IRAM0_EXCEPTION_MONITOR_RLS : R/W ;bitpos:[10] ;default: 1'b0 ; */
/*description: */
#define ASSIST_DEBUG_CORE_0_IRAM0_EXCEPTION_MONITOR_RLS  (BIT(10))
#define ASSIST_DEBUG_CORE_0_IRAM0_EXCEPTION_MONITOR_RLS_M  (BIT(10))
#define ASSIST_DEBUG_CORE_0_IRAM0_EXCEPTION_MONITOR_RLS_V  0x1
#define ASSIST_DEBUG_CORE_0_IRAM0_EXCEPTION_MONITOR_RLS_S  10
/* ASSIST_DEBUG_CORE_0_SP_SPILL_MAX_RLS : R/W ;bitpos:[9] ;default: 1'b0 ; */
/*description: */
#define ASSIST_DEBUG_CORE_0_SP_SPILL_MAX_RLS  (BIT(9))
#define ASSIST_DEBUG_CORE_0_SP_SPILL_MAX_RLS_M  (BIT(9))
#define ASSIST_DEBUG_CORE_0_SP_SPILL_MAX_RLS_V  0x1
#define ASSIST_DEBUG_CORE_0_SP_SPILL_MAX_RLS_S  9
/* ASSIST_DEBUG_CORE_0_SP_SPILL_MIN_RLS : R/W ;bitpos:[8] ;default: 1'b0 ; */
/*description: */
#define ASSIST_DEBUG_CORE_0_SP_SPILL_MIN_RLS  (BIT(8))
#define ASSIST_DEBUG_CORE_0_SP_SPILL_MIN_RLS_M  (BIT(8))
#define ASSIST_DEBUG_CORE_0_SP_SPILL_MIN_RLS_V  0x1
#define ASSIST_DEBUG_CORE_0_SP_SPILL_MIN_RLS_S  8
/* ASSIST_DEBUG_CORE_0_AREA_PIF_1_WR_RLS : R/W ;bitpos:[7] ;default: 1'b0 ; */
/*description: */
#define ASSIST_DEBUG_CORE_0_AREA_PIF_1_WR_RLS  (BIT(7))
#define ASSIST_DEBUG_CORE_0_AREA_PIF_1_WR_RLS_M  (BIT(7))
#define ASSIST_DEBUG_CORE_0_AREA_PIF_1_WR_RLS_V  0x1
#define ASSIST_DEBUG_CORE_0_AREA_PIF_1_WR_RLS_S  7
/* ASSIST_DEBUG_CORE_0_AREA_PIF_1_RD_RLS : R/W ;bitpos:[6] ;default: 1'b0 ; */
/*description: */
#define ASSIST_DEBUG_CORE_0_AREA_PIF_1_RD_RLS  (BIT(6))
#define ASSIST_DEBUG_CORE_0_AREA_PIF_1_RD_RLS_M  (BIT(6))
#define ASSIST_DEBUG_CORE_0_AREA_PIF_1_RD_RLS_V  0x1
#define ASSIST_DEBUG_CORE_0_AREA_PIF_1_RD_RLS_S  6
/* ASSIST_DEBUG_CORE_0_AREA_PIF_0_WR_RLS : R/W ;bitpos:[5] ;default: 1'b0 ; */
/*description: */
#define ASSIST_DEBUG_CORE_0_AREA_PIF_0_WR_RLS  (BIT(5))
#define ASSIST_DEBUG_CORE_0_AREA_PIF_0_WR_RLS_M  (BIT(5))
#define ASSIST_DEBUG_CORE_0_AREA_PIF_0_WR_RLS_V  0x1
#define ASSIST_DEBUG_CORE_0_AREA_PIF_0_WR_RLS_S  5
/* ASSIST_DEBUG_CORE_0_AREA_PIF_0_RD_RLS : R/W ;bitpos:[4] ;default: 1'b0 ; */
/*description: */
#define ASSIST_DEBUG_CORE_0_AREA_PIF_0_RD_RLS  (BIT(4))
#define ASSIST_DEBUG_CORE_0_AREA_PIF_0_RD_RLS_M  (BIT(4))
#define ASSIST_DEBUG_CORE_0_AREA_PIF_0_RD_RLS_V  0x1
#define ASSIST_DEBUG_CORE_0_AREA_PIF_0_RD_RLS_S  4
/* ASSIST_DEBUG_CORE_0_AREA_DRAM0_1_WR_RLS : R/W ;bitpos:[3] ;default: 1'b0 ; */
/*description: */
#define ASSIST_DEBUG_CORE_0_AREA_DRAM0_1_WR_RLS  (BIT(3))
#define ASSIST_DEBUG_CORE_0_AREA_DRAM0_1_WR_RLS_M  (BIT(3))
#define ASSIST_DEBUG_CORE_0_AREA_DRAM0_1_WR_RLS_V  0x1
#define ASSIST_DEBUG_CORE_0_AREA_DRAM0_1_WR_RLS_S  3
/* ASSIST_DEBUG_CORE_0_AREA_DRAM0_1_RD_RLS : R/W ;bitpos:[2] ;default: 1'b0 ; */
/*description: */
#define ASSIST_DEBUG_CORE_0_AREA_DRAM0_1_RD_RLS  (BIT(2))
#define ASSIST_DEBUG_CORE_0_AREA_DRAM0_1_RD_RLS_M  (BIT(2))
#define ASSIST_DEBUG_CORE_0_AREA_DRAM0_1_RD_RLS_V  0x1
#define ASSIST_DEBUG_CORE_0_AREA_DRAM0_1_RD_RLS_S  2
/* ASSIST_DEBUG_CORE_0_AREA_DRAM0_0_WR_RLS : R/W ;bitpos:[1] ;default: 1'b0 ; */
/*description: */
#define ASSIST_DEBUG_CORE_0_AREA_DRAM0_0_WR_RLS  (BIT(1))
#define ASSIST_DEBUG_CORE_0_AREA_DRAM0_0_WR_RLS_M  (BIT(1))
#define ASSIST_DEBUG_CORE_0_AREA_DRAM0_0_WR_RLS_V  0x1
#define ASSIST_DEBUG_CORE_0_AREA_DRAM0_0_WR_RLS_S  1
/* ASSIST_DEBUG_CORE_0_AREA_DRAM0_0_RD_RLS : R/W ;bitpos:[0] ;default: 1'b0 ; */
/*description: */
#define ASSIST_DEBUG_CORE_0_AREA_DRAM0_0_RD_RLS  (BIT(0))
#define ASSIST_DEBUG_CORE_0_AREA_DRAM0_0_RD_RLS_M  (BIT(0))
#define ASSIST_DEBUG_CORE_0_AREA_DRAM0_0_RD_RLS_V  0x1
#define ASSIST_DEBUG_CORE_0_AREA_DRAM0_0_RD_RLS_S  0

#define ASSIST_DEBUG_CORE_0_INTR_CLR_REG          (DR_REG_ASSIST_DEBUG_BASE + 0x00C)
/* ASSIST_DEBUG_CORE_0_DRAM0_EXCEPTION_MONITOR_CLR : R/W ;bitpos:[11] ;default: 1'b0 ; */
/*description: */
#define ASSIST_DEBUG_CORE_0_DRAM0_EXCEPTION_MONITOR_CLR  (BIT(11))
#define ASSIST_DEBUG_CORE_0_DRAM0_EXCEPTION_MONITOR_CLR_M  (BIT(11))
#define ASSIST_DEBUG_CORE_0_DRAM0_EXCEPTION_MONITOR_CLR_V  0x1
#define ASSIST_DEBUG_CORE_0_DRAM0_EXCEPTION_MONITOR_CLR_S  11
/* ASSIST_DEBUG_CORE_0_IRAM0_EXCEPTION_MONITOR_CLR : R/W ;bitpos:[10] ;default: 1'b0 ; */
/*description: */
#define ASSIST_DEBUG_CORE_0_IRAM0_EXCEPTION_MONITOR_CLR  (BIT(10))
#define ASSIST_DEBUG_CORE_0_IRAM0_EXCEPTION_MONITOR_CLR_M  (BIT(10))
#define ASSIST_DEBUG_CORE_0_IRAM0_EXCEPTION_MONITOR_CLR_V  0x1
#define ASSIST_DEBUG_CORE_0_IRAM0_EXCEPTION_MONITOR_CLR_S  10
/* ASSIST_DEBUG_CORE_0_SP_SPILL_MAX_CLR : R/W ;bitpos:[9] ;default: 1'b0 ; */
/*description: */
#define ASSIST_DEBUG_CORE_0_SP_SPILL_MAX_CLR  (BIT(9))
#define ASSIST_DEBUG_CORE_0_SP_SPILL_MAX_CLR_M  (BIT(9))
#define ASSIST_DEBUG_CORE_0_SP_SPILL_MAX_CLR_V  0x1
#define ASSIST_DEBUG_CORE_0_SP_SPILL_MAX_CLR_S  9
/* ASSIST_DEBUG_CORE_0_SP_SPILL_MIN_CLR : R/W ;bitpos:[8] ;default: 1'b0 ; */
/*description: */
#define ASSIST_DEBUG_CORE_0_SP_SPILL_MIN_CLR  (BIT(8))
#define ASSIST_DEBUG_CORE_0_SP_SPILL_MIN_CLR_M  (BIT(8))
#define ASSIST_DEBUG_CORE_0_SP_SPILL_MIN_CLR_V  0x1
#define ASSIST_DEBUG_CORE_0_SP_SPILL_MIN_CLR_S  8
/* ASSIST_DEBUG_CORE_0_AREA_PIF_1_WR_CLR : R/W ;bitpos:[7] ;default: 1'b0 ; */
/*description: */
#define ASSIST_DEBUG_CORE_0_AREA_PIF_1_WR_CLR  (BIT(7))
#define ASSIST_DEBUG_CORE_0_AREA_PIF_1_WR_CLR_M  (BIT(7))
#define ASSIST_DEBUG_CORE_0_AREA_PIF_1_WR_CLR_V  0x1
#define ASSIST_DEBUG_CORE_0_AREA_PIF_1_WR_CLR_S  7
/* ASSIST_DEBUG_CORE_0_AREA_PIF_1_RD_CLR : R/W ;bitpos:[6] ;default: 1'b0 ; */
/*description: */
#define ASSIST_DEBUG_CORE_0_AREA_PIF_1_RD_CLR  (BIT(6))
#define ASSIST_DEBUG_CORE_0_AREA_PIF_1_RD_CLR_M  (BIT(6))
#define ASSIST_DEBUG_CORE_0_AREA_PIF_1_RD_CLR_V  0x1
#define ASSIST_DEBUG_CORE_0_AREA_PIF_1_RD_CLR_S  6
/* ASSIST_DEBUG_CORE_0_AREA_PIF_0_WR_CLR : R/W ;bitpos:[5] ;default: 1'b0 ; */
/*description: */
#define ASSIST_DEBUG_CORE_0_AREA_PIF_0_WR_CLR  (BIT(5))
#define ASSIST_DEBUG_CORE_0_AREA_PIF_0_WR_CLR_M  (BIT(5))
#define ASSIST_DEBUG_CORE_0_AREA_PIF_0_WR_CLR_V  0x1
#define ASSIST_DEBUG_CORE_0_AREA_PIF_0_WR_CLR_S  5
/* ASSIST_DEBUG_CORE_0_AREA_PIF_0_RD_CLR : R/W ;bitpos:[4] ;default: 1'b0 ; */
/*description: */
#define ASSIST_DEBUG_CORE_0_AREA_PIF_0_RD_CLR  (BIT(4))
#define ASSIST_DEBUG_CORE_0_AREA_PIF_0_RD_CLR_M  (BIT(4))
#define ASSIST_DEBUG_CORE_0_AREA_PIF_0_RD_CLR_V  0x1
#define ASSIST_DEBUG_CORE_0_AREA_PIF_0_RD_CLR_S  4
/* ASSIST_DEBUG_CORE_0_AREA_DRAM0_1_WR_CLR : R/W ;bitpos:[3] ;default: 1'b0 ; */
/*description: */
#define ASSIST_DEBUG_CORE_0_AREA_DRAM0_1_WR_CLR  (BIT(3))
#define ASSIST_DEBUG_CORE_0_AREA_DRAM0_1_WR_CLR_M  (BIT(3))
#define ASSIST_DEBUG_CORE_0_AREA_DRAM0_1_WR_CLR_V  0x1
#define ASSIST_DEBUG_CORE_0_AREA_DRAM0_1_WR_CLR_S  3
/* ASSIST_DEBUG_CORE_0_AREA_DRAM0_1_RD_CLR : R/W ;bitpos:[2] ;default: 1'b0 ; */
/*description: */
#define ASSIST_DEBUG_CORE_0_AREA_DRAM0_1_RD_CLR  (BIT(2))
#define ASSIST_DEBUG_CORE_0_AREA_DRAM0_1_RD_CLR_M  (BIT(2))
#define ASSIST_DEBUG_CORE_0_AREA_DRAM0_1_RD_CLR_V  0x1
#define ASSIST_DEBUG_CORE_0_AREA_DRAM0_1_RD_CLR_S  2
/* ASSIST_DEBUG_CORE_0_AREA_DRAM0_0_WR_CLR : R/W ;bitpos:[1] ;default: 1'b0 ; */
/*description: */
#define ASSIST_DEBUG_CORE_0_AREA_DRAM0_0_WR_CLR  (BIT(1))
#define ASSIST_DEBUG_CORE_0_AREA_DRAM0_0_WR_CLR_M  (BIT(1))
#define ASSIST_DEBUG_CORE_0_AREA_DRAM0_0_WR_CLR_V  0x1
#define ASSIST_DEBUG_CORE_0_AREA_DRAM0_0_WR_CLR_S  1
/* ASSIST_DEBUG_CORE_0_AREA_DRAM0_0_RD_CLR : R/W ;bitpos:[0] ;default: 1'b0 ; */
/*description: */
#define ASSIST_DEBUG_CORE_0_AREA_DRAM0_0_RD_CLR  (BIT(0))
#define ASSIST_DEBUG_CORE_0_AREA_DRAM0_0_RD_CLR_M  (BIT(0))
#define ASSIST_DEBUG_CORE_0_AREA_DRAM0_0_RD_CLR_V  0x1
#define ASSIST_DEBUG_CORE_0_AREA_DRAM0_0_RD_CLR_S  0

#define ASSIST_DEBUG_CORE_0_AREA_DRAM0_0_MIN_REG          (DR_REG_ASSIST_DEBUG_BASE + 0x010)
/* ASSIST_DEBUG_CORE_0_AREA_DRAM0_0_MIN : R/W ;bitpos:[31:0] ;default: ~32'b0 ; */
/*description: */
#define ASSIST_DEBUG_CORE_0_AREA_DRAM0_0_MIN  0xFFFFFFFF
#define ASSIST_DEBUG_CORE_0_AREA_DRAM0_0_MIN_M  ((ASSIST_DEBUG_CORE_0_AREA_DRAM0_0_MIN_V)<<(ASSIST_DEBUG_CORE_0_AREA_DRAM0_0_MIN_S))
#define ASSIST_DEBUG_CORE_0_AREA_DRAM0_0_MIN_V  0xFFFFFFFF
#define ASSIST_DEBUG_CORE_0_AREA_DRAM0_0_MIN_S  0

#define ASSIST_DEBUG_CORE_0_AREA_DRAM0_0_MAX_REG          (DR_REG_ASSIST_DEBUG_BASE + 0x014)
/* ASSIST_DEBUG_CORE_0_AREA_DRAM0_0_MAX : R/W ;bitpos:[31:0] ;default: 32'b0 ; */
/*description: */
#define ASSIST_DEBUG_CORE_0_AREA_DRAM0_0_MAX  0xFFFFFFFF
#define ASSIST_DEBUG_CORE_0_AREA_DRAM0_0_MAX_M  ((ASSIST_DEBUG_CORE_0_AREA_DRAM0_0_MAX_V)<<(ASSIST_DEBUG_CORE_0_AREA_DRAM0_0_MAX_S))
#define ASSIST_DEBUG_CORE_0_AREA_DRAM0_0_MAX_V  0xFFFFFFFF
#define ASSIST_DEBUG_CORE_0_AREA_DRAM0_0_MAX_S  0

#define ASSIST_DEBUG_CORE_0_AREA_DRAM0_1_MIN_REG          (DR_REG_ASSIST_DEBUG_BASE + 0x018)
/* ASSIST_DEBUG_CORE_0_AREA_DRAM0_1_MIN : R/W ;bitpos:[31:0] ;default: ~32'b0 ; */
/*description: */
#define ASSIST_DEBUG_CORE_0_AREA_DRAM0_1_MIN  0xFFFFFFFF
#define ASSIST_DEBUG_CORE_0_AREA_DRAM0_1_MIN_M  ((ASSIST_DEBUG_CORE_0_AREA_DRAM0_1_MIN_V)<<(ASSIST_DEBUG_CORE_0_AREA_DRAM0_1_MIN_S))
#define ASSIST_DEBUG_CORE_0_AREA_DRAM0_1_MIN_V  0xFFFFFFFF
#define ASSIST_DEBUG_CORE_0_AREA_DRAM0_1_MIN_S  0

#define ASSIST_DEBUG_CORE_0_AREA_DRAM0_1_MAX_REG          (DR_REG_ASSIST_DEBUG_BASE + 0x01C)
/* ASSIST_DEBUG_CORE_0_AREA_DRAM0_1_MAX : R/W ;bitpos:[31:0] ;default: 32'b0 ; */
/*description: */
#define ASSIST_DEBUG_CORE_0_AREA_DRAM0_1_MAX  0xFFFFFFFF
#define ASSIST_DEBUG_CORE_0_AREA_DRAM0_1_MAX_M  ((ASSIST_DEBUG_CORE_0_AREA_DRAM0_1_MAX_V)<<(ASSIST_DEBUG_CORE_0_AREA_DRAM0_1_MAX_S))
#define ASSIST_DEBUG_CORE_0_AREA_DRAM0_1_MAX_V  0xFFFFFFFF
#define ASSIST_DEBUG_CORE_0_AREA_DRAM0_1_MAX_S  0

#define ASSIST_DEBUG_CORE_0_AREA_PIF_0_MIN_REG          (DR_REG_ASSIST_DEBUG_BASE + 0x020)
/* ASSIST_DEBUG_CORE_0_AREA_PIF_0_MIN : R/W ;bitpos:[31:0] ;default: ~32'b0 ; */
/*description: */
#define ASSIST_DEBUG_CORE_0_AREA_PIF_0_MIN  0xFFFFFFFF
#define ASSIST_DEBUG_CORE_0_AREA_PIF_0_MIN_M  ((ASSIST_DEBUG_CORE_0_AREA_PIF_0_MIN_V)<<(ASSIST_DEBUG_CORE_0_AREA_PIF_0_MIN_S))
#define ASSIST_DEBUG_CORE_0_AREA_PIF_0_MIN_V  0xFFFFFFFF
#define ASSIST_DEBUG_CORE_0_AREA_PIF_0_MIN_S  0

#define ASSIST_DEBUG_CORE_0_AREA_PIF_0_MAX_REG          (DR_REG_ASSIST_DEBUG_BASE + 0x024)
/* ASSIST_DEBUG_CORE_0_AREA_PIF_0_MAX : R/W ;bitpos:[31:0] ;default: 32'b0 ; */
/*description: */
#define ASSIST_DEBUG_CORE_0_AREA_PIF_0_MAX  0xFFFFFFFF
#define ASSIST_DEBUG_CORE_0_AREA_PIF_0_MAX_M  ((ASSIST_DEBUG_CORE_0_AREA_PIF_0_MAX_V)<<(ASSIST_DEBUG_CORE_0_AREA_PIF_0_MAX_S))
#define ASSIST_DEBUG_CORE_0_AREA_PIF_0_MAX_V  0xFFFFFFFF
#define ASSIST_DEBUG_CORE_0_AREA_PIF_0_MAX_S  0

#define ASSIST_DEBUG_CORE_0_AREA_PIF_1_MIN_REG          (DR_REG_ASSIST_DEBUG_BASE + 0x028)
/* ASSIST_DEBUG_CORE_0_AREA_PIF_1_MIN : R/W ;bitpos:[31:0] ;default: ~32'b0 ; */
/*description: */
#define ASSIST_DEBUG_CORE_0_AREA_PIF_1_MIN  0xFFFFFFFF
#define ASSIST_DEBUG_CORE_0_AREA_PIF_1_MIN_M  ((ASSIST_DEBUG_CORE_0_AREA_PIF_1_MIN_V)<<(ASSIST_DEBUG_CORE_0_AREA_PIF_1_MIN_S))
#define ASSIST_DEBUG_CORE_0_AREA_PIF_1_MIN_V  0xFFFFFFFF
#define ASSIST_DEBUG_CORE_0_AREA_PIF_1_MIN_S  0

#define ASSIST_DEBUG_CORE_0_AREA_PIF_1_MAX_REG          (DR_REG_ASSIST_DEBUG_BASE + 0x02C)
/* ASSIST_DEBUG_CORE_0_AREA_PIF_1_MAX : R/W ;bitpos:[31:0] ;default: 32'b0 ; */
/*description: */
#define ASSIST_DEBUG_CORE_0_AREA_PIF_1_MAX  0xFFFFFFFF
#define ASSIST_DEBUG_CORE_0_AREA_PIF_1_MAX_M  ((ASSIST_DEBUG_CORE_0_AREA_PIF_1_MAX_V)<<(ASSIST_DEBUG_CORE_0_AREA_PIF_1_MAX_S))
#define ASSIST_DEBUG_CORE_0_AREA_PIF_1_MAX_V  0xFFFFFFFF
#define ASSIST_DEBUG_CORE_0_AREA_PIF_1_MAX_S  0

#define ASSIST_DEBUG_CORE_0_AREA_PC_REG          (DR_REG_ASSIST_DEBUG_BASE + 0x030)
/* ASSIST_DEBUG_CORE_0_AREA_PC : RO ;bitpos:[31:0] ;default: 32'b0 ; */
/*description: */
#define ASSIST_DEBUG_CORE_0_AREA_PC  0xFFFFFFFF
#define ASSIST_DEBUG_CORE_0_AREA_PC_M  ((ASSIST_DEBUG_CORE_0_AREA_PC_V)<<(ASSIST_DEBUG_CORE_0_AREA_PC_S))
#define ASSIST_DEBUG_CORE_0_AREA_PC_V  0xFFFFFFFF
#define ASSIST_DEBUG_CORE_0_AREA_PC_S  0

#define ASSIST_DEBUG_CORE_0_AREA_SP_REG          (DR_REG_ASSIST_DEBUG_BASE + 0x034)
/* ASSIST_DEBUG_CORE_0_AREA_SP : RO ;bitpos:[31:0] ;default: 32'b0 ; */
/*description: */
#define ASSIST_DEBUG_CORE_0_AREA_SP  0xFFFFFFFF
#define ASSIST_DEBUG_CORE_0_AREA_SP_M  ((ASSIST_DEBUG_CORE_0_AREA_SP_V)<<(ASSIST_DEBUG_CORE_0_AREA_SP_S))
#define ASSIST_DEBUG_CORE_0_AREA_SP_V  0xFFFFFFFF
#define ASSIST_DEBUG_CORE_0_AREA_SP_S  0

#define ASSIST_DEBUG_CORE_0_SP_MIN_REG          (DR_REG_ASSIST_DEBUG_BASE + 0x038)
/* ASSIST_DEBUG_CORE_0_SP_MIN : RW ;bitpos:[31:0] ;default: 32'b0 ; */
/*description: */
#define ASSIST_DEBUG_CORE_0_SP_MIN  0xFFFFFFFF
#define ASSIST_DEBUG_CORE_0_SP_MIN_M  ((ASSIST_DEBUG_CORE_0_SP_MIN_V)<<(ASSIST_DEBUG_CORE_0_SP_MIN_S))
#define ASSIST_DEBUG_CORE_0_SP_MIN_V  0xFFFFFFFF
#define ASSIST_DEBUG_CORE_0_SP_MIN_S  0

#define ASSIST_DEBUG_CORE_0_SP_MAX_REG          (DR_REG_ASSIST_DEBUG_BASE + 0x03C)
/* ASSIST_DEBUG_CORE_0_SP_MAX : RW ;bitpos:[31:0] ;default: ~32'b0 ; */
/*description: */
#define ASSIST_DEBUG_CORE_0_SP_MAX  0xFFFFFFFF
#define ASSIST_DEBUG_CORE_0_SP_MAX_M  ((ASSIST_DEBUG_CORE_0_SP_MAX_V)<<(ASSIST_DEBUG_CORE_0_SP_MAX_S))
#define ASSIST_DEBUG_CORE_0_SP_MAX_V  0xFFFFFFFF
#define ASSIST_DEBUG_CORE_0_SP_MAX_S  0

#define ASSIST_DEBUG_CORE_0_SP_PC_REG          (DR_REG_ASSIST_DEBUG_BASE + 0x040)
/* ASSIST_DEBUG_CORE_0_SP_PC : RO ;bitpos:[31:0] ;default: 32'b0 ; */
/*description: */
#define ASSIST_DEBUG_CORE_0_SP_PC  0xFFFFFFFF
#define ASSIST_DEBUG_CORE_0_SP_PC_M  ((ASSIST_DEBUG_CORE_0_SP_PC_V)<<(ASSIST_DEBUG_CORE_0_SP_PC_S))
#define ASSIST_DEBUG_CORE_0_SP_PC_V  0xFFFFFFFF
#define ASSIST_DEBUG_CORE_0_SP_PC_S  0

#define ASSIST_DEBUG_CORE_0_RCD_EN_REG          (DR_REG_ASSIST_DEBUG_BASE + 0x044)
/* ASSIST_DEBUG_CORE_0_RCD_PDEBUGEN : RW ;bitpos:[1] ;default: 1'b0 ; */
/*description: enable CPU Pdebug function  if enable  CPU will update PdebugPC*/
#define ASSIST_DEBUG_CORE_0_RCD_PDEBUGEN  (BIT(1))
#define ASSIST_DEBUG_CORE_0_RCD_PDEBUGEN_M  (BIT(1))
#define ASSIST_DEBUG_CORE_0_RCD_PDEBUGEN_V  0x1
#define ASSIST_DEBUG_CORE_0_RCD_PDEBUGEN_S  1
/* ASSIST_DEBUG_CORE_0_RCD_RECORDEN : RW ;bitpos:[0] ;default: 1'b0 ; */
/*description: enable recording function  if enable  assist_debug will update
 PdebugPC  so you can read it*/
#define ASSIST_DEBUG_CORE_0_RCD_RECORDEN  (BIT(0))
#define ASSIST_DEBUG_CORE_0_RCD_RECORDEN_M  (BIT(0))
#define ASSIST_DEBUG_CORE_0_RCD_RECORDEN_V  0x1
#define ASSIST_DEBUG_CORE_0_RCD_RECORDEN_S  0

#define ASSIST_DEBUG_CORE_0_RCD_PDEBUGPC_REG          (DR_REG_ASSIST_DEBUG_BASE + 0x048)
/* ASSIST_DEBUG_CORE_0_RCD_PDEBUGPC : RO ;bitpos:[31:0] ;default: 32'h0 ; */
/*description: */
#define ASSIST_DEBUG_CORE_0_RCD_PDEBUGPC  0xFFFFFFFF
#define ASSIST_DEBUG_CORE_0_RCD_PDEBUGPC_M  ((ASSIST_DEBUG_CORE_0_RCD_PDEBUGPC_V)<<(ASSIST_DEBUG_CORE_0_RCD_PDEBUGPC_S))
#define ASSIST_DEBUG_CORE_0_RCD_PDEBUGPC_V  0xFFFFFFFF
#define ASSIST_DEBUG_CORE_0_RCD_PDEBUGPC_S  0

#define ASSIST_DEBUG_CORE_0_RCD_PDEBUGSP_REG          (DR_REG_ASSIST_DEBUG_BASE + 0x04C)
/* ASSIST_DEBUG_CORE_0_RCD_PDEBUGSP : RO ;bitpos:[31:0] ;default: 32'h0 ; */
/*description: */
#define ASSIST_DEBUG_CORE_0_RCD_PDEBUGSP  0xFFFFFFFF
#define ASSIST_DEBUG_CORE_0_RCD_PDEBUGSP_M  ((ASSIST_DEBUG_CORE_0_RCD_PDEBUGSP_V)<<(ASSIST_DEBUG_CORE_0_RCD_PDEBUGSP_S))
#define ASSIST_DEBUG_CORE_0_RCD_PDEBUGSP_V  0xFFFFFFFF
#define ASSIST_DEBUG_CORE_0_RCD_PDEBUGSP_S  0

#define ASSIST_DEBUG_CORE_0_IRAM0_EXCEPTION_MONITOR_0_REG          (DR_REG_ASSIST_DEBUG_BASE + 0x050)
/* ASSIST_DEBUG_CORE_0_IRAM0_RECORDING_LOADSTORE_0 : RO ;bitpos:[25] ;default: 1'b0 ; */
/*description: */
#define ASSIST_DEBUG_CORE_0_IRAM0_RECORDING_LOADSTORE_0  (BIT(25))
#define ASSIST_DEBUG_CORE_0_IRAM0_RECORDING_LOADSTORE_0_M  (BIT(25))
#define ASSIST_DEBUG_CORE_0_IRAM0_RECORDING_LOADSTORE_0_V  0x1
#define ASSIST_DEBUG_CORE_0_IRAM0_RECORDING_LOADSTORE_0_S  25
/* ASSIST_DEBUG_CORE_0_IRAM0_RECORDING_WR_0 : RO ;bitpos:[24] ;default: 1'b0 ; */
/*description: */
#define ASSIST_DEBUG_CORE_0_IRAM0_RECORDING_WR_0  (BIT(24))
#define ASSIST_DEBUG_CORE_0_IRAM0_RECORDING_WR_0_M  (BIT(24))
#define ASSIST_DEBUG_CORE_0_IRAM0_RECORDING_WR_0_V  0x1
#define ASSIST_DEBUG_CORE_0_IRAM0_RECORDING_WR_0_S  24
/* ASSIST_DEBUG_CORE_0_IRAM0_RECORDING_ADDR_0 : RO ;bitpos:[23:0] ;default: 24'b0 ; */
/*description: */
#define ASSIST_DEBUG_CORE_0_IRAM0_RECORDING_ADDR_0  0x00FFFFFF
#define ASSIST_DEBUG_CORE_0_IRAM0_RECORDING_ADDR_0_M  ((ASSIST_DEBUG_CORE_0_IRAM0_RECORDING_ADDR_0_V)<<(ASSIST_DEBUG_CORE_0_IRAM0_RECORDING_ADDR_0_S))
#define ASSIST_DEBUG_CORE_0_IRAM0_RECORDING_ADDR_0_V  0xFFFFFF
#define ASSIST_DEBUG_CORE_0_IRAM0_RECORDING_ADDR_0_S  0

#define ASSIST_DEBUG_CORE_0_IRAM0_EXCEPTION_MONITOR_1_REG          (DR_REG_ASSIST_DEBUG_BASE + 0x054)
/* ASSIST_DEBUG_CORE_0_IRAM0_RECORDING_LOADSTORE_1 : RO ;bitpos:[25] ;default: 1'b0 ; */
/*description: */
#define ASSIST_DEBUG_CORE_0_IRAM0_RECORDING_LOADSTORE_1  (BIT(25))
#define ASSIST_DEBUG_CORE_0_IRAM0_RECORDING_LOADSTORE_1_M  (BIT(25))
#define ASSIST_DEBUG_CORE_0_IRAM0_RECORDING_LOADSTORE_1_V  0x1
#define ASSIST_DEBUG_CORE_0_IRAM0_RECORDING_LOADSTORE_1_S  25
/* ASSIST_DEBUG_CORE_0_IRAM0_RECORDING_WR_1 : RO ;bitpos:[24] ;default: 1'b0 ; */
/*description: */
#define ASSIST_DEBUG_CORE_0_IRAM0_RECORDING_WR_1  (BIT(24))
#define ASSIST_DEBUG_CORE_0_IRAM0_RECORDING_WR_1_M  (BIT(24))
#define ASSIST_DEBUG_CORE_0_IRAM0_RECORDING_WR_1_V  0x1
#define ASSIST_DEBUG_CORE_0_IRAM0_RECORDING_WR_1_S  24
/* ASSIST_DEBUG_CORE_0_IRAM0_RECORDING_ADDR_1 : RO ;bitpos:[23:0] ;default: 24'b0 ; */
/*description: */
#define ASSIST_DEBUG_CORE_0_IRAM0_RECORDING_ADDR_1  0x00FFFFFF
#define ASSIST_DEBUG_CORE_0_IRAM0_RECORDING_ADDR_1_M  ((ASSIST_DEBUG_CORE_0_IRAM0_RECORDING_ADDR_1_V)<<(ASSIST_DEBUG_CORE_0_IRAM0_RECORDING_ADDR_1_S))
#define ASSIST_DEBUG_CORE_0_IRAM0_RECORDING_ADDR_1_V  0xFFFFFF
#define ASSIST_DEBUG_CORE_0_IRAM0_RECORDING_ADDR_1_S  0

#define ASSIST_DEBUG_CORE_0_DRAM0_EXCEPTION_MONITOR_0_REG          (DR_REG_ASSIST_DEBUG_BASE + 0x058)
/* ASSIST_DEBUG_CORE_0_DRAM0_RECORDING_BYTEEN_0 : RO ;bitpos:[28:25] ;default: 4'b0 ; */
/*description: */
#define ASSIST_DEBUG_CORE_0_DRAM0_RECORDING_BYTEEN_0  0x0000000F
#define ASSIST_DEBUG_CORE_0_DRAM0_RECORDING_BYTEEN_0_M  ((ASSIST_DEBUG_CORE_0_DRAM0_RECORDING_BYTEEN_0_V)<<(ASSIST_DEBUG_CORE_0_DRAM0_RECORDING_BYTEEN_0_S))
#define ASSIST_DEBUG_CORE_0_DRAM0_RECORDING_BYTEEN_0_V  0xF
#define ASSIST_DEBUG_CORE_0_DRAM0_RECORDING_BYTEEN_0_S  25
/* ASSIST_DEBUG_CORE_0_DRAM0_RECORDING_WR_0 : RO ;bitpos:[24] ;default: 1'b0 ; */
/*description: */
#define ASSIST_DEBUG_CORE_0_DRAM0_RECORDING_WR_0  (BIT(24))
#define ASSIST_DEBUG_CORE_0_DRAM0_RECORDING_WR_0_M  (BIT(24))
#define ASSIST_DEBUG_CORE_0_DRAM0_RECORDING_WR_0_V  0x1
#define ASSIST_DEBUG_CORE_0_DRAM0_RECORDING_WR_0_S  24
/* ASSIST_DEBUG_CORE_0_DRAM0_RECORDING_ADDR_0 : RO ;bitpos:[23:0] ;default: 24'b0 ; */
/*description: */
#define ASSIST_DEBUG_CORE_0_DRAM0_RECORDING_ADDR_0  0x00FFFFFF
#define ASSIST_DEBUG_CORE_0_DRAM0_RECORDING_ADDR_0_M  ((ASSIST_DEBUG_CORE_0_DRAM0_RECORDING_ADDR_0_V)<<(ASSIST_DEBUG_CORE_0_DRAM0_RECORDING_ADDR_0_S))
#define ASSIST_DEBUG_CORE_0_DRAM0_RECORDING_ADDR_0_V  0xFFFFFF
#define ASSIST_DEBUG_CORE_0_DRAM0_RECORDING_ADDR_0_S  0

#define ASSIST_DEBUG_CORE_0_DRAM0_EXCEPTION_MONITOR_1_REG          (DR_REG_ASSIST_DEBUG_BASE + 0x05C)
/* ASSIST_DEBUG_CORE_0_DRAM0_RECORDING_PC_0 : RO ;bitpos:[31:0] ;default: 32'b0 ; */
/*description: */
#define ASSIST_DEBUG_CORE_0_DRAM0_RECORDING_PC_0  0xFFFFFFFF
#define ASSIST_DEBUG_CORE_0_DRAM0_RECORDING_PC_0_M  ((ASSIST_DEBUG_CORE_0_DRAM0_RECORDING_PC_0_V)<<(ASSIST_DEBUG_CORE_0_DRAM0_RECORDING_PC_0_S))
#define ASSIST_DEBUG_CORE_0_DRAM0_RECORDING_PC_0_V  0xFFFFFFFF
#define ASSIST_DEBUG_CORE_0_DRAM0_RECORDING_PC_0_S  0

#define ASSIST_DEBUG_CORE_0_DRAM0_EXCEPTION_MONITOR_2_REG          (DR_REG_ASSIST_DEBUG_BASE + 0x060)
/* ASSIST_DEBUG_CORE_0_DRAM0_RECORDING_BYTEEN_1 : RO ;bitpos:[28:25] ;default: 4'b0 ; */
/*description: */
#define ASSIST_DEBUG_CORE_0_DRAM0_RECORDING_BYTEEN_1  0x0000000F
#define ASSIST_DEBUG_CORE_0_DRAM0_RECORDING_BYTEEN_1_M  ((ASSIST_DEBUG_CORE_0_DRAM0_RECORDING_BYTEEN_1_V)<<(ASSIST_DEBUG_CORE_0_DRAM0_RECORDING_BYTEEN_1_S))
#define ASSIST_DEBUG_CORE_0_DRAM0_RECORDING_BYTEEN_1_V  0xF
#define ASSIST_DEBUG_CORE_0_DRAM0_RECORDING_BYTEEN_1_S  25
/* ASSIST_DEBUG_CORE_0_DRAM0_RECORDING_WR_1 : RO ;bitpos:[24] ;default: 1'b0 ; */
/*description: */
#define ASSIST_DEBUG_CORE_0_DRAM0_RECORDING_WR_1  (BIT(24))
#define ASSIST_DEBUG_CORE_0_DRAM0_RECORDING_WR_1_M  (BIT(24))
#define ASSIST_DEBUG_CORE_0_DRAM0_RECORDING_WR_1_V  0x1
#define ASSIST_DEBUG_CORE_0_DRAM0_RECORDING_WR_1_S  24
/* ASSIST_DEBUG_CORE_0_DRAM0_RECORDING_ADDR_1 : RO ;bitpos:[23:0] ;default: 24'b0 ; */
/*description: */
#define ASSIST_DEBUG_CORE_0_DRAM0_RECORDING_ADDR_1  0x00FFFFFF
#define ASSIST_DEBUG_CORE_0_DRAM0_RECORDING_ADDR_1_M  ((ASSIST_DEBUG_CORE_0_DRAM0_RECORDING_ADDR_1_V)<<(ASSIST_DEBUG_CORE_0_DRAM0_RECORDING_ADDR_1_S))
#define ASSIST_DEBUG_CORE_0_DRAM0_RECORDING_ADDR_1_V  0xFFFFFF
#define ASSIST_DEBUG_CORE_0_DRAM0_RECORDING_ADDR_1_S  0

#define ASSIST_DEBUG_CORE_0_DRAM0_EXCEPTION_MONITOR_3_REG          (DR_REG_ASSIST_DEBUG_BASE + 0x064)
/* ASSIST_DEBUG_CORE_0_DRAM0_RECORDING_PC_1 : RO ;bitpos:[31:0] ;default: 32'b0 ; */
/*description: */
#define ASSIST_DEBUG_CORE_0_DRAM0_RECORDING_PC_1  0xFFFFFFFF
#define ASSIST_DEBUG_CORE_0_DRAM0_RECORDING_PC_1_M  ((ASSIST_DEBUG_CORE_0_DRAM0_RECORDING_PC_1_V)<<(ASSIST_DEBUG_CORE_0_DRAM0_RECORDING_PC_1_S))
#define ASSIST_DEBUG_CORE_0_DRAM0_RECORDING_PC_1_V  0xFFFFFFFF
#define ASSIST_DEBUG_CORE_0_DRAM0_RECORDING_PC_1_S  0

#define ASSIST_DEBUG_CORE_X_IRAM0_DRAM0_EXCEPTION_MONITOR_0_REG          (DR_REG_ASSIST_DEBUG_BASE + 0x068)
/* ASSIST_DEBUG_CORE_X_IRAM0_DRAM0_LIMIT_CYCLE_0 : R/W ;bitpos:[19:0] ;default: 20'b0 ; */
/*description: */
#define ASSIST_DEBUG_CORE_X_IRAM0_DRAM0_LIMIT_CYCLE_0  0x000FFFFF
#define ASSIST_DEBUG_CORE_X_IRAM0_DRAM0_LIMIT_CYCLE_0_M  ((ASSIST_DEBUG_CORE_X_IRAM0_DRAM0_LIMIT_CYCLE_0_V)<<(ASSIST_DEBUG_CORE_X_IRAM0_DRAM0_LIMIT_CYCLE_0_S))
#define ASSIST_DEBUG_CORE_X_IRAM0_DRAM0_LIMIT_CYCLE_0_V  0xFFFFF
#define ASSIST_DEBUG_CORE_X_IRAM0_DRAM0_LIMIT_CYCLE_0_S  0

#define ASSIST_DEBUG_CORE_X_IRAM0_DRAM0_EXCEPTION_MONITOR_1_REG          (DR_REG_ASSIST_DEBUG_BASE + 0x06C)
/* ASSIST_DEBUG_CORE_X_IRAM0_DRAM0_LIMIT_CYCLE_1 : R/W ;bitpos:[19:0] ;default: 20'b0 ; */
/*description: */
#define ASSIST_DEBUG_CORE_X_IRAM0_DRAM0_LIMIT_CYCLE_1  0x000FFFFF
#define ASSIST_DEBUG_CORE_X_IRAM0_DRAM0_LIMIT_CYCLE_1_M  ((ASSIST_DEBUG_CORE_X_IRAM0_DRAM0_LIMIT_CYCLE_1_V)<<(ASSIST_DEBUG_CORE_X_IRAM0_DRAM0_LIMIT_CYCLE_1_S))
#define ASSIST_DEBUG_CORE_X_IRAM0_DRAM0_LIMIT_CYCLE_1_V  0xFFFFF
#define ASSIST_DEBUG_CORE_X_IRAM0_DRAM0_LIMIT_CYCLE_1_S  0

#define ASSIST_DEBUG_LOG_SETTING_REG          (DR_REG_ASSIST_DEBUG_BASE + 0x070)
/* ASSIST_DEBUG_LOG_MEM_LOOP_ENABLE : R/W ;bitpos:[7] ;default: 1'b1 ; */
/*description: */
#define ASSIST_DEBUG_LOG_MEM_LOOP_ENABLE  (BIT(7))
#define ASSIST_DEBUG_LOG_MEM_LOOP_ENABLE_M  (BIT(7))
#define ASSIST_DEBUG_LOG_MEM_LOOP_ENABLE_V  0x1
#define ASSIST_DEBUG_LOG_MEM_LOOP_ENABLE_S  7
/* ASSIST_DEBUG_LOG_MODE : R/W ;bitpos:[6:3] ;default: 4'b0 ; */
/*description: */
#define ASSIST_DEBUG_LOG_MODE  0x0000000F
#define ASSIST_DEBUG_LOG_MODE_M  ((ASSIST_DEBUG_LOG_MODE_V)<<(ASSIST_DEBUG_LOG_MODE_S))
#define ASSIST_DEBUG_LOG_MODE_V  0xF
#define ASSIST_DEBUG_LOG_MODE_S  3
/* ASSIST_DEBUG_LOG_ENA : R/W ;bitpos:[2:0] ;default: 3'b0 ; */
/*description: */
#define ASSIST_DEBUG_LOG_ENA  0x00000007
#define ASSIST_DEBUG_LOG_ENA_M  ((ASSIST_DEBUG_LOG_ENA_V)<<(ASSIST_DEBUG_LOG_ENA_S))
#define ASSIST_DEBUG_LOG_ENA_V  0x7
#define ASSIST_DEBUG_LOG_ENA_S  0

#define ASSIST_DEBUG_LOG_DATA_0_REG          (DR_REG_ASSIST_DEBUG_BASE + 0x074)
/* ASSIST_DEBUG_LOG_DATA_0 : R/W ;bitpos:[31:0] ;default: 32'b0 ; */
/*description: */
#define ASSIST_DEBUG_LOG_DATA_0  0xFFFFFFFF
#define ASSIST_DEBUG_LOG_DATA_0_M  ((ASSIST_DEBUG_LOG_DATA_0_V)<<(ASSIST_DEBUG_LOG_DATA_0_S))
#define ASSIST_DEBUG_LOG_DATA_0_V  0xFFFFFFFF
#define ASSIST_DEBUG_LOG_DATA_0_S  0

#define ASSIST_DEBUG_LOG_DATA_MASK_REG          (DR_REG_ASSIST_DEBUG_BASE + 0x078)
/* ASSIST_DEBUG_LOG_DATA_SIZE : R/W ;bitpos:[15:0] ;default: 16'b0 ; */
/*description: */
#define ASSIST_DEBUG_LOG_DATA_SIZE  0x0000FFFF
#define ASSIST_DEBUG_LOG_DATA_SIZE_M  ((ASSIST_DEBUG_LOG_DATA_SIZE_V)<<(ASSIST_DEBUG_LOG_DATA_SIZE_S))
#define ASSIST_DEBUG_LOG_DATA_SIZE_V  0xFFFF
#define ASSIST_DEBUG_LOG_DATA_SIZE_S  0

#define ASSIST_DEBUG_LOG_MIN_REG          (DR_REG_ASSIST_DEBUG_BASE + 0x07C)
/* ASSIST_DEBUG_LOG_MIN : R/W ;bitpos:[31:0] ;default: 32'b0 ; */
/*description: */
#define ASSIST_DEBUG_LOG_MIN  0xFFFFFFFF
#define ASSIST_DEBUG_LOG_MIN_M  ((ASSIST_DEBUG_LOG_MIN_V)<<(ASSIST_DEBUG_LOG_MIN_S))
#define ASSIST_DEBUG_LOG_MIN_V  0xFFFFFFFF
#define ASSIST_DEBUG_LOG_MIN_S  0

#define ASSIST_DEBUG_LOG_MAX_REG          (DR_REG_ASSIST_DEBUG_BASE + 0x080)
/* ASSIST_DEBUG_LOG_MAX : R/W ;bitpos:[31:0] ;default: 32'b0 ; */
/*description: */
#define ASSIST_DEBUG_LOG_MAX  0xFFFFFFFF
#define ASSIST_DEBUG_LOG_MAX_M  ((ASSIST_DEBUG_LOG_MAX_V)<<(ASSIST_DEBUG_LOG_MAX_S))
#define ASSIST_DEBUG_LOG_MAX_V  0xFFFFFFFF
#define ASSIST_DEBUG_LOG_MAX_S  0

#define ASSIST_DEBUG_LOG_MEM_START_REG          (DR_REG_ASSIST_DEBUG_BASE + 0x084)
/* ASSIST_DEBUG_LOG_MEM_START : R/W ;bitpos:[31:0] ;default: 32'b0 ; */
/*description: */
#define ASSIST_DEBUG_LOG_MEM_START  0xFFFFFFFF
#define ASSIST_DEBUG_LOG_MEM_START_M  ((ASSIST_DEBUG_LOG_MEM_START_V)<<(ASSIST_DEBUG_LOG_MEM_START_S))
#define ASSIST_DEBUG_LOG_MEM_START_V  0xFFFFFFFF
#define ASSIST_DEBUG_LOG_MEM_START_S  0

#define ASSIST_DEBUG_LOG_MEM_END_REG          (DR_REG_ASSIST_DEBUG_BASE + 0x088)
/* ASSIST_DEBUG_LOG_MEM_END : R/W ;bitpos:[31:0] ;default: 32'b0 ; */
/*description: */
#define ASSIST_DEBUG_LOG_MEM_END  0xFFFFFFFF
#define ASSIST_DEBUG_LOG_MEM_END_M  ((ASSIST_DEBUG_LOG_MEM_END_V)<<(ASSIST_DEBUG_LOG_MEM_END_S))
#define ASSIST_DEBUG_LOG_MEM_END_V  0xFFFFFFFF
#define ASSIST_DEBUG_LOG_MEM_END_S  0

#define ASSIST_DEBUG_LOG_MEM_WRITING_ADDR_REG          (DR_REG_ASSIST_DEBUG_BASE + 0x08C)
/* ASSIST_DEBUG_LOG_MEM_WRITING_ADDR : RO ;bitpos:[31:0] ;default: 32'b0 ; */
/*description: */
#define ASSIST_DEBUG_LOG_MEM_WRITING_ADDR  0xFFFFFFFF
#define ASSIST_DEBUG_LOG_MEM_WRITING_ADDR_M  ((ASSIST_DEBUG_LOG_MEM_WRITING_ADDR_V)<<(ASSIST_DEBUG_LOG_MEM_WRITING_ADDR_S))
#define ASSIST_DEBUG_LOG_MEM_WRITING_ADDR_V  0xFFFFFFFF
#define ASSIST_DEBUG_LOG_MEM_WRITING_ADDR_S  0

#define ASSIST_DEBUG_LOG_MEM_FULL_FLAG_REG          (DR_REG_ASSIST_DEBUG_BASE + 0x090)
/* ASSIST_DEBUG_CLR_LOG_MEM_FULL_FLAG : R/W ;bitpos:[1] ;default: 1'b0 ; */
/*description: */
#define ASSIST_DEBUG_CLR_LOG_MEM_FULL_FLAG  (BIT(1))
#define ASSIST_DEBUG_CLR_LOG_MEM_FULL_FLAG_M  (BIT(1))
#define ASSIST_DEBUG_CLR_LOG_MEM_FULL_FLAG_V  0x1
#define ASSIST_DEBUG_CLR_LOG_MEM_FULL_FLAG_S  1
/* ASSIST_DEBUG_LOG_MEM_FULL_FLAG : RO ;bitpos:[0] ;default: 1'b0 ; */
/*description: */
#define ASSIST_DEBUG_LOG_MEM_FULL_FLAG  (BIT(0))
#define ASSIST_DEBUG_LOG_MEM_FULL_FLAG_M  (BIT(0))
#define ASSIST_DEBUG_LOG_MEM_FULL_FLAG_V  0x1
#define ASSIST_DEBUG_LOG_MEM_FULL_FLAG_S  0

#define ASSIST_DEBUG_CORE_0_LASTPC_BEFORE_EXCEPTION_REG          (DR_REG_ASSIST_DEBUG_BASE + 0x094)
/* ASSIST_DEBUG_CORE_0_LASTPC_BEFORE_EXC : RO ;bitpos:[31:0] ;default: 32'b0 ; */
/*description: */
#define ASSIST_DEBUG_CORE_0_LASTPC_BEFORE_EXC  0xFFFFFFFF
#define ASSIST_DEBUG_CORE_0_LASTPC_BEFORE_EXC_M  ((ASSIST_DEBUG_CORE_0_LASTPC_BEFORE_EXC_V)<<(ASSIST_DEBUG_CORE_0_LASTPC_BEFORE_EXC_S))
#define ASSIST_DEBUG_CORE_0_LASTPC_BEFORE_EXC_V  0xFFFFFFFF
#define ASSIST_DEBUG_CORE_0_LASTPC_BEFORE_EXC_S  0

#define ASSIST_DEBUG_CORE_0_DEBUG_MODE_REG          (DR_REG_ASSIST_DEBUG_BASE + 0x098)
/* ASSIST_DEBUG_CORE_0_DEBUG_MODULE_ACTIVE : RO ;bitpos:[1] ;default: 1'b0 ; */
/*description: */
#define ASSIST_DEBUG_CORE_0_DEBUG_MODULE_ACTIVE  (BIT(1))
#define ASSIST_DEBUG_CORE_0_DEBUG_MODULE_ACTIVE_M  (BIT(1))
#define ASSIST_DEBUG_CORE_0_DEBUG_MODULE_ACTIVE_V  0x1
#define ASSIST_DEBUG_CORE_0_DEBUG_MODULE_ACTIVE_S  1
/* ASSIST_DEBUG_CORE_0_DEBUG_MODE : RO ;bitpos:[0] ;default: 1'b0 ; */
/*description: */
#define ASSIST_DEBUG_CORE_0_DEBUG_MODE  (BIT(0))
#define ASSIST_DEBUG_CORE_0_DEBUG_MODE_M  (BIT(0))
#define ASSIST_DEBUG_CORE_0_DEBUG_MODE_V  0x1
#define ASSIST_DEBUG_CORE_0_DEBUG_MODE_S  0

#define ASSIST_DEBUG_DATE_REG          (DR_REG_ASSIST_DEBUG_BASE + 0x1FC)
/* ASSIST_DEBUG_DATE : R/W ;bitpos:[27:0] ;default: 28'h2008010 ; */
/*description: */
#define ASSIST_DEBUG_DATE  0x0FFFFFFF
#define ASSIST_DEBUG_DATE_M  ((ASSIST_DEBUG_DATE_V)<<(ASSIST_DEBUG_DATE_S))
#define ASSIST_DEBUG_DATE_V  0xFFFFFFF
#define ASSIST_DEBUG_DATE_S  0

#ifdef __cplusplus
}
#endif



#endif /*_SOC_ASSIST_DEBUG_REG_H_ */
