/*
 * simd_common.h
 *
 *  Description:
 *    Global include file
 */

#ifndef SIMD_COMMON_INCLUDE_SIMD_COMMON_H_
#define SIMD_COMMON_INCLUDE_SIMD_COMMON_H_

// System components
#include "simd_pref.h"
#include "simd_sys_bmux.h"
#include "simd_sys_scalar.h"
#include "simd_sys_crm.h"
#include "simd_sys_core.h"
#include "simd_sys_pool.h"

// Simulation infrastructure
#include "simd_assert.h"
#include "simd_report.h"
#include "simd_time.h"
#include "simd_trace.h"
#include "simd_dump.h"

namespace simd {
   extern simd::simd_pref_c   simd_pref;
   extern simd::simd_report_c simd_report;
   extern simd::simd_time_c   simd_time;
   extern simd::simd_dump_c   simd_dump;

   extern simd::simd_sys_pool_c  simd_sys_pool;
}

#endif /* SIMD_COMMON_INCLUDE_SIMD_COMMON_H_ */
