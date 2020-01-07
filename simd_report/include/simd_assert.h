/*
 * simd_assert.h
 *
 *  Description:
 *    assert macro specific to simulation SystemC phase
 */

#ifndef SIMD_REPORT_INCLUDE_SIMD_ASSERT_H_
#define SIMD_REPORT_INCLUDE_SIMD_ASSERT_H_

#include <cassert>
#include <systemc>

#define simd_assert( expr ) sc_assert( expr );

#endif /* SIMD_REPORT_INCLUDE_SIMD_ASSERT_H_ */
