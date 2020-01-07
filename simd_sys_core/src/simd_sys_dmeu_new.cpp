/*
 * simd_sys_dmeu_new.cpp
 *
 *  Description:
 *    Method for creating new DM or EU blocks from the functional name
 */

#include "simd_sys_core.h"
#include "simd_assert.h"
#include "simd_report.h"

// Declaration of the EU classes
#include "simd_sys_eu_add_sub_2.h"
#include "simd_sys_eu_transp_s_1.h"
#include "simd_sys_eu_transp_a_1.h"

// Declaration of the DM classes
#include "simd_sys_dm_ram_1rw.h"
#include "simd_sys_dm_ram_1r1w.h"

// Declaration of the Stream classes
#include "simd_sys_st_inp_1.h"
#include "simd_sys_st_out_1.h"
#include "simd_sys_st_sigana_1.h"
#include "simd_sys_st_siggen_1.h"

namespace simd {

boost::optional<simd_sys_dmeu_c &> simd_sys_core_c::simd_sys_dmeu_new(
      const std::string& func,
      const sc_core::sc_module_name& name ) {
   simd_sys_dmeu_c* ptr = NULL;

   // Resolve EU functions
   if(      func == simd_sys_eu_add_sub_2_c::func() ) ptr = new simd_sys_eu_add_sub_2_c(  name );
   else if( func == simd_sys_eu_transp_s_1_c::func()) ptr = new simd_sys_eu_transp_s_1_c( name );
   else if( func == simd_sys_eu_transp_a_1_c::func()) ptr = new simd_sys_eu_transp_a_1_c( name );

   // Resolve DM functions
   else if( func == simd_sys_dm_ram_1rw_c::func()   ) ptr = new simd_sys_dm_ram_1rw_c(  name );
   else if( func == simd_sys_dm_ram_1r1w_c::func()  ) ptr = new simd_sys_dm_ram_1r1w_c( name );

   // Resolve Stream functions
   else if( func == simd_sys_st_siggen_1_c::func()  ) ptr = new simd_sys_st_siggen_1_c( name );
   else if( func == simd_sys_st_sigana_1_c::func()  ) ptr = new simd_sys_st_sigana_1_c( name );
   else if( func == simd_sys_st_inp_1_c::func() ) ptr = new simd_sys_st_inp_1_c( name );
   else if( func == simd_sys_st_out_1_c::func() ) ptr = new simd_sys_st_out_1_c( name );

   // Give up
   else
      SIMD_REPORT_ERROR( "simd::sys_core" ) << "Unable to resolve <" << func << ">";

   return boost::optional<simd_sys_dmeu_c &>( *ptr );
}

} // namespace simd
