/*
 * simd_sys_eb.h
 *
 *  Description:
 *    Methods of the elastic buffer
 */

#include <boost/foreach.hpp>
#include "simd_sys_eb.h"
#include "simd_assert.h"
#include "simd_report.h"

namespace simd {

SC_HAS_PROCESS( simd::simd_sys_eb_src_c );
simd_sys_eb_src_c::simd_sys_eb_src_c(
      sc_core::sc_module_name nm )
   : sc_core::sc_module( nm )
   , clock_i( "clock_i")
   , reset_i( "reset_i")
   , data_i(  "data_i" )
   , valid_i( "valid_i")
   , ready_o( "ready_o")
   , data_o(  "data_o" )
   , valid_o( "valid_o")
   , ready_i( "ready_i")
   , proc_i(  "proc_i" )
   , tail_o(  "tail_o" ) {

   // Process registrations
   SC_CTHREAD( sync_thrd, clock_i.pos() ); //  clocked thread
   reset_signal_is( reset_i, true );
}

void simd_sys_eb_src_c::init(
      void ) {

}

// Trace internal signals
void simd_sys_eb_src_c::add_trace(
      sc_core::sc_trace_file* tf,
      const std::string& top_name ) {
   std::string mod_name = top_name + "." + std::string( name());

   sc_core::sc_trace( tf, data_i,  mod_name + ".data_i"  );
   sc_core::sc_trace( tf, valid_i, mod_name + ".valid_i" );
   sc_core::sc_trace( tf, ready_o, mod_name + ".ready_o" );

   sc_core::sc_trace( tf, data_o,  mod_name + ".data_o"  );
   sc_core::sc_trace( tf, valid_o, mod_name + ".valid_o" );
   sc_core::sc_trace( tf, ready_i, mod_name + ".ready_i" );

   sc_core::sc_trace( tf, proc_i,  mod_name + ".proc_i" );
   sc_core::sc_trace( tf, tail_o,  mod_name + ".tail_o" );
}

// Clocked thread
void simd_sys_eb_src_c::sync_thrd(
      void ) {

   simd_sig_dmeu_ready_c ready_w;
   simd_sig_dmeu_data_c   data_w;
   simd_sig_dmeu_valid_c valid_w;

   simd_dmeu_ready_t ready_i_s0;
   simd_dmeu_valid_t valid_i_s0, valid_i_s1;
   simd_dmeu_data_c   data_i_s0,  data_i_s1;
   bool               proc_i_s0;

   simd_dmeu_ready_t ready_o_s0;
   simd_dmeu_valid_t valid_o_s0;

   ready_o->write( ready_w.set( true ));
   tail_o->write( false );

   for(;;) {
      sc_core::wait();

      ready_i_s0 = ready_i->read().get();
      valid_i_s0 = valid_i->read().get();
      data_i_s0  =  data_i->read().get();
      proc_i_s0  =  proc_i->read();

      ready_o_s0 = ready_o->read().get();
      valid_o_s0 = valid_o->read().get();

      // Register signals at the output to XBAR
      if( ready_i_s0 ) {
         valid_o->write( valid_w.set( ready_o_s0 ? valid_i_s0 : valid_i_s1 ));
          data_o->write(  data_w.set( ready_o_s0 ?  data_i_s0 :  data_i_s1 ));
      }

      if( ready_o_s0 ) {
         if( valid_i_s0 != SIMD_IDLE && !ready_i_s0 ) {
            ready_o->write( ready_w.set( false ));

            valid_i_s1 = valid_i_s0;
             data_i_s1 =  data_i_s0;
         }
      }
      else {
         if( ready_i_s0 ) {
            ready_o->write( ready_w.set( true  ));
         }
      }

      // Tail detector
      if( proc_i_s0 == false ) {
         tail_o->write( false );
      }
      else if( ready_i_s0 ) {
         if( valid_o_s0 == SIMD_HEAD || valid_o_s0 == SIMD_BODY ) {
            tail_o->write( false );
         }
         else if( ready_i_s0 && valid_o_s0 == SIMD_TAIL ) {
            tail_o->write( true );
         }
      }
   }
}

} // namespace simd
