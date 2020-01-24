/*
 * simd_sys_eb_dst.h
 *
 *  Description:
 *    Methods of the elastic buffer
 */

#include <boost/foreach.hpp>
#include "simd_sys_eb.h"
#include "simd_assert.h"
#include "simd_report.h"

namespace simd {

SC_HAS_PROCESS( simd::simd_sys_eb_dst_c );
simd_sys_eb_dst_c::simd_sys_eb_dst_c(
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
   SC_THREAD( async_thrd ); // Async thread

   SC_CTHREAD( sync_thrd, clock_i.pos() ); //  clocked thread
   reset_signal_is( reset_i, true );
}

void simd_sys_eb_dst_c::init(
      void ) {

}

// Trace internal signals
void simd_sys_eb_dst_c::add_trace(
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

// Asynchronous thread
void simd_sys_eb_dst_c::async_thrd(
      void ) {

   sc_core::wait(sc_core::SC_ZERO_TIME);

   for(;;) {
      // Create combined event
      sc_core::sc_event_or_list evt_or_list = ready_o.value_changed_event() |
                                            data_w_s1.value_changed_event() |
                                            data_w_s2.value_changed_event() |
                                           valid_w_s1.value_changed_event() |
                                           valid_w_s2.value_changed_event();

      sc_core::wait( evt_or_list );

      // MUXed output to DM/EU block
       data_o->write( ready_o->read().get() ?  data_w_s1.read() :  data_w_s2.read());
      valid_o->write( ready_o->read().get() ? valid_w_s1.read() : valid_w_s2.read());
   }
}

// Clocked thread
void simd_sys_eb_dst_c::sync_thrd(
      void ) {

   simd_sig_dmeu_ready_c ready_w;
   simd_sig_dmeu_data_c   data_w;
   simd_sig_dmeu_valid_c valid_w;

   simd_dmeu_ready_t ready_i_s0;
   simd_dmeu_valid_t valid_i_s0, valid_i_s1, valid_i_s2;
   simd_dmeu_data_c   data_i_s0,  data_i_s1,  data_i_s2;
   bool               proc_i_s0;

   simd_dmeu_ready_t ready_o_s0;
   simd_dmeu_valid_t valid_o_s0;

   // Reset state
   ready_o->write( ready_w.set( true ));
   tail_o->write( false );

   valid_i_s1 = SIMD_IDLE;
   valid_i_s2 = SIMD_IDLE;

   valid_w_s1.write( valid_w.set( valid_i_s1 ));
   valid_w_s2.write( valid_w.set( valid_i_s2 ));

   for(;;) {
      sc_core::wait();

      ready_i_s0 = ready_i->read().get();
      valid_i_s0 = valid_i->read().get();
      data_i_s0  =  data_i->read().get();
      proc_i_s0  =  proc_i->read();

      ready_o_s0 = ready_o->read().get();
      valid_o_s0 = valid_o->read().get();

      if( ready_o_s0 ) {
         if( valid_i_s1 != SIMD_IDLE && !ready_i_s0 ) {
            ready_o->write( ready_w.set( false ));

            valid_i_s2 = valid_i_s1;
             data_i_s2 =  data_i_s1;
         }
      }
      else {
         if( ready_i_s0 ) {
            ready_o->write( ready_w.set( true  ));
         }
      }

      // Register signals at the input from XBAR
      if( ready_o_s0 ) {
         valid_i_s1 = valid_i_s0;
          data_i_s1 =  data_i_s0;
      }

       data_w_s1.write(  data_w.set(  data_i_s1 ));
       data_w_s2.write(  data_w.set(  data_i_s2 ));
      valid_w_s1.write( valid_w.set( valid_i_s1 ));
      valid_w_s2.write( valid_w.set( valid_i_s2 ));

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
