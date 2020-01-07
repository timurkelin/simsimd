/*
 * simd_sys_eu_transp_a_1.cpp
 *
 *  Description:
 *    Methods of the 1-input, 1-output async transparent execution unit
 *    (used for testing)
 */

#include <boost/foreach.hpp>
#include "simd_sys_eu_transp_a_1.h"
#include "simd_assert.h"
#include "simd_report.h"

#define SC_INCLUDE_DYNAMIC_PROCESSES
#include <systemc>

namespace simd {

void simd_sys_eu_transp_a_1_c::init(
      boost::optional<const boost_pt::ptree&> _pref_p ) {
   const boost_pt::ptree& pref = _pref_p.get();
   pref_p = _pref_p;

   // Set port sizes
   data_vi.init( n_data_i );
   data_vo.init( n_data_o );

   // Set size of the internal signals
   ready_v.init( n_ready );

   // Register dynamic processes
   sc_core::sc_spawn( sc_core::sc_bind( &simd_sys_eu_transp_a_1_c::data_thrd, this ));

   // Set the vector sizes for the config and status slots
   try {
      std::size_t conf_size = pref.get<std::size_t>( "config_slots" );
      std::size_t stat_size = pref.get<std::size_t>( "status_slots" );
      std::size_t fifo_size = pref.get<std::size_t>( "fifo_depth" );

      if( conf_size > 0 ) {
         config.resize( conf_size );
      }
      else {
         SIMD_REPORT_ERROR( "simd::sys_eu" ) << name() << " Incorrect config size";
      }

      if( stat_size >= 0 ) {
         status.resize( stat_size );
      }
      else {
         SIMD_REPORT_ERROR( "simd::sys_eu" ) << name() << " Incorrect status size";
      }

      if( fifo_size > 0 ) {
         active_config_p = boost::optional<sc_core::sc_fifo<std::size_t>&>(
               *( new sc_core::sc_fifo<std::size_t>( std::string( name()).append( "_idx_fifo" ).c_str(), fifo_size )));
      }

      active_config = conf_size - 1;
      active_status = stat_size ? stat_size - 1 : 0;
   }
   catch( const boost_pt::ptree_error& err ) {
      SIMD_REPORT_ERROR( "simd::sys_eu" ) << name() << " " <<  err.what();
   }
   catch( const std::exception& err ) {
      SIMD_REPORT_ERROR( "simd::sys_eu" ) << name() << " " <<  err.what();
   }
   catch( ... ) {
      SIMD_REPORT_ERROR( "simd::sys_eu" ) << name() << " Unexpected";
   }
}

void simd_sys_eu_transp_a_1_c::add_trace(
      sc_core::sc_trace_file* tf,
      const std::string& top_name ) {
   //std::string mod_name = top_name + "." + std::string( name());

   add_trace_ports(
         tf,
         top_name );
}

void simd_sys_eu_transp_a_1_c::req_run(   // Execution request
      void ) {
   // No specific settings
}

void simd_sys_eu_transp_a_1_c::proc_thrd(
      void ) {

   simd_dmeu_valid_t dst_valid = SIMD_IDLE;
   simd_dmeu_ready_t dst_ready = false;
   bool              dst_avail;

   simd_dmeu_valid_t src_valid = SIMD_IDLE;
   simd_dmeu_data_c  src_data_dummy;
   bool              vec_tail = false;

   simd_sig_dmeu_state_c sig_state;

   sc_core::wait();

   for(;;) {
      sc_core::wait();

      // Read ports
      simd_dmeu_state_t state   = state_o->read().get(); // current state been sent to xbar
      bool              proc_en = proc_i->read();        // proc_en from xbar

      dst_ready = data_vo.at( 0 )->is_ready();

      if(( state == DMEU_ST_IDLE || state == DMEU_ST_PROC ) && proc_en ) {
         if( dst_ready ) { // Ready was set in the previous clock cycle
            data_vi.at( 0 )->nb_read( src_data_dummy, src_valid, false );
         }

         // Read ports
         dst_avail = data_vo.at( 0 )->is_avail();

         if( !vec_tail && dst_ready ) {
            // Try reading and transmit a sample on this clock cycle
            if( src_valid == SIMD_TAIL ) { // This condition should go first according to VRI
               dst_valid = SIMD_TAIL;
               vec_tail  = true;
            }
            else if( src_valid == SIMD_HEAD ) {
               state_o->write( sig_state.set( DMEU_ST_PROC ));
               dst_valid = SIMD_HEAD;
               event( "vec_head" );
            }
            else if( src_valid == SIMD_BODY ) {
               dst_valid = SIMD_BODY;
            }
            else {
               dst_valid = SIMD_IDLE;
            }
         }
         else if( vec_tail && dst_avail ) {
            // Acknowledged TAIL
            if( dst_valid != SIMD_TAIL ) {
               SIMD_REPORT_ERROR( "simd::sys_eu" ) << name() << " Unexpected state";
            }

            event( "vec_tail" );   // Generate event on the acknowledged vector tail marker

            state_o->write( sig_state.set( DMEU_ST_DONE ));

            vec_tail = false;
         }
      }

      if( state == DMEU_ST_DONE && !proc_en ) {
         bool run_here = next_conf_run( active_config );

         if( !run_here && active_config_p && active_config_p.get().num_available()) {
            std::size_t conf_idx = active_config_p.get().read();

            active_config = conf_idx;  // Specify active config index
            active_status = curr_stat_idx( conf_idx );

            req_run();  // Function-specific configuration
         }

         state_o->write( sig_state.set( DMEU_ST_IDLE ));
      }

      // Parse request on busw (1 per clock cycle)
      if( busw_i->num_available()) {
         simd_sig_ptree_c req = busw_i->read();

         parse_busw_req( req );
      }
   }
}

// Async thread to propagate ready signal
void simd_sys_eu_transp_a_1_c::ready_thrd(
      void ) {
   for(;;) {
      sc_core::wait( data_vo.at( 0 )->ready_event());

      data_vi.at( 0 )->nb_ready( data_vo.at( 0 )->is_ready());
   }
}

// Async thread to propagate data signal
void simd_sys_eu_transp_a_1_c::data_thrd(
      void ) {

   simd_dmeu_data_c  src_data;
   simd_dmeu_valid_t src_valid = SIMD_IDLE;
   bool              src_first = true;
   simd_dmeu_data_c  prv_data;
   simd_dmeu_data_c  dst_data;

   simd_dump_buf_c<simd_dmeu_data_c> dump_buf_data_i0( std::string( name()) + ".data_i0" );
   simd_dump_buf_c<simd_dmeu_data_c> dump_buf_data_o0( std::string( name()) + ".data_o0" );

   for(;;) {
      sc_core::sc_event_or_list evt_or_list;

      evt_or_list |= data_vi.at( 0 )->data_event();
      evt_or_list |= data_vi.at( 0 )->valid_event();

      sc_core::wait( evt_or_list );

      data_vi.at( 0 )->nb_read( src_data, src_valid, false );

      if( !( src_data == prv_data ) || src_first ) { // This condition is just to improve the simulation performance
         dst_data = src_data;

         prv_data  = src_data;
         src_first = false;
      }

      data_vo.at( 0 )->nb_write( dst_data, src_valid, false );

      // Write to the dump buffers
      dump_buf_data_i0.write(
            src_data,
            src_valid != SIMD_TAIL ? BUF_WRITE_CONT : BUF_WRITE_LAST );

      dump_buf_data_o0.write(
            dst_data,
            src_valid != SIMD_TAIL ? BUF_WRITE_CONT : BUF_WRITE_LAST );

   }
}

} // namespace simd
