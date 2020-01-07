/*
 * simd_sys_st_inp_1.cpp
 *
 *  Description:
 *    Member functions of the stream block for data input to the vector core
 *    From the Core perspective this block represents Data source.
 *
 */

#include <boost/foreach.hpp>
#include <boost/functional/hash.hpp>
#include "simd_sys_st_inp_1.h"
#include "simd_sys_pool.h"
#include "simd_dump.h"
#include "simd_report.h"
#include "simd_assert.h"
#include "simd_trace.h"

namespace simd {

extern simd::simd_sys_pool_c  simd_sys_pool;

void simd_sys_st_inp_1_c::init(
      boost::optional<const boost_pt::ptree&> _pref_p ) {
   const boost_pt::ptree& pref = _pref_p.get();
   pref_p = _pref_p;

   // Set port sizes
   data_vo.init( n_data_o );

   // Set size of the internal signals
   ready_v.init( n_ready );

   // Set the vector sizes for the config and status slots
   try {
      std::size_t conf_size = pref.get<std::size_t>( "config_slots" );
      std::size_t stat_size = pref.get<std::size_t>( "status_slots" );
      std::size_t fifo_size = pref.get<std::size_t>( "fifo_depth" );

      if( conf_size > 0 ) {
         config.resize( conf_size );
      }
      else {
         SIMD_REPORT_ERROR( "simd::sys_st" ) << name() << " Incorrect config size";
      }

      if( stat_size >= 0 ) {
         status.resize( stat_size );
      }
      else {
         SIMD_REPORT_ERROR( "simd::sys_st" ) << name() << " Incorrect status size";
      }

      if( fifo_size > 0 ) {
         active_config_p = boost::optional<sc_core::sc_fifo<std::size_t>&>(
               *( new sc_core::sc_fifo<std::size_t>( std::string( name()).append( "_idx_fifo" ).c_str(), fifo_size )));
      }

      active_config = UINT_MAX;
      active_status = UINT_MAX;
   }
   catch( const boost_pt::ptree_error& err ) {
      SIMD_REPORT_ERROR( "simd::sys_st" ) << name() << " " <<  err.what();
   }
   catch( const std::exception& err ) {
      SIMD_REPORT_ERROR( "simd::sys_st" ) << name() << " " <<  err.what();
   }
   catch( ... ) {
      SIMD_REPORT_ERROR( "simd::sys_st" ) << name() << " Unexpected";
   }

   // seed rng engine with name
   boost::hash<std::string> hash_str;
   rng_eng.seed( hash_str( name()));
} // void simd_sys_st_inp_1_c::init(

void simd_sys_st_inp_1_c::add_trace(
      sc_core::sc_trace_file* tf,
      const std::string& top_name ) {
   std::string mod_name = top_name + "." + std::string( name());

   add_trace_ports(
         tf,
         top_name );

   sc_core::sc_trace( tf, smp_cnt,       mod_name + ".smp_cnt"  );  // sample counter

   sc_core::sc_trace( tf, blk_start,    mod_name + ".blk_start" );
   sc_core::sc_trace( tf, blk_size,     mod_name + ".blk_size"  );
   sc_core::sc_trace( tf, blk_offs,     mod_name + ".blk_offs"  );

   sc_core::sc_trace( tf, ws_min,       mod_name + ".ws_min"    );
   sc_core::sc_trace( tf, ws_max,       mod_name + ".ws_max"    );
   sc_core::sc_trace( tf, ws_cnt,       mod_name + ".ws_cnt"    );
} // void simd_sys_st_inp_1_c::add_trace(

void simd_sys_st_inp_1_c::req_run(   // Execution request
      void ) {

   try {
      pool_seg  = config.at( active_config ).get<std::string>( "pool_seg"  );
      blk_start = config.at( active_config ).get<std::size_t>( "blk_start" );
      blk_size  = config.at( active_config ).get<std::size_t>( "blk_size"  );

      ws_min    = config.at( active_config ).get<std::size_t>( "ws_min" ); // Range of the wait states
      ws_max    = config.at( active_config ).get<std::size_t>( "ws_max" );
   }
   catch( const boost_pt::ptree_error& err ) {
      SIMD_REPORT_ERROR( "simd::sys_st" ) << name() << " " <<  err.what();
   }
   catch( const std::exception& err ) {
      SIMD_REPORT_ERROR( "simd::sys_st" ) << name() << " " <<  err.what();
   }
   catch( ... ) {
      SIMD_REPORT_ERROR( "simd::sys_st" ) << name() << " Unexpected";
   }

   pool_seg_hash = simd_sys_pool.hash( pool_seg );

   blk_offs = 0;
   smp_cnt  = 0;      // Sample counter
   ws_cnt   = 0;      // Wait state counter

   // Reserve field for the vector size
   try {
      status.at( active_status ).put( "vec_size", 0 );
   }
   catch( const boost_pt::ptree_error& err ) {
      SIMD_REPORT_ERROR( "simd::sys_st" ) << name() << " " <<  err.what();
   }
   catch( const std::exception& err ) {
      SIMD_REPORT_ERROR( "simd::sys_st" ) << name() << " " <<  err.what();
   }
   catch( ... ) {
      SIMD_REPORT_ERROR( "simd::sys_st" ) << name() << " Unexpected";
   }

   // Initialize wait state rng
   boost_rn::uniform_int_distribution<>::param_type dist_param( ws_min, ws_max );
   rng_dist_uni.param( dist_param );
   rng_ws.engine() = rng_eng;
   rng_ws.distribution() = rng_dist_uni;

   // Generate event on the config slot set active
   event( "conf_active" );

   return;

} // void simd_sys_st_inp_1_c::req_run(

void simd_sys_st_inp_1_c::proc_thrd(
      void ) {

   // Data dump buffer
   simd_dump_buf_c<simd_dmeu_data_c> dump_buf_data( std::string( name()) + ".data" );

   // Process start
   sc_core::wait();

   simd_dmeu_valid_t     src_valid = SIMD_IDLE;
   simd_dmeu_ready_t     dst_ready;
   bool                  dst_avail;
   simd_dmeu_data_c      src_data, src_csum;
   simd_sig_dmeu_state_c sig_state;
   simd_dmeu_state_t     state;
   bool                  state_upd;
   bool                  proc_en;
   bool                  smp_count_expired = false;
   bool                  data_en = true;

   for(;;) {
      sc_core::wait();

      // Read ports
      dst_avail = data_vo.at( 0 )->is_avail();
      dst_ready = data_vo.at( 0 )->is_ready();
      state     = state_o->read().get();
      proc_en   = proc_i->read();

      // Update counter for the wait states
      if( data_en ) {
         ws_cnt = rng_ws();
      }
      else {
         if( ws_cnt != 0 ) {
            ws_cnt --;
         }
      }

      data_en = false;

      if(( state == DMEU_ST_IDLE || state == DMEU_ST_PROC ) && proc_en ) {
         state_upd = false;

         if(( !smp_count_expired ) && ( ws_cnt == 0 ) && dst_ready ) {
            data_en = true;

            // Transmit sample on this clock cycle
            if(( blk_offs + src_data.dim ) >= blk_size ) {   // Vector tail
               src_valid = SIMD_TAIL;

               if( blk_size <= src_data.dim ) {
                  event( "vec_head" );
               }

               smp_cnt ++;
               smp_count_expired = true;
            }
            else if( blk_offs == 0 ) {                       // Vector head
               src_valid = SIMD_HEAD;

               state_o->write( sig_state.set( DMEU_ST_PROC ));
               state_upd = true;

               event( "vec_head" );   // Generate event on the vector head marker
               smp_cnt ++;
            }
            else {                                           // Vector body
               src_valid = SIMD_BODY;
               smp_cnt ++;
            }

            // Read data from the pool segment
            for( std::size_t idx = 0; idx < src_data.dim; idx ++ ) {
               if( blk_offs < blk_size ) {
                  src_data[idx].ena = true;
                  src_data[idx].smp = simd_sys_pool.read(
                        pool_seg_hash,
                        blk_start + blk_offs );

                  blk_offs ++;
               }
               else {
                  src_data[idx].ena = false;
                  src_data[idx].smp = simd_dmeu_smp_t( 0.0, 0.0 );
               }
            }

            dump_buf_data.write(
                  src_data,
                  ( src_valid != SIMD_TAIL ) ? BUF_WRITE_CONT : BUF_WRITE_LAST );

            // Write data to channel
            data_vo.at( 0 )->nb_write( src_data, src_valid );
         }
         else if(( !smp_count_expired ) && ( ws_cnt != 0 )) {
            // Idle clock cycle
            if(( src_valid != SIMD_IDLE ) && dst_ready ) {
               src_valid = SIMD_IDLE;
               data_vo.at( 0 )->nb_valid( src_valid );
            }
         }
         else if( smp_count_expired && dst_avail ) {
            // Acknowledged TAIL
            if( src_valid != SIMD_TAIL ) {
               SIMD_REPORT_ERROR( "simd::sys_st" ) << name() << " Unexpected state";
            }

            event( "vec_tail" );   // Generate event on the acknowledged vector tail marker

            src_valid = SIMD_IDLE;
            data_vo.at( 0 )->nb_valid( src_valid );

            // Wait for the IDLE state confirmation
            state_o->write( sig_state.set( DMEU_ST_DONE ));
            state_upd = true;

            smp_count_expired = false;
         }

         if( state == DMEU_ST_IDLE && proc_en && !state_upd ) {
            // XBAR has established routing for the src module. Wait for valid data slot
            state_o->write( sig_state.set( DMEU_ST_PROC ));
         }
      }
      else if( state == DMEU_ST_DONE && !proc_en ) {
         // Try next configuration slot
         bool run_here = next_conf_run( active_config );

         // Try executing from FIFO (deferred)
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
   } // for(;;)
} // void simd_sys_st_inp_1_c::proc_thrd(

void simd_sys_st_inp_1_c::ready_thrd(
      void ) {

   sc_core::wait();
} // void simd_sys_st_inp_1_c::ready_thrd(

} // namespace simd
