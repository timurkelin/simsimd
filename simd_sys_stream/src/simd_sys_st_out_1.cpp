/*
 * simd_sys_st_out_1.cpp
 *
 *  Description:
 *    Member functions of the stream block for data output from the vector core
 *    From the Core perspective this block represents Data sink.
 *
 */
#include <boost/foreach.hpp>
#include <boost/functional/hash.hpp>
#include "simd_sys_st_out_1.h"
#include "simd_sys_pool.h"
#include "simd_dump.h"
#include "simd_report.h"
#include "simd_assert.h"
#include "simd_trace.h"

namespace simd {

extern simd::simd_sys_pool_c  simd_sys_pool;

void simd_sys_st_out_1_c::init(
      boost::optional<const boost_pt::ptree&> _pref_p ) {
   const boost_pt::ptree& pref = _pref_p.get();
   pref_p = _pref_p;

   // Set port sizes
   data_vi.init( n_data_i );

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
} // void simd_sys_st_out_1_c::init(

void simd_sys_st_out_1_c::add_trace(
      sc_core::sc_trace_file* tf,
      const std::string& top_name ) {
   std::string mod_name = top_name + "." + std::string( name());

   add_trace_ports(
         tf,
         top_name );

   sc_core::sc_trace( tf, smp_cnt,       mod_name + ".smp_cnt"  );  // sample counter

   sc_core::sc_trace( tf, blk_start,    mod_name + ".blk_start" );
   sc_core::sc_trace( tf, blk_offs,     mod_name + ".blk_offs"  );

   sc_core::sc_trace( tf, ws_min,       mod_name + ".ws_min"    );
   sc_core::sc_trace( tf, ws_max,       mod_name + ".ws_max"    );
   sc_core::sc_trace( tf, ws_cnt,       mod_name + ".ws_cnt"    );
} // void simd_sys_st_out_1_c::add_trace(

void simd_sys_st_out_1_c::req_run(   // Execution request
      void ) {

   try {
      pool_seg  = config.at( active_config ).get<std::string>( "pool_seg"  );
      blk_start = config.at( active_config ).get<std::size_t>( "blk_start" );

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

   blk_offs  = 0;   // Pool segment address
   smp_cnt   = 0;   // Sample counter
   ws_cnt    = 0;   // Wait state counter

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

} // void simd_sys_st_out_1_c::req_run(

void simd_sys_st_out_1_c::proc_thrd(
      void ) {

   // Data dump buffer
   simd_dump_buf_c<simd_dmeu_data_c> dump_buf_data( std::string( name()) + ".data" );

   // Process start
   sc_core::wait();

   simd_dmeu_ready_t     dst_ready = false;
   simd_dmeu_valid_t     src_valid;
   simd_dmeu_data_c      src_data;
   simd_sig_dmeu_state_c sig_state;
   simd_dmeu_state_t     state;
   bool                  state_upd;
   bool                  proc_en;

   for(;;) {
      sc_core::wait();

      bool data_en = false;

      state   = state_o->read().get();
      proc_en = proc_i->read();

      // Process state
      if(( state == DMEU_ST_IDLE || state == DMEU_ST_PROC ) && proc_en ) {
         state_upd = false;

         if( dst_ready ) {
            data_vi.at( 0 )->nb_read( src_data, src_valid );

            // Write sample into the pool segment
            if( src_valid != SIMD_IDLE ) {
               for( std::size_t idx = 0; idx < src_data.dim; idx ++ ) {
                  if( src_data[idx].ena ) {
                     simd_sys_pool.write(
                           pool_seg_hash,
                           blk_start + blk_offs,
                           src_data[idx].smp );

                     blk_offs ++;
                     data_en = true;
                  }
               }

               smp_cnt ++;

               // Save sample counter and block offset to the active status slot
               try {
                  status.at( active_status ).put( "smp_cnt",  smp_cnt  );
                  status.at( active_status ).put( "blk_offs", blk_offs );
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
            }

            // Process different stages of the vector
            if( src_valid == SIMD_HEAD ) {
               dump_buf_data.write( src_data, BUF_WRITE_CONT );      // Dump vector head and body

               // Generate event on the vector head marker
               state_o->write( sig_state.set( DMEU_ST_PROC ));
               state_upd = true;
               event( "vec_head" );
            }
            else if( src_valid == SIMD_BODY ) {
               dump_buf_data.write( src_data, BUF_WRITE_CONT );      // Dump vector head and body
            }
            else if( src_valid == SIMD_TAIL ) {
               dump_buf_data.write( src_data, BUF_WRITE_LAST );      // Dump vector tail with "last" flag

               // Generate event on the vector tail marker
               event( "vec_tail" );

               smp_cnt = 0;

               state_o->write( sig_state.set( DMEU_ST_DONE ));
               state_upd = true;
            }
         }

         if( state == DMEU_ST_IDLE && proc_en && !state_upd ) {
            // XBAR has established routing for the dst module. Wait for data slot
            state_o->write( sig_state.set( DMEU_ST_PROC ));
         }
      }
      else if( state == DMEU_ST_DONE && !proc_en ) {
         bool run_here = next_conf_run( active_config );

         if( !run_here && active_config_p && active_config_p.get().num_available()) {
            std::size_t conf_idx = active_config_p.get().read();

            active_config = conf_idx;  // Specify active config index
            active_status = curr_stat_idx( conf_idx );

            req_run();  // Function-specific configuration
         }

         state_o->write( sig_state.set( DMEU_ST_IDLE ));
      }

      // Update counter for the wait states
      if( data_en ) {
         ws_cnt = rng_ws();
      }
      else {
         if( ws_cnt != 0 ) {
            ws_cnt --;
         }
      }

      dst_ready = ( ws_cnt == 0 );
      data_vi.at( 0 )->nb_ready( dst_ready );

      // Parse request on busw (1 per clock cycle)
      if( busw_i->num_available()) {
         simd_sig_ptree_c req = busw_i->read();

         parse_busw_req( req );
      }
   } // for(;;) {
} // void simd_sys_st_out_1_c::proc_thrd(

void simd_sys_st_out_1_c::ready_thrd(
      void ) {

   sc_core::wait();
} // void simd_sys_st_out_1_c::ready_thrd(

} // namespace simd
