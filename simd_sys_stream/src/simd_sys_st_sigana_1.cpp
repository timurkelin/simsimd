/*
 * simd_sys_st_sigana_1.cpp
 *
 *  Description:
 *    Methods of the 1-input signal analiser
 *    (used for Valid-Ready Interface  testing)
 */

#include <boost/foreach.hpp>
#include <boost/functional/hash.hpp>

#include "simd_sys_st_sigana_1.h"
#include "simd_dump.h"
#include "simd_report.h"
#include "simd_assert.h"
#include "simd_trace.h"

namespace simd {

void simd_sys_st_sigana_1_c::init(
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

   // seed rng engine
   boost::hash<std::string> hash_str;
   rng_eng.seed( hash_str( name()));
}

void simd_sys_st_sigana_1_c::add_trace(
      sc_core::sc_trace_file* tf,
      const std::string& top_name ) {
   std::string mod_name = top_name + "." + std::string( name());

   add_trace_ports(
         tf,
         top_name );

   sc_core::sc_trace( tf, smp_count,   mod_name + ".smp_cnt"  );  // sample counter
}

void simd_sys_st_sigana_1_c::req_run(   // Execution request
      void ) {

   // Optional rng seed
   boost::optional<std::size_t> seed_p = config.at( active_config ).get_optional<std::size_t>( "seed" );

   if( seed_p ) { // seed exists
      rng_eng.seed( seed_p.get());
   }

   smp_count = 0;

   // Reserve field for the vector size
   try {
      status.at( active_status ).put( "vec_size", 0 );
      ready_ovr = config.at( active_config ).get<bool>( "ready_ovr" );
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

   // Generate event on the config slot set active
   event( "conf_active" );

   return;

}

void simd_sys_st_sigana_1_c::proc_thrd(
      void ) {

   // RNG id used for slot enables
   boost_rn::bernoulli_distribution <>   rng_dist_bern( 0.5 );
   boost_rn::variate_generator<boost::mt19937&, boost_rn::bernoulli_distribution<> > rng_bern( rng_eng, rng_dist_bern );

   // Data dump buffer
   simd_dump_buf_c<simd_dmeu_data_c> dump_buf_data( std::string( name()) + ".data" );

   // Process start
   sc_core::wait();

   simd_dmeu_ready_t     dst_ready = false;
   simd_dmeu_valid_t     src_valid;
   simd_dmeu_data_c      src_data, src_csum;
   simd_sig_dmeu_state_c sig_state;
   simd_dmeu_state_t     state;
   bool                  state_upd;
   bool                  proc_en;

   for(;;) {
      sc_core::wait();

      state   = state_o->read().get();
      proc_en = proc_i->read();

      // Process state
      if(( state == DMEU_ST_IDLE || state == DMEU_ST_PROC ) && proc_en ) {
         state_upd = false;

         if( dst_ready ) {
            data_vi.at( 0 )->nb_read( src_data, src_valid );

            if(( src_valid == SIMD_HEAD ) || ( src_valid == SIMD_BODY )) {
               for( std::size_t idx = 0; idx < src_csum.dim; idx ++ ) {
                  if( src_valid == SIMD_HEAD ) {
                     src_csum[idx].smp = simd_dmeu_smp_t( 0.0, 0.0 );
                     src_csum[idx].ena = true;
                  }

                  if( idx == 0 ) {
                     if( src_data[idx].ena != true ||
                         src_data[idx].smp.real() !=  smp_count ||
                         src_data[idx].smp.imag() != -smp_count ) {
                        SIMD_REPORT_WARNING( "simd::sys_st" ) << name() << " Incorrect sample sequence: "
                              << src_data[idx].smp.real() << " != " <<  smp_count << " "
                              << src_data[idx].smp.imag() << " != " << -smp_count;
                     }
                  }

                  if( src_data[idx].ena ) {
                     src_csum[idx].smp += src_data[idx].smp;
                  }
               }

               // Dump vector head and body
               dump_buf_data.write( src_data, BUF_WRITE_CONT );

               // Generate event on the vector head marker
               if( src_valid == SIMD_HEAD ) {
                  state_o->write( sig_state.set( DMEU_ST_PROC ));
                  state_upd = true;
                  event( "vec_head" );
               }

               smp_count ++;
            }
            else if( src_valid == SIMD_TAIL ) {
               if( !( src_data == src_csum )) {
                  std::string slot_str;

                  for( std::size_t idx = 0; idx < src_csum.dim; idx ++ ) {
                     if( src_csum[idx].smp != src_data[idx].smp ) {
                        slot_str += std::to_string( idx ) + " ";
                     }
                  }

                  SIMD_REPORT_WARNING( "simd::sys_st" ) << name() << " Checksum error. Slot: " << slot_str;
               }

               // Dump vector tail with "last" flag
               dump_buf_data.write( src_data, BUF_WRITE_LAST );

               // Generate event on the vector tail marker
               event( "vec_tail" );

               // Save received vector size to the active status slot
               try {
                  status.at( active_status ).put( "vec_size", smp_count + 1 );
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

               smp_count = 0;

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

      dst_ready = rng_bern() || ready_ovr;
      data_vi.at( 0 )->nb_ready( dst_ready );

      // Parse request on busw (1 per clock cycle)
      if( busw_i->num_available()) {
         simd_sig_ptree_c req = busw_i->read();

         parse_busw_req( req );
      }
   }
}

void simd_sys_st_sigana_1_c::ready_thrd(
      void ) {

   sc_core::wait();
}

} // namespace simd
