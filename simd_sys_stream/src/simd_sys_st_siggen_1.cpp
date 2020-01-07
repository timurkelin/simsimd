/*
 * simd_sys_st_siggen_1.cpp
 *
 *  Description:
 *    Methods of the 1-output 1-output Signal generator
 *    (used for Valid-Ready Interface testing)
 */

#include <boost/foreach.hpp>
#include <boost/functional/hash.hpp>

#include "simd_sys_st_siggen_1.h"
#include "simd_dump.h"
#include "simd_report.h"
#include "simd_assert.h"
#include "simd_trace.h"

namespace simd {

void simd_sys_st_siggen_1_c::init(
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

   // seed rng engine
   boost::hash<std::string> hash_str;
   rng_eng.seed( hash_str( name()));
}

void simd_sys_st_siggen_1_c::add_trace(
      sc_core::sc_trace_file* tf,
      const std::string& top_name ) {
   std::string mod_name = top_name + "." + std::string( name());

   add_trace_ports(
         tf,
         top_name );

   sc_core::sc_trace( tf, smp_count,   mod_name + ".smp_cnt"  );  // sample counter
}

// Function-specific execution request
void simd_sys_st_siggen_1_c::req_run(
      void ) {

   // Optional rng seed
   boost::optional<std::size_t> seed_p = config.at( active_config ).get_optional<std::size_t>( "seed" );

   if( seed_p ) { // seed exists
      rng_eng.seed( seed_p.get());
   }

   // Initialize vector size
   bool vec_size_init_done = false;

   if( !vec_size_init_done ) {
      boost::optional<std::size_t> vec_size_init_p;

      try {
         vec_size_init_p = config.at( active_config ).get_optional<std::size_t>( "vec_size" );
      }
      catch( const boost_pt::ptree_error& err ) {
         SIMD_REPORT_ERROR( "simd::sys_dm" ) << name() << " " <<  err.what();
      }
      catch( ... ) {
         SIMD_REPORT_ERROR( "simd::sys_dm" ) << name() << " Unexpected";
      }

      if( vec_size_init_p.is_initialized()) {
         vec_size = vec_size_init_p.get();
         vec_size_init_done = true;
      }
   }

   if( !vec_size_init_done ) {
      std::string vec_size_init;

      try {
         vec_size_init = config.at( active_config ).get<std::string>( "vec_size" );
      }
      catch( const boost_pt::ptree_error& err ) {
         SIMD_REPORT_ERROR( "simd::sys_dm" ) << name() << " " <<  err.what();
      }
      catch( ... ) {
         SIMD_REPORT_ERROR( "simd::sys_dm" ) << name() << " Unexpected";
      }

      if( vec_size_init == "rng" ) {
         // Vector size is Constrained random
         boost_rn::uniform_int_distribution<> rng_dist( 50, 100 );
         boost_rn::variate_generator<boost::mt19937&, boost_rn::uniform_int_distribution<> > rng_size( rng_eng, rng_dist );

         vec_size = rng_size();
         vec_size_init_done = true;
      }
      else { // No initialisation
         SIMD_REPORT_ERROR( "simd::sys_dm" ) << name() << " Invalid vec_size specification";
      }
   }

   smp_count = 0;

   try {
      valid_ovr    = config.at( active_config ).get<bool>( "valid_ovr" );
      slot_ena_ovr = config.at( active_config ).get<bool>( "slot_ena_ovr" );
      status.at( active_status ).put( "vec_size", vec_size );
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

// Clocked processing thread
void simd_sys_st_siggen_1_c::proc_thrd(
      void ) {

   boost_rn::uniform_real_distribution<> rng_dist_smp( -1.0, 1.0 );
   boost_rn::bernoulli_distribution <>   rng_dist_bern( 0.5 );

   // Used for values
   boost_rn::variate_generator<boost::mt19937&, boost_rn::uniform_real_distribution<> > rng_smp(  rng_eng, rng_dist_smp );

   // Used for slot enables
   boost_rn::variate_generator<boost::mt19937&, boost_rn::bernoulli_distribution<>    > rng_bern( rng_eng, rng_dist_bern );

   // Data dump buffer
   simd_dump_buf_c<simd_dmeu_data_c> dump_buf_data( std::string( name()) + ".data" );

   sc_core::wait();

   simd_dmeu_valid_t     src_valid = SIMD_IDLE;
   simd_dmeu_ready_t     dst_ready;
   bool                  dst_avail;
   simd_dmeu_data_c      src_data, src_csum;
   bool                  is_data = rng_bern();
   simd_sig_dmeu_state_c sig_state;
   simd_dmeu_state_t     state;
   bool                  state_upd;
   bool                  proc_en;
   bool                  smp_count_expired = false;

   for(;;) {
      sc_core::wait();

      // Read ports
      dst_avail = data_vo.at( 0 )->is_avail();
      dst_ready = data_vo.at( 0 )->is_ready();
      state     = state_o->read().get();
      proc_en   = proc_i->read();
      is_data  |= valid_ovr;

      if(( state == DMEU_ST_IDLE || state == DMEU_ST_PROC ) && proc_en ) {
         state_upd = false;

         if(( !smp_count_expired ) && is_data && dst_ready ) {
            // Transmit sample on this clock cycle
            if( smp_count == 0 ) { // Vector head
               src_valid = SIMD_HEAD;
               state_o->write( sig_state.set( DMEU_ST_PROC ));
               state_upd = true;
               event( "vec_head" );   // Generate event on the vector head marker
            }
            else if( smp_count == vec_size - 1 ) {   // Vector tail
               src_valid = SIMD_TAIL;
            }
            else {                        // Vector body
               src_valid = SIMD_BODY;
            }

            if(( src_valid == SIMD_HEAD ) || ( src_valid == SIMD_BODY )) {
               // Head or body - transmit random data and generate checksum
               for( std::size_t idx = 0; idx < src_csum.dim; idx ++ ) {
                  if( src_valid == SIMD_HEAD ) { // Initialise checksum
                     src_csum[idx].smp = simd_dmeu_smp_t( 0.0, 0.0 );
                     src_csum[idx].ena = true;
                  }

                  if( idx == 0 ) {  // Sample counter into slot 0
                     src_data[idx].ena = true;

                     src_data[idx].smp  = simd_dmeu_smp_t( smp_count, -smp_count );
                     src_csum[idx].smp += src_data[idx].smp;
                  }
                  else { // Random data into slot 1..dim-1
                     src_data[idx].ena = rng_bern() || slot_ena_ovr;

                     if( src_data[idx].ena ) {
                        src_data[idx].smp  = simd_dmeu_smp_t( rng_smp(), rng_smp());
                        src_csum[idx].smp += src_data[idx].smp;
                     }
                  }
               }

               // Dump vector head and body
               dump_buf_data.write( src_data, BUF_WRITE_CONT );
            }
            else {   // Tail - transmit checksum
               src_data = src_csum;

               // Dump vector tail with "last" flag
               dump_buf_data.write( src_data, BUF_WRITE_LAST );
            }

            // Write data to channel
            data_vo.at( 0 )->nb_write( src_data, src_valid );

            // Next state
            if( smp_count < vec_size - 1 ) {
               smp_count ++;
            }
            else {
               smp_count_expired = true;
            }

            is_data = rng_bern();
         }
         else if(( !smp_count_expired ) && ( !is_data )) {
            // Idle clock cycle
            if(( src_valid != SIMD_IDLE ) && dst_ready ) {
               src_valid = SIMD_IDLE;
               data_vo.at( 0 )->nb_valid( src_valid );
            }

            is_data = rng_bern();
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
            is_data = rng_bern();
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
   }
}

// Asynchronous ready thread
void simd_sys_st_siggen_1_c::ready_thrd(
      void ) {

   sc_core::wait();
}

} // namespace simd
