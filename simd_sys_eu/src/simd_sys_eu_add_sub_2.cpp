/*
 * simd_sys_eu_add_sub_2.cpp
 *
 *  Description:
 *    Methods for the 2-input add/subtract execution unit
 *    This is an example of the multiple IN ports feeding data into a single OUT port
 */

#include "simd_sys_eu_add_sub_2.h"
#include "simd_assert.h"
#include "simd_report.h"

namespace simd {

void simd_sys_eu_add_sub_2_c::init(
      boost::optional<const boost_pt::ptree&> _pref_p ) {

   const boost_pt::ptree& pref = _pref_p.get();
   pref_p = _pref_p;

   // Set port sizes
   data_vi.init( n_data_i );
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

void simd_sys_eu_add_sub_2_c::add_trace(
      sc_core::sc_trace_file* tf,
      const std::string& top_name ) {
   //std::string mod_name = top_name + "." + std::string( name());

   add_trace_ports(
         tf,
         top_name );
}

void simd_sys_eu_add_sub_2_c::req_run(   // Execution request
      void ) {

   bool flg_neg_re0 = false;
   bool flg_neg_im0 = false;
   bool flg_neg_re1 = false;
   bool flg_neg_im1 = false;

   // Sign manipulation of the operands
   try {
      flg_neg_re0 = config.at( active_config ).get<bool>( "neg_re0" );
      flg_neg_im0 = config.at( active_config ).get<bool>( "neg_im0" );
      flg_neg_re1 = config.at( active_config ).get<bool>( "neg_re1" );
      flg_neg_im1 = config.at( active_config ).get<bool>( "neg_im1" );
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

   // Parse the sign strings
   sign_re0 = flg_neg_re0 ? -1.0 : +1.0;
   sign_im0 = flg_neg_im0 ? -1.0 : +1.0;
   sign_re1 = flg_neg_re1 ? -1.0 : +1.0;
   sign_im1 = flg_neg_im1 ? -1.0 : +1.0;

   // Operation when the sample in the frame is not available
   try {
      smp_nena_op = config.at( active_config ).get<std::string>( "smp_nena_op" );
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

   if( smp_nena_op != "zero" &&
       smp_nena_op != "nena" &&
       smp_nena_op != "error" ) {
      SIMD_REPORT_ERROR( "simd::sys_eu" ) << name() << " Incorrect specification";
   }
}

void simd_sys_eu_add_sub_2_c::proc_thrd(
      void ) {

   simd_dmeu_valid_t dst_valid = SIMD_IDLE;
   simd_dmeu_data_c  dst_data;
   simd_dmeu_ready_t dst_ready = false;
   bool              dst_avail;

   std::vector<simd_dmeu_valid_t> src_valid( n_data_i, SIMD_IDLE );
   std::vector<simd_dmeu_data_c>  src_data(  n_data_i );
   std::vector<simd_dmeu_ready_t> src_ready( n_data_i );
   bool                           vec_tail = false;

   simd_sig_dmeu_state_c sig_state;

   std::vector<simd_dump_buf_c<simd_dmeu_data_c>> dump_buf_data_vi{
      std::string( name()) + ".data_i0",
      std::string( name()) + ".data_i1"
   };

   simd_dump_buf_c<simd_dmeu_data_c> dump_buf_data_o0(
      std::string( name()) + ".data_o0" );

   sc_core::wait();

   for(;;) {
      sc_core::wait();

      bool all_inp_avail = true;

      // Read ports
      simd_dmeu_state_t state   = state_o->read().get(); // current state been sent to xbar
      bool              proc_en = proc_i->read();        // proc_en from xbar

      dst_ready = data_vo.at( 0 )->is_ready();

      if(( state == DMEU_ST_IDLE || state == DMEU_ST_PROC ) && proc_en ) {

         // Read data and valid from the input ports if their "ready" is set
         for( size_t data_i = 0; data_i < n_data_i; data_i ++ ) {
            if( src_ready.at( data_i ) ) { // SRC Ready was set in the previous clock cycle
               data_vi.at( data_i )->nb_read( // Read the input port if its "ready" signal is set
                     src_data.at(  data_i ),
                     src_valid.at( data_i ));
            }
         }

         // Read ports
         dst_avail = data_vo.at( 0 )->is_avail();

         if( !vec_tail && dst_ready ) {
            for( size_t data_i = 0; data_i < n_data_i; data_i ++ ) {
               if( src_valid.at( data_i ) == SIMD_IDLE ) {
                  all_inp_avail = false;
                  break;
               }
               else if(( data_i != 0 ) && ( src_valid.at( data_i ) != src_valid.at( 0 ))) {
                  SIMD_REPORT_ERROR( "simd::sys_eu" ) << name() << " Size mismatch for the input vectors";
               }
            }

            // Try reading and transmit a sample on this clock cycle
            if( all_inp_avail ) {
               if( src_valid.at( 0 ) == SIMD_TAIL ) { // This condition should go first according to VRI
                  dst_valid = SIMD_TAIL;
                  vec_tail  = true;
               }
               else if( src_valid.at( 0 ) == SIMD_HEAD ) {
                  state_o->write( sig_state.set( DMEU_ST_PROC ));
                  dst_valid = SIMD_HEAD;
                  event( "vec_head" ); // Generate "head" event for the output vector
               }
               else if( src_valid.at( 0 ) == SIMD_BODY ) {
                  dst_valid = SIMD_BODY;
               }
               else {
                  dst_valid = SIMD_IDLE;
               }

               // EU processing core
               for( size_t slot = 0; slot < simd_dmeu_data_c::dim; slot ++ ) {
                  if( src_data.at( 0 )[slot].ena & src_data.at( 1 )[slot].ena ) { // both samples are available
                     dst_data[slot].smp = simd_dmeu_smp_t(
                           src_data.at( 0 )[slot].smp.real() * sign_re0 + src_data.at( 1 )[slot].smp.real() * sign_re1,
                           src_data.at( 0 )[slot].smp.imag() * sign_im0 + src_data.at( 1 )[slot].smp.imag() * sign_im1 );
                     dst_data[slot].ena = true;
                  }
                  else if( src_data.at( 0 )[slot].ena ^ src_data.at( 1 )[slot].ena ) { // One sample is unavailable
                     if(      smp_nena_op == "zero" ) {
                        if( src_data.at( 0 )[slot].ena ) {
                           dst_data[slot].smp = simd_dmeu_smp_t(
                                 src_data.at( 0 )[slot].smp.real() * sign_re0,
                                 src_data.at( 0 )[slot].smp.imag() * sign_im0 );
                        }
                        else {
                           dst_data[slot].smp = simd_dmeu_smp_t(
                                 src_data.at( 1 )[slot].smp.real() * sign_re1,
                                 src_data.at( 1 )[slot].smp.imag() * sign_im1 );
                        }

                        dst_data[slot].ena = true;
                     }
                     else if( smp_nena_op == "nena" )  {
                        dst_data[slot].smp = simd_dmeu_smp_t( 0.0, 0.0 );
                        dst_data[slot].ena = false;
                     }
                     else if( smp_nena_op == "error" ) {
                        SIMD_REPORT_ERROR( "simd::sys_eu" ) << name() << " Incorrect sample";
                     }
                  }
                  else {
                     dst_data[slot].smp = simd_dmeu_smp_t( 0.0, 0.0 );
                     dst_data[slot].ena = false;
                  }
               }

               // Write data to channel
               data_vo.at( 0 )->nb_write( dst_data, dst_valid );

               // Write to the dump buffers
               for( size_t data_i = 0; data_i < n_data_i; data_i ++ ) {
                  dump_buf_data_vi.at( data_i ).write(
                        src_data.at( data_i ),
                        src_valid.at( 0 ) != SIMD_TAIL ? BUF_WRITE_CONT : BUF_WRITE_LAST );
               }

               dump_buf_data_o0.write(
                     dst_data,
                     dst_valid != SIMD_TAIL ? BUF_WRITE_CONT : BUF_WRITE_LAST );

            } // if( all_inp_avail )
            else {
               data_vo.at( 0 )->nb_valid( SIMD_IDLE );
            } // if( all_inp_avail ) ... else ...

         } // if( !vec_tail && dst_ready )
         else if( vec_tail && dst_avail ) {
            // Acknowledged TAIL
            if( dst_valid != SIMD_TAIL ) {
               SIMD_REPORT_ERROR( "simd::sys_eu" ) << name() << " Unexpected state";
            }

            event( "vec_tail" );   // Generate event on the acknowledged tail marker of the output vector

            state_o->write( sig_state.set( DMEU_ST_DONE ));
            dst_valid = SIMD_IDLE;
            data_vo.at( 0 )->nb_valid( dst_valid );

            vec_tail = false;
         }
      }

      // Propagate dst_ready to the source ports
      for( size_t data_i = 0; data_i < n_data_i; data_i ++ ) {
         if( all_inp_avail || src_valid.at( data_i ) == SIMD_IDLE ) {
            src_ready.at( data_i ) = dst_ready;
         }
         else {
            src_ready.at( data_i ) = false;
         }

         data_vi.at( data_i )->nb_ready( src_ready.at( data_i ));
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

void simd_sys_eu_add_sub_2_c::ready_thrd(
      void ) {

   for(;;) {
      sc_core::wait(); // We process "ready" in the synchronous process
   }
}

} // namespace simd
