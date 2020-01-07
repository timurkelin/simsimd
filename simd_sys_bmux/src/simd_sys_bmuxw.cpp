/*
 * simd_sys_bmuxw.cpp
 *
 *  Description:
 *    System component: Config Bus multiplexer
 */

#include <boost/foreach.hpp>
#include "simd_sys_bmux.h"
#include "simd_assert.h"
#include "simd_report.h"

namespace simd {

SC_HAS_PROCESS( simd::simd_sys_bmuxw_c );
simd_sys_bmuxw_c::simd_sys_bmuxw_c(
      sc_core::sc_module_name nm )
   : sc_core::sc_module( nm )
   , clock_i(  "clock_i")
   , reset_i(  "reset_i")
   , busw_i( "busw_i" )
   , busw_vo("busw_vo") {

   // Process registrations
   SC_THREAD( exec_thrd );
}

void simd_sys_bmuxw_c::init(
      const std::vector<simd_sys_dmeu_info_t>& dmeu_info ) {

   // Set port and channel sizes
   busw_vo.init( dmeu_info.size() + 1 ); // We need to configure xbar as well

   // Save pointer to the vector of dm/eu parameters
   dmeu_info_p = boost::optional<const std::vector<simd_sys_dmeu_info_t>&>( dmeu_info );
}

void simd_sys_bmuxw_c::exec_thrd( void ) {
   sc_core::wait(sc_core::SC_ZERO_TIME);

   simd_dump_buf_c<boost_pt::ptree> dump_buf_data( std::string( name()) + ".data" );

   for(;;) {
      sc_core::wait( busw_i->data_written_event());

      do {
         simd_sig_ptree_c busw_sig_i = busw_i->read();
         simd_sig_ptree_c busw_sig_o;

         std::string      dest_name; // List of destinations
         std::size_t      dest_idx;
         boost_pt::ptree  config;    // Common configuration

         try {
            dest_name     = busw_sig_i.get().get<std::string>("dest");
            busw_sig_o.set( busw_sig_i.get());
         }
         catch( const boost_pt::ptree_error& err ) {
            SIMD_REPORT_ERROR( "simd::sys_bmuxw" ) << err.what();
         }
         catch( ... ) {
            SIMD_REPORT_ERROR( "simd::sys_bmuxw" ) << "Unexpected";
         }

         // Dump data as it passes through the router
         dump_buf_data.write( busw_sig_i.get(), BUF_WRITE_LAST );

         if( dest_name == "broadcast" ) {
            // broadcast configuration packet to all dm-s, eu-s and to xbar
            for( dest_idx = 0; dest_idx <= dmeu_info_p.get().size(); dest_idx ++ ) {
               busw_vo.at(dest_idx)->write( busw_sig_o );
            }
         }
         else if( dest_name == "xbar" ) {
            // send configuration packet to xbar
            dest_idx = dmeu_info_p.get().size();
            busw_vo.at(dest_idx)->write( busw_sig_o );
         }
         else {
            // send configuration packet to the corresponding dm or eu
            for( dest_idx = 0; dest_idx < dmeu_info_p.get().size(); dest_idx ++ ) {
               if( dmeu_info_p.get().at(dest_idx).name == dest_name ) {
                  break;
               }
            }

            if( dest_idx == dmeu_info_p.get().size()) {
               SIMD_REPORT_ERROR( "simd::sys_bmuxw" ) << "Can't resolve destination:" << dest_name;
            }

            busw_vo.at(dest_idx)->write( busw_sig_o );
         }

      } while( busw_i->num_available());
   } // for(;;)
}

} // namespace simd
