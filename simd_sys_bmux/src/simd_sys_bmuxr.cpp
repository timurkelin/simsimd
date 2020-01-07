/*
 * simd_sys_bmuxr.cpp
 *
 *  Description:
 *    System component: Status Bus multiplexer
 */

#include <boost/foreach.hpp>
#include "simd_sys_bmux.h"
#include "simd_assert.h"
#include "simd_report.h"

namespace simd {

SC_HAS_PROCESS( simd::simd_sys_bmuxr_c );
simd_sys_bmuxr_c::simd_sys_bmuxr_c(
      sc_core::sc_module_name nm )
   : sc_core::sc_module( nm )
   , clock_i( "clock_i")
   , reset_i( "reset_i")
   , busr_o( "busr_o" )
   , busr_vi("busr_vi") {

   // Process registrations
   SC_THREAD( exec_thrd );
}

void simd_sys_bmuxr_c::init(
      const std::vector<simd_sys_dmeu_info_t>& dmeu_info ) {

   // Set port and channel sizes
   busr_vi.init( dmeu_info.size() ); // We can request status from EU or DM

   // Save pointer to the vector of dm/eu parameters
   dmeu_info_p = boost::optional<const std::vector<simd_sys_dmeu_info_t>&>( dmeu_info );
}

void simd_sys_bmuxr_c::exec_thrd( void ) {
   sc_core::wait(sc_core::SC_ZERO_TIME);

   simd_dump_buf_c<boost_pt::ptree> dump_buf_data( std::string( name()) + ".data" );

   for(;;) {
      // Create combined event
      sc_core::sc_event_or_list status_vi_or_list;

      for( std::size_t n_status = 0; n_status < busr_vi.size(); n_status ++ ) {
         status_vi_or_list |= busr_vi.at( n_status )->data_written_event();
      }

      sc_core::wait( status_vi_or_list );

      // Resolve events which were triggered, and make a list of event sources
      for( std::size_t n_status = 0; n_status < busr_vi.size(); n_status ++ ) {
         while( busr_vi.at( n_status )->num_available()) {
            boost_pt::ptree  status_pt = busr_vi.at( n_status )->read().get();
            simd_sig_ptree_c status_out;

            status_pt.put( "source", dmeu_info_p.get().at( n_status ).name );

            // Write to the BMUXS data to the output
            busr_o->write( status_out.set( status_pt ));

            // Dump data as it passes through the router
            dump_buf_data.write( status_pt, BUF_WRITE_LAST );
         }
      }
   } // for(;;)
}

} // namespace simd
