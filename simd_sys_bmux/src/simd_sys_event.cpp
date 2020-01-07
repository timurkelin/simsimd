/*
 * simd_sys_event.h
 *
 *  Description:
 *    Methods of the event mux
 */

#include <boost/foreach.hpp>
#include "simd_sys_bmux.h"
#include "simd_dump.h"
#include "simd_assert.h"
#include "simd_report.h"

namespace simd {

SC_HAS_PROCESS( simd::simd_sys_event_c );
simd_sys_event_c::simd_sys_event_c(
      sc_core::sc_module_name nm )
   : sc_core::sc_module( nm )
   , clock_i(  "clock_i")
   , reset_i(  "reset_i")
   , event_vi( "event_vi")
   , event_o(  "event_o" ) {

   // Process registrations
   SC_THREAD( exec_thrd );
}

void simd_sys_event_c::init(
      const std::vector<simd_sys_dmeu_info_t>& dmeu_info ) {

   // Set port and channel sizes
   event_vi.init( dmeu_info.size() + 1 ); // We need to configure xbar as well

   // Save pointer to the vector of dm/eu parameters
   dmeu_info_p = boost::optional<const std::vector<simd_sys_dmeu_info_t>&>( dmeu_info );
}

void simd_sys_event_c::exec_thrd( void ) {
   sc_core::wait(sc_core::SC_ZERO_TIME);

   simd_dump_buf_c<boost_pt::ptree> dump_buf_data( std::string( name()) + ".data" );

   for(;;) {
      // Create combined event
      sc_core::sc_event_or_list event_vi_or_list;

      for( std::size_t n_event = 0; n_event < event_vi.size(); n_event ++ ) {
         event_vi_or_list |= event_vi.at( n_event )->data_written_event();
      }

      sc_core::wait( event_vi_or_list );

      // Resolve events which were triggered
      boost_pt::ptree  event_pt;
      simd_sig_ptree_c event_out;

      for( std::size_t n_event = 0; n_event < event_vi.size(); n_event ++ ) {

         while( event_vi.at( n_event )->num_available()) {
            event_pt = event_vi.at( n_event )->read().get();
            event_pt.put( "source",
                  ( n_event < event_vi.size() - 1 ) ? dmeu_info_p.get().at( n_event ).name
                                                    : "xbar" );

            // Write to the BMUXS data to the output
            event_o->write( event_out.set( event_pt ));

            // Dump events as they pass through the router
            dump_buf_data.write( event_pt, BUF_WRITE_LAST );
         }
      }
   }
}

} // namespace simd
