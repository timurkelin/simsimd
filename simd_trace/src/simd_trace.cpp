/*
 * simd_trace.cpp
 *
 *  Description:
 *    VCD trace
 */

#include <utility>
#include <boost/foreach.hpp>
#include "simd_trace.h"
#include "simd_report.h"

namespace simd {
void simd_trace_c::init(
      boost::optional<const boost_pt::ptree&> pref_p,
      sc_core::sc_trace_file* tf_ ) {

   const boost_pt::ptree& pref = pref_p.get();

   boost::optional<std::string> trace_p = pref.get_optional<std::string>("file");

   // Set log file name
   if(( !trace_p ) || ( trace_p.get().length() == 0 )) {
      SIMD_REPORT_INFO( "simd::trace" ) << "VCD trace is disabled";
      tf = NULL;
   }
   else if( tf_ == NULL ) {
      tf = sc_core::sc_create_vcd_trace_file( trace_p.get().c_str() );

      if( tf ) {
         SIMD_REPORT_INFO(  "simd::trace" ) << "VCD trace is set to <" << trace_p.get() << ".vcd>";
      }
      else {
         SIMD_REPORT_ERROR( "simd::trace" ) << "Unable to set VCD trace <" << trace_p.get() << ".vcd>";
      }
   }
   else {
      tf = tf_;

      SIMD_REPORT_INFO(  "simd::trace" ) << "VCD trace is set by reference";
   }

   tf_ref = ( tf_ != NULL );
} // void simd_trace_c::init(

simd_trace_c::~simd_trace_c(
      void ) {
   if( tf != NULL && !tf_ref ) {
      sc_core::sc_close_vcd_trace_file( tf );
   }
}

// Trace instance
simd_trace_c simd_trace;

} // namespace simd
