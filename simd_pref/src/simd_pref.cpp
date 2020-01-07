/*
 * simd_pref.cpp
 *
 *  Description:
 *    Reader and initial parser of the simulation preferences
 */

#include <string>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/detail/file_parser_error.hpp>
#include "simd_pref.h"
#include "simd_report.h"

namespace boost_jp = boost::property_tree::json_parser;

namespace simd {
void simd_pref_c::load(
      const std::string& fname  ) {

   try {
      boost_pt::read_json(
            fname,
            root );
   }
   catch( const boost_jp::json_parser_error& err ) {
      SIMD_REPORT_ERROR( "simd::pref" ) << err.what();
   }
   catch( ... ) {
      SIMD_REPORT_ERROR( "simd::pref" ) << "Unexpected";
   }
}

void simd_pref_c::save(
      const std::string& fname  ) {

   try {
      boost_pt::write_json(
            fname,
            root );
   }
   catch( const boost_jp::json_parser_error& err ) {
      SIMD_REPORT_ERROR( "simd::pref" ) << err.what();
   }
   catch( ... ) {
      SIMD_REPORT_ERROR( "simd::pref" ) << "Unexpected";
   }
}

void simd_pref_c::parse(
      void ) {

   core_p   = root.get_child_optional( "core" );
   if( !core_p ) {
      SIMD_REPORT_ERROR( "simd::pref" ) << "Missing <core> specification.";
   }

   pool_p   = root.get_child_optional( "pool" );
   if( !pool_p ) {
      SIMD_REPORT_ERROR( "simd::pref" ) << "Missing <pool> specification.";
   }

   clock_p   = root.get_child_optional( "clock" );
   if( !clock_p ) {
      SIMD_REPORT_ERROR( "simd::pref" ) << "Missing <clock> specification.";
   }

   scalar_p = root.get_child_optional( "scalar" );
   if( !scalar_p ) {
      SIMD_REPORT_ERROR( "simd::pref" ) << "Missing <scalar> specification.";
   }

   time_p   = root.get_child_optional( "time" );
   if( !time_p ) {
      SIMD_REPORT_ERROR( "simd::pref" ) << "Missing <time> specification.";
   }

   report_p = root.get_child_optional( "report" );
   if( !report_p ) {
      SIMD_REPORT_INFO( "simd::pref" ) << "Missing <report> specification.";
   }

   dump_p   = root.get_child_optional( "dump" );
   if( !dump_p ) {
      SIMD_REPORT_INFO( "simd::pref" ) << "Missing <dump> specification.";
   }

   trace_p  = root.get_child_optional( "trace" );
   if( !trace_p ) {
      SIMD_REPORT_INFO( "simd::pref" ) << "Missing <trace> specification.";
   }
} // void simd_pref_c::parse(

} // namespace simd
