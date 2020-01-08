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
} // simd_pref_c::load(

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
} // simd_pref_c::save(

boost::optional<const boost_pt::ptree&> simd_pref_c::get_pref(
      const std::string& field_name,
      bool               check_error ) {

   const boost_pt::ptree& root_r = root;
   boost::optional<const boost_pt::ptree&> field_p = root_r.get_child_optional( field_name );

   if( check_error && !field_p.is_initialized()) {
      SIMD_REPORT_ERROR( "schd::pref" ) << "Missing <"
                                        << field_name
                                        << "> specification.";
   }

   return field_p;
} // schd_pref_c::get_pref(

void simd_pref_c::parse(
      void ) {
   bool check_error = true;

   core_p   = get_pref( "core",   check_error );
   pool_p   = get_pref( "pool",   check_error );
   clock_p  = get_pref( "clock",  check_error );
   scalar_p = get_pref( "scalar", check_error );
   time_p   = get_pref( "time",   check_error );
   report_p = get_pref( "report", check_error );
   trace_p  = get_pref( "trace",  check_error );
   dump_p   = get_pref( "dump",   check_error );
} // simd_pref_c::parse(

} // namespace simd
