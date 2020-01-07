/*
 * simd_time.cpp
 *
 *  Description:
 *    Simulation time and resolution
 */

#include <utility>
#include <map>
#include <tuple>
#include <boost/container/throw_exception.hpp>

// temporarily disable pedantic warning for GCC
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include <boost/spirit/include/qi.hpp>
#pragma GCC diagnostic pop

#include <boost/lexical_cast.hpp>

#include "simd_report.h"
#include "simd_time.h"

namespace boost_cn = boost::container;
namespace boost_qi = boost::spirit::qi;

namespace simd {
void simd_time_c::init(
      boost::optional<const boost_pt::ptree&> pref_p ) {

   const boost_pt::ptree& pref = pref_p.get();

   // Set resolution
   boost::optional<std::string> res_p = pref.get_optional<std::string>("resolution");

   if( res_p.is_initialized() && ( res_p.get().length() != 0 )) {
      res_str = res_p.get();
      res_sec = to_time_sec( res_str );
      SIMD_REPORT_INFO( "simd::time" ) << "Time resolution is set to <" << res_sec << " s >";
   }
   else {
      SIMD_REPORT_FATAL( "simd::time" ) << "Time resolution is not set";
   }

   // Set end time
   boost::optional<std::string> end_p = pref.get_optional<std::string>("finish");

   if( end_p.is_initialized() && ( end_p.get().length() != 0 )) {
      end_str = end_p.get();
      end_sec = to_time_sec( end_str );
      SIMD_REPORT_INFO( "simd::time" ) << "Finish time is set to <" << end_sec << " s >";
   }
   else {
      SIMD_REPORT_WARNING( "simd::time" ) << "Finish time is not set";
      end_sec = 0.0;
      end_str = "0.0 s";
   }
} // simd_time_c::init(...)

std::map<std::string, std::tuple<sc_core::sc_time_unit, double>> known_units_time = {
   { "sec", std::tuple<sc_core::sc_time_unit, double>{sc_core::SC_SEC, 1.0     }},
   { "ms",  std::tuple<sc_core::sc_time_unit, double>{sc_core::SC_MS,  1.0e-3  }},
   { "us",  std::tuple<sc_core::sc_time_unit, double>{sc_core::SC_US,  1.0e-6  }},
   { "ns",  std::tuple<sc_core::sc_time_unit, double>{sc_core::SC_NS,  1.0e-9  }},
   { "ps",  std::tuple<sc_core::sc_time_unit, double>{sc_core::SC_PS,  1.0e-12 }},
   { "fs",  std::tuple<sc_core::sc_time_unit, double>{sc_core::SC_FS,  1.0e-15 }}};

double simd_time_c::to_time_sec(
      const std::string& str) {
   double                val;
   std::string           unit_str;

   bool done = boost_qi::phrase_parse(
         str.begin(),
         str.end(),
         boost_qi::double_ >> ( boost_qi::string("sec") |
                                boost_qi::string("ms" ) |
                                boost_qi::string("us" ) |
                                boost_qi::string("ns" ) |
                                boost_qi::string("ps" ) |
                                boost_qi::string("fs" ) ),
         boost_qi::space,
         val,
         unit_str );

   if( !done ) {
      SIMD_REPORT_ERROR( "simd::time" ) << "Incorrect format: " << str;
   }
   else if( val <= 0 ) {
      SIMD_REPORT_FATAL( "simd::time" ) << "Incorrect frequency value";
   }

   return val * std::get<1>( known_units_time[unit_str] );
} // simd_time_c::to_time_sec(...)

} // namespace simd
