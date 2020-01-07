/*
 * simd_sys_crm.cpp
 *
 *  Description:
 *    Clock and reset manager: channel and interface to the clocked modules
 */

#include <map>
#include <boost/container/throw_exception.hpp>

// temporarily disable pedantic warning for GCC
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include <boost/spirit/include/qi.hpp>
#pragma GCC diagnostic pop

#include <boost/lexical_cast.hpp>
#include "simd_sys_crm.h"

namespace boost_cn = boost::container;
namespace boost_qi = boost::spirit::qi;

namespace simd {

SC_HAS_PROCESS( simd::simd_sys_crm_c );
simd_sys_crm_c::simd_sys_crm_c(
      sc_core::sc_module_name nm ) : sc_core::sc_module( nm ) {
   // Initialisations
   clock_p = NULL;

   // Internal Channel allocations

   // Submodule allocations

   // Submodule Connections

   // Process registrations
   SC_THREAD( clock_thrd );   // thread for clock signal assignment to the output port
   SC_THREAD( reset_thrd );   // Reset generator
}

void simd_sys_crm_c::init(
      boost::optional<const boost_pt::ptree&> pref_p ) {

   const boost_pt::ptree& pref = pref_p.get();

   // Set resolution
   boost::optional<std::string> freq_p = pref.get_optional<std::string>("freq");

   if(( !freq_p ) || ( freq_p.get().length() == 0 )) {
      SIMD_REPORT_FATAL( "simd::sys_crm" ) << "Operating frequency is not set";
   }
   else {
      period = to_period( freq_p.get() );
      SIMD_REPORT_INFO( "simd::sys_crm" ) << "Operating frequency is set to <" << freq_p.get() << ">";
   }

   clock_p = new sc_core::sc_clock(
         /*name_         */ "clk",
         /*period_       */ period,
         /*duty_cycle_   */ 0.5,
         /*start_time_   */ sc_core::SC_ZERO_TIME,
         /*posedge_first_*/ true );
}

void simd_sys_crm_c::clock_thrd(
      void ) {
   clock_o->write( false );

   for(;;) {
      sc_core::wait( clock_p->value_changed_event());
      clock_o->write( clock_p->read());
   }
}

void simd_sys_crm_c::reset_thrd(
      void ) {
   reset_o->write( false );
   sc_core::wait( sc_core::SC_ZERO_TIME );

   reset_o->write( false );
   for( int cyc = 0; cyc < 2; cyc ++ ) { // Wait for 2 clock cycles
      sc_core::wait( clock_p->posedge_event());
   }

   reset_o->write( true );
   for( int cyc = 0; cyc < 2; cyc ++ ) { // Wait for 2 clock cycles
      sc_core::wait( clock_p->posedge_event());
   }

   reset_o->write( false );
   for(;;) {    // Wait forever;
      sc_core::wait(); // Static sensitivity with no events in the sensitivity list
   }
}

std::map<std::string, sc_core::sc_time_unit> known_units_freq = {
   { "Hz" , sc_core::SC_SEC},
   { "kHz", sc_core::SC_MS },
   { "MHz", sc_core::SC_US },
   { "GHz", sc_core::SC_NS },
   { "THz", sc_core::SC_PS }
};

sc_core::sc_time simd_sys_crm_c::to_period(
      const std::string& str ) {
   double                val;
   std::string           unit_str;

   bool done = boost_qi::phrase_parse(
         str.begin(),
         str.end(),
         boost_qi::double_ >> ( boost_qi::string("Hz")  |
                                boost_qi::string("kHz") |
                                boost_qi::string("MHz") |
                                boost_qi::string("GHz") |
                                boost_qi::string("THz") ),
         boost_qi::space,
         val,
         unit_str );

   if( !done ) {
      SIMD_REPORT_ERROR( "simd::sys_crm" ) << "Incorrect format: " << str;
   }
   else if( val <= 0 ) {
      SIMD_REPORT_FATAL( "simd::sys_crm" ) << "Incorrect frequency value";
   }

   return sc_core::sc_time( 1./val, known_units_freq[unit_str] );
} // simd_sys_crm_c::to_period(...)

} // namespace simd
