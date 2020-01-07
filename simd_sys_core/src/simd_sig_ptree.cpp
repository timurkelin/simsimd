/*
 * simd_sig_ptree.cpp
 *
 *  Description:
 *    Class methods for the signal which carries a generic property tree
 *    This signal is used to transfer configuration and status data
 */

#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/detail/file_parser_error.hpp>
#include <boost/container/map.hpp>
#include <boost/container/throw_exception.hpp>
#include "simd_sig_ptree.h"
#include "simd_report.h"
#include "simd_hash_ptree.h"

namespace boost_jp = boost::property_tree::json_parser;

namespace simd {

// Signal assignment
simd_sig_ptree_c& simd_sig_ptree_c::set(
      const boost_pt::ptree& rhs ) {
   data = rhs;

   tr_hash = 0;
   boost::hash_combine(
         tr_hash,
         rhs );

   return *this;
}

const boost_pt::ptree& simd_sig_ptree_c::get(
      void ) const {
   return data;
}

// Required by sc_signal<> and sc_fifo<>
simd_sig_ptree_c& simd_sig_ptree_c::operator = (
      const simd_sig_ptree_c& rhs ) {
   data = rhs.data;

   tr_hash = 0;
   boost::hash_combine(
         tr_hash,
         rhs.data );

   return *this;
}

// Required by sc_signal<>
bool simd_sig_ptree_c::operator == (
      const simd_sig_ptree_c& rhs) const {
   return ( data == rhs.data );
}

std::ostream& operator << (
      std::ostream& os,
      const simd::simd_sig_ptree_c& sig ) {

   try {
      boost_jp::write_json(
            os,
            sig.data );
   }
   catch( const boost_jp::json_parser_error& err ) {
      SIMD_REPORT_ERROR( "simd::sig_ptree" ) << err.what();
   }
   catch( ... ) {
      SIMD_REPORT_ERROR( "simd::sig_ptree" ) << "Unexpected";
   }

   return os;
}
} // namespace simd

namespace sc_core {
void sc_trace(
      sc_core::sc_trace_file* tf,
      const simd::simd_sig_ptree_c& sig,
      const std::string& name ) {

   // for property tree we trace only hash value to observe the changes
   sc_core::sc_trace(
         tf,
         sig.tr_hash,
         name + ".hash" );
}
} // namespace sc_core
