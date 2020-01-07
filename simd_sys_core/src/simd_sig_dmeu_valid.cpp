/*
 * simd_sig_dmeu_valid.cpp
 *
 *  Description:
 *    Class methods for the signal which carries "source data valid" flag
 */

#include "simd_sig_dmeu_valid.h"
#include "simd_report.h"

namespace simd {

// Signal assignment
simd_sig_dmeu_valid_c& simd_sig_dmeu_valid_c::set(
      const simd_dmeu_valid_t& rhs ) {
   valid    = rhs;
   tr_valid = rhs;
   return *this;
}

const simd_dmeu_valid_t& simd_sig_dmeu_valid_c::get(
      void ) const {
   return valid;
}

// Required by sc_signal<> and sc_fifo<>
simd_sig_dmeu_valid_c& simd_sig_dmeu_valid_c::operator = (
      const simd_sig_dmeu_valid_c& rhs ) {
   valid    = rhs.valid;  // assign to the value
   tr_valid = rhs.valid;  // assign to the trace storage
   return *this;
}

// Required by sc_signal<>
bool simd_sig_dmeu_valid_c::operator == (
      const simd_sig_dmeu_valid_c& rhs) const {
   return ( valid == rhs.valid );
}

std::ostream& operator << (
      std::ostream& os,
      const simd::simd_sig_dmeu_valid_c& sig ) {
   static std::string val_name[] = {
         "IDLE",
         "BODY",
         "HEAD",
         "TAIL" };

   os << val_name[(int)sig.valid];

   return os;
}

} // namespace simd

namespace sc_core {

void sc_trace(
      sc_core::sc_trace_file* tf,
      const simd::simd_sig_dmeu_valid_c& sig,
      const std::string& name ) {

   // for property tree we trace only hash value to observe the changes
   sc_core::sc_trace(
         tf,
         sig.tr_valid,
         name, 2 );
}

} // namespace sc_core
