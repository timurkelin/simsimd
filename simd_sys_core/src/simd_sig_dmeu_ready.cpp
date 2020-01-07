/*
 * simd_sig_dmeu_ready.cpp
 *
 *  Description:
 *    Class methods for the signal which carries "destination ready" flag
 */

#include "simd_sig_dmeu_ready.h"
#include "simd_report.h"

namespace simd {

// Signal assignment
simd_sig_dmeu_ready_c& simd_sig_dmeu_ready_c::set(
      const simd_dmeu_ready_t& rhs ) {
   ready    = rhs;
   tr_ready = rhs;
   return *this;
}

const simd_dmeu_ready_t& simd_sig_dmeu_ready_c::get(
      void ) const {
   return ready;
}

// Required by sc_signal<> and sc_fifo<>
simd_sig_dmeu_ready_c& simd_sig_dmeu_ready_c::operator = (
      const simd_sig_dmeu_ready_c& rhs ) {
   ready    = rhs.ready;  // assign to the value
   tr_ready = rhs.ready;  // assign to the trace storage
   return *this;
}

// Required by sc_signal<>
bool simd_sig_dmeu_ready_c::operator == (
      const simd_sig_dmeu_ready_c& rhs) const {
   return ( ready == rhs.ready );
}

std::ostream& operator << (
      std::ostream& os,
      const simd::simd_sig_dmeu_ready_c& sig ) {

   os << (bool)( sig.ready );

   return os;
}
} // namespace simd

namespace sc_core {
void sc_trace(
      sc_core::sc_trace_file* tf,
      const simd::simd_sig_dmeu_ready_c& sig,
      const std::string& name ) {

   // for property tree we trace only hash value to observe the changes
   sc_core::sc_trace(
         tf,
         sig.tr_ready,
         name );
}
} // namespace sc_core
