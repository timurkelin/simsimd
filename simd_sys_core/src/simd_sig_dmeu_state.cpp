/*
 * simd_sig_dmeu_state.cpp
 *
 *  Description:
 *    Class methods for the signal which carries DM/EU state
 */

#include "simd_sig_dmeu_state.h"
#include "simd_report.h"

namespace simd {

// Signal assignment
simd_sig_dmeu_state_c& simd_sig_dmeu_state_c::set(
      const simd_dmeu_state_t& rhs ) {
   state    = rhs;
   tr_state = rhs;
   return *this;
}

const simd_dmeu_state_t& simd_sig_dmeu_state_c::get(
      void ) const {
   return state;
}

// Required by sc_signal<> and sc_fifo<>
simd_sig_dmeu_state_c& simd_sig_dmeu_state_c::operator = (
      const simd_sig_dmeu_state_c& rhs ) {
   state    = rhs.state;  // assign to the value
   tr_state = rhs.state;  // assign to the trace storage
   return *this;
}

// Required by sc_signal<>
bool simd_sig_dmeu_state_c::operator == (
      const simd_sig_dmeu_state_c& rhs) const {
   return ( state == rhs.state );
}

std::ostream& operator << (
      std::ostream& os,
      const simd::simd_sig_dmeu_state_c& sig ) {
   static std::string state_name[] = {
         "IDLE",
         "WAIT",
         "PROC" };

   os << state_name[(int)sig.state];

   return os;
}

} // namespace simd

namespace sc_core {

void sc_trace(
      sc_core::sc_trace_file* tf,
      const simd::simd_sig_dmeu_state_c& sig,
      const std::string& name ) {

   // for property tree we trace only hash value to observe the changes
   sc_core::sc_trace(
         tf,
         sig.tr_state,
         name, 2 );
}

} // namespace sc_core
