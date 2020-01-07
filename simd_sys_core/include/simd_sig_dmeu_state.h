/*
 * simd_sig_dmeu_state.h
 *
 *  Description:
 *    Declaration of the signal which carries DM/EU state
 */

#ifndef SIMD_SYS_CORE_INCLUDE_SIMD_SIG_DMEU_STATE_H_
#define SIMD_SYS_CORE_INCLUDE_SIMD_SIG_DMEU_STATE_H_

#include <string>
#include <iostream>
#include <systemc>

namespace simd {
   class simd_sig_dmeu_state_c;  // Forward declaration
} // namespace simd

namespace sc_core {

   void sc_trace(
         sc_core::sc_trace_file* tf,
         const simd::simd_sig_dmeu_state_c& sig,
         const std::string& name );

} // namespace sc_core

namespace simd {

   typedef enum {
      DMEU_ST_IDLE  = 0,   // DM/EU is waiting to be configured or for the first sample
      DMEU_ST_PROC  = 1,   // DM/EU is processing samples
      DMEU_ST_DONE  = 2    // DM/EU is done processing a vector and can try fetching new configuration slot
   } simd_dmeu_state_t;

   std::ostream& operator << (
         std::ostream& os,
         const simd::simd_sig_dmeu_state_c& sig );

   class simd_sig_dmeu_state_c {
   private:
      simd_dmeu_state_t state    = DMEU_ST_IDLE;
      unsigned int      tr_state = DMEU_ST_IDLE;

   public:
      // Required for the assignment operations
      simd_sig_dmeu_state_c& set(
            const simd_dmeu_state_t& rhs );

      const simd_dmeu_state_t& get(
            void ) const;

      // Required by sc_signal<> and sc_fifo<>
      simd_sig_dmeu_state_c& operator = (
            const simd_sig_dmeu_state_c& rhs );

      // Required by sc_signal<>
      bool operator == (
            const simd_sig_dmeu_state_c& rhs) const;

      friend void sc_core::sc_trace(
            sc_core::sc_trace_file* tf,
            const simd::simd_sig_dmeu_state_c& sig,
            const std::string& name );

      friend std::ostream& operator << (
            std::ostream& os,
            const simd::simd_sig_dmeu_state_c& sig );

   }; // class simd_sig_dmeu_state_c

} // namespace simd

#endif /* SIMD_SYS_CORE_INCLUDE_SIMD_SIG_DMEU_STATE_H_ */
