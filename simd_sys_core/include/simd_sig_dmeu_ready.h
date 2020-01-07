/*
 * simd_sig_dmeu_ready.h
 *
 *  Description:
 *    Declaration of the signal which carries "destination ready" flag
 */

#ifndef SIMD_SYS_CORE_INCLUDE_SIMD_SIG_DMEU_READY_H_
#define SIMD_SYS_CORE_INCLUDE_SIMD_SIG_DMEU_READY_H_

#include <string>
#include <iostream>
#include <systemc>

namespace simd {
   class simd_sig_dmeu_ready_c;  // Forward declaration
} // namespace simd

namespace sc_core {

   void sc_trace(
         sc_core::sc_trace_file* tf,
         const simd::simd_sig_dmeu_ready_c& sig,
         const std::string& name );
} // namespace sc_core

namespace simd {

   typedef bool simd_dmeu_ready_t;

   std::ostream& operator << (
         std::ostream& os,
         const simd_sig_dmeu_ready_c& sig );

   class simd_sig_dmeu_ready_c {
   private:
      simd_dmeu_ready_t ready    = false;
      bool              tr_ready = false;

   public:
      // Required for the assignment operations
      simd_sig_dmeu_ready_c& set(
            const simd_dmeu_ready_t& rhs );

      const simd_dmeu_ready_t& get(
            void ) const;

      // Required by sc_signal<> and sc_fifo<>
      simd_sig_dmeu_ready_c& operator = (
            const simd_sig_dmeu_ready_c& rhs );

      // Required by sc_signal<>
      bool operator == (
            const simd_sig_dmeu_ready_c& rhs) const;

      friend void sc_core::sc_trace(
            sc_core::sc_trace_file* tf,
            const simd::simd_sig_dmeu_ready_c& sig,
            const std::string& name );

      friend std::ostream& operator << (
            std::ostream& os,
            const simd_sig_dmeu_ready_c& sig );

   }; // class simd_sig_dmeu_ready_c
} // namespace simd

#endif /* SIMD_SYS_CORE_INCLUDE_SIMD_SIG_DMEU_READY_H_ */
