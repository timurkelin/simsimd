/*
 * simd_sig_dmeu_valid.h
 *
 *  Description:
 *    Declaration of the signal which carries "source data valid" flag
 */

#ifndef SIMD_SYS_CORE_INCLUDE_SIMD_SIG_DMEU_VALID_H_
#define SIMD_SYS_CORE_INCLUDE_SIMD_SIG_DMEU_VALID_H_

#include <string>
#include <iostream>
#include <systemc>

namespace simd {
   class simd_sig_dmeu_valid_c;  // Forward declaration
} // namespace simd

namespace sc_core {

   void sc_trace(
         sc_core::sc_trace_file* tf,
         const simd::simd_sig_dmeu_valid_c& sig,
         const std::string& name );

} // namespace sc_core

namespace simd {

   typedef enum {
      SIMD_IDLE = 0,
      SIMD_BODY = 1,
      SIMD_HEAD = 2,
      SIMD_TAIL = 3
   } simd_dmeu_valid_t;

   std::ostream& operator << (
         std::ostream& os,
         const simd::simd_sig_dmeu_valid_c& sig );

   class simd_sig_dmeu_valid_c {
   private:
      simd_dmeu_valid_t valid    = SIMD_IDLE;
      unsigned int      tr_valid = SIMD_IDLE;

   public:
      // Required for the assignment operations
      simd_sig_dmeu_valid_c& set(
            const simd_dmeu_valid_t& rhs );

      const simd_dmeu_valid_t& get(
            void ) const;

      // Required by sc_signal<> and sc_fifo<>
      simd_sig_dmeu_valid_c& operator = (
            const simd_sig_dmeu_valid_c& rhs );

      // Required by sc_signal<>
      bool operator == (
            const simd_sig_dmeu_valid_c& rhs) const;

      friend void sc_core::sc_trace(
            sc_core::sc_trace_file* tf,
            const simd::simd_sig_dmeu_valid_c& sig,
            const std::string& name );

      friend std::ostream& operator << (
            std::ostream& os,
            const simd::simd_sig_dmeu_valid_c& sig );

   }; // class simd_sig_dmeu_valid_c

} // namespace simd

#endif /* SIMD_SYS_CORE_INCLUDE_SIMD_SIG_DMEU_VALID_H_ */
