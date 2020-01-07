/*
 * simd_sig_dm_addr.h
 *
 *  Description:
 *    Declaration of the DM address type and corresponding signal
 *
 */

#ifndef SIMD_SYS_DM_INCLUDE_SIMD_SIG_DM_ADDR_H_
#define SIMD_SYS_DM_INCLUDE_SIMD_SIG_DM_ADDR_H_

#include <string>
#include <iostream>
#include <complex>
#include <systemc>
#include "simd_sig_dmeu_data.h"
#include "simd_report.h"

namespace simd {
   class simd_sig_dm_addr_c;  // Forward declaration
} // namespace simd

namespace sc_core {

   void sc_trace(
         sc_core::sc_trace_file* tf,
         const simd::simd_sig_dm_addr_c& sig,
         const std::string& name );

   } // namespace sc_core

namespace simd {

   typedef struct {
              bool ena;     // Operation enabled
       std::size_t addr;    // Address
       std::size_t perm;    // Permutation
   } simd_way_addr_t;

   class simd_dm_addr_c {
   public:
      const static std::size_t dim = simd_dmeu_data_c::dim;

      simd_way_addr_t& operator [](const std::size_t idx) {
         if(( idx < 0 ) || ( idx >= dim )) {
            SIMD_REPORT_ERROR( "simd::sys_dag" ) << "Index is out of bounds";
         }

         return slot[idx];
      }

      simd_way_addr_t  operator [](const std::size_t idx) const {
         if(( idx < 0 ) || ( idx >= dim )) {
            SIMD_REPORT_ERROR( "simd::sys_dag" ) << "Index is out of bounds";
         }

         return slot[idx];
      }


      simd_dm_addr_c& operator = (
            const simd_dm_addr_c& rhs );

      bool operator == (
            const simd_dm_addr_c& rhs) const;

   private:
      simd_way_addr_t slot[dim];

   }; // class simd_dm_addr_c

   std::ostream& operator << (
         std::ostream& os,
         const simd_sig_dm_addr_c& sig );

   class simd_sig_dm_addr_c {
   private:
      simd_dm_addr_c addr;
      bool   tr_addr0_ena,  tr_addr1_ena,  tr_addr2_ena,  tr_addr3_ena;
      double tr_addr0_addr, tr_addr1_addr, tr_addr2_addr, tr_addr3_addr;
      double tr_addr0_perm, tr_addr1_perm, tr_addr2_perm, tr_addr3_perm;

   public:
      // Required for the assignment operations
      simd_sig_dm_addr_c& set(
            const simd_dm_addr_c& rhs );

      const simd_dm_addr_c& get(
            void ) const;

      // Required by sc_signal<> and sc_fifo<>
      simd_sig_dm_addr_c& operator = (
            const simd_sig_dm_addr_c& rhs );

      // Required by sc_signal<>
      bool operator == (
            const simd_sig_dm_addr_c& rhs) const;

      friend void sc_core::sc_trace(
            sc_core::sc_trace_file* tf,
            const simd::simd_sig_dm_addr_c& sig,
            const std::string& name );

      friend std::ostream& operator << (
            std::ostream& os,
            const simd_sig_dm_addr_c& sig );

   }; // class simd_sig_dm_addr_c
} // namespace simd

#endif /* SIMD_SYS_DM_INCLUDE_SIMD_SIG_DM_ADDR_H_ */
