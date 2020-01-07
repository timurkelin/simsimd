/*
 * simd_sig_dmeu_data.h
 *
 *  Description:
 *    Declaration of the DM/EU data type and signal
 *
 */

#ifndef SIMD_SYS_CORE_INCLUDE_SIMD_SIG_DMEU_DATA_H_
#define SIMD_SYS_CORE_INCLUDE_SIMD_SIG_DMEU_DATA_H_

#include <string>
#include <iostream>
#include <complex>
#include <systemc>
#include "simd_report.h"

namespace simd {
   class simd_sig_dmeu_data_c;  // Forward declaration
} // namespace simd

namespace sc_core {

void sc_trace(
      sc_core::sc_trace_file* tf,
      const simd::simd_sig_dmeu_data_c& sig,
      const std::string& name );

} // namespace sc_core

namespace simd {

   typedef std::complex<double> simd_dmeu_smp_t;

   class simd_dmeu_slot_t {
   public:
      bool            ena = false;
      simd_dmeu_smp_t smp = simd_dmeu_smp_t( 0.0, 0.0 );
   };

   class simd_dmeu_data_c {
   public:
      const static std::size_t dim = 4;

      simd_dmeu_data_c() {
         for( std::size_t idx = 0; idx < dim; idx ++ ) {
            slot[idx].smp = simd_dmeu_smp_t( 0.0, 0.0 );
            slot[idx].ena = false;
         }
      }

      simd_dmeu_slot_t& operator [](const std::size_t idx) {
         if(( idx < 0 ) || ( idx >= dim )) {
            SIMD_REPORT_ERROR( "simd::chn_dmeu_data" ) << "Index is out of bounds";
         }

         return slot[idx];
      }

      simd_dmeu_slot_t  operator [](const std::size_t idx) const {
         if(( idx < 0 ) || ( idx >= dim )) {
            SIMD_REPORT_ERROR( "simd::chn_dmeu_data" ) << "Index is out of bounds";
         }

         return slot[idx];
      }

      simd_dmeu_data_c& operator = (
            const simd_dmeu_data_c& rhs );

      bool operator == (
            const simd_dmeu_data_c& rhs) const;

   private:
      simd_dmeu_slot_t slot[dim];

   };

   std::ostream& operator << (
         std::ostream& os,
         const simd_sig_dmeu_data_c& sig );

   class simd_sig_dmeu_data_c {
   private:
      simd_dmeu_data_c data;
      bool   tr_data0_en, tr_data1_en, tr_data2_en, tr_data3_en;
      double tr_data0_re, tr_data1_re, tr_data2_re, tr_data3_re;
      double tr_data0_im, tr_data1_im, tr_data2_im, tr_data3_im;

   public:
      // Required for the assignment operations
      simd_sig_dmeu_data_c& set(
            const simd_dmeu_data_c& rhs );

      const simd_dmeu_data_c& get(
            void ) const;

      // Required by sc_signal<> and sc_fifo<>
      simd_sig_dmeu_data_c& operator = (
            const simd_sig_dmeu_data_c& rhs );

      // Required by sc_signal<>
      bool operator == (
            const simd_sig_dmeu_data_c& rhs) const;

      friend void sc_core::sc_trace(
            sc_core::sc_trace_file* tf,
            const simd::simd_sig_dmeu_data_c& sig,
            const std::string& name );

      friend std::ostream& operator << (
            std::ostream& os,
            const simd_sig_dmeu_data_c& sig );

   }; // class simd_sig_dmeu_data_c
} // namespace simd

#endif /* SIMD_SYS_CORE_INCLUDE_SIMD_SIG_DMEU_DATA_H_ */
