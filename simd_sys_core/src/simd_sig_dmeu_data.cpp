/*
 * simd_sig_dmeu_data.cpp
 *
 *  Description:
 *    Methods of the DM/EU data type and signal
 *
 */

#include "simd_sig_dmeu_data.h"
#include "simd_report.h"

namespace simd {

simd_dmeu_data_c& simd_dmeu_data_c::operator = (
      const simd_dmeu_data_c& rhs ) {

   for( std::size_t sl = 0; sl < dim; sl ++ ) {
      slot[sl] = rhs[sl];
   }

   return *this;
}

bool simd_dmeu_data_c::operator == (
      const simd_dmeu_data_c& rhs) const {
   bool cmp = true;

   for( std::size_t sl = 0; sl < dim; sl ++ ) {
      if(( slot[sl].ena != rhs[sl].ena ) ||
         ( slot[sl].smp != rhs[sl].smp )) {
         cmp = false;
         break;
      }
   }

   return cmp;
}

// Signal assignment
simd_sig_dmeu_data_c& simd_sig_dmeu_data_c::set(
      const simd_dmeu_data_c& rhs ) {
   data = rhs;

   tr_data0_en = rhs[0].ena;
   tr_data0_re = rhs[0].smp.real();
   tr_data0_im = rhs[0].smp.imag();

   tr_data1_en = rhs[1].ena;
   tr_data1_re = rhs[1].smp.real();
   tr_data1_im = rhs[1].smp.imag();

   tr_data2_en = rhs[2].ena;
   tr_data2_re = rhs[2].smp.real();
   tr_data2_im = rhs[2].smp.imag();

   tr_data3_en = rhs[3].ena;
   tr_data3_re = rhs[3].smp.real();
   tr_data3_im = rhs[3].smp.imag();
   return *this;
}

const simd_dmeu_data_c& simd_sig_dmeu_data_c::get(
      void ) const {
   return data;
}

// Required by sc_signal<> and sc_fifo<>
simd_sig_dmeu_data_c& simd_sig_dmeu_data_c::operator = (
      const simd_sig_dmeu_data_c& rhs ) {
   data = rhs.data;

   tr_data0_en = rhs.data[0].ena;
   tr_data0_re = rhs.data[0].smp.real();
   tr_data0_im = rhs.data[0].smp.imag();

   tr_data1_en = rhs.data[1].ena;
   tr_data1_re = rhs.data[1].smp.real();
   tr_data1_im = rhs.data[1].smp.imag();

   tr_data2_en = rhs.data[2].ena;
   tr_data2_re = rhs.data[2].smp.real();
   tr_data2_im = rhs.data[2].smp.imag();

   tr_data3_en = rhs.data[3].ena;
   tr_data3_re = rhs.data[3].smp.real();
   tr_data3_im = rhs.data[3].smp.imag();

   return *this;
}

// Required by sc_signal<>
bool simd_sig_dmeu_data_c::operator == (
      const simd_sig_dmeu_data_c& rhs) const {
   return ( data == rhs.data );
}

std::ostream& operator << (
      std::ostream& os,
      const simd::simd_sig_dmeu_data_c& sig ) {

   for( std::size_t sl = 0; sl < sig.data.dim; sl ++ ) {
      os << sig.data[sl].smp
         << ( sig.data[sl].ena ? " ena" : " dis" );

      if( sl != sig.data.dim - 1 ) {
         os << std::endl;
      }
   }

   return os;
}
} // namespace simd

namespace sc_core {

void sc_trace(
      sc_core::sc_trace_file* tf,
      const simd::simd_sig_dmeu_data_c& sig,
      const std::string& name ) {

   sc_core::sc_trace( tf, sig.tr_data0_re, name + "(0).re" );
   sc_core::sc_trace( tf, sig.tr_data0_im, name + "(0).im" );
   sc_core::sc_trace( tf, sig.tr_data0_en, name + "(0).en" );

   sc_core::sc_trace( tf, sig.tr_data1_re, name + "(1).re" );
   sc_core::sc_trace( tf, sig.tr_data1_im, name + "(1).im" );
   sc_core::sc_trace( tf, sig.tr_data1_en, name + "(1).en" );

   sc_core::sc_trace( tf, sig.tr_data2_re, name + "(2).re" );
   sc_core::sc_trace( tf, sig.tr_data2_im, name + "(2).im" );
   sc_core::sc_trace( tf, sig.tr_data2_en, name + "(2).en" );

   sc_core::sc_trace( tf, sig.tr_data3_re, name + "(3).re" );
   sc_core::sc_trace( tf, sig.tr_data3_im, name + "(3).im" );
   sc_core::sc_trace( tf, sig.tr_data3_en, name + "(3).en" );

}
} // namespace sc_core
