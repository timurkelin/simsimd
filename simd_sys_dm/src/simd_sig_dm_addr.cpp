/*
 * simd_sig_dm_addr.cpp
 *
 *  Description:
 *    Methods of the DM address type and signal
 *
 */

#include "simd_sig_dm_addr.h"
#include "simd_report.h"

namespace simd {

simd_dm_addr_c& simd_dm_addr_c::operator = (
      const simd_dm_addr_c& rhs ) {

   for( std::size_t sl = 0; sl < dim; sl ++ ) {
      slot[sl] = rhs[sl];
   }

   return *this;
}

bool simd_dm_addr_c::operator == (
      const simd_dm_addr_c& rhs) const {
   bool cmp = true;

   for( std::size_t sl = 0; sl < dim; sl ++ ) {
      if(( slot[sl].ena  != rhs[sl].ena  ) ||
         ( slot[sl].addr != rhs[sl].addr ) ||
         ( slot[sl].perm != rhs[sl].perm )) {
         cmp = false;
         break;
      }
   }

   return cmp;
}

// Signal assignment
simd_sig_dm_addr_c& simd_sig_dm_addr_c::set(
      const simd_dm_addr_c& rhs ) {
   addr = rhs;

   tr_addr0_ena  = rhs[0].ena;
   tr_addr0_perm = rhs[0].perm;
   tr_addr0_addr = rhs[0].addr;

   tr_addr1_ena  = rhs[1].ena;
   tr_addr1_perm = rhs[1].perm;
   tr_addr1_addr = rhs[1].addr;

   tr_addr2_ena  = rhs[2].ena;
   tr_addr2_perm = rhs[2].perm;
   tr_addr2_addr = rhs[2].addr;

   tr_addr3_ena  = rhs[3].ena;
   tr_addr3_perm = rhs[3].perm;
   tr_addr3_addr = rhs[3].addr;

   return *this;
}

const simd_dm_addr_c& simd_sig_dm_addr_c::get(
      void ) const {
   return addr;
}

// Required by sc_signal<> and sc_fifo<>
simd_sig_dm_addr_c& simd_sig_dm_addr_c::operator = (
      const simd_sig_dm_addr_c& rhs ) {
   addr = rhs.addr;

   tr_addr0_ena  = rhs.addr[0].ena;
   tr_addr0_perm = rhs.addr[0].perm;
   tr_addr0_addr = rhs.addr[0].addr;

   tr_addr1_ena  = rhs.addr[1].ena;
   tr_addr1_perm = rhs.addr[1].perm;
   tr_addr1_addr = rhs.addr[1].addr;

   tr_addr2_ena  = rhs.addr[2].ena;
   tr_addr2_perm = rhs.addr[2].perm;
   tr_addr2_addr = rhs.addr[2].addr;

   tr_addr3_ena  = rhs.addr[3].ena;
   tr_addr3_perm = rhs.addr[3].perm;
   tr_addr3_addr = rhs.addr[3].addr;

   return *this;
}

// Required by sc_signal<>
bool simd_sig_dm_addr_c::operator == (
      const simd_sig_dm_addr_c& rhs) const {
   return ( addr == rhs.addr );
}

std::ostream& operator << (
      std::ostream& os,
      const simd::simd_sig_dm_addr_c& sig ) {

   for( std::size_t sl = 0; sl < sig.addr.dim; sl ++ ) {
      os << sig.addr[sl].perm
         << sig.addr[sl].addr
         << ( sig.addr[sl].ena ? " ena" : " dis" );

      if( sl != sig.addr.dim - 1 ) {
         os << std::endl;
      }
   }

   return os;
}
} // namespace simd

namespace sc_core {

void sc_trace(
      sc_core::sc_trace_file* tf,
      const simd::simd_sig_dm_addr_c& sig,
      const std::string& name ) {

   sc_core::sc_trace( tf, sig.tr_addr0_ena , name + "(0).ena"  );
   sc_core::sc_trace( tf, sig.tr_addr0_perm, name + "(0).perm" );
   sc_core::sc_trace( tf, sig.tr_addr0_addr, name + "(0).addr" );

   sc_core::sc_trace( tf, sig.tr_addr1_ena , name + "(1).ena"  );
   sc_core::sc_trace( tf, sig.tr_addr1_perm, name + "(1).perm" );
   sc_core::sc_trace( tf, sig.tr_addr1_addr, name + "(1).addr" );

   sc_core::sc_trace( tf, sig.tr_addr2_ena , name + "(2).ena"  );
   sc_core::sc_trace( tf, sig.tr_addr2_perm, name + "(2).perm" );
   sc_core::sc_trace( tf, sig.tr_addr2_addr, name + "(2).addr" );

   sc_core::sc_trace( tf, sig.tr_addr3_ena , name + "(3).ena"  );
   sc_core::sc_trace( tf, sig.tr_addr3_perm, name + "(3).perm" );
   sc_core::sc_trace( tf, sig.tr_addr3_addr, name + "(3).addr" );

}
} // namespace sc_core
