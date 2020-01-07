/*
 * simd_sig_ptree.h
 *
 *  Description:
 *    Declaration of the signal which carries a generic property tree
 *    This signal is used to transfer configuration and status data
 */

#ifndef SIMD_SYS_CORE_INCLUDE_SIMD_SIG_PTREE_H_
#define SIMD_SYS_CORE_INCLUDE_SIMD_SIG_PTREE_H_

#include <string>
#include <iostream>
#include <boost/property_tree/ptree.hpp>
#include <systemc>

// Short alias for the namespace
namespace boost_pt = boost::property_tree;

namespace simd {
   class simd_sig_ptree_c;  // Forward declaration
} // namespace simd

namespace sc_core {

   void sc_trace(
         sc_core::sc_trace_file* tf,
         const simd::simd_sig_ptree_c& sig,
         const std::string& name );

} // namespace sc_core

namespace simd {
   std::ostream& operator << (
         std::ostream& os,
         const simd_sig_ptree_c& sig );

   class simd_sig_ptree_c {
   private:
      boost_pt::ptree data;
      std::size_t tr_hash = 0;

   public:
      // Required for the assignment operations
      simd_sig_ptree_c& set(
            const boost_pt::ptree& rhs );

      const boost_pt::ptree& get(
            void ) const;

      // Required by sc_signal<> and sc_fifo<>
      simd_sig_ptree_c& operator = (
            const simd_sig_ptree_c& rhs );

      // Required by sc_signal<>
      bool operator == (
            const simd_sig_ptree_c& rhs) const;

      friend void sc_core::sc_trace(
            sc_core::sc_trace_file* tf,
            const simd_sig_ptree_c& sig,
            const std::string& name );

      friend std::ostream& operator << (
            std::ostream& os,
            const simd_sig_ptree_c& sig );
   }; // class simd_sig_ptree_c
} // namespace simd

#endif /* SIMD_SYS_CORE_INCLUDE_SIMD_SIG_PTREE_H_ */
