/*
 * simd_ptree_xbar.h
 *
 *  Description:
 *    Declaration of the system component: ptree cross bar
 */

#ifndef SIMD_CORE_INCLUDE_SIMD_PTREE_XBAR_H_
#define SIMD_CORE_INCLUDE_SIMD_PTREE_XBAR_H_

#include <vector>
#include <string>
#include <boost/property_tree/ptree.hpp>
#include <boost/optional.hpp>
#include <boost/regex.hpp>
#include <systemc>
#include "simd_sig_ptree.h"

// Short alias for the namespace
namespace boost_pt = boost::property_tree;

namespace simd {

   SC_MODULE( simd_ptree_xbar_c ) { // declare module class

   public:
      // Module I/O Ports
      sc_core::sc_vector<sc_core::sc_port<sc_core::sc_fifo_in_if <simd_sig_ptree_c>>> vi;
      sc_core::sc_vector<sc_core::sc_port<sc_core::sc_fifo_out_if<simd_sig_ptree_c>>> vo;

      // Constructor declaration
      SC_CTOR( simd_ptree_xbar_c );

      // Init declaration (to be called after the instantiation and before the port binding)
      void init(
            boost::optional<const boost_pt::ptree&> _pref_p );

   private:
      // Process declarations
      void exec_thrd(
            void );

      class port_map_t {
      public:
         bool         regex = false;
         std::string  name;
         boost::regex mask;
      };

      std::vector<port_map_t> vi_map;
      std::vector<port_map_t> vo_map;

   }; // SC_MODULE( simd_ptree_xbar_c )
} // namespace simd

#endif /* SIMD_CORE_INCLUDE_SIMD_PTREE_XBAR_H_ */
