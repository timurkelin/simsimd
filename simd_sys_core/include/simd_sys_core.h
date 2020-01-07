/*
 * simd_sys_core.h
 *
 *  Description:
 *    Declaration of the SIMD core module
 */

#ifndef SIMD_SYS_CORE_INCLUDE_SIMD_SYS_CORE_H_
#define SIMD_SYS_CORE_INCLUDE_SIMD_SYS_CORE_H_

#include <string>
#include <vector>
#include <boost/optional.hpp>
#include "simd_sys_dmeu.h"
#include "simd_sys_bmux.h"
#include "simd_sys_xbar.h"

// Short alias for the namespace
namespace boost_pt = boost::property_tree;

namespace simd {

   SC_MODULE( simd_sys_core_c ) { // declare module class

   public:
      // Module I/O Ports
      sc_core::sc_in<bool>                                          clock_i;    // Clock
      sc_core::sc_in<bool>                                          reset_i;    // Reset
      sc_core::sc_export<sc_core::sc_fifo_in_if< simd_sig_ptree_c>> event_ei;   // Events fifo export through the input interface
      sc_core::sc_export<sc_core::sc_fifo_out_if<simd_sig_ptree_c>> busw_eo;    // Configuration input
      sc_core::sc_export<sc_core::sc_fifo_in_if< simd_sig_ptree_c>> busr_ei;    // Status output

      // Constructor declaration
      SC_CTOR( simd_sys_core_c );

      // Init declaration (to be used after the instantiation and before the port binding)
      void init(
            boost::optional<const boost_pt::ptree&> _pref_p );

   private:
      boost::optional<simd_sys_dmeu_c &> simd_sys_dmeu_new(
            const std::string& func,
            const sc_core::sc_module_name& name );

      std::vector<simd_sys_dmeu_info_t> dmeu_info;

      // Pointers to the submodules
      boost::optional<simd::simd_sys_xbar_c  &> xbar_p  = boost::none;
      boost::optional<simd::simd_sys_event_c &> event_p = boost::none;
      boost::optional<simd::simd_sys_bmuxw_c &> bmuxw_p = boost::none;
      boost::optional<simd::simd_sys_bmuxr_c &> bmuxr_p = boost::none;

      // dmeu state and enable channels
      sc_core::sc_vector<sc_core::sc_signal<simd_sig_dmeu_state_c>> state_v;
      sc_core::sc_vector<sc_core::sc_signal<bool>>                  proc_v;

      // IRQ, Configuration and Status channels
      sc_core::sc_vector<sc_core::sc_fifo<simd_sig_ptree_c>> event_chn_v;
      sc_core::sc_fifo<simd_sig_ptree_c>                     event_chn_o;

      sc_core::sc_vector<sc_core::sc_fifo<simd_sig_ptree_c>> busw_chn_v;
      sc_core::sc_fifo<simd_sig_ptree_c>                     busw_chn_i;

      sc_core::sc_vector<sc_core::sc_fifo<simd_sig_ptree_c>> busr_chn_v;
      sc_core::sc_fifo<simd_sig_ptree_c>                     busr_chn_o;

      boost::optional<const boost_pt::ptree&> pref_p;

   }; // SC_MODULE( simd_sys_core_c )
} // namespace simd


#endif /* SIMD_SYS_CORE_INCLUDE_SIMD_SYS_CORE_H_ */
