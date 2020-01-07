/*
 * simd_sys_bmux.h
 *
 *  Description:
 *    Declaration of the system component: Bus multiplexers for configuration and status
 */

#ifndef SIMD_SYS_BMUX_INCLUDE_SIMD_SYS_BMUX_H_
#define SIMD_SYS_BMUX_INCLUDE_SIMD_SYS_BMUX_H_

#include <boost/property_tree/ptree.hpp>
#include <boost/optional.hpp>
#include <systemc>
#include "simd_sig_ptree.h"
#include "simd_sys_dmeu.h"

// Short alias for the namespace
namespace boost_pt = boost::property_tree;

namespace simd {

   SC_MODULE( simd_sys_bmuxw_c ) { // declare module class

   public:
      // Module I/O Ports
      sc_core::sc_in<bool> clock_i;
      sc_core::sc_in<bool> reset_i;
      sc_core::sc_port<sc_core::sc_fifo_in_if<simd_sig_ptree_c>>                      busw_i;  // Common configuration input
      sc_core::sc_vector<sc_core::sc_port<sc_core::sc_fifo_out_if<simd_sig_ptree_c>>> busw_vo; // configuration output for each block inside core

      // Constructor declaration
      SC_CTOR( simd_sys_bmuxw_c );

      // Init declaration (to be used after the instantiation and before the port binding)
      void init(
            const std::vector<simd_sys_dmeu_info_t>& dmeu_info );

   private:
      // Process declarations
      void exec_thrd(
            void );

      // Pointer to the vector which contains information about dm/eu instances
      boost::optional<const std::vector<simd_sys_dmeu_info_t> &> dmeu_info_p = boost::none;

   }; // SC_MODULE( simd_sys_bmuxw_c )

   SC_MODULE( simd_sys_bmuxr_c ) { // declare module class

   public:
      // Module I/O Ports
      sc_core::sc_in<bool> clock_i;
      sc_core::sc_in<bool> reset_i;
      sc_core::sc_port<sc_core::sc_fifo_out_if<simd_sig_ptree_c>>                    busr_o;  // Common status input
      sc_core::sc_vector<sc_core::sc_port<sc_core::sc_fifo_in_if<simd_sig_ptree_c>>> busr_vi; // Status input for each block inside core

      // Constructor declaration
      SC_CTOR( simd_sys_bmuxr_c );

      // Init declaration (to be used after the instantiation and before the port binding)
      void init(
            const std::vector<simd_sys_dmeu_info_t>& dmeu_info );

   private:
      // Process declarations
      void exec_thrd(
            void );

      // Pointer to the vector which contains information about dm/eu instances
      boost::optional<const std::vector<simd_sys_dmeu_info_t>&> dmeu_info_p = boost::none;

   }; // SC_MODULE( simd_sys_bmuxr_c )

   SC_MODULE( simd_sys_event_c ) {
   public:
      // Module I/O Ports
      sc_core::sc_in<bool> clock_i;      // Reserve clock input though icu works asynchronously
      sc_core::sc_in<bool> reset_i;      // Reserve reset input though icu works asynchronously
      sc_core::sc_vector<sc_core::sc_port<sc_core::sc_fifo_in_if<simd_sig_ptree_c>>> event_vi;  // Interrupt request inputs
      sc_core::sc_port<sc_core::sc_fifo_out_if<simd_sig_ptree_c>>                    event_o;   // Interrupt request output

      // Constructor declaration
      SC_CTOR( simd_sys_event_c );

      // Init declaration (to be used after the instantiation and before the port binding)
      void init(
            const std::vector<simd_sys_dmeu_info_t>& dmeu_info );

   private:
      // Process declarations
      void exec_thrd(
            void );

      // Pointer to the vector which contains information about dm/eu instances
      boost::optional<const std::vector<simd_sys_dmeu_info_t>&> dmeu_info_p = boost::none;

   }; // simd_sys_event_c

} // namespace simd

#endif /* SIMD_SYS_BMUX_INCLUDE_SIMD_SYS_BMUX_H_ */
