/*
 * simd_sys_dmeu.h
 *
 *  Description:
 *    Declaration of the basic class for the DM and EU modules
 */

#ifndef SIMD_SYS_CORE_INCLUDE_SIMD_SYS_DMEU_H_
#define SIMD_SYS_CORE_INCLUDE_SIMD_SYS_DMEU_H_

#include <string>
#include <vector>
#include <iostream>
#include <boost/optional.hpp>
#include <boost/property_tree/ptree.hpp>
#include <systemc>
#include "simd_chn_dmeu_data.h"
#include "simd_sig_dmeu_state.h"
#include "simd_sig_ptree.h"
#include "simd_dump.h"

// Short alias for the namespace
namespace boost_pt = boost::property_tree;

namespace simd {

   SC_MODULE( simd_sys_dmeu_c ) { // declare module class

   public:
      // Module I/O Ports
      sc_core::sc_in<bool>    clock_i;      // Clock
      sc_core::sc_in<bool>    reset_i;      // Reset
      sc_core::sc_vector< sc_core::sc_port<simd_chn_dmeu_data_i_if> > data_vi;   // Vector of  input data ports
      sc_core::sc_vector< sc_core::sc_port<simd_chn_dmeu_data_o_if> > data_vo;   // Vector of output data ports
      sc_core::sc_port<sc_core::sc_fifo_out_if<simd_sig_ptree_c>>     event_o;   // Interrupt request output
      sc_core::sc_port<sc_core::sc_fifo_in_if< simd_sig_ptree_c>>     busw_i;    // Configuration input
      sc_core::sc_port<sc_core::sc_fifo_out_if<simd_sig_ptree_c>>     busr_o;    // Status output
      sc_core::sc_in<bool>                                            proc_i;    // Processing enable input from XBAR
      sc_core::sc_out<simd_sig_dmeu_state_c>                          state_o;   // DM/EU processing state

      // Constructor declaration
      SC_CTOR( simd_sys_dmeu_c );

      // Init declaration (to be used after the instantiation and before the port binding)
      virtual void init(
            boost::optional<const boost_pt::ptree&> _pref_p ) = 0;

      virtual void add_trace(
            sc_core::sc_trace_file* tf,
            const std::string& top_name ) = 0;

   protected:
      // Process declarations
      virtual void proc_thrd(      // Processing and configuration thread
            void ) = 0;

      virtual void ready_thrd(     // Ready interconnect thread
            void ) = 0;

      // Definition of the channels for communication between threads
      sc_core::sc_vector< sc_core::sc_signal<simd_sig_dmeu_ready_c>> ready_v; // Set of internal ready signals (typically 1)

      // Other declarations
      std::size_t active_config;
      std::vector<boost_pt::ptree> config;   // The size of the config vector is set at the init stage
      boost::optional<sc_core::sc_fifo<std::size_t>&> active_config_p = boost::none;

      std::size_t active_status;
      std::vector<boost_pt::ptree> status;   // The size of the status vector is set at the init stage

      // Pointer to the preferences. To be used by the callbacks
      // before_end_of_elaboration, end_of_elaboration, start_of_simulation, end_of_simulation
      boost::optional<const boost_pt::ptree&> pref_p;

      boost::optional<simd_dump_buf_c<std::string    >&> dump_buf_event_p;       // Event  dump buffer
      boost::optional<simd_dump_buf_c<boost_pt::ptree>&> dump_buf_config_p;      // Config dump buffer
      boost::optional<simd_dump_buf_c<boost_pt::ptree>&> dump_buf_status_p;      // Status dump buffer

      // Parse request from the busw
      void parse_busw_req(
            const simd_sig_ptree_c& req );

      // Execution request
      virtual void req_run(
            void ) = 0;

      // Check if IRQ is generated
      bool is_event_enabled(
            std::size_t  conf_idx,
            const std::string& event_id );

      // Generate Event to notify the scalar core
      bool event(
            const std::string& event_id );

      bool event(
            const boost_pt::ptree& event_pt );

      bool is_conf_empty(
            std::size_t conf_idx );

      std::size_t next_conf_idx(
            std::size_t conf_idx );

      bool next_conf_run(
            std::size_t conf_idx );

      std::size_t curr_stat_idx(
            std::size_t conf_idx );

      void add_trace_ports(
            sc_core::sc_trace_file* tf,
            const std::string& top_name );

   }; // SC_MODULE( simd_sys_dmeu_c )

   class simd_sys_dmeu_info_t {
   public:
      std::size_t  idx;     // Block index
      std::string  name;    // Name of the block
      std::size_t  hash;    // name hash
      std::string  func;    // function name
      boost::optional<simd_sys_dmeu_c&> mod_p;        // Pointer to module
      boost::optional<const boost_pt::ptree&> pref_p; // Pointer to preferences
      std::vector<std::size_t> xbar_dmeu;        // data_i properties
      std::vector<std::size_t> dmeu_xbar;        // data_o properties
   };

} // namespace simd

#endif /* SIMD_SYS_CORE_INCLUDE_SIMD_SYS_DMEU_H_ */
