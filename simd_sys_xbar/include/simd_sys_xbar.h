/*
 * simd_sys_xbar.h
 *
 *  Description:
 *    Declaration of the cross-bar switch.
 */

#ifndef SIMD_SYS_XBAR_INCLUDE_SIMD_SYS_XBAR_H_
#define SIMD_SYS_XBAR_INCLUDE_SIMD_SYS_XBAR_H_

#include <string>
#include <climits>
#include <boost/optional.hpp>
#include <boost/property_tree/ptree.hpp>
#include <systemc>
#include "simd_sys_eb.h"
#include "simd_sys_dmeu.h"
#include "simd_chn_dmeu_data.h"
#include "simd_sig_ptree.h"

// Short alias for the namespace
namespace boost_pt = boost::property_tree;

namespace simd {

SC_MODULE( simd_sys_xbar_c ) {
public:
   // Module I/O Ports
   sc_core::sc_in<bool> clock_i;      // Reserve clock input though xbar works asynchronously
   sc_core::sc_in<bool> reset_i;      // Reserve reset input though xbar works asynchronously
   sc_core::sc_vector< sc_core::sc_export<simd_chn_dmeu_data_i_if > > data_xvi; // Vector of exports of data towards dmeu
   sc_core::sc_vector< sc_core::sc_export<simd_chn_dmeu_data_o_if > > data_xvo; // Vector of exports of data from    dmeu
   sc_core::sc_vector< sc_core::sc_in<simd_sig_dmeu_state_c> >        state_vi; // Vector of dmeu processing states
   sc_core::sc_vector< sc_core::sc_out<bool> >                        proc_vo;// Vector of enable signals to dmeu
   sc_core::sc_port<sc_core::sc_fifo_in_if< simd_sig_ptree_c>>        busw_i;   // Configuration port
   sc_core::sc_port<sc_core::sc_fifo_out_if<simd_sig_ptree_c>>        event_o;  // Event output

   // Constructor declaration
   SC_CTOR( simd_sys_xbar_c );

   // Init declaration (to be used after the instantiation and before the port binding)
   void init(
         boost::optional<const boost_pt::ptree&> _pref_p,
         const std::vector<simd_sys_dmeu_info_t>& dmeu_info );

   void add_trace(
         sc_core::sc_trace_file* tf,
         const std::string& top_name );

private:
   // vector of destination ports
   class xbar_dst_vec_c
      : public std::vector<bool> {
   public:
      // Index of the destination port which generates master READY signal
      // in the configuration with multiple destinations
      std::size_t dst_master = UINT_MAX; // Master destination port
      bool        event = false;         // Generate event on the DMEU_ST_DONE state
      xbar_dst_vec_c( std::size_t _size );
   };

   // Static attributes for XBAR ports (static mapping of the xbar port to the module port)
   class xbar_port_attr_c {
   public:
      std::size_t                                  mod_port   = 0;           // port number in DM/EU
      boost::optional<const simd_sys_dmeu_info_t&> mod_info_p = boost::none; // pointer to DM/EU information
   };

   enum  conf_state_t {
      ST_CONF_IDLE      = 0, // Configuration slot is idle
      ST_CONF_ACTIVE    = 1, // Configuration slot is active
      ST_CONF_WAIT      = 2, // Configuration slot is waiting to be connected
      ST_CONF_WAIT_NEXT = 3  // Configuration slot is waiting to be switched to the next conf
   };

   // Vectors of input and output interfaces.
   sc_core::sc_vector<simd_chn_dmeu_data_o> data_chn_src_v;
   sc_core::sc_vector<simd_chn_dmeu_data_i> data_chn_dst_v;

   // Vectors of the elastic buffers for src and dst ports
   sc_core::sc_vector<simd_sys_eb_src_c> eb_src_v;
   sc_core::sc_vector<simd_sys_eb_dst_c> eb_dst_v;

   // output from the interface and input to the elastic buffer
   sc_core::sc_vector<sc_core::sc_signal<simd_sig_dmeu_data_c >> sif_eb_data_v ;
   sc_core::sc_vector<sc_core::sc_signal<simd_sig_dmeu_valid_c>> sif_eb_valid_v;
   sc_core::sc_vector<sc_core::sc_signal<simd_sig_dmeu_ready_c>> sif_eb_ready_v;

   // output from the elastic buffer and input to the xbar matrix
   sc_core::sc_vector<sc_core::sc_signal<simd_sig_dmeu_data_c >> src_data_v;
   sc_core::sc_vector<sc_core::sc_signal<simd_sig_dmeu_valid_c>> src_valid_v;
   sc_core::sc_vector<sc_core::sc_signal<simd_sig_dmeu_ready_c>> src_ready_v;
   sc_core::sc_vector<sc_core::sc_signal<bool>>                  src_tail_v;

   // output from the xbar matrix and input to the elastic buffer
   sc_core::sc_vector<sc_core::sc_signal<simd_sig_dmeu_data_c >> dst_data_v;
   sc_core::sc_vector<sc_core::sc_signal<simd_sig_dmeu_valid_c>> dst_valid_v;
   sc_core::sc_vector<sc_core::sc_signal<simd_sig_dmeu_ready_c>> dst_ready_v;
   sc_core::sc_vector<sc_core::sc_signal<bool>>                  dst_tail_v;

   sc_core::sc_vector<sc_core::sc_signal<bool>>                  mod_proc_v;

   // output from the elastic buffer and input to the interface
   sc_core::sc_vector<sc_core::sc_signal<simd_sig_dmeu_data_c >> eb_dif_data_v ;
   sc_core::sc_vector<sc_core::sc_signal<simd_sig_dmeu_valid_c>> eb_dif_valid_v;
   sc_core::sc_vector<sc_core::sc_signal<simd_sig_dmeu_ready_c>> eb_dif_ready_v;

   // Process declarations
   void exec_thrd(
         void );

   // Other declarations
   std::vector<boost_pt::ptree> config;       // The size of the config vector is set at the init stage
   std::vector<conf_state_t>    config_state; // Flags which indicate configuration state

   // Input to output routing matrix
   std::vector<xbar_dst_vec_c> map_src_to_dst;   // Switching matrix

   std::vector<bool> src_port_conn;    // is src port connected?
   std::vector<bool> dst_port_conn;    // is dst port connected?
   std::vector<bool> proc_rb;

   // static attributes of the XBAR ports
   std::vector<xbar_port_attr_c> src_port_attr;
   std::vector<xbar_port_attr_c> dst_port_attr;

   // Pointer to the preferences. To be used by the callbacks
   // before_end_of_elaboration, end_of_elaboration, start_of_simulation, end_of_simulation
   boost::optional<const boost_pt::ptree&> pref_p;

   // Parse request from the busw
   void parse_busw_req(
         const simd_sig_ptree_c& req );

   std::size_t src_dmeu2xbar(
         const std::string& mod_name,
         const std::size_t  mod_port );

   std::size_t dst_dmeu2xbar(
         const std::string& mod_name,
         const std::size_t  mod_port );

   bool is_conf_empty(
         const std::size_t conf_idx );

   std::size_t next_conf_idx(
         const std::size_t conf_idx );

   bool is_active(
         const std::size_t conf_idx );

   // Generate Event
   void event(
         const std::string& mod_name,
         const std::string& event_id );

   void xbar_set(
         const std::size_t  conf_idx,
         const conf_state_t state );

   void xbar_thrd(      // Switching thread
         void );

   void conf_thrd(      // Configuration and control thread
         void );

   // Check if any src port is connected to a given dst port
   bool is_map_to_dst(
         const std::size_t dst_port );

   // Check if any dst port is connected to a given src port
   bool is_map_to_src(
         const std::size_t src_port );

   // Pointer to the vector which contains information about dm/eu instances
   boost::optional<const std::vector<simd_sys_dmeu_info_t>&> dmeu_info_p = boost::none;

   sc_core::sc_event evt_conf;

}; // simd_sys_xbar_c

} // namespace simd

#endif /* SIMD_SYS_XBAR_INCLUDE_SIMD_SYS_XBAR_H_ */
