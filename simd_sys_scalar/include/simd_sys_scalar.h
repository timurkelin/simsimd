/*
 * simd_sys_scalar.h
 *
 *  Description:
 *    Declaration of the system component: Scalar processor
 */

#ifndef SIMD_SYS_SCALAR_INCLUDE_SIMD_SYS_SCALAR_H_
#define SIMD_SYS_SCALAR_INCLUDE_SIMD_SYS_SCALAR_H_

#include <string>
#include <boost/property_tree/ptree.hpp>
#include <boost/optional/optional.hpp>
#include <systemc>
#include "simd_sig_ptree.h"

// Short alias for the namespace
namespace boost_pt = boost::property_tree;

namespace simd {

   SC_MODULE( simd_sys_scalar_c ) { // declare module class

   public:
      // Module I/O Ports
      sc_core::sc_in<bool>                                        clock_i;    // Clock
      sc_core::sc_in<bool>                                        reset_i;    // Reset
      sc_core::sc_port<sc_core::sc_fifo_in_if< simd_sig_ptree_c>> event_i;    // Interrupt request fifo export through the input interface
      sc_core::sc_port<sc_core::sc_fifo_out_if<simd_sig_ptree_c>> busw_o;     // Config output
      sc_core::sc_port<sc_core::sc_fifo_in_if< simd_sig_ptree_c>> busr_i;     // Status input

      // Constructor declaration
      SC_CTOR( simd_sys_scalar_c );

      // Init/config declaration
      void init(
            boost::optional<const boost_pt::ptree&> pref_p );

   private:
      // Process declarations
      void exec_thrd(
            void );

      void wr_conf(
            const std::string& conf );

      // Event processing
      typedef enum {
         EVT_PROC_ERROR_ALL   = -2,
         EVT_PROC_ERROR_ANY   = -1,
         EVT_PROC_NONE        =  0,
         EVT_PROC_VALID_ANY   =  1,
         EVT_PROC_VALID_ALL   =  2
      } evt_proc_t;

      void evt_proc_clear(
            void );

      void evt_proc_init(
            const std::string& evt_list );

      evt_proc_t evt_proc_check(
            const boost_pt::ptree& event_pt );

      typedef struct {
         std::string mod_name;
         std::string evt_name;
         bool        valid_all;
         bool        valid_any;
         bool        error_all;
         bool        error_any;
         bool        evt;
      } evt_proc_data_t;

      std::vector<evt_proc_data_t> evt_proc;

      bool valid_all_present = false;
      bool valid_any_present = false;
      bool error_all_present = false;
      bool error_any_present = false;
   };
}

#endif /* SIMD_SYS_SCALAR_INCLUDE_SIMD_SYS_SCALAR_H_ */
