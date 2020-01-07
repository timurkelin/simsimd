/*
 * simd_sys_dm_ram_1r1w.h
 *
 *  Description:
 *    Declaration of the 4-way 1r 1w register file with dual address buses
 */

#ifndef SIMD_SYS_DM_INCLUDE_SIMD_SYS_DM_RAM_1R1W_H_
#define SIMD_SYS_DM_INCLUDE_SIMD_SYS_DM_RAM_1R1W_H_

#include <vector>
#include <map>
#include <boost/optional.hpp>
#include "simd_sys_dmeu.h"
#include "simd_sig_dm_addr.h"
#include "simd_sys_dm_ag.h"

// Short alias for the namespace
namespace boost_pt = boost::property_tree;

namespace simd {

   class simd_sys_dm_ram_1r1w_c
   : public simd::simd_sys_dmeu_c { // declare module class
   public:
      // Constructor
      simd_sys_dm_ram_1r1w_c(
            sc_core::sc_module_name nm) : simd_sys_dmeu_c( nm ) {};

      // Init declaration
      void init(
            boost::optional<const boost_pt::ptree&> _pref_p );

      void add_trace(
            sc_core::sc_trace_file* tf,
            const std::string& top_name );

      static inline std::string func( void ) { return "dm_ram_1r1w"; }

   protected:
      void proc_thrd(      // Processing and configuration thread
            void );

      void ready_thrd(     // Ready interconnect thread
            void );

      void req_run(        // Execution request
            void );

   private:
      static const std::size_t n_data_i = 1;
      static const std::size_t n_data_o = 1;
      static const std::size_t n_ready  = 1;

      std::map<std::string, boost::optional<simd_sys_dm_ag_c &>> wr_ag_list;       // list of AGs for wr side
      std::map<std::string, boost::optional<simd_sys_dm_ag_c &>> rd_ag_list;       // list of AGs for rd side

      // Rd/wr enablers for the 1rf rf
      bool          wr_en      = false; // Write transactions are enabled
      std::size_t   wr_smp_cnt = 0;     // Sample counter
      std::size_t   wr_smp_num = 0;     // Sample counter end
      boost::optional<simd_sys_dm_ag_c &> p_wr_ag = boost::none;  // Current AG


      bool          rd_en      = false; // Read transactions are enabled
      std::size_t   rd_smp_cnt = 0;     // Sample counter
      std::size_t   rd_smp_num = 0;     // Sample counter end
      boost::optional<simd_sys_dm_ag_c &> p_rd_ag = boost::none;  // Current AG

      // Access to memory array (more or less represents HW)
      std::vector<std::vector<simd_dmeu_slot_t>> mem;             // memory array

      simd_sig_dmeu_data_c mem_wr_data;   // Put signal to be able tracing
      simd_sig_dm_addr_c   mem_wr_addr;
      bool                 mem_wr_ena = false;

      simd_sig_dmeu_data_c mem_rd_data;   // Put signal to be able tracing
      simd_sig_dm_addr_c   mem_rd_addr;
      bool                 mem_rd_ena = false;

      typedef enum {
         ACC_MEM_IDLE       = 0,
         ACC_MEM_ADDR       = 1,
         ACC_MEM_ADDR_DATA  = 2,
         ACC_MEM_DATA       = 3,
         ACC_MEM_WACK       = 4
      } simd_dm_ram_1r1w_acc_t;
   }; // class simd_sys_dm_ram_1r1w_c
} // namespace simd

#endif /* SIMD_SYS_DM_INCLUDE_SIMD_SYS_DM_RAM_1R1W_H_ */
