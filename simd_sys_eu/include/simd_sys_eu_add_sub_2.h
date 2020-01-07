/*
 * simd_sys_eu_add_sub_2.h
 *
 *  Description:
 *    Declaration of the 2-input, 1-output add-subtract execution unit
 *    (this is used for testing)
 */

#ifndef SIMD_SYS_EU_INCLUDE_SIMD_SYS_EU_ADD_SUB_2_H_
#define SIMD_SYS_EU_INCLUDE_SIMD_SYS_EU_ADD_SUB_2_H_

#include "simd_sys_dmeu.h"

// Short alias for the namespace
namespace boost_pt = boost::property_tree;

namespace simd {

   class simd_sys_eu_add_sub_2_c
   : public simd::simd_sys_dmeu_c { // declare module class
   public:
      // Constructor
      simd_sys_eu_add_sub_2_c(
            sc_core::sc_module_name nm ) : simd_sys_dmeu_c( nm ) {};

      // Init declaration
      void init(
            boost::optional<const boost_pt::ptree&> _pref_p );

      void add_trace(
            sc_core::sc_trace_file* tf,
            const std::string& top_name );

      static inline std::string func( void ) { return "eu_add_sub_2"; }

   protected:
      void proc_thrd(      // Processing and configuration thread
            void );

      void ready_thrd(     // Ready interconnect thread
            void );

      void req_run(        // Execution request
            void );

   private:
      static const std::size_t  n_data_i = 2;
      static const std::size_t  n_data_o = 1;
      static const std::size_t  n_ready  = 2;

      // Multipliers for the sign change
      double sign_re0 = 0.;
      double sign_im0 = 0.;
      double sign_re1 = 0.;
      double sign_im1 = 0.;

      // Operation when the samples in the frame is not available
      std::string smp_nena_op;

   }; // class simd_sys_eu_add_sub_2_c

} // namespace simd

#endif /* SIMD_SYS_EU_INCLUDE_SIMD_SYS_EU_ADD_SUB_2_H_ */
