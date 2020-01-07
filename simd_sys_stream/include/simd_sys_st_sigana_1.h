/*
 * simd_sys_st_sigana_1.h
 *
 *  Description:
 *    Declaration of the 1-input signal analiser
 *    (used for Valid-Ready Interface  testing)
 */

#ifndef SIMD_SYS_STREAM_INCLUDE_SIMD_SYS_ST_SIGANA_1_H_
#define SIMD_SYS_STREAM_INCLUDE_SIMD_SYS_ST_SIGANA_1_H_

#include <boost/random.hpp>
#include "simd_sys_dmeu.h"

// Short alias for the namespace
namespace boost_pt = boost::property_tree;
namespace boost_rn = boost::random;

namespace simd {

   class simd_sys_st_sigana_1_c
   : public simd::simd_sys_dmeu_c { // declare module class
   public:
      // Constructor
      simd_sys_st_sigana_1_c(
            sc_core::sc_module_name nm ) : simd_sys_dmeu_c( nm ) {};

      // Init declaration
      void init(
            boost::optional<const boost_pt::ptree&> _pref_p );

      void add_trace(
            sc_core::sc_trace_file* tf,
            const std::string& top_name );

      static inline std::string func( void ) { return "st_sigana_1"; }

   protected:
      void proc_thrd(      // Processing and configuration thread
            void );

      void ready_thrd(     // Ready interconnect thread
            void );

      void req_run(   // Execution request
            void );

   private:
      static const std::size_t n_data_i = 1;
      static const std::size_t n_ready  = 1;
      int  smp_count = 0;
      bool ready_ovr = false;
      boost_rn::mt19937 rng_eng;

   }; // class simd_sys_st_sigana_1_c
} // namespace simd

#endif /* SIMD_SYS_STREAM_INCLUDE_SIMD_SYS_ST_SIGANA_1_H_ */
