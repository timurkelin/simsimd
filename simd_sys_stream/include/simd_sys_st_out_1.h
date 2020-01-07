/*
 * simd_sys_st_out_1.h
 *
 *  Description:
 *    Declaration of the stream block for data output from the core
 *    From the Core perspective this block represents Data source.
 *
 */

#ifndef SIMD_SYS_STREAM_INCLUDE_SIMD_SYS_ST_OUT_1_H_
#define SIMD_SYS_STREAM_INCLUDE_SIMD_SYS_ST_OUT_1_H_

#include <boost/random.hpp>
#include "simd_sys_dmeu.h"

// Short alias for the namespace
namespace boost_pt = boost::property_tree;
namespace boost_rn = boost::random;

namespace simd {

   class simd_sys_st_out_1_c
   : public simd::simd_sys_dmeu_c { // declare module class
   public:
      // Constructor
      simd_sys_st_out_1_c(
            sc_core::sc_module_name nm )
            : simd_sys_dmeu_c( nm )
            , rng_eng()
            , rng_dist_uni()
            , rng_ws( rng_eng, rng_dist_uni ) {};

      // Init declaration
      void init(
            boost::optional<const boost_pt::ptree&> _pref_p );

      void add_trace(
            sc_core::sc_trace_file* tf,
            const std::string& top_name );

      static inline std::string func( void ) { return "st_out_1"; }

   protected:
      void proc_thrd(   // Processing and configuration thread
            void );

      void ready_thrd(  // Ready interconnect thread
            void );

      void req_run(     // Execution request
            void );

   private:
      static const std::size_t n_data_i = 1; // Number of input ports to the module
      static const std::size_t n_ready  = 1;

      std::size_t smp_cnt  = 0;   // Sample counter

      // Pool addressing
      std::string pool_seg;          // Pool segment
      std::size_t pool_seg_hash = 0;
      std::size_t blk_start     = 0; // Block start. Block size is identified by the data source
      std::size_t blk_offs      = 0; // Pool segment address

      // Wait states
      std::size_t ws_min = 0;    // min wait states
      std::size_t ws_max = 0;    // max wait states
      std::size_t ws_cnt = 0;    // wait state counter for the on-going transaction

      boost_rn::mt19937 rng_eng;
      boost_rn::uniform_int_distribution<> rng_dist_uni;
      boost_rn::variate_generator<boost::mt19937&, boost_rn::uniform_int_distribution<> > rng_ws;

   }; // class simd_sys_st_out_1_c
} // namespace simd

#endif /* SIMD_SYS_STREAM_INCLUDE_SIMD_SYS_ST_OUT_1_H_ */
