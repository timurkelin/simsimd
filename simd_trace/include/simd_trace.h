/*
 * simd_trace.h
 *
 *  Description:
 *    VCD trace handler
 */

#ifndef SIMD_TRACE_INCLUDE_SIMD_TRACE_H_
#define SIMD_TRACE_INCLUDE_SIMD_TRACE_H_

#include <boost/property_tree/ptree.hpp>
#include <boost/optional/optional.hpp>
#include <systemc>

// Short alias for the namespace
namespace boost_pt = boost::property_tree;

namespace simd {
   class simd_trace_c {
   public:
      void init(
            boost::optional<const boost_pt::ptree&> pref_p,
            sc_core::sc_trace_file* tf_ = NULL );

      ~simd_trace_c( void );

      sc_core::sc_trace_file* tf     = NULL;
      bool					  tf_ref = false;
   }; // class simd_trace_c

   extern simd_trace_c simd_trace;
} // namespace simd


#endif /* SIMD_TRACE_INCLUDE_SIMD_TRACE_H_ */
