/*
 * simd_time.h
 *
 *  Description:
 *    Simulation time and resolution
 */

#ifndef SIMD_TIME_INCLUDE_SIMD_TIME_H_
#define SIMD_TIME_INCLUDE_SIMD_TIME_H_

#include <string>
#include <boost/property_tree/ptree.hpp>
#include <boost/optional/optional.hpp>
#include <systemc>

// Short alias for the namespace
namespace boost_pt = boost::property_tree;

namespace simd {
   class simd_time_c {
   public:
      void init(
            boost::optional<const boost_pt::ptree&> pref_p );

      std::string res_str;
      std::string end_str;

      double res_sec;   // We can't use sc_time because of the call to sc_set_time_resolution()
      double end_sec;

   private:
      double to_time_sec(
            const std::string& str);
   }; // class simd_time_c
} // namespace simd

#endif /* SIMD_TIME_INCLUDE_SIMD_TIME_H_ */
