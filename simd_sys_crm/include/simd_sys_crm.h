/*
 * simd_sys_crm.h
 *
 *  Description:
 *    Clock and reset manager: channel and interface to the clocked modules
 */


#ifndef SIMD_SYS_CRM_INCLUDE_SIMD_SYS_CRM_H_
#define SIMD_SYS_CRM_INCLUDE_SIMD_SYS_CRM_H_

#include <boost/property_tree/ptree.hpp>
#include <boost/optional/optional.hpp>
#include <systemc>
#include "simd_report.h"

// Short alias for the namespace
namespace boost_pt = boost::property_tree;

namespace simd {
   SC_MODULE( simd_sys_crm_c ) {
   public:
      // Module I/O Ports
      sc_core::sc_out<bool> clock_o;
      sc_core::sc_out<bool> reset_o;

      // Constructor declaration
      SC_CTOR( simd_sys_crm_c );

      // Init declaration (to be used after the instantiation and before the port binding)
      void init(
            boost::optional<const boost_pt::ptree&> pref_p );

   private:
      // Process declarations
      void clock_thrd(
            void );

      void reset_thrd(
            void );

      sc_core::sc_clock *clock_p;
      sc_core::sc_time   period;

      // Frequency to period conversion
      sc_core::sc_time to_period(
            const std::string& str );

      // Pointer to the preferences. To be used by the callbacks
      // before_end_of_elaboration, end_of_elaboration, start_of_simulation, end_of_simulation
      boost::optional<const boost_pt::ptree&> pref_ptr;
   };
} // namespace simd

#endif /* SIMD_SYS_CRM_INCLUDE_SIMD_SYS_CRM_H_ */
