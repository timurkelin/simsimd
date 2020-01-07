/*
 * simd_sys_dm_ag.h
 *
 *  Description:
 *    Declaration of the address generator classes and functions
 */

#ifndef SIMD_SYS_DM_INCLUDE_SIMD_SYS_DM_AG_H_
#define SIMD_SYS_DM_INCLUDE_SIMD_SYS_DM_AG_H_

#include <string>
#include <boost/optional.hpp>
#include "simd_sys_dmeu.h"
#include "simd_sig_dm_addr.h"

// Short alias for the namespace
namespace boost_pt = boost::property_tree;

namespace simd {


   class simd_sys_dm_ag_c {
   public:
      simd_sys_dm_ag_c( boost::optional<const simd_sys_dmeu_c &> _caller_p )
      : caller_p( _caller_p ) {};

      virtual ~simd_sys_dm_ag_c( void ){};
      
      // Initialize with AG preferences
      virtual void init(
            boost::optional<const boost_pt::ptree&> _conf_p ) = 0;

      // Convert frame counter value into RAM addresses
      virtual void conv(
            const int count,
            simd_dm_addr_c& addr ) = 0;

   protected:
      boost::optional<const simd_sys_dmeu_c&> caller_p;
      boost::optional<const boost_pt::ptree&> conf_p;
   }; // class simd_sys_dm_ag_c;

   boost::optional<simd_sys_dm_ag_c &> simd_sys_dm_ag_new(
         const std::string& func,
         boost::optional<const simd_sys_dmeu_c &> _caller_p );

} // namespace simd

#endif /* SIMD_SYS_DM_INCLUDE_SIMD_SYS_DM_AG_H_ */
