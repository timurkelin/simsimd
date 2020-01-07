/*
 * simd_sys_dm_init.h
 *
 *  Description:
 *    Declaration of the DM initialization function
 */


#ifndef SIMD_SYS_DM_INCLUDE_SIMD_SYS_DM_INIT_H_
#define SIMD_SYS_DM_INCLUDE_SIMD_SYS_DM_INIT_H_

#include <vector>
#include "simd_sys_dmeu.h"


// Short alias for the namespace
namespace boost_pt = boost::property_tree;

namespace simd {

void simd_sys_dm_init(
      boost::optional<const boost_pt::ptree&>    ini_p,
      std::vector<std::vector<simd_dmeu_slot_t>> &mem,
      const simd::simd_sys_dmeu_c                &dmeu );

} // namespace simd


#endif /* SIMD_SYS_DM_INCLUDE_SIMD_SYS_DM_INIT_H_ */
