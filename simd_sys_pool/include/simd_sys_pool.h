/*
 * simd_sys_pool.h
 *
 *  Description:
 *    Declaration of the data and memory pool class
 */

#ifndef SIMD_SYS_POOL_INCLUDE_SIMD_SYS_POOL_H_
#define SIMD_SYS_POOL_INCLUDE_SIMD_SYS_POOL_H_

#include <vector>
#include "simd_sys_dmeu.h"


// Short alias for the namespace
namespace boost_pt = boost::property_tree;

namespace simd {

// Type for the contents of the memory pool
typedef simd_dmeu_slot_t simd_pool_t;
typedef simd_dmeu_smp_t  simd_elem_t;

class simd_sys_pool_c
: public sc_core::sc_attr_base {
public:
   simd_sys_pool_c(
         void );

   void init(
         boost::optional<const boost_pt::ptree&> pref_p );

   std::size_t hash(
         const std::string& seg_name );

   // Address segment by its name hash
   const simd_elem_t& read(
         const std::size_t  seg_hash,
         const std::size_t  addr );

   void write(
         const std::size_t  seg_hash,
         const std::size_t  addr,
         const simd_elem_t& data );

   std::size_t size(
         const std::size_t seg_hash );

private:
   // Calculate the size of the pool segment for size:auto option
   std::size_t seg_size(
         boost::optional<const boost_pt::ptree&> ini_p );

   // Initialize pool segment
   void   seg_init(
         std::vector<simd_pool_t>&               seg_vec,
         boost::optional<const boost_pt::ptree&> ini_p );

   typedef struct {
      std::string              name;   // Segment name
      std::vector<simd_pool_t> seg;    // Pool segment
   } simd_sys_pool_seg_t;

   // associative array indexed with hash
   std::map<std::size_t, simd_sys_pool_seg_t> pool;
};

} // namespace simd

#endif /* SIMD_SYS_POOL_INCLUDE_SIMD_SYS_POOL_H_ */
