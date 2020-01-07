/*
 * simd_hash_ptree.h
 *
 *  Description:
 *    hash for boost property tree
 */

#ifndef SIMD_COMMON_INCLUDE_SIMD_HASH_PTREE_H_
#define SIMD_COMMON_INCLUDE_SIMD_HASH_PTREE_H_

#include <string>
#include <boost/property_tree/ptree.hpp>
#include <boost/functional/hash.hpp>

// Short alias for the namespace
namespace boost_pt = boost::property_tree;

namespace boost {
   template<typename Key, typename Data, typename KeyCompare>
   struct hash<boost_pt::basic_ptree<Key, Data, KeyCompare> > {
      size_t operator()(boost::property_tree::basic_ptree<Key, Data, KeyCompare> const& pt) const {
         std::size_t seed = 0;

         boost::hash_combine(seed, pt.template get_value<std::string>());
         boost::hash_range(seed, pt.begin(), pt.end());
         return seed;
      }
   }; // struct hash
} // namespace boost



#endif /* SIMD_COMMON_INCLUDE_SIMD_HASH_PTREE_H_ */
