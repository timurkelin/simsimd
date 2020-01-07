/*
 * simd_ptree_time.h
 *
 *  Description:
 *    sc_time translator for boost ptree
 */

#ifndef SIMD_COMMON_INCLUDE_SIMD_PTREE_TIME_H_
#define SIMD_COMMON_INCLUDE_SIMD_PTREE_TIME_H_

#include <boost/property_tree/ptree.hpp>
#include <systemc>

namespace boost {
   namespace property_tree {

   // Create translator for sc_time
   struct sc_time_translator_t {
      typedef std::string      internal_type;
      typedef sc_core::sc_time external_type;

      // Translate from string into sc_time
      boost::optional<external_type> get_value( const internal_type& str ) {
         return boost::optional<external_type>( sc_core::sc_time::from_string( str.c_str()));
      } // get_value(

      // Translate from sc_time into string
      boost::optional<internal_type> put_value( const external_type& val ) {
         return boost::optional<internal_type>( val.to_string() );
      } // put_value(
   }; // struct sc_time_translator_t

   // Register translator
   template<typename Ch, typename Traits, typename Alloc>
   struct translator_between<std::basic_string<Ch, Traits, Alloc>, sc_core::sc_time> {
      typedef sc_time_translator_t type;
   };

   } // namespace property_tree
} // namespace boost

#endif /* SIMD_COMMON_INCLUDE_SIMD_PTREE_TIME_H_ */
