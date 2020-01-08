/*
 * simd_pref.h
 *
 *  Description:
 *    Simulation preferences builder, reader and parser
 */

#ifndef SIMD_PREF_INCLUDE_SIMD_PREF_H_
#define SIMD_PREF_INCLUDE_SIMD_PREF_H_

#include <boost/optional.hpp>
#include <boost/property_tree/ptree.hpp>

// Short alias for the namespace
namespace boost_pt = boost::property_tree;

namespace simd {
   class simd_pref_c {
   public:
      void load(
            const std::string& fname );

      void save(
            const std::string& fname );

      void init(
            void );

      void parse(
            void );

      boost::optional<const boost_pt::ptree&> get_pref(
            const std::string& field_name,
            bool               check_error = true );

      boost::optional<const boost_pt::ptree&> core_p;
      boost::optional<const boost_pt::ptree&> pool_p;
      boost::optional<const boost_pt::ptree&> clock_p;
      boost::optional<const boost_pt::ptree&> scalar_p;
      boost::optional<const boost_pt::ptree&> time_p;
      boost::optional<const boost_pt::ptree&> report_p;
      boost::optional<const boost_pt::ptree&> trace_p;
      boost::optional<const boost_pt::ptree&> dump_p;

   protected:
      boost_pt::ptree root;

   }; // class simd_pref_c
} // namespace simd

#endif /* SIMD_PREF_INCLUDE_SIMD_PREF_H_ */
