/*
 * simd_dump_vec_wr.h
 *
 *  Description:
 *    Prototypes for vector writers for different data types
 */
#include <vector>
#include <string>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <matio.h>
#include "simd_sig_dmeu_data.h" // definition of simd_dmeu_data_c
#include "simd_sig_dm_addr.h"   // Definition of simd_dm_addr_c

#ifndef SIMD_DUMP_INCLUDE_SIMD_DUMP_VEC_WR_H_
#define SIMD_DUMP_INCLUDE_SIMD_DUMP_VEC_WR_H_

// Short alias for the namespace
namespace boost_pt = boost::property_tree;
namespace boost_jp = boost::property_tree::json_parser;

namespace simd {

// Vector writers for basic types
matvar_t *vec_writer(
      const std::vector<int> &vec );
matvar_t *vec_writer(
      const std::vector<unsigned int> &vec );
matvar_t *vec_writer(
      const std::vector<std::size_t> &vec );
matvar_t *vec_writer(
      const std::vector<bool> &vec );
matvar_t *vec_writer(
      const std::vector<double> &vec );

// Vector writers for aggregate types
matvar_t *vec_writer(
      const std::vector<std::complex<double>> &vec );
matvar_t *vec_writer(
      const std::vector<std::string> &vec,
      const bool _copy = false );

// Vector writers for simd core types
matvar_t *vec_writer(
      const std::vector<simd_dmeu_data_c> &vec );
matvar_t *vec_writer(
      const std::vector<boost_pt::ptree> &vec );
matvar_t *vec_writer(
      const std::vector<simd_dm_addr_c> &vec );

} // namespace simd

#endif /* SIMD_DUMP_INCLUDE_SIMD_DUMP_VEC_WR_H_ */
