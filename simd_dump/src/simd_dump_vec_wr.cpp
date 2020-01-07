/*
 * simd_dump_vec_wr.cpp
 *
 *  Description:
 *    Vector writers for different data types
 */
#include <boost/property_tree/detail/file_parser_error.hpp>
#include <boost/foreach.hpp>
#include "simd_dump_vec_wr.h"
#include "simd_conv_ptree.h"
#include "simd_report.h"

namespace simd {

matvar_t *vec_writer(
      const std::vector<int> &vec ) {

   matvar_t *matvar_p;
   const int matvar_rank = 2;
   size_t    matvar_dims[matvar_rank];

   // Convert types using range constructor
   std::vector<int64_t> vec_conv(
         vec.begin(),
         vec.end());

   // Save vector
   matvar_dims[0] = vec_conv.size();
   matvar_dims[1] = 1;
   matvar_p = Mat_VarCreate(
         NULL,
         MAT_C_INT64,
         MAT_T_INT64,
         matvar_rank,
         matvar_dims,
         (void *)vec_conv.data(),
         0 );

   return matvar_p;
}

matvar_t *vec_writer(
      const std::vector<unsigned int> &vec ) {

   matvar_t *matvar_p;
   const int matvar_rank = 2;
   size_t    matvar_dims[matvar_rank];

   // Convert types using range constructor
   std::vector<uint64_t> vec_conv(
         vec.begin(),
         vec.end());

   // Save vector
   matvar_dims[0] = vec_conv.size();
   matvar_dims[1] = 1;
   matvar_p = Mat_VarCreate(
         NULL,
         MAT_C_UINT64,
         MAT_T_UINT64,
         matvar_rank,
         matvar_dims,
         (void *)vec_conv.data(),
         0 );

   return matvar_p;
}

matvar_t *vec_writer(
      const std::vector<std::size_t> &vec ) {

   matvar_t *matvar_p;
   const int matvar_rank = 2;
   size_t    matvar_dims[matvar_rank];

   // Convert types using range constructor
   std::vector<uint64_t> vec_conv(
         vec.begin(),
         vec.end());

   // Save vector
   matvar_dims[0] = vec_conv.size();
   matvar_dims[1] = 1;
   matvar_p = Mat_VarCreate(
         NULL,
         MAT_C_UINT64,
         MAT_T_UINT64,
         matvar_rank,
         matvar_dims,
         (void *)vec_conv.data(),
         0 );

   return matvar_p;
}

matvar_t *vec_writer(
      const std::vector<bool> &vec ) {

   matvar_t *matvar_p;
   const int matvar_rank = 2;
   size_t    matvar_dims[matvar_rank];

   // Convert types using range constructor
   std::vector<int16_t> vec_conv(
         vec.begin(),
         vec.end());

   // Save vector
   matvar_dims[0] = vec_conv.size();
   matvar_dims[1] = 1;
   matvar_p = Mat_VarCreate(
         NULL,
         MAT_C_INT16,
         MAT_T_INT16,
         matvar_rank,
         matvar_dims,
         (void *)vec_conv.data(),
         MAT_F_LOGICAL );

   return matvar_p;
}

matvar_t *vec_writer(
      const std::vector<double> &vec ) {

   matvar_t *matvar_p;
   const int matvar_rank = 2;
   size_t    matvar_dims[matvar_rank];

   // Save vector
   matvar_dims[0] = vec.size();
   matvar_dims[1] = 1;
   matvar_p = Mat_VarCreate(
         NULL,
         MAT_C_DOUBLE,
         MAT_T_DOUBLE,
         matvar_rank,
         matvar_dims,
         (void *)vec.data(),
         0 );

   return matvar_p;
}

matvar_t *vec_writer(
      const std::vector<std::string> &vec,
      const bool _copy /*=false*/ ) {

   // Create array of structures. Array of strings is not currently supported in matio (1.5.15)
   const unsigned matstr_nfields = 1;
   const char    *matstr_fields[matstr_nfields] = {
        "str" };       // string

   const int     matstr_rank = 2;
         size_t  matstr_dims[matstr_rank] = {vec.size(), 1};

   matvar_t *matstr_p = Mat_VarCreateStruct(
         NULL,
         matstr_rank,
         matstr_dims,
         matstr_fields,
         matstr_nfields );

   std::size_t vec_idx = 0;

   matvar_t *field_p;
   const int field_rank = 2;
   size_t    field_dims[field_rank];

   BOOST_FOREACH( const std::string &el, vec ) {
      field_dims[0] = 1;
      field_dims[1] = el.size();
      field_p = Mat_VarCreate(
            NULL,
            MAT_C_CHAR,
            MAT_T_UTF8,
            field_rank,
            field_dims,
            (void *)( el.c_str()),
            _copy ? 0 : MAT_F_DONT_COPY_DATA );

      Mat_VarSetStructFieldByName(
            matstr_p,
            "str",
            vec_idx,
            field_p );

      vec_idx ++;
   }

   return matstr_p;
}

matvar_t *vec_writer(
      const std::vector<std::complex<double>> &vec ) {

   matvar_t *matvar_p;
   const int matvar_rank = 2;
   size_t    matvar_dims[matvar_rank];

   // Separate re and im parts from the array of complex to match the mat format
   std::vector<double> vec_real( vec.size());
   std::vector<double> vec_imag( vec.size());

   std::vector<double>::iterator vec_real_it = vec_real.begin();
   std::vector<double>::iterator vec_imag_it = vec_imag.begin();

   BOOST_FOREACH( const std::complex<double> &el, vec ) {
      *vec_real_it = el.real();
      *vec_imag_it = el.imag();

      vec_real_it ++;
      vec_imag_it ++;
   }

   mat_complex_split_t mat_complex = {
         (void *)vec_real.data(),
         (void *)vec_imag.data()};

   // Save vector
   matvar_dims[0] = vec.size();
   matvar_dims[1] = 1;
   matvar_p = Mat_VarCreate(
         NULL,
         MAT_C_DOUBLE,
         MAT_T_DOUBLE,
         matvar_rank,
         matvar_dims,
         (void *)&mat_complex,
         MAT_F_COMPLEX );

   return matvar_p;
}

matvar_t *vec_writer(
      const std::vector<simd_dmeu_data_c> &vec ) {

   const unsigned matstr_nfields = 2;
   const char    *matstr_fields[matstr_nfields] = {
        "smp",   // smp
        "ena" }; // ena

   const int     matstr_rank = 2;
         size_t  matstr_dims[matstr_rank] = {vec.size(), 1};

   matvar_t *matstr_p = Mat_VarCreateStruct(
         NULL,
         matstr_rank,
         matstr_dims,
         matstr_fields,
         matstr_nfields );

   std::size_t vec_idx = 0;

   double  smp_real[simd_dmeu_data_c::dim];
   double  smp_imag[simd_dmeu_data_c::dim];
   int16_t      ena[simd_dmeu_data_c::dim];

   mat_complex_split_t mat_complex = {
         (void *)smp_real,
         (void *)smp_imag};

   matvar_t *field_p;
   const int field_rank = 2;
   size_t    field_dims[field_rank] = {1, simd_dmeu_data_c::dim};

   BOOST_FOREACH( const simd_dmeu_data_c &el, vec ) {
      for( std::size_t idx = 0; idx < simd_dmeu_data_c::dim; idx ++ ) {
         ena[idx] = el[idx].ena;

         smp_real[idx] = el[idx].smp.real();
         smp_imag[idx] = el[idx].smp.imag();
      }

      // Save smp vector
      field_p = Mat_VarCreate(
            NULL,
            MAT_C_DOUBLE,
            MAT_T_DOUBLE,
            field_rank,
            field_dims,
            (void *)&mat_complex,
            MAT_F_COMPLEX );

      Mat_VarSetStructFieldByName(
            matstr_p,
            "smp",
            vec_idx,
            field_p );

      // Save ena vector
      field_p = Mat_VarCreate(
            NULL,
            MAT_C_INT16,
            MAT_T_INT16,
            field_rank,
            field_dims,
            (void *)ena,
            MAT_F_LOGICAL );

      Mat_VarSetStructFieldByName(
            matstr_p,
            "ena",
            vec_idx,
            field_p );

      vec_idx ++;
   }

   return matstr_p;
}

matvar_t *vec_writer(
      const std::vector<boost_pt::ptree> &vec ) {

   std::vector<std::string> vec_conv( vec.size());

   std::size_t vec_idx = 0;

   BOOST_FOREACH( const boost_pt::ptree &el, vec ) {
      pt2str( el, vec_conv.at( vec_idx ));

      vec_idx ++;
   }

   return vec_writer( vec_conv, true );
}

matvar_t *vec_writer(
      const std::vector<simd_dm_addr_c> &vec ) {

   const unsigned matstr_nfields = 3;
   const char    *matstr_fields[matstr_nfields] = {
        "ena",    // ena
        "addr",   // addr
        "perm" }; // parm

   const int     matstr_rank = 2;
         size_t  matstr_dims[matstr_rank] = {vec.size(), 1};

   matvar_t *matstr_p = Mat_VarCreateStruct(
         NULL,
         matstr_rank,
         matstr_dims,
         matstr_fields,
         matstr_nfields );

   std::size_t vec_idx = 0;

   uint64_t  addr[simd_dm_addr_c::dim];
   uint8_t   perm[simd_dm_addr_c::dim];
    int16_t   ena[simd_dm_addr_c::dim];

   matvar_t *field_p;
   const int field_rank = 2;
   size_t    field_dims[field_rank] = {1, simd_dm_addr_c::dim};

   BOOST_FOREACH( const simd_dm_addr_c &el, vec ) {
      for( std::size_t idx = 0; idx < simd_dm_addr_c::dim; idx ++ ) {
          ena[idx] = el[idx].ena;
         perm[idx] = el[idx].perm;
         addr[idx] = el[idx].addr;
      }

      // Save ena vector
      field_p = Mat_VarCreate(
            NULL,
            MAT_C_INT16,
            MAT_T_INT16,
            field_rank,
            field_dims,
            (void *)ena,
            MAT_F_LOGICAL );

      Mat_VarSetStructFieldByName(
            matstr_p,
            "ena",
            vec_idx,
            field_p );

      // Save addr vector
      field_p = Mat_VarCreate(
            NULL,
            MAT_C_UINT64,
            MAT_T_UINT64,
            field_rank,
            field_dims,
            (void *)addr,
            0 );

      Mat_VarSetStructFieldByName(
            matstr_p,
            "addr",
            vec_idx,
            field_p );

      // Save perm vector
      field_p = Mat_VarCreate(
            NULL,
            MAT_C_UINT8,
            MAT_T_UINT8,
            field_rank,
            field_dims,
            (void *)perm,
            0 );

      Mat_VarSetStructFieldByName(
            matstr_p,
            "perm",
            vec_idx,
            field_p );

      vec_idx ++;
   }

   return matstr_p;
}

} // namespace simd
