/*
 * simd_sys_dm_init.cpp
 *
 *  Description:
 *    DM initialization function
 */


#include <boost/foreach.hpp>
#include <matio.h>
#include <string>
#include "simd_sys_dm_init.h"

namespace simd {

void simd_sys_dm_init(
      boost::optional<const boost_pt::ptree&>    ini_p,
      std::vector<std::vector<simd_dmeu_slot_t>> &mem,
      const simd::simd_sys_dmeu_c                &dmeu ) {

   const boost_pt::ptree& ini = ini_p.get();

   BOOST_FOREACH( const boost_pt::ptree::value_type& seg, ini ) {
      if( !seg.first.empty()) {
         SIMD_REPORT_ERROR( "simd::sys_dm" ) << "Incorrect format";
      }

      // Try accessing the mat file
      std::string  fname;
      std::string  vname;
      size_t       base = 0;

      try {
         base  = seg.second.get<size_t>(       "base" );
         fname = seg.second.get<std::string>(  "file" );
         vname = seg.second.get<std::string>(  "var"  );
      }
      catch( const boost_pt::ptree_error& err ) {
         SIMD_REPORT_ERROR( "simd::sys_dm" ) << dmeu.name() << " " <<  err.what();
      }
      catch( ... ) {
         SIMD_REPORT_ERROR( "simd::sys_dm" ) << dmeu.name() << " Unexpected";
      }

      mat_t *matfp = Mat_Open( fname.c_str(), MAT_ACC_RDONLY );

      if( matfp == NULL ) {
         SIMD_REPORT_ERROR( "simd::sys_dm" ) << dmeu.name() << " Unable to open: " << fname;
      }

      // access the specified variable
      matvar_t *matvar = Mat_VarRead( matfp, vname.c_str());

      if( matvar == NULL ) {
         SIMD_REPORT_ERROR( "simd::sys_dm" ) << dmeu.name() << " Unable to read: " << vname;
      }

      Mat_Close( matfp );     // Close file

      // Check variable it its type is "Column vector of complex double"
      if( !( matvar->rank == 2 && matvar->dims[1] == 1 &&
             matvar->class_type == MAT_C_DOUBLE && matvar->data_type == MAT_T_DOUBLE && matvar->isComplex != 0 )) {
         SIMD_REPORT_ERROR( "simd::sys_dm" ) << dmeu.name() << " Incorrect format: " << vname;
      }

      if( base + matvar->dims[0] > mem.size() * mem.at( 0 ).size()) {
         SIMD_REPORT_ERROR( "simd::sys_dm" ) << dmeu.name() << " Initialization is out of memory";
      }

      mat_complex_split_t *ini_z_split = static_cast<mat_complex_split_t *>(matvar->data); // Pointers to data

      // Transfer data into memory array
      for( size_t ini_data_idx = 0; ini_data_idx < matvar->dims[0]; ini_data_idx ++ ) {
         double re = ( static_cast<double *>( ini_z_split->Re ))[ini_data_idx];
         double im = ( static_cast<double *>( ini_z_split->Im ))[ini_data_idx];

         size_t full_addr = base + ini_data_idx;

         simd_dmeu_slot_t &mem_el = mem.at( full_addr % simd_dmeu_data_c::dim )
                                       .at( full_addr / simd_dmeu_data_c::dim );

         if( mem_el.ena ) {
            SIMD_REPORT_ERROR( "simd::sys_dm" ) << dmeu.name() << " Duplicate initialization";
         }

         try {
            mem_el.smp = simd_dmeu_smp_t( re, im );
            mem_el.ena = true;
         }
         catch( const std::out_of_range& err ) {
            SIMD_REPORT_ERROR( "simd::sys_dm" ) << dmeu.name() << " " <<  err.what();
         }
         catch( ... ) {
            SIMD_REPORT_ERROR( "simd::sys_dm" ) << dmeu.name() << " Unexpected";
         }
      } // for( size_t ini_data_idx = 0; ...

      Mat_VarFree( matvar );  // Free matvar memory
   } // BOOST_FOREACH( const boost_pt::ptree::value_type& seg, ini ) { ...
}

} // namespace simd
