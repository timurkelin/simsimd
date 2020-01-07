/*
 * simd_sys_pool.cpp
 *
 *  Description:
 *    Methods the data and memory pool class
 */

#include <boost/foreach.hpp>
#include <matio.h>
#include "simd_sys_pool.h"
#include "simd_report.h"

namespace simd {

// Constructor
simd_sys_pool_c::simd_sys_pool_c(
      void )
            : sc_core::sc_attr_base( "pool" ) {
   pool.clear();
} // simd_sys_pool_c::simd_sys_pool_c(

// Calculate hash from seg_name
std::size_t simd_sys_pool_c::hash(
      const std::string& seg_name ) {
   size_t seg_hash = 0;

   boost::hash_combine(
         seg_hash,
         seg_name );

   return seg_hash;
} // size_t simd_sys_pool_c::hash(

// Init
void simd_sys_pool_c::init(
      boost::optional<const boost_pt::ptree&> _pref_p ) {

   const boost_pt::ptree& pref = _pref_p.get();

   BOOST_FOREACH( const boost_pt::ptree::value_type& seg, pref ) {
      if( !seg.first.empty()) {
         SIMD_REPORT_ERROR( "simd::sys_pool" ) << "Incorrect format";
      }

      // Get segment
      std::string seg_name;

      try {
         seg_name = seg.second.get<std::string>( "name" );
      }
      catch( const boost_pt::ptree_error& err ) {
         SIMD_REPORT_ERROR( "simd::sys_pool" ) << name() << " " <<  err.what();
      }
      catch( ... ) {
         SIMD_REPORT_ERROR( "simd::sys_pool" ) << name() << " Unexpected";
      }

      // Calculate hash from seg_name
      size_t seg_hash = this->hash( seg_name );

      // Check if the segment name already exists
      if( pool.find( seg_hash ) == pool.end()) {
         pool[seg_hash].name = seg_name;
      }
      else {
         SIMD_REPORT_ERROR( "simd::sys_dump" ) << " Duplicate segment name " << seg_name;
      }

      // Reference to the segment vector
      std::vector<simd_pool_t>& seg_vec = pool[seg_hash].seg;

      // Get the initialization section
      boost::optional<const boost_pt::ptree&> seg_init_p;

      try {
         seg_init_p = seg.second.get_child( "init" );
      }
      catch( const boost_pt::ptree_error& err ) {
         SIMD_REPORT_ERROR( "simd::sys_pool" ) << name() << " " <<  err.what();
      }
      catch( ... ) {
         SIMD_REPORT_ERROR( "simd::sys_pool" ) << name() << " Unexpected";
      }

      // Get the size of the segment
      bool init_done = false;
      std::size_t seg_init_size;

      if( !init_done ) {
         boost::optional<std::size_t> init_p;

         try {
            init_p = seg.second.get_optional<std::size_t>( "size" );
         }
         catch( const boost_pt::ptree_error& err ) {
            SIMD_REPORT_ERROR( "simd::sys_pool" ) << name() << " " <<  err.what();
         }
         catch( ... ) {
            SIMD_REPORT_ERROR( "simd::sys_pool" ) << name() << " Unexpected";
         }

         if( init_p ) {
            seg_init_size = init_p.get();
            init_done = true;
         }
      }

      // String parser should be the last one
      if( !init_done ) {
         boost::optional<std::string> init_p;

         try {
            init_p = seg.second.get_optional<std::string>( "size" );
         }
         catch( const boost_pt::ptree_error& err ) {
            SIMD_REPORT_ERROR( "simd::sys_pool" ) << name() << " " <<  err.what();
         }
         catch( ... ) {
            SIMD_REPORT_ERROR( "simd::sys_pool" ) << name() << " Unexpected";
         }

         if( !init_p  ) {
            SIMD_REPORT_ERROR( "simd::sys_pool" ) << name() << " Incorrect node";
         }

         if( init_p.get() == "auto" ) {
            seg_init_size = seg_size( seg_init_p );
            init_done = true;
         }
         else {
            SIMD_REPORT_ERROR( "simd::sys_pool" ) << name() << " Incorrect size specification";
         }
      }

      // Resize segment vector and initialize the segment
      if( init_done ) {
         seg_vec.resize( seg_init_size );
         seg_init(
               seg_vec,
               seg_init_p );
      }
      else {
         SIMD_REPORT_ERROR( "simd::sys_pool" ) << name() << " Incorrect size specification";
      }

   } // BOOST_FOREACH( const boost_pt::ptree::value_type& seg, pref )
} // void simd_sys_pool_c::init(

// Read an element from the segment of the pool
const simd_elem_t& simd_sys_pool_c::read(
      const std::size_t seg_hash,
      const std::size_t addr ) {

   // Find segment by hash
   auto seg_it = pool.find( seg_hash );

   if( seg_it == pool.end()) {
      SIMD_REPORT_ERROR( "simd::sys_pool" ) << name() << " Incorrect name of the segment";
   }

   simd_pool_t seg_data;

   try {
      seg_data = seg_it->second.seg.at( addr );
   }
   catch( const std::out_of_range& err ) {
      SIMD_REPORT_ERROR( "simd::sys_pool" ) << name() << " " <<  err.what();
   }
   catch( ... ) {
      SIMD_REPORT_ERROR( "simd::sys_pool" ) << name() << " Unexpected";
   }

   if( !seg_data.ena ) {
      SIMD_REPORT_ERROR( "simd::sys_pool" ) << name() << " Read from un-initialized memory" <<
            " addr: " << addr;
   }

   return seg_it->second.seg.at( addr ).smp;
} // const simd_pool_t& simd_sys_pool_c::read(

// Write an element to the segment of the pool
void simd_sys_pool_c::write(
      const std::size_t seg_hash,
      const std::size_t addr,
      const simd_elem_t& data ) {

   // Find segment by hash
   auto seg_it = pool.find( seg_hash );

   if( seg_it == pool.end()) {
      SIMD_REPORT_ERROR( "simd::sys_pool" ) << name() << " Incorrect name of the segment";
   }

   try {
      seg_it->second.seg.at( addr ).ena = true;
      seg_it->second.seg.at( addr ).smp = data;
   }
   catch( const std::out_of_range& err ) {
      SIMD_REPORT_ERROR( "simd::sys_pool" ) << name() << " " <<  err.what();
   }
   catch( ... ) {
      SIMD_REPORT_ERROR( "simd::sys_pool" ) << name() << " Unexpected";
   }
} // void simd_sys_pool_c::write(

// Segment size
std::size_t simd_sys_pool_c::size(
      const std::size_t seg_hash ) {

   // Find segment by hash
   auto seg_it = pool.find( seg_hash );

   if( seg_it == pool.end()) {
      SIMD_REPORT_ERROR( "simd::sys_pool" ) << name() << " Incorrect name of the segment";
   }

   return seg_it->second.seg.size();
} // size_t simd_sys_pool_c::size(

// Calculate the size of the pool segment for size:auto option
std::size_t simd_sys_pool_c::seg_size(
      boost::optional<const boost_pt::ptree&> ini_p ) {

   const boost_pt::ptree& ini = ini_p.get();

   size_t auto_size = 0;

   BOOST_FOREACH( const boost_pt::ptree::value_type& seg, ini ) {
      if( !seg.first.empty()) {
         SIMD_REPORT_ERROR( "simd::sys_pool" ) << "Incorrect format";
      }

      // Try accessing the mat file
      std::string  fname;
      std::string  vname;
      std::size_t  base = 0;

      try {
         base  = seg.second.get<size_t>(       "base" );
         fname = seg.second.get<std::string>(  "file" );
         vname = seg.second.get<std::string>(  "var"  );
      }
      catch( const boost_pt::ptree_error& err ) {
         SIMD_REPORT_ERROR( "simd::sys_pool" ) << name() << " " <<  err.what();
      }
      catch( ... ) {
         SIMD_REPORT_ERROR( "simd::sys_pool" ) << name() << " Unexpected";
      }

      mat_t *matfp = Mat_Open( fname.c_str(), MAT_ACC_RDONLY );

      if( matfp == NULL ) {
         SIMD_REPORT_ERROR( "simd::sys_pool" ) << name() << " Unable to open: " << fname;
      }

      // access the specified variable
      matvar_t *matvar = Mat_VarRead( matfp, vname.c_str());

      if( matvar == NULL ) {
         SIMD_REPORT_ERROR( "simd::sys_pool" ) << name() << " Unable to read: " << vname;
      }

      Mat_Close( matfp );     // Close file

      // Check variable it its type is "Column vector of complex double"
      if( !( matvar->rank == 2 && matvar->dims[1] == 1 &&
             matvar->class_type == MAT_C_DOUBLE && matvar->data_type == MAT_T_DOUBLE && matvar->isComplex != 0 )) {
         SIMD_REPORT_ERROR( "simd::sys_pool" ) << name() << " Incorrect format: " << vname;
      }

      if( auto_size < base + matvar->dims[0] ) {
         auto_size = base + matvar->dims[0];
      }

      Mat_VarFree( matvar );  // Free matvar memory
   } // BOOST_FOREACH( const boost_pt::ptree::value_type& seg, ini ) { ...

   return auto_size;
} // size_t simd_sys_pool_c::seg_size(

// Initialize pool segment
void simd_sys_pool_c::seg_init(
      std::vector<simd_dmeu_slot_t>&          seg_vec,
      boost::optional<const boost_pt::ptree&> ini_p ) {

   const boost_pt::ptree& ini = ini_p.get();

   BOOST_FOREACH( const boost_pt::ptree::value_type& seg, ini ) {
      if( !seg.first.empty()) {
         SIMD_REPORT_ERROR( "simd::sys_pool" ) << "Incorrect format";
      }

      // Try accessing the mat file
      std::string  fname;
      std::string  vname;
      std::size_t  base = 0;

      try {
         base  = seg.second.get<size_t>(       "base" );
         fname = seg.second.get<std::string>(  "file" );
         vname = seg.second.get<std::string>(  "var"  );
      }
      catch( const boost_pt::ptree_error& err ) {
         SIMD_REPORT_ERROR( "simd::sys_pool" ) << name() << " " <<  err.what();
      }
      catch( ... ) {
         SIMD_REPORT_ERROR( "simd::sys_pool" ) << name() << " Unexpected";
      }

      mat_t *matfp = Mat_Open( fname.c_str(), MAT_ACC_RDONLY );

      if( matfp == NULL ) {
         SIMD_REPORT_ERROR( "simd::sys_pool" ) << name() << " Unable to open: " << fname;
      }

      // access the specified variable
      matvar_t *matvar = Mat_VarRead( matfp, vname.c_str());

      if( matvar == NULL ) {
         SIMD_REPORT_ERROR( "simd::sys_pool" ) << name() << " Unable to read: " << vname;
      }

      Mat_Close( matfp );     // Close file

      // Check variable it its type is "Column vector of complex double"
      if( !( matvar->rank == 2 && matvar->dims[1] == 1 &&
             matvar->class_type == MAT_C_DOUBLE && matvar->data_type == MAT_T_DOUBLE && matvar->isComplex != 0 )) {
         SIMD_REPORT_ERROR( "simd::sys_pool" ) << name() << " Incorrect format: " << vname;
      }

      if( base + matvar->dims[0] > seg_vec.size()) {
         SIMD_REPORT_ERROR( "simd::sys_pool" ) << name() << " Initialization is out of segment size";
      }

      mat_complex_split_t *ini_z_split = static_cast<mat_complex_split_t *>(matvar->data); // Pointers to data

      // Transfer data into memory array
      for( size_t ini_data_idx = 0; ini_data_idx < matvar->dims[0]; ini_data_idx ++ ) {
         double re = ( static_cast<double *>( ini_z_split->Re ))[ini_data_idx];
         double im = ( static_cast<double *>( ini_z_split->Im ))[ini_data_idx];

         simd_dmeu_slot_t &seg_el = seg_vec.at( base + ini_data_idx );

         if( seg_el.ena ) {
            SIMD_REPORT_ERROR( "simd::sys_pool" ) << name() << " Duplicate initialization";
         }

         try {
            seg_el.smp = simd_dmeu_smp_t( re, im );
            seg_el.ena = true;
         }
         catch( const std::out_of_range& err ) {
            SIMD_REPORT_ERROR( "simd::sys_pool" ) << name() << " " <<  err.what();
         }
         catch( ... ) {
            SIMD_REPORT_ERROR( "simd::sys_pool" ) << name() << " Unexpected";
         }
      } // for( size_t ini_data_idx = 0; ...

      Mat_VarFree( matvar );  // Free matvar memory
   } // BOOST_FOREACH( const boost_pt::ptree::value_type& seg, ini ) { ...
} // void simd_sys_pool_c::seg_init( ...

} // namespace simd {
