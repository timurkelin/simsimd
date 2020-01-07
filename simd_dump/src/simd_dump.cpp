/*
 * simd_dump.cpp
 *
 *  Description:
 *    Methods for dump file io
 */

#include <boost/foreach.hpp>
#include <boost/regex.hpp>
#include "simd_dump.h"
#include "simd_ptree_time.h"
#include "simd_common.h" // Simulation end time
#include "simd_report.h"

namespace simd {

void simd_dump_c::init(
      boost::optional<const boost_pt::ptree&> _pref_p ) {

   const boost_pt::ptree& pref = _pref_p.get();

   BOOST_FOREACH( const boost_pt::ptree::value_type& dmp, pref ) {
      simd_dump_dreg_t dump_data;
      std::string      file_name;
      std::string      str_time_str;
      std::string      end_time_str;

      if( !dmp.first.empty()) {
         SIMD_REPORT_ERROR( "simd::sys_dump" ) << "Incorrect format";
      }

      // Get preferences
      try {
         dump_data.re = dmp.second.get<std::string>("buf_regex"); // assign to regex
         dump_data.ts = dmp.second.get<sc_core::sc_time>("time_start");
         end_time_str = dmp.second.get<std::string>("time_end");
         dump_data.sr = dmp.second.get<size_t>("size_res");
         dump_data.sm = dmp.second.get<size_t>("size_max");
         file_name    = dmp.second.get<std::string>("file");
      }
      catch( const boost_pt::ptree_error& err ) {
         SIMD_REPORT_ERROR( "simd::sys_dump" ) << err.what();
      }
      catch( const boost::regex_error& err ) {
         SIMD_REPORT_ERROR( "simd::sys_dump" ) << err.what();
      }
      catch( ... ) {
         SIMD_REPORT_ERROR( "simd::sys_dump" ) << "Unexpected";
      }

      if( dump_data.sr > dump_data.sm ) {
         SIMD_REPORT_ERROR( "simd::sys_dump" ) << " Incorrect specification of the buffer size";
      }

      // Convert end time
      dump_data.te = sc_core::sc_time::from_string(
            ( end_time_str == "finish" ) ? simd::simd_time.end_str.c_str()    // Dump until simulation finishes
                                         : end_time_str.c_str() );            // Dump until predetermined time

      // Find if the file register already contains the file name
      size_t fc;
      for( fc = 0; fc < file_reg.size(); fc ++ ) {
         if( file_reg.at( fc ).fn == file_name ) {
            break;
         }
      }

      if( fc != file_reg.size()) {
         file_reg.at( fc ).ns ++;
         file_reg.at( fc ).ts = ( file_reg.at( fc ).ts > dump_data.ts ) ? dump_data.ts : file_reg.at( fc ).ts; // min
         file_reg.at( fc ).te = ( file_reg.at( fc ).te < dump_data.te ) ? dump_data.te : file_reg.at( fc ).te; // max
      }
      else {
         simd_dump_freg_t file_data;

         file_data.fn = file_name;
         file_data.fp = NULL;
         file_data.ns = 0;
         file_data.ts = dump_data.ts;
         file_data.te = dump_data.te;
         file_reg.push_back( file_data );
      }

      dump_data.fr = fc;

      dump_reg.push_back( dump_data );
   }
} // void simd_dump_c::init(

// Ensure that all files are closed
void simd_dump_c::close_all(
      void ) {
   BOOST_FOREACH( simd_dump_freg_t& file_data, file_reg ) {
      if( file_data.fp != NULL ) {

         write_index( file_data );

         int res = Mat_Close(
               file_data.fp );

         if( res != 0 ) {
            SIMD_REPORT_ERROR( "simd::sys_dump" ) << " file close " << file_data.fn;
         }

         file_data.fp = NULL;
      }
   }
} // void simd_dump_c::close_all(

// Look through the file register and close the files which are not going to be written any more
void simd_dump_c::housekeeping(
      void ) {
   BOOST_FOREACH( simd_dump_freg_t& file_data, file_reg ) {
      if( file_data.fp != NULL &&
          file_data.ns == 0    &&
          file_data.te <= sc_core::sc_time_stamp() &&
          file_data.te != sc_core::SC_ZERO_TIME ) {

         write_index( file_data );

         int res = Mat_Close(
               file_data.fp );

         if( res != 0 ) {
            SIMD_REPORT_ERROR( "simd::sys_dump" ) << " file close " << file_data.fn;
         }

         file_data.fp = NULL;
      }
   }
} // void simd_dump_c::housekeeping(

// Save hash-name index to mat file
void simd_dump_c::write_index(
      const simd_dump_freg_t& file_data ) {

   // Create mat structure
   const unsigned matstr_nfields = 3;
   const char    *matstr_fields[matstr_nfields] = {
        "hash",         // name hash
        "hash_str",     // name hash (string representation)
        "name" };       // full name

   const int     matstr_rank = 2;
         size_t  matstr_dims[matstr_rank] = {file_data.bi.size(), 1};

   if( file_data.bi.size() == 0 ) {
      return;
   }

   matvar_t *matstr_p = Mat_VarCreateStruct(
         "index",
         matstr_rank,
         matstr_dims,
         matstr_fields,
         matstr_nfields );

   if( matstr_p == NULL ) {
      SIMD_REPORT_ERROR( "simd::sys_dump" ) << " struct for index";
   }

   std::size_t bi_idx = 0;

   BOOST_FOREACH( const bi_t::value_type &be, file_data.bi ) {
      matvar_t *field_p;
      const int field_rank = 2;
      size_t    field_dims[field_rank];

      // Save hash
      uint64_t name_hash = be.first;
      field_dims[0] = 1;
      field_dims[1] = 1;
      field_p = Mat_VarCreate(
            NULL,
            MAT_C_UINT64,
            MAT_T_UINT64,
            field_rank,
            field_dims,
            (void *)&name_hash,
            0 );

      if( field_p == NULL ) {
         SIMD_REPORT_ERROR( "simd::sys_dump" ) << " struct field for index";
      }

      Mat_VarSetStructFieldByName(
            matstr_p,
            "hash",
            bi_idx,
            field_p );

      // Save hash string
      std::string name_hash_str = hash_str( be.first );
      field_dims[0] = 1;
      field_dims[1] = name_hash_str.size();
      field_p = Mat_VarCreate(
            NULL,
            MAT_C_CHAR,
            MAT_T_UTF8,
            field_rank,
            field_dims,
            (void *)name_hash_str.c_str(),
            0 );

      if( field_p == NULL ) {
         SIMD_REPORT_ERROR( "simd::sys_dump" ) << " struct field for index";
      }

      Mat_VarSetStructFieldByName(
            matstr_p,
            "hash_str",
            bi_idx,
            field_p );

      // Save name
      field_dims[0] = 1;
      field_dims[1] = be.second.size();
      field_p = Mat_VarCreate(
            NULL,
            MAT_C_CHAR,
            MAT_T_UTF8,
            field_rank,
            field_dims,
            (void *)be.second.c_str(),
            MAT_F_DONT_COPY_DATA );

      if( field_p == NULL ) {
         SIMD_REPORT_ERROR( "simd::sys_dump" ) << " struct field for index";
      }

      Mat_VarSetStructFieldByName(
            matstr_p,
            "name",
            bi_idx,
            field_p );

      bi_idx ++;
   }

   // Write structure to a file
   int res = Mat_VarWrite(
         file_data.fp,
         matstr_p,
         MAT_COMPRESSION );

   if( res != 0 ) {
      SIMD_REPORT_ERROR( "simd::sys_dump" ) << " File I/O error for index";
   }

   // Free memory
   Mat_VarFree( matstr_p );
}

// Convert hash value into a string
std::string simd_dump_c::hash_str(
      std::size_t hash ) {

   std::stringstream os;
   os << std::setfill('0')
      << std::setw( sizeof( hash ) * 2 )
      << std::hex
      << hash;

   return os.str();
}

} // namespace simd
