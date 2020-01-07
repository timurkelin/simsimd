/*
 * simd_dump.h
 *
 *  Description:
 *    Class declaration for dump file io
 */

#ifndef SIMD_DUMP_INCLUDE_SIMD_DUMP_H_
#define SIMD_DUMP_INCLUDE_SIMD_DUMP_H_

#include <vector>
#include <string>
#include <map>
#include <sstream>
#include <iomanip>
#include <boost/property_tree/ptree.hpp>
#include <boost/optional.hpp>
#include <boost/regex.hpp>
#include <systemc>
#include <matio.h>
#include "simd_dump_vec_wr.h"
#include "simd_report.h"

// Short alias for the namespace
namespace boost_pt = boost::property_tree;

namespace simd {
   typedef enum {
      BUF_GENERAL_OK   = 0,  // General OK
      BUF_BELOW_MIN    = 1,  // Sample number is within the reserved size
      BUF_ABOVE_MIN    = 2,  // Buffer is resized
      BUF_PENULT_MAX   = 3,  // Buffer is resized. Only single free element is left
      BUF_FULL_MAX     = 4,  // Buffer is resized. No free elements are left
      BUF_OVERFLOW     = 5,  // Buffer overflow. No operation is done
      BUF_NO_OPERATION = 6,  // No file opened for this dump
      BUF_GEN_ERROR    = 100 // General error
   } simd_dump_buf_wret_t;

   typedef enum {
      BUF_WRITE_LAST = true,
      BUF_WRITE_CONT = false
   } simd_dump_buf_flag_t;

   class  simd_dump_c; // Forward declaration to become a friend

   template <class T>
   class simd_dump_buf_c
   : public sc_core::sc_attr_base {
   public:
      simd_dump_buf_c(
            const std::string& name_ );

      // Write single element to the dump buffer
      simd_dump_buf_wret_t write(
            const T &elem,
            simd_dump_buf_flag_t _flag = BUF_WRITE_CONT );

      // Write a vector of elements to the dump buffer
      simd_dump_buf_wret_t write(
            const std::vector<T> &vec,
            simd_dump_buf_flag_t _flag = BUF_WRITE_CONT );

      friend class simd_dump_c;  // To access private members

   private:
      static const int reg_ref_free = -1;
      static const int reg_ref_skip = -2;
      static const int reg_ref_expr = -3;

      std::size_t size_res = 0;           // Reserved size
      std::size_t size_max = 0;           // maximal  size

      sc_core::sc_time time_str;          // Start time of the current frame
      sc_core::sc_time time_end;          // End   time of the current frame

      std::vector<T> buf;                 // main dump buffer

      int         reg_ref = reg_ref_free; // reference to the entry in the dump register
      std::size_t frame   = 0;            // dump frame counter for the unique variable name
      std::size_t name_hash;              // Name hash. This is used as a part of the variable name in the .mat file

   }; // class simd_dump_buf_c

   class simd_dump_c {
   public:
      void init(
            boost::optional<const boost_pt::ptree&> _pref_p );

      void close_all(
            void );

      template <class T>
      friend class simd_dump_buf_c;

   private:
      typedef struct {
         boost::regex     re; // regex to resolve buffer name
         sc_core::sc_time ts; // Start time
         sc_core::sc_time te; // End   time
         size_t           sr; // Reserved size
         size_t           sm; // Max size
         size_t           fr; // File reference
      } simd_dump_dreg_t;

      typedef std::map<std::size_t, std::string> bi_t;
      typedef struct {
         sc_core::sc_time ts; // Start time
         sc_core::sc_time te; // End   time
         size_t           ns; // Number of active dump sources
         std::string      fn; // destination filename
         mat_t           *fp; // matio file pointer
         bi_t             bi; // source buffer index
      } simd_dump_freg_t;

      std::vector<simd_dump_dreg_t> dump_reg; // dump register
      std::vector<simd_dump_freg_t> file_reg; // file register

      template <class T>
      int  check_dump(
            const simd_dump_buf_c<T> &buf );

      template <class T>
      void write_dump(
            const simd_dump_buf_c<T> &buf );

      // Housekeeping function which keeps track of the files which are not going to be used any longer
      void housekeeping(
            void );

      // Save hash-name index
      void write_index(
            const simd_dump_freg_t& file_data );

      std::string hash_str(
            std::size_t hash );

      static const enum mat_ft       MAT_FILE_VER    = MAT_FT_MAT5;           // _MAT5 - FAST; _MAT73 - SLOW
      static const matio_compression MAT_COMPRESSION = MAT_COMPRESSION_ZLIB;  // Compression slightly slows down the dumping
   }; // class simd_dump_c

extern simd::simd_dump_c   simd_dump;

template <class T> simd_dump_buf_c<T>::simd_dump_buf_c(
      const std::string& name_ )
            : sc_core::sc_attr_base( name_ ) {
   buf.resize( 0 );

   // Create hash for the supplied name
   name_hash = 0;
   boost::hash_combine(
         name_hash,
         name() );
}

template <class T> simd_dump_buf_wret_t simd_dump_buf_c<T>::write( // Write element to the buffer
            const T &elem,
            simd_dump_buf_flag_t _flag /*= BUF_WRITE_CONT*/ ) {
   // Check if the dump buffer update is skipped
   if( reg_ref == reg_ref_expr ) {
      return simd_dump_buf_wret_t::BUF_NO_OPERATION;
   }
   else if( reg_ref == reg_ref_skip ) {
      if( _flag ) {
         reg_ref = reg_ref_free;
      }

      return simd_dump_buf_wret_t::BUF_NO_OPERATION;
   }

   // First write to the dump buffer. Check if there is a valid reference in the register
   if( buf.size() == 0 ) {
      // Check if the buffer object is in the correct state
      if( reg_ref != reg_ref_free ) {
         SIMD_REPORT_ERROR( "simd::sys_dump" ) << name() << " Incorrect buffer state";
         return simd_dump_buf_wret_t::BUF_GEN_ERROR;
      }

      time_str = sc_core::sc_time_stamp();

      reg_ref = simd_dump.check_dump( *this );

      if( reg_ref >= 0 ) {
         // Get buffer sizes
         size_res = simd_dump.dump_reg.at( reg_ref ).sr;
         size_max = simd_dump.dump_reg.at( reg_ref ).sm;

         // Resize the buffer to the reserved size
         buf.resize( size_res );
         buf.clear();
      }
      else {
         if( reg_ref != reg_ref_expr ) {
            reg_ref = _flag ? reg_ref_free : reg_ref_skip;
         }

         return simd_dump_buf_wret_t::BUF_NO_OPERATION;
      }
   }

   // Check for the overflow
   if( buf.size() >= size_max ) {
      SIMD_REPORT_ERROR( "simd::sys_dump" ) << name() << " Buffer overflow";
      return simd_dump_buf_wret_t::BUF_OVERFLOW;
   }

   // Dump buffer update
   buf.push_back( elem );

   // Save simulation time of the last buffer update operation
   time_end = sc_core::sc_time_stamp();

   simd_dump_buf_wret_t ret;

   if( buf.size() == size_max ) {
      ret = simd_dump_buf_wret_t::BUF_FULL_MAX;
   }
   else if( buf.size() == size_max - 1 ) {
      ret = simd_dump_buf_wret_t::BUF_PENULT_MAX;
   }
   else if( buf.size() > size_res ) {
      ret = simd_dump_buf_wret_t::BUF_ABOVE_MIN;
   }
   else {
      ret = simd_dump_buf_wret_t::BUF_BELOW_MIN;
   }

   // Last update to the buffer. Write dump to a file and clear buffer
   if( _flag ) {
      simd_dump.write_dump( *this );

      // Set buffer size to zero while waiting for the next vector to start
      buf.resize( 0 );

      reg_ref = reg_ref_free; // Dump buffer is free
      frame ++;               // Increment frame counter
   }

   return ret;
}

template <class T> simd_dump_buf_wret_t simd_dump_buf_c<T>::write( // Write an array of elements to the buffer
      const std::vector<T> &vec,
      simd_dump_buf_flag_t _flag /*= BUF_WRITE_CONT*/ ) {

   // Check if the dump buffer update is skipped
   if( reg_ref == reg_ref_expr ) {
      return simd_dump_buf_wret_t::BUF_NO_OPERATION;
   }
   else if( reg_ref == reg_ref_skip ) {
      if( _flag ) {
         reg_ref = reg_ref_free;
      }

      return simd_dump_buf_wret_t::BUF_NO_OPERATION;
   }

   // First write to the dump buffer. Check if there is a valid reference in the register
   if( buf.size() == 0 ) {
      if( reg_ref != reg_ref_free ) {
         // Check if the buffer object is in the correct state
         SIMD_REPORT_ERROR( "simd::sys_dump" ) << name() << " Incorrect buffer state";
         return simd_dump_buf_wret_t::BUF_GEN_ERROR;
      }

      time_str = sc_core::sc_time_stamp();

      reg_ref = simd_dump.check_dump( *this );

      if( reg_ref >= 0 ) {
         // Get buffer sizes
         size_res = simd_dump.dump_reg.at( reg_ref ).sr;
         size_max = simd_dump.dump_reg.at( reg_ref ).sm;

         // Resize the buffer to the reserved size
         buf.resize( size_res );
         buf.clear();
      }
      else {
         if( reg_ref != reg_ref_expr ) {
            reg_ref = _flag ? reg_ref_free : reg_ref_skip;
         }

         return simd_dump_buf_wret_t::BUF_NO_OPERATION;
      }
   }

   // Check for the overflow
   if( buf.size() + vec.size() > size_max ) {
      SIMD_REPORT_ERROR( "simd::sys_dump" ) << name() << " Buffer overflow";
      return simd_dump_buf_wret_t::BUF_OVERFLOW;
   }

   // Dump buffer update
   buf.insert(
         buf.end(),
         vec.begin(),
         vec.end());

   // Save simulation time of the last buffer update operation
   time_end = sc_core::sc_time_stamp();

   simd_dump_buf_wret_t ret;

   if( buf.size() == size_max ) {
      ret = simd_dump_buf_wret_t::BUF_FULL_MAX;
   }
   else if( buf.size() == size_max - 1 ) {
      ret = simd_dump_buf_wret_t::BUF_PENULT_MAX;
   }
   else if( buf.size() > size_res ) {
      ret = simd_dump_buf_wret_t::BUF_ABOVE_MIN;
   }
   else {
      ret = simd_dump_buf_wret_t::BUF_BELOW_MIN;
   }

   // Last update to the buffer. Write dump to a file and clear buffer
   if( _flag ) {
      simd_dump.write_dump( *this );

      // Set buffer size to zero while waiting for the next vector to start
      buf.resize( 0 );

      reg_ref = reg_ref_free; // Dump buffer is now free
      frame ++;               // Increment frame counter
   }

   return ret;
}

// Check if the buffer is going to be dumped
template <class T> int  simd_dump_c::check_dump(
      const simd_dump_buf_c<T> &buf ) {

   int  dump_reg_ref = buf.reg_ref_skip;
   bool buf_to_be_used = false;

   for( int dump_idx = 0; dump_idx < dump_reg.size(); dump_idx ++ ) {
      bool buf_match = ( buf.time_str <= dump_reg.at( dump_idx ).te ) &&
                         boost::regex_match( buf.name(), dump_reg.at( dump_idx ).re );

      buf_to_be_used |= buf_match;

      if( buf.time_str >= dump_reg.at( dump_idx ).ts && buf_match ) {

         // Save index
         dump_reg_ref = dump_idx;

         // Increment source counter for the file
         file_reg.at( dump_reg.at( dump_idx ).fr ).ns ++;

         break;
      }
   }

   // Run housekeeping for the file register
   housekeeping();

   return ( buf_to_be_used ? dump_reg_ref : buf.reg_ref_expr );
} // template <class T> int simd_dump_c::check_dump(

// Create dump variable and write dump buffer to file
template <class T> void simd_dump_c::write_dump(
      const simd_dump_buf_c<T> &buf ) {

   // Create mat structure
   const unsigned matstr_nfields = 5;
   const char    *matstr_fields[matstr_nfields] = {
        "name",         // Full name
        "frame",        // Frame counter
        "time_start",   // Frame start (simulation time)
        "time_end",     // Frame end   (simulation time)
        "data" };       // Data
   const int     matstr_rank = 2;
         size_t  matstr_dims[matstr_rank] = {1, 1};

   // Variable name
   std::stringstream os; // Convert frame counter
   os << std::setfill('0')
      << std::setw( sizeof( buf.frame ) * 2 )
      << std::hex
      << buf.frame;

   std::string matstr_name = "v" + hash_str( buf.name_hash ) + "_f" + os.str();

   matvar_t *matstr_p = Mat_VarCreateStruct(
         matstr_name.c_str(),
         matstr_rank,
         matstr_dims,
         matstr_fields,
         matstr_nfields );

   if( matstr_p == NULL ) {
      SIMD_REPORT_ERROR( "simd::sys_dump" ) << " struct for " << buf.name();
   }

   matvar_t *field_p;
   const int field_rank = 2;
   size_t    field_dims[field_rank] = {1, 1};

   // Update start time
   double time_start = buf.time_str.to_double();
   field_p = Mat_VarCreate(
         NULL,
         MAT_C_DOUBLE,
         MAT_T_DOUBLE,
         field_rank,
         field_dims,
         (void *)&time_start,
         MAT_F_DONT_COPY_DATA );

   if( field_p == NULL ) {
      SIMD_REPORT_ERROR( "simd::sys_dump" ) << " struct field for " << buf.name();
   }

   Mat_VarSetStructFieldByName(
         matstr_p,
         "time_start",
         0,
         field_p );

   // Update end time
   double time_end = buf.time_end.to_double();
   field_p = Mat_VarCreate(
         NULL,
         MAT_C_DOUBLE,
         MAT_T_DOUBLE,
         field_rank,
         field_dims,
         (void *)&time_end,
         MAT_F_DONT_COPY_DATA );

   if( field_p == NULL ) {
      SIMD_REPORT_ERROR( "simd::sys_dump" ) << " struct field for " << buf.name();
   }

   Mat_VarSetStructFieldByName(
         matstr_p,
         "time_end",
         0,
         field_p );

   // Update frame count
   uint64_t frame = (double)buf.frame;
   field_p = Mat_VarCreate(
         NULL,
         MAT_C_UINT64,
         MAT_T_UINT64,
         field_rank,
         field_dims,
         (void *)&frame,
         MAT_F_DONT_COPY_DATA );

   if( field_p == NULL ) {
      SIMD_REPORT_ERROR( "simd::sys_dump" ) << " struct field for " << buf.name();
   }

   Mat_VarSetStructFieldByName(
         matstr_p,
         "frame",
         0,
         field_p );

   // Call data writer
   field_p = vec_writer( buf.buf );

   if( field_p == NULL ) {
      SIMD_REPORT_ERROR( "simd::sys_dump" ) << " struct field for " << buf.name();
   }

   Mat_VarSetStructFieldByName(
         matstr_p,
         "data",
         0,
         field_p );

   // Update name
   field_dims[0] = 1;
   field_dims[1] = buf.name().size();

   field_p = Mat_VarCreate(
         NULL,
         MAT_C_CHAR,
         MAT_T_UTF8,
         field_rank,
         field_dims,
         (void *)buf.name().c_str(),
         MAT_F_DONT_COPY_DATA );

   if( field_p == NULL ) {
      SIMD_REPORT_ERROR( "simd::sys_dump" ) << " struct field for " << buf.name();
   }

   Mat_VarSetStructFieldByName(
         matstr_p,
         "name",
         0,
         field_p );

   // File IO
   size_t file_reg_ref = dump_reg.at( buf.reg_ref ).fr;

   // Check if the file needs to be created
   if( file_reg.at( file_reg_ref ).fp == NULL ) {
      file_reg.at( file_reg_ref ).fp = Mat_CreateVer(
            file_reg.at( file_reg_ref ).fn.c_str(),
            NULL,
            MAT_FILE_VER );

      if( file_reg.at( file_reg_ref ).fp == NULL ) {
         SIMD_REPORT_ERROR( "simd::sys_dump" ) << " File I/O error for " << buf.name();
      }
   }

   // Write structure to a file
   int res = Mat_VarWrite(
         file_reg.at( file_reg_ref ).fp,
         matstr_p,
         MAT_COMPRESSION );

   if( res != 0 ) {
      SIMD_REPORT_ERROR( "simd::sys_dump" ) << " File I/O error for " << buf.name() << " ret:" << res;
   }

   // Free memory
   Mat_VarFree( matstr_p );

   // Update buffer index
   auto bi_it = file_reg.at( file_reg_ref ).bi.find( buf.name_hash );

   if( bi_it == file_reg.at( file_reg_ref ).bi.end()) {
      file_reg.at( file_reg_ref ).bi[buf.name_hash] = buf.name();
   }
   else if( bi_it->second != buf.name()) {
      SIMD_REPORT_ERROR( "simd::sys_dump" ) << " hash collision for " << buf.name();
   }

   // Decrement source counter for the file
   file_reg.at( file_reg_ref ).ns --;

   // Run housekeeping for the file register
   housekeeping();

} // template <class T> int simd_dump_c::write_dump(

} // namespace simd

#endif /* SIMD_DUMP_INCLUDE_SIMD_DUMP_H_ */
