/*
 * simd_ptree_xbar.cpp
 *
 *  Description:
 *    System component: ptree cross bar
 */

#include <algorithm>
#include <iterator>
#include <boost/foreach.hpp>
#include "simd_ptree_xbar.h"
#include "simd_conv_ptree.h"
#include "simd_assert.h"
#include "simd_report.h"

namespace simd {

SC_HAS_PROCESS( simd::simd_ptree_xbar_c );
simd_ptree_xbar_c::simd_ptree_xbar_c(
      sc_core::sc_module_name nm )
   : sc_core::sc_module( nm )
   , vi( "vi" )
   , vo( "vo" ) {

   // Process registrations
   SC_THREAD( exec_thrd );
}

void simd_ptree_xbar_c::init(
      boost::optional<const boost_pt::ptree&> _pref_p ) {

   boost::optional<const boost_pt::ptree&> src_list_p = _pref_p.get().get_child_optional("src_list");
   boost::optional<const boost_pt::ptree&> dst_list_p = _pref_p.get().get_child_optional("dst_list");

   if( !src_list_p.is_initialized()) {
      SIMD_REPORT_ERROR( "simd::xbar" ) << name() << " Source list not found";
   }

   std::size_t src_list_size = src_list_p.get().size();

   if( src_list_size == 0 ) {
      SIMD_REPORT_ERROR( "simd::xbar" ) << name() << " Source list is empty";
   }

   // Initialise vector of src ports and src map
   vi.init(       src_list_size );
   vi_map.resize( src_list_size );

   std::vector<port_map_t>::iterator src_map_it = vi_map.begin();

   BOOST_FOREACH( const boost_pt::ptree::value_type& src_el, src_list_p.get()) {
      if( !src_el.first.empty()) {
         SIMD_REPORT_ERROR( "simd::xbar" ) << name() <<  " Incorrect structure";
      }

      boost::optional<std::string> src_name_p = src_el.second.get_optional<std::string>("name");
      boost::optional<std::string> src_mask_p = src_el.second.get_optional<std::string>("mask");
      boost::optional<std::string> src_dump_p = src_el.second.get_optional<std::string>("dump");

      if(( !src_name_p.is_initialized() &&
           !src_mask_p.is_initialized()) ||
         (  src_name_p.is_initialized() &&
            src_mask_p.is_initialized()) ||
           !src_dump_p.is_initialized() ) {
         std::string _str;

         SIMD_REPORT_ERROR( "simd::xbar" ) << name()
                                           << " Incorrect format for src_list element: "
                                           << pt2str( src_el.second, _str );
      }

      if( src_name_p.is_initialized() ) {
         src_map_it->name = src_name_p.get();
      }
      else if( src_mask_p.is_initialized() ) {
         try {
            src_map_it->mask = src_mask_p.get();
         }
         catch( const boost::regex_error& err ) {
            SIMD_REPORT_ERROR( "simd::xbar" ) << err.what();
         }
         catch( ... ) {
            SIMD_REPORT_ERROR( "simd::xbar" ) << "Unexpected";
         }
      }

      src_map_it->dump_name = src_dump_p.get();

      if( src_map_it->dump_name.size()) {
         simd_dump_buf_c<boost_pt::ptree> *dump_raw_ptr =
               new simd_dump_buf_c<boost_pt::ptree>( src_map_it->dump_name );

         src_map_it->dump_p =
               boost::optional<simd_dump_buf_c<boost_pt::ptree> &>( *dump_raw_ptr );
      }

      src_map_it = std::next( src_map_it );
   } // BOOST_FOREACH( const boost_pt::ptree::value_type& src_el, src_list_p.get())

   if( !dst_list_p.is_initialized()) {
      SIMD_REPORT_ERROR( "simd::xbar" ) << name() << " Destination list not found";
   }

   std::size_t dst_list_size = dst_list_p.get().size();

   if( dst_list_size == 0 ) {
      SIMD_REPORT_ERROR( "simd::xbar" ) << name() << " Destination list is empty";
   }

   // Initialise vector of dst ports and dst map
   vo.init(       dst_list_size );
   vo_map.resize( dst_list_size );

   std::vector<port_map_t>::iterator dst_map_it = vo_map.begin();

   BOOST_FOREACH( const boost_pt::ptree::value_type& dst_el, dst_list_p.get()) {
      if( !dst_el.first.empty()) {
         SIMD_REPORT_ERROR( "simd::xbar" ) << name() <<  " Incorrect structure";
      }

      boost::optional<std::string> dst_name_p = dst_el.second.get_optional<std::string>("name");
      boost::optional<std::string> dst_mask_p = dst_el.second.get_optional<std::string>("mask");
      boost::optional<std::string> dst_dump_p = dst_el.second.get_optional<std::string>("dump");

      if(( !dst_name_p.is_initialized() &&
           !dst_mask_p.is_initialized()) ||
         (  dst_name_p.is_initialized() &&
            dst_mask_p.is_initialized()) ||
           !dst_dump_p.is_initialized() ) {
         std::string _str;

         SIMD_REPORT_ERROR( "simd::xbar" ) << name()
                                           << " Incorrect format for dst_list element: "
                                           << pt2str( dst_el.second, _str );
      }

      if( dst_name_p.is_initialized() ) {
         dst_map_it->name  = dst_name_p.get();
         dst_map_it->regex = false;
      }
      else if( dst_mask_p.is_initialized() ) {
         try {
            dst_map_it->mask = dst_mask_p.get();
         }
         catch( const boost::regex_error& err ) {
            SIMD_REPORT_ERROR( "simd::xbar" ) << err.what();
         }
         catch( ... ) {
            SIMD_REPORT_ERROR( "simd::xbar" ) << "Unexpected";
         }

         dst_map_it->regex = true;
      }

      dst_map_it->dump_name = dst_dump_p.get();

      if( dst_map_it->dump_name.size()) {
         simd_dump_buf_c<boost_pt::ptree> *dump_raw_ptr =
               new simd_dump_buf_c<boost_pt::ptree>( dst_map_it->dump_name );

         dst_map_it->dump_p =
               boost::optional<simd_dump_buf_c<boost_pt::ptree> &>( *dump_raw_ptr );
      }

      dst_map_it = std::next( dst_map_it );
   } // BOOST_FOREACH( const boost_pt::ptree::value_type& dst_el, dst_list_p.get())
} // void simd_ptree_xbar_c::init(

void simd_ptree_xbar_c::exec_thrd( void ) {
   sc_core::wait(sc_core::SC_ZERO_TIME);

   for(;;) {
      // Create an event combined from all the input ports
      sc_core::sc_event_or_list status_vi_or_list;

      for( std::size_t n_stat = 0; n_stat < vi.size(); n_stat ++ ) {
         status_vi_or_list |= vi.at( n_stat )->data_written_event();
      }

      sc_core::wait( status_vi_or_list );

      // Resolve events which were triggered, and make a list of event sources
      for( std::size_t n_stat = 0; n_stat < vi.size(); n_stat ++ ) {
         while( vi.at( n_stat )->num_available()) {
            boost_pt::ptree        pt_raw = vi.at( n_stat )->read().get();
            const boost_pt::ptree& pt_inp = pt_raw;
            simd_sig_ptree_c       pt_out;

            if( vi_map.at( n_stat ).dump_p.is_initialized()) {
               vi_map.at( n_stat ).dump_p.get().write( pt_inp, BUF_WRITE_LAST );
            }

            if( vo.size() == 1 ) { // Single output. Don't check dst
               pt_out.set( pt_inp );
               vo.at( 0 )->write( pt_out );

               if( vo_map.at( 0 ).dump_p.is_initialized()) {
                  vo_map.at( 0 ).dump_p.get().write( pt_inp, BUF_WRITE_LAST );
               }
            } // if( vo.size() == 1 )
            else {
               boost::optional<const boost_pt::ptree&> dst_list_p = pt_inp.get_child_optional("dst");
               boost::optional<std::string>            dst_str_p  = pt_inp.get_optional<std::string>("dst");

               if( dst_str_p.is_initialized() && dst_str_p.get() == "broadcast" ) { // Broadcast to all outputs
                  pt_out.set( pt_inp );

                  for( std::size_t dst_idx = 0; dst_idx < vo.size(); dst_idx ++ ) {
                     vo.at( dst_idx )->write( pt_out );

                     if( vo_map.at( dst_idx ).dump_p.is_initialized()) {
                        vo_map.at( dst_idx ).dump_p.get().write( pt_inp, BUF_WRITE_LAST );
                     }
                  }
               } else if( dst_list_p.is_initialized()) { // Sent to the outputs specified in the list
                  std::vector<bool> dst_wr( vo.size(), false );

                  BOOST_FOREACH( const boost_pt::ptree::value_type& dst_el, dst_list_p.get()) {
                     if( !dst_el.first.empty()) {
                        SIMD_REPORT_ERROR( "simd::xbar" ) << name() <<  " Incorrect structure";
                     }

                     boost::optional<std::string> dst_mod_p = dst_el.second.get_value_optional<std::string>();

                     if( !dst_mod_p.is_initialized()) {
                        SIMD_REPORT_ERROR( "simd::xbar" ) << name() << " Incorrect format";
                     }

                     auto vo_map_it = std::find_if(
                         vo_map.begin(),
                         vo_map.end(),
                         [dst_mod_p]( const port_map_t &el )->bool {
                            return el.regex ? boost::regex_match( dst_mod_p.get(),   el.mask )
                                            :                   ( dst_mod_p.get() == el.name ); } );

                     if( vo_map_it == vo_map.end()) {
                        SIMD_REPORT_ERROR( "simd::xbar" ) << name() << " Destination not found: " << dst_mod_p.get();
                     }

                     std::size_t dst_idx = std::distance(
                           vo_map.begin(), vo_map_it );

                     if( dst_wr.at( dst_idx )) {
                        SIMD_REPORT_ERROR( "simd::xbar" ) << name() << " Duplicate access: " << dst_mod_p.get();
                     }
                     else {
                        vo.at( dst_idx )->write( pt_out.set( pt_inp )); // Write data to the output
                        dst_wr.at( dst_idx ) = true;

                        if( vo_map.at( dst_idx ).dump_p.is_initialized()) {
                           vo_map.at( dst_idx ).dump_p.get().write( pt_inp, BUF_WRITE_LAST );
                        }
                     }
                  } // BOOST_FOREACH( const boost_pt::ptree::value_type& dst_el, dst_list_p.get())
               } // if( dst_list_p.is_initialized())
               else {
                  SIMD_REPORT_ERROR( "simd::xbar" ) << name() << " Destination not found";
               }
            } // if( vo.size() == 1 ) ... else ...
         } // while( vi.at( n_stat )->num_available())
      } // for( std::size_t n_stat = 0; n_stat < vi.size(); n_stat ++ )
   } // for(;;)
}

} // namespace simd
