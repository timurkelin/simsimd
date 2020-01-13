/*
 * simd_scalar.cpp
 *
 *  Description:
 *    Methods of the system component: scalar processor
 */

#include <boost/foreach.hpp>
#include "simd_sys_scalar.h"
#include "simd_conv_ptree.h"
#include "simd_assert.h"
#include "simd_report.h"

namespace simd {

SC_HAS_PROCESS( simd::simd_sys_scalar_c );
simd_sys_scalar_c::simd_sys_scalar_c(
      sc_core::sc_module_name nm )
   : sc_core::sc_module( nm )
   , clock_i( "clock_i" )
   , reset_i( "reset_i" )
   , event_i( "event_i" )
   , busw_o(  "busw_o"  )
   , busr_i(  "busr_i"  ) {

   // Process registrations
   SC_CTHREAD( exec_thrd, clock_i.pos() ); //  Synchronous thread for data processing
   reset_signal_is( reset_i, true );
} // simd_sys_scalar_c::simd_sys_scalar_c(

void simd_sys_scalar_c::wr_conf(
      const std::string& conf ) {
   boost_pt::ptree wr_pt;
   simd_sig_ptree_c wr;

   str2pt( conf, wr_pt );

   busw_o->write( wr.set( wr_pt ));
} // void simd_sys_scalar_c::wr_conf(

void simd_sys_scalar_c::evt_proc_clear(
      void ) {
   BOOST_FOREACH( evt_proc_data_t& evt_data, evt_proc ) {
      evt_data.evt = false;
   }
} // void simd_sys_scalar_c::evt_proc_clear(

simd_sys_scalar_c::evt_proc_t simd_sys_scalar_c::evt_proc_check(
      const boost_pt::ptree& event_pt ) {

   std::string mod_name;
   std::string evt_name;

   try {
      mod_name = event_pt.get<std::string>( "source" );
      evt_name = event_pt.get<std::string>( "event_id" );
   }
   catch( const boost_pt::ptree_error& err ) {
      SIMD_REPORT_ERROR( "simd::sys_scalar" ) << err.what();
   }
   catch( ... ) {
      SIMD_REPORT_ERROR( "simd::sys_scalar" ) << "Unexpected";
   }

   bool valid_all = true;
   bool valid_any = false;
   bool error_all = true;
   bool error_any = false;

   BOOST_FOREACH( evt_proc_data_t& evt_data, evt_proc ) {
      evt_data.evt |= ( evt_data.mod_name == mod_name &&
                        evt_data.evt_name == evt_name );

      if( evt_data.valid_all ) {
         valid_all &= evt_data.evt;
      }

      if( evt_data.valid_any ) {
         valid_any |= evt_data.evt;
      }

      if( evt_data.error_all ) {
         error_all &= evt_data.evt;
      }

      if( evt_data.error_any ) {
         error_any |= evt_data.evt;
      }
   }

   if(      error_all && error_all_present ) {
      return EVT_PROC_ERROR_ALL;
   }
   else if( error_any && error_any_present ) {
      return EVT_PROC_ERROR_ANY;
   }
   else if( valid_all && valid_all_present ) {
      return EVT_PROC_VALID_ALL;
   }
   else if( valid_any && valid_any_present ) {
      return EVT_PROC_VALID_ANY;
   }
   else {
      return EVT_PROC_NONE;
   }
} // simd_sys_scalar_c::evt_proc_t simd_sys_scalar_c::evt_proc_check(

void simd_sys_scalar_c::evt_proc_init(
      const std::string& evt_list ) {

   boost_pt::ptree evt_list_pt;

   str2pt(
         evt_list,
         evt_list_pt );

   // Parse
   boost::optional<const boost_pt::ptree &> valid_all_p( evt_list_pt.get_child_optional( "valid_all" ));
   boost::optional<const boost_pt::ptree &> valid_any_p( evt_list_pt.get_child_optional( "valid_any" ));
   boost::optional<const boost_pt::ptree &> error_all_p( evt_list_pt.get_child_optional( "error_all" ));
   boost::optional<const boost_pt::ptree &> error_any_p( evt_list_pt.get_child_optional( "error_any" ));

   if( valid_all_p == boost::none &&
       valid_any_p == boost::none &&
       error_any_p == boost::none &&
       error_all_p == boost::none ) {
      SIMD_REPORT_ERROR( "simd::sys_scalar" ) << " Initialization list is empty";
   }

   evt_proc.clear();

   evt_proc_data_t evt_proc_elem;

   if( valid_all_p != boost::none ) {
      evt_proc_elem.valid_all = true;
      evt_proc_elem.valid_any = false;
      evt_proc_elem.error_any = false;
      evt_proc_elem.error_all = false;
      evt_proc_elem.evt       = false;

      BOOST_FOREACH( const boost_pt::ptree::value_type& evt, valid_all_p.get() ) {
         if( !evt.first.empty()) {
            SIMD_REPORT_ERROR( "simd::sys_scalar" ) << "Incorrect format";
         }

         try {
            evt_proc_elem.mod_name = evt.second.get<std::string>( "mod" );
            evt_proc_elem.evt_name = evt.second.get<std::string>( "evt" );
         }
         catch( const boost_pt::ptree_error& err ) {
            SIMD_REPORT_ERROR( "simd::sys_scalar" ) << err.what();
         }
         catch( ... ) {
            SIMD_REPORT_ERROR( "simd::sys_scalar" ) << "Unexpected";
         }

         evt_proc.push_back( evt_proc_elem );

         valid_all_present = true;
      }
   } // if( valid_all_p != boost::none )

   if( valid_any_p != boost::none ) {
      evt_proc_elem.valid_all = false;
      evt_proc_elem.valid_any = true;
      evt_proc_elem.error_any = false;
      evt_proc_elem.error_all = false;
      evt_proc_elem.evt       = false;

      BOOST_FOREACH( const boost_pt::ptree::value_type& evt, valid_any_p.get() ) {
         if( !evt.first.empty()) {
            SIMD_REPORT_ERROR( "simd::sys_scalar" ) << "Incorrect format";
         }

         try {
            evt_proc_elem.mod_name = evt.second.get<std::string>( "mod" );
            evt_proc_elem.evt_name = evt.second.get<std::string>( "evt" );
         }
         catch( const boost_pt::ptree_error& err ) {
            SIMD_REPORT_ERROR( "simd::sys_scalar" ) << err.what();
         }
         catch( ... ) {
            SIMD_REPORT_ERROR( "simd::sys_scalar" ) << "Unexpected";
         }

         evt_proc.push_back( evt_proc_elem );

         valid_any_present = true;
      }
   } // if( valid_any_p != boost::none )

   if( error_any_p != boost::none ) {
      evt_proc_elem.valid_all = false;
      evt_proc_elem.valid_any = false;
      evt_proc_elem.error_any = true;
      evt_proc_elem.error_all = false;
      evt_proc_elem.evt       = false;

      BOOST_FOREACH( const boost_pt::ptree::value_type& evt, error_any_p.get() ) {
         if( !evt.first.empty()) {
            SIMD_REPORT_ERROR( "simd::sys_scalar" ) << "Incorrect format";
         }

         try {
            evt_proc_elem.mod_name = evt.second.get<std::string>( "mod" );
            evt_proc_elem.evt_name = evt.second.get<std::string>( "evt" );
         }
         catch( const boost_pt::ptree_error& err ) {
            SIMD_REPORT_ERROR( "simd::sys_scalar" ) << err.what();
         }
         catch( ... ) {
            SIMD_REPORT_ERROR( "simd::sys_scalar" ) << "Unexpected";
         }

         evt_proc.push_back( evt_proc_elem );

         error_any_present = true;
      }
   } // if( error_any_p != boost::none )

   if( error_all_p != boost::none ) {
      evt_proc_elem.valid_all = false;
      evt_proc_elem.valid_any = false;
      evt_proc_elem.error_any = false;
      evt_proc_elem.error_all = true;
      evt_proc_elem.evt       = false;

      BOOST_FOREACH( const boost_pt::ptree::value_type& evt, error_all_p.get() ) {
         if( !evt.first.empty()) {
            SIMD_REPORT_ERROR( "simd::sys_scalar" ) << "Incorrect format";
         }

         try {
            evt_proc_elem.mod_name = evt.second.get<std::string>( "mod" );
            evt_proc_elem.evt_name = evt.second.get<std::string>( "evt" );
         }
         catch( const boost_pt::ptree_error& err ) {
            SIMD_REPORT_ERROR( "simd::sys_scalar" ) << err.what();
         }
         catch( ... ) {
            SIMD_REPORT_ERROR( "simd::sys_scalar" ) << "Unexpected";
         }

         evt_proc.push_back( evt_proc_elem );

         error_all_present = true;
      }
   } // if( error_all_p != boost::none )
} // void simd_sys_scalar_c::evt_proc_init(

} // namespace simd
