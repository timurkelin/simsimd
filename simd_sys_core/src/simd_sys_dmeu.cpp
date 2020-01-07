/*
 * simd_sys_dmeu.cpp
 *
 *  Description:
 *    Methods of the the basic class for data memory (DM) and execution unit (EU) module
 */

#include <stdexcept>
#include <boost/foreach.hpp>
#include "simd_sys_dmeu.h"
#include "simd_conv_ptree.h"
#include "simd_assert.h"
#include "simd_report.h"

namespace simd {

SC_HAS_PROCESS( simd::simd_sys_dmeu_c );
simd_sys_dmeu_c::simd_sys_dmeu_c(
      sc_core::sc_module_name nm )
   : sc_core::sc_module( nm )
   , clock_i( "clock_i" )
   , reset_i( "reset_i" )
   , data_vi( "data_vi" )
   , data_vo( "data_vo" )
   , event_o( "event_o" )
   , busw_i(  "busw_i"  )
   , busr_o(  "busr_o"  )
   , proc_i(  "proc_i"  )
   , state_o( "state_o" )
   , ready_v( "ready_v" ) {

   // Initialisations
   active_config   = UINT_MAX;
   active_status   = UINT_MAX;

   // Internal channels allocation

   // Submodules allocation

   // Connectivity

   // Process registrations
   SC_CTHREAD( proc_thrd, clock_i.pos() ); //  Synchronous thread for data processing
   reset_signal_is( reset_i, true );

   SC_THREAD( ready_thrd );            // Asynchronous thread for ready signal propagation

   // Dump buffers
   dump_buf_event_p  = boost::optional<simd_dump_buf_c<std::string>    &>( *( new simd_dump_buf_c<std::string>    ( std::string( name()) + ".event"  )));
   dump_buf_config_p = boost::optional<simd_dump_buf_c<boost_pt::ptree>&>( *( new simd_dump_buf_c<boost_pt::ptree>( std::string( name()) + ".config" )));
   dump_buf_status_p = boost::optional<simd_dump_buf_c<boost_pt::ptree>&>( *( new simd_dump_buf_c<boost_pt::ptree>( std::string( name()) + ".status" )));
} // simd_sys_dmeu_c::simd_sys_dmeu_c(

void simd_sys_dmeu_c::add_trace_ports(
      sc_core::sc_trace_file* tf,
      const std::string& top_name ) {
   std::string mod_name = top_name + "." + std::string( name());

   for( std::size_t n_i = 0; n_i < data_vi.size(); n_i ++ ) {
      data_vi.at( n_i )->add_trace(
            tf,
            mod_name );
   }

   for( std::size_t n_o = 0; n_o < data_vo.size(); n_o ++ ) {
      data_vo.at( n_o )->add_trace(
            tf,
            mod_name );
   }

   sc_core::sc_trace(
         tf,
         proc_i,
         mod_name + ".proc_i" );

   sc_core::sc_trace(
         tf,
         state_o,
         mod_name + ".state_o" );
} // void simd_sys_dmeu_c::add_trace_ports(

bool simd_sys_dmeu_c::is_conf_empty(
      std::size_t conf_idx ) {
   bool empty = false;

   try {
      empty = config.at( conf_idx ).empty();
   }
   catch( const boost_pt::ptree_error& err ) {
      SIMD_REPORT_ERROR( "simd::sys_xbar" ) << name() << " " <<  err.what();
   }
   catch( const std::exception& err ) {
      SIMD_REPORT_ERROR( "simd::sys_xbar" ) << name() << " " <<  err.what();
   }
   catch( ... ) {
      SIMD_REPORT_ERROR( "simd::sys_xbar" ) << name() << " Unexpected";
   }

   return empty;
} // bool simd_sys_dmeu_c::is_conf_empty(

void simd_sys_dmeu_c::parse_busw_req(
      const simd_sig_ptree_c& req ) {
   std::size_t idx = 0;
   std::string cmd = "";

   try {
      idx = req.get().get<std::size_t>( "idx" );
      cmd = req.get().get<std::string>( "cmd" );
   }
   catch( const boost_pt::ptree_error& err ) {
      SIMD_REPORT_ERROR( "simd::sys_dmeu" ) << name() << " " <<  err.what();
   }
   catch( ... ) {
      SIMD_REPORT_ERROR( "simd::sys_dmeu" ) << name() << " Unexpected";
   }

   // Identify packet type
   if( cmd == "get") {
      // Report status
      try {
         simd_sig_ptree_c stat;
         busr_o->write( stat.set( status.at( idx )));
      }
      catch( const std::exception& err ) {
         SIMD_REPORT_ERROR( "simd::sys_dmeu" ) << name() << " " << err.what();
      }
      catch( ... ) {
         SIMD_REPORT_ERROR( "simd::sys_dmeu" ) << name() << " Unexpected";
      }

      if( idx == active_status && state_o->read().get() != DMEU_ST_IDLE ) {
         SIMD_REPORT_INFO( "simd::sys_dmeu" ) << name() << " Active status reported";
      }

      // Dump the status
      dump_buf_status_p.get().write( status.at( idx ), BUF_WRITE_LAST );
   }
   else if( cmd == "put" ) {
      if( idx == active_config && state_o->read().get() != DMEU_ST_IDLE ) {
         SIMD_REPORT_ERROR( "simd::sys_dmeu" ) << name() << " Request to change active configuration";
      }

      try {
         config.at( idx ) = req.get().get_child( "data" );
      }
      catch( const boost_pt::ptree_error& err ) {
         SIMD_REPORT_ERROR( "simd::sys_dmeu" ) << name() << " " <<  err.what();
      }
      catch( const std::exception& err ) {
         SIMD_REPORT_ERROR( "simd::sys_dmeu" ) << name() << " " <<  err.what();
      }
      catch( ... ) {
         SIMD_REPORT_ERROR( "simd::sys_dmeu" ) << name() << " Unexpected";
      }
   }
   else if( cmd == "run") {
      bool is_exec_idx_found = false;
      std::size_t exec_idx = UINT_MAX;

      for( std::size_t conf_idx = 0; conf_idx < config.size(); conf_idx ++ ) {

         // Check if configuration slot is initialised
         if( is_conf_empty( conf_idx )) {
            continue;
         }

         try {
            exec_idx = config.at( conf_idx ).get<std::size_t>( "exec_idx" );
         }
         catch( const boost_pt::ptree_error& err ) {
            SIMD_REPORT_ERROR( "simd::sys_dmeu" ) << name() << " " <<  err.what();
         }
         catch( const std::exception& err ) {
            SIMD_REPORT_ERROR( "simd::sys_dmeu" ) << name() << " " <<  err.what();
         }
         catch( ... ) {
            SIMD_REPORT_ERROR( "simd::sys_dmeu" ) << name() << " Unexpected";
         }

         if( exec_idx == idx ) {
            if( !is_exec_idx_found ) {
               is_exec_idx_found = true;

               if( state_o->read().get() == DMEU_ST_IDLE ) {
                  // DM/EU is idle. Immediately start the execution
                  active_config = conf_idx;  // Specify active config index
                  active_status = curr_stat_idx( conf_idx );

                  req_run();  // Function-specific configuration
               }
               else if( active_config_p ) {
                  // DM/EU is active. Push config index into fifo for deferred execution
                  if( active_config_p.get().num_free()) {
                     active_config_p.get().write( conf_idx );
                  }
                  else {
                     SIMD_REPORT_ERROR( "simd::sys_dmeu" ) << name() << " Index FIFO is full";
                  }
               }
               else {
                  // DM/EU is active and no fifo is available
                  SIMD_REPORT_ERROR( "simd::sys_dmeu" ) << name() << " The module is already active";
               }
            }
            else {
               SIMD_REPORT_ERROR( "simd::sys_dmeu" ) << name() << " Non-unique execution index";
            }
         }
      }
   }
   else {
      SIMD_REPORT_ERROR( "simd::sys_dmeu" ) << "Unresolved command to " << name();
   }

   // Dump the request
   dump_buf_config_p.get().write( req.get(), BUF_WRITE_LAST );
} // void simd_sys_dmeu_c::parse_busw_req(

bool simd_sys_dmeu_c::next_conf_run(
      std::size_t conf_idx ) {

   std::size_t new_conf_idx = next_conf_idx( conf_idx );

   if( new_conf_idx < config.size()) {
      if( state_o->read().get() != DMEU_ST_DONE &&
          state_o->read().get() != DMEU_ST_IDLE ) {
         SIMD_REPORT_ERROR( "simd::sys_dmeu" ) << name() << " The module is already active";
      }

      active_config = new_conf_idx;   // Specify active config index
      active_status = curr_stat_idx( new_conf_idx );

      req_run();  // Function-specific configuration

      return true;
   }
   else {
      return false;
   }
} // bool simd_sys_dmeu_c::next_conf_run(

bool simd_sys_dmeu_c::is_event_enabled(
      std::size_t  conf_idx,
      const std::string &event_id ) {

   // Check the format of the list of the address generators
   const boost_pt::ptree &events = [&]() {
      try {
         return config.at( conf_idx ).get_child("events");
      }
      catch( const boost_pt::ptree_error& err ) {
         SIMD_REPORT_ERROR( "simd::sys_dmeu" ) << name() << " " << err.what();
      }
      catch( const std::exception& err ) {
         SIMD_REPORT_ERROR( "simd::sys_dmeu" ) << name() << " " << err.what();
      }
      catch( ... ) {
         SIMD_REPORT_ERROR( "simd::sys_dmeu" ) << name() << " Unexpected";
      }

      return config.at( conf_idx ).get_child("events"); // this is unreachable
   }(); // iife

   bool event_ena = false;

   BOOST_FOREACH( const boost_pt::ptree::value_type& v, events ) {
      if( !v.first.empty()) {
         SIMD_REPORT_ERROR( "simd::sys_dmeu" ) << name() << " Incorrect structure";
      }

      try {
         event_ena = ( v.second.get_value<std::string>() == event_id );
      }
      catch( const boost_pt::ptree_error& err ) {
         SIMD_REPORT_ERROR( "simd::sys_dmeu" ) << name() << " " <<  err.what();
      }
      catch( ... ) {
         SIMD_REPORT_ERROR( "simd::sys_dmeu" ) << name() << " Unexpected";
      }

      if( event_ena ) {
         break;
      }
   }

   return event_ena;
} // bool simd_sys_dmeu_c::is_event_enabled(

bool simd_sys_dmeu_c::event(
      const std::string& event_id ) {

   // Dump the event request regardless of the mask
   dump_buf_event_p.get().write( event_id, BUF_WRITE_LAST );

   // Check mask
   if( is_event_enabled( active_config, event_id )) {
      if( event_o->num_free()) {
         boost_pt::ptree  event_pt;
         simd_sig_ptree_c event_out;

         event_pt.put( "event_id", event_id );
         event_o->nb_write( event_out.set( event_pt ));
      }
      else {
         SIMD_REPORT_ERROR( "simd::sys_dmeu" ) << name() << " Event FIFO is full";
      }

      return true;
   }
   else {
      return false;
   }
} // bool simd_sys_dmeu_c::event(

bool simd_sys_dmeu_c::event(
      const boost_pt::ptree& event_pt ) {

   // Dump the event request regardless of the mask
   std::string event_id;

   dump_buf_event_p.get().write(       // Dump as json string
         pt2str( event_pt, event_id ),
         BUF_WRITE_LAST );

   // extract event ID
   try {
      event_id = event_pt.get<std::string>( "event_id" );
   }
   catch( const boost_pt::ptree_error& err ) {
      SIMD_REPORT_ERROR( "simd::sys_dmeu" ) << name() << " " <<  err.what();
   }
   catch( ... ) {
      SIMD_REPORT_ERROR( "simd::sys_dmeu" ) << name() << " Unexpected";
   }

   // Check mask
   if( is_event_enabled( active_config, event_id )) {
      if( event_o->num_free()) {
         simd_sig_ptree_c event_out;

         event_o->nb_write( event_out.set( event_pt ));
      }
      else {
         SIMD_REPORT_ERROR( "simd::sys_dmeu" ) << name() << " Event FIFO is full";
      }

      return true;
   }
   else {
      return false;
   }
} // bool simd_sys_dmeu_c::event(

std::size_t simd_sys_dmeu_c::next_conf_idx(
      std::size_t conf_idx ) {

   std::size_t conf_next = config.size();

   // Check the format of the list of the address generators
   try {
      conf_next = config.at( conf_idx ).get<std::size_t>("conf_next", config.size());
   }
   catch( const boost_pt::ptree_error& err ) {
      SIMD_REPORT_ERROR( "simd::sys_dmeu" ) << name() << " " << err.what();
   }
   catch( const std::exception& err ) {
      SIMD_REPORT_ERROR( "simd::sys_dmeu" ) << name() << " " << err.what();
   }
   catch( ... ) {
      SIMD_REPORT_ERROR( "simd::sys_dmeu" ) << name() << " Unexpected";
   }

   return conf_next;
} // std::size_t simd_sys_dmeu_c::next_conf_idx(

std::size_t simd_sys_dmeu_c::curr_stat_idx(
      std::size_t conf_idx ) {

   std::size_t stat_idx = status.size();

   // Check the format of the list of the address generators
   try {
      stat_idx = config.at( conf_idx ).get<std::size_t>("stat_idx", status.size());
   }
   catch( const boost_pt::ptree_error& err ) {
      SIMD_REPORT_ERROR( "simd::sys_dmeu" ) << name() << " " << err.what();
   }
   catch( const std::exception& err ) {
      SIMD_REPORT_ERROR( "simd::sys_dmeu" ) << name() << " " << err.what();
   }
   catch( ... ) {
      SIMD_REPORT_ERROR( "simd::sys_dmeu" ) << name() << " Unexpected";
   }

   return stat_idx;
} // std::size_t simd_sys_dmeu_c::curr_stat_idx(

} // namespace simd
