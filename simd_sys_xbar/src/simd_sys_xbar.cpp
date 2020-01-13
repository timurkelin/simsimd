/*
 * simd_sys_xbar.cpp
 *
 *  Description:
 *    Methods of the cross-bar switch
 */

#include <boost/foreach.hpp>
#include "simd_sys_xbar.h"
#include "simd_assert.h"
#include "simd_report.h"

namespace simd {

SC_HAS_PROCESS( simd::simd_sys_xbar_c );
simd_sys_xbar_c::simd_sys_xbar_c(
      sc_core::sc_module_name nm )
   : sc_core::sc_module( nm )
   , clock_i(  "clock_i"  )
   , reset_i(  "reset_i"  )
   , data_xvi( "data_xvi" )
   , data_xvo( "data_xvo" )
   , state_vi( "state_vi" )
   , proc_vo(  "proc_vo"  )
   , busw_i(   "busw_i"   )
   , event_o(  "event_o"  )
   , data_chn_src_v("data_chn_src_v")
   , data_chn_dst_v("data_chn_dst_v")
   , eb_src_v("eb_src_v")
   , eb_dst_v("eb_dst_v")
   , sif_eb_data_v( "sif_eb_data_v" )
   , sif_eb_valid_v("sif_eb_valid_v")
   , sif_eb_ready_v("sif_eb_ready_v")
   , src_data_v(  "src_data_v"  )
   , src_valid_v( "src_valid_v" )
   , src_ready_v( "src_ready_v" )
   , src_tail_v(  "src_tail_v"  )
   , dst_data_v(  "dst_data_v"  )
   , dst_valid_v( "dst_valid_v" )
   , dst_ready_v( "dst_ready_v" )
   , dst_tail_v(  "dst_tail_v"  )
   , mod_proc_v(  "mod_proc_v"  )
   , eb_dif_data_v( "eb_dif_data_v")
   , eb_dif_valid_v("eb_dif_valid_v")
   , eb_dif_ready_v("eb_dif_ready_v") {

   // Process registrations
   SC_THREAD( xbar_thrd ); // Switching thread (async)

   SC_CTHREAD( conf_thrd, clock_i.pos() ); //  Configuration thread (clocked)
   reset_signal_is( reset_i, true );
}

void simd_sys_xbar_c::init(
      boost::optional<const boost_pt::ptree&> _pref_p,
      const std::vector<simd_sys_dmeu_info_t>& dmeu_info ) {
   const boost_pt::ptree& pref = _pref_p.get();
   pref_p = _pref_p;

   // Count the number of the inputs and outputs of the xbar
   std::size_t n_dst_tot = 0;
   std::size_t n_src_tot = 0;

   BOOST_FOREACH( const simd_sys_dmeu_info_t& v, dmeu_info ) {
      n_dst_tot += v.xbar_dmeu.size();
      n_src_tot += v.dmeu_xbar.size();
   }

   // Set port and channel sizes
   data_xvi.init( n_dst_tot );
   data_chn_dst_v.init( n_dst_tot );
   eb_dst_v.init( n_dst_tot );

   dst_data_v.init(  n_dst_tot );
   dst_valid_v.init( n_dst_tot );
   dst_ready_v.init( n_dst_tot );
   dst_tail_v.init(  n_dst_tot );

   eb_dif_data_v.init(  n_dst_tot );
   eb_dif_valid_v.init( n_dst_tot );
   eb_dif_ready_v.init( n_dst_tot );

   dst_port_attr.resize( n_dst_tot );
   dst_port_conn.resize( n_dst_tot );

   data_xvo.init( n_src_tot );
   data_chn_src_v.init( n_src_tot );
   eb_src_v.init( n_src_tot );

   src_data_v.init(  n_src_tot );
   src_valid_v.init( n_src_tot );
   src_ready_v.init( n_src_tot );
   src_tail_v.init(  n_src_tot );

   sif_eb_data_v.init(  n_src_tot );
   sif_eb_valid_v.init( n_src_tot );
   sif_eb_ready_v.init( n_src_tot );

   src_port_attr.resize( n_src_tot );
   src_port_conn.resize( n_src_tot );

   // Initialise I/O ports which are dimensioned with dm/eu module count
   state_vi.init(  dmeu_info.size());
   proc_vo.init(   dmeu_info.size());
   mod_proc_v.init(dmeu_info.size());
   proc_rb.resize( dmeu_info.size());

   // Bind interface exports to the corresponding channels
   for( std::size_t n_dst = 0; n_dst < data_chn_dst_v.size(); n_dst ++ ) {
      // Connect interface exports to channels
      data_xvi.at( n_dst ).bind(
            data_chn_dst_v.at( n_dst ));

      eb_dst_v.at( n_dst ).init();
      eb_dst_v.at( n_dst ).clock_i.bind( clock_i );
      eb_dst_v.at( n_dst ).reset_i.bind( reset_i );

      // Connect to elastic buffer
      eb_dst_v.at( n_dst ).data_i.bind(  dst_data_v.at(  n_dst ));
      eb_dst_v.at( n_dst ).valid_i.bind( dst_valid_v.at( n_dst ));
      eb_dst_v.at( n_dst ).ready_o.bind( dst_ready_v.at( n_dst ));

      eb_dst_v.at( n_dst ).data_o.bind(  eb_dif_data_v.at(  n_dst ));
      eb_dst_v.at( n_dst ).valid_o.bind( eb_dif_valid_v.at( n_dst ));
      eb_dst_v.at( n_dst ).ready_i.bind( eb_dif_ready_v.at( n_dst ));

      eb_dst_v.at( n_dst ).tail_o.bind(  dst_tail_v.at( n_dst ));

      // Connect to channel ports
      data_chn_dst_v.at( n_dst ).data_i.bind(  eb_dif_data_v.at(  n_dst ));
      data_chn_dst_v.at( n_dst ).valid_i.bind( eb_dif_valid_v.at( n_dst ));
      data_chn_dst_v.at( n_dst ).ready_o.bind( eb_dif_ready_v.at( n_dst ));
   }

   for( std::size_t n_src = 0; n_src < data_chn_src_v.size(); n_src ++ ) {
      // Connect interface exports to channels
      data_xvo.at( n_src ).bind(
            data_chn_src_v.at( n_src ));

      eb_src_v.at( n_src ).init();
      eb_src_v.at( n_src ).clock_i.bind( clock_i );
      eb_src_v.at( n_src ).reset_i.bind( reset_i );

      // Connect to elastic buffer
      eb_src_v.at( n_src ).data_o.bind(  src_data_v.at(  n_src ));
      eb_src_v.at( n_src ).valid_o.bind( src_valid_v.at( n_src ));
      eb_src_v.at( n_src ).ready_i.bind( src_ready_v.at( n_src ));

      eb_src_v.at( n_src ).data_i.bind(  sif_eb_data_v.at(  n_src ));
      eb_src_v.at( n_src ).valid_i.bind( sif_eb_valid_v.at( n_src ));
      eb_src_v.at( n_src ).ready_o.bind( sif_eb_ready_v.at( n_src ));

      eb_src_v.at( n_src ).tail_o.bind(  src_tail_v.at( n_src ));

      // Connect to channel ports
      data_chn_src_v.at( n_src ).data_o.bind(  sif_eb_data_v.at(  n_src ));
      data_chn_src_v.at( n_src ).valid_o.bind( sif_eb_valid_v.at( n_src ));
      data_chn_src_v.at( n_src ).ready_i.bind( sif_eb_ready_v.at( n_src ));
   }

   // Initialise switching matrix (ports are disconnected and inactive at the start)
   map_src_to_dst.resize(
         data_chn_src_v.size(),
         xbar_dst_vec_c( data_chn_dst_v.size()));

   // Initialise static information for XBAR ports: data rearrangement from dmeu_info
   for( std::size_t n_mod = 0; n_mod < dmeu_info.size(); n_mod ++ ) {
      for( std::size_t n_src = 0; n_src < dmeu_info.at( n_mod ).dmeu_xbar.size(); n_src ++ ) {
         // Connect to corresponding valid-ready elastic buffers
         eb_src_v.at( dmeu_info.at( n_mod ).dmeu_xbar.at( n_src )).proc_i.bind( mod_proc_v.at( n_mod ));

         try {
            std::size_t xbar_src_port = dmeu_info.at( n_mod ).dmeu_xbar.at( n_src );
            src_port_attr.at( xbar_src_port ).mod_port   = n_src; // src port number in dmeu
            src_port_attr.at( xbar_src_port ).mod_info_p =
                  boost::optional<const simd_sys_dmeu_info_t&>( dmeu_info.at( n_mod )); // Create pointer to corresponding dmeu info
         }
         catch( const std::exception& err ) {
            SIMD_REPORT_ERROR( "simd::sys_xbar" ) << name() << " " <<  err.what();
         }
         catch( ... ) {
            SIMD_REPORT_ERROR( "simd::sys_xbar" ) << name() << " Unexpected";
         }
      }

      for( std::size_t n_dst = 0; n_dst < dmeu_info.at( n_mod ).xbar_dmeu.size(); n_dst ++ ) {
         // Connect to corresponding valid-ready elastic buffers
         eb_dst_v.at( dmeu_info.at( n_mod ).xbar_dmeu.at( n_dst )).proc_i.bind( mod_proc_v.at( n_mod ));

         try {
            std::size_t xbar_dst_port = dmeu_info.at( n_mod ).xbar_dmeu.at( n_dst );
            dst_port_attr.at( xbar_dst_port ).mod_port   = n_dst; // dst port number in dmeu
            dst_port_attr.at( xbar_dst_port ).mod_info_p =
                  boost::optional<const simd_sys_dmeu_info_t&>( dmeu_info.at( n_mod )); // Create pointer to corresponding dmeu info
         }
         catch( const std::exception& err ) {
            SIMD_REPORT_ERROR( "simd::sys_xbar" ) << name() << " " <<  err.what();
         }
         catch( ... ) {
            SIMD_REPORT_ERROR( "simd::sys_xbar" ) << name() << " Unexpected";
         }
      }
   }

   // Set the sizes of the config and status vectors
   try {
      std::size_t conf_size = pref.get<std::size_t>( "config_slots" );

      if( conf_size > 0 ) {
         config.resize( conf_size );
         config_state.resize( conf_size, ST_CONF_IDLE );
      }
      else {
         SIMD_REPORT_ERROR( "simd::sys_xbar" ) << name() << " Incorrect configuration size";
      }
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

   // Save pointer to the vector of dm/eu parameters
   dmeu_info_p = boost::optional<const std::vector<simd_sys_dmeu_info_t>&>( dmeu_info );
}

// Trace internal signals
void simd_sys_xbar_c::add_trace(
      sc_core::sc_trace_file* tf,
      const std::string& top_name ) {
   std::string mod_name = top_name + "." + std::string( name());

   for( std::size_t n_dst = 0; n_dst < data_chn_dst_v.size(); n_dst ++ ) {
      std::string idx_name = "(" + std::to_string( n_dst ) + ")";

      sc_core::sc_trace( tf, dst_data_v.at(  n_dst ), mod_name + ".dst_data_v"  + idx_name );
      sc_core::sc_trace( tf, dst_valid_v.at( n_dst ), mod_name + ".dst_valid_v" + idx_name );
      sc_core::sc_trace( tf, dst_ready_v.at( n_dst ), mod_name + ".dst_ready_v" + idx_name );

      eb_dst_v.at( n_dst ).add_trace( tf, mod_name + ".eb_dst_v" + idx_name );
   }

   for( std::size_t n_src = 0; n_src < data_chn_src_v.size(); n_src ++ ) {
      std::string idx_name = "(" + std::to_string( n_src ) + ")";

      sc_core::sc_trace( tf, src_data_v.at(  n_src ), mod_name + ".src_data_v"  + idx_name );
      sc_core::sc_trace( tf, src_valid_v.at( n_src ), mod_name + ".src_valid_v" + idx_name );
      sc_core::sc_trace( tf, src_ready_v.at( n_src ), mod_name + ".src_ready_v" + idx_name );

      eb_src_v.at( n_src ).add_trace( tf, mod_name + ".eb_src_v" + idx_name );
   }
}

// Convert DM/EU name and output port into XBAR src port
std::size_t simd_sys_xbar_c::src_dmeu2xbar(
      const std::string& mod_name,
      const std::size_t  mod_port ) {
   std::size_t src_port;
   std::size_t mod_name_hash = 0;

   boost::hash_combine( mod_name_hash, mod_name );

   for( src_port = 0; src_port < src_port_attr.size(); src_port ++ ) {
      if( src_port_attr.at( src_port ).mod_info_p.get().hash == mod_name_hash &&
          src_port_attr.at( src_port ).mod_port              == mod_port ) {
         break;
      }
   }

   if( src_port >= src_port_attr.size() ) {
      SIMD_REPORT_ERROR( "simd::sys_xbar" ) << name() << " Source not found";
   }

   return src_port;
}

  // Convert DM/EU name and input port into XBAR dst port
std::size_t simd_sys_xbar_c::dst_dmeu2xbar(
      const std::string &mod_name,
      const std::size_t  mod_port ) {
   std::size_t dst_port;
   std::size_t mod_name_hash = 0;

   boost::hash_combine( mod_name_hash, mod_name );

   for( dst_port = 0; dst_port < dst_port_attr.size(); dst_port ++ ) {
      if( dst_port_attr.at( dst_port ).mod_info_p.get().hash == mod_name_hash &&
          dst_port_attr.at( dst_port ).mod_port              == mod_port ) {
         break;
      }
   }

   if( dst_port >= dst_port_attr.size() ) {
      SIMD_REPORT_ERROR( "simd::sys_xbar" ) << name() << " Destination not found: " << mod_name << "." << std::to_string( mod_port );
   }

   return dst_port;
}

// Check if the configuration slot is empty
bool simd_sys_xbar_c::is_conf_empty(
      const std::size_t conf_idx ) {
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
}

// Generate event from XBAR
void simd_sys_xbar_c::event(
      const std::string& mod_name,
      const std::string& event_id ) {

   if( event_o->num_free()) {
      boost_pt::ptree  event_pt;
      simd_sig_ptree_c event_out;

      event_pt.put( "event_id", mod_name + "." + event_id );
      event_o->nb_write( event_out.set( event_pt ));
   }
   else {
      SIMD_REPORT_ERROR( "simd::sys_dmeu" ) << name() << " Event FIFO is full";
   }
}

// Parse request from the busw
void simd_sys_xbar_c::parse_busw_req(
      const simd_sig_ptree_c& req ) {
   std::size_t idx = 0;
   std::string  cmd = "";

   try {
      idx = req.get().get<std::size_t>( "idx" );
      cmd = req.get().get<std::string>( "cmd" );
   }
   catch( const boost_pt::ptree_error& err ) {
      SIMD_REPORT_ERROR( "simd::sys_xbar" ) << name() << " " <<  err.what();
   }
   catch( ... ) {
      SIMD_REPORT_ERROR( "simd::sys_xbar" ) << name() << " Unexpected";
   }

   // Identify packet type
   if( cmd == "put" ) { // Update configuration
      if( !is_conf_empty( idx ) && config_state.at( idx ) != ST_CONF_IDLE ) {
         SIMD_REPORT_ERROR( "simd::sys_xbar" ) << name() << " Request to change active connection or active configuration";
      }

      try {
         config.at( idx ) = req.get().get_child( "data" );
      }
      catch( const boost_pt::ptree_error& err ) {
         SIMD_REPORT_ERROR( "simd::sys_xbar" ) << name() << " " <<  err.what();
      }
      catch( ... ) {
         SIMD_REPORT_ERROR( "simd::sys_xbar" ) << name() << " Unexpected";
      }
   }
   else if( cmd == "run") { // Execute
      for( std::size_t conf_idx = 0; conf_idx < config.size(); conf_idx ++ ) {
         std::size_t exec_idx = UINT_MAX;

         try {
            if( !is_conf_empty( conf_idx )) {
               exec_idx = config.at( conf_idx ).get<std::size_t>( "exec_idx" );
            }
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

         if( exec_idx == idx ) {
            if( config_state.at( conf_idx ) == ST_CONF_IDLE ) {
               xbar_set( conf_idx,
                     is_active( conf_idx ) ? ST_CONF_WAIT : ST_CONF_ACTIVE );
            }
            else {
               SIMD_REPORT_ERROR( "simd::sys_xbar" ) << name() << " Request to run awaiting configuration";
            }

            evt_conf.notify(); // Notify XBAR async. thread that the configuration was changed
         }
      }
   }
   else {
      SIMD_REPORT_ERROR( "simd::sys_xbar" ) << "Unresolved command to " << name();
   }
}

// Request to update the state of the XBAR from the configuration slot
void simd_sys_xbar_c::xbar_set(
      const std::size_t  conf_idx,
      const conf_state_t state ) {

   is_conf_empty( conf_idx ); // Check that the index is valid

   bool conn = false;

   if( config_state.at( conf_idx ) != state ) {
      config_state.at( conf_idx ) = state; // Save the config state

      if( state == ST_CONF_WAIT || state == ST_CONF_WAIT_NEXT ) {
         return;
      }

      conn = ( state == ST_CONF_ACTIVE );
   }
   else {
      SIMD_REPORT_ERROR( "simd::sys_xbar" ) << name() << " Duplicate routing configuration";
   }

   const boost_pt::ptree &src_list = [&]() {
      try {
         return config.at( conf_idx ).get_child("routing");
      }
      catch( const std::exception& err ) {
         SIMD_REPORT_ERROR( "simd::sys_xbar" ) << name() << " " <<  err.what();
      }
      catch( ... ) {
         SIMD_REPORT_ERROR( "simd::sys_xbar" ) << name() << " Unexpected";
      }

      return config.at( conf_idx ).get_child("routing"); // This is unreachable
   }(); // iife

   BOOST_FOREACH( const boost_pt::ptree::value_type& src_cfg, src_list ) {
      if( !src_cfg.first.empty()) {
         SIMD_REPORT_ERROR( "simd::sys_xbar" ) << name() << " Incorrect config structure";
      }

      bool        event = false;
      std::size_t src_port = 0, src_mod = 0;
      std::size_t dst_port = 0, dst_mod = 0;
      std::size_t dst_master = UINT_MAX;
      bool        is_dst_master = false;

      simd_dmeu_state_t src_mod_state = DMEU_ST_IDLE;
      simd_dmeu_state_t dst_mod_state = DMEU_ST_IDLE;

      try {
         event = src_cfg.second.get<bool>( "event" );

         // get src port from the XBAR perspective
         std::string src_dmeu_mod  = src_cfg.second.get<std::string>( "mod"  );
         std::size_t src_dmeu_port = src_cfg.second.get<std::size_t>( "port" );

         src_port = src_dmeu2xbar( src_dmeu_mod, src_dmeu_port );

         // Get actual state of the src module
         src_mod = src_port_attr.at( src_port ).mod_info_p.get().idx;

         src_mod_state = state_vi.at( src_mod )->read().get();
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

      // Check if the state of the source module allows for XBAR switching
      if( !( src_mod_state == DMEU_ST_IDLE ||
            ( src_mod_state == DMEU_ST_DONE && proc_rb.at( src_mod ) == false ))) {
         SIMD_REPORT_ERROR( "simd::sys_xbar" ) << name() << " Unable to change routing for active source DM/EU";
      }

      // Set the connection state of the src port
      src_port_conn.at( src_port ) = conn;

      const boost_pt::ptree &dst_list = [&]() {
         try {
            return src_cfg.second.get_child("dst");
         }
         catch( const std::exception& err ) {
            SIMD_REPORT_ERROR( "simd::sys_xbar" ) << name() << " " <<  err.what();
         }
         catch( ... ) {
            SIMD_REPORT_ERROR( "simd::sys_xbar" ) << name() << " Unexpected";
         }

         return src_cfg.second.get_child("dst"); // This is unreachable
      }(); // iife

      BOOST_FOREACH( const boost_pt::ptree::value_type& dst_cfg, dst_list ) {
         if( !dst_cfg.first.empty()) {
            SIMD_REPORT_ERROR( "simd::sys_xbar" ) << name() << " Incorrect config structure";
         }

         try {
            // dst port from the XBAR perspective
            std::string dst_dmeu_mod  = dst_cfg.second.get<std::string>( "mod"  );
            std::size_t dst_dmeu_port = dst_cfg.second.get<std::size_t>( "port" );

            dst_port = dst_dmeu2xbar( dst_dmeu_mod, dst_dmeu_port );

            // Get actual state of the dst module
            dst_mod = dst_port_attr.at( dst_port ).mod_info_p.get().idx;
            dst_mod_state = state_vi.at( dst_mod )->read().get();

            is_dst_master = dst_cfg.second.get<bool>( "master" );
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

         // Single DST master
         if(( dst_master != UINT_MAX ) && is_dst_master ) {
            SIMD_REPORT_ERROR( "simd::sys_xbar" ) << name() << " Second destination master";
         }

         // Check if the state of the destination module allows for XBAR switching
         if( !( dst_mod_state == DMEU_ST_IDLE ||
               ( dst_mod_state == DMEU_ST_DONE && proc_rb.at( dst_mod ) == false ))) {
            SIMD_REPORT_ERROR( "simd::sys_xbar" ) << name() << " Unable to change routing for active destination DM/EU";
         }

         // Set the connection state of the destination port
         dst_port_conn.at( dst_port ) = conn;

         // Set the XBAR connection
         if( conn ) {
            map_src_to_dst.at( src_port ).at( dst_port ) = true;

            if( is_dst_master ) {
               dst_master = dst_port;
            }
         }
         else {
            map_src_to_dst.at( src_port ).at( dst_port ) = false;
         }
      }

      map_src_to_dst.at( src_port ).dst_master = dst_master;
      map_src_to_dst.at( src_port ).event      = conn && event;
   }

   return;
}

// Check if there is any active module or port for the specified configuration slot
bool simd_sys_xbar_c::is_active(
      const std::size_t conf_idx ) {

   bool is_conn_active = false;
   bool is_dmeu_active = false;

   is_conf_empty( conf_idx );

   const boost_pt::ptree &src_list = [&]() {
      try {
         return config.at( conf_idx ).get_child("routing");
      }
      catch( const std::exception& err ) {
         SIMD_REPORT_ERROR( "simd::sys_xbar" ) << name() << " " <<  err.what();
      }
      catch( ... ) {
         SIMD_REPORT_ERROR( "simd::sys_xbar" ) << name() << " Unexpected";
      }

      return config.at( conf_idx ).get_child("routing"); // This is unreachable
   }(); // iife

   BOOST_FOREACH( const boost_pt::ptree::value_type& src_cfg, src_list ) {
      if( !src_cfg.first.empty()) {
         SIMD_REPORT_ERROR( "simd::sys_xbar" ) << name() << " Incorrect config structure";
      }

      if( is_dmeu_active || is_conn_active ) {
         break;
      }

      std::size_t src_port = 0, src_mod = 0;
      std::size_t dst_port = 0, dst_mod = 0;
      simd_dmeu_state_t src_mod_state = DMEU_ST_IDLE;
      simd_dmeu_state_t dst_mod_state = DMEU_ST_IDLE;

      try {
         // src port from the XBAR perspective
         std::string src_dmeu_mod  = src_cfg.second.get<std::string>( "mod"  );
         std::size_t src_dmeu_port = src_cfg.second.get<std::size_t>( "port" );

         src_port = src_dmeu2xbar( src_dmeu_mod, src_dmeu_port );

         // Get actual state of the src module
         src_mod = src_port_attr.at( src_port ).mod_info_p.get().idx;
         src_mod_state = state_vi.at( src_mod )->read().get();
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

      is_dmeu_active = is_dmeu_active || !( src_mod_state == DMEU_ST_IDLE ||
            ( src_mod_state == DMEU_ST_DONE && proc_rb.at( src_mod ) == false ));
      is_conn_active = is_conn_active || src_port_conn.at( src_port );

      const boost_pt::ptree &dst_list = [&]() {
         try {
            return src_cfg.second.get_child("dst");
         }
         catch( const std::exception& err ) {
            SIMD_REPORT_ERROR( "simd::sys_xbar" ) << name() << " " <<  err.what();
         }
         catch( ... ) {
            SIMD_REPORT_ERROR( "simd::sys_xbar" ) << name() << " Unexpected";
         }

         return src_cfg.second.get_child("dst"); // This is unreachable
      }(); // iife

      BOOST_FOREACH( const boost_pt::ptree::value_type& dst_cfg, dst_list ) {
         if( !dst_cfg.first.empty()) {
            SIMD_REPORT_ERROR( "simd::sys_xbar" ) << name() << " Incorrect config structure";
         }

         if( is_dmeu_active || is_conn_active ) {
            break;
         }

         try {
            // dst port from the XBAR perspective
            std::string dst_dmeu_mod  = dst_cfg.second.get<std::string>( "mod"  );
            std::size_t dst_dmeu_port = dst_cfg.second.get<std::size_t>( "port" );

            dst_port = dst_dmeu2xbar( dst_dmeu_mod, dst_dmeu_port );

            // Get actual state of the dst module
            dst_mod = dst_port_attr.at( dst_port ).mod_info_p.get().idx;
            dst_mod_state = state_vi.at( dst_mod )->read().get();
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

         is_dmeu_active = is_dmeu_active || !( dst_mod_state == DMEU_ST_IDLE ||
               ( dst_mod_state == DMEU_ST_DONE && proc_rb.at( dst_mod ) == false ));
         is_conn_active = is_conn_active || dst_port_conn.at( dst_port );
      }
   }

   return ( is_dmeu_active || is_conn_active );
}

// Get the next configuration slot in the execution chain
std::size_t simd_sys_xbar_c::next_conf_idx(
      const std::size_t conf_idx ) {

   std::size_t conf_next = UINT_MAX;

   // Check the format of the list of the address generators
   try {
      conf_next = config.at( conf_idx ).get<std::size_t>("conf_next", config.size());
   }
   catch( const boost_pt::ptree_error& err ) {
      SIMD_REPORT_ERROR( "simd::sys_xbar" ) << name() << " " << err.what();
   }
   catch( const std::exception& err ) {
      SIMD_REPORT_ERROR( "simd::sys_xbar" ) << name() << " " << err.what();
   }
   catch( ... ) {
      SIMD_REPORT_ERROR( "simd::sys_xbar" ) << name() << " Unexpected";
   }

   return conf_next;
}

// Asynchronous routing thread
void simd_sys_xbar_c::xbar_thrd(
      void ) {

   simd_dmeu_data_c      data_zero;
   simd_sig_dmeu_valid_c src_valid, src_valid_idle;
   simd_sig_dmeu_data_c  src_data,  src_data_zero;
   simd_sig_dmeu_ready_c dst_ready;

   simd_sig_dmeu_ready_c dst_master_ready;
   std::size_t           dst_master_idx;

   src_valid_idle.set( SIMD_IDLE );

   for( std::size_t idx = 0; idx < data_zero.dim; idx ++ ) {
      data_zero[idx].smp = simd_dmeu_smp_t( 0.0, 0.0 );
      data_zero[idx].ena = false;
   }

   src_data_zero.set( data_zero );

   sc_core::wait(sc_core::SC_ZERO_TIME);

   for(;;) {
      // Create combined event
      sc_core::sc_event_or_list evt_or_list;

      // event on valid/data signal from source DM/EUs
      for( std::size_t n_src = 0; n_src < data_chn_src_v.size(); n_src ++ ) {
         evt_or_list |= src_data_v.at(  n_src ).value_changed_event();
         evt_or_list |= src_valid_v.at( n_src ).value_changed_event();
      }

      // event on ready signal from destination DM/EUs
      for( std::size_t n_dst = 0; n_dst < data_chn_dst_v.size(); n_dst ++ ) {
         evt_or_list |= dst_ready_v.at( n_dst ).value_changed_event();
      }

      // event on config change
      evt_or_list |= evt_conf;

      sc_core::wait( evt_or_list ); // Wait for the combined event to trigger

      // Processing of the connected XBAR ports
      for( std::size_t n_src = 0; n_src < data_chn_src_v.size(); n_src ++ ) {
         // Get valid and data from the channel
         src_valid = src_valid_v.at( n_src ).read();
         src_data  = src_data_v.at(  n_src ).read();

         // Is there a connection of the src port to any dst port
         if( is_map_to_src( n_src )) {
            dst_master_idx = map_src_to_dst.at( n_src ).dst_master;

            try {
               dst_master_ready = dst_ready_v.at( dst_master_idx ).read();
            }
            catch( const std::exception& err ) {
               SIMD_REPORT_ERROR( "simd::sys_xbar" ) << name() << " " << err.what();
            }
            catch( ... ) {
               SIMD_REPORT_ERROR( "simd::sys_xbar" ) << name() << " Unexpected";
            }

            // Connect master READY to the source
            src_ready_v.at( n_src ).write( dst_master_ready );

            // Connect VALID and DATA signals from the source to the destinations
            for( std::size_t n_dst = 0; n_dst < data_chn_dst_v.size(); n_dst ++ ) {
               if( map_src_to_dst.at( n_src ).at( n_dst )) {
                  if( n_dst == dst_master_idx ) {
                     // VALID signal goes unchanged to the master dst
                     dst_valid_v.at( n_dst ).write( src_valid );
                  }
                  else {
                     // VALID signal is enabled with the master READY for non-master destinations
                     dst_valid_v.at( n_dst ).write( dst_master_ready.get() ? src_valid : src_valid_idle );
                  }

                  // Deliver data to all the destinations
                  dst_data_v.at(  n_dst ).write( src_data  );
               }
            }
         }
      }

      // Processing of the unconnected XBAR ports
      for( std::size_t n_src = 0; n_src < src_ready_v.size(); n_src ++ ) {
         if( !is_map_to_src( n_src )) {
            src_ready_v.at( n_src ).write( dst_ready.set( false )); // READY = 0
         }
      }

      for( std::size_t n_dst = 0; n_dst < dst_valid_v.size(); n_dst ++ ) {
         if( !is_map_to_dst( n_dst )) {
            dst_valid_v.at( n_dst ).write( src_valid_idle ); // VALID = IDLE;
            dst_data_v.at(  n_dst ).write( src_data_zero  ); // DATA = 0
         }
      }
   }
}

// Clocked Configuration and control thread
void simd_sys_xbar_c::conf_thrd(
      void ) {

   std::vector<bool> proc_done( proc_vo.size());
   std::vector<bool> all_tail_det( dmeu_info_p.get().size());
   std::vector<bool> is_connected( dmeu_info_p.get().size());

   sc_core::wait();

   for(;;) {
      sc_core::wait();

      bool new_routing = false;

      // Identify DM/EU modules which have finished their jobs
      BOOST_FOREACH( const simd_sys_dmeu_info_t& mod, dmeu_info_p.get() ) {
         // Populate readback array for proc flags
         proc_rb.at( mod.idx ) = proc_vo.at( mod.idx )->read();

         proc_done.at( mod.idx ) = false;
         all_tail_det.at( mod.idx ) = false;

         if( state_vi.at( mod.idx )->read().get() == DMEU_ST_DONE ) {
            all_tail_det.at( mod.idx ) = true;
            is_connected.at( mod.idx ) = false;

            BOOST_FOREACH( const std::size_t &src_port, mod.dmeu_xbar ) {
               if(( !all_tail_det.at( mod.idx )) ||     // Check if the tail is not detected
                     ( !is_map_to_src( src_port ))) {   // Check if there is data routing from source port (port can be not connected for DM)
                  break;
               }

               // Check if TAIL is detected the output of the source elastic buffer
               all_tail_det.at( mod.idx ) = all_tail_det.at( mod.idx ) & src_tail_v.at( src_port );

               for( std::size_t dst_port = 0; dst_port < map_src_to_dst.at( src_port ).size(); dst_port ++ ) {
                  if( !all_tail_det.at( mod.idx ) ) {
                     break;
                  }

                  // Check if TAIL is detected at all outputs of the destination elastic buffers which are connected to the source
                  if( map_src_to_dst.at( src_port ).at( dst_port )) {
                     all_tail_det.at( mod.idx ) = all_tail_det.at( mod.idx ) & dst_tail_v.at( dst_port );
                     is_connected.at( mod.idx ) = true;
                  }
               }
            }

            // Check destination ports of the DM/EU module
            BOOST_FOREACH( const std::size_t &dst_port, mod.xbar_dmeu ) {
               if( !all_tail_det.at( mod.idx )) {
                  break;
               }

               if( dst_port_conn.at( dst_port )) {
                  all_tail_det.at( mod.idx ) = all_tail_det.at( mod.idx ) & dst_tail_v.at( dst_port );
                  is_connected.at( mod.idx ) = true;
               }
            }
         }
      }

      // Disconnect ports of the DM/EU modules which have finished their jobs
      BOOST_FOREACH( const simd_sys_dmeu_info_t& mod, dmeu_info_p.get() ) {
         if( all_tail_det.at( mod.idx ) && is_connected.at( mod.idx )) {
            bool done_event = false;

            proc_done.at( mod.idx ) = true;

            // Disconnect source ports of the DM/EU module
            BOOST_FOREACH( const std::size_t &src_port, mod.dmeu_xbar ) {
               src_port_conn.at( src_port ) = false;

               for( std::size_t dst_port = 0; dst_port < map_src_to_dst.at( src_port ).size(); dst_port ++ ) {
                  map_src_to_dst.at( src_port ).at( dst_port ) = false;
               }

               map_src_to_dst.at( src_port ).dst_master = UINT_MAX;

               done_event |= map_src_to_dst.at( src_port ).event;
            }

            // Generate done event
            if( done_event ) {
               event( mod.name, "complete" );
            }

            // Disconnect destination ports of the DM/EU module
            BOOST_FOREACH( const std::size_t &dst_port, mod.xbar_dmeu ) {
               dst_port_conn.at( dst_port ) = false;

               for( std::size_t src_port = 0; src_port < map_src_to_dst.size(); src_port ++ ) {
                  map_src_to_dst.at( src_port ).at( dst_port ) = false;
               }
            }

            new_routing = true;
         }
      }

      // Switch configurations
      for( std::size_t n_conf = 0; n_conf < config_state.size(); n_conf ++ ) {
         if( config_state.at( n_conf ) == ST_CONF_ACTIVE && !is_active( n_conf )) {
            // Configuration is still marked as active but no active ports are connected
            xbar_set( n_conf, ST_CONF_IDLE );

            // Get next configuration
            std::size_t n_conf_next = next_conf_idx( n_conf );

            if( n_conf_next < config_state.size()) {
               if( is_active( n_conf_next )) {
                  // Next configuration is still busy. Mark the current configuration slot as waiting for the next to become free.
                  xbar_set( n_conf, ST_CONF_WAIT_NEXT );
               }
               else {
                  // Next configuration is free to run. Make it active
                  xbar_set( n_conf_next, ST_CONF_ACTIVE );
                  new_routing = true;
               }
            }
         }
      }

      // Process configuration slots which were marked as waiting
      for( std::size_t n_conf = 0; n_conf < config_state.size(); n_conf ++ ) {
         if( config_state.at( n_conf ) == ST_CONF_WAIT_NEXT ) {
            // current configuration slot is waiting for the next to become free. (Chained execution)
            std::size_t n_conf_next = next_conf_idx( n_conf );

            if( n_conf_next < config_state.size()) {
               if( !is_active( n_conf_next )) {
                  config_state.at( n_conf ) = ST_CONF_IDLE; // Mark current slot as IDLE
                  xbar_set( n_conf_next, ST_CONF_ACTIVE );  // Mark next slot as ACTIVE
                  new_routing = true;
               }
            }
            else {
               SIMD_REPORT_ERROR( "simd::sys_xbar" ) << name() << " Incorrect target config";
            }
         }
         else if( config_state.at( n_conf ) == ST_CONF_WAIT ) {
            // current configuration slot is waiting to be executed. (Execution is enabled from scalar core)
            if( !is_active( n_conf )) {
               xbar_set( n_conf, ST_CONF_ACTIVE ); // Mark current slot as ACTIVE
               new_routing = true;
            }
         }
      }

      // Generate enable signal for DM/EU if there is routing for any of its ports
      for( std::size_t n_mod = 0; n_mod < dmeu_info_p.get().size(); n_mod ++ ) {
         bool conn = false;

         BOOST_FOREACH( const std::size_t &src_port, dmeu_info_p.get().at( n_mod ).dmeu_xbar ) {
            conn |= src_port_conn.at( src_port );

            if( conn ) break;
         }

         BOOST_FOREACH( const std::size_t &dst_port, dmeu_info_p.get().at( n_mod ).xbar_dmeu ) {
            conn |= dst_port_conn.at( dst_port );

            if( conn ) break;
         }

         bool mod_proc = proc_done.at( n_mod ) ? false : conn;
         proc_vo.at( n_mod )->write(   mod_proc );
         mod_proc_v.at( n_mod ).write( mod_proc );
      }

      // Notify XBAR routing thread on the change of the routing
      if( new_routing ) {
         evt_conf.notify();
      }

      // Parse request on busw (1 per clock cycle)
      if( busw_i->num_available()) {
         simd_sig_ptree_c req = busw_i->read();

         parse_busw_req( req );
      }
   }
}

// Check if there is at least a single connection for the given dst port
bool simd_sys_xbar_c::is_map_to_dst(
      const std::size_t dst_port ) {
   bool conn = false;

   for( std::size_t src_port = 0; src_port < map_src_to_dst.size(); src_port ++ ) {
      try {
         conn |= map_src_to_dst.at( src_port ).at( dst_port );
      }
      catch( const std::exception& err ) {
         SIMD_REPORT_ERROR( "simd::sys_xbar" ) << name() << " " << err.what();
      }
      catch( ... ) {
         SIMD_REPORT_ERROR( "simd::sys_xbar" ) << name() << " Unexpected";
      }

      if( conn ) {
         break;
      }
   }

   return conn;
}

// Check if there is at least a single connection for the given src port
bool simd_sys_xbar_c::is_map_to_src(
      const std::size_t src_port ) {
   bool conn = false;

   for( std::size_t dst_port = 0; dst_port < map_src_to_dst.at( src_port ).size(); dst_port ++ ) {
      try {
         conn |= map_src_to_dst.at( src_port ).at( dst_port );
      }
      catch( const std::exception& err ) {
         SIMD_REPORT_ERROR( "simd::sys_xbar" ) << name() << " " << err.what();
      }
      catch( ... ) {
         SIMD_REPORT_ERROR( "simd::sys_xbar" ) << name() << " Unexpected";
      }

      if( conn ) {
         break;
      }
   }

   return conn;
}

simd_sys_xbar_c::xbar_dst_vec_c::xbar_dst_vec_c( std::size_t _size )
   : std::vector<bool>( _size, false ) {}

} // namespace simd
