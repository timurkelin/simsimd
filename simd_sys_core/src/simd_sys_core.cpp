/*
 * simd_sys_core.cpp
 *
 *  Description:
 *    Methods of the SIMD core module
 */

#include <boost/foreach.hpp>
#include "simd_sys_core.h"
#include "simd_assert.h"
#include "simd_report.h"
#include "simd_trace.h"

namespace simd {

SC_HAS_PROCESS( simd::simd_sys_core_c );
simd_sys_core_c::simd_sys_core_c(
      sc_core::sc_module_name nm )
   : sc_core::sc_module( nm )
   , clock_i( "clock_i" )
   , reset_i( "reset_i" )
   , event_ei( "event_ei" )
   , busw_eo( "busw_eo" )
   , busr_ei( "busr_ei" )
   , state_v( "state_v" )
   , proc_v(  "proc_v"  )
   , event_chn_v( "event_chn_v" )
   , event_chn_o( "event_cnh_o" )
   , busw_chn_v( "busw_chn_v" )
   , busw_chn_i( "busw_chn_i" )
   , busr_chn_v( "busr_chn_v" )
   , busr_chn_o( "busr_chn_o" ) {

   // Internal channels allocation

   // Submodules allocation

   // Connectivity

   // Process registrations
}

void simd_sys_core_c::init(
      boost::optional<const boost_pt::ptree&> _pref_p ) {
   const boost_pt::ptree& pref = _pref_p.get();
   pref_p = _pref_p;

   // Initialise DM/EU modules
   boost::optional<const boost_pt::ptree::value_type &> xbar_pref_p = boost::none;

   std::size_t dmeu_idx = 0;
   std::size_t xbar_src_port = 0;
   std::size_t xbar_dst_port = 0;

   BOOST_FOREACH( const boost_pt::ptree::value_type& mod, pref ) {
      if( !mod.first.empty()) {
         SIMD_REPORT_ERROR( "simd::sys_core" ) << "Incorrect format";
      }

      simd_sys_dmeu_info_t inst;

      try {
         inst.func = mod.second.get<std::string>("function");
      }
      catch( const boost_pt::ptree_error& err ) {
         SIMD_REPORT_ERROR( "simd::sys_core" ) << err.what();
      }
      catch( ... ) {
         SIMD_REPORT_ERROR( "simd::sys_core" ) << "Unexpected";
      }

      try {
         inst.name = mod.second.get<std::string>("name");
      }
      catch( const boost_pt::ptree_error& err ) {
         SIMD_REPORT_ERROR( "simd::sys_core" ) << err.what();
      }
      catch( ... ) {
         SIMD_REPORT_ERROR( "simd::sys_core" ) << "Unexpected";
      }

      // Calculate name hash
      inst.hash = 0;
      boost::hash_combine( inst.hash, inst.name);

      if( inst.func == "xbar" ) { // Save X-BAR reference for later
         if( !xbar_pref_p  ) {
            xbar_pref_p = boost::optional<const boost_pt::ptree::value_type &>( mod );
            continue;
         }
         else {
            SIMD_REPORT_ERROR( "simd::sys_core" ) << "Duplicate instance of " << inst.func;
         }
      }

      // Check if the instance name is unique
      BOOST_FOREACH( const simd_sys_dmeu_info_t& di, dmeu_info ) {
         if( di.name == inst.name ) {
            SIMD_REPORT_ERROR( "simd::sys_core" ) << "Duplicate name " << inst.name;
         }
      }

      // Create DM/EU module
      inst.mod_p = simd_sys_dmeu_new(
            inst.func,
            inst.name.c_str() );

      // Initialise DM/EU module
      inst.pref_p = mod.second.get_child_optional( "param" );
      inst.mod_p.get().init( inst.pref_p );

      // Sizes of the data port vectors.
      std::size_t n_i = inst.mod_p.get().data_vi.size();
      n_i ? inst.xbar_dmeu.resize( n_i, UINT_MAX ) : inst.xbar_dmeu.reserve( n_i );

      BOOST_FOREACH( std::size_t& dst_port, inst.xbar_dmeu ) {
         dst_port = xbar_dst_port;
         xbar_dst_port ++;
      }

      std::size_t n_o = inst.mod_p.get().data_vo.size();
      n_o ? inst.dmeu_xbar.resize( n_o, UINT_MAX ) : inst.dmeu_xbar.reserve( n_o );

      BOOST_FOREACH( std::size_t& src_port, inst.dmeu_xbar ) {
         src_port = xbar_src_port;
         xbar_src_port ++;
      }

      inst.idx = dmeu_idx;
      dmeu_idx ++;

      dmeu_info.push_back( inst );

      SIMD_REPORT_INFO( "simd::sys_core" ) << "DM/EU inst: " << inst.mod_p.get().name()
                                           << " func: " << inst.func
                                           << " Ni: "   << inst.mod_p.get().data_vi.size()
                                           << " No: "   << inst.mod_p.get().data_vo.size();
   }

   state_v.init(  dmeu_info.size());
   proc_v.init( dmeu_info.size());

   // Create and initialise XBAR
   std::string mod_name;

   if( !xbar_pref_p ) {
      SIMD_REPORT_ERROR( "simd::sys_core" ) << "Unable to find XBAR preferences";
   }

   mod_name = xbar_pref_p.get().second.get<std::string>("name");
   xbar_p = boost::optional<simd::simd_sys_xbar_c &>(
         *( new simd_sys_xbar_c( mod_name.c_str())));

   xbar_p.get().init(
         xbar_pref_p.get().second.get_child_optional( "param" ),
         dmeu_info );

   SIMD_REPORT_INFO( "simd::sys_core" ) << "XBAR inst: " << xbar_p.get().name()
                                        << " Nxi: " << xbar_p.get().data_xvi.size()
                                        << " Nxo: " << xbar_p.get().data_xvo.size();

   // Create and initialise Event router
   mod_name = std::string( name()) + "_event";
   event_p = boost::optional<simd::simd_sys_event_c &>(
         *( new simd_sys_event_c( mod_name.c_str())));

   event_p.get().init(
         dmeu_info );

   SIMD_REPORT_INFO( "simd::sys_core" ) << "ICU inst: " << event_p.get().name()
                                        << " Nxi: " << event_p.get().event_vi.size();

   // Create and initialise BMUXW
   mod_name = std::string( name()) + "_bmuxw";
   bmuxw_p = boost::optional<simd::simd_sys_bmuxw_c &>(
         *( new simd_sys_bmuxw_c( mod_name.c_str())));

   bmuxw_p.get().init(
         dmeu_info );

   SIMD_REPORT_INFO( "simd::sys_core" ) << "BMUXW inst: " << bmuxw_p.get().name()
                                        << " Nxi: " << bmuxw_p.get().busw_vo.size();

   // Create and initialise BMUXR
   mod_name = std::string( name()) + "_bmuxr";
   bmuxr_p = boost::optional<simd::simd_sys_bmuxr_c &>(
         *( new simd_sys_bmuxr_c( mod_name.c_str())));

   bmuxr_p.get().init(
         dmeu_info );

   SIMD_REPORT_INFO( "simd::sys_core" ) << "BMUXR inst: " << bmuxr_p.get().name()
                                        << " Nxi: " << bmuxr_p.get().busr_vi.size();

   // Initialise event, Config and Status channel vectors
   event_chn_v.init( dmeu_info.size() + 1); // We also need a channel for xbar events
   busr_chn_v.init(  dmeu_info.size() + 0);
   busw_chn_v.init(  dmeu_info.size() + 1); // We also need a channel for xbar configuration

   // Bind DM/EU
   BOOST_FOREACH( simd_sys_dmeu_info_t& v, dmeu_info ) {
      // Connect XBAR dst ports
      for( std::size_t n_i = 0; n_i < v.xbar_dmeu.size(); n_i ++ ) {
         v.mod_p.get().data_vi.at( n_i ).bind(
               xbar_p.get().data_xvi.at( v.xbar_dmeu.at( n_i )));
      }

      // Connect XBAR src ports
      for( std::size_t n_o = 0; n_o < v.dmeu_xbar.size(); n_o ++ ) {
         v.mod_p.get().data_vo.at( n_o ).bind(
               xbar_p.get().data_xvo.at( v.dmeu_xbar.at( n_o )));
      }

      v.mod_p.get().state_o.bind( state_v.at( v.idx ));
      xbar_p.get().state_vi.at( v.idx ).bind( state_v.at( v.idx ));

      v.mod_p.get().proc_i.bind( proc_v.at( v.idx ));
      xbar_p.get().proc_vo.at( v.idx ).bind( proc_v.at( v.idx ));

      // Connect event, Config and Status channels
      v.mod_p.get().event_o.bind( event_chn_v.at( v.idx ));
      event_p.get().event_vi.at( v.idx ).bind( event_chn_v.at( v.idx ));

      v.mod_p.get().busw_i.bind( busw_chn_v.at( v.idx ));
      v.mod_p.get().busr_o.bind( busr_chn_v.at( v.idx ));

      bmuxw_p.get().busw_vo.at( v.idx ).bind( busw_chn_v.at( v.idx ));
      bmuxr_p.get().busr_vi.at( v.idx ).bind( busr_chn_v.at( v.idx ));

      // Connect clock and reset for DM/EU
      v.mod_p.get().clock_i.bind( clock_i );
      v.mod_p.get().reset_i.bind( reset_i );
   }

   // Connect clock and reset for XBAR
   xbar_p.get().clock_i.bind( clock_i );
   xbar_p.get().reset_i.bind( reset_i );

   // Connect config port for XBAR
   xbar_p.get().busw_i.bind(
         busw_chn_v.at(
               busw_chn_v.size() - 1 ));
   bmuxw_p.get().busw_vo.at(
         bmuxw_p.get().busw_vo.size() - 1 ).bind(
               busw_chn_v.at(
                     busw_chn_v.size() - 1 ));

   // Connect event port for XBAR
   xbar_p.get().event_o.bind(
         event_chn_v.at(
               event_chn_v.size() - 1 ));
   event_p.get().event_vi.at(
         event_p.get().event_vi.size() - 1 ).bind(
               event_chn_v.at(
                     event_chn_v.size() - 1 ));

   // Connect clock, reset and output port for Event MUX
   event_p.get().clock_i.bind( clock_i );
   event_p.get().reset_i.bind( reset_i );
   event_p.get().event_o.bind( event_chn_o );
   event_ei.bind( event_chn_o );

   // Connect clock, reset and output port for BMUXR
   bmuxr_p.get().clock_i.bind( clock_i );
   bmuxr_p.get().reset_i.bind( reset_i );
   bmuxr_p.get().busr_o.bind( busr_chn_o );
   busr_ei.bind( busr_chn_o );

   // Connect clock, reset and output port for BMUXC
   bmuxw_p.get().clock_i.bind( clock_i );
   bmuxw_p.get().reset_i.bind( reset_i );
   bmuxw_p.get().busw_i.bind( busw_chn_i );
   busw_eo.bind( busw_chn_i );

   // Add signal traces only after everything has been connected
   sc_core::sc_trace( simd_trace.tf, clock_i, "clock" );
   sc_core::sc_trace( simd_trace.tf, reset_i, "reset" );

   BOOST_FOREACH( simd_sys_dmeu_info_t& v, dmeu_info ) {
      v.mod_p.get().add_trace( simd_trace.tf, name());
   }

   xbar_p.get().add_trace( simd_trace.tf, name());

   return;
}

} // namespace simd
