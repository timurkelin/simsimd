/*
 * simd_report.cpp
 *
 *  Description:
 *    Logging message formatter
 */

#include <utility>
#include <map>
#include <boost/foreach.hpp>

#include "simd_report.h"
#include "sc_stop_here.h"

namespace simd {

const std::string report_compose_message(
      const sc_core::sc_report& rep ) {

   static const char * severity_names[] = {
         "*I:", "*W:", "*E:", "*F:" };

   ::std::stringstream msg;

   sc_core::sc_simcontext* simc = sc_core::sc_get_curr_simcontext();
   bool is_running = sc_core::sc_is_running();

   msg << severity_names[rep.get_severity()];

   if( simc && is_running ) {
      msg << " " << rep.get_time().to_string();
   }

   if( rep.get_id() >= 0 ) {
      msg << " " << rep.get_id() << ":";
   }

   if( rep.get_msg_type()) {
      msg << " " << rep.get_msg_type() << ":";
   }

   if( rep.get_msg() ){
      msg << " " << rep.get_msg();
   }

   msg << std::endl;

   if( rep.get_severity() > sc_core::SC_INFO ) {
      if( simc && is_running ) {
         const char* proc_name = rep.get_process_name();

         if( proc_name ) {
            msg << proc_name << " ";
         }
      }

      msg << rep.get_file_name() << ":" << rep.get_line_number() << std::endl;
   }

   return msg.str();
}


// Private class to handle log files
class log_file_handle {
protected:
   // not CopyConstructible
   log_file_handle(
         log_file_handle const &);

   // not CopyAssignable
   void operator=(
         log_file_handle const &);

public:
   log_file_handle();
   log_file_handle(
         const char* );

   void update_file_name(
         const char* );

   bool release();

   ::std::ofstream& operator*();

private:
   std::string     log_file_name;
   ::std::ofstream log_stream;
};

log_file_handle::log_file_handle()
{}

log_file_handle::log_file_handle(
      const char* fname) : log_file_name(fname), log_stream(fname)
{}

void log_file_handle::update_file_name(
      const char* fname ) {

   if( !fname ) { // filename is NULL
      release();
   }
   else if( log_file_name.empty() ) {
      if( log_stream.is_open()) { //should be closed already
         log_stream.close();
      }

      log_file_name = fname;
      log_stream.open(fname);
   }
   else if( log_file_name != fname ) { // new filename
      release();
      log_file_name = fname;
      log_stream.open(fname);
   }
   else { // nothing to do
   }
}

bool log_file_handle::release() {
   if( log_stream.is_open()) {
      log_stream.close();
      log_file_name.clear();
      return false;
   }

   return true;
}

::std::ofstream& log_file_handle::operator*() {
   return log_stream;
}

static log_file_handle log_stream;

void report_handler(
      const sc_core::sc_report& rep,
      const sc_core::sc_actions& actions) {

   if ( actions & sc_core::SC_DISPLAY ) {
      ::std::cout << report_compose_message(rep);
   }

   if(( actions & sc_core::SC_LOG) && sc_core::sc_report_handler::get_log_file_name() ) {
      log_stream.update_file_name(
            sc_core::sc_report_handler::get_log_file_name());

      *log_stream << report_compose_message(rep);
   }

   if( actions & sc_core::SC_STOP ) {
      sc_core::sc_stop_here(
            rep.get_msg_type(),
            rep.get_severity());

      sc_core::sc_stop();
   }

   if( actions & sc_core::SC_INTERRUPT ) {
      sc_core::sc_interrupt_here(
            rep.get_msg_type(),
            rep.get_severity());
   }

   if( actions & sc_core::SC_ABORT ) {
      sc_core::sc_abort();
   }

   if( actions & sc_core::SC_THROW ) {
      if( !sc_core::sc_is_unwinding() ) {
         throw rep;
      }
   }
}

void simd_report_c::init(
      boost::optional<const boost_pt::ptree&> pref_p ) {

   const boost_pt::ptree& pref = pref_p.get();

   boost::optional<std::string> log_p = pref.get_optional<std::string>("log_file");

   // Set log file name
   if(( !log_p.is_initialized() ) || ( log_p.get().size() == 0 )) {
      SIMD_REPORT_INFO( "simd::report" ) << "Log file name unchanged";
   }
   else if( sc_core::sc_report_handler::set_log_file_name( log_p.get().c_str())) {
      SIMD_REPORT_INFO( "simd::report" ) << "Log file name is set to <"
                                         << log_p.get()
                                         << ">";
   }
   else {
      SIMD_REPORT_INFO( "simd::report" ) << "Log file has already been set to <"
                                         << sc_core::sc_report_handler::get_log_file_name()
                                         << ">";
   }

   bool log_file = ( sc_core::sc_report_handler::get_log_file_name() != NULL );

   // Set actions
   boost::optional<const boost_pt::ptree&> bea_p = pref.get_child_optional("bearing");

   if( bea_p.is_initialized() )  {
      BOOST_FOREACH( const boost_pt::ptree::value_type& bea, bea_p.get() ) {
         if( !bea.first.empty()) {
            SIMD_REPORT_ERROR( "simd::report" ) << "Incorrect structure";
         }

         std::string msg_type;

         try {
            msg_type = bea.second.get<std::string>("msg_type");
         }
         catch( const boost_pt::ptree_error& err ) {
            SIMD_REPORT_ERROR( "simd::report" ) << err.what();
         }
         catch( ... ) {
            SIMD_REPORT_ERROR( "simd::report" ) << "Unexpected";
         }

         typedef std::pair<std::string, sc_core::sc_severity> severity_t;

         std::vector<severity_t> severity_v = {
               {"info",    sc_core::SC_INFO   },
               {"warning", sc_core::SC_WARNING},
               {"error",   sc_core::SC_ERROR  },
               {"fatal",   sc_core::SC_FATAL  }};

         std::map<std::string, sc_core::sc_actions> actions_m = {
               {"unspecified",  sc_core::SC_UNSPECIFIED  },
               {"do_nothing",   sc_core::SC_DO_NOTHING   },
               {"throw",        sc_core::SC_THROW        },
               {"log",          sc_core::SC_LOG          },
               {"display",      sc_core::SC_DISPLAY      },
               {"cache_report", sc_core::SC_CACHE_REPORT },
               {"interrupt",    sc_core::SC_INTERRUPT    },
               {"stop",         sc_core::SC_STOP         },
               {"abort",        sc_core::SC_ABORT        }};

         BOOST_FOREACH( const severity_t& svr, severity_v ) {
            boost::optional<const boost_pt::ptree&> prm_p = bea.second.get_child_optional( svr.first );

            if( prm_p.is_initialized() ) {
               sc_core::sc_actions actions = 0;
               int                 limit   = 0;

               boost::optional<const boost_pt::ptree&> act_p = prm_p.get().get_child_optional( "actions" );

               if( act_p.is_initialized()) {
                  BOOST_FOREACH( const boost_pt::ptree::value_type& act, act_p.get() ) {
                     if( !act.first.empty()) {
                        SIMD_REPORT_ERROR( "simd::report" ) << "Incorrect structure";
                     }

                     try {
                        actions |= actions_m.at( act.second.get_value<std::string>());
                     }
                     catch( const boost_pt::ptree_error& err ) {
                        SIMD_REPORT_ERROR( "simd::report" ) << err.what();
                     }
                     catch( const std::exception& err ) {
                        SIMD_REPORT_ERROR( "simd::report" ) << err.what();
                     }
                     catch( ... ) {
                        SIMD_REPORT_ERROR( "simd::report" ) << "Unexpected";
                     }
                  }
               }
               else {
                  SIMD_REPORT_ERROR( "simd::report" ) << "Actions are not specified.";
               }

               try {
                  limit   = prm_p.get().get<int>( "limit" );
               }
               catch( const boost_pt::ptree_error& err ) {
                  SIMD_REPORT_ERROR( "simd::report" ) << err.what();
               }
               catch( ... ) {
                  SIMD_REPORT_ERROR( "simd::report" ) << "Unexpected";
               }

               if(( actions & sc_core::SC_LOG ) && !log_file ) {
                  SIMD_REPORT_WARNING( "simd::report" ) << "Logging is enabled for msg_type: <"
                                                        << msg_type
                                                        << ">, severity: <"
                                                        << svr.first
                                                        << ">, but no log file is specified in simd scope.";
               }

               if( msg_type.length()) {
                  sc_core::sc_report_handler::set_actions(
                        msg_type.c_str(),
                        svr.second,
                        actions );

                  sc_core::sc_report_handler::stop_after(
                        msg_type.c_str(),
                        svr.second,
                        limit );
               }
               else {
                  sc_core::sc_report_handler::set_actions(
                        svr.second,
                        actions );

                   sc_core::sc_report_handler::stop_after(
                        svr.second,
                        limit );
               }

            }
         } // BOOST_FOREACH(
      } // BOOST_FOREACH(
   }
   else {
      SIMD_REPORT_INFO( "simd::report" ) << "Bearing is not specified";
   }

   // Register report handler
   boost::optional<std::string> msg_p = pref.get_optional<std::string>("handler");

   if( msg_p.get() == "unchanged" ) {
      SIMD_REPORT_INFO( "simd::report" ) << "Handler is unchanged";
   }
   else if( msg_p.get() == "default" ) {
      SIMD_REPORT_INFO( "simd::report" ) << "SC handler";
      sc_core::sc_report_handler::set_handler( NULL );
   }
   else if( msg_p.get() == "simd" ) {
      SIMD_REPORT_INFO( "simd::report" ) << "SIMD handler";
      sc_core::sc_report_handler::set_handler( simd::report_handler );
   }
   else {
      SIMD_REPORT_ERROR( "simd::report" ) << "Incorrect handler specification <"
                                          << msg_p.get()
                                          << ">";
   }

} // void simd_report_c::init(

} // namespace simd
