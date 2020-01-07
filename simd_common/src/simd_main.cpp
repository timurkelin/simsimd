#include "simd_common.h"

int sc_main(
   int argc,
   char *argv[] ) {

   // Check command line arguments
   if( argc == 1 ) {
      SIMD_REPORT_INFO( "simd::cmdline" ) << "Default preferences.";
      simd::simd_pref.init(
            );
   }
   else if( argc == 2 ) {
      SIMD_REPORT_INFO( "simd::cmdline" ) << "Preferences file: " << argv[1];
      simd::simd_pref.load(
            std::string( argv[1] ));
   }
   else {
      SIMD_REPORT_ERROR( "simd::cmdline" ) << "Incorrect command line arguments";
   }

   // Debug. should be moved to simd_dump
   simd::simd_pref.save(
         "./config.out");

   simd::simd_pref.parse();
   simd::simd_report.init(
         simd::simd_pref.report_p );
   simd::simd_time.init(
         simd::simd_pref.time_p );

   sc_core::sc_set_time_resolution(
         simd::simd_time.res_sec,
         sc_core::SC_SEC );

   simd::simd_trace.init(
         simd::simd_pref.trace_p );

   // Create and initialise CRM instance
   simd::simd_sys_crm_c crm_i00(
         "crm_i00");
   crm_i00.init(
         simd::simd_pref.clock_p );

   // Create and initialise CORE instance
   simd::simd_sys_core_c core_i00(
         "core_i00" );
   core_i00.init(
         simd::simd_pref.core_p );

   // Create and initialise SCALAR instance
   simd::simd_sys_scalar_c scalar_i00(
         "scalar_i00" );
   scalar_i00.init(
         simd::simd_pref.scalar_p );

   // Create connect clock and reset channels
   sc_core::sc_signal<bool> reset;
   sc_core::sc_signal<bool> clock;

   crm_i00.clock_o.bind(
         clock );
   crm_i00.reset_o.bind(
         reset );

   core_i00.clock_i.bind(
         clock );
   core_i00.reset_i.bind(
         reset );

   scalar_i00.clock_i.bind(
         clock );
   scalar_i00.reset_i.bind(
         reset );

   // Connect SCALAR to CORE
   scalar_i00.event_i.bind(
         core_i00.event_ei );
   scalar_i00.busr_i.bind(
         core_i00.busr_ei );
   scalar_i00.busw_o.bind(
         core_i00.busw_eo );

   // Init memory pool (virtual component)
   simd::simd_sys_pool.init(
         simd::simd_pref.pool_p );

   // Init data dump class
   simd::simd_dump.init(
         simd::simd_pref.dump_p );

   // Invoke the simulation
   if( simd::simd_time.end_sec != 0.0 ) {
      sc_core::sc_start(
            simd::simd_time.end_sec,
            sc_core::SC_SEC );
   }
   else {
      sc_core::sc_start();
   }

   SIMD_REPORT_INFO( "simd::main" ) << "Done.";

   // Ensure that all the dump files are closed before exiting
   simd::simd_dump.close_all();

   return 0;
}
