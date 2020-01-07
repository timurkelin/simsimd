/*
 * simd_scalar_run.cpp
 *
 *  Description:
 *    System components: Scalar processor
 */

#include <boost/random.hpp>
#include "simd_sys_scalar.h"
#include "simd_assert.h"
#include "simd_report.h"
#include "simd_trace.h"

namespace simd {

bool check_event(
      const boost_pt::ptree& event_pt,
      const std::string& source,
      const std::string& event_id ) {
   if( source == "xbar" ) {
      return ( event_pt.get<std::string>( "source"   ) == source    &&
               event_pt.get<std::string>( "mod_name" ) == event_id  &&
               event_pt.get<std::string>( "event_id" ) == "mod_done" );
   }
   else {
      return ( event_pt.get<std::string>( "source"   ) == source    &&
               event_pt.get<std::string>( "event_id" ) == event_id );
   }
}

namespace boost_rn = boost::random;

unsigned int n_test;

void simd_sys_scalar_c::init(
      boost::optional<const boost_pt::ptree&> pref_p ) {

   sc_core::sc_trace( simd_trace.tf, n_test, "n_test" );
}

void simd_sys_scalar_c::exec_thrd(
      void ) {

   boost_pt::ptree conf_data;
   boost_rn::mt19937 rng_eng;

   boost_rn::uniform_int_distribution<> rng_dist_uni( 1, 7 );
   boost_rn::variate_generator<boost::mt19937&, boost_rn::uniform_int_distribution<> > rng_test( rng_eng, rng_dist_uni );

   boost_rn::bernoulli_distribution <>   rng_dist_bern( 0.5 );
   boost_rn::variate_generator<boost::mt19937&, boost_rn::bernoulli_distribution<> > rng_val( rng_eng, rng_dist_bern );

   sc_core::wait();
   sc_core::wait();

   for(;;) {

      n_test = rng_test();

      SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Start test " << n_test;

      switch( n_test ) {

      case 1:
         //*************************************
         // Test 01: Run 2 SRC->DST streams in parallel
         // SRC1->DST1; SRC2->DST2
         //*************************************

         // Configure XBAR
         sc_core::wait();
         wr_conf( std::string("")
            + "{ \"dest\":\"xbar\",\"idx\":0,\"cmd\":\"put\",\"data\":"
            + "    {\"exec_idx\":100,\"conf_next\":8,\"routing\":"
            + "       [{\"mod\":\"st_gen1\",\"port\":0,\"event\":1,\"dst\":"
            + "           [{\"mod\":\"st_ana1\",\"port\":0,\"master\":1"
            + "            }"
            + "           ]"
            + "        },"
            + "        {\"mod\":\"st_gen2\",\"port\":0,\"event\":1,\"dst\":"
            + "           [{\"mod\":\"st_ana2\",\"port\":0,\"master\":1"
            + "            }"
            + "           ]"
            + "        }"
            + "       ]"
            + "    }"
            + "}" );
         SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 01.01";

         // Configure VRI Source 1
         sc_core::wait();
         wr_conf( std::string("")
            + "{ \"dest\":\"st_gen1\",\"idx\":0,\"cmd\":\"put\",\"data\":"
            + "    {\"exec_idx\":100,\"stat_idx\":0,\"conf_next\":8,\"valid_ovr\":" + std::to_string( rng_val()) + ","
            + "     \"vec_size\":\"rng\",\"slot_ena_ovr\":0,\"events\":"
            + "       [\"vec_head\",\"vec_tail\""
            + "       ]"
            + "    }"
            + "}" );
         SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 01.02";

         // Configure VRI Source 2
         sc_core::wait();
         wr_conf( std::string("")
            + "{ \"dest\":\"st_gen2\",\"idx\":0,\"cmd\":\"put\",\"data\":"
            + "    {\"exec_idx\":100,\"stat_idx\":0,\"conf_next\":8,\"valid_ovr\":" + std::to_string( rng_val()) + ","
            + "     \"vec_size\":\"rng\",\"slot_ena_ovr\":0,\"events\":"
            + "       [\"vec_head\",\"vec_tail\""
            + "       ]"
            + "    }"
            + "}" );
         SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 01.03";

         // Configure VRI Destination 1
         sc_core::wait();
         wr_conf( std::string("")
            + "{ \"dest\":\"st_ana1\",\"idx\":0,\"cmd\":\"put\",\"data\":"
            + "    {\"exec_idx\":100,\"stat_idx\":0,\"conf_next\":8,\"ready_ovr\":" + std::to_string( rng_val()) + ",\"events\":"
            + "       [\"vec_head\",\"vec_tail\""
            + "       ]"
            + "    }"
            + "}" );
         SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 01.04";

         // Configure VRI Destination 2
         sc_core::wait();
         wr_conf( std::string("")
            + "{ \"dest\":\"st_ana2\",\"idx\":0,\"cmd\":\"put\",\"data\":"
            + "    {\"exec_idx\":100,\"stat_idx\":0,\"conf_next\":8,\"ready_ovr\":" + std::to_string( rng_val()) + ",\"events\":"
            + "       [\"vec_head\",\"vec_tail\""
            + "       ]"
            + "    }"
            + "}" );
         SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 01.05";

         // Run
         sc_core::wait();
         wr_conf( std::string("")
            + "{ \"dest\":\"broadcast\",\"idx\":100,\"cmd\":\"run\""
            + "}" );
         SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Start  01.06";

         // Wait for the completion events from the streaming modules and xbar
         evt_proc_init( std::string("")
            + "{\"valid_all\":"
            + "  [{\"mod\":\"st_ana1\",\"evt\":\"vec_tail\"},"
            + "   {\"mod\":\"st_ana2\",\"evt\":\"vec_tail\"},"
            + "   {\"mod\":\"st_gen1\",\"evt\":\"vec_tail\"},"
            + "   {\"mod\":\"st_gen2\",\"evt\":\"vec_tail\"},"
            + "   {\"mod\":\"xbar\",   \"evt\":\"st_gen1\" },"
            + "   {\"mod\":\"xbar\",   \"evt\":\"st_gen2\" }"
            + "  ]"
            + "}" );

         for(;;) {
            sc_core::wait();

            if( event_i->num_available()) {
               boost_pt::ptree event_pt = event_i->read().get();

               evt_proc_t evt_proc = evt_proc_check( event_pt );

               if( evt_proc == EVT_PROC_VALID_ALL ) {
                  break;
               }
               else if( evt_proc != EVT_PROC_NONE ) {
                  SIMD_REPORT_ERROR( "simd::sys_scalar" ) << " Incorrect event source";
               }
            }
         }
         SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Check  01.07";

         // Read status
         sc_core::wait();
         wr_conf( "{ \"dest\":\"st_gen1\",\"idx\":0,\"cmd\":\"get\"}" );

         sc_core::wait();
         wr_conf( "{ \"dest\":\"st_gen2\",\"idx\":0,\"cmd\":\"get\"}" );

         sc_core::wait();
         wr_conf( "{ \"dest\":\"st_ana1\",\"idx\":0,\"cmd\":\"get\"}" );

         sc_core::wait();
         wr_conf( "{ \"dest\":\"st_ana2\",\"idx\":0,\"cmd\":\"get\"}" );

         sc_core::wait();
         sc_core::wait();
         {
            int st_gen1_vec_size = 0;
            int st_gen2_vec_size = 0;
            int st_ana1_vec_size = 0;
            int st_ana2_vec_size = 0;

            try {
               simd_sig_ptree_c rd;
               std::string      src;

               //===
               busr_i->nb_read( rd );
               src = rd.get().get<std::string>( "source" );

               if( !( src == "st_gen1" )) {
                  SIMD_REPORT_ERROR( "simd::sys_scalar" ) << name() << " Incorrect sequence of responses";
               }

               st_gen1_vec_size = rd.get().get<int>( "vec_size" );

               //===
               busr_i->nb_read( rd );
               src = rd.get().get<std::string>( "source" );

               if( !( src == "st_gen2" )) {
                  SIMD_REPORT_ERROR( "simd::sys_scalar" ) << name() << " Incorrect sequence of responses";
               }

               st_gen2_vec_size = rd.get().get<int>( "vec_size" );

               //===
               busr_i->nb_read( rd );
               src = rd.get().get<std::string>( "source" );

               if( !( src == "st_ana1" )) {
                  SIMD_REPORT_ERROR( "simd::sys_scalar" ) << name() << " Incorrect sequence of responses";
               }

               st_ana1_vec_size = rd.get().get<int>( "vec_size" );

               //===
               busr_i->nb_read( rd );
               src = rd.get().get<std::string>( "source" );

               if( !( src == "st_ana2" )) {
                  SIMD_REPORT_ERROR( "simd::sys_scalar" ) << name() << " Incorrect sequence of responses";
               }

               st_ana2_vec_size = rd.get().get<int>( "vec_size" );
            }
            catch( const boost_pt::ptree_error& err ) {
               SIMD_REPORT_ERROR( "simd::sys_scalar" ) << name() << " " <<  err.what();
            }
            catch( ... ) {
               SIMD_REPORT_ERROR( "simd::sys_scalar" ) << name() << " Unexpected";
            }

            if( st_gen1_vec_size != st_ana1_vec_size ) {
               SIMD_REPORT_ERROR( "simd::sys_scalar" ) << name() << " Vector sizes mismatch";
            }

            if( st_gen2_vec_size != st_ana2_vec_size ) {
               SIMD_REPORT_ERROR( "simd::sys_scalar" ) << name() << " Vector sizes mismatch";
            }

         }
         SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Check  01.08";
         break;

      case 2:
         //*************************************
         // Test 02: Run 2 SRC->DST streams in parallel.
         // SRC1->DST2; SRC2->DST1
         // DST2 has its ready always on
         //*************************************

         // Configure XBAR with separate slots for the streams
         sc_core::wait();
         wr_conf( std::string("")
            + "{ \"dest\":\"xbar\",\"idx\":0,\"cmd\":\"put\",\"data\":"
            + "    {\"exec_idx\":200,\"conf_next\":8,\"routing\":"
            + "       [{\"mod\":\"st_gen1\",\"port\":0,\"event\":1,\"dst\":"
            + "           [{\"mod\":\"st_ana2\",\"port\":0,\"master\":1"
            + "            }"
            + "           ]"
            + "        }"
            + "       ]"
            + "    }"
            + "}" );
         SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 02.01";

         sc_core::wait();
         wr_conf( std::string("")
            + "{ \"dest\":\"xbar\",\"idx\":1,\"cmd\":\"put\",\"data\":"
            + "    {\"exec_idx\":200,\"conf_next\":8,\"routing\":"
            + "       [{\"mod\":\"st_gen2\",\"port\":0,\"event\":1,\"dst\":"
            + "           [{\"mod\":\"st_ana1\",\"port\":0,\"master\":1"
            + "            }"
            + "           ]"
            + "        }"
            + "       ]"
            + "    }"
            + "}" );
         SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 02.02";

         // Configure VRI Source 1
         sc_core::wait();
         wr_conf( std::string("")
            + "{ \"dest\":\"st_gen1\",\"idx\":0,\"cmd\":\"put\",\"data\":"
            + "    {\"exec_idx\":200,\"stat_idx\":0,\"conf_next\":8,\"valid_ovr\":" + std::to_string( rng_val()) + ","
            + "     \"vec_size\":\"rng\",\"slot_ena_ovr\":0,\"events\":"
            + "       [\"vec_head\",\"vec_tail\""
            + "       ]"
            + "    }"
            + "}" );
         SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 02.03";

         // Configure VRI Source 2
         sc_core::wait();
         wr_conf( std::string("")
            + "{ \"dest\":\"st_gen2\",\"idx\":0,\"cmd\":\"put\",\"data\":"
            + "    {\"exec_idx\":200,\"stat_idx\":0,\"conf_next\":8,\"valid_ovr\":" + std::to_string( rng_val()) + ","
            + "     \"vec_size\":\"rng\",\"slot_ena_ovr\":0,\"events\":"
            + "       [\"vec_head\",\"vec_tail\""
            + "       ]"
            + "    }"
            + "}" );
         SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 02.04";

         // Configure VRI Destination 1
         sc_core::wait();
         wr_conf( std::string("")
            + "{ \"dest\":\"st_ana1\",\"idx\":0,\"cmd\":\"put\",\"data\":"
            + "    {\"exec_idx\":200,\"stat_idx\":0,\"conf_next\":8,\"ready_ovr\":0,\"events\":"
            + "       [\"vec_head\",\"vec_tail\""
            + "       ]"
            + "    }"
            + "}" );
         SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 02.05";

         // Configure VRI Destination 2
         sc_core::wait();
         wr_conf( std::string("")
            + "{ \"dest\":\"st_ana2\",\"idx\":0,\"cmd\":\"put\",\"data\":"
            + "    {\"exec_idx\":200,\"stat_idx\":0,\"conf_next\":8,\"ready_ovr\":1,\"events\":"
            + "       [\"vec_head\",\"vec_tail\""
            + "       ]"
            + "    }"
            + "}" );
         SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 02.06";

         // Run
         sc_core::wait();
         wr_conf( std::string("")
            + "{ \"dest\":\"broadcast\",\"idx\":200,\"cmd\":\"run\""
            + "}" );
         SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Start  02.07";

         // Wait for the completion events from the streaming modules and xbar
         evt_proc_init( std::string("")
            + "{\"valid_all\":"
            + "  [{\"mod\":\"st_ana1\",\"evt\":\"vec_tail\"},"
            + "   {\"mod\":\"st_ana2\",\"evt\":\"vec_tail\"},"
            + "   {\"mod\":\"st_gen1\",\"evt\":\"vec_tail\"},"
            + "   {\"mod\":\"st_gen2\",\"evt\":\"vec_tail\"},"
            + "   {\"mod\":\"xbar\",   \"evt\":\"st_gen1\" },"
            + "   {\"mod\":\"xbar\",   \"evt\":\"st_gen2\" }"
            + "  ]"
            + "}" );

         for(;;) {
            sc_core::wait();

            if( event_i->num_available()) {
               boost_pt::ptree event_pt = event_i->read().get();

               evt_proc_t evt_proc = evt_proc_check( event_pt );

               if( evt_proc == EVT_PROC_VALID_ALL ) {
                  break;
               }
               else if( evt_proc != EVT_PROC_NONE ) {
                  SIMD_REPORT_ERROR( "simd::sys_scalar" ) << " Incorrect event source";
               }
            }
         }
         SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Check  02.08";

         // Read status
         sc_core::wait();
         {
            int st_gen1_vec_size = 0;
            int st_gen2_vec_size = 0;
            int st_ana1_vec_size = 0;
            int st_ana2_vec_size = 0;

            try {
               simd_sig_ptree_c rd;
               std::string      src;

               //===
               wr_conf( "{ \"dest\":\"st_gen1\",\"idx\":0,\"cmd\":\"get\"}" );
               sc_core::wait( 2 );

               busr_i->nb_read( rd );
               src = rd.get().get<std::string>( "source" );

               if( !( src == "st_gen1" )) {
                  SIMD_REPORT_ERROR( "simd::sys_scalar" ) << name() << " Incorrect sequence of responses";
               }

               st_gen1_vec_size = rd.get().get<int>( "vec_size" );

               //===
               wr_conf( "{ \"dest\":\"st_gen2\",\"idx\":0,\"cmd\":\"get\"}" );
               sc_core::wait( 2 );

               busr_i->nb_read( rd );
               src = rd.get().get<std::string>( "source" );

               if( !( src == "st_gen2" )) {
                  SIMD_REPORT_ERROR( "simd::sys_scalar" ) << name() << " Incorrect sequence of responses";
               }

               st_gen2_vec_size = rd.get().get<int>( "vec_size" );

               //===
               wr_conf( "{ \"dest\":\"st_ana1\",\"idx\":0,\"cmd\":\"get\"}" );
               sc_core::wait( 2 );

               busr_i->nb_read( rd );
               src = rd.get().get<std::string>( "source" );

               if( !( src == "st_ana1" )) {
                  SIMD_REPORT_ERROR( "simd::sys_scalar" ) << name() << " Incorrect sequence of responses";
               }

               st_ana1_vec_size = rd.get().get<int>( "vec_size" );

               //===
               wr_conf( "{ \"dest\":\"st_ana2\",\"idx\":0,\"cmd\":\"get\"}" );
               sc_core::wait( 2 );

               busr_i->nb_read( rd );
               src = rd.get().get<std::string>( "source" );

               if( !( src == "st_ana2" )) {
                  SIMD_REPORT_ERROR( "simd::sys_scalar" ) << name() << " Incorrect sequence of responses";
               }

               st_ana2_vec_size = rd.get().get<int>( "vec_size" );
            }
            catch( const boost_pt::ptree_error& err ) {
               SIMD_REPORT_ERROR( "simd::sys_scalar" ) << name() << " " <<  err.what();
            }
            catch( ... ) {
               SIMD_REPORT_ERROR( "simd::sys_scalar" ) << name() << " Unexpected";
            }

            if( st_gen1_vec_size != st_ana2_vec_size ) {
               SIMD_REPORT_ERROR( "simd::sys_scalar" ) << name() << " Vector sizes mismatch";
            }

            if( st_gen2_vec_size != st_ana1_vec_size ) {
               SIMD_REPORT_ERROR( "simd::sys_scalar" ) << name() << " Vector sizes mismatch";
            }

         }
         SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Check  02.09";

         break;

      case 3:
         //*************************************
         // Test 03: Run 1xSRC->2xDST stream.
         // SRC1->DST1(master),DST2
         // DST2 has its ready always on
         //*************************************

         // Configure XBAR
         sc_core::wait();
         wr_conf( std::string("")
            + "{ \"dest\":\"xbar\",\"idx\":0,\"cmd\":\"put\",\"data\":"
            + "    {\"exec_idx\":300,\"conf_next\":8,\"routing\":"
            + "       [{\"mod\":\"st_gen1\",\"port\":0,\"event\":1,\"dst\":"
            + "           [{\"mod\":\"st_ana1\",\"port\":0,\"master\":1"
            + "            },"
            + "            {\"mod\":\"st_ana2\",\"port\":0,\"master\":0"
            + "            }"
            + "           ]"
            + "        }"
            + "       ]"
            + "    }"
            + "}" );
         SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 03.01";

         // Configure XBAR (inactive slot)
         sc_core::wait();
         wr_conf( std::string("")
            + "{ \"dest\":\"xbar\",\"idx\":1,\"cmd\":\"put\",\"data\":"
            + "    {\"exec_idx\":999,\"conf_next\":8,\"routing\":"
            + "       [{\"mod\":\"st_gen2\",\"port\":0,\"event\":1,\"dst\":"
            + "           [{\"mod\":\"st_ana1\",\"port\":0,\"master\":1"
            + "            }"
            + "           ]"
            + "        }"
            + "       ]"
            + "    }"
            + "}" );
         SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 03.02";

         // Configure VRI Source 1
         sc_core::wait();
         wr_conf( std::string("")
            + "{ \"dest\":\"st_gen1\",\"idx\":0,\"cmd\":\"put\",\"data\":"
            + "    {\"exec_idx\":300,\"stat_idx\":0,\"conf_next\":8,\"valid_ovr\":" + std::to_string( rng_val()) + ","
            + "     \"vec_size\":\"rng\",\"slot_ena_ovr\":0,\"events\":"
            + "       [\"vec_head\",\"vec_tail\""
            + "       ]"
            + "    }"
            + "}" );
         SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 03.03";

         // Configure VRI Source 2 (inactive dmeu)
         sc_core::wait();
         wr_conf( std::string("")
            + "{ \"dest\":\"st_gen2\",\"idx\":0,\"cmd\":\"put\",\"data\":"
            + "    {\"exec_idx\":999,\"stat_idx\":0,\"conf_next\":8,\"valid_ovr\":" + std::to_string( rng_val()) +","
            + "     \"vec_size\":\"rng\",\"slot_ena_ovr\":0,\"events\":"
            + "       [\"vec_head\",\"vec_tail\""
            + "       ]"
            + "    }"
            + "}" );
         SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 03.04";

         // Configure VRI Destination 1
         sc_core::wait();
         wr_conf( std::string("")
            + "{ \"dest\":\"st_ana1\",\"idx\":0,\"cmd\":\"put\",\"data\":"
            + "    {\"exec_idx\":300,\"stat_idx\":0,\"conf_next\":8,\"ready_ovr\":0,\"events\":"
            + "       [\"vec_head\",\"vec_tail\""
            + "       ]"
            + "    }"
            + "}" );
         SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 03.05";

         // Configure VRI Destination 2
         sc_core::wait();
         wr_conf( std::string("")
            + "{ \"dest\":\"st_ana2\",\"idx\":0,\"cmd\":\"put\",\"data\":"
            + "    {\"exec_idx\":300,\"stat_idx\":0,\"conf_next\":8,\"ready_ovr\":1,\"events\":"
            + "       [\"vec_head\",\"vec_tail\""
            + "       ]"
            + "    }"
            + "}" );
         SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 03.06";

         // Run
         sc_core::wait();
         wr_conf( std::string("")
            + "{ \"dest\":\"broadcast\",\"idx\":300,\"cmd\":\"run\""
            + "}" );
         SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Start  03.04";

         // Wait for the completion events from the streaming modules and xbar
         evt_proc_init( std::string("")
            + "{\"valid_all\":"
            + "  [{\"mod\":\"st_ana1\",\"evt\":\"vec_tail\"},"
            + "   {\"mod\":\"st_ana2\",\"evt\":\"vec_tail\"},"
            + "   {\"mod\":\"st_gen1\",\"evt\":\"vec_tail\"},"
            + "   {\"mod\":\"xbar\",   \"evt\":\"st_gen1\" }"
            + "  ],"
            + " \"error_any\":"
            + "  [{\"mod\":\"st_gen2\",\"evt\":\"vec_tail\"},"
            + "   {\"mod\":\"xbar\",   \"evt\":\"st_gen2\" }"
            + "  ]"
            + "}" );

         for(;;) {
            sc_core::wait();

            if( event_i->num_available()) {
               boost_pt::ptree event_pt = event_i->read().get();

               evt_proc_t evt_proc = evt_proc_check( event_pt );

               if( evt_proc == EVT_PROC_VALID_ALL ) {
                  break;
               }
               else if( evt_proc != EVT_PROC_NONE ) {
                  SIMD_REPORT_ERROR( "simd::sys_scalar" ) << " Incorrect event source";
               }
            }
         }
         SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Check 03.07";

         // Read status
         sc_core::wait();
         {
            int st_gen1_vec_size = 0;
            int st_ana1_vec_size = 0;
            int st_ana2_vec_size = 0;

            try {
               simd_sig_ptree_c rd;
               std::string      src;

               //===
               wr_conf( "{ \"dest\":\"st_gen1\",\"idx\":0,\"cmd\":\"get\"}" );
               sc_core::wait( 2 );

               busr_i->nb_read( rd );
               src = rd.get().get<std::string>( "source" );

               if( !( src == "st_gen1" )) {
                  SIMD_REPORT_ERROR( "simd::sys_scalar" ) << name() << " Incorrect sequence of responses";
               }

               st_gen1_vec_size = rd.get().get<int>( "vec_size" );

               //===
               wr_conf( "{ \"dest\":\"st_ana1\",\"idx\":0,\"cmd\":\"get\"}" );
               sc_core::wait( 2 );

               busr_i->nb_read( rd );
               src = rd.get().get<std::string>( "source" );

               if( !( src == "st_ana1" )) {
                  SIMD_REPORT_ERROR( "simd::sys_scalar" ) << name() << " Incorrect sequence of responses";
               }

               st_ana1_vec_size = rd.get().get<int>( "vec_size" );

               //===
               wr_conf( "{ \"dest\":\"st_ana2\",\"idx\":0,\"cmd\":\"get\"}" );
               sc_core::wait( 2 );

               busr_i->nb_read( rd );
               src = rd.get().get<std::string>( "source" );

               if( !( src == "st_ana2" )) {
                  SIMD_REPORT_ERROR( "simd::sys_scalar" ) << name() << " Incorrect sequence of responses";
               }

               st_ana2_vec_size = rd.get().get<int>( "vec_size" );
            }
            catch( const boost_pt::ptree_error& err ) {
               SIMD_REPORT_ERROR( "simd::sys_scalar" ) << name() << " " <<  err.what();
            }
            catch( ... ) {
               SIMD_REPORT_ERROR( "simd::sys_scalar" ) << name() << " Unexpected";
            }

            if(( st_gen1_vec_size != st_ana1_vec_size ) || ( st_gen1_vec_size != st_ana2_vec_size )) {
               SIMD_REPORT_ERROR( "simd::sys_scalar" ) << name() << " Vector sizes mismatch";
            }
         }
         SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Check  03.08";

         break;

      case 4:
         //*************************************
         // Test 04: Execute 3 steps under the distributed control of the vector core.
         // Step 1. SRC1->DST1; SRC2->DST2
         // Step 2. SRC1->DST2; SRC2->DST1
         // Step 3. SRC1->DST1(master),DST2
         //*************************************

         // Configure XBAR separately for each route src1->dst1; src2->dst2 (step1). No events
         sc_core::wait();
         wr_conf( std::string("")
            + "{ \"dest\":\"xbar\",\"idx\":0,\"cmd\":\"put\",\"data\":"
            + "    {\"exec_idx\":400,\"conf_next\":2,\"routing\":"
            + "       [{\"mod\":\"st_gen1\",\"port\":0,\"event\":0,\"dst\":"
            + "           [{\"mod\":\"st_ana1\",\"port\":0,\"master\":1"
            + "            }"
            + "           ]"
            + "        }"
            + "       ]"
            + "    }"
            + "}" );
         SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 04.01";

         // no conf_next for this route
         sc_core::wait();
         wr_conf( std::string("")
            + "{ \"dest\":\"xbar\",\"idx\":1,\"cmd\":\"put\",\"data\":"
            + "    {\"exec_idx\":400,\"conf_next\":8,\"routing\":"
            + "       [{\"mod\":\"st_gen2\",\"port\":0,\"event\":0,\"dst\":"
            + "           [{\"mod\":\"st_ana2\",\"port\":0,\"master\":1"
            + "            }"
            + "           ]"
            + "        }"
            + "       ]"
            + "    }"
            + "}" );
         SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 04.02";

         // Configure XBAR jointly for 2 routes src1->dst2; src2->dst1 (step2). No events
         sc_core::wait();
         wr_conf( std::string("")
            + "{ \"dest\":\"xbar\",\"idx\":2,\"cmd\":\"put\",\"data\":"
            + "    {\"exec_idx\":999,\"conf_next\":3,\"routing\":"
            + "       [{\"mod\":\"st_gen1\",\"port\":0,\"event\":0,\"dst\":"
            + "           [{\"mod\":\"st_ana2\",\"port\":0,\"master\":1"
            + "            }"
            + "           ]"
            + "        },"
            + "        {\"mod\":\"st_gen2\",\"port\":0,\"event\":0,\"dst\":"
            + "           [{\"mod\":\"st_ana1\",\"port\":0,\"master\":1"
            + "            }"
            + "           ]"
            + "        }"
            + "       ]"
            + "    }"
            + "}" );
         SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 04.03";

         // Configure XBAR jointly for src1->dst1,dst2 (step3). Generate events
         sc_core::wait();
         wr_conf( std::string("")
            + "{ \"dest\":\"xbar\",\"idx\":3,\"cmd\":\"put\",\"data\":"
            + "    {\"exec_idx\":999,\"conf_next\":8,\"routing\":"
            + "       [{\"mod\":\"st_gen1\",\"port\":0,\"event\":1,\"dst\":"
            + "           [{\"mod\":\"st_ana1\",\"port\":0,\"master\":1"
            + "            },"
            + "            {\"mod\":\"st_ana2\",\"port\":0,\"master\":0"
            + "            }"
            + "           ]"
            + "        }"
            + "       ]"
            + "    }"
            + "}" );
         SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 04.04";

         // Configure VRI Source 1 for the infinite loop operation (steps 1,2,3 ...). No events
         sc_core::wait();
         wr_conf( std::string("")
            + "{ \"dest\":\"st_gen1\",\"idx\":0,\"cmd\":\"put\",\"data\":"
            + "    {\"exec_idx\":400,\"stat_idx\":0,\"conf_next\":0,\"valid_ovr\":" + std::to_string( rng_val()) + ","
            + "     \"vec_size\":\"rng\",\"slot_ena_ovr\":0,\"events\":"
            + "       ["
            + "       ]"
            + "    }"
            + "}" );
         SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 04.05";

         // Configure VRI Source 2 (step1). No events
         sc_core::wait();
         wr_conf( std::string("")
            + "{ \"dest\":\"st_gen2\",\"idx\":0,\"cmd\":\"put\",\"data\":"
            + "    {\"exec_idx\":400,\"stat_idx\":0,\"conf_next\":1,\"valid_ovr\":" + std::to_string( rng_val()) + ","
            + "     \"vec_size\":\"rng\",\"slot_ena_ovr\":0,\"events\":"
            + "       ["
            + "       ]"
            + "    }"
            + "}" );
         SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 04.06";

         // Configure VRI Source 2 (step2). Generate tail event.
         sc_core::wait();
         wr_conf( std::string("")
            + "{ \"dest\":\"st_gen2\",\"idx\":1,\"cmd\":\"put\",\"data\":"
            + "    {\"exec_idx\":999,\"stat_idx\":1,\"conf_next\":8,\"valid_ovr\":" + std::to_string( rng_val()) +","
            + "     \"vec_size\":\"rng\",\"slot_ena_ovr\":0,\"events\":"
            + "       [\"vec_tail\""
            + "       ]"
            + "    }"
            + "}" );
         SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 04.07";

         // Configure VRI Destination 1 for the infinite loop operation (steps 1,2,3 ...). No events
         sc_core::wait();
         wr_conf( std::string("")
            + "{ \"dest\":\"st_ana1\",\"idx\":0,\"cmd\":\"put\",\"data\":"
            + "    {\"exec_idx\":400,\"stat_idx\":0,\"conf_next\":0,\"ready_ovr\":0,\"events\":"
            + "       ["
            + "       ]"
            + "    }"
            + "}" );
         SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 04.08";

         // Configure VRI Destination 2 (step 1). No events
         sc_core::wait();
         wr_conf( std::string("")
            + "{ \"dest\":\"st_ana2\",\"idx\":0,\"cmd\":\"put\",\"data\":"
            + "    {\"exec_idx\":400,\"stat_idx\":0,\"conf_next\":1,\"ready_ovr\":0,\"events\":"
            + "       ["
            + "       ]"
            + "    }"
            + "}" );
         SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 04.09";

         // Configure VRI Destination 2 (step 2). No events
         sc_core::wait();
         wr_conf( std::string("")
            + "{ \"dest\":\"st_ana2\",\"idx\":1,\"cmd\":\"put\",\"data\":"
            + "    {\"exec_idx\":999,\"stat_idx\":1,\"conf_next\":2,\"ready_ovr\":0,\"events\":"
            + "       ["
            + "       ]"
            + "    }"
            + "}" );
         SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 04.10";

         // Configure VRI Destination 2 (step 3). Generate tail event
         sc_core::wait();
         wr_conf( std::string("")
            + "{ \"dest\":\"st_ana2\",\"idx\":2,\"cmd\":\"put\",\"data\":"
            + "    {\"exec_idx\":999,\"stat_idx\":2,\"conf_next\":8,\"ready_ovr\":1,\"events\":"
            + "       [\"vec_tail\""
            + "       ]"
            + "    }"
            + "}" );
         SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 04.11";

         // Run the succession of tests
         sc_core::wait();
         wr_conf( std::string("")
            + "{ \"dest\":\"broadcast\",\"idx\":400,\"cmd\":\"run\""
            + "}" );
         SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Start  04.12";

         // Wait for the completion events from the streaming modules and xbar
         evt_proc_init( std::string("")
            + "{\"valid_all\":"
            + "  [{\"mod\":\"st_gen2\",\"evt\":\"vec_tail\"},"
            + "   {\"mod\":\"st_ana2\",\"evt\":\"vec_tail\"},"
            + "   {\"mod\":\"xbar\",   \"evt\":\"st_gen1\" }"
            + "  ],"
            + " \"error_any\":"
            + "  [{\"mod\":\"st_ana1\",\"evt\":\"vec_head\"},"
            + "   {\"mod\":\"st_ana2\",\"evt\":\"vec_head\"},"
            + "   {\"mod\":\"st_gen1\",\"evt\":\"vec_head\"},"
            + "   {\"mod\":\"st_gen2\",\"evt\":\"vec_head\"},"
            + "   {\"mod\":\"st_gen1\",\"evt\":\"vec_tail\"},"
            + "   {\"mod\":\"st_ana1\",\"evt\":\"vec_tail\"},"
            + "   {\"mod\":\"xbar\",   \"evt\":\"st_gen2\" }"
            + "  ]"
            + "}" );

         for(;;) {
            sc_core::wait();

            if( event_i->num_available()) {
               boost_pt::ptree event_pt = event_i->read().get();

               evt_proc_t evt_proc = evt_proc_check( event_pt );

               if( evt_proc == EVT_PROC_VALID_ALL ) {
                  break;
               }
               else if( evt_proc != EVT_PROC_NONE ) {
                  SIMD_REPORT_ERROR( "simd::sys_scalar" ) << " Incorrect event source";
               }
            }
         }
         SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Check  04.13";

         // Read status
         sc_core::wait();
         {
            int st0_src1_vec_size = 0;
            int st0_src2_vec_size = 0;
            int st1_src2_vec_size = 0;
            int st0_dst1_vec_size = 0;
            int st0_dst2_vec_size = 0;
            int st1_dst2_vec_size = 0;
            int st2_dst2_vec_size = 0;

            try {
               simd_sig_ptree_c rd;
               std::string      src;

               //=== st_gen1, status[0]
               wr_conf( "{ \"dest\":\"st_gen1\",\"idx\":0,\"cmd\":\"get\"}" );
               sc_core::wait( 2 );

               busr_i->nb_read( rd );
               src = rd.get().get<std::string>( "source" );

               if( !( src == "st_gen1" )) {
                  SIMD_REPORT_ERROR( "simd::sys_scalar" ) << name() << " Incorrect sequence of responses";
               }

               st0_src1_vec_size = rd.get().get<int>( "vec_size" );

               //=== st_gen2, status[0]
               wr_conf( "{ \"dest\":\"st_gen2\",\"idx\":0,\"cmd\":\"get\"}" );
               sc_core::wait( 2 );

               busr_i->nb_read( rd );
               src = rd.get().get<std::string>( "source" );

               if( !( src == "st_gen2" )) {
                  SIMD_REPORT_ERROR( "simd::sys_scalar" ) << name() << " Incorrect sequence of responses";
               }

               st0_src2_vec_size = rd.get().get<int>( "vec_size" );

               //=== st_gen2, status[1]
               wr_conf( "{ \"dest\":\"st_gen2\",\"idx\":1,\"cmd\":\"get\"}" );
               sc_core::wait( 2 );

               busr_i->nb_read( rd );
               src = rd.get().get<std::string>( "source" );

               if( !( src == "st_gen2" )) {
                  SIMD_REPORT_ERROR( "simd::sys_scalar" ) << name() << " Incorrect sequence of responses";
               }

               st1_src2_vec_size = rd.get().get<int>( "vec_size" );

               //=== st_ana1, status[0]
               wr_conf( "{ \"dest\":\"st_ana1\",\"idx\":0,\"cmd\":\"get\"}" );
               sc_core::wait( 2 );

               busr_i->nb_read( rd );
               src = rd.get().get<std::string>( "source" );

               if( !( src == "st_ana1" )) {
                  SIMD_REPORT_ERROR( "simd::sys_scalar" ) << name() << " Incorrect sequence of responses";
               }

               st0_dst1_vec_size = rd.get().get<int>( "vec_size" );

               //=== st_ana2, status[0]
               wr_conf( "{ \"dest\":\"st_ana2\",\"idx\":0,\"cmd\":\"get\"}" );
               sc_core::wait( 2 );

               busr_i->nb_read( rd );
               src = rd.get().get<std::string>( "source" );

               if( !( src == "st_ana2" )) {
                  SIMD_REPORT_ERROR( "simd::sys_scalar" ) << name() << " Incorrect sequence of responses";
               }

               st0_dst2_vec_size = rd.get().get<int>( "vec_size" );

               //=== st_ana2, status[1]
               wr_conf( "{ \"dest\":\"st_ana2\",\"idx\":1,\"cmd\":\"get\"}" );
               sc_core::wait( 2 );

               busr_i->nb_read( rd );
               src = rd.get().get<std::string>( "source" );

               if( !( src == "st_ana2" )) {
                  SIMD_REPORT_ERROR( "simd::sys_scalar" ) << name() << " Incorrect sequence of responses";
               }

               st1_dst2_vec_size = rd.get().get<int>( "vec_size" );

               //=== st_ana2, status[2]
               wr_conf( "{ \"dest\":\"st_ana2\",\"idx\":2,\"cmd\":\"get\"}" );
               sc_core::wait( 2 );

               busr_i->nb_read( rd );
               src = rd.get().get<std::string>( "source" );

               if( !( src == "st_ana2" )) {
                  SIMD_REPORT_ERROR( "simd::sys_scalar" ) << name() << " Incorrect sequence of responses";
               }

               st2_dst2_vec_size = rd.get().get<int>( "vec_size" );
            }
            catch( const boost_pt::ptree_error& err ) {
               SIMD_REPORT_ERROR( "simd::sys_scalar" ) << name() << " " <<  err.what();
            }
            catch( ... ) {
               SIMD_REPORT_ERROR( "simd::sys_scalar" ) << name() << " Unexpected";
            }

            if( st0_src2_vec_size != st0_dst2_vec_size ) {
               SIMD_REPORT_ERROR( "simd::sys_scalar" ) << name() << " Vector sizes mismatch";
            }
         }
         SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Check  04.14";

         break;

      case 5:
         //*************************************
         // Test 05: Deferred execution (XBAR only).
         // Config1: SRC1->DST1
         // Run execution of Config1
         // Whilst Config1 is being executed create Config2 and run its execution
         // Config2: SRC1->DST2
         // Config2 execution should start after the execution of Config1 is finished
         //*************************************

         // Configure XBAR for the route src1->dst1.
         sc_core::wait();
         wr_conf( std::string("")
            + "{ \"dest\":\"xbar\",\"idx\":0,\"cmd\":\"put\",\"data\":"
            + "    {\"exec_idx\":500,\"conf_next\":8,\"routing\":"
            + "       [{\"mod\":\"st_gen1\",\"port\":0,\"event\":0,\"dst\":"
            + "           [{\"mod\":\"st_ana1\",\"port\":0,\"master\":1"
            + "            }"
            + "           ]"
            + "        }"
            + "       ]"
            + "    }"
            + "}" );
         SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 05.01";

         // Configure VRI Source 1 for the infinite loop operation (steps 1,2,3 ...). No events
         sc_core::wait();
         wr_conf( std::string("")
            + "{ \"dest\":\"st_gen1\",\"idx\":0,\"cmd\":\"put\",\"data\":"
            + "    {\"exec_idx\":500,\"stat_idx\":0,\"conf_next\":0,\"valid_ovr\":" + std::to_string( rng_val()) + ","
            + "     \"vec_size\":\"rng\",\"slot_ena_ovr\":0,\"events\":"
            + "       ["
            + "       ]"
            + "    }"
            + "}" );
         SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 05.02";

         // Configure VRI Destination 1
         sc_core::wait();
         wr_conf( std::string("")
            + "{ \"dest\":\"st_ana1\",\"idx\":0,\"cmd\":\"put\",\"data\":"
            + "    {\"exec_idx\":500,\"stat_idx\":0,\"conf_next\":8,\"ready_ovr\":0,\"events\":"
            + "       [\"vec_tail\""
            + "       ]"
            + "    }"
            + "}" );
         SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 05.03";

         // Run the test
         sc_core::wait();
         wr_conf( std::string("")
            + "{ \"dest\":\"broadcast\",\"idx\":500,\"cmd\":\"run\""
            + "}" );
         SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Start  05.04";

         // Configure XBAR for the route src1->dst2.
         sc_core::wait();
         wr_conf( std::string("")
            + "{ \"dest\":\"xbar\",\"idx\":1,\"cmd\":\"put\",\"data\":"
            + "    {\"exec_idx\":501,\"conf_next\":8,\"routing\":"
            + "       [{\"mod\":\"st_gen1\",\"port\":0,\"event\":0,\"dst\":"
            + "           [{\"mod\":\"st_ana2\",\"port\":0,\"master\":1"
            + "            }"
            + "           ]"
            + "        }"
            + "       ]"
            + "    }"
            + "}" );
         SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 05.05";

         // Configure VRI Destination 2
         sc_core::wait();
         wr_conf( std::string("")
            + "{ \"dest\":\"st_ana2\",\"idx\":1,\"cmd\":\"put\",\"data\":"
            + "    {\"exec_idx\":501,\"stat_idx\":0,\"conf_next\":8,\"ready_ovr\":1,\"events\":"
            + "       [\"vec_tail\""
            + "       ]"
            + "    }"
            + "}" );
         SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 05.06";

         // Run the test
         sc_core::wait();
         wr_conf( std::string("")
            + "{ \"dest\":\"broadcast\",\"idx\":501,\"cmd\":\"run\""
            + "}" );
         SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Start  05.07";

         // Wait for the completion events from the streaming modules and xbar
         evt_proc_init( std::string("")
            + "{\"valid_all\":"
            + "  [{\"mod\":\"st_ana1\",\"evt\":\"vec_tail\"},"
            + "   {\"mod\":\"st_ana2\",\"evt\":\"vec_tail\"}"
            + "  ],"
            + " \"error_any\":"
            + "  [{\"mod\":\"st_ana1\",\"evt\":\"vec_head\"},"
            + "   {\"mod\":\"st_ana2\",\"evt\":\"vec_head\"},"
            + "   {\"mod\":\"st_gen1\",\"evt\":\"vec_head\"},"
            + "   {\"mod\":\"st_gen2\",\"evt\":\"vec_head\"},"
            + "   {\"mod\":\"st_gen1\",\"evt\":\"vec_tail\"},"
            + "   {\"mod\":\"st_gen2\",\"evt\":\"vec_tail\"},"
            + "   {\"mod\":\"xbar\",   \"evt\":\"st_gen1\" },"
            + "   {\"mod\":\"xbar\",   \"evt\":\"st_gen2\" }"
            + "  ]"
            + "}" );

         for(;;) {
            sc_core::wait();

            if( event_i->num_available()) {
               boost_pt::ptree event_pt = event_i->read().get();

               evt_proc_t evt_proc = evt_proc_check( event_pt );

               if( evt_proc == EVT_PROC_VALID_ALL ) {
                  break;
               }
               else if( evt_proc != EVT_PROC_NONE ) {
                  SIMD_REPORT_ERROR( "simd::sys_scalar" ) << " Incorrect event source";
               }
            }
         }
         SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Check  05.08";

         break;

      case 6:
         //*************************************
         // Test 06: Deferred execution (XBAR and DM/EU).
         // Config1: SRC1->DST1, SRC2->DST2
         // Run execution of Config1
         // Whilst Config1 is being executed create Config2 and run its execution
         // Config2: SRC1->DST2, SRC2->DST1
         // Config2 execution should start after the execution of Config1 is finished
         //*************************************

         // Configure XBAR
         sc_core::wait();
         wr_conf( std::string("")
            + "{ \"dest\":\"xbar\",\"idx\":0,\"cmd\":\"put\",\"data\":"
            + "    {\"exec_idx\":600,\"conf_next\":8,\"routing\":"
            + "       [{\"mod\":\"st_gen1\",\"port\":0,\"event\":0,\"dst\":"
            + "           [{\"mod\":\"st_ana1\",\"port\":0,\"master\":1"
            + "            }"
            + "           ]"
            + "        },"
            + "        {\"mod\":\"st_gen2\",\"port\":0,\"event\":0,\"dst\":"
            + "           [{\"mod\":\"st_ana2\",\"port\":0,\"master\":1"
            + "            }"
            + "           ]"
            + "        }"
            + "       ]"
            + "    }"
            + "}" );
         SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 06.01";

         // Configure VRI Source 1
         sc_core::wait();
         wr_conf( std::string("")
            + "{ \"dest\":\"st_gen1\",\"idx\":0,\"cmd\":\"put\",\"data\":"
            + "    {\"exec_idx\":600,\"stat_idx\":0,\"conf_next\":8,\"valid_ovr\":" + std::to_string( rng_val()) + ","
            + "     \"vec_size\":\"rng\",\"slot_ena_ovr\":0,\"events\":"
            + "       ["
            + "       ]"
            + "    }"
            + "}" );
         SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 06.02";

         // Configure VRI Source 2
         sc_core::wait();
         wr_conf( std::string("")
            + "{ \"dest\":\"st_gen2\",\"idx\":0,\"cmd\":\"put\",\"data\":"
            + "    {\"exec_idx\":600,\"stat_idx\":0,\"conf_next\":8,\"valid_ovr\":" + std::to_string( rng_val()) + ","
            + "     \"vec_size\":\"rng\",\"slot_ena_ovr\":0,\"events\":"
            + "       ["
            + "       ]"
            + "    }"
            + "}" );
         SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 06.03";

         // Configure VRI Destination 1
         sc_core::wait();
         wr_conf( std::string("")
            + "{ \"dest\":\"st_ana1\",\"idx\":0,\"cmd\":\"put\",\"data\":"
            + "    {\"exec_idx\":600,\"stat_idx\":0,\"conf_next\":8,\"ready_ovr\":0,\"events\":"
            + "       ["
            + "       ]"
            + "    }"
            + "}" );
         SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 06.04";

         // Configure VRI Destination 2
         sc_core::wait();
         wr_conf( std::string("")
            + "{ \"dest\":\"st_ana2\",\"idx\":0,\"cmd\":\"put\",\"data\":"
            + "    {\"exec_idx\":600,\"stat_idx\":0,\"conf_next\":8,\"ready_ovr\":0,\"events\":"
            + "       ["
            + "       ]"
            + "    }"
            + "}" );
         SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 06.05";

         // Run
         sc_core::wait();
         wr_conf( std::string("")
            + "{ \"dest\":\"broadcast\",\"idx\":600,\"cmd\":\"run\""
            + "}" );
         SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Start  06.06";

         // Configure XBAR
         sc_core::wait();
         wr_conf( std::string("")
            + "{ \"dest\":\"xbar\",\"idx\":1,\"cmd\":\"put\",\"data\":"
            + "    {\"exec_idx\":601,\"conf_next\":8,\"routing\":"
            + "       [{\"mod\":\"st_gen1\",\"port\":0,\"event\":0,\"dst\":"
            + "           [{\"mod\":\"st_ana2\",\"port\":0,\"master\":1"
            + "            }"
            + "           ]"
            + "        },"
            + "        {\"mod\":\"st_gen2\",\"port\":0,\"event\":0,\"dst\":"
            + "           [{\"mod\":\"st_ana1\",\"port\":0,\"master\":1"
            + "            }"
            + "           ]"
            + "        }"
            + "       ]"
            + "    }"
            + "}" );
         SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 06.07";

         // Configure VRI Source 1
         sc_core::wait();
         wr_conf( std::string("")
            + "{ \"dest\":\"st_gen1\",\"idx\":1,\"cmd\":\"put\",\"data\":"
            + "    {\"exec_idx\":601,\"stat_idx\":0,\"conf_next\":8,\"valid_ovr\":" + std::to_string( rng_val()) + ","
            + "     \"vec_size\":\"rng\",\"slot_ena_ovr\":0,\"events\":"
            + "       [\"vec_tail\""
            + "       ]"
            + "    }"
            + "}" );
         SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 06.08";

         // Configure VRI Source 2
         sc_core::wait();
         wr_conf( std::string("")
            + "{ \"dest\":\"st_gen2\",\"idx\":1,\"cmd\":\"put\",\"data\":"
            + "    {\"exec_idx\":601,\"stat_idx\":0,\"conf_next\":8,\"valid_ovr\":" + std::to_string( rng_val()) + ","
            + "     \"vec_size\":\"rng\",\"slot_ena_ovr\":0,\"events\":"
            + "       [\"vec_tail\""
            + "       ]"
            + "    }"
            + "}" );
         SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 06.09";

         // Configure VRI Destination 1
         sc_core::wait();
         wr_conf( std::string("")
            + "{ \"dest\":\"st_ana1\",\"idx\":1,\"cmd\":\"put\",\"data\":"
            + "    {\"exec_idx\":601,\"stat_idx\":0,\"conf_next\":8,\"ready_ovr\":0,\"events\":"
            + "       [\"vec_tail\""
            + "       ]"
            + "    }"
            + "}" );
         SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 06.10";

         // Configure VRI Destination 2
         sc_core::wait();
         wr_conf( std::string("")
            + "{ \"dest\":\"st_ana2\",\"idx\":1,\"cmd\":\"put\",\"data\":"
            + "    {\"exec_idx\":601,\"stat_idx\":0,\"conf_next\":8,\"ready_ovr\":0,\"events\":"
            + "       [\"vec_tail\""
            + "       ]"
            + "    }"
            + "}" );
         SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 06.11";

         // Run
         sc_core::wait();
         wr_conf( std::string("")
            + "{ \"dest\":\"broadcast\",\"idx\":601,\"cmd\":\"run\""
            + "}" );
         SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Start  06.12";

         // Wait for the completion events from the streaming modules and xbar
         evt_proc_init( std::string("")
            + "{\"valid_all\":"
            + "  [{\"mod\":\"st_ana1\",\"evt\":\"vec_tail\"},"
            + "   {\"mod\":\"st_ana2\",\"evt\":\"vec_tail\"},"
            + "   {\"mod\":\"st_gen1\",\"evt\":\"vec_tail\"},"
            + "   {\"mod\":\"st_gen2\",\"evt\":\"vec_tail\"}"
            + "  ],"
            + " \"error_any\":"
            + "  [{\"mod\":\"st_ana1\",\"evt\":\"vec_head\"},"
            + "   {\"mod\":\"st_ana2\",\"evt\":\"vec_head\"},"
            + "   {\"mod\":\"st_gen1\",\"evt\":\"vec_head\"},"
            + "   {\"mod\":\"st_gen2\",\"evt\":\"vec_head\"},"
            + "   {\"mod\":\"xbar\",   \"evt\":\"st_gen1\" },"
            + "   {\"mod\":\"xbar\",   \"evt\":\"st_gen2\" }"
            + "  ]"
            + "}" );

         for(;;) {
            sc_core::wait();

            if( event_i->num_available()) {
               boost_pt::ptree event_pt = event_i->read().get();

               evt_proc_t evt_proc = evt_proc_check( event_pt );

               if( evt_proc == EVT_PROC_VALID_ALL ) {
                  break;
               }
               else if( evt_proc != EVT_PROC_NONE ) {
                  SIMD_REPORT_ERROR( "simd::sys_scalar" ) << " Incorrect event source";
               }
            }
         }

         SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Check  06.13";
         break;

      case 7:
         //*************************************
         // Test 07: Execution priorities.
         // Config1: SRC2->DST2
         // Run execution of Config1
         // Config2, step1: SRC1->DST1
         // Config2, step2: SRC1->DST2
         // Run execution of Config1
         // Whilst Config1 is being executed create Config2 and run its execution
         // Config3: SRC1->DST1
         // Config3 execution should start after the execution of Config2 step2 is finished
         //*************************************

         // Configure XBAR for the route src2->dst2.
         sc_core::wait();
         wr_conf( std::string("")
            + "{ \"dest\":\"xbar\",\"idx\":0,\"cmd\":\"put\",\"data\":"
            + "    {\"exec_idx\":700,\"conf_next\":8,\"routing\":"
            + "       [{\"mod\":\"st_gen2\",\"port\":0,\"event\":0,\"dst\":"
            + "           [{\"mod\":\"st_ana2\",\"port\":0,\"master\":1"
            + "            }"
            + "           ]"
            + "        }"
            + "       ]"
            + "    }"
            + "}" );
         SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 07.01";

         // Configure VRI Source 2
         sc_core::wait();
         wr_conf( std::string("")
            + "{ \"dest\":\"st_gen2\",\"idx\":0,\"cmd\":\"put\",\"data\":"
            + "    {\"exec_idx\":700,\"stat_idx\":0,\"conf_next\":8,\"valid_ovr\":1," // Run fast
            + "     \"vec_size\":\"rng\",\"slot_ena_ovr\":0,\"events\":"
            + "       [\"vec_tail\""
            + "       ]"
            + "    }"
            + "}" );
         SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 07.02";

         // Configure VRI Destination 2
         sc_core::wait();
         wr_conf( std::string("")
            + "{ \"dest\":\"st_ana2\",\"idx\":0,\"cmd\":\"put\",\"data\":"
            + "    {\"exec_idx\":700,\"stat_idx\":0,\"conf_next\":8,\"ready_ovr\":1,\"events\":" // Run fast
            + "       [\"vec_tail\""
            + "       ]"
            + "    }"
            + "}" );
         SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 07.03";

         // Run
         sc_core::wait();
         wr_conf( std::string("")
            + "{ \"dest\":\"broadcast\",\"idx\":700,\"cmd\":\"run\""
            + "}" );
         SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Start  07.04";

         // Configure XBAR for the route src1->dst1.
         sc_core::wait();
         wr_conf( std::string("")
            + "{ \"dest\":\"xbar\",\"idx\":1,\"cmd\":\"put\",\"data\":"
            + "    {\"exec_idx\":701,\"conf_next\":2,\"routing\":"
            + "       [{\"mod\":\"st_gen1\",\"port\":0,\"event\":0,\"dst\":"
            + "           [{\"mod\":\"st_ana1\",\"port\":0,\"master\":1"
            + "            }"
            + "           ]"
            + "        }"
            + "       ]"
            + "    }"
            + "}" );
         SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 07.05";

         // Configure XBAR for the route src1->dst2.
         sc_core::wait();
         wr_conf( std::string("")
            + "{ \"dest\":\"xbar\",\"idx\":2,\"cmd\":\"put\",\"data\":"
            + "    {\"exec_idx\":799,\"conf_next\":8,\"routing\":"
            + "       [{\"mod\":\"st_gen1\",\"port\":0,\"event\":0,\"dst\":"
            + "           [{\"mod\":\"st_ana2\",\"port\":0,\"master\":1"
            + "            }"
            + "           ]"
            + "        }"
            + "       ]"
            + "    }"
            + "}" );
         SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 07.06";

         // Configure VRI Source 1
         sc_core::wait();
         wr_conf( std::string("")
            + "{ \"dest\":\"st_gen1\",\"idx\":1,\"cmd\":\"put\",\"data\":"
            + "    {\"exec_idx\":701,\"stat_idx\":0,\"conf_next\":2,\"valid_ovr\":0," // Run slow
            + "     \"vec_size\":\"rng\",\"slot_ena_ovr\":0,\"events\":"
            + "       [\"vec_tail\""
            + "       ]"
            + "    }"
            + "}" );
         SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 07.07";

         // Configure VRI Source 1
         sc_core::wait();
         wr_conf( std::string("")
            + "{ \"dest\":\"st_gen1\",\"idx\":2,\"cmd\":\"put\",\"data\":"
            + "    {\"exec_idx\":799,\"stat_idx\":0,\"conf_next\":8,\"valid_ovr\":" + std::to_string( rng_val()) + ","
            + "     \"vec_size\":\"rng\",\"slot_ena_ovr\":0,\"events\":"
            + "       [\"vec_tail\""
            + "       ]"
            + "    }"
            + "}" );
         SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 07.08";

         // Configure VRI Destination 1
         sc_core::wait();
         wr_conf( std::string("")
            + "{ \"dest\":\"st_ana1\",\"idx\":1,\"cmd\":\"put\",\"data\":"
            + "    {\"exec_idx\":701,\"stat_idx\":0,\"conf_next\":8,\"ready_ovr\":0,\"events\":"
            + "       [\"vec_tail\""
            + "       ]"
            + "    }"
            + "}" );
         SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 07.09";

         // Configure VRI Destination 2
         sc_core::wait();
         wr_conf( std::string("")
            + "{ \"dest\":\"st_ana2\",\"idx\":2,\"cmd\":\"put\",\"data\":"
            + "    {\"exec_idx\":701,\"stat_idx\":0,\"conf_next\":8,\"ready_ovr\":0,\"events\":"
            + "       [\"vec_tail\""
            + "       ]"
            + "    }"
            + "}" );
         SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 07.10";

         // Run
         sc_core::wait();
         wr_conf( std::string("")
            + "{ \"dest\":\"broadcast\",\"idx\":701,\"cmd\":\"run\""
            + "}" );
         SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Start  07.11";

         // Configure XBAR for the route src1->dst1.
         sc_core::wait();
         wr_conf( std::string("")
            + "{ \"dest\":\"xbar\",\"idx\":3,\"cmd\":\"put\",\"data\":"
            + "    {\"exec_idx\":703,\"conf_next\":8,\"routing\":"
            + "       [{\"mod\":\"st_gen1\",\"port\":0,\"event\":0,\"dst\":"
            + "           [{\"mod\":\"st_ana1\",\"port\":0,\"master\":1"
            + "            }"
            + "           ]"
            + "        }"
            + "       ]"
            + "    }"
            + "}" );
         SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 07.12";

         // Configure VRI Source 1
         sc_core::wait();
         wr_conf( std::string("")
            + "{ \"dest\":\"st_gen1\",\"idx\":3,\"cmd\":\"put\",\"data\":"
            + "    {\"exec_idx\":703,\"stat_idx\":0,\"conf_next\":8,\"valid_ovr\":" + std::to_string( rng_val()) + ","
            + "     \"vec_size\":\"rng\",\"slot_ena_ovr\":0,\"events\":"
            + "       [\"vec_tail\""
            + "       ]"
            + "    }"
            + "}" );
         SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 07.13";

         // Configure VRI Destination 1
         sc_core::wait();
         wr_conf( std::string("")
            + "{ \"dest\":\"st_ana1\",\"idx\":3,\"cmd\":\"put\",\"data\":"
            + "    {\"exec_idx\":703,\"stat_idx\":0,\"conf_next\":8,\"ready_ovr\":0,\"events\":"
            + "       [\"vec_tail\""
            + "       ]"
            + "    }"
            + "}" );
         SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 07.14";

         // Run
         sc_core::wait();
         wr_conf( std::string("")
            + "{ \"dest\":\"broadcast\",\"idx\":703,\"cmd\":\"run\""
            + "}" );
         SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Start  07.15";

         // Wait for the completion events from the streaming modules and xbar
         evt_proc_init( std::string("")
            + "{\"valid_all\":"
            + "  [{\"mod\":\"st_ana2\",\"evt\":\"vec_tail\"},"
            + "   {\"mod\":\"st_gen2\",\"evt\":\"vec_tail\"}"
            + "  ],"
            + " \"error_any\":"
            + "  [{\"mod\":\"st_ana1\",\"evt\":\"vec_head\"},"
            + "   {\"mod\":\"st_ana2\",\"evt\":\"vec_head\"},"
            + "   {\"mod\":\"st_gen1\",\"evt\":\"vec_head\"},"
            + "   {\"mod\":\"st_gen2\",\"evt\":\"vec_head\"},"
            + "   {\"mod\":\"st_gen1\",\"evt\":\"vec_tail\"},"
            + "   {\"mod\":\"st_ana1\",\"evt\":\"vec_tail\"},"
            + "   {\"mod\":\"xbar\",   \"evt\":\"st_gen1\" },"
            + "   {\"mod\":\"xbar\",   \"evt\":\"st_gen2\" }"
            + "  ]"
            + "}" );

         for(;;) {
            sc_core::wait();

            if( event_i->num_available()) {
               boost_pt::ptree event_pt = event_i->read().get();

               evt_proc_t evt_proc = evt_proc_check( event_pt );

               if( evt_proc == EVT_PROC_VALID_ALL ) {
                  break;
               }
               else if( evt_proc != EVT_PROC_NONE ) {
                  SIMD_REPORT_ERROR( "simd::sys_scalar" ) << " Incorrect event source";
               }
            }
         }
         SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Check  07.16";

         // Wait for the completion events from the streaming modules and xbar
         evt_proc_init( std::string("")
            + "{\"valid_all\":"
            + "  [{\"mod\":\"st_ana1\",\"evt\":\"vec_tail\"},"
            + "   {\"mod\":\"st_gen1\",\"evt\":\"vec_tail\"}"
            + "  ],"
            + " \"error_any\":"
            + "  [{\"mod\":\"st_ana1\",\"evt\":\"vec_head\"},"
            + "   {\"mod\":\"st_ana2\",\"evt\":\"vec_head\"},"
            + "   {\"mod\":\"st_gen1\",\"evt\":\"vec_head\"},"
            + "   {\"mod\":\"st_gen2\",\"evt\":\"vec_head\"},"
            + "   {\"mod\":\"st_gen2\",\"evt\":\"vec_tail\"},"
            + "   {\"mod\":\"st_ana2\",\"evt\":\"vec_tail\"},"
            + "   {\"mod\":\"xbar\",   \"evt\":\"st_gen1\" },"
            + "   {\"mod\":\"xbar\",   \"evt\":\"st_gen2\" }"
            + "  ]"
            + "}" );

         for(;;) {
            sc_core::wait();

            if( event_i->num_available()) {
               boost_pt::ptree event_pt = event_i->read().get();

               evt_proc_t evt_proc = evt_proc_check( event_pt );

               if( evt_proc == EVT_PROC_VALID_ALL ) {
                  break;
               }
               else if( evt_proc != EVT_PROC_NONE ) {
                  SIMD_REPORT_ERROR( "simd::sys_scalar" ) << " Incorrect event source";
               }
            }
         }
         SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Check  07.17";

         // Wait for the completion events from the streaming modules and xbar
         evt_proc_init( std::string("")
            + "{\"valid_all\":"
            + "  [{\"mod\":\"st_ana2\",\"evt\":\"vec_tail\"},"
            + "   {\"mod\":\"st_gen1\",\"evt\":\"vec_tail\"}"
            + "  ],"
            + " \"error_any\":"
            + "  [{\"mod\":\"st_ana1\",\"evt\":\"vec_head\"},"
            + "   {\"mod\":\"st_ana2\",\"evt\":\"vec_head\"},"
            + "   {\"mod\":\"st_gen1\",\"evt\":\"vec_head\"},"
            + "   {\"mod\":\"st_gen2\",\"evt\":\"vec_head\"},"
            + "   {\"mod\":\"st_gen2\",\"evt\":\"vec_tail\"},"
            + "   {\"mod\":\"st_ana1\",\"evt\":\"vec_tail\"},"
            + "   {\"mod\":\"xbar\",   \"evt\":\"st_gen1\" },"
            + "   {\"mod\":\"xbar\",   \"evt\":\"st_gen2\" }"
            + "  ]"
            + "}" );

         for(;;) {
            sc_core::wait();

            if( event_i->num_available()) {
               boost_pt::ptree event_pt = event_i->read().get();

               evt_proc_t evt_proc = evt_proc_check( event_pt );

               if( evt_proc == EVT_PROC_VALID_ALL ) {
                  break;
               }
               else if( evt_proc != EVT_PROC_NONE ) {
                  SIMD_REPORT_ERROR( "simd::sys_scalar" ) << " Incorrect event source";
               }
            }
         }
         SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Check  07.18";

         evt_proc_init( std::string("")
            + "{\"valid_all\":"
            + "  [{\"mod\":\"st_ana1\",\"evt\":\"vec_tail\"},"
            + "   {\"mod\":\"st_gen1\",\"evt\":\"vec_tail\"}"
            + "  ],"
            + " \"error_any\":"
            + "  [{\"mod\":\"st_ana1\",\"evt\":\"vec_head\"},"
            + "   {\"mod\":\"st_ana2\",\"evt\":\"vec_head\"},"
            + "   {\"mod\":\"st_gen1\",\"evt\":\"vec_head\"},"
            + "   {\"mod\":\"st_gen2\",\"evt\":\"vec_head\"},"
            + "   {\"mod\":\"st_gen2\",\"evt\":\"vec_tail\"},"
            + "   {\"mod\":\"st_ana2\",\"evt\":\"vec_tail\"},"
            + "   {\"mod\":\"xbar\",   \"evt\":\"st_gen1\" },"
            + "   {\"mod\":\"xbar\",   \"evt\":\"st_gen2\" }"
            + "  ]"
            + "}" );

         for(;;) {
            sc_core::wait();

            if( event_i->num_available()) {
               boost_pt::ptree event_pt = event_i->read().get();

               evt_proc_t evt_proc = evt_proc_check( event_pt );

               if( evt_proc == EVT_PROC_VALID_ALL ) {
                  break;
               }
               else if( evt_proc != EVT_PROC_NONE ) {
                  SIMD_REPORT_ERROR( "simd::sys_scalar" ) << " Incorrect event source";
               }
            }
         }
         SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Check  07.19";
         break;

      default:
         SIMD_REPORT_ERROR( "simd::sys_scalar" ) << name() << " Incorrect test index";
         break;

      } // switch random

   } // for(;;)
}

} // namespace simd
