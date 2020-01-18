/*
 * simd_sys_scalar_run.cpp
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

   boost_rn::uniform_int_distribution<> rng_dist_uni( 16, 255 );
   boost_rn::variate_generator<boost::mt19937&, boost_rn::uniform_int_distribution<> > rng_size( rng_eng, rng_dist_uni );

   boost_rn::bernoulli_distribution <>   rng_dist_bern( 0.5 );
   boost_rn::variate_generator<boost::mt19937&, boost_rn::bernoulli_distribution<> >   rng_val( rng_eng, rng_dist_bern );
   boost_rn::variate_generator<boost::mt19937&, boost_rn::bernoulli_distribution<> >   rng_rdy( rng_eng, rng_dist_bern );

   sc_core::wait();
   sc_core::wait();

   for( n_test = 0; ; n_test ++ ) {

      SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Start test " << n_test;

      unsigned int vec_size1 = rng_size();
      unsigned int vec_size2 = rng_size();

      //*************************************
      // Test 01: Execute 3 steps under the distributed control of the vector core.
      // Step1: SRC1->DM1(LO),DST1
      // Step2: DM1(LO)->DM2,DST2; SRC1->DM1(HI),DST1
      // Step3: DM2->DST2, DM1(LO); DM1(HI)->DST1
      //*************************************

      // Configure XBAR for step1
      sc_core::wait();
      wr_conf( std::string("")
         + "{ \"dst\":[\"xbar\"],\"idx\":0,\"cmd\":\"put\",\"data\":"
         + "    {\"exec_idx\":100,\"conf_next\":1,\"routing\":"
         + "       [{\"mod\":\"st_gen1\",\"port\":0,\"event\":0,\"dst\":"
         + "           [{\"mod\":\"dm_1\",   \"port\":0,\"master\":0"
         + "            },"
         + "            {\"mod\":\"st_ana1\",\"port\":0,\"master\":1"
         + "            }"
         + "           ]"
         + "        }"
         + "       ]"
         + "    }"
         + "}" );
      SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 01.01";

      // Configure VRI Source 1 for step1
      sc_core::wait();
      wr_conf( std::string("")
         + "{ \"dst\":[\"st_gen1\"],\"idx\":0,\"cmd\":\"put\",\"data\":"
         + "    {\"exec_idx\":100,\"stat_idx\":0,\"conf_next\":1,\"events\":[],"
         + "     \"vec_size\":" + std::to_string( vec_size1 ) + ",\"slot_ena_ovr\":1,\"valid_ovr\":" + std::to_string( rng_val())
         + "    }"
         + "}" );
      SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 01.02";

      // DM 1 for step1
      sc_core::wait();
      wr_conf( std::string("")
         + "{ \"dst\":[\"dm_1\"],\"idx\":0,\"cmd\":\"put\",\"data\":"
         + "    {\"exec_idx\":100,\"stat_idx\":0,\"conf_next\":1,\"events\":[],"
         + "     \"op_mode\":\"write\",\"wr_smp_num\":" + std::to_string( vec_size1 ) + ",\"wr_ag_conf\":"
         + "       {\"mode\":\"linear\", \"param\":"
         + "          [{\"offset\":0, \"ena\":1, \"perm\":0 },"
         + "           {\"offset\":0, \"ena\":1, \"perm\":1 },"
         + "           {\"offset\":0, \"ena\":1, \"perm\":2 },"
         + "           {\"offset\":0, \"ena\":1, \"perm\":3 } "
         + "          ]"
         + "       }"
         + "    }"
         + "}" );
      SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 01.03";

      // Configure VRI Destination 1
      sc_core::wait();
      wr_conf( std::string("")
         + "{ \"dst\":[\"st_ana1\"],\"idx\":0,\"cmd\":\"put\",\"data\":"
         + "    {\"exec_idx\":100,\"stat_idx\":0,\"conf_next\":1,\"events\":[]," // no events for this stage
         + "     \"ready_ovr\":" + std::to_string( rng_val())
         + "    }"
         + "}" );
      SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 01.04";

      // Configure XBAR for step2
      sc_core::wait();
      wr_conf( std::string("")
         + "{ \"dst\":[\"xbar\"],\"idx\":1,\"cmd\":\"put\",\"data\":"
         + "    {\"exec_idx\":999,\"conf_next\":2,\"routing\":"
         + "       [{\"mod\":\"dm_1\",\"port\":0,\"event\":0,\"dst\":"
         + "           [{\"mod\":\"dm_2\",   \"port\":0,\"master\":0"
         + "            },"
         + "            {\"mod\":\"st_ana2\",\"port\":0,\"master\":1"
         + "            }"
         + "           ]"
         + "        },"
         + "        {\"mod\":\"st_gen1\",\"port\":0,\"event\":0,\"dst\":"
         + "           [{\"mod\":\"dm_1\",   \"port\":0,\"master\":1"
         + "            },"
         + "            {\"mod\":\"st_ana1\",\"port\":0,\"master\":0"
         + "            }"
         + "           ]"
         + "        }"
         + "       ]"
         + "    }"
         + "}" );
      SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 01.05";

      // Configure VRI Source 1 for step2
      sc_core::wait();
      wr_conf( std::string("")
         + "{ \"dst\":[\"st_gen1\"],\"idx\":1,\"cmd\":\"put\",\"data\":"
         + "    {\"exec_idx\":999,\"stat_idx\":0,\"conf_next\":8,\"events\":[],"
         + "     \"vec_size\":" + std::to_string( vec_size2 ) + ",\"slot_ena_ovr\":1,\"valid_ovr\":" + std::to_string( rng_val())
         + "    }"
         + "}" );
      SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 01.06";


      // DM 1 for step2
      sc_core::wait();
      wr_conf( std::string("")
         + "{ \"dst\":[\"dm_1\"],\"idx\":1,\"cmd\":\"put\",\"data\":"
         + "    {\"exec_idx\":999,\"stat_idx\":0,\"conf_next\":2,\"events\":[],"
         + "     \"op_mode\":\"rd_wr\","
         + "     \"rd_smp_num\":" + std::to_string( vec_size1 ) + ",\"rd_ag_conf\":"
         + "       {\"mode\":\"linear\", \"param\":"
         + "          [{\"offset\":0,   \"ena\":1, \"perm\":0 },"
         + "           {\"offset\":0,   \"ena\":1, \"perm\":1 },"
         + "           {\"offset\":0,   \"ena\":1, \"perm\":2 },"
         + "           {\"offset\":0,   \"ena\":1, \"perm\":3 } "
         + "          ]"
         + "       },"
         + "     \"wr_smp_num\":" + std::to_string( vec_size2 ) + ",\"wr_ag_conf\":"
         + "       {\"mode\":\"linear\", \"param\":"
         + "          [{\"offset\":256, \"ena\":1, \"perm\":0 },"
         + "           {\"offset\":256, \"ena\":1, \"perm\":1 },"
         + "           {\"offset\":256, \"ena\":1, \"perm\":2 },"
         + "           {\"offset\":256, \"ena\":1, \"perm\":3 } "
         + "          ]"
         + "       }"
         + "    }"
         + "}" );
      SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 01.07";

      // Configure VRI Destination 1 for step 2
      sc_core::wait();
      wr_conf( std::string("")
         + "{ \"dst\":[\"st_ana1\"],\"idx\":1,\"cmd\":\"put\",\"data\":"
         + "    {\"exec_idx\":999,\"stat_idx\":0,\"conf_next\":2,\"events\":[],"
         + "     \"ready_ovr\":1" // In this chain dm_1 serves as master
         + "    }"
         + "}" );
      SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 01.08";

      // DM 2 for step2 (starts with step 1)
      sc_core::wait();
      wr_conf( std::string("")
         + "{ \"dst\":[\"dm_2\"],\"idx\":1,\"cmd\":\"put\",\"data\":"
         + "    {\"exec_idx\":100,\"stat_idx\":0,\"conf_next\":2,\"events\":[],"
         + "     \"op_mode\":\"write\",\"smp_num\":" + std::to_string( vec_size1 ) + ",\"ag_conf\":"
         + "       {\"mode\":\"linear\", \"param\":"
         + "          [{\"offset\":0, \"ena\":1, \"perm\":0 },"
         + "           {\"offset\":0, \"ena\":1, \"perm\":1 },"
         + "           {\"offset\":0, \"ena\":1, \"perm\":2 },"
         + "           {\"offset\":0, \"ena\":1, \"perm\":3 } "
         + "          ]"
         + "       }"
         + "    }"
         + "}" );
      SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 01.09";

      // Configure VRI Destination 2 for step 2 (starts with step 1)
      sc_core::wait();
      wr_conf( std::string("")
         + "{ \"dst\":[\"st_ana2\"],\"idx\":1,\"cmd\":\"put\",\"data\":"
         + "    {\"exec_idx\":100,\"stat_idx\":0,\"conf_next\":2,\"events\":[],"
         + "     \"ready_ovr\":" + std::to_string( rng_val())
         + "    }"
         + "}" );
      SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 01.10";

      // Configure XBAR for step3
      sc_core::wait();
      wr_conf( std::string("")
         + "{ \"dst\":[\"xbar\"],\"idx\":2,\"cmd\":\"put\",\"data\":"
         + "    {\"exec_idx\":999,\"conf_next\":8,\"routing\":"
         + "       [{\"mod\":\"dm_2\",\"port\":0,\"event\":1,\"dst\":"
         + "           [{\"mod\":\"dm_1\",   \"port\":0,\"master\":0"
         + "            },"
         + "            {\"mod\":\"st_ana2\",\"port\":0,\"master\":1"
         + "            }"
         + "           ]"
         + "        },"
         + "        {\"mod\":\"dm_1\",\"port\":0,\"event\":1,\"dst\":"
         + "           [{\"mod\":\"st_ana1\",\"port\":0,\"master\":1"
         + "            }"
         + "           ]"
         + "        }"
         + "       ]"
         + "    }"
         + "}" );
      SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 01.11";

      // DM 2 for step3
      sc_core::wait();
      wr_conf( std::string("")
         + "{ \"dst\":[\"dm_2\"],\"idx\":2,\"cmd\":\"put\",\"data\":"
         + "    {\"exec_idx\":999,\"stat_idx\":0,\"conf_next\":8,\"events\":"
         + "       [\"rd_vec_head\",\"rd_vec_tail\""
         + "       ],"
         + "     \"op_mode\":\"read\",\"smp_num\":" + std::to_string( vec_size1 ) + ",\"ag_conf\":"
         + "       {\"mode\":\"linear\", \"param\":"
         + "          [{\"offset\":0, \"ena\":1, \"perm\":0 },"
         + "           {\"offset\":0, \"ena\":1, \"perm\":1 },"
         + "           {\"offset\":0, \"ena\":1, \"perm\":2 },"
         + "           {\"offset\":0, \"ena\":1, \"perm\":3 } "
         + "          ]"
         + "       }"
         + "    }"
         + "}" );
      SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 01.12";

      // DM 1 for step3
      sc_core::wait();
      wr_conf( std::string("")
         + "{ \"dst\":[\"dm_1\"],\"idx\":2,\"cmd\":\"put\",\"data\":"
         + "    {\"exec_idx\":999,\"stat_idx\":0,\"conf_next\":8,\"events\":"
         + "       [\"wr_vec_head\",\"wr_vec_tail\",\"rd_vec_head\",\"rd_vec_tail\""
         + "       ],"
         + "     \"op_mode\":\"rd_wr\","
         +"      \"wr_smp_num\":" + std::to_string( vec_size1 ) + ",\"wr_ag_conf\":"
         + "       {\"mode\":\"linear\", \"param\":"
         + "          [{\"offset\":0,   \"ena\":1, \"perm\":0 },"
         + "           {\"offset\":0,   \"ena\":1, \"perm\":1 },"
         + "           {\"offset\":0,   \"ena\":1, \"perm\":2 },"
         + "           {\"offset\":0,   \"ena\":1, \"perm\":3 } "
         + "          ]"
         + "       },"
         +"      \"rd_smp_num\":" + std::to_string( vec_size2 ) + ",\"rd_ag_conf\":"
         + "       {\"mode\":\"linear\", \"param\":"
         + "          [{\"offset\":256, \"ena\":1, \"perm\":0 },"
         + "           {\"offset\":256, \"ena\":1, \"perm\":1 },"
         + "           {\"offset\":256, \"ena\":1, \"perm\":2 },"
         + "           {\"offset\":256, \"ena\":1, \"perm\":3 } "
         + "          ]"
         + "       }"
         + "    }"
         + "}" );
      SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 01.13";

      // Configure VRI Destination 1 for step 3
      sc_core::wait();
      wr_conf( std::string("")
         + "{ \"dst\":[\"st_ana1\"],\"idx\":2,\"cmd\":\"put\",\"data\":"
         + "    {\"exec_idx\":999,\"stat_idx\":0,\"conf_next\":8,\"events\":"
         + "       [\"vec_head\",\"vec_tail\""
         + "       ],"
         + "     \"ready_ovr\":" + std::to_string( rng_val())
         + "    }"
         + "}" );
      SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 01.12";

      // Configure VRI Destination 2 for step 3
      sc_core::wait();
      wr_conf( std::string("")
         + "{ \"dst\":[\"st_ana2\"],\"idx\":2,\"cmd\":\"put\",\"data\":"
         + "    {\"exec_idx\":999,\"stat_idx\":0,\"conf_next\":8,\"events\":"
         + "       [\"vec_head\",\"vec_tail\""
         + "       ],"
         + "     \"ready_ovr\":" + std::to_string( rng_val())
         + "    }"
         + "}" );
      SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 01.14";

      // Run
      sc_core::wait();
      wr_conf( std::string("")
         + "{ \"dst\":\"broadcast\",\"idx\":100,\"cmd\":\"run\""
         + "}" );
      SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Start  01.15";

      // Wait for the completion events from the modules and xbar
      evt_proc_init( std::string("")
         + "{\"valid_all\":"
         + "  [{\"src\":\"st_ana1\",\"evt\":\"vec_head\"   },"
         + "   {\"src\":\"st_ana2\",\"evt\":\"vec_head\"   },"
         + "   {\"src\":\"dm_1\",   \"evt\":\"rd_vec_head\"},"
         + "   {\"src\":\"dm_1\",   \"evt\":\"wr_vec_head\"},"
         + "   {\"src\":\"dm_2\",   \"evt\":\"rd_vec_head\"}"
         + "  ],"
         + " \"error_any\":"
         + "  [{\"src\":\"st_gen1\",\"evt\":\"vec_head\"   },"
         + "   {\"src\":\"st_ana1\",\"evt\":\"vec_tail\"   },"
         + "   {\"src\":\"st_ana2\",\"evt\":\"vec_tail\"   },"
         + "   {\"src\":\"st_gen1\",\"evt\":\"vec_tail\"   },"
         + "   {\"src\":\"dm_1\",   \"evt\":\"rd_vec_tail\"},"
         + "   {\"src\":\"dm_1\",   \"evt\":\"wr_vec_tail\"},"
         + "   {\"src\":\"dm_2\",   \"evt\":\"rd_vec_tail\"},"
         + "   {\"src\":\"xbar\",   \"evt\":\"dm_1.complete\" },"
         + "   {\"src\":\"xbar\",   \"evt\":\"dm_2.complete\" }"
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
      SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Check  01.16";


      // Wait for the completion events from the modules and xbar
      evt_proc_init( std::string("")
         + "{\"valid_all\":"
         + "  [{\"src\":\"st_ana1\",\"evt\":\"vec_tail\"   },"
         + "   {\"src\":\"st_ana2\",\"evt\":\"vec_tail\"   },"
         + "   {\"src\":\"dm_1\",   \"evt\":\"rd_vec_tail\"},"
         + "   {\"src\":\"dm_1\",   \"evt\":\"wr_vec_tail\"},"
         + "   {\"src\":\"dm_2\",   \"evt\":\"rd_vec_tail\"},"
         + "   {\"src\":\"xbar\",   \"evt\":\"dm_1.complete\" },"
         + "   {\"src\":\"xbar\",   \"evt\":\"dm_2.complete\" }"
         + "  ],"
         + " \"error_any\":"
         + "  [{\"src\":\"st_gen1\",\"evt\":\"vec_head\"   },"
         + "   {\"src\":\"st_gen1\",\"evt\":\"vec_tail\"   },"
         + "   {\"src\":\"st_ana1\",\"evt\":\"vec_head\"   },"
         + "   {\"src\":\"st_ana2\",\"evt\":\"vec_head\"   },"
         + "   {\"src\":\"dm_1\",   \"evt\":\"rd_vec_head\"},"
         + "   {\"src\":\"dm_1\",   \"evt\":\"wr_vec_head\"},"
         + "   {\"src\":\"dm_2\",   \"evt\":\"rd_vec_head\"}"
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
      SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Check  01.17";

   } // for(;;)
}

} // namespace simd
