/*
 * simd_sys_scalar_run.cpp
 *
 *  Description:
 *    System components: Scalar processor
 */

#include <boost/random.hpp>
#include "simd_sig_dmeu_data.h"
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

      //*************************************
      // Test 01: Execute 4 steps under the distributed control of the vector core.
      // Step1: DM1->DST1 - Initialized region 1
      // Step2: DM1->DST1 - Initialized region 2
      // Step3: DM1->DST1 - Initialized region 3
      // Step4: DM1->DST1 - Initialized region 4
      //*************************************

      // Configure XBAR for step1
      sc_core::wait();
      wr_conf( std::string("")
         + "{ \"dst\":[\"xbar\"],\"idx\":0,\"cmd\":\"put\",\"data\":"
         + "    {\"exec_idx\":100,\"conf_next\":1,\"routing\":"
         + "       [{\"mod\":\"dm_1\",\"port\":0,\"event\":0,\"dst\":"
         + "           [{\"mod\":\"st_ana1\",\"port\":0,\"master\":1"
         + "            }"
         + "           ]"
         + "        }"
         + "       ]"
         + "    }"
         + "}" );
      SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 01.01";

      // DM 1 for step1
      sc_core::wait();
      wr_conf( std::string("")
         + "{ \"dst\":[\"dm_1\"],\"idx\":0,\"cmd\":\"put\",\"data\":"
         + "    {\"exec_idx\":100,\"stat_idx\":0,\"conf_next\":1,\"events\":[],"
         + "     \"op_mode\":\"read\",\"smp_num\":" + std::to_string( 100 / simd_dmeu_data_c::dim ) +",\"ag_conf\":"
         + "       {\"mode\":\"linear\", \"param\":"
         + "          [{\"offset\":" + std::to_string( 0 / simd_dmeu_data_c::dim ) + ", \"ena\":1, \"perm\":0 },"
         + "           {\"offset\":" + std::to_string( 0 / simd_dmeu_data_c::dim ) + ", \"ena\":1, \"perm\":1 },"
         + "           {\"offset\":" + std::to_string( 0 / simd_dmeu_data_c::dim ) + ", \"ena\":1, \"perm\":2 },"
         + "           {\"offset\":" + std::to_string( 0 / simd_dmeu_data_c::dim ) + ", \"ena\":1, \"perm\":3 } "
         + "          ]"
         + "       }"
         + "    }"
         + "}" );
      SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 01.02";

      // Configure VRI Destination for step1
      sc_core::wait();
      wr_conf( std::string("")
         + "{ \"dst\":[\"st_ana1\"],\"idx\":0,\"cmd\":\"put\",\"data\":"
         + "    {\"exec_idx\":100,\"stat_idx\":0,\"conf_next\":1,\"events\":[]," // no events for this stage
         + "     \"ready_ovr\":" + std::to_string( rng_val())
         + "    }"
         + "}" );
      SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 01.03";

      // Configure XBAR for step2
      sc_core::wait();
      wr_conf( std::string("")
         + "{ \"dst\":[\"xbar\"],\"idx\":1,\"cmd\":\"put\",\"data\":"
         + "    {\"exec_idx\":999,\"conf_next\":2,\"routing\":"
         + "       [{\"mod\":\"dm_1\",\"port\":0,\"event\":0,\"dst\":"
         + "           [{\"mod\":\"st_ana1\",\"port\":0,\"master\":1"
         + "            }"
         + "           ]"
         + "        }"
         + "       ]"
         + "    }"
         + "}" );
      SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 01.04";

      // DM 1 for step2
      sc_core::wait();
      wr_conf( std::string("")
         + "{ \"dst\":[\"dm_1\"],\"idx\":1,\"cmd\":\"put\",\"data\":"
         + "    {\"exec_idx\":999,\"stat_idx\":0,\"conf_next\":2,\"events\":[],"
         + "     \"op_mode\":\"read\",\"smp_num\":" + std::to_string( 200 / simd_dmeu_data_c::dim ) +",\"ag_conf\":"
         + "       {\"mode\":\"linear\", \"param\":"
         + "          [{\"offset\":" + std::to_string( 1100 / simd_dmeu_data_c::dim ) + ", \"ena\":1, \"perm\":0 },"
         + "           {\"offset\":" + std::to_string( 1100 / simd_dmeu_data_c::dim ) + ", \"ena\":1, \"perm\":1 },"
         + "           {\"offset\":" + std::to_string( 1100 / simd_dmeu_data_c::dim ) + ", \"ena\":1, \"perm\":2 },"
         + "           {\"offset\":" + std::to_string( 1100 / simd_dmeu_data_c::dim ) + ", \"ena\":1, \"perm\":3 } "
         + "          ]"
         + "       }"
         + "    }"
         + "}" );
      SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 01.05";

      // Configure VRI Destination for step2
      sc_core::wait();
      wr_conf( std::string("")
         + "{ \"dst\":[\"st_ana1\"],\"idx\":1,\"cmd\":\"put\",\"data\":"
         + "    {\"exec_idx\":999,\"stat_idx\":0,\"conf_next\":2,\"events\":[]," // no events for this stage
         + "     \"ready_ovr\":" + std::to_string( rng_val())
         + "    }"
         + "}" );
      SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 01.06";

      // Configure XBAR for step3
      sc_core::wait();
      wr_conf( std::string("")
         + "{ \"dst\":[\"xbar\"],\"idx\":2,\"cmd\":\"put\",\"data\":"
         + "    {\"exec_idx\":999,\"conf_next\":3,\"routing\":"
         + "       [{\"mod\":\"dm_1\",\"port\":0,\"event\":0,\"dst\":"
         + "           [{\"mod\":\"st_ana1\",\"port\":0,\"master\":1"
         + "            }"
         + "           ]"
         + "        }"
         + "       ]"
         + "    }"
         + "}" );
      SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 01.07";

      // DM 1 for step3
      sc_core::wait();
      wr_conf( std::string("")
         + "{ \"dst\":[\"dm_1\"],\"idx\":2,\"cmd\":\"put\",\"data\":"
         + "    {\"exec_idx\":999,\"stat_idx\":0,\"conf_next\":3,\"events\":[],"
         + "     \"op_mode\":\"read\",\"smp_num\":" + std::to_string( 300 / simd_dmeu_data_c::dim ) +",\"ag_conf\":"
         + "       {\"mode\":\"linear\", \"param\":"
         + "          [{\"offset\":" + std::to_string( 2100 / simd_dmeu_data_c::dim ) + ", \"ena\":1, \"perm\":0 },"
         + "           {\"offset\":" + std::to_string( 2100 / simd_dmeu_data_c::dim ) + ", \"ena\":1, \"perm\":1 },"
         + "           {\"offset\":" + std::to_string( 2100 / simd_dmeu_data_c::dim ) + ", \"ena\":1, \"perm\":2 },"
         + "           {\"offset\":" + std::to_string( 2100 / simd_dmeu_data_c::dim ) + ", \"ena\":1, \"perm\":3 } "
         + "          ]"
         + "       }"
         + "    }"
         + "}" );
      SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 01.08";

      // Configure VRI Destination for step3
      sc_core::wait();
      wr_conf( std::string("")
         + "{ \"dst\":[\"st_ana1\"],\"idx\":2,\"cmd\":\"put\",\"data\":"
         + "    {\"exec_idx\":999,\"stat_idx\":0,\"conf_next\":3,\"events\":[]," // no events for this stage
         + "     \"ready_ovr\":" + std::to_string( rng_val())
         + "    }"
         + "}" );
      SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 01.09";

      // Configure XBAR for step4
      sc_core::wait();
      wr_conf( std::string("")
         + "{ \"dst\":[\"xbar\"],\"idx\":3,\"cmd\":\"put\",\"data\":"
         + "    {\"exec_idx\":999,\"conf_next\":8,\"routing\":"
         + "       [{\"mod\":\"dm_1\",\"port\":0,\"event\":1,\"dst\":"
         + "           [{\"mod\":\"st_ana1\",\"port\":0,\"master\":1"
         + "            }"
         + "           ]"
         + "        }"
         + "       ]"
         + "    }"
         + "}" );
      SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 01.10";

      // DM 1 for step4
      sc_core::wait();
      wr_conf( std::string("")
         + "{ \"dst\":[\"dm_1\"],\"idx\":3,\"cmd\":\"put\",\"data\":"
         + "    {\"exec_idx\":999,\"stat_idx\":0,\"conf_next\":8,\"events\":"
         + "       [\"rd_vec_head\",\"rd_vec_tail\""
         + "       ],"
         + "     \"op_mode\":\"read\",\"smp_num\":" + std::to_string( 400 / simd_dmeu_data_c::dim ) +",\"ag_conf\":"
         + "       {\"mode\":\"linear\", \"param\":"
         + "          [{\"offset\":" + std::to_string( 3100 / simd_dmeu_data_c::dim ) + ", \"ena\":1, \"perm\":0 },"
         + "           {\"offset\":" + std::to_string( 3100 / simd_dmeu_data_c::dim ) + ", \"ena\":1, \"perm\":1 },"
         + "           {\"offset\":" + std::to_string( 3100 / simd_dmeu_data_c::dim ) + ", \"ena\":1, \"perm\":2 },"
         + "           {\"offset\":" + std::to_string( 3100 / simd_dmeu_data_c::dim ) + ", \"ena\":1, \"perm\":3 } "
         + "          ]"
         + "       }"
         + "    }"
         + "}" );
      SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 01.11";

      // Configure VRI Destination for step4
      sc_core::wait();
      wr_conf( std::string("")
         + "{ \"dst\":[\"st_ana1\"],\"idx\":3,\"cmd\":\"put\",\"data\":"
         + "    {\"exec_idx\":999,\"stat_idx\":0,\"conf_next\":8,\"events\":"
         + "       [\"vec_head\",\"vec_tail\""
         + "       ],"
         + "     \"ready_ovr\":" + std::to_string( rng_val())
         + "    }"
         + "}" );
      SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 01.12";

      // Run
      sc_core::wait();
      wr_conf( std::string("")
         + "{ \"dst\":\"broadcast\",\"idx\":100,\"cmd\":\"run\""
         + "}" );
      SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 01.13";

      // Wait for the completion events from the modules and xbar
      evt_proc_init( std::string("")
         + "{\"valid_all\":"
         + "  [{\"src\":\"st_ana1\",\"evt\":\"vec_head\"   },"
         + "   {\"src\":\"dm_1\",   \"evt\":\"rd_vec_head\"}"
         + "  ],"
         + " \"error_any\":"
         + "  [{\"src\":\"st_ana1\",\"evt\":\"vec_tail\"   },"
         + "   {\"src\":\"dm_1\",   \"evt\":\"rd_vec_tail\"},"
         + "   {\"src\":\"dm_1\",   \"evt\":\"wr_vec_head\"},"
         + "   {\"src\":\"dm_1\",   \"evt\":\"wr_vec_tail\"},"
         + "   {\"src\":\"xbar\",   \"evt\":\"dm_1.complete\" }"
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
      SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Start  01.14";

      // Wait for the completion events from the modules and xbar
      evt_proc_init( std::string("")
         + "{\"valid_all\":"
         + "  [{\"src\":\"st_ana1\",\"evt\":\"vec_tail\"   },"
         + "   {\"src\":\"dm_1\",   \"evt\":\"rd_vec_tail\"},"
         + "   {\"src\":\"xbar\",   \"evt\":\"dm_1.complete\" }"
         + "  ],"
         + " \"error_any\":"
         + "  [{\"src\":\"st_ana1\",\"evt\":\"vec_head\"   },"
         + "   {\"src\":\"dm_1\",   \"evt\":\"rd_vec_head\"},"
         + "   {\"src\":\"dm_1\",   \"evt\":\"wr_vec_head\"},"
         + "   {\"src\":\"dm_1\",   \"evt\":\"wr_vec_tail\"}"
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
      SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Check  01.15";

   } // for(;;)
}

} // namespace simd
