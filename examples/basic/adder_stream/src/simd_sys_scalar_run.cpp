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

   boost_rn::uniform_int_distribution<> rng_dist_uni( 0, 2 );
   boost_rn::variate_generator<boost::mt19937&, boost_rn::uniform_int_distribution<> > rng_ws( rng_eng, rng_dist_uni );

   boost_rn::bernoulli_distribution <>   rng_dist_bern( 0.5 );
   boost_rn::variate_generator<boost::mt19937&, boost_rn::bernoulli_distribution<> >   rng_val( rng_eng, rng_dist_bern );

   sc_core::wait();
   sc_core::wait();

   for( n_test = 0; ; n_test ++ ) {

      SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Start test " << n_test;

      std::size_t rng_ws_min = rng_ws();

      //*************************************
      // Test 01:           SRC3-+
      // Step1: STI1,STI2->ADDER1->ADDER2->STO1,DST2
      //                    SRC3-+
      // Step2: STI2,STI1->ADDER1->ADDER2->STO1,DST2
      //*************************************

      // Configure XBAR for step1
      sc_core::wait();
      wr_conf( std::string("")
         + "{ \"dest\":\"xbar\",\"idx\":0,\"cmd\":\"put\",\"data\":"
         + "    {\"exec_idx\":100,\"conf_next\":1,\"routing\":"
         + "       [{\"mod\":\"st_inp1\",\"port\":0,\"event\":0,\"dst\":"
         + "           [{\"mod\":\"arith1\",\"port\":0,\"master\":1"
         + "            },"
         + "            {\"mod\":\"st_chk1\",\"port\":0,\"master\":0"
         + "            }"
         + "           ]"
         + "        },"
         + "        {\"mod\":\"st_inp2\",\"port\":0,\"event\":0,\"dst\":"
         + "           [{\"mod\":\"arith1\",\"port\":1,\"master\":1"
         + "            },"
         + "            {\"mod\":\"st_chk2\",\"port\":0,\"master\":0"
         + "            }"
         + "           ]"
         + "        },"
         + "        {\"mod\":\"arith1\",\"port\":0,\"event\":0,\"dst\":"
         + "           [{\"mod\":\"arith2\",\"port\":0,\"master\":1"
         + "            }"
         + "           ]"
         + "        },"
         + "        {\"mod\":\"st_gen3\",\"port\":0,\"event\":0,\"dst\":"
         + "           [{\"mod\":\"arith2\",\"port\":1,\"master\":1"
         + "            }"
         + "           ]"
         + "        },"
         + "        {\"mod\":\"arith2\",\"port\":0,\"event\":0,\"dst\":"
         + "           [{\"mod\":\"st_out1\",\"port\":0,\"master\":1"
         + "            },"
         + "            {\"mod\":\"st_ana2\",\"port\":0,\"master\":0"
         + "            }"
         + "           ]"
         + "        }"
         + "       ]"
         + "    }"
         + "}" );
      SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 01.01";

      // Configure Input Stream 1 for step1
      sc_core::wait();
      wr_conf( std::string("")
         + "{ \"dest\":\"st_inp1\",\"idx\":0,\"cmd\":\"put\",\"data\":"
         + "    {\"exec_idx\":100,\"stat_idx\":0,\"conf_next\":1,\"events\":[]," // no events for this stage
         + "     \"pool_seg\":\"seg_rd0\",\"blk_start\":0,\"blk_size\":\"800\","
         + "     \"ws_min\":" + std::to_string( rng_ws_min ) + ","
         + "     \"ws_max\":" + std::to_string( rng_ws_min + rng_ws())
         + "    }"
         + "}" );
      SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 01.02";

      // Configure Input Stream 2 for step1
      sc_core::wait();
      wr_conf( std::string("")
      + "{ \"dest\":\"st_inp2\",\"idx\":0,\"cmd\":\"put\",\"data\":"
      + "    {\"exec_idx\":100,\"stat_idx\":0,\"conf_next\":1,\"events\":[]," // no events for this stage
      + "     \"pool_seg\":\"seg_rd1\",\"blk_start\":512,\"blk_size\":\"800\","
      + "     \"ws_min\":" + std::to_string( rng_ws_min ) + ","
      + "     \"ws_max\":" + std::to_string( rng_ws_min + rng_ws())
      + "    }"
      + "}" );
      SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 01.03";

      // Configure ADD/SUB EU 1 for step1
      sc_core::wait();
      wr_conf( std::string("")
         + "{ \"dest\":\"arith1\",\"idx\":0,\"cmd\":\"put\",\"data\":"
         + "    {\"exec_idx\":100,\"stat_idx\":0,\"conf_next\":1,\"events\":[]," // no events for this stage
         + "     \"neg_re0\":\"0\",\"neg_im0\":\"0\",\"neg_re1\":\"0\",\"neg_im1\":\"0\",\"smp_nena_op\":\"error\""
         + "    }"
         + "}" );
      SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 01.04";

      // Configure VRI Source 3 for step1
      sc_core::wait();
      wr_conf( std::string("")
         + "{ \"dest\":\"st_gen3\",\"idx\":0,\"cmd\":\"put\",\"data\":"
         + "    {\"exec_idx\":100,\"stat_idx\":0,\"conf_next\":1,\"events\":[]," // no events for this stage
         + "     \"vec_size\":\"200\",\"slot_ena_ovr\":1,\"valid_ovr\":" + std::to_string( rng_val())
         + "    }"
         + "}" );
      SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 01.05";

      // Configure ADD/SUB EU 2 for step1
      sc_core::wait();
      wr_conf( std::string("")
         + "{ \"dest\":\"arith2\",\"idx\":0,\"cmd\":\"put\",\"data\":"
         + "    {\"exec_idx\":100,\"stat_idx\":0,\"conf_next\":1,\"events\":[]," // no events for this stage
         + "     \"neg_re0\":\"0\",\"neg_im0\":\"0\",\"neg_re1\":\"1\",\"neg_im1\":\"1\",\"smp_nena_op\":\"error\"" // Subtract on port1
         + "    }"
         + "}" );
      SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 01.06";

      // Configure Output Stream 1 for step1
      sc_core::wait();
      wr_conf( std::string("")
      + "{ \"dest\":\"st_out1\",\"idx\":0,\"cmd\":\"put\",\"data\":"
      + "    {\"exec_idx\":100,\"stat_idx\":0,\"conf_next\":1,\"events\":[]," // no events for this stage
      + "     \"pool_seg\":\"seg_wr0\",\"blk_start\":\"0\","
      + "     \"ws_min\":" + std::to_string( rng_ws_min ) + ","
      + "     \"ws_max\":" + std::to_string( rng_ws_min + rng_ws())
      + "    }"
      + "}" );
      SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 01.07";

      // Configure VRI Check 1
      sc_core::wait();
      wr_conf( std::string("")
         + "{ \"dest\":\"st_chk1\",\"idx\":0,\"cmd\":\"put\",\"data\":"
         + "    {\"exec_idx\":100,\"stat_idx\":0,\"conf_next\":1,\"events\":[]," // no events for this stage
         + "     \"ready_ovr\":1" // To operate as Slave
         + "    }"
         + "}" );
      SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 01.08";

      // Configure VRI Check 2
      sc_core::wait();
      wr_conf( std::string("")
         + "{ \"dest\":\"st_chk2\",\"idx\":0,\"cmd\":\"put\",\"data\":"
         + "    {\"exec_idx\":100,\"stat_idx\":0,\"conf_next\":1,\"events\":[]," // no events for this stage
         + "     \"ready_ovr\":1" // To operate as Slave
         + "    }"
         + "}" );
      SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 01.08";

      // Configure VRI Destination 2
      sc_core::wait();
      wr_conf( std::string("")
         + "{ \"dest\":\"st_ana2\",\"idx\":0,\"cmd\":\"put\",\"data\":"
         + "    {\"exec_idx\":100,\"stat_idx\":0,\"conf_next\":1,\"events\":[]," // no events for this stage
         + "     \"ready_ovr\":1" // To operate as Slave
         + "    }"
         + "}" );
      SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 01.08";

      // Configure XBAR for step2
      sc_core::wait();
      wr_conf( std::string("")
         + "{ \"dest\":\"xbar\",\"idx\":1,\"cmd\":\"put\",\"data\":"
         + "    {\"exec_idx\":999,\"conf_next\":8,\"routing\":"
         + "       [{\"mod\":\"st_inp2\",\"port\":0,\"event\":1,\"dst\":"
         + "           [{\"mod\":\"arith1\",\"port\":0,\"master\":1"
         + "            },"
         + "            {\"mod\":\"st_chk2\",\"port\":0,\"master\":0"
         + "            }"
         + "           ]"
         + "        },"
         + "        {\"mod\":\"st_inp1\",\"port\":0,\"event\":1,\"dst\":"
         + "           [{\"mod\":\"arith1\",\"port\":1,\"master\":1"
         + "            },"
         + "            {\"mod\":\"st_chk1\",\"port\":0,\"master\":0"
         + "            }"
         + "           ]"
         + "        },"
         + "        {\"mod\":\"arith1\",\"port\":0,\"event\":1,\"dst\":"
         + "           [{\"mod\":\"arith2\",\"port\":1,\"master\":1"
         + "            }"
         + "           ]"
         + "        },"
         + "        {\"mod\":\"st_gen3\",\"port\":0,\"event\":1,\"dst\":"
         + "           [{\"mod\":\"arith2\",\"port\":0,\"master\":1"
         + "            }"
         + "           ]"
         + "        },"
         + "        {\"mod\":\"arith2\",\"port\":0,\"event\":1,\"dst\":"
         + "           [{\"mod\":\"st_out1\",\"port\":0,\"master\":1"
         + "            },"
         + "            {\"mod\":\"st_ana2\",\"port\":0,\"master\":0"
         + "            }"
         + "           ]"
         + "        }"
         + "       ]"
         + "    }"
         + "}" );
      SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 01.09";

      // Configure VRI Source 1 for step2
      sc_core::wait();
      wr_conf( std::string("")
         + "{ \"dest\":\"st_inp1\",\"idx\":1,\"cmd\":\"put\",\"data\":"
         + "    {\"exec_idx\":999,\"stat_idx\":0,\"conf_next\":8,\"events\":"
         + "       [\"vec_head\",\"vec_tail\""
         + "       ],"
         + "     \"pool_seg\":\"seg_rd1\",\"blk_start\":512,\"blk_size\":\"800\","
         + "     \"ws_min\":" + std::to_string( rng_ws_min ) + ","
         + "     \"ws_max\":" + std::to_string( rng_ws_min + rng_ws())
         + "    }"
         + "}" );
      SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 01.10";

      // Configure VRI Source 1 for step2
      sc_core::wait();
      wr_conf( std::string("")
         + "{ \"dest\":\"st_inp2\",\"idx\":1,\"cmd\":\"put\",\"data\":"
         + "    {\"exec_idx\":999,\"stat_idx\":0,\"conf_next\":8,\"events\":"
         + "       [\"vec_head\",\"vec_tail\""
         + "       ],"
         + "     \"pool_seg\":\"seg_wr0\",\"blk_start\":0,\"blk_size\":\"800\","
         + "     \"ws_min\":" + std::to_string( rng_ws_min ) + ","
         + "     \"ws_max\":" + std::to_string( rng_ws_min + rng_ws())
         + "    }"
         + "}" );
      SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 01.11";

      // Configure ADD/SUB EU 1 for step2
      sc_core::wait();
      wr_conf( std::string("")
         + "{ \"dest\":\"arith1\",\"idx\":1,\"cmd\":\"put\",\"data\":"
         + "    {\"exec_idx\":999,\"stat_idx\":0,\"conf_next\":8,\"events\":"
         + "       [\"vec_head\",\"vec_tail\""
         + "       ],"
         + "     \"neg_re0\":\"0\",\"neg_im0\":\"0\",\"neg_re1\":\"0\",\"neg_im1\":\"0\",\"smp_nena_op\":\"zero\""
         + "    }"
         + "}" );
      SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 01.12";

      // Configure VRI Source 1 for step2
      sc_core::wait();
      wr_conf( std::string("")
         + "{ \"dest\":\"st_gen3\",\"idx\":1,\"cmd\":\"put\",\"data\":"
         + "    {\"exec_idx\":999,\"stat_idx\":0,\"conf_next\":8,\"events\":"
         + "       [\"vec_head\",\"vec_tail\""
         + "       ],"
         + "     \"vec_size\":\"200\",\"slot_ena_ovr\":1,\"valid_ovr\":" + std::to_string( rng_val())
         + "    }"
         + "}" );
      SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 01.13";

      // Configure ADD/SUB EU 2 for step2
      sc_core::wait();
      wr_conf( std::string("")
         + "{ \"dest\":\"arith2\",\"idx\":1,\"cmd\":\"put\",\"data\":"
         + "    {\"exec_idx\":999,\"stat_idx\":0,\"conf_next\":8,\"events\":"
         + "       [\"vec_head\",\"vec_tail\""
         + "       ],"
         + "     \"neg_re0\":\"1\",\"neg_im0\":\"1\",\"neg_re1\":\"0\",\"neg_im1\":\"0\",\"smp_nena_op\":\"zero\"" // Subtract on port2
         + "    }"
         + "}" );
      SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 01.14";

      // Configure Output Stream 1 for step1
      sc_core::wait();
      wr_conf( std::string("")
      + "{ \"dest\":\"st_out1\",\"idx\":1,\"cmd\":\"put\",\"data\":"
      + "    {\"exec_idx\":999,\"stat_idx\":0,\"conf_next\":8,\"events\":"
      + "       [\"vec_head\",\"vec_tail\""
      + "       ],"
      + "     \"pool_seg\":\"seg_wr0\",\"blk_start\":\"2048\","
      + "     \"ws_min\":" + std::to_string( rng_ws_min ) + ","
      + "     \"ws_max\":" + std::to_string( rng_ws_min + rng_ws())
      + "    }"
      + "}" );
      SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 01.15";

      // Configure VRI Destination
      sc_core::wait();
      wr_conf( std::string("")
         + "{ \"dest\":\"st_chk1\",\"idx\":1,\"cmd\":\"put\",\"data\":"
         + "    {\"exec_idx\":999,\"stat_idx\":0,\"conf_next\":8,\"events\":"
         + "       [\"vec_head\",\"vec_tail\""
         + "       ],"
         + "     \"ready_ovr\":1" // To operate as Slave
         + "    }"
         + "}" );
      SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 01.16";

      // Configure VRI Destination
      sc_core::wait();
      wr_conf( std::string("")
         + "{ \"dest\":\"st_chk2\",\"idx\":1,\"cmd\":\"put\",\"data\":"
         + "    {\"exec_idx\":999,\"stat_idx\":0,\"conf_next\":8,\"events\":"
         + "       [\"vec_head\",\"vec_tail\""
         + "       ],"
         + "     \"ready_ovr\":1" // To operate as Slave
         + "    }"
         + "}" );
      SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 01.16";

      // Configure VRI Destination
      sc_core::wait();
      wr_conf( std::string("")
         + "{ \"dest\":\"st_ana2\",\"idx\":1,\"cmd\":\"put\",\"data\":"
         + "    {\"exec_idx\":999,\"stat_idx\":0,\"conf_next\":8,\"events\":"
         + "       [\"vec_head\",\"vec_tail\""
         + "       ],"
         + "     \"ready_ovr\":1" // To operate as Slave
         + "    }"
         + "}" );
      SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 01.16";

      // Run
      sc_core::wait();
      wr_conf( std::string("")
         + "{ \"dest\":\"broadcast\",\"idx\":100,\"cmd\":\"run\""
         + "}" );
      SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Start  01.17";

      // Wait for the completion events from the modules and xbar
      evt_proc_init( std::string("")
         + "{\"valid_all\":"
         + "  [{\"mod\":\"st_inp1\", \"evt\":\"vec_head\"},"
         + "   {\"mod\":\"st_inp2\", \"evt\":\"vec_head\"},"
         + "   {\"mod\":\"st_gen3\", \"evt\":\"vec_head\"},"
         + "   {\"mod\":\"st_out1\", \"evt\":\"vec_head\"},"
         + "   {\"mod\":\"st_ana2\", \"evt\":\"vec_head\"},"
         + "   {\"mod\":\"arith1\",  \"evt\":\"vec_head\"},"
         + "   {\"mod\":\"arith2\",  \"evt\":\"vec_head\"}"
         + "  ],"
         + " \"error_any\":"
         + "  [{\"mod\":\"st_inp1\", \"evt\":\"vec_tail\"},"
         + "   {\"mod\":\"st_inp2\", \"evt\":\"vec_tail\"},"
         + "   {\"mod\":\"st_gen3\", \"evt\":\"vec_tail\"},"
         + "   {\"mod\":\"st_out1\", \"evt\":\"vec_tail\"},"
         + "   {\"mod\":\"st_ana2\", \"evt\":\"vec_tail\"},"
         + "   {\"mod\":\"arith1\",  \"evt\":\"vec_tail\"},"
         + "   {\"mod\":\"arith2\",  \"evt\":\"vec_tail\"},"
         + "   {\"mod\":\"xbar\",    \"evt\":\"st_inp1.complete\" },"
         + "   {\"mod\":\"xbar\",    \"evt\":\"st_inp2.complete\" },"
         + "   {\"mod\":\"xbar\",    \"evt\":\"st_gen3.complete\" },"
         + "   {\"mod\":\"xbar\",    \"evt\":\"arith1.complete\"  },"
         + "   {\"mod\":\"xbar\",    \"evt\":\"arith2.complete\"  }"
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
      SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Check  01.18";

      // Wait for the completion events from the modules and xbar
      evt_proc_init( std::string("")
         + "{\"error_any\":"
         + "  [{\"mod\":\"st_inp1\", \"evt\":\"vec_head\"},"
         + "   {\"mod\":\"st_inp2\", \"evt\":\"vec_head\"},"
         + "   {\"mod\":\"st_gen3\", \"evt\":\"vec_head\"},"
         + "   {\"mod\":\"st_out1\", \"evt\":\"vec_head\"},"
         + "   {\"mod\":\"st_ana2\", \"evt\":\"vec_head\"},"
         + "   {\"mod\":\"arith1\",  \"evt\":\"vec_head\"},"
         + "   {\"mod\":\"arith2\",  \"evt\":\"vec_head\"}"
         + "  ],"
         + " \"valid_all\":"
         + "  [{\"mod\":\"st_inp1\", \"evt\":\"vec_tail\"},"
         + "   {\"mod\":\"st_inp2\", \"evt\":\"vec_tail\"},"
         + "   {\"mod\":\"st_gen3\", \"evt\":\"vec_tail\"},"
         + "   {\"mod\":\"st_out1\", \"evt\":\"vec_tail\"},"
         + "   {\"mod\":\"st_ana2\", \"evt\":\"vec_tail\"},"
         + "   {\"mod\":\"arith1\",  \"evt\":\"vec_tail\"},"
         + "   {\"mod\":\"arith2\",  \"evt\":\"vec_tail\"},"
         + "   {\"mod\":\"xbar\",    \"evt\":\"st_inp1.complete\" },"
         + "   {\"mod\":\"xbar\",    \"evt\":\"st_inp2.complete\" },"
         + "   {\"mod\":\"xbar\",    \"evt\":\"st_gen3.complete\" },"
         + "   {\"mod\":\"xbar\",    \"evt\":\"arith1.complete\"  },"
         + "   {\"mod\":\"xbar\",    \"evt\":\"arith2.complete\"  }"
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
      SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Check  01.19";

   } // for(;;)
}

} // namespace simd
