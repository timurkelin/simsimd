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
      // Test 01:           SRC3-+
      // Step1: SRC1,SRC2->ADDER1->ADDER2->DST1
      //                    SRC3-+
      // Step2: SRC2,SRC1->ADDER1->ADDER2->DST1
      //*************************************

      // Configure XBAR for step1
      sc_core::wait();
      wr_conf( std::string("")
         + "{ \"dst\":[\"xbar\"],\"idx\":0,\"cmd\":\"put\",\"data\":"
         + "    {\"exec_idx\":100,\"conf_next\":1,\"routing\":"
         + "       [{\"mod\":\"st_gen1\",\"port\":0,\"event\":0,\"dst\":"
         + "           [{\"mod\":\"arith1\",\"port\":0,\"master\":1"
         + "            }"
         + "           ]"
         + "        },"
         + "        {\"mod\":\"st_gen2\",\"port\":0,\"event\":0,\"dst\":"
         + "           [{\"mod\":\"arith1\",\"port\":1,\"master\":1"
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
         + "           [{\"mod\":\"st_ana1\",\"port\":0,\"master\":1"
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
         + "    {\"exec_idx\":100,\"stat_idx\":0,\"conf_next\":1,\"events\":[]," // no events for this stage
         + "     \"vec_size\":" + std::to_string( vec_size1 ) + ",\"slot_ena_ovr\":1,\"valid_ovr\":" + std::to_string( rng_val())
         + "    }"
         + "}" );
      SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 01.02";

      // Configure VRI Source 2 for step1
      sc_core::wait();
      wr_conf( std::string("")
         + "{ \"dst\":[\"st_gen2\"],\"idx\":0,\"cmd\":\"put\",\"data\":"
         + "    {\"exec_idx\":100,\"stat_idx\":0,\"conf_next\":1,\"events\":[]," // no events for this stage
         + "     \"vec_size\":" + std::to_string( vec_size1 ) + ",\"slot_ena_ovr\":1,\"valid_ovr\":" + std::to_string( rng_val())
         + "    }"
         + "}" );
      SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 01.03";

      // Configure ADD/SUB EU 1 for step1
      sc_core::wait();
      wr_conf( std::string("")
         + "{ \"dst\":[\"arith1\"],\"idx\":0,\"cmd\":\"put\",\"data\":"
         + "    {\"exec_idx\":100,\"stat_idx\":0,\"conf_next\":1,\"events\":[]," // no events for this stage
         + "     \"neg_re0\":\"0\",\"neg_im0\":\"0\",\"neg_re1\":\"0\",\"neg_im1\":\"0\",\"smp_nena_op\":\"error\""
         + "    }"
         + "}" );
      SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 01.04";

      // Configure VRI Source 3 for step1
      sc_core::wait();
      wr_conf( std::string("")
         + "{ \"dst\":[\"st_gen3\"],\"idx\":0,\"cmd\":\"put\",\"data\":"
         + "    {\"exec_idx\":100,\"stat_idx\":0,\"conf_next\":1,\"events\":[]," // no events for this stage
         + "     \"vec_size\":" + std::to_string( vec_size1 ) + ",\"slot_ena_ovr\":1,\"valid_ovr\":" + std::to_string( rng_val())
         + "    }"
         + "}" );
      SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 01.05";

      // Configure ADD/SUB EU 2 for step1
      sc_core::wait();
      wr_conf( std::string("")
         + "{ \"dst\":[\"arith2\"],\"idx\":0,\"cmd\":\"put\",\"data\":"
         + "    {\"exec_idx\":100,\"stat_idx\":0,\"conf_next\":1,\"events\":[]," // no events for this stage
         + "     \"neg_re0\":\"0\",\"neg_im0\":\"0\",\"neg_re1\":\"1\",\"neg_im1\":\"1\",\"smp_nena_op\":\"error\"" // Subtract on port1
         + "    }"
         + "}" );
      SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 01.06";

      // Configure VRI Destination
      sc_core::wait();
      wr_conf( std::string("")
         + "{ \"dst\":[\"st_ana1\"],\"idx\":0,\"cmd\":\"put\",\"data\":"
         + "    {\"exec_idx\":100,\"stat_idx\":0,\"conf_next\":1,\"events\":[]," // no events for this stage
         + "     \"ready_ovr\":" + std::to_string( rng_val())
         + "    }"
         + "}" );
      SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 01.07";

      // Configure XBAR for step2
      sc_core::wait();
      wr_conf( std::string("")
         + "{ \"dst\":[\"xbar\"],\"idx\":1,\"cmd\":\"put\",\"data\":"
         + "    {\"exec_idx\":999,\"conf_next\":8,\"routing\":"
         + "       [{\"mod\":\"st_gen2\",\"port\":0,\"event\":1,\"dst\":"
         + "           [{\"mod\":\"arith1\",\"port\":0,\"master\":1"
         + "            }"
         + "           ]"
         + "        },"
         + "        {\"mod\":\"st_gen1\",\"port\":0,\"event\":1,\"dst\":"
         + "           [{\"mod\":\"arith1\",\"port\":1,\"master\":1"
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
         + "           [{\"mod\":\"st_ana1\",\"port\":0,\"master\":1"
         + "            }"
         + "           ]"
         + "        }"
         + "       ]"
         + "    }"
         + "}" );
      SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 01.08";

      // Configure VRI Source 1 for step2
      sc_core::wait();
      wr_conf( std::string("")
         + "{ \"dst\":[\"st_gen1\"],\"idx\":1,\"cmd\":\"put\",\"data\":"
         + "    {\"exec_idx\":999,\"stat_idx\":0,\"conf_next\":8,\"events\":"
         + "       [\"vec_head\",\"vec_tail\""
         + "       ],"
         + "     \"vec_size\":" + std::to_string( vec_size2 ) + ",\"slot_ena_ovr\":1,\"valid_ovr\":" + std::to_string( rng_val())
         + "    }"
         + "}" );
      SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 01.09";

      // Configure VRI Source 1 for step2
      sc_core::wait();
      wr_conf( std::string("")
         + "{ \"dst\":[\"st_gen2\"],\"idx\":1,\"cmd\":\"put\",\"data\":"
         + "    {\"exec_idx\":999,\"stat_idx\":0,\"conf_next\":8,\"events\":"
         + "       [\"vec_head\",\"vec_tail\""
         + "       ],"
         + "     \"vec_size\":" + std::to_string( vec_size2 ) + ",\"slot_ena_ovr\":1,\"valid_ovr\":" + std::to_string( rng_val())
         + "    }"
         + "}" );
      SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 01.10";

      // Configure ADD/SUB EU 1 for step2
      sc_core::wait();
      wr_conf( std::string("")
         + "{ \"dst\":[\"arith1\"],\"idx\":1,\"cmd\":\"put\",\"data\":"
         + "    {\"exec_idx\":999,\"stat_idx\":0,\"conf_next\":8,\"events\":"
         + "       [\"vec_head\",\"vec_tail\""
         + "       ],"
         + "     \"neg_re0\":\"0\",\"neg_im0\":\"0\",\"neg_re1\":\"0\",\"neg_im1\":\"0\",\"smp_nena_op\":\"zero\""
         + "    }"
         + "}" );
      SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 01.11";

      // Configure VRI Source 1 for step2
      sc_core::wait();
      wr_conf( std::string("")
         + "{ \"dst\":[\"st_gen3\"],\"idx\":1,\"cmd\":\"put\",\"data\":"
         + "    {\"exec_idx\":999,\"stat_idx\":0,\"conf_next\":8,\"events\":"
         + "       [\"vec_head\",\"vec_tail\""
         + "       ],"
         + "     \"vec_size\":" + std::to_string( vec_size2 ) + ",\"slot_ena_ovr\":1,\"valid_ovr\":" + std::to_string( rng_val())
         + "    }"
         + "}" );
      SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 01.12";

      // Configure ADD/SUB EU 2 for step2
      sc_core::wait();
      wr_conf( std::string("")
         + "{ \"dst\":[\"arith2\"],\"idx\":1,\"cmd\":\"put\",\"data\":"
         + "    {\"exec_idx\":999,\"stat_idx\":0,\"conf_next\":8,\"events\":"
         + "       [\"vec_head\",\"vec_tail\""
         + "       ],"
         + "     \"neg_re0\":\"1\",\"neg_im0\":\"1\",\"neg_re1\":\"0\",\"neg_im1\":\"0\",\"smp_nena_op\":\"zero\"" // Subtract on port2
         + "    }"
         + "}" );
      SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 01.13";

      // Configure VRI Destination
      sc_core::wait();
      wr_conf( std::string("")
         + "{ \"dst\":[\"st_ana1\"],\"idx\":1,\"cmd\":\"put\",\"data\":"
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
         + "  [{\"src\":\"st_gen1\", \"evt\":\"vec_head\"},"
         + "   {\"src\":\"st_gen2\", \"evt\":\"vec_head\"},"
         + "   {\"src\":\"st_gen3\", \"evt\":\"vec_head\"},"
         + "   {\"src\":\"st_ana1\", \"evt\":\"vec_head\"},"
         + "   {\"src\":\"arith1\",  \"evt\":\"vec_head\"},"
         + "   {\"src\":\"arith2\",  \"evt\":\"vec_head\"}"
         + "  ],"
         + " \"error_any\":"
         + "  [{\"src\":\"st_gen1\",\"evt\":\"vec_tail\"},"
         + "   {\"src\":\"st_gen2\",\"evt\":\"vec_tail\"},"
         + "   {\"src\":\"st_gen3\",\"evt\":\"vec_tail\"},"
         + "   {\"src\":\"st_ana1\",\"evt\":\"vec_tail\"},"
         + "   {\"src\":\"arith1\", \"evt\":\"vec_tail\"},"
         + "   {\"src\":\"arith2\", \"evt\":\"vec_tail\"},"
         + "   {\"src\":\"xbar\",   \"evt\":\"st_gen1.complete\" },"
         + "   {\"src\":\"xbar\",   \"evt\":\"st_gen2.complete\" },"
         + "   {\"src\":\"xbar\",   \"evt\":\"st_gen3.complete\" },"
         + "   {\"src\":\"xbar\",   \"evt\":\"arith1.complete\"  },"
         + "   {\"src\":\"xbar\",   \"evt\":\"arith2.complete\"  }"
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
         + "  [{\"src\":\"st_gen1\",\"evt\":\"vec_tail\"},"
         + "   {\"src\":\"st_gen2\",\"evt\":\"vec_tail\"},"
         + "   {\"src\":\"st_gen3\",\"evt\":\"vec_tail\"},"
         + "   {\"src\":\"st_ana1\",\"evt\":\"vec_tail\"},"
         + "   {\"src\":\"arith1\", \"evt\":\"vec_tail\"},"
         + "   {\"src\":\"arith2\", \"evt\":\"vec_tail\"},"
         + "   {\"src\":\"xbar\",   \"evt\":\"st_gen1.complete\" },"
         + "   {\"src\":\"xbar\",   \"evt\":\"st_gen2.complete\" },"
         + "   {\"src\":\"xbar\",   \"evt\":\"st_gen3.complete\" },"
         + "   {\"src\":\"xbar\",   \"evt\":\"arith1.complete\"  },"
         + "   {\"src\":\"xbar\",   \"evt\":\"arith2.complete\"  }"
         + "  ],"
         + " \"error_any\":"
         + "  [{\"src\":\"st_gen1\", \"evt\":\"vec_head\"},"
         + "   {\"src\":\"st_gen2\", \"evt\":\"vec_head\"},"
         + "   {\"src\":\"st_gen3\", \"evt\":\"vec_head\"},"
         + "   {\"src\":\"st_ana1\", \"evt\":\"vec_head\"},"
         + "   {\"src\":\"arith1\",  \"evt\":\"vec_head\"},"
         + "   {\"src\":\"arith2\",  \"evt\":\"vec_head\"}"
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
