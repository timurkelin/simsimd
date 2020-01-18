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
      //*************************************

      // Configure XBAR for initial transaction
      sc_core::wait();
      wr_conf( std::string("")
         + "{ \"dst\":[\"xbar\"],\"idx\":0,\"cmd\":\"put\",\"data\":"
         + "    {\"exec_idx\":101,\"conf_next\":8,\"routing\":"
         + "       [{\"mod\":\"sti_1\",\"port\":0,\"event\":1,\"dst\":"
         + "           [{\"mod\":\"dm_1\",\"port\":0,\"master\":1"
         + "            },"
         + "            {\"mod\":\"ana_1\",\"port\":0,\"master\":0"
         + "            }"
         + "           ]"
         + "        },"
         + "        {\"mod\":\"sti_2\",\"port\":0,\"event\":1,\"dst\":"
         + "           [{\"mod\":\"dm_2\",\"port\":0,\"master\":1"
         + "            },"
         + "            {\"mod\":\"ana_2\",\"port\":0,\"master\":0"
         + "            }"
         + "           ]"
         + "        }"
         + "       ]"
         + "    }"
         + "}" );
      SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 01.01";

      // Configure Input Stream 1
      sc_core::wait();
      wr_conf( std::string("")
         + "{ \"dst\":[\"sti_1\"],\"idx\":0,\"cmd\":\"put\",\"data\":"
         + "    {\"exec_idx\":101,\"stat_idx\":0,\"conf_next\":8,\"events\":[\"vec_head\",\"vec_tail\"]," // no events for this stage
         + "     \"pool_seg\":\"seg_rd0\",\"blk_start\":0,\"blk_size\":\"800\","
         + "     \"ws_min\":" + std::to_string( rng_ws_min ) + ","
         + "     \"ws_max\":" + std::to_string( rng_ws_min + rng_ws())
         + "    }"
         + "}" );
      SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 01.02";

      // DM 1
      sc_core::wait();
      wr_conf( std::string("")
         + "{ \"dst\":[\"dm_1\"],\"idx\":0,\"cmd\":\"put\",\"data\":"
         + "    {\"exec_idx\":101,\"stat_idx\":0,\"conf_next\":8,\"events\":[\"wr_vec_head\",\"wr_vec_tail\",\"rd_vec_head\",\"rd_vec_tail\"],"
         + "     \"op_mode\":\"write\",\"smp_num\":200,\"ag_conf\":"
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

      // Configure analyzer 1
      sc_core::wait();
      wr_conf( std::string("")
         + "{ \"dst\":[\"ana_1\"],\"idx\":0,\"cmd\":\"put\",\"data\":"
         + "    {\"exec_idx\":101,\"stat_idx\":0,\"conf_next\":8,\"events\":[\"vec_head\",\"vec_tail\"],"
         + "     \"ready_ovr\":1" // To operate as Slave
         + "    }"
         + "}" );
      SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 01.04";

      // Configure Input Stream 2
      sc_core::wait();
      wr_conf( std::string("")
      + "{ \"dst\":[\"sti_2\"],\"idx\":0,\"cmd\":\"put\",\"data\":"
      + "    {\"exec_idx\":101,\"stat_idx\":0,\"conf_next\":8,\"events\":[\"vec_head\",\"vec_tail\"],"
      + "     \"pool_seg\":\"seg_rdwr1\",\"blk_start\":512,\"blk_size\":\"800\","
      + "     \"ws_min\":" + std::to_string( rng_ws_min ) + ","
      + "     \"ws_max\":" + std::to_string( rng_ws_min + rng_ws())
      + "    }"
      + "}" );
      SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 01.05";

      // DM 2
      sc_core::wait();
      wr_conf( std::string("")
         + "{ \"dst\":[\"dm_2\"],\"idx\":0,\"cmd\":\"put\",\"data\":"
         + "    {\"exec_idx\":101,\"stat_idx\":0,\"conf_next\":8,\"events\":[\"wr_vec_head\",\"wr_vec_tail\",\"rd_vec_head\",\"rd_vec_tail\"],"
         + "     \"op_mode\":\"write\",\"smp_num\":200,\"ag_conf\":"
         + "       {\"mode\":\"linear\", \"param\":"
         + "          [{\"offset\":0, \"ena\":1, \"perm\":0 },"
         + "           {\"offset\":0, \"ena\":1, \"perm\":1 },"
         + "           {\"offset\":0, \"ena\":1, \"perm\":2 },"
         + "           {\"offset\":0, \"ena\":1, \"perm\":3 } "
         + "          ]"
         + "       }"
         + "    }"
         + "}" );
      SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 01.06";

      // Configure analyzer 2
      sc_core::wait();
      wr_conf( std::string("")
         + "{ \"dst\":[\"ana_2\"],\"idx\":0,\"cmd\":\"put\",\"data\":"
         + "    {\"exec_idx\":101,\"stat_idx\":0,\"conf_next\":8,\"events\":[\"vec_head\",\"vec_tail\"],"
         + "     \"ready_ovr\":1" // To operate as Slave
         + "    }"
         + "}" );
      SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 01.07";

      // Configure XBAR for computation
      sc_core::wait();
      wr_conf( std::string("")
         + "{ \"dst\":[\"xbar\"],\"idx\":1,\"cmd\":\"put\",\"data\":"
         + "    {\"exec_idx\":102,\"conf_next\":8,\"routing\":"
         + "       [{\"mod\":\"dm_1\",\"port\":0,\"event\":1,\"dst\":"
         + "           [{\"mod\":\"arith_1\",\"port\":0,\"master\":1"
         + "            },"
         + "            {\"mod\":\"ana_1\",\"port\":0,\"master\":0"
         + "            }"
         + "           ]"
         + "        },"
         + "        {\"mod\":\"dm_2\",\"port\":0,\"event\":1,\"dst\":"
         + "           [{\"mod\":\"arith_1\",\"port\":1,\"master\":1"
         + "            },"
         + "            {\"mod\":\"ana_2\",\"port\":0,\"master\":0"
         + "            }"
         + "           ]"
         + "        },"
         + "        {\"mod\":\"arith_1\",\"port\":0,\"event\":1,\"dst\":"
         + "           [{\"mod\":\"arith_2\",\"port\":0,\"master\":1"
         + "            }"
         + "           ]"
         + "        },"
         + "        {\"mod\":\"gen_1\",\"port\":0,\"event\":1,\"dst\":"
         + "           [{\"mod\":\"arith_2\",\"port\":1,\"master\":1"
         + "            }"
         + "           ]"
         + "        },"
         + "        {\"mod\":\"arith_2\",\"port\":0,\"event\":1,\"dst\":"
         + "           [{\"mod\":\"dm_3\",\"port\":0,\"master\":1"
         + "            },"
         + "            {\"mod\":\"ana_3\",\"port\":0,\"master\":0"
         + "            }"
         + "           ]"
         + "        }"
         + "       ]"
         + "    }"
         + "}" );
      SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 02.01";

      // DM 1
      sc_core::wait();
      wr_conf( std::string("")
         + "{ \"dst\":[\"dm_1\"],\"idx\":1,\"cmd\":\"put\",\"data\":"
         + "    {\"exec_idx\":102,\"stat_idx\":0,\"conf_next\":8,\"events\":[\"wr_vec_head\",\"wr_vec_tail\",\"rd_vec_head\",\"rd_vec_tail\"],"
         + "     \"op_mode\":\"read\",\"smp_num\":200,\"ag_conf\":"
         + "       {\"mode\":\"linear\", \"param\":"
         + "          [{\"offset\":0, \"ena\":1, \"perm\":0 },"
         + "           {\"offset\":0, \"ena\":1, \"perm\":1 },"
         + "           {\"offset\":0, \"ena\":1, \"perm\":2 },"
         + "           {\"offset\":0, \"ena\":1, \"perm\":3 } "
         + "          ]"
         + "       }"
         + "    }"
         + "}" );
      SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 02.02";

      // Configure analyzer 1
      sc_core::wait();
      wr_conf( std::string("")
         + "{ \"dst\":[\"ana_1\"],\"idx\":1,\"cmd\":\"put\",\"data\":"
         + "    {\"exec_idx\":102,\"stat_idx\":0,\"conf_next\":8,\"events\":[\"vec_head\",\"vec_tail\"]," // no events for this stage
         + "     \"ready_ovr\":1" // To operate as Slave
         + "    }"
         + "}" );
      SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 02.03";

      // DM 1
      sc_core::wait();
      wr_conf( std::string("")
         + "{ \"dst\":[\"dm_2\"],\"idx\":1,\"cmd\":\"put\",\"data\":"
         + "    {\"exec_idx\":102,\"stat_idx\":0,\"conf_next\":8,\"events\":[\"wr_vec_head\",\"wr_vec_tail\",\"rd_vec_head\",\"rd_vec_tail\"],"
         + "     \"op_mode\":\"read\",\"smp_num\":200,\"ag_conf\":"
         + "       {\"mode\":\"linear\", \"param\":"
         + "          [{\"offset\":0, \"ena\":1, \"perm\":0 },"
         + "           {\"offset\":0, \"ena\":1, \"perm\":1 },"
         + "           {\"offset\":0, \"ena\":1, \"perm\":2 },"
         + "           {\"offset\":0, \"ena\":1, \"perm\":3 } "
         + "          ]"
         + "       }"
         + "    }"
         + "}" );
      SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 02.04";

      // Configure analyzer 1
      sc_core::wait();
      wr_conf( std::string("")
         + "{ \"dst\":[\"ana_2\"],\"idx\":1,\"cmd\":\"put\",\"data\":"
         + "    {\"exec_idx\":102,\"stat_idx\":0,\"conf_next\":8,\"events\":[\"vec_head\",\"vec_tail\"]," // no events for this stage
         + "     \"ready_ovr\":1" // To operate as Slave
         + "    }"
         + "}" );
      SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 02.05";

      // Configure ADD/SUB EU 1
      sc_core::wait();
      wr_conf( std::string("")
         + "{ \"dst\":[\"arith_1\"],\"idx\":1,\"cmd\":\"put\",\"data\":"
         + "    {\"exec_idx\":102,\"stat_idx\":0,\"conf_next\":8,\"events\":[\"vec_head\",\"vec_tail\"]," // no events for this stage
         + "     \"neg_re0\":\"0\",\"neg_im0\":\"0\",\"neg_re1\":\"0\",\"neg_im1\":\"0\",\"smp_nena_op\":\"error\""
         + "    }"
         + "}" );
      SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 02.06";

      // Configure VRI Source 3
      sc_core::wait();
      wr_conf( std::string("")
         + "{ \"dst\":[\"gen_1\"],\"idx\":1,\"cmd\":\"put\",\"data\":"
         + "    {\"exec_idx\":102,\"stat_idx\":0,\"conf_next\":8,\"events\":[\"vec_head\",\"vec_tail\"]," // no events for this stage
         + "     \"vec_size\":\"200\",\"slot_ena_ovr\":1,\"valid_ovr\":1"
         + "    }"
         + "}" );
      SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 02.07";

      // Configure ADD/SUB EU 2
      sc_core::wait();
      wr_conf( std::string("")
         + "{ \"dst\":[\"arith_2\"],\"idx\":1,\"cmd\":\"put\",\"data\":"
         + "    {\"exec_idx\":102,\"stat_idx\":0,\"conf_next\":8,\"events\":[\"vec_head\",\"vec_tail\"]," // no events for this stage
         + "     \"neg_re0\":\"0\",\"neg_im0\":\"0\",\"neg_re1\":\"1\",\"neg_im1\":\"1\",\"smp_nena_op\":\"error\"" // Subtract on port1
         + "    }"
         + "}" );
      SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 02.08";

      // DM 3
      sc_core::wait();
      wr_conf( std::string("")
         + "{ \"dst\":[\"dm_3\"],\"idx\":1,\"cmd\":\"put\",\"data\":"
         + "    {\"exec_idx\":102,\"stat_idx\":0,\"conf_next\":8,\"events\":[\"wr_vec_head\",\"wr_vec_tail\",\"rd_vec_head\",\"rd_vec_tail\"],"
         + "     \"op_mode\":\"write\",\"smp_num\":200,\"ag_conf\":"
         + "       {\"mode\":\"linear\", \"param\":"
         + "          [{\"offset\":0, \"ena\":1, \"perm\":0 },"
         + "           {\"offset\":0, \"ena\":1, \"perm\":1 },"
         + "           {\"offset\":0, \"ena\":1, \"perm\":2 },"
         + "           {\"offset\":0, \"ena\":1, \"perm\":3 } "
         + "          ]"
         + "       }"
         + "    }"
         + "}" );
      SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 02.09";

      // Configure analyzer 3
      sc_core::wait();
      wr_conf( std::string("")
         + "{ \"dst\":[\"ana_3\"],\"idx\":1,\"cmd\":\"put\",\"data\":"
         + "    {\"exec_idx\":102,\"stat_idx\":0,\"conf_next\":8,\"events\":[\"vec_head\",\"vec_tail\"]," // no events for this stage
         + "     \"ready_ovr\":1" // To operate as Slave
         + "    }"
         + "}" );
      SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 02.10";

      // Configure XBAR for the subsequent transactions
      sc_core::wait();
      wr_conf( std::string("")
         + "{ \"dst\":[\"xbar\"],\"idx\":2,\"cmd\":\"put\",\"data\":"
         + "    {\"exec_idx\":103,\"conf_next\":8,\"routing\":"
         + "       [{\"mod\":\"sti_1\",\"port\":0,\"event\":1,\"dst\":"
         + "           [{\"mod\":\"dm_1\",\"port\":0,\"master\":1"
         + "            },"
         + "            {\"mod\":\"ana_1\",\"port\":0,\"master\":0"
         + "            }"
         + "           ]"
         + "        },"
         + "        {\"mod\":\"sti_2\",\"port\":0,\"event\":1,\"dst\":"
         + "           [{\"mod\":\"dm_2\",\"port\":0,\"master\":1"
         + "            },"
         + "            {\"mod\":\"ana_2\",\"port\":0,\"master\":0"
         + "            }"
         + "           ]"
         + "        },"
         + "        {\"mod\":\"dm_3\",\"port\":0,\"event\":1,\"dst\":"
         + "           [{\"mod\":\"sto_1\",\"port\":0,\"master\":1"
         + "            },"
         + "            {\"mod\":\"ana_3\",\"port\":0,\"master\":0"
         + "            }"
         + "           ]"
         + "        }"
         + "       ]"
         + "    }"
         + "}" );
      SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 03.01";

      // Configure Input Stream 1
      sc_core::wait();
      wr_conf( std::string("")
         + "{ \"dst\":[\"sti_1\"],\"idx\":2,\"cmd\":\"put\",\"data\":"
         + "    {\"exec_idx\":103,\"stat_idx\":0,\"conf_next\":8,\"events\":[\"vec_head\",\"vec_tail\"]," // no events for this stage
         + "     \"pool_seg\":\"seg_rd0\",\"blk_start\":0,\"blk_size\":\"800\","
         + "     \"ws_min\":" + std::to_string( rng_ws_min ) + ","
         + "     \"ws_max\":" + std::to_string( rng_ws_min + rng_ws())
         + "    }"
         + "}" );
      SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 03.02";

      // DM 1
      sc_core::wait();
      wr_conf( std::string("")
         + "{ \"dst\":[\"dm_1\"],\"idx\":2,\"cmd\":\"put\",\"data\":"
         + "    {\"exec_idx\":103,\"stat_idx\":0,\"conf_next\":8,\"events\":[\"wr_vec_head\",\"wr_vec_tail\",\"rd_vec_head\",\"rd_vec_tail\"],"
         + "     \"op_mode\":\"write\",\"smp_num\":200,\"ag_conf\":"
         + "       {\"mode\":\"linear\", \"param\":"
         + "          [{\"offset\":0, \"ena\":1, \"perm\":0 },"
         + "           {\"offset\":0, \"ena\":1, \"perm\":1 },"
         + "           {\"offset\":0, \"ena\":1, \"perm\":2 },"
         + "           {\"offset\":0, \"ena\":1, \"perm\":3 } "
         + "          ]"
         + "       }"
         + "    }"
         + "}" );
      SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 03.03";

      // Configure analyzer 1
      sc_core::wait();
      wr_conf( std::string("")
         + "{ \"dst\":[\"ana_1\"],\"idx\":2,\"cmd\":\"put\",\"data\":"
         + "    {\"exec_idx\":103,\"stat_idx\":0,\"conf_next\":8,\"events\":[\"vec_head\",\"vec_tail\"]," // no events for this stage
         + "     \"ready_ovr\":1" // To operate as Slave
         + "    }"
         + "}" );
      SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 03.04";

      // Configure Input Stream 2
      sc_core::wait();
      wr_conf( std::string("")
      + "{ \"dst\":[\"sti_2\"],\"idx\":2,\"cmd\":\"put\",\"data\":"
      + "    {\"exec_idx\":103,\"stat_idx\":0,\"conf_next\":8,\"events\":[\"vec_head\",\"vec_tail\"]," // no events for this stage
      + "     \"pool_seg\":\"seg_rdwr1\",\"blk_start\":512,\"blk_size\":\"800\","
      + "     \"ws_min\":" + std::to_string( rng_ws_min ) + ","
      + "     \"ws_max\":" + std::to_string( rng_ws_min + rng_ws())
      + "    }"
      + "}" );
      SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 03.05";

      // DM 2
      sc_core::wait();
      wr_conf( std::string("")
         + "{ \"dst\":[\"dm_2\"],\"idx\":2,\"cmd\":\"put\",\"data\":"
         + "    {\"exec_idx\":103,\"stat_idx\":0,\"conf_next\":8,\"events\":[\"wr_vec_head\",\"wr_vec_tail\",\"rd_vec_head\",\"rd_vec_tail\"],"
         + "     \"op_mode\":\"write\",\"smp_num\":200,\"ag_conf\":"
         + "       {\"mode\":\"linear\", \"param\":"
         + "          [{\"offset\":0, \"ena\":1, \"perm\":0 },"
         + "           {\"offset\":0, \"ena\":1, \"perm\":1 },"
         + "           {\"offset\":0, \"ena\":1, \"perm\":2 },"
         + "           {\"offset\":0, \"ena\":1, \"perm\":3 } "
         + "          ]"
         + "       }"
         + "    }"
         + "}" );
      SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 03.06";

      // Configure analyzer 2
      sc_core::wait();
      wr_conf( std::string("")
         + "{ \"dst\":[\"ana_2\"],\"idx\":2,\"cmd\":\"put\",\"data\":"
         + "    {\"exec_idx\":103,\"stat_idx\":0,\"conf_next\":8,\"events\":[\"vec_head\",\"vec_tail\"]," // no events for this stage
         + "     \"ready_ovr\":1" // To operate as Slave
         + "    }"
         + "}" );
      SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 03.07";

      // DM 3
      sc_core::wait();
      wr_conf( std::string("")
         + "{ \"dst\":[\"dm_3\"],\"idx\":2,\"cmd\":\"put\",\"data\":"
         + "    {\"exec_idx\":103,\"stat_idx\":0,\"conf_next\":8,\"events\":[\"wr_vec_head\",\"wr_vec_tail\",\"rd_vec_head\",\"rd_vec_tail\"],"
         + "     \"op_mode\":\"read\",\"smp_num\":200,\"ag_conf\":"
         + "       {\"mode\":\"linear\", \"param\":"
         + "          [{\"offset\":0, \"ena\":1, \"perm\":0 },"
         + "           {\"offset\":0, \"ena\":1, \"perm\":1 },"
         + "           {\"offset\":0, \"ena\":1, \"perm\":2 },"
         + "           {\"offset\":0, \"ena\":1, \"perm\":3 } "
         + "          ]"
         + "       }"
         + "    }"
         + "}" );
      SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 03.08";

      // Configure analyzer 3
      sc_core::wait();
      wr_conf( std::string("")
         + "{ \"dst\":[\"ana_3\"],\"idx\":2,\"cmd\":\"put\",\"data\":"
         + "    {\"exec_idx\":103,\"stat_idx\":0,\"conf_next\":8,\"events\":[\"vec_head\",\"vec_tail\"]," // no events for this stage
         + "     \"ready_ovr\":1" // To operate as Slave
         + "    }"
         + "}" );
      SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 03.09";

      // Configure Output Stream 1
      sc_core::wait();
      wr_conf( std::string("")
      + "{ \"dst\":[\"sto_1\"],\"idx\":2,\"cmd\":\"put\",\"data\":"
      + "    {\"exec_idx\":103,\"stat_idx\":0,\"conf_next\":8,\"events\":[\"vec_head\",\"vec_tail\"]," // no events for this stage
      + "     \"pool_seg\":\"seg_wr0\",\"blk_start\":512,"
      + "     \"ws_min\":" + std::to_string( rng_ws_min ) + ","
      + "     \"ws_max\":" + std::to_string( rng_ws_min + rng_ws())
      + "    }"
      + "}" );
      SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 03.10";

      // Configure XBAR for last transaction
      sc_core::wait();
      wr_conf( std::string("")
         + "{ \"dst\":[\"xbar\"],\"idx\":3,\"cmd\":\"put\",\"data\":"
         + "    {\"exec_idx\":104,\"conf_next\":8,\"routing\":"
         + "       [{\"mod\":\"dm_3\",\"port\":0,\"event\":1,\"dst\":"
         + "           [{\"mod\":\"sto_1\",\"port\":0,\"master\":1"
         + "            },"
         + "            {\"mod\":\"ana_3\",\"port\":0,\"master\":0"
         + "            }"
         + "           ]"
         + "        }"
         + "       ]"
         + "    }"
         + "}" );
      SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 04.01";

      // DM 3
      sc_core::wait();
      wr_conf( std::string("")
         + "{ \"dst\":[\"dm_3\"],\"idx\":3,\"cmd\":\"put\",\"data\":"
         + "    {\"exec_idx\":104,\"stat_idx\":0,\"conf_next\":8,\"events\":[\"wr_vec_head\",\"wr_vec_tail\",\"rd_vec_head\",\"rd_vec_tail\"],"
         + "     \"op_mode\":\"read\",\"smp_num\":200,\"ag_conf\":"
         + "       {\"mode\":\"linear\", \"param\":"
         + "          [{\"offset\":0, \"ena\":1, \"perm\":0 },"
         + "           {\"offset\":0, \"ena\":1, \"perm\":1 },"
         + "           {\"offset\":0, \"ena\":1, \"perm\":2 },"
         + "           {\"offset\":0, \"ena\":1, \"perm\":3 } "
         + "          ]"
         + "       }"
         + "    }"
         + "}" );
      SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 04.02";

      // Configure analyzer 3
      sc_core::wait();
      wr_conf( std::string("")
         + "{ \"dst\":[\"ana_3\"],\"idx\":3,\"cmd\":\"put\",\"data\":"
         + "    {\"exec_idx\":104,\"stat_idx\":0,\"conf_next\":8,\"events\":[\"vec_head\",\"vec_tail\"]," // no events for this stage
         + "     \"ready_ovr\":1" // To operate as Slave
         + "    }"
         + "}" );
      SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 04.03";

      // Configure Output Stream 1
      sc_core::wait();
      wr_conf( std::string("")
      + "{ \"dst\":[\"sto_1\"],\"idx\":3,\"cmd\":\"put\",\"data\":"
      + "    {\"exec_idx\":104,\"stat_idx\":0,\"conf_next\":8,\"events\":[\"vec_head\",\"vec_tail\"]," // no events for this stage
      + "     \"pool_seg\":\"seg_rdwr1\",\"blk_start\":512,"
      + "     \"ws_min\":" + std::to_string( rng_ws_min ) + ","
      + "     \"ws_max\":" + std::to_string( rng_ws_min + rng_ws())
      + "    }"
      + "}" );
      SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Config 03.10";

      // RUN
      std::size_t n_cyc_end = 2 + rng_ws();

      for( std::size_t n_cyc = 0; n_cyc <= n_cyc_end; n_cyc ++ ) {
         sc_core::wait();

         if( n_cyc == 0 ) {
            // Run the very first transaction
            wr_conf( std::string("")
               + "{ \"dst\":\"broadcast\",\"idx\":101,\"cmd\":\"run\""
               + "}" );
            SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Start  01.00";

            // Wait for the thread to start
            evt_proc_init( std::string("")
               + "{\"valid_all\":"
               + "  [{\"src\":\"sti_1\",   \"evt\":\"vec_head\"   },"
               + "   {\"src\":\"sti_2\",   \"evt\":\"vec_head\"   },"
               + "   {\"src\":\"dm_1\",    \"evt\":\"wr_vec_head\"},"
               + "   {\"src\":\"dm_2\",    \"evt\":\"wr_vec_head\"},"
               + "   {\"src\":\"ana_1\",   \"evt\":\"vec_head\"   },"
               + "   {\"src\":\"ana_2\",   \"evt\":\"vec_head\"   }"
               + "  ],"
               + " \"error_any\":"
               + "  [{\"src\":\"sti_1\",   \"evt\":\"vec_tail\"   },"
               + "   {\"src\":\"sti_2\",   \"evt\":\"vec_tail\"   },"
               + "   {\"src\":\"dm_1\",    \"evt\":\"wr_vec_tail\"},"
               + "   {\"src\":\"dm_2\",    \"evt\":\"wr_vec_tail\"},"
               + "   {\"src\":\"ana_1\",   \"evt\":\"vec_tail\"   },"
               + "   {\"src\":\"ana_2\",   \"evt\":\"vec_tail\"   },"

               + "   {\"src\":\"sto_1\",   \"evt\":\"vec_head\"   },"
               + "   {\"src\":\"dm_1\",    \"evt\":\"rd_vec_head\"},"
               + "   {\"src\":\"dm_2\",    \"evt\":\"rd_vec_head\"},"
               + "   {\"src\":\"dm_1\",    \"evt\":\"rd_vec_tail\"},"
               + "   {\"src\":\"dm_2\",    \"evt\":\"rd_vec_tail\"},"
               + "   {\"src\":\"dm_3\",    \"evt\":\"wr_vec_head\"},"
               + "   {\"src\":\"dm_3\",    \"evt\":\"rd_vec_head\"},"
               + "   {\"src\":\"ana_3\",   \"evt\":\"vec_head\"   },"
               + "   {\"src\":\"gen_1\",   \"evt\":\"vec_head\"   },"
               + "   {\"src\":\"arith_1\", \"evt\":\"vec_head\"   },"
               + "   {\"src\":\"arith_2\", \"evt\":\"vec_head\"   },"

               + "   {\"src\":\"sto_1\",   \"evt\":\"vec_tail\"   },"
               + "   {\"src\":\"dm_3\",    \"evt\":\"wr_vec_tail\"},"
               + "   {\"src\":\"dm_3\",    \"evt\":\"rd_vec_tail\"},"
               + "   {\"src\":\"ana_3\",   \"evt\":\"vec_tail\"   },"
               + "   {\"src\":\"gen_1\",   \"evt\":\"vec_tail\"   },"
               + "   {\"src\":\"arith_1\", \"evt\":\"vec_tail\"   },"
               + "   {\"src\":\"arith_2\", \"evt\":\"vec_tail\"   },"

               + "   {\"src\":\"xbar\",    \"evt\":\"sti_1.complete\"   },"
               + "   {\"src\":\"xbar\",    \"evt\":\"sti_2.complete\"   },"
               + "   {\"src\":\"xbar\",    \"evt\":\"dm_1.complete\"    },"
               + "   {\"src\":\"xbar\",    \"evt\":\"dm_2.complete\"    },"
               + "   {\"src\":\"xbar\",    \"evt\":\"dm_3.complete\"    },"
               + "   {\"src\":\"xbar\",    \"evt\":\"gen_1.complete\"   },"
               + "   {\"src\":\"xbar\",    \"evt\":\"arith_1.complete\" },"
               + "   {\"src\":\"xbar\",    \"evt\":\"arith_2.complete\" }"
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
            SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Check  01.01";

            // Wait for the completion
            evt_proc_init( std::string("")
            + "{\"error_any\":"
            + "  [{\"src\":\"sti_1\",   \"evt\":\"vec_head\"   },"
            + "   {\"src\":\"sti_2\",   \"evt\":\"vec_head\"   },"
            + "   {\"src\":\"dm_1\",    \"evt\":\"wr_vec_head\"},"
            + "   {\"src\":\"dm_2\",    \"evt\":\"wr_vec_head\"},"
            + "   {\"src\":\"ana_1\",   \"evt\":\"vec_head\"   },"
            + "   {\"src\":\"ana_2\",   \"evt\":\"vec_head\"   },"

            + "   {\"src\":\"sto_1\",   \"evt\":\"vec_head\"   },"
            + "   {\"src\":\"dm_1\",    \"evt\":\"rd_vec_head\"},"
            + "   {\"src\":\"dm_2\",    \"evt\":\"rd_vec_head\"},"
            + "   {\"src\":\"dm_1\",    \"evt\":\"rd_vec_tail\"},"
            + "   {\"src\":\"dm_2\",    \"evt\":\"rd_vec_tail\"},"
            + "   {\"src\":\"dm_3\",    \"evt\":\"wr_vec_head\"},"
            + "   {\"src\":\"dm_3\",    \"evt\":\"rd_vec_head\"},"
            + "   {\"src\":\"ana_3\",   \"evt\":\"vec_head\"   },"
            + "   {\"src\":\"gen_1\",   \"evt\":\"vec_head\"   },"
            + "   {\"src\":\"arith_1\", \"evt\":\"vec_head\"   },"
            + "   {\"src\":\"arith_2\", \"evt\":\"vec_head\"   },"

            + "   {\"src\":\"sto_1\",   \"evt\":\"vec_tail\"   },"
            + "   {\"src\":\"dm_3\",    \"evt\":\"wr_vec_tail\"},"
            + "   {\"src\":\"dm_3\",    \"evt\":\"rd_vec_tail\"},"
            + "   {\"src\":\"ana_3\",   \"evt\":\"vec_tail\"   },"
            + "   {\"src\":\"gen_1\",   \"evt\":\"vec_tail\"   },"
            + "   {\"src\":\"arith_1\", \"evt\":\"vec_tail\"   },"
            + "   {\"src\":\"arith_2\", \"evt\":\"vec_tail\"   },"

            + "   {\"src\":\"xbar\",    \"evt\":\"dm_1.complete\"    },"
            + "   {\"src\":\"xbar\",    \"evt\":\"dm_2.complete\"    },"
            + "   {\"src\":\"xbar\",    \"evt\":\"dm_3.complete\"    },"
            + "   {\"src\":\"xbar\",    \"evt\":\"gen_1.complete\"   },"
            + "   {\"src\":\"xbar\",    \"evt\":\"arith_1.complete\" },"
            + "   {\"src\":\"xbar\",    \"evt\":\"arith_2.complete\" }"
            + "  ],"
            + " \"valid_all\":"
            + "  [{\"src\":\"sti_1\",   \"evt\":\"vec_tail\"   },"
            + "   {\"src\":\"sti_2\",   \"evt\":\"vec_tail\"   },"
            + "   {\"src\":\"dm_1\",    \"evt\":\"wr_vec_tail\"},"
            + "   {\"src\":\"dm_2\",    \"evt\":\"wr_vec_tail\"},"
            + "   {\"src\":\"ana_1\",   \"evt\":\"vec_tail\"   },"
            + "   {\"src\":\"ana_2\",   \"evt\":\"vec_tail\"   },"

            + "   {\"src\":\"xbar\",    \"evt\":\"sti_1.complete\" },"
            + "   {\"src\":\"xbar\",    \"evt\":\"sti_2.complete\" }"
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
            SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Check  01.02";

         }
         else if( n_cyc < n_cyc_end ) {
            // Run a number of subsequent transactions
            wr_conf( std::string("")
               + "{ \"dst\":\"broadcast\",\"idx\":103,\"cmd\":\"run\""
               + "}" );
            SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Start  03.00";

            // Wait for the thread to start
            evt_proc_init( std::string("")
               + "{\"valid_all\":"
               + "  [{\"src\":\"sti_1\",   \"evt\":\"vec_head\"   },"
               + "   {\"src\":\"sti_2\",   \"evt\":\"vec_head\"   },"
               + "   {\"src\":\"dm_1\",    \"evt\":\"wr_vec_head\"},"
               + "   {\"src\":\"dm_2\",    \"evt\":\"wr_vec_head\"},"
               + "   {\"src\":\"ana_1\",   \"evt\":\"vec_head\"   },"
               + "   {\"src\":\"ana_2\",   \"evt\":\"vec_head\"   },"
               + "   {\"src\":\"dm_3\",    \"evt\":\"rd_vec_head\"},"
               + "   {\"src\":\"ana_3\",   \"evt\":\"vec_head\"   },"
               + "   {\"src\":\"sto_1\",   \"evt\":\"vec_head\"   }"
               + "  ],"
               + " \"error_any\":"
               + "  [{\"src\":\"sti_1\",   \"evt\":\"vec_tail\"   },"
               + "   {\"src\":\"sti_2\",   \"evt\":\"vec_tail\"   },"
               + "   {\"src\":\"dm_1\",    \"evt\":\"wr_vec_tail\"},"
               + "   {\"src\":\"dm_2\",    \"evt\":\"wr_vec_tail\"},"
               + "   {\"src\":\"ana_1\",   \"evt\":\"vec_tail\"   },"
               + "   {\"src\":\"ana_2\",   \"evt\":\"vec_tail\"   },"

               + "   {\"src\":\"dm_1\",    \"evt\":\"rd_vec_head\"},"
               + "   {\"src\":\"dm_2\",    \"evt\":\"rd_vec_head\"},"
               + "   {\"src\":\"dm_1\",    \"evt\":\"rd_vec_tail\"},"
               + "   {\"src\":\"dm_2\",    \"evt\":\"rd_vec_tail\"},"
               + "   {\"src\":\"dm_3\",    \"evt\":\"wr_vec_head\"},"
               + "   {\"src\":\"gen_1\",   \"evt\":\"vec_head\"   },"
               + "   {\"src\":\"arith_1\", \"evt\":\"vec_head\"   },"
               + "   {\"src\":\"arith_2\", \"evt\":\"vec_head\"   },"

               + "   {\"src\":\"sto_1\",   \"evt\":\"vec_tail\"   },"
               + "   {\"src\":\"dm_3\",    \"evt\":\"wr_vec_tail\"},"
               + "   {\"src\":\"dm_3\",    \"evt\":\"rd_vec_tail\"},"
               + "   {\"src\":\"ana_3\",   \"evt\":\"vec_tail\"   },"
               + "   {\"src\":\"gen_1\",   \"evt\":\"vec_tail\"   },"
               + "   {\"src\":\"arith_1\", \"evt\":\"vec_tail\"   },"
               + "   {\"src\":\"arith_2\", \"evt\":\"vec_tail\"   },"

               + "   {\"src\":\"xbar\",    \"evt\":\"sti_1.complete\"   },"
               + "   {\"src\":\"xbar\",    \"evt\":\"sti_2.complete\"   },"
               + "   {\"src\":\"xbar\",    \"evt\":\"dm_1.complete\"    },"
               + "   {\"src\":\"xbar\",    \"evt\":\"dm_2.complete\"    },"
               + "   {\"src\":\"xbar\",    \"evt\":\"dm_3.complete\"    },"
               + "   {\"src\":\"xbar\",    \"evt\":\"gen_1.complete\"   },"
               + "   {\"src\":\"xbar\",    \"evt\":\"arith_1.complete\" },"
               + "   {\"src\":\"xbar\",    \"evt\":\"arith_2.complete\" }"
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
            SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Check  03.01";

            // Wait for the completion
            evt_proc_init( std::string("")
            + "{\"error_any\":"
            + "  [{\"src\":\"sti_1\",   \"evt\":\"vec_head\"   },"
            + "   {\"src\":\"sti_2\",   \"evt\":\"vec_head\"   },"
            + "   {\"src\":\"dm_1\",    \"evt\":\"wr_vec_head\"},"
            + "   {\"src\":\"dm_2\",    \"evt\":\"wr_vec_head\"},"
            + "   {\"src\":\"ana_1\",   \"evt\":\"vec_head\"   },"
            + "   {\"src\":\"ana_2\",   \"evt\":\"vec_head\"   },"

            + "   {\"src\":\"sto_1\",   \"evt\":\"vec_head\"   },"
            + "   {\"src\":\"dm_1\",    \"evt\":\"rd_vec_head\"},"
            + "   {\"src\":\"dm_2\",    \"evt\":\"rd_vec_head\"},"
            + "   {\"src\":\"dm_1\",    \"evt\":\"rd_vec_tail\"},"
            + "   {\"src\":\"dm_2\",    \"evt\":\"rd_vec_tail\"},"
            + "   {\"src\":\"dm_3\",    \"evt\":\"wr_vec_head\"},"
            + "   {\"src\":\"dm_3\",    \"evt\":\"rd_vec_head\"},"
            + "   {\"src\":\"ana_3\",   \"evt\":\"vec_head\"   },"
            + "   {\"src\":\"gen_1\",   \"evt\":\"vec_head\"   },"
            + "   {\"src\":\"arith_1\", \"evt\":\"vec_head\"   },"
            + "   {\"src\":\"arith_2\", \"evt\":\"vec_head\"   },"

            + "   {\"src\":\"dm_3\",    \"evt\":\"wr_vec_tail\"},"
            + "   {\"src\":\"gen_1\",   \"evt\":\"vec_tail\"   },"
            + "   {\"src\":\"arith_1\", \"evt\":\"vec_tail\"   },"
            + "   {\"src\":\"arith_2\", \"evt\":\"vec_tail\"   },"

            + "   {\"src\":\"xbar\",    \"evt\":\"dm_1.complete\"    },"
            + "   {\"src\":\"xbar\",    \"evt\":\"dm_2.complete\"    },"
            + "   {\"src\":\"xbar\",    \"evt\":\"gen_1.complete\"   },"
            + "   {\"src\":\"xbar\",    \"evt\":\"arith_1.complete\" },"
            + "   {\"src\":\"xbar\",    \"evt\":\"arith_2.complete\" }"
            + "  ],"
            + " \"valid_all\":"
            + "  [{\"src\":\"sti_1\",   \"evt\":\"vec_tail\"   },"
            + "   {\"src\":\"sti_2\",   \"evt\":\"vec_tail\"   },"
            + "   {\"src\":\"dm_1\",    \"evt\":\"wr_vec_tail\"},"
            + "   {\"src\":\"dm_2\",    \"evt\":\"wr_vec_tail\"},"
            + "   {\"src\":\"ana_1\",   \"evt\":\"vec_tail\"   },"
            + "   {\"src\":\"ana_2\",   \"evt\":\"vec_tail\"   },"
            + "   {\"src\":\"dm_3\",    \"evt\":\"rd_vec_tail\"},"
            + "   {\"src\":\"ana_3\",   \"evt\":\"vec_tail\"   },"
            + "   {\"src\":\"sto_1\",   \"evt\":\"vec_tail\"   },"

            + "   {\"src\":\"xbar\",    \"evt\":\"sti_1.complete\" },"
            + "   {\"src\":\"xbar\",    \"evt\":\"sti_2.complete\" },"
            + "   {\"src\":\"xbar\",    \"evt\":\"dm_3.complete\"  }"
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
            SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Check  03.02";

         }
         else {
            // Run the last transaction
            wr_conf( std::string("")
               + "{ \"dst\":\"broadcast\",\"idx\":104,\"cmd\":\"run\""
               + "}" );
            SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Start  04.00";

            // Wait for the thread to start
            evt_proc_init( std::string("")
               + "{\"valid_all\":"
               + "  [{\"src\":\"dm_3\",    \"evt\":\"rd_vec_head\"},"
               + "   {\"src\":\"ana_3\",   \"evt\":\"vec_head\"   },"
               + "   {\"src\":\"sto_1\",   \"evt\":\"vec_head\"   }"
               + "  ],"
               + " \"error_any\":"
               + "  [{\"src\":\"sti_1\",   \"evt\":\"vec_head\"   },"
               + "   {\"src\":\"sti_2\",   \"evt\":\"vec_head\"   },"
               + "   {\"src\":\"dm_1\",    \"evt\":\"wr_vec_head\"},"
               + "   {\"src\":\"dm_2\",    \"evt\":\"wr_vec_head\"},"
               + "   {\"src\":\"ana_1\",   \"evt\":\"vec_head\"   },"
               + "   {\"src\":\"ana_2\",   \"evt\":\"vec_head\"   },"
               + "   {\"src\":\"sti_1\",   \"evt\":\"vec_tail\"   },"
               + "   {\"src\":\"sti_2\",   \"evt\":\"vec_tail\"   },"
               + "   {\"src\":\"dm_1\",    \"evt\":\"wr_vec_tail\"},"
               + "   {\"src\":\"dm_2\",    \"evt\":\"wr_vec_tail\"},"
               + "   {\"src\":\"ana_1\",   \"evt\":\"vec_tail\"   },"
               + "   {\"src\":\"ana_2\",   \"evt\":\"vec_tail\"   },"

               + "   {\"src\":\"dm_1\",    \"evt\":\"rd_vec_head\"},"
               + "   {\"src\":\"dm_2\",    \"evt\":\"rd_vec_head\"},"
               + "   {\"src\":\"dm_1\",    \"evt\":\"rd_vec_tail\"},"
               + "   {\"src\":\"dm_2\",    \"evt\":\"rd_vec_tail\"},"
               + "   {\"src\":\"dm_3\",    \"evt\":\"wr_vec_head\"},"
               + "   {\"src\":\"gen_1\",   \"evt\":\"vec_head\"   },"
               + "   {\"src\":\"arith_1\", \"evt\":\"vec_head\"   },"
               + "   {\"src\":\"arith_2\", \"evt\":\"vec_head\"   },"

               + "   {\"src\":\"sto_1\",   \"evt\":\"vec_tail\"   },"
               + "   {\"src\":\"dm_3\",    \"evt\":\"wr_vec_tail\"},"
               + "   {\"src\":\"dm_3\",    \"evt\":\"rd_vec_tail\"},"
               + "   {\"src\":\"ana_3\",   \"evt\":\"vec_tail\"   },"
               + "   {\"src\":\"gen_1\",   \"evt\":\"vec_tail\"   },"
               + "   {\"src\":\"arith_1\", \"evt\":\"vec_tail\"   },"
               + "   {\"src\":\"arith_2\", \"evt\":\"vec_tail\"   },"

               + "   {\"src\":\"xbar\",    \"evt\":\"sti_1.complete\"   },"
               + "   {\"src\":\"xbar\",    \"evt\":\"sti_2.complete\"   },"
               + "   {\"src\":\"xbar\",    \"evt\":\"dm_1.complete\"    },"
               + "   {\"src\":\"xbar\",    \"evt\":\"dm_2.complete\"    },"
               + "   {\"src\":\"xbar\",    \"evt\":\"dm_3.complete\"    },"
               + "   {\"src\":\"xbar\",    \"evt\":\"gen_1.complete\"   },"
               + "   {\"src\":\"xbar\",    \"evt\":\"arith_1.complete\" },"
               + "   {\"src\":\"xbar\",    \"evt\":\"arith_2.complete\" }"
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
            SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Check  04.01";

            // Wait for the completion
            evt_proc_init( std::string("")
            + "{\"error_any\":"
            + "  [{\"src\":\"sti_1\",   \"evt\":\"vec_head\"   },"
            + "   {\"src\":\"sti_2\",   \"evt\":\"vec_head\"   },"
            + "   {\"src\":\"dm_1\",    \"evt\":\"wr_vec_head\"},"
            + "   {\"src\":\"dm_2\",    \"evt\":\"wr_vec_head\"},"
            + "   {\"src\":\"ana_1\",   \"evt\":\"vec_head\"   },"
            + "   {\"src\":\"ana_2\",   \"evt\":\"vec_head\"   },"

            + "   {\"src\":\"sto_1\",   \"evt\":\"vec_head\"   },"
            + "   {\"src\":\"dm_1\",    \"evt\":\"rd_vec_head\"},"
            + "   {\"src\":\"dm_2\",    \"evt\":\"rd_vec_head\"},"
            + "   {\"src\":\"dm_1\",    \"evt\":\"rd_vec_tail\"},"
            + "   {\"src\":\"dm_2\",    \"evt\":\"rd_vec_tail\"},"
            + "   {\"src\":\"dm_3\",    \"evt\":\"wr_vec_head\"},"
            + "   {\"src\":\"dm_3\",    \"evt\":\"rd_vec_head\"},"
            + "   {\"src\":\"ana_3\",   \"evt\":\"vec_head\"   },"
            + "   {\"src\":\"gen_1\",   \"evt\":\"vec_head\"   },"
            + "   {\"src\":\"arith_1\", \"evt\":\"vec_head\"   },"
            + "   {\"src\":\"arith_2\", \"evt\":\"vec_head\"   },"

            + "   {\"src\":\"dm_3\",    \"evt\":\"wr_vec_tail\"},"
            + "   {\"src\":\"gen_1\",   \"evt\":\"vec_tail\"   },"
            + "   {\"src\":\"arith_1\", \"evt\":\"vec_tail\"   },"
            + "   {\"src\":\"arith_2\", \"evt\":\"vec_tail\"   },"
            + "   {\"src\":\"sti_1\",   \"evt\":\"vec_tail\"   },"
            + "   {\"src\":\"sti_2\",   \"evt\":\"vec_tail\"   },"
            + "   {\"src\":\"dm_1\",    \"evt\":\"wr_vec_tail\"},"
            + "   {\"src\":\"dm_2\",    \"evt\":\"wr_vec_tail\"},"
            + "   {\"src\":\"ana_1\",   \"evt\":\"vec_tail\"   },"
            + "   {\"src\":\"ana_2\",   \"evt\":\"vec_tail\"   },"

            + "   {\"src\":\"xbar\",    \"evt\":\"sti_1.complete\"   },"
            + "   {\"src\":\"xbar\",    \"evt\":\"sti_2.complete\"   },"
            + "   {\"src\":\"xbar\",    \"evt\":\"dm_1.complete\"    },"
            + "   {\"src\":\"xbar\",    \"evt\":\"dm_2.complete\"    },"
            + "   {\"src\":\"xbar\",    \"evt\":\"gen_1.complete\"   },"
            + "   {\"src\":\"xbar\",    \"evt\":\"arith_1.complete\" },"
            + "   {\"src\":\"xbar\",    \"evt\":\"arith_2.complete\" }"
            + "  ],"
            + " \"valid_all\":"
            + "  [{\"src\":\"dm_3\",    \"evt\":\"rd_vec_tail\"},"
            + "   {\"src\":\"ana_3\",   \"evt\":\"vec_tail\"   },"
            + "   {\"src\":\"sto_1\",   \"evt\":\"vec_tail\"   },"

            + "   {\"src\":\"xbar\",    \"evt\":\"dm_3.complete\"    }"
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
            SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Check  04.02";

         }

         if( n_cyc < n_cyc_end ) {
            // Run the computation
            wr_conf( std::string("")
               + "{ \"dst\":\"broadcast\",\"idx\":102,\"cmd\":\"run\""
               + "}" );
            SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Start  02.00";

            // Wait for the thread to start
            evt_proc_init( std::string("")
               + "{\"valid_all\":"
               + "  [{\"src\":\"dm_1\",    \"evt\":\"rd_vec_head\"},"
               + "   {\"src\":\"dm_2\",    \"evt\":\"rd_vec_head\"},"
               + "   {\"src\":\"ana_1\",   \"evt\":\"vec_head\"   },"
               + "   {\"src\":\"ana_2\",   \"evt\":\"vec_head\"   },"
               + "   {\"src\":\"gen_1\",   \"evt\":\"vec_head\"   },"
               + "   {\"src\":\"arith_1\", \"evt\":\"vec_head\"   },"
               + "   {\"src\":\"arith_2\", \"evt\":\"vec_head\"   },"
               + "   {\"src\":\"ana_3\",   \"evt\":\"vec_head\"   },"
               + "   {\"src\":\"dm_3\",    \"evt\":\"wr_vec_head\"}"
               + "  ],"
               + " \"error_any\":"
               + "  [{\"src\":\"sti_1\",   \"evt\":\"vec_head\"   },"
               + "   {\"src\":\"sti_2\",   \"evt\":\"vec_head\"   },"
               + "   {\"src\":\"sti_1\",   \"evt\":\"vec_tail\"   },"
               + "   {\"src\":\"sti_2\",   \"evt\":\"vec_tail\"   },"
               + "   {\"src\":\"dm_1\",    \"evt\":\"wr_vec_head\"},"
               + "   {\"src\":\"dm_2\",    \"evt\":\"wr_vec_head\"},"
               + "   {\"src\":\"dm_1\",    \"evt\":\"wr_vec_tail\"},"
               + "   {\"src\":\"dm_2\",    \"evt\":\"wr_vec_tail\"},"
               + "   {\"src\":\"sti_1\",   \"evt\":\"vec_head\"   },"
               + "   {\"src\":\"sti_2\",   \"evt\":\"vec_head\"   },"
               + "   {\"src\":\"ana_1\",   \"evt\":\"vec_tail\"   },"
               + "   {\"src\":\"ana_2\",   \"evt\":\"vec_tail\"   },"

               + "   {\"src\":\"sto_1\",   \"evt\":\"vec_head\"   },"
               + "   {\"src\":\"sto_1\",   \"evt\":\"vec_tail\"   },"
               + "   {\"src\":\"dm_1\",    \"evt\":\"rd_vec_tail\"},"
               + "   {\"src\":\"dm_2\",    \"evt\":\"rd_vec_tail\"},"

               + "   {\"src\":\"dm_3\",    \"evt\":\"rd_vec_head\"},"
               + "   {\"src\":\"dm_3\",    \"evt\":\"rd_vec_tail\"},"
               + "   {\"src\":\"dm_3\",    \"evt\":\"wr_vec_tail\"},"

               + "   {\"src\":\"ana_3\",   \"evt\":\"vec_tail\"   },"
               + "   {\"src\":\"gen_1\",   \"evt\":\"vec_tail\"   },"
               + "   {\"src\":\"arith_1\", \"evt\":\"vec_tail\"   },"
               + "   {\"src\":\"arith_2\", \"evt\":\"vec_tail\"   },"

               + "   {\"src\":\"xbar\",    \"evt\":\"sti_1.complete\"   },"
               + "   {\"src\":\"xbar\",    \"evt\":\"sti_2.complete\"   },"
               + "   {\"src\":\"xbar\",    \"evt\":\"dm_1.complete\"    },"
               + "   {\"src\":\"xbar\",    \"evt\":\"dm_2.complete\"    },"
               + "   {\"src\":\"xbar\",    \"evt\":\"dm_3.complete\"    },"
               + "   {\"src\":\"xbar\",    \"evt\":\"gen_1.complete\"   },"
               + "   {\"src\":\"xbar\",    \"evt\":\"arith_1.complete\" },"
               + "   {\"src\":\"xbar\",    \"evt\":\"arith_2.complete\" }"
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
            SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Check  02.01";

            // Wait for the completion
            evt_proc_init( std::string("")
               + "{\"valid_all\":"
               + "  [{\"src\":\"dm_1\",    \"evt\":\"rd_vec_tail\"},"
               + "   {\"src\":\"dm_2\",    \"evt\":\"rd_vec_tail\"},"
               + "   {\"src\":\"ana_1\",   \"evt\":\"vec_tail\"   },"
               + "   {\"src\":\"ana_2\",   \"evt\":\"vec_tail\"   },"
               + "   {\"src\":\"gen_1\",   \"evt\":\"vec_tail\"   },"
               + "   {\"src\":\"arith_1\", \"evt\":\"vec_tail\"   },"
               + "   {\"src\":\"arith_2\", \"evt\":\"vec_tail\"   },"
               + "   {\"src\":\"ana_3\",   \"evt\":\"vec_tail\"   },"
               + "   {\"src\":\"dm_3\",    \"evt\":\"wr_vec_tail\"},"
               + "   {\"src\":\"xbar\",    \"evt\":\"dm_1.complete\"    },"
               + "   {\"src\":\"xbar\",    \"evt\":\"dm_2.complete\"    },"
               + "   {\"src\":\"xbar\",    \"evt\":\"gen_1.complete\"   },"
               + "   {\"src\":\"xbar\",    \"evt\":\"arith_1.complete\" },"
               + "   {\"src\":\"xbar\",    \"evt\":\"arith_2.complete\" }"
               + "  ],"
               + " \"error_any\":"
               + "  [{\"src\":\"sti_1\",   \"evt\":\"vec_head\"   },"
               + "   {\"src\":\"sti_2\",   \"evt\":\"vec_head\"   },"
               + "   {\"src\":\"sti_1\",   \"evt\":\"vec_tail\"   },"
               + "   {\"src\":\"sti_2\",   \"evt\":\"vec_tail\"   },"
               + "   {\"src\":\"dm_1\",    \"evt\":\"wr_vec_head\"},"
               + "   {\"src\":\"dm_2\",    \"evt\":\"wr_vec_head\"},"
               + "   {\"src\":\"dm_1\",    \"evt\":\"wr_vec_tail\"},"
               + "   {\"src\":\"dm_2\",    \"evt\":\"wr_vec_tail\"},"

               + "   {\"src\":\"sto_1\",   \"evt\":\"vec_head\"   },"
               + "   {\"src\":\"sto_1\",   \"evt\":\"vec_tail\"   },"

               + "   {\"src\":\"dm_1\",    \"evt\":\"rd_vec_head\"},"
               + "   {\"src\":\"dm_2\",    \"evt\":\"rd_vec_head\"},"
               + "   {\"src\":\"ana_1\",   \"evt\":\"vec_head\"   },"
               + "   {\"src\":\"ana_2\",   \"evt\":\"vec_head\"   },"
               + "   {\"src\":\"gen_1\",   \"evt\":\"vec_head\"   },"
               + "   {\"src\":\"arith_1\", \"evt\":\"vec_head\"   },"
               + "   {\"src\":\"arith_2\", \"evt\":\"vec_head\"   },"
               + "   {\"src\":\"ana_3\",   \"evt\":\"vec_head\"   },"
               + "   {\"src\":\"dm_3\",    \"evt\":\"wr_vec_head\"},"

               + "   {\"src\":\"dm_3\",    \"evt\":\"rd_vec_head\"},"
               + "   {\"src\":\"dm_3\",    \"evt\":\"rd_vec_tail\"},"

               + "   {\"src\":\"xbar\",    \"evt\":\"sti_1.complete\" },"
               + "   {\"src\":\"xbar\",    \"evt\":\"sti_2.complete\" },"
               + "   {\"src\":\"xbar\",    \"evt\":\"dm_3.complete\"  }"
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
            SIMD_REPORT_INFO( "simd::sys_scalar" ) << " Check  02.02";

         }

      } // for( std::size_t n_cyc = 0; n_cyc < n_cyc_end; n_cyc ++ )
   } // for( n_test = 0; ; n_test ++ ) {
}

} // namespace simd
