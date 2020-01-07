/*
 * simd_pref_init.cpp
 *
 *  Description:
 *    Default simulation preferences
 */

#include <string>
#include <utility>
#include "simd_conv_ptree.h"
#include "simd_pref.h"
#include "simd_report.h"

namespace simd {

std::string TIME_FS  = "fs";
std::string TIME_PS  = "ps";
std::string TIME_NS  = "ns";
std::string TIME_US  = "us";
std::string TIME_MS  = "ms";
std::string TIME_SEC = "s";

std::string FREQ_HZ  = "Hz";
std::string FREQ_KHZ = "kHz";
std::string FREQ_MHZ = "MHz";
std::string FREQ_GHZ = "GHz";

// Create elements of the preference structure
boost_pt::ptree init_trace(
      void ) {
   boost_pt::ptree elem;

   str2pt( std::string("")
      + "{ \"file\":\"trace\""
      + "}", elem );

   return elem;
}

boost_pt::ptree init_report(
      void ) {
   boost_pt::ptree elem;

   str2pt( std::string("")
      + "{ \"log_file\":\"simd.log\",\"handler\":\"simd\",\"bearing\":"
      + "    [{\"msg_type\":\"\","
      + "      \"info\":   {\"limit\":0,\"actions\":[\"log\", \"display\"]},"
      + "      \"warning\":{\"limit\":0,\"actions\":[\"log\", \"display\"]},"
      + "      \"error\":  {\"limit\":0,\"actions\":[\"log\", \"display\", \"cache_report\", \"throw\"]},"
      + "      \"fatal\":  {\"limit\":0,\"actions\":[\"log\", \"display\", \"cache_report\", \"throw\"]} "
      + "     },"
      + "     {\"msg_type\":\"simd:test\","
      + "      \"info\":   {\"limit\":0,\"actions\":[\"log\", \"display\"]},"
      + "      \"warning\":{\"limit\":0,\"actions\":[\"log\", \"display\"]},"
      + "      \"error\":  {\"limit\":0,\"actions\":[\"log\", \"display\", \"cache_report\", \"throw\"]},"
      + "      \"fatal\":  {\"limit\":0,\"actions\":[\"log\", \"display\", \"cache_report\", \"throw\"]} "
      + "     }"
      + "    ]"
      + "}", elem );

   return elem;
}

boost_pt::ptree init_time(
      void ) {
   boost_pt::ptree elem;

   str2pt( std::string("")
      + "{ \"resolution\":\"1.0" + TIME_NS + "\","
      + "  \"finish\":    \"1.0" + TIME_MS + "\""
      + "}", elem );

   return elem;
}

boost_pt::ptree init_clock(
      void ) {
   boost_pt::ptree elem;

   str2pt( std::string("")
      + "{ \"freq\":\"100.0" + FREQ_MHZ + "\""
      + "}", elem );

   return elem;
}

boost_pt::ptree init_core(
      void ) {
   boost_pt::ptree elem;

   str2pt( std::string("")
      + "[{\"name\":\"xbar\",\"function\":\"xbar\",\"param\":"
      + "  { \"config_slots\":6"
      + "  }"
      + " },"
      + " {\"name\":\"sti_1\",\"function\":\"st_inp_1\",\"param\":"  //  Input stream 1
      + "  { \"config_slots\":4,"
      + "    \"status_slots\":3,"
      + "    \"fifo_depth\":2"
      + "  }"
      + " },"
      + " {\"name\":\"sti_2\",\"function\":\"st_inp_1\",\"param\":"  //  Input stream 2
      + "  { \"config_slots\":4,"
      + "    \"status_slots\":3,"
      + "    \"fifo_depth\":2"
      + "  }"
      + " },"
      + " {\"name\":\"sto_1\",\"function\":\"st_out_1\",\"param\":"  // Output stream 1
      + "  { \"config_slots\":4,"
      + "    \"status_slots\":3,"
      + "    \"fifo_depth\":2"
      + "  }"
      + " },"
      + " {\"name\":\"dm_1\",\"function\":\"dm_ram_1rw\",\"param\":" // DM1 (read cache)
      + "  { \"config_slots\":4,"
      + "    \"status_slots\":3,"
      + "    \"fifo_depth\":2,"
      + "    \"size\":2048,"
      + "    \"init\":\"(1.2,3.4)\","
      + "    \"ag_modes\":[\"linear\"]"
      + "  }"
      + " },"
      + " {\"name\":\"dm_2\",\"function\":\"dm_ram_1rw\",\"param\":" // DM2 (read cache)
      + "  { \"config_slots\":4,"
      + "    \"status_slots\":3,"
      + "    \"fifo_depth\":2,"
      + "    \"size\":2048,"
      + "    \"init\":\"(1.2,3.4)\","
      + "    \"ag_modes\":[\"linear\"]"
      + "  }"
      + " },"
      + " {\"name\":\"dm_3\",\"function\":\"dm_ram_1rw\",\"param\":" // DM3 (write cache)
      + "  { \"config_slots\":4,"
      + "    \"status_slots\":3,"
      + "    \"fifo_depth\":2,"
      + "    \"size\":2048,"
      + "    \"init\":\"(1.2,3.4)\","
      + "    \"ag_modes\":[\"linear\"]"
      + "  }"
      + " },"
      + " {\"name\":\"ana_1\",\"function\":\"st_sigana_1\",\"param\":" // Analyzer 1 (sti_1/dm_1 output)
      + "  { \"config_slots\":4,"
      + "    \"status_slots\":3,"
      + "    \"fifo_depth\":2"
      + "  }"
      + " },"
      + " {\"name\":\"ana_2\",\"function\":\"st_sigana_1\",\"param\":" // Analyzer 2 (sti_1/dm_1 output)
      + "  { \"config_slots\":4,"
      + "    \"status_slots\":3,"
      + "    \"fifo_depth\":2"
      + "  }"
      + " },"
      + " {\"name\":\"ana_3\",\"function\":\"st_sigana_1\",\"param\":" // Analyzer 1 (arith2/dm_3 output)
      + "  { \"config_slots\":4,"
      + "    \"status_slots\":3,"
      + "    \"fifo_depth\":2"
      + "  }"
      + " },"
      + " {\"name\":\"gen_1\",\"function\":\"st_siggen_1\",\"param\":" // Generator 1 (for the result adjustment)
      + "  { \"config_slots\":4,"
      + "    \"status_slots\":3,"
      + "    \"fifo_depth\":2"
      + "  }"
      + " },"
      + " {\"name\":\"arith_1\",\"function\":\"eu_add_sub_2\",\"param\":" // Arithmetic unit 1
      + "  { \"config_slots\":4,"
      + "    \"status_slots\":3,"
      + "    \"fifo_depth\":2"
      + "  }"
      + " },"
      + " {\"name\":\"arith_2\",\"function\":\"eu_add_sub_2\",\"param\":" // Arithmetic unit 2 (for the result adjustment)
      + "  { \"config_slots\":4,"
      + "    \"status_slots\":3,"
      + "    \"fifo_depth\":2"
      + "  }"
      + " }"
      + "]", elem );

   return elem;
}

boost_pt::ptree init_dump(
      void ) {
   boost_pt::ptree elem;
   // Dump only data outputs of the EUs
   str2pt( std::string("")
      + "[{\"buf_regex\":\"^.*arith_1.*data_.*$\", \"time_start\":\"00.0" + TIME_US + "\", \"time_end\":\"finish\", \"size_res\":512, \"size_max\":1024, \"file\":\"dump_arith1.mat\""
      + " },"
      + " {\"buf_regex\":\"^.*arith_2.*data_.*$\", \"time_start\":\"00.0" + TIME_US + "\", \"time_end\":\"finish\", \"size_res\":512, \"size_max\":1024, \"file\":\"dump_arith2.mat\""
      + " },"
      + " {\"buf_regex\":\"^.*event.*data\",       \"time_start\":\"00.0" + TIME_US + "\", \"time_end\":\"finish\", \"size_res\":512, \"size_max\":1024, \"file\":\"dump_event.mat\""
      + " }"
      + "]", elem );
   return elem;
}

boost_pt::ptree init_pool(
      void ) {
   boost_pt::ptree elem;

   str2pt( std::string("")
      + "[{\"name\":\"seg_rd0\", \"size\":\"4096\", \"init\":"
      + "  [{\"base\":\"0\"   , \"file\":\"./examples/basic/adder_cache/mat/seg_init.mat\", \"var\":\"seg_init_data_a\""
      + "   },"
      + "   {\"base\":\"1024\", \"file\":\"./examples/basic/adder_cache/mat/seg_init.mat\", \"var\":\"seg_init_data_b\""
      + "   }"
      + "  ]"
      + " },"
      + " {\"name\":\"seg_rdwr1\", \"size\":\"auto\", \"init\":"
      + "  [{\"base\":\"512\" , \"file\":\"./examples/basic/adder_cache/mat/seg_init.mat\", \"var\":\"seg_init_data_c\""
      + "   },"
      + "   {\"base\":\"2048\", \"file\":\"./examples/basic/adder_cache/mat/seg_init.mat\", \"var\":\"seg_init_data_d\""
      + "   }"
      + "  ]"
      + " },"
      + " {\"name\":\"seg_wr0\", \"size\":\"8192\", \"init\":[]"
      + " }"
      + "]", elem );
   return elem;
}

boost_pt::ptree init_scalar(
      void ) {
   boost_pt::ptree elem;

   return elem;
}

void simd_pref_c::init(
      void ) {

   boost_pt::ptree elem;

   // Create overall structure
   root.put_child("report", init_report());
   root.put_child("trace",  init_trace() );
   root.put_child("dump",   init_dump()  );
   root.put_child("time",   init_time()  );
   root.put_child("core",   init_core()  );
   root.put_child("pool",   init_pool()  );
   root.put_child("clock",  init_clock() );
   root.put_child("scalar", init_scalar());

   return;
}

} // namespace simd
