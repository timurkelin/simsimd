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
      + "{ \"resolution\":\"1.0"   + TIME_NS + "\","
      + "  \"finish\":    \"100.0" + TIME_US + "\""
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
      + "  { \"config_slots\":4"
      + "  }"
      + " },"
      + " {\"name\":\"dm_1\",\"function\":\"dm_ram_1rw\",\"param\":"
      + "  { \"config_slots\":4,"
      + "    \"status_slots\":4,"
      + "    \"fifo_depth\":2,"
      + "    \"size\":4096,"
      + "    \"ag_modes\":[\"linear\"],"
      + "    \"init\":"
      + "    [{ \"base\":   0,\"file\":\"./examples/basic/dm_init/mat/dm_init.mat\",\"var\":\"dm_init_data_a\""
      + "     },"
      + "     { \"base\":1100,\"file\":\"./examples/basic/dm_init/mat/dm_init.mat\",\"var\":\"dm_init_data_b\""
      + "     },"
      + "     { \"base\":2100,\"file\":\"./examples/basic/dm_init/mat/dm_init.mat\",\"var\":\"dm_init_data_c\""
      + "     },"
      + "     { \"base\":3100,\"file\":\"./examples/basic/dm_init/mat/dm_init.mat\",\"var\":\"dm_init_data_d\""
      + "     }"
      + "    ]"
      + "  }"
      + " },"
      + " {\"name\":\"st_ana1\",\"function\":\"st_sigana_1\",\"param\":"
      + "  { \"config_slots\":4,"
      + "    \"status_slots\":4,"
      + "    \"fifo_depth\":2"
      + "  }"
      + " }"
      + "]", elem );

   return elem;
}

boost_pt::ptree init_dump(
      void ) {
   boost_pt::ptree elem;

   str2pt( std::string("")
      + "[{\"buf_regex\":\".*\", \"time_start\":\"00.0" + TIME_US + "\", \"time_end\":\"50.0" + TIME_US + "\", \"size_res\":512, \"size_max\":1024, \"file\":\"dump01.mat\""
      + " },"
      + " {\"buf_regex\":\".*\", \"time_start\":\"50.0" + TIME_US + "\", \"time_end\":\"finish\",              \"size_res\":512, \"size_max\":1024, \"file\":\"dump02.mat\""
      + " }"
      + "]", elem );

   return elem;
}

boost_pt::ptree init_pool(
      void ) {
   boost_pt::ptree elem;

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
