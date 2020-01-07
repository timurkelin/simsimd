/*
 * simd_sys_dm_ram_1r1w.cpp
 *
 *  Description:
 *    Methods of the 4-way 1r 1w register file with dual address buses
 */

#include <boost/foreach.hpp>
#include <boost/random.hpp>
#include <boost/functional/hash.hpp>
#include "simd_sys_dm_init.h"
#include "simd_sys_dm_ram_1r1w.h"
#include "simd_dump.h"
#include "simd_assert.h"
#include "simd_report.h"

namespace boost_rn = boost::random;

namespace simd {

void simd_sys_dm_ram_1r1w_c::init(
      boost::optional<const boost_pt::ptree&> _pref_p ) {
   const boost_pt::ptree& pref = _pref_p.get();
   pref_p = _pref_p;

   // Set port sizes
   data_vi.init( n_data_i );
   data_vo.init( n_data_o );

   // Set size of the internal signals
   ready_v.init( n_ready );

   // Set the vector sizes for the config and status slots
   try {
      std::size_t conf_size = pref.get<std::size_t>( "config_slots" );
      std::size_t stat_size = pref.get<std::size_t>( "status_slots" );
      std::size_t fifo_size = pref.get<std::size_t>( "fifo_depth" );

      if( conf_size > 0 ) {
         config.resize( conf_size );
      }
      else {
         SIMD_REPORT_ERROR( "simd::sys_dm" ) << name() << " Incorrect config size";
      }

      if( stat_size >= 0 ) {
         status.resize( stat_size );
      }
      else {
         SIMD_REPORT_ERROR( "simd::sys_dm" ) << name() << " Incorrect status size";
      }

      if( fifo_size > 0 ) {
         active_config_p = boost::optional<sc_core::sc_fifo<std::size_t>&>(
               *( new sc_core::sc_fifo<std::size_t>( std::string( name()).append( "_idx_fifo" ).c_str(), fifo_size )));
      }

      active_config = conf_size - 1;
      active_status = stat_size ? stat_size - 1 : 0;
   }
   catch( const boost_pt::ptree_error& err ) {
      SIMD_REPORT_ERROR( "simd::sys_dm" ) << name() << " " <<  err.what();
   }
   catch( const std::exception& err ) {
      SIMD_REPORT_ERROR( "simd::sys_dm" ) << name() << " " <<  err.what();
   }
   catch( ... ) {
      SIMD_REPORT_ERROR( "simd::sys_dm" ) << name() << " Unexpected";
   }

   // Set memory size
   try {
      std::size_t mem_size = pref.get<std::size_t>( "size" );

      if(( mem_size > 0 ) &&
            (((std::size_t)( mem_size / simd_dmeu_data_c::dim )) * simd_dmeu_data_c::dim == mem_size )) {

         mem.resize( simd_dmeu_data_c::dim );

         for( std::size_t dim = 0; dim < simd_dmeu_data_c::dim; dim ++ ) {
            mem.at( dim ).resize( mem_size / simd_dmeu_data_c::dim );
         }
      }
      else {
         SIMD_REPORT_ERROR( "simd::sys_dm" ) << name() << " Incorrect memory size";
      }
   }
   catch( const boost_pt::ptree_error& err ) {
      SIMD_REPORT_ERROR( "simd::sys_dm" ) << name() << " " <<  err.what();
   }
   catch( const std::exception& err ) {
      SIMD_REPORT_ERROR( "simd::sys_dm" ) << name() << " " <<  err.what();
   }
   catch( ... ) {
      SIMD_REPORT_ERROR( "simd::sys_dm" ) << name() << " Unexpected";
   }

   // Initialise memory
   bool init_done = false;

   if( !init_done ) {
      boost::optional<simd_dmeu_smp_t> init_p;

      try {
         init_p = pref.get_optional<simd_dmeu_smp_t>( "init" );
      }
      catch( const boost_pt::ptree_error& err ) {
         SIMD_REPORT_ERROR( "simd::sys_dm" ) << name() << " " <<  err.what();
      }
      catch( ... ) {
         SIMD_REPORT_ERROR( "simd::sys_dm" ) << name() << " Unexpected";
      }

      if( init_p.is_initialized()) {
         // Initialise with a constant
         simd_dmeu_slot_t mem_init;
         mem_init.ena = true;
         mem_init.smp = init_p.get();

         for( std::size_t dim = 0; dim < simd_dmeu_data_c::dim; dim ++ ) {
            mem.at( dim ).assign(
                  mem.at( dim ).size(),
                  mem_init );
         }

         init_done = true;
      }
   }

   // Unformatted string parser should be the last one
   if( !init_done ) {
      std::string init;

      try {
         init = pref.get<std::string>( "init" );
      }
      catch( const boost_pt::ptree_error& err ) {
         SIMD_REPORT_ERROR( "simd::sys_dm" ) << name() << " " <<  err.what();
      }
      catch( ... ) {
         SIMD_REPORT_ERROR( "simd::sys_dm" ) << name() << " Unexpected";
      }

      if( init == "none" ) { // No initialisation
         // Do nothing
         init_done = true;
      }
      else if( init == "rng" ) { // RNG initialisation
         boost::hash<std::string> hash_str;
         boost_rn::mt19937                        rng_eng( hash_str( name())); //boost::hash<std::string>( name())); // can use std::time() + hash(name())
         boost_rn::uniform_real_distribution<>    rng_dist(-1.0, 1.0);
         boost_rn::variate_generator<boost_rn::mt19937&, boost_rn::uniform_real_distribution<> > rng( rng_eng, rng_dist );

         for( std::size_t dim = 0; dim < simd_dmeu_data_c::dim; dim ++ ) {
            BOOST_FOREACH( simd_dmeu_slot_t& el, mem.at( dim )) {
               el.ena = true;
               el.smp = simd_dmeu_smp_t( rng(), rng());
            }
         }

         init_done = true;
      }
      else { // Initialisation from file
         boost::optional<const boost_pt::ptree&> ini_p;

         try {
            ini_p = pref.get_child_optional( "init" );
         }
         catch( const boost_pt::ptree_error& err ) {
            SIMD_REPORT_ERROR( "simd::sys_dm" ) << name() << " " <<  err.what();
         }
         catch( ... ) {
            SIMD_REPORT_ERROR( "simd::sys_dm" ) << name() << " Unexpected";
         }

         if( ini_p ) {
            simd_sys_dm_init( ini_p, mem, *this );
         }
         else {
            SIMD_REPORT_ERROR( "simd::sys_dm" ) << name() << " Incorrect node";
         }
      }
   }

   // Initialise the list of the address generators for wr side
   const boost_pt::ptree& wr_ag_modes = [&]() {
      try {
         return pref.get_child("wr_ag_modes");
      }
      catch( const boost_pt::ptree_error& err ) {
         SIMD_REPORT_ERROR( "simd::sys_dm" ) << name() << " " << err.what();
      }
      catch( ... ) {
         SIMD_REPORT_ERROR( "simd::sys_dm" ) << name() << " Unexpected";
      }

      return pref.get_child("wr_ag_modes");
   }();

   BOOST_FOREACH( const boost_pt::ptree::value_type& mode, wr_ag_modes ) {
      if( !mode.first.empty()) {
         SIMD_REPORT_ERROR( "simd::sys_dm" ) << name() << " Incorrect structure";
      }

      try {
         std::string wr_ag_func = mode.second.get_value<std::string>();

         if( wr_ag_list.find( wr_ag_func ) != wr_ag_list.end()) {
            SIMD_REPORT_ERROR( "simd::sys_dm" ) << name() << " Duplicate WR AG mode";
         }

         wr_ag_list[wr_ag_func] = simd_sys_dm_ag_new( wr_ag_func,
               boost::optional<const simd_sys_dmeu_c&>( *((simd_sys_dmeu_c *)this )));
      }
      catch( const boost_pt::ptree_error& err ) {
         SIMD_REPORT_ERROR( "simd::sys_dm" ) << name() << " " <<  err.what();
      }
      catch( const std::exception& err ) {
         SIMD_REPORT_ERROR( "simd::sys_dm" ) << name() << " " <<  err.what();
      }
      catch( ... ) {
         SIMD_REPORT_ERROR( "simd::sys_dm" ) << name() << " Unexpected";
      }
   }

   // Initialise the list of the address generators for wr side
   const boost_pt::ptree& rd_ag_modes = [&]() {
      try {
         return pref.get_child("rd_ag_modes");
      }
      catch( const boost_pt::ptree_error& err ) {
         SIMD_REPORT_ERROR( "simd::sys_dm" ) << name() << " " << err.what();
      }
      catch( ... ) {
         SIMD_REPORT_ERROR( "simd::sys_dm" ) << name() << " Unexpected";
      }

      return pref.get_child("rd_ag_modes");
   }();

   BOOST_FOREACH( const boost_pt::ptree::value_type& mode, rd_ag_modes ) {
      if( !mode.first.empty()) {
         SIMD_REPORT_ERROR( "simd::sys_dm" ) << name() << " Incorrect structure";
      }

      try {
         std::string rd_ag_func = mode.second.get_value<std::string>();

         if( rd_ag_list.find( rd_ag_func ) != rd_ag_list.end()) {
            SIMD_REPORT_ERROR( "simd::sys_dm" ) << name() << " Duplicate WR AG mode";
         }

         rd_ag_list[rd_ag_func] = simd_sys_dm_ag_new( rd_ag_func,
               boost::optional<const simd_sys_dmeu_c&>( *((simd_sys_dmeu_c *)this )));
      }
      catch( const boost_pt::ptree_error& err ) {
         SIMD_REPORT_ERROR( "simd::sys_dm" ) << name() << " " <<  err.what();
      }
      catch( const std::exception& err ) {
         SIMD_REPORT_ERROR( "simd::sys_dm" ) << name() << " " <<  err.what();
      }
      catch( ... ) {
         SIMD_REPORT_ERROR( "simd::sys_dm" ) << name() << " Unexpected";
      }
   }

   // Disable all the transactions until the block is configured
   wr_en = false;
   rd_en = false;

   // Reset sample counters
   wr_smp_cnt = 0;
   rd_smp_cnt = 0;
}

void simd_sys_dm_ram_1r1w_c::add_trace(
      sc_core::sc_trace_file* tf,
      const std::string& top_name ) {
   std::string mod_name = top_name + "." + std::string( name());

   // XBAR interface
   add_trace_ports(
         tf,
         top_name );

   // memory wr side
   sc_core::sc_trace( tf, wr_smp_cnt,   mod_name + ".wr_smp_cnt"  );  // sample counter
   sc_core::sc_trace( tf, wr_en,        mod_name + ".wr_en"       );  // operation enable
   sc_core::sc_trace( tf, mem_wr_addr,  mod_name + ".mem_wr_addr" );  // memory address
   sc_core::sc_trace( tf, mem_wr_data,  mod_name + ".mem_wr_data" );  // memory data
   sc_core::sc_trace( tf, mem_wr_ena,   mod_name + ".mem_wr_ena"  );  // memory enable

   // memory rd side
   sc_core::sc_trace( tf, rd_smp_cnt,   mod_name + ".rd_smp_cnt"  );  // sample counter
   sc_core::sc_trace( tf, rd_en,        mod_name + ".rd_en"       );  // operation enable
   sc_core::sc_trace( tf, mem_rd_addr,  mod_name + ".mem_rd_addr" );  // memory address
   sc_core::sc_trace( tf, mem_rd_data,  mod_name + ".mem_rd_data" );  // memory data
   sc_core::sc_trace( tf, mem_rd_ena,   mod_name + ".mem_rd_ena"  );  // memory enable
}

void simd_sys_dm_ram_1r1w_c::req_run(   // Execution request
      void ) {

   // Get operation type: read or write and smp size
   std::string  op_mode;

   try {
      op_mode   = config.at( active_config ).get<std::string>(  "op_mode"   );
   }
   catch( const boost_pt::ptree_error& err ) {
      SIMD_REPORT_ERROR( "simd::sys_dm" ) << name() << " " <<  err.what();
   }
   catch( const std::exception& err ) {
      SIMD_REPORT_ERROR( "simd::sys_dm" ) << name() << " " <<  err.what();
   }
   catch( ... ) {
      SIMD_REPORT_ERROR( "simd::sys_dm" ) << name() << " Unexpected";
   }

   // Read and Write can run concurrently for ram_1r1w
   if( op_mode == "write" ) {
      wr_en = true;
      rd_en = false;
   }
   else if( op_mode == "read" ) {
      wr_en = false;
      rd_en = true;
   }
   else if( op_mode == "rd_wr" ) {
      wr_en = true;
      rd_en = true;
   }
   else {
      SIMD_REPORT_ERROR( "simd::sys_dm" ) << name() << " Unsupported operation";
   }

   // Set parameters for ram wr transactions
   if( wr_en ) {
      // Get ag type and supply parameters to ag from the current configuration
      std::string                             wr_ag_mode;
      boost::optional<const boost_pt::ptree&> wr_ag_prm_p;

      try {
         wr_smp_num  = config.at( active_config ).get<std::size_t>(   "wr_smp_num" );
         wr_ag_mode  = config.at( active_config ).get<std::string>(   "wr_ag_conf.mode" );
         wr_ag_prm_p = config.at( active_config ).get_child_optional( "wr_ag_conf.param" );
      }
      catch( const boost_pt::ptree_error& err ) {
         SIMD_REPORT_ERROR( "simd::sys_dm" ) << name() << " " <<  err.what();
      }
      catch( const std::exception& err ) {
         SIMD_REPORT_ERROR( "simd::sys_dm" ) << name() << " " <<  err.what();
      }
      catch( ... ) {
         SIMD_REPORT_ERROR( "simd::sys_dm" ) << name() << " Unexpected";
      }

      // Check if the requested ag mode is in the list, which was specified in the preferences
      auto wr_ag_found = wr_ag_list.find( wr_ag_mode );

      if( wr_ag_found == wr_ag_list.end()) {
         SIMD_REPORT_ERROR( "simd::sys_dm" ) << name() << " " << wr_ag_mode << " AG mode is unavailable";
      }

      p_wr_ag = wr_ag_found->second;
      p_wr_ag.get().init( wr_ag_prm_p );
      wr_smp_cnt = 0;  // Reset sample counter
   } // if( wr_ena )

   // Set parameters for ram rd transactions
   if( rd_en ) {
      // Get ag type and supply parameters to ag from the current configuration
      std::string                             rd_ag_mode;
      boost::optional<const boost_pt::ptree&> rd_ag_prm_p;

      try {
         rd_smp_num  = config.at( active_config ).get<std::size_t>(   "rd_smp_num" );
         rd_ag_mode  = config.at( active_config ).get<std::string>(   "rd_ag_conf.mode" );
         rd_ag_prm_p = config.at( active_config ).get_child_optional( "rd_ag_conf.param" );
      }
      catch( const boost_pt::ptree_error& err ) {
         SIMD_REPORT_ERROR( "simd::sys_dm" ) << name() << " " <<  err.what();
      }
      catch( const std::exception& err ) {
         SIMD_REPORT_ERROR( "simd::sys_dm" ) << name() << " " <<  err.what();
      }
      catch( ... ) {
         SIMD_REPORT_ERROR( "simd::sys_dm" ) << name() << " Unexpected";
      }

      // Check if the requested ag mode is in the list, which was specified in the preferences
      auto rd_ag_found = rd_ag_list.find( rd_ag_mode );

      if( rd_ag_found == rd_ag_list.end()) {
         SIMD_REPORT_ERROR( "simd::sys_dm" ) << name() << " " << rd_ag_mode << " AG mode is unavailable";
      }

      p_rd_ag = rd_ag_found->second;
      p_rd_ag.get().init( rd_ag_prm_p );
      rd_smp_cnt = 0;    // Reset sample counter
   } // if( rd_ena )

}

void simd_sys_dm_ram_1r1w_c::proc_thrd(
      void ) {

   // WR side variables
   simd_dmeu_valid_t     wr_valid = SIMD_IDLE;
   simd_dmeu_data_c      wr_data;
   simd_dm_addr_c        wr_addr;

   simd_dmeu_ready_t     wr_ready = false;
   simd_dmeu_state_t     wr_state     = DMEU_ST_IDLE;

   // RD side variables
   simd_dmeu_valid_t     rd_valid = SIMD_IDLE;
   simd_dmeu_data_c      rd_data;
   simd_dm_addr_c        rd_addr;

   simd_dmeu_ready_t     rd_ready = false;
   bool                  rd_avail;

   simd_dmeu_state_t     rd_state = DMEU_ST_IDLE;

   simd_dm_ram_1r1w_acc_t rd_mem_acc = ACC_MEM_IDLE;
   bool                   rd_smp_cnt_zro = false;  // Sample counter is zero
   bool                   rd_smp_cnt_exp = false;  // Sample counter is expired

   simd_dump_buf_c<simd_dm_addr_c>   dump_buf_addr_wr( std::string( name()) + ".addr_wr" );
   simd_dump_buf_c<simd_dm_addr_c>   dump_buf_addr_rd( std::string( name()) + ".addr_rd" );
   simd_dump_buf_c<simd_dmeu_data_c> dump_buf_data_wr( std::string( name()) + ".data_wr" );
   simd_dump_buf_c<simd_dmeu_data_c> dump_buf_data_rd( std::string( name()) + ".data_rd" );

   sc_core::wait();

   for(;;) {
      sc_core::wait();

      // Read ports
      simd_dmeu_state_t state   = state_o->read().get(); // current state sent to xbar
      bool              proc_en = proc_i->read();        // proc_en from xbar

      // RD access to memory array. Occurs before write
      if( mem_rd_ena ) {
         simd_dmeu_data_c rd_data;
         simd_dm_addr_c   rd_addr = mem_rd_addr.get();

         for( std::size_t dim = 0; dim < rd_addr.dim; dim ++ ) {
            if( rd_addr[dim].ena ) {
               simd_dmeu_slot_t &mem_el = mem.at( rd_addr[dim].perm ).at( rd_addr[dim].addr );

               if( !mem_el.ena ) {
                  SIMD_REPORT_ERROR( "simd::sys_dm" ) << name() << " Read from un-initialized memory" <<
                        " dim: "  << rd_addr[dim].perm <<
                        " addr: " << rd_addr[dim].addr;

               }

               rd_data[dim] = mem_el;
            }
            else {
               rd_data[dim].ena = false;
            }
         }

         mem_rd_data.set( rd_data );
      }

      // WR access to memory array. Occurs after read
      if( mem_wr_ena ) {
         simd_dmeu_data_c wr_data = mem_wr_data.get();
         simd_dm_addr_c   wr_addr = mem_wr_addr.get();

         for( std::size_t dim = 0; dim < wr_addr.dim; dim ++ ) {
            if( wr_addr[dim].ena && wr_data[dim].ena ) {
               mem.at( wr_addr[dim].perm ).at( wr_addr[dim].addr ) = wr_data[dim];
            }
         }
      }

      // Initiate RAM WR transaction
      if( wr_en && (    state == DMEU_ST_IDLE ||    state == DMEU_ST_PROC ) && proc_en &&
                   ( wr_state == DMEU_ST_IDLE || wr_state == DMEU_ST_PROC )) {

         if( wr_ready ) { // Ready was set in the previous clock cycle
            data_vi.at( 0 )->nb_read( wr_data, wr_valid );

            if( wr_valid != SIMD_IDLE ) {
               // Generate event on the vector head marker
               if( wr_valid == SIMD_HEAD ) {
                  wr_state = DMEU_ST_PROC;
                  event( "wr_vec_head" );
               }

               // Generate event on the vector tail marker
               if( wr_valid == SIMD_TAIL ) {
                  wr_state = DMEU_ST_DONE;
                  event( "wr_vec_tail" );
               }

               p_wr_ag.get().conv( wr_smp_cnt, wr_addr );  // Generate addresses for the RAM blocks
               mem_wr_addr.set( wr_addr );
               mem_wr_data.set( wr_data );
               mem_wr_ena  = true;

               // Write to dump buffer
               dump_buf_addr_wr.write(
                     wr_addr,
                     wr_valid != SIMD_TAIL ? BUF_WRITE_CONT : BUF_WRITE_LAST );

               dump_buf_data_wr.write(
                     wr_data,
                     wr_valid != SIMD_TAIL ? BUF_WRITE_CONT : BUF_WRITE_LAST );

               if( wr_valid != SIMD_TAIL ) {
                  wr_smp_cnt ++; // Update sample counter
               }
            } // if( wr_valid != SIMD_IDLE )
            else {
               mem_wr_ena = false;
            }
         } // if( wr_ready )
         else {
            mem_wr_ena = false;
         }
      } // if( wr_ena && ( state == ...
      else {
         mem_wr_ena = false;
      }

      // RAM RD transaction
      if( rd_en && (    state == DMEU_ST_IDLE ||    state == DMEU_ST_PROC ) && proc_en &&
                   ( rd_state == DMEU_ST_IDLE || rd_state == DMEU_ST_PROC )) {
         // Read ports
         rd_avail = data_vo.at( 0 )->is_avail();
         rd_ready = data_vo.at( 0 )->is_ready();

         // Read access to the memory array
         if(( rd_mem_acc == ACC_MEM_ADDR_DATA || rd_mem_acc == ACC_MEM_DATA ) && rd_ready ) {
            // Try reading and transmit a sample on this clock cycle
            if( rd_smp_cnt_exp ) { // This condition should go first according to VRI
               rd_valid = SIMD_TAIL;
            }
            else if( rd_smp_cnt_zro ) {
               rd_state     = DMEU_ST_PROC;
               rd_valid = SIMD_HEAD;
               event( "rd_vec_head" );
            }
            else {
               rd_valid = SIMD_BODY;
            }

            rd_data = mem_rd_data.get();

            // Write data to channel
            data_vo.at( 0 )->nb_write( rd_data, rd_valid );

            // Write to dump buffer
            dump_buf_addr_rd.write(
                  rd_addr,
                  rd_valid != SIMD_TAIL ? BUF_WRITE_CONT : BUF_WRITE_LAST );

            dump_buf_data_rd.write(
                  rd_data,
                  rd_valid != SIMD_TAIL ? BUF_WRITE_CONT : BUF_WRITE_LAST );

         }
         else if( rd_mem_acc == ACC_MEM_WACK && rd_avail ) {
            // Acknowledged TAIL
            if( rd_valid != SIMD_TAIL ) {
               SIMD_REPORT_ERROR( "simd::sys_dm" ) << name() << " Unexpected state";
            }

            event( "rd_vec_tail" );   // Generate event on the acknowledged vector tail marker

            rd_state = DMEU_ST_DONE;
            rd_valid = SIMD_IDLE;
            data_vo.at( 0 )->nb_valid( rd_valid );
         }

         // Generate read address for the next transaction
         if(( rd_mem_acc == ACC_MEM_ADDR || rd_mem_acc == ACC_MEM_ADDR_DATA || rd_smp_cnt == 0 ) && rd_ready ) {
            rd_smp_cnt_zro = ( rd_smp_cnt == 0 );              // Counter is zero    flag
            rd_smp_cnt_exp = ( rd_smp_cnt == rd_smp_num - 1 ); // Counter is expired flag
            p_rd_ag.get().conv( rd_smp_cnt, rd_addr );          // Generate addresses for the RAM blocks
            mem_rd_addr.set( rd_addr );
            mem_rd_ena = true;
         }
         else {
            mem_rd_ena = false;
         }

         // Update access FSM and sample counter
         if( rd_ready && ( rd_mem_acc == ACC_MEM_ADDR || rd_smp_cnt == 0 )) {
            rd_mem_acc = ACC_MEM_ADDR_DATA;
            rd_smp_cnt ++;
         }
         else if( rd_ready && rd_mem_acc == ACC_MEM_ADDR_DATA && rd_smp_cnt == rd_smp_num - 1 ) {
            rd_mem_acc = ACC_MEM_DATA;
         }
         else if( rd_ready && rd_mem_acc == ACC_MEM_ADDR_DATA ) {
            rd_smp_cnt ++;
         }
         else if( rd_ready && rd_mem_acc == ACC_MEM_DATA ) {
            rd_mem_acc = ACC_MEM_WACK;
         }
         else if( rd_avail && rd_mem_acc == ACC_MEM_WACK ) {
            rd_mem_acc = ACC_MEM_IDLE;
         }
      } // if( rd_ena && ( state == ...

      // Update ready signal of the input interface.
      wr_ready = wr_en;
      data_vi.at( 0 )->nb_ready( wr_ready );

      // Proceed with updates to state output
      simd_sig_dmeu_state_c sig_state;

      if(( rd_state != DMEU_ST_IDLE && rd_en) ||
         ( wr_state != DMEU_ST_IDLE && wr_en )) {
         if( rd_state == DMEU_ST_PROC || wr_state == DMEU_ST_PROC ) {
            state_o->write( sig_state.set( DMEU_ST_PROC ));

            rd_state = ( rd_state == DMEU_ST_PROC ) ? DMEU_ST_IDLE : rd_state;
            wr_state = ( wr_state == DMEU_ST_PROC ) ? DMEU_ST_IDLE : wr_state;
         }
         else if(( rd_state == DMEU_ST_DONE || rd_en == false ) &&
                 ( wr_state == DMEU_ST_DONE || wr_en == false )) {
            state_o->write( sig_state.set( DMEU_ST_DONE ));

            rd_state = DMEU_ST_IDLE;
            wr_state = DMEU_ST_IDLE;
         }
      }

      if( state == DMEU_ST_DONE && !proc_en ) {
         bool run_here = next_conf_run( active_config );

         if( !run_here && active_config_p && active_config_p.get().num_available()) {
            std::size_t conf_idx = active_config_p.get().read();

            active_config = conf_idx;  // Specify active config index
            active_status = curr_stat_idx( conf_idx );

            req_run();  // Function-specific configuration
         }

         state_o->write( sig_state.set( DMEU_ST_IDLE ));
      }

      // Parse request on busw (1 per clock cycle)
      if( busw_i->num_available()) {
         simd_sig_ptree_c req = busw_i->read();

         parse_busw_req( req );
      }
   }
}

void simd_sys_dm_ram_1r1w_c::ready_thrd(
      void ) {

   sc_core::wait();
}

} // namespace simd
