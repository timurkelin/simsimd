/*
 * simd_chn_dmeu_data.cpp
 *
 *  Description:
 *    Methods of the of the DM/EU i/o channels and interfaces.
 *
 */

#include "simd_chn_dmeu_data.h"
#include "simd_report.h"

namespace simd {

static bool valid_trans_sync[4][4] = // [prev_state][curr_state]
{ /*       IDLE,  BODY   HEAD   TAIL */
/*IDLE*/ { true,  true,  true,  true  },
/*BODY*/ { true,  true,  false, true  },
/*HEAD*/ { true,  true,  false, true  },
/*TAIL*/ { true,  false, true,  true  }}; // tail->tail when the current vector is of size 1

static bool valid_trans_async[4][4] = // [prev_state][curr_state]
{ /*       IDLE,  BODY   HEAD   TAIL */
/*IDLE*/ { true,  true,  true,  true  },
/*BODY*/ { true,  true,  false, true  },
/*HEAD*/ { true,  true,  true,  true  },
/*TAIL*/ { true,  false, true,  true  }}; // tail->tail when the current vector is of size 1

simd_chn_dmeu_data_o::simd_chn_dmeu_data_o(
      sc_core::sc_module_name nm )
   : sc_core::sc_channel( nm )
   , data_o(  "data_o" )
   , valid_o( "valid_o")
   , ready_i( "ready_i") {
   valid_sig.set( SIMD_IDLE );
}

simd_chn_dmeu_data_o::simd_chn_dmeu_data_o( // Constructor
      void )
   : sc_core::sc_channel(
         sc_core:: sc_gen_unique_name( "chn_dmeu_data_o" ))
   , data_o(  "data_o" )
   , valid_o( "valid_o")
   , ready_i( "ready_i") {
   valid_sig.set( SIMD_IDLE );
}

// This method should be called every clock cycle
inline bool simd_chn_dmeu_data_o::is_avail(
      void ) {
   bool is_ready_curr = (bool)ready_i->read().get();

   return is_ready_curr || ( !( is_waiting || ( valid_sig.get() != SIMD_IDLE )));
}

inline simd_dmeu_ready_t simd_chn_dmeu_data_o::is_ready(
      void ) {
   return ready_i->read().get();
}

const sc_core::sc_event& simd_chn_dmeu_data_o::default_event(
      void ) const {
   return ready_i->value_changed_event();
}

const sc_core::sc_event& simd_chn_dmeu_data_o::ready_event(
      void ) const {
   return ready_i->value_changed_event();
}

inline void simd_chn_dmeu_data_o::nb_valid(
      const simd_dmeu_valid_t& valid,
      const bool sync = true ) {

   bool is_ready_curr = (bool)ready_i->read().get();

   if( is_ready_curr || ( !( is_waiting || ( valid_sig.get() != SIMD_IDLE ))) || ( !sync )) {
      bool is_trans_valid = sync ? valid_trans_sync[ valid_prev][valid]
                                 : valid_trans_async[valid_prev][valid];

      if( !is_trans_valid ) {
         SIMD_REPORT_ERROR( "simd::sys_dmeu" ) << name() << " VRI sequence violation";
      }

      valid_prev = valid_sig.get();

      // Change if the previous transaction has been accepted
      valid_o->write( valid_sig.set( valid ));

      // We are waiting for the transaction to be accepted by the destination
      is_waiting = ( is_ready_curr == false ) && ( valid != SIMD_IDLE );
   }
   else if( valid_sig.get() == valid ) {
      valid_o->write( valid_sig ); // Update if the value is unchanged
   }
   else {
      SIMD_REPORT_ERROR( "simd::sys_dmeu" ) << name() << " VRI protocol violation";
   }
}

inline void simd_chn_dmeu_data_o::nb_write(
      const simd_dmeu_data_c&  data,
      const simd_dmeu_valid_t& valid,
      const bool sync = true ) {

   bool is_ready_curr = (bool)ready_i->read().get();

   if( is_ready_curr || ( !( is_waiting || ( valid_sig.get() != SIMD_IDLE ))) || ( !sync )) {
      bool is_trans_valid = sync ? valid_trans_sync[ valid_sig.get()][valid]
                                 : valid_trans_async[valid_sig.get()][valid];

      if( !is_trans_valid ) {
         SIMD_REPORT_ERROR( "simd::sys_dmeu" ) << name() << " VRI sequence violation";
      }

      // Change if the previous transaction has been accepted
      data_o->write(  data_sig.set(  data  ));
      valid_o->write( valid_sig.set( valid ) );

      // We are waiting for the transaction to be accepted by the destination
      is_waiting = ( is_ready_curr == false ) && ( valid != SIMD_IDLE );
   }
   else if(( valid_sig.get() == valid ) && ( data_sig.get() == data )) {
      data_o->write(  data_sig );    // Update if the value is unchanged
      valid_o->write( valid_sig );
   }
   else {
      SIMD_REPORT_ERROR( "simd::sys_dmeu" ) << name() << " VRI protocol violation";
   }
}

void simd_chn_dmeu_data_o::add_trace(
      sc_core::sc_trace_file* tf,
      const std::string& top_name ) {

   std::string mod_name = top_name + "." + std::string( name());

   sc_core::sc_trace( tf, ready_i,    mod_name + ".ready_i" );
   sc_core::sc_trace( tf, data_o,     mod_name + ".data_o"  );
   sc_core::sc_trace( tf, valid_o,    mod_name + ".valid_o" );
   sc_core::sc_trace( tf, is_waiting, mod_name + ".is_waiting" );
}

simd_chn_dmeu_data_i::simd_chn_dmeu_data_i( // Constructor
      sc_core::sc_module_name nm )
   : sc_core::sc_channel( nm )
   , data_i(  "data_i" )
   , valid_i( "valid_i")
   , ready_o( "ready_o") {
   valid_sig.set( SIMD_IDLE );
}

simd_chn_dmeu_data_i::simd_chn_dmeu_data_i( // Constructor
      void ) // Constructor
   : sc_core::sc_channel(
         sc_core:: sc_gen_unique_name( "chn_dmeu_data_i" ))
   , data_i(  "data_i" )
   , valid_i( "valid_i")
   , ready_o( "ready_o") {
   valid_sig.set( SIMD_IDLE );
}

// This method should be called every clock cycle
inline void simd_chn_dmeu_data_i::nb_read(
      simd_dmeu_data_c&  data,
      simd_dmeu_valid_t& valid,
      const bool sync = true ) {
   data  =  data_i->read().get();
   valid = valid_i->read().get();

   // Checker
   if( ready_sig.get() || is_read_done || ( !sync )) {
      bool is_trans_valid = sync ? valid_trans_sync[ valid_prev][valid]
                                 : valid_trans_async[valid_prev][valid];

      if( !is_trans_valid ) {
         SIMD_REPORT_ERROR( "simd::sys_dmeu" ) << name() << " VRI sequence violation";
      }

      is_read_done = ready_sig.get();
   }
   else if(( data_sig.get() == data ) && ( valid_sig.get() == valid )) {
      is_read_done = ready_sig.get();
   }
   else {
      SIMD_REPORT_ERROR( "simd::sys_dmeu" ) << name() << " VRI protocol violation";
   }

   data_sig.set(  data  );
   valid_sig.set( valid );

   valid_prev = valid;
}

const sc_core::sc_event& simd_chn_dmeu_data_i::default_event(
      void ) const {
   return valid_i->value_changed_event();
}

const sc_core::sc_event& simd_chn_dmeu_data_i::valid_event(
      void ) const {
   return valid_i->value_changed_event();
}

const sc_core::sc_event& simd_chn_dmeu_data_i::data_event(
      void ) const {
   return data_i->value_changed_event();
}

inline void simd_chn_dmeu_data_i::nb_ready(
      const simd_dmeu_ready_t& ready ) {
   ready_o->write( ready_sig.set( ready ));
}

void simd_chn_dmeu_data_i::add_trace(
      sc_core::sc_trace_file* tf,
      const std::string& top_name ) {

   std::string mod_name = top_name + "." + std::string( name());

   sc_core::sc_trace( tf, ready_o,      mod_name + ".ready_o" );
   sc_core::sc_trace( tf, data_i,       mod_name + ".data_i"  );
   sc_core::sc_trace( tf, valid_i,      mod_name + ".valid_i" );
   sc_core::sc_trace( tf, is_read_done, mod_name + ".is_read_done" );
}

} // namespace simd

