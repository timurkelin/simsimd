/*
 * simd_chn_dmeu_data.h
 *
 *  Description:
 *    Declaration of the DM/EU i/o channels and interfaces.
 */

#ifndef SIMD_SYS_CORE_INCLUDE_SIMD_CHN_DMEU_DATA_H_
#define SIMD_SYS_CORE_INCLUDE_SIMD_CHN_DMEU_DATA_H_

#include <systemc>
#include "simd_sig_dmeu_data.h"
#include "simd_sig_dmeu_ready.h"
#include "simd_sig_dmeu_valid.h"
#include "simd_report.h"

namespace simd {

   // input interface (read)
   class simd_chn_dmeu_data_i_if
   : public sc_core::sc_interface {
   public:
      virtual void nb_read(
            simd_dmeu_data_c& data,
            simd_dmeu_valid_t& valid,
            const bool sync = true ) = 0;

      virtual void nb_ready(
            const simd_dmeu_ready_t& ready ) = 0;

      virtual void add_trace(
            sc_core::sc_trace_file* tf,
            const std::string& top_name ) = 0;

      virtual const sc_core::sc_event& default_event(
            void ) const = 0;

      virtual const sc_core::sc_event& valid_event(
            void ) const = 0;

      virtual const sc_core::sc_event& data_event(
            void ) const = 0;

   };

   class simd_chn_dmeu_data_i
   : public sc_core::sc_channel
   , public simd_chn_dmeu_data_i_if {
   public: // connection signals
      sc_core::sc_in< simd_sig_dmeu_data_c > data_i;
      sc_core::sc_in< simd_sig_dmeu_valid_c> valid_i;
      sc_core::sc_out<simd_sig_dmeu_ready_c> ready_o;

   public:
      SC_HAS_PROCESS(simd_chn_dmeu_data_i);
      explicit simd_chn_dmeu_data_i( // Constructor
            sc_core::sc_module_name nm );

      explicit simd_chn_dmeu_data_i( // Constructor
            void );

   public: // channel access methods and operators:
      void nb_read(
            simd_dmeu_data_c&  data,
            simd_dmeu_valid_t& valid,
            const bool sync );

      void nb_ready(
            const simd_dmeu_ready_t& ready );

      const sc_core::sc_event& default_event(
            void ) const;

      const sc_core::sc_event& valid_event(
            void ) const;

      const sc_core::sc_event& data_event(
            void ) const;

      void add_trace(
            sc_core::sc_trace_file* tf,
            const std::string& top_name );

   private:
      // Copy constructor so compiler won't create one
      simd_chn_dmeu_data_i(
            const simd_chn_dmeu_data_i& rhs ) {}; //end copy constructor

      simd_sig_dmeu_ready_c ready_sig;
      simd_sig_dmeu_data_c  data_sig;
      simd_sig_dmeu_valid_c valid_sig;
      bool                  is_read_done = true;
      simd_dmeu_valid_t     valid_prev = SIMD_TAIL;
   };

   // output interface (write)
   class simd_chn_dmeu_data_o_if
   : public sc_core::sc_interface {
   public:
      virtual simd_dmeu_ready_t is_ready(
            void ) = 0;

      virtual bool is_avail(
            void ) = 0;

      virtual void nb_write(
            const simd_dmeu_data_c& data,
            const simd_dmeu_valid_t& valid,
            const bool sync = true ) = 0;

      virtual void nb_valid(
            const simd_dmeu_valid_t& valid,
            const bool sync = true ) = 0;

      virtual void add_trace(
            sc_core::sc_trace_file* tf,
            const std::string& top_name ) = 0;

      virtual const sc_core::sc_event& default_event(
            void ) const = 0;

      virtual const sc_core::sc_event& ready_event(
            void ) const = 0;
   };

   class simd_chn_dmeu_data_o
   : public sc_core::sc_channel
   , public simd_chn_dmeu_data_o_if {
   public: // connection signals
      sc_core::sc_out<simd_sig_dmeu_data_c > data_o;
      sc_core::sc_out<simd_sig_dmeu_valid_c> valid_o;
      sc_core::sc_in< simd_sig_dmeu_ready_c> ready_i;

   public:
      SC_HAS_PROCESS(simd_chn_dmeu_data_o);
      explicit simd_chn_dmeu_data_o( // Constructor
            sc_core::sc_module_name nm );

      explicit simd_chn_dmeu_data_o( // Constructor
            void );

   public: // channel access methods and operators:
      bool is_ready(
            void );

      bool is_avail(
            void );

      void nb_valid(
            const simd_dmeu_valid_t& valid,
            const bool sync );

      void nb_write(
            const simd_dmeu_data_c&  data,
            const simd_dmeu_valid_t& valid,
            const bool sync );

      const sc_core::sc_event& default_event(
            void ) const;

      const sc_core::sc_event& ready_event(
            void ) const;

      void add_trace(
            sc_core::sc_trace_file* tf,
            const std::string& top_name );

      void ack_clear(
            void );

   private:
      // Copy constructor so compiler won't create one
      simd_chn_dmeu_data_o(
            const simd_chn_dmeu_data_o& rhs ) {}; //end copy constructor

      simd_sig_dmeu_data_c  data_sig;
      simd_sig_dmeu_valid_c valid_sig;
      bool                  is_waiting = false;
      simd_dmeu_valid_t     valid_prev = SIMD_TAIL;
   };
} // namespace simd

#endif /* SIMD_SYS_CORE_INCLUDE_SIMD_CHN_DMEU_DATA_H_ */
