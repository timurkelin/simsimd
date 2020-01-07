/*
 * simd_sys_eb.h
 *
 *  Description:
 *    Declaration of the Full-Bandwidth 2-Slot Elastic Buffer
 */

#ifndef SIMD_SYS_XBAR_INCLUDE_SIMD_SYS_EB_H_
#define SIMD_SYS_XBAR_INCLUDE_SIMD_SYS_EB_H_

#include <string>
#include <systemc>
#include "simd_sig_dmeu_data.h"
#include "simd_sig_dmeu_ready.h"
#include "simd_sig_dmeu_valid.h"

namespace simd {

SC_MODULE( simd_sys_eb_src_c ) {
public:
   // Module I/O Ports
   sc_core::sc_in<bool> clock_i;      // Reserve clock input though xbar works asynchronously
   sc_core::sc_in<bool> reset_i;      // Reserve reset input though xbar works asynchronously

   sc_core::sc_in< simd_sig_dmeu_data_c > data_i;
   sc_core::sc_in< simd_sig_dmeu_valid_c> valid_i;
   sc_core::sc_out<simd_sig_dmeu_ready_c> ready_o;

   sc_core::sc_out<simd_sig_dmeu_data_c > data_o;
   sc_core::sc_out<simd_sig_dmeu_valid_c> valid_o;
   sc_core::sc_in< simd_sig_dmeu_ready_c> ready_i;

   sc_core::sc_in<bool>  proc_i;       // Processing is active (this is to reset tail detector)
   sc_core::sc_out<bool> tail_o;       // Tail detected at the side of XBAR

   // Constructor declaration
   SC_CTOR( simd_sys_eb_src_c );

   // Init declaration (to be used after the instantiation and before the port binding)
   void init(
         void );

   void add_trace(
         sc_core::sc_trace_file* tf,
         const std::string& top_name );

private:

   // Process declarations
   void sync_thrd(
         void );

}; // simd_sys_eb_src_c

SC_MODULE( simd_sys_eb_dst_c ) {
public:
   // Module I/O Ports
   sc_core::sc_in<bool> clock_i;      // Reserve clock input though xbar works asynchronously
   sc_core::sc_in<bool> reset_i;      // Reserve reset input though xbar works asynchronously

   sc_core::sc_in< simd_sig_dmeu_data_c > data_i;
   sc_core::sc_in< simd_sig_dmeu_valid_c> valid_i;
   sc_core::sc_out<simd_sig_dmeu_ready_c> ready_o;

   sc_core::sc_out<simd_sig_dmeu_data_c > data_o;
   sc_core::sc_out<simd_sig_dmeu_valid_c> valid_o;
   sc_core::sc_in< simd_sig_dmeu_ready_c> ready_i;

   sc_core::sc_in<bool>  proc_i;       // Processing is active (this is to reset tail detector)
   sc_core::sc_out<bool> tail_o;       // Tail detected at the side of DMEU

   // Constructor declaration
   SC_CTOR( simd_sys_eb_dst_c );

   // Init declaration (to be used after the instantiation and before the port binding)
   void init(
         void );

   void add_trace(
         sc_core::sc_trace_file* tf,
         const std::string& top_name );

private:

   sc_core::sc_signal<simd_sig_dmeu_data_c >  data_w_s1,  data_w_s2;
   sc_core::sc_signal<simd_sig_dmeu_valid_c> valid_w_s1, valid_w_s2;

   // Process declarations
   void sync_thrd(
         void );

   void async_thrd(
         void );

}; // simd_sys_eb_dst_c

} // namespace simd


#endif /* SIMD_SYS_XBAR_INCLUDE_SIMD_SYS_EB_H_ */
