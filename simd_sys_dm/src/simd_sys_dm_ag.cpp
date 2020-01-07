/*
 * simd_sys_dm_ag.cpp
 *
 *  Description:
 *    Address generator classes and functions
 */

#include "simd_sys_dm_ag.h"
#include "simd_assert.h"
#include "simd_report.h"

namespace simd {

// Linear address generator
class simd_sys_dm_ag_linear_c
: public simd_sys_dm_ag_c {
public:
   simd_sys_dm_ag_linear_c( boost::optional<const simd_sys_dmeu_c&> _caller_p )
   : simd_sys_dm_ag_c( _caller_p ) {};

   ~simd_sys_dm_ag_linear_c( void ){};

   void init(
         const boost::optional<const boost_pt::ptree&> _conf_p ) {
      conf_p = _conf_p; // Save AG configuration
   }

   void conv(
         const int count,
         simd_dm_addr_c& addr ) {

      // Read addressing parameters
      boost_pt::ptree::const_iterator it_ag = conf_p.get().begin();

      // Addressing parameters for each of the RAM blocks
      for( std::size_t dim = 0; dim < addr.dim; dim ++, it_ag ++ ) {
         if( it_ag == conf_p.get().end() ) {
            SIMD_REPORT_ERROR( "simd::sys_dm" ) << caller_p.get().name() << " Incorrect size";
         }

         if( !it_ag->first.empty() ) {
            SIMD_REPORT_ERROR( "simd::sys_dm" ) << caller_p.get().name() << " Incorrect structure";
         }

         try {
            addr[dim].addr =       it_ag->second.get<std::size_t>("offset") + count;
            addr[dim].ena  = (bool)it_ag->second.get<std::size_t>("ena");
            addr[dim].perm =       it_ag->second.get<std::size_t>("perm");
         }
         catch( const boost_pt::ptree_error& err ) {
            SIMD_REPORT_ERROR( "simd::sys_dm" ) << caller_p.get().name() << " " <<  err.what();
         }
         catch( ... ) {
            SIMD_REPORT_ERROR( "simd::sys_dm" ) << caller_p.get().name() << " Unexpected";
         }

         if( addr[dim].perm >= addr.dim ) {
            SIMD_REPORT_ERROR( "simd::sys_dm" ) << caller_p.get().name() << " Incorrect permutation target";
         }
      }
   }

   friend boost::optional<simd_sys_dm_ag_c &>simd_sys_dm_ag_new(
         const std::string& func,
         boost::optional<const simd_sys_dmeu_c &> _caller_p );

private:
   static inline std::string func( void ) { return "linear"; }
};

// Function to resolve name and install AG
boost::optional<simd_sys_dm_ag_c &> simd_sys_dm_ag_new(
      const std::string& func,
      boost::optional<const simd_sys_dmeu_c &> _caller_p ) {

   simd_sys_dm_ag_c* ptr = NULL;

   // Resolve DAG functions
   if(      func == simd_sys_dm_ag_linear_c::func() ) ptr = new simd_sys_dm_ag_linear_c( _caller_p );

   // Give up
   else
      SIMD_REPORT_ERROR( "simd::sys_dm" ) << "Unable to resolve <" << func << ">";

   return boost::optional<simd_sys_dm_ag_c &>( *ptr );
}

} // namespace simd
