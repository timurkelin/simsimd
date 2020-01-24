# Include directories
list(
   APPEND
      PRIVATE_INC_DIR
      "${CMAKE_CURRENT_LIST_DIR}/include"
)

# Sources definition
set(
   SRC_CXX_LIST
		"simd_sys_core.cpp"      
		"simd_sys_dmeu.cpp"      
		"simd_sys_dmeu_new.cpp"  
		"simd_sig_ptree.cpp"     
		"simd_sig_dmeu_valid.cpp"
		"simd_sig_dmeu_ready.cpp"
		"simd_sig_dmeu_state.cpp"
		"simd_sig_dmeu_data.cpp" 
		"simd_chn_dmeu_data.cpp" 
		"simd_ptree_xbar.cpp"
)

# Full path
list(
   TRANSFORM
      SRC_CXX_LIST
   PREPEND
      "${CMAKE_CURRENT_LIST_DIR}/src/"
)

# add to the global list of the source files
list(
   APPEND
      PRIVATE_CXX
      "${SRC_CXX_LIST}"
)
