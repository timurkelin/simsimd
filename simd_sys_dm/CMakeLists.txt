# Include directories
list(
   APPEND
      PRIVATE_INC_DIR
      "${CMAKE_CURRENT_LIST_DIR}/include"
)

# Sources definition
set(
   SRC_CXX_LIST
		"simd_sig_dm_addr.cpp"
		"simd_sys_dm_ag.cpp"
		"simd_sys_dm_init.cpp"
		"simd_sys_dm_ram_1rw.cpp"
		"simd_sys_dm_ram_1r1w.cpp"
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
