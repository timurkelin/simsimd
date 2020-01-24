# Sources definition
set(
   SRC_CXX_LIST
		"simd_pref_init.cpp"
	   "simd_sys_scalar_run.cpp" 
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
