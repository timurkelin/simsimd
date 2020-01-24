# Include directories
list(
   APPEND
      PRIVATE_INC_DIR
      "${CMAKE_CURRENT_LIST_DIR}/include"
)

# Sources definition
set(
   SRC_CXX_LIST
		"simd_sys_xbar.cpp"
      "simd_sys_eb_src.cpp"
      "simd_sys_eb_dst.cpp"
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
