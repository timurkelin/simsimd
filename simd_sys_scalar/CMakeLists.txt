# Include directories
list(
   APPEND
      PRIVATE_INC_DIR
      "${CMAKE_CURRENT_LIST_DIR}/include"
)

# Sources definition
set(
   SRC_CXX_LIST
		"simd_sys_scalar.cpp"
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
