# Sources definition
SRC_CXX_DIR = ./src

SRC_CXX = simd_sys_xbar.cpp \
          simd_sys_eb_src.cpp \
          simd_sys_eb_dst.cpp

# Specify variables for Boilermake
SOURCES := $(addprefix $(SRC_CXX_DIR)/,$(SRC_CXX))
