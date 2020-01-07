# Sources definition
SRC_CXX_DIR = ./src

SRC_CXX = simd_common.cpp \
          simd_conv_ptree.cpp

# build and link sc_main only for the top level of the project          
ifeq "${EXE_OUT}" "simsimd"
	SRC_CXX += simd_main.cpp
endif

# Specify variables for Boilermake
SOURCES := $(addprefix $(SRC_CXX_DIR)/,$(SRC_CXX))
