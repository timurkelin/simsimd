# Sources definition
SRC_CXX_DIR = ./src

SRC_CXX = simd_sys_st_siggen_1.cpp \
		  simd_sys_st_sigana_1.cpp \
		  simd_sys_st_inp_1.cpp \
		  simd_sys_st_out_1.cpp

# Specify variables for Boilermake
SOURCES := $(addprefix $(SRC_CXX_DIR)/,$(SRC_CXX))
