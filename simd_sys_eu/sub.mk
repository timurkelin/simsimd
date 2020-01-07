# Sources definition
SRC_CXX_DIR = ./src

SRC_CXX = simd_sys_eu_transp_s_1.cpp \
          simd_sys_eu_transp_a_1.cpp \
		  simd_sys_eu_add_sub_2.cpp	

# Specify variables for Boilermake
SOURCES := $(addprefix $(SRC_CXX_DIR)/,$(SRC_CXX))
