# Sources definition
SRC_CXX_DIR = ./src

SRC_CXX = simd_pref_init.cpp \
		  simd_sys_scalar_run.cpp 

# Specify variables for Boilermake
SOURCES := $(addprefix $(SRC_CXX_DIR)/,$(SRC_CXX))
