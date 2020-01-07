# Sources definition
SRC_CXX_DIR = ./src

SRC_CXX =  simd_sig_dm_addr.cpp \
		   simd_sys_dm_ag.cpp   \
		   simd_sys_dm_init.cpp \
		   simd_sys_dm_ram_1rw.cpp \
		   simd_sys_dm_ram_1r1w.cpp

# Specify variables for Boilermake
SOURCES := $(addprefix $(SRC_CXX_DIR)/,$(SRC_CXX))
