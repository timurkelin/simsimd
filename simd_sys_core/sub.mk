# Sources definition
SRC_CXX_DIR = ./src

SRC_CXX = simd_sys_core.cpp       \
		    simd_sys_dmeu.cpp       \
		    simd_sys_dmeu_new.cpp   \
		    simd_sig_ptree.cpp      \
          simd_sig_dmeu_valid.cpp \
          simd_sig_dmeu_ready.cpp \
          simd_sig_dmeu_state.cpp \
          simd_sig_dmeu_data.cpp  \
          simd_chn_dmeu_data.cpp  \
          simd_ptree_xbar.cpp

# Specify variables for Boilermake
SOURCES := $(addprefix $(SRC_CXX_DIR)/,$(SRC_CXX))
