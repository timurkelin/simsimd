# Sources definition
SRC_CXX_DIR = ./src

SRC_CXX = simd_sys_bmuxr.cpp \
          simd_sys_bmuxw.cpp \
          simd_sys_event.cpp

# Specify variables for Boilermake
SOURCES := $(addprefix $(SRC_CXX_DIR)/,$(SRC_CXX))
