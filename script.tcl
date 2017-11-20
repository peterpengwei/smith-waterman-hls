############################################################
## This file is generated automatically by Vivado HLS.
## Please DO NOT edit it.
## Copyright (C) 2015 Xilinx Inc. All rights reserved.
############################################################
open_project sw_pe_array
set_top mmult
add_files sw_pe_array.cpp
#add_files -tb test-cl.c
open_solution -reset "solution1"
set_part virtex7
create_clock -period 5 -name default
config_compile -unsafe_math_optimizations
#csim_design -clean
csynth_design
#cosim_design -trace_level all -tool xsim
#export_design -format ip_catalog
