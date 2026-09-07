#!/bin/csh -f
vcs -sverilog -R -full64 -fsdb -timescale=1ns/100ps -debug_region+cell+encrypt\
-v ../../tb/prim_sim.v       \
./dqs_138k_vcs.vp	\
../../tb/tb.v     \
../../project/src/top.v       \
../../project/src/ddrtest.v    \
../../project/src/ddr3_memory_interface/ddr3_memory_interface.vo  \
../../project/src/gowin_pll/gowin_pll.v  \
../../tb/sim_model/ddr3_model.v   \
+incdir+../../project/src/ \
+incdir+../../project/src/ddr3_memory_interface/   \
+incdir+../../tb/sim_model/  \
+incdir+../../tb/  | tee log

