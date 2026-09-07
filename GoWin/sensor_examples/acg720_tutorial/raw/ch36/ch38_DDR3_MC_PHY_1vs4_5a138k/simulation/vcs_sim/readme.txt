Attention:
	1. if you reconfig the ip parameters,which reload the ddr_memory_interface.ipc and re-generator. 
	   please attention bits width of the top.v and tb.v file. for example, in the tb.v:
	   	ddr_dq, ddr_dqs, ddr_addr......

	   int the top.v file:
		ddr_addr, ddr_ba,ddr_dqm,ddr_dq,ddr_dqs,ddr_dqs_n,app_wdf_mask,
		app_wdf_data,app_addr,app_rd_data,phy_rd_data


../project/src/*.v : 
                        test design for DDR3 ip
../project/src/ddr3_memory_interface/ddr3_memory_interface.vo :
                        DDR3 IP
../tb/tb.v :
                        testbench for simulation
prim_sim.v:             simulation library