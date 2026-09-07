________________________________________________________________________
	  Example DDR3 Design Read Me
-------------------------------------------------------------------------
Object device:GW5AST-138B-FCPBGA676A
---------------------------------------------------------------------------
File List:
---------------------------------------------------------------------------
.
|-- doc
|   `-- Readme.txt                              -->  Read Me file (this file)
|-- tb 
|   `-- tb.v                                    -->  TestBench for Example DDR3 design
|   `-- prim_sim.v                              -->  TestBench for Example DDR3 design
|   |-- sim_model                               -->  DDR3 model for simulation(please contact to FAE to get)
|-- project
|   `-- ddr3_1v4_hs.gprj          	        	-->  Gowin Project File for Example Design
|   `-- ddr3_1v4_hs.gprj.user               	-->  Gowin Project File for Example Design
|   |-- impl
|   |   `-- project_process_config.json
|   |-- temp 
|   |-- src                          
|       `-- top.v                      			-->  File for Gowin Project
|       `-- ddr3_1v4_hs.cst                     -->  File for Gowin Project
|       `-- ddr3_1v4_hs.sdc                     -->  File for Gowin Project
|       `-- ddrtest.v                         	-->  File for Gowin Project
|       `-- pll_mDRP_intf.v                     -->  File for Gowin Project
|       |-- ddr3_memory_interface          
|       |   `-- ddr3_memory_interface.v         -->  File for Gowin Project(Encrypted)
|       |   `-- ddr3_memory_interface.vo        -->  File for Gowin Project
|       |   `-- ddr3_memory_interface.ipc       -->  File for Gowin Project
|       |   `-- ddr3_memory_interface_tmp.v     -->  File for Gowin Project
|       |   |-- temp                            -->  File for Gowin Project
|       |-- gowin_pll          
|           `-- gowin_pll.v                     -->  File for Gowin Project(Encrypted)
|           `-- gowin_pll.mod                  	-->  File for Gowin Project
|           `-- gowin_pll.ipc                  	-->  File for Gowin Project
|           `-- gowin_pll_tmp.v                	-->  File for Gowin Project
|-- simulation                                  -->  Simulation File
|   |-- modelsim_sim
|   |   `-- readme.txt                          -->  Read Me file for modelsim simulation
|   |   `-- cmd.do                              -->  File for Simulation run command
|   |-- vcs_sim
|       `-- readme.txt                          -->  Read Me file for modelsim simulation
|       `-- vcs_verilog.sh                      -->  File for Simulation run command
|       `-- verdi_verilog.sh                    -->  File for Simulation run command

---------------------------------------------------------------------------------------------------------------
HOW TO OPEN A PROJECT IN Gowin:
---------------------------------------------------------------------------------------------------------------
1. Unzip the respective design files.
2. Launch Gowin and select "File -> Open -> Project"
3. In the Open Project dialog, enter the Project location -- "project",select the project"ddr3_1v4_hs.gprj".
4. Click Finish. Now the project is successfully loaded. 

---------------------------------------------------------------------------------------------------------------
HOW TO RUN SYNTHESIZE, PLACE AND ROUTE, IP CORE GENERATION, AND TIMING ANALYSIS IN Gowin:
---------------------------------------------------------------------------------------------------------------

1. Click the Process tab in the process panel of the Gowin dashboard. 
   Double click on Synthsize. This will bring the design through synthesis.
2. Click the Process tab in the process panel of the Gowin dashboard. 
   Double click on Place & Route. This will bring the design through mapping, place and route.
3. Once Place & Route is done, user can double click on Timing Analysis Report to get 
   the timing analysis result.
4. Click on "Project -> Configuration -> Place & Route" to configurate the Post-Place File 
   and SDF File of the design.
----------------------------------------------------------------------------------------------------------------

HOW TO RUN SIMULATION
1. User can run functional simulation by software VCS. 
2. User can check waveform by software Verdi.
----------------------------------------------------------------------------------------------------------------

HOW TO  GENERATE IP CORE
1. Click the IP Core Generator tab in the Window panel of the Gowin dashboard.
   Double click on "DDR3 Memory Interface". This will generate the IP Core for the design.
--------------------------------------------------------------------------------------------------------------

